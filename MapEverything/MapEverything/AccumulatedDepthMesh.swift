import Foundation
import simd

nonisolated struct ColoredSceneMesh: Sendable {
    let vertices: [SIMD3<Float>]
    let indices: [UInt32]
    let colors: [SIMD3<UInt8>]
    var viewpoint: CapturedMeshViewpoint? = nil

    var isEmpty: Bool { indices.isEmpty }
}

/// Fuses world-space, already-colored Depth Anything triangles while scanning.
/// Camera buffers and individual frame meshes are not retained.
actor AccumulatedDepthMesh {
    struct Statistics: Sendable {
        let vertexCount: Int
        let triangleCount: Int
        let reachedCapacity: Bool
    }

    private struct Vertex {
        var position: SIMD3<Float>
        var color: SIMD3<Float>
        var weight: Float
        // LiDAR-provenance mass: accrues only from frames whose calibration
        // came from LiDAR, with incidence-only observation weights. The
        // reference-closure invariant (docs/mesh-propagated-calibration.md)
        // guarantees every vertex with lidarWeight > 0 has a position formed
        // exclusively from LiDAR-calibrated observations. Lives in the
        // struct's tail padding, so the 48-byte stride is unchanged.
        var lidarWeight: Float
    }

    private struct Face: Hashable {
        let low: UInt32
        let middle: UInt32
        let high: UInt32

        init(_ a: UInt32, _ b: UInt32, _ c: UInt32) {
            low = min(a, min(b, c))
            high = max(a, max(b, c))
            middle = a + b + c - low - high
        }
    }

    /// A grazing or distant observation never falls below this weight, so a
    /// surface first seen badly still exists and a later frontal view (weight
    /// up to 1) dominates its color and position almost immediately.
    static let minimumObservationWeight: Float = 0.05
    /// Distance at which an observation still carries full weight; quality
    /// falls off with the inverse square beyond it, matching how camera
    /// texel density on the surface degrades.
    static let fullWeightDistance: Float = 1
    /// Exposure normalization is capped at ±1 EV so a wildly different
    /// metering never bleaches or crushes accumulated colors.
    static let maximumExposureGain: Float = 2
    /// LiDAR mass needed before a vertex can serve as a calibration anchor.
    /// One observation contributes at most 1, so this proves at least two
    /// LiDAR-calibrated sightings, filtering single-frame ghosts.
    static let anchorLidarWeight: Float = 1.5
    /// Mesh-propagated observations carry a quarter of their view-quality
    /// weight, so one subsequent frontal LiDAR view dominates up to four
    /// accumulated fallback views of fallback-born geometry.
    static let fallbackObservationWeightPenalty: Float = 0.25
    /// Anchors deduplicate on a grid this many times coarser than the weld
    /// voxel; calibration needs spread, not density.
    static let anchorVoxelScale: Float = 4
    /// Hard cap on the anchor index (~2,600 m^2 of anchored surface at the
    /// default spacing). Insertion stops at the cap; coverage remains valid.
    static let maximumAnchors = 65_536

    private let voxelSize: Float
    private let maximumVertices: Int
    private let maximumTriangles: Int
    private var vertexIDs: [SIMD3<Int>: UInt32] = [:]
    private var vertices: [Vertex] = []
    private var faces: Set<Face> = []
    private var indices: [UInt32] = []
    private var cachedSnapshot: ColoredSceneMesh?
    private var reachedCapacity = false
    private var viewpoint: CapturedMeshViewpoint?
    private var referenceExposureOffset: Float?
    private var anchorKeys: Set<SIMD3<Int>> = []
    private var anchorVertexIndices: [UInt32] = []
    private var cachedAnchorPositions: [SIMD3<Float>]?

    init(voxelSize: Float = 0.05, maximumVertices: Int = 2_000_000, maximumTriangles: Int = 4_000_000) {
        self.voxelSize = voxelSize.isFinite ? max(0.01, voxelSize) : 0.05
        self.maximumVertices = max(0, min(maximumVertices, 2_000_000))
        self.maximumTriangles = max(0, min(maximumTriangles, 4_000_000))
    }

    func integrate(_ mesh: MeshGenerator.DepthAnythingMeshSnapshot, workSession: ScanWorkSession? = nil,
                   viewpoint: CapturedMeshViewpoint? = nil,
                   calibrationSource: DepthCalibrationSource = .lidar) -> Statistics {
        // Recheck after the actor hop: tracking can be lost while a completed
        // inference waits to enter the accumulator.
        guard !Task.isCancelled, workSession?.isActive != false else { return statistics }
        // Never fabricate colors from another frame or silently import an
        // uncolored mesh. The producer supplies one camera color per vertex.
        guard mesh.colors.count == mesh.vertices.count else { return statistics }
        var remap = [UInt32?](repeating: nil, count: mesh.vertices.count)
        let keys = mesh.vertices.map(key)
        let colorTable = exposureColorTable(for: mesh.exposureOffset)
        // Neighboring grid vertices routinely weld into one accumulated
        // vertex within a single frame; anchor mass must count sightings
        // across frames, so each vertex accrues at most once per integrate.
        var lidarAccrued: Set<Int> = []
        var acceptedFace = false
        for offset in stride(from: 0, to: mesh.indices.count - mesh.indices.count % 3, by: 3) {
            let local = (Int(mesh.indices[offset]), Int(mesh.indices[offset + 1]), Int(mesh.indices[offset + 2]))
            guard local.0 < keys.count, local.1 < keys.count, local.2 < keys.count,
                  let a = keys[local.0], let b = keys[local.1], let c = keys[local.2],
                  a != b, b != c, a != c else { continue }
            let crossProduct = simd_cross(mesh.vertices[local.1] - mesh.vertices[local.0],
                                          mesh.vertices[local.2] - mesh.vertices[local.0])
            let area = simd_length_squared(crossProduct)
            guard area.isFinite, area > 1e-12 else { continue }
            let newCount = [a, b, c].reduce(0) { $0 + (vertexIDs[$1] == nil ? 1 : 0) }
            guard vertices.count + newCount <= maximumVertices, faces.count < maximumTriangles else {
                reachedCapacity = true
                break
            }
            let weights = faceWeights(
                faceCross: crossProduct, faceCrossLength: area.squareRoot(),
                centroid: (mesh.vertices[local.0] + mesh.vertices[local.1] + mesh.vertices[local.2]) / 3,
                cameraPosition: mesh.cameraPosition
            )
            let ia = fuse(local.0, key: a, mesh: mesh, remap: &remap, weights: weights,
                          colorTable: colorTable, source: calibrationSource,
                          lidarAccrued: &lidarAccrued)
            let ib = fuse(local.1, key: b, mesh: mesh, remap: &remap, weights: weights,
                          colorTable: colorTable, source: calibrationSource,
                          lidarAccrued: &lidarAccrued)
            let ic = fuse(local.2, key: c, mesh: mesh, remap: &remap, weights: weights,
                          colorTable: colorTable, source: calibrationSource,
                          lidarAccrued: &lidarAccrued)
            if faces.insert(Face(ia, ib, ic)).inserted {
                indices.append(contentsOf: [ia, ib, ic])
            }
            cachedSnapshot = nil
            acceptedFace = true
        }
        if acceptedFace {
            if let viewpoint {
                self.viewpoint = viewpoint
                cachedSnapshot = nil
            }
            if referenceExposureOffset == nil {
                referenceExposureOffset = mesh.exposureOffset
            }
            if calibrationSource == .lidar {
                // Anchored vertex positions only move on LiDAR frames, so the
                // anchor cache stays warm through an entire fallback streak.
                cachedAnchorPositions = nil
            }
        }
        return statistics
    }

    /// World positions of the LiDAR-anchored calibration references, in
    /// deterministic insertion order. Cached between LiDAR-sourced
    /// integrations, so repeated calls during a fallback streak are O(1).
    func calibrationAnchors() -> [SIMD3<Float>] {
        if let cachedAnchorPositions { return cachedAnchorPositions }
        let positions = anchorVertexIndices.map { vertices[Int($0)].position }
        cachedAnchorPositions = positions
        return positions
    }

    /// Per-face view-quality weights. `observation` is the cosine of the
    /// incidence angle attenuated by inverse-square distance beyond
    /// `fullWeightDistance` (drives position/color averaging); `anchor` is the
    /// incidence term alone — a frontal LiDAR-calibrated sighting at range is
    /// a valid anchor certificate, and depth-dependent noise is handled by the
    /// calibration sigma model instead. Frames without a camera position keep
    /// the historical equal weighting.
    private func faceWeights(
        faceCross: SIMD3<Float>, faceCrossLength: Float,
        centroid: SIMD3<Float>, cameraPosition: SIMD3<Float>?
    ) -> (observation: Float, anchor: Float) {
        guard let cameraPosition else { return (1, 1) }
        let toCamera = cameraPosition - centroid
        let distance = simd_length(toCamera)
        guard distance > 1e-6, faceCrossLength > 0 else {
            return (Self.minimumObservationWeight, Self.minimumObservationWeight)
        }
        let cosineIncidence = abs(simd_dot(faceCross, toCamera)) / (faceCrossLength * distance)
        let proximity = min(1, (Self.fullWeightDistance / distance) * (Self.fullWeightDistance / distance))
        let weight = cosineIncidence * proximity
        guard weight.isFinite, cosineIncidence.isFinite else {
            return (Self.minimumObservationWeight, Self.minimumObservationWeight)
        }
        return (min(1, max(Self.minimumObservationWeight, weight)),
                min(1, max(Self.minimumObservationWeight, cosineIncidence)))
    }

    /// Maps incoming 8-bit channels onto the scan's reference exposure by
    /// applying the EV delta as a linear-light gain (2.2 gamma approximation),
    /// so auto-exposure swings between viewpoints do not leave brightness
    /// seams. Nil when no normalization is needed.
    private func exposureColorTable(for exposureOffset: Float?) -> [Float]? {
        guard let exposureOffset else { return nil }
        let reference = referenceExposureOffset ?? exposureOffset
        let gain = min(Self.maximumExposureGain,
                       max(1 / Self.maximumExposureGain, exp2(reference - exposureOffset)))
        guard abs(gain - 1) > 0.01 else { return nil }
        return (0...255).map { value in
            let linear = pow(Float(value) / 255, 2.2) * gain
            return 255 * pow(min(1, max(0, linear)), 1 / 2.2)
        }
    }

    func snapshot() -> ColoredSceneMesh {
        if let cachedSnapshot { return cachedSnapshot }
        let result = ColoredSceneMesh(
            vertices: vertices.map(\.position), indices: indices,
            colors: vertices.map {
                SIMD3<UInt8>(UInt8(clamping: Int($0.color.x.rounded())),
                             UInt8(clamping: Int($0.color.y.rounded())),
                             UInt8(clamping: Int($0.color.z.rounded())))
            }, viewpoint: viewpoint
        )
        cachedSnapshot = result
        return result
    }

    private var statistics: Statistics {
        Statistics(vertexCount: vertices.count, triangleCount: faces.count, reachedCapacity: reachedCapacity)
    }

    private func fuse(_ local: Int, key: SIMD3<Int>, mesh: MeshGenerator.DepthAnythingMeshSnapshot,
                      remap: inout [UInt32?], weights: (observation: Float, anchor: Float),
                      colorTable: [Float]?, source: DepthCalibrationSource,
                      lidarAccrued: inout Set<Int>) -> UInt32 {
        if let id = remap[local] { return id }
        let rgb = mesh.colors[local]
        let color: SIMD3<Float>
        if let colorTable {
            color = SIMD3<Float>(colorTable[Int(rgb.x)], colorTable[Int(rgb.y)], colorTable[Int(rgb.z)])
        } else {
            color = SIMD3<Float>(Float(rgb.x), Float(rgb.y), Float(rgb.z))
        }
        let observationWeight = source == .lidar
            ? weights.observation
            : weights.observation * Self.fallbackObservationWeightPenalty
        let id: UInt32
        if let existing = vertexIDs[key] {
            id = existing
            let index = Int(id)
            // Cap the accumulated weight so fresh observations always retain
            // influence; a frontal close-up (weight 1) quickly overrides a
            // vertex first seen at a grazing angle or from far away.
            let accumulated = min(vertices[index].weight, 7)
            let total = accumulated + observationWeight
            switch source {
            case .lidar where vertices[index].lidarWeight == 0:
                // First LiDAR contact with fallback-born geometry replaces the
                // propagated running average outright, keeping every
                // LiDAR-touched position a pure combination of LiDAR
                // observations (the reference-closure invariant).
                vertices[index].position = mesh.vertices[local]
                vertices[index].color = color
                vertices[index].weight = observationWeight
            case .lidar:
                vertices[index].position =
                    (vertices[index].position * accumulated + mesh.vertices[local] * observationWeight) / total
                vertices[index].color =
                    (vertices[index].color * accumulated + color * observationWeight) / total
                vertices[index].weight = total
            case .meshPropagated where vertices[index].lidarWeight > 0:
                // Propagated observations must never move LiDAR-touched
                // geometry: color refreshes, position/weight stay frozen.
                vertices[index].color =
                    (vertices[index].color * accumulated + color * observationWeight) / total
            case .meshPropagated:
                vertices[index].position =
                    (vertices[index].position * accumulated + mesh.vertices[local] * observationWeight) / total
                vertices[index].color =
                    (vertices[index].color * accumulated + color * observationWeight) / total
                vertices[index].weight = total
            }
            if source == .lidar, lidarAccrued.insert(index).inserted {
                accrueLidarMass(vertexIndex: index, anchorWeight: weights.anchor)
            }
        } else {
            id = UInt32(vertices.count)
            vertexIDs[key] = id
            vertices.append(Vertex(position: mesh.vertices[local], color: color,
                                   weight: observationWeight,
                                   lidarWeight: source == .lidar ? weights.anchor : 0))
            if source == .lidar {
                lidarAccrued.insert(Int(id))
            }
        }
        remap[local] = id
        return id
    }

    /// Adds LiDAR-provenance mass and registers the vertex as a calibration
    /// anchor the moment it crosses the eligibility threshold — a
    /// once-per-vertex event, so the integrate hot path stays free of
    /// per-observation anchor bookkeeping.
    private func accrueLidarMass(vertexIndex: Int, anchorWeight: Float) {
        let before = vertices[vertexIndex].lidarWeight
        let after = min(before, 7) + anchorWeight
        vertices[vertexIndex].lidarWeight = after
        guard before < Self.anchorLidarWeight, after >= Self.anchorLidarWeight,
              anchorVertexIndices.count < Self.maximumAnchors else { return }
        let anchorVoxel = voxelSize * Self.anchorVoxelScale
        let position = vertices[vertexIndex].position
        let anchorKey = SIMD3<Int>(Int(floor(position.x / anchorVoxel)),
                                   Int(floor(position.y / anchorVoxel)),
                                   Int(floor(position.z / anchorVoxel)))
        guard anchorKeys.insert(anchorKey).inserted else { return }
        anchorVertexIndices.append(UInt32(vertexIndex))
        cachedAnchorPositions = nil
    }

    private func key(_ position: SIMD3<Float>) -> SIMD3<Int>? {
        guard position.x.isFinite, position.y.isFinite, position.z.isFinite,
              abs(position.x) < 1_000_000, abs(position.y) < 1_000_000, abs(position.z) < 1_000_000 else { return nil }
        return SIMD3<Int>(Int(floor(position.x / voxelSize)), Int(floor(position.y / voxelSize)),
                          Int(floor(position.z / voxelSize)))
    }
}
