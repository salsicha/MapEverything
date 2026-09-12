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

    init(voxelSize: Float = 0.05, maximumVertices: Int = 2_000_000, maximumTriangles: Int = 4_000_000) {
        self.voxelSize = voxelSize.isFinite ? max(0.01, voxelSize) : 0.05
        self.maximumVertices = max(0, min(maximumVertices, 2_000_000))
        self.maximumTriangles = max(0, min(maximumTriangles, 4_000_000))
    }

    func integrate(_ mesh: MeshGenerator.DepthAnythingMeshSnapshot, workSession: ScanWorkSession? = nil,
                   viewpoint: CapturedMeshViewpoint? = nil) -> Statistics {
        // Recheck after the actor hop: tracking can be lost while a completed
        // inference waits to enter the accumulator.
        guard !Task.isCancelled, workSession?.isActive != false else { return statistics }
        // Never fabricate colors from another frame or silently import an
        // uncolored mesh. The producer supplies one camera color per vertex.
        guard mesh.colors.count == mesh.vertices.count else { return statistics }
        var remap = [UInt32?](repeating: nil, count: mesh.vertices.count)
        let keys = mesh.vertices.map(key)
        let colorTable = exposureColorTable(for: mesh.exposureOffset)
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
            let weight = observationWeight(
                faceCross: crossProduct, faceCrossLength: area.squareRoot(),
                centroid: (mesh.vertices[local.0] + mesh.vertices[local.1] + mesh.vertices[local.2]) / 3,
                cameraPosition: mesh.cameraPosition
            )
            let ia = fuse(local.0, key: a, mesh: mesh, remap: &remap, observationWeight: weight, colorTable: colorTable)
            let ib = fuse(local.1, key: b, mesh: mesh, remap: &remap, observationWeight: weight, colorTable: colorTable)
            let ic = fuse(local.2, key: c, mesh: mesh, remap: &remap, observationWeight: weight, colorTable: colorTable)
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
        }
        return statistics
    }

    /// How well the camera saw this face: the cosine of the incidence angle
    /// attenuated by inverse-square distance beyond `fullWeightDistance`.
    /// Frames without a camera position keep the historical equal weighting.
    private func observationWeight(
        faceCross: SIMD3<Float>, faceCrossLength: Float,
        centroid: SIMD3<Float>, cameraPosition: SIMD3<Float>?
    ) -> Float {
        guard let cameraPosition else { return 1 }
        let toCamera = cameraPosition - centroid
        let distance = simd_length(toCamera)
        guard distance > 1e-6, faceCrossLength > 0 else { return Self.minimumObservationWeight }
        let cosineIncidence = abs(simd_dot(faceCross, toCamera)) / (faceCrossLength * distance)
        let proximity = min(1, (Self.fullWeightDistance / distance) * (Self.fullWeightDistance / distance))
        let weight = cosineIncidence * proximity
        guard weight.isFinite else { return Self.minimumObservationWeight }
        return min(1, max(Self.minimumObservationWeight, weight))
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
                      remap: inout [UInt32?], observationWeight: Float, colorTable: [Float]?) -> UInt32 {
        if let id = remap[local] { return id }
        let rgb = mesh.colors[local]
        let color: SIMD3<Float>
        if let colorTable {
            color = SIMD3<Float>(colorTable[Int(rgb.x)], colorTable[Int(rgb.y)], colorTable[Int(rgb.z)])
        } else {
            color = SIMD3<Float>(Float(rgb.x), Float(rgb.y), Float(rgb.z))
        }
        let id: UInt32
        if let existing = vertexIDs[key] {
            id = existing
            let index = Int(id)
            // Cap the accumulated weight so fresh observations always retain
            // influence; a frontal close-up (weight 1) quickly overrides a
            // vertex first seen at a grazing angle or from far away.
            let accumulated = min(vertices[index].weight, 7)
            let total = accumulated + observationWeight
            vertices[index].position =
                (vertices[index].position * accumulated + mesh.vertices[local] * observationWeight) / total
            vertices[index].color =
                (vertices[index].color * accumulated + color * observationWeight) / total
            vertices[index].weight = total
        } else {
            id = UInt32(vertices.count)
            vertexIDs[key] = id
            vertices.append(Vertex(position: mesh.vertices[local], color: color, weight: observationWeight))
        }
        remap[local] = id
        return id
    }

    private func key(_ position: SIMD3<Float>) -> SIMD3<Int>? {
        guard position.x.isFinite, position.y.isFinite, position.z.isFinite,
              abs(position.x) < 1_000_000, abs(position.y) < 1_000_000, abs(position.z) < 1_000_000 else { return nil }
        return SIMD3<Int>(Int(floor(position.x / voxelSize)), Int(floor(position.y / voxelSize)),
                          Int(floor(position.z / voxelSize)))
    }
}
