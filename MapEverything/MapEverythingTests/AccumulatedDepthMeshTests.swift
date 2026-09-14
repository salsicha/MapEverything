import CoreVideo
import Foundation
import RealityKit
import SceneKit
import Testing
import simd
@testable import MapEverything

struct AccumulatedDepthMeshTests {
    @Test("A frame queued before tracking loss cannot enter the accumulated scene")
    func cancelledTrackingFrameIsRejected() async {
        let map = AccumulatedDepthMesh()
        let session = ScanWorkSession()
        session.cancel()
        _ = await map.integrate(triangle(color: red), workSession: session)
        #expect(await map.snapshot().isEmpty)
    }

    private let red = SIMD3<UInt8>(220, 30, 20)
    private let green = SIMD3<UInt8>(20, 210, 40)

    private func triangle(x: Float = 0, z: Float = -10, color: SIMD3<UInt8>, indices: [UInt32] = [0, 1, 2],
                          cameraPosition: SIMD3<Float>? = nil,
                          exposureOffset: Float? = nil) -> MeshGenerator.DepthAnythingMeshSnapshot {
        let vertices = [SIMD3<Float>(x, 0, z), SIMD3<Float>(x + 1, 0, z), SIMD3<Float>(x, 1, z)]
        var descriptor = MeshDescriptor()
        descriptor.positions = MeshBuffers.Positions(vertices)
        descriptor.primitives = .triangles(indices)
        return .init(descriptor: descriptor, vertices: vertices, indices: indices,
                     colors: Array(repeating: color, count: 3),
                     cameraPosition: cameraPosition, exposureOffset: exposureOffset)
    }

    /// One metre in front of the test triangle's centroid, looking at it
    /// head-on: the highest-quality observation the weighting model knows.
    private let frontalCamera = SIMD3<Float>(1.0 / 3, 1.0 / 3, -9)
    /// In the triangle's own plane and 30 m away: a grazing, distant view
    /// whose observation weight clamps to the minimum.
    private let grazingCamera = SIMD3<Float>(30, 1.0 / 3, -10)

    private func channelDistance(_ a: SIMD3<UInt8>, _ b: SIMD3<UInt8>) -> Int {
        max(abs(Int(a.x) - Int(b.x)), max(abs(Int(a.y) - Int(b.y)), abs(Int(a.z) - Int(b.z))))
    }

    @Test("A grazing distant view barely disturbs a frontal close-up's colors")
    func frontalObservationResistsGrazingUpdate() async throws {
        let map = AccumulatedDepthMesh()
        _ = await map.integrate(triangle(color: red, cameraPosition: frontalCamera))
        _ = await map.integrate(triangle(color: green, cameraPosition: grazingCamera))
        let fused = try #require(await map.snapshot().colors.first)
        // Equal weighting would land halfway (distance 100); the minimum
        // observation weight keeps the result pinned near the frontal color.
        #expect(channelDistance(fused, red) < 15)
        #expect(channelDistance(fused, green) > 150)
    }

    @Test("A later frontal view overrides a surface first seen at a grazing angle")
    func frontalObservationOverridesPoorFirstView() async throws {
        let map = AccumulatedDepthMesh()
        _ = await map.integrate(triangle(color: red, cameraPosition: grazingCamera))
        _ = await map.integrate(triangle(color: green, cameraPosition: frontalCamera))
        let fused = try #require(await map.snapshot().colors.first)
        #expect(channelDistance(fused, green) < 15)
        #expect(channelDistance(fused, red) > 150)
    }

    @Test("Frames without camera context keep the historical equal weighting")
    func missingCameraContextAveragesEqually() async throws {
        let map = AccumulatedDepthMesh()
        _ = await map.integrate(triangle(color: SIMD3<UInt8>(200, 200, 200)))
        _ = await map.integrate(triangle(color: SIMD3<UInt8>(100, 100, 100)))
        let fused = try #require(await map.snapshot().colors.first)
        #expect(channelDistance(fused, SIMD3<UInt8>(150, 150, 150)) <= 1)
    }

    @Test("LiDAR provenance accrues only from LiDAR-calibrated frames")
    func lidarWeightAccumulatesOnlyFromLidarFrames() async {
        let map = AccumulatedDepthMesh()
        _ = await map.integrate(triangle(color: red, cameraPosition: frontalCamera),
                                calibrationSource: .meshPropagated)
        _ = await map.integrate(triangle(color: red, cameraPosition: frontalCamera),
                                calibrationSource: .meshPropagated)
        #expect(await map.calibrationAnchors().isEmpty)

        _ = await map.integrate(triangle(color: red, cameraPosition: frontalCamera))
        #expect(await map.calibrationAnchors().isEmpty)
        _ = await map.integrate(triangle(color: red, cameraPosition: frontalCamera))
        // The triangle's vertices sit 1 m apart: three distinct anchor voxels.
        #expect(await map.calibrationAnchors().count == 3)
    }

    @Test("Same-frame voxel welds cannot fake a second LiDAR sighting")
    func sameFrameWeldsDoNotPromoteAnchors() async {
        let map = AccumulatedDepthMesh()
        // Two triangles whose corners weld pairwise into the same voxels, as
        // neighboring grid vertices routinely do at production mesh density.
        func doubled(color: SIMD3<UInt8>) -> MeshGenerator.DepthAnythingMeshSnapshot {
            let vertices = [SIMD3<Float>(0, 0, -10), SIMD3<Float>(1, 0, -10), SIMD3<Float>(0, 1, -10),
                            SIMD3<Float>(0.01, 0, -10), SIMD3<Float>(1.01, 0, -10), SIMD3<Float>(0.01, 1, -10)]
            var descriptor = MeshDescriptor()
            descriptor.positions = MeshBuffers.Positions(vertices)
            descriptor.primitives = .triangles([0, 1, 2, 3, 4, 5])
            return .init(descriptor: descriptor, vertices: vertices, indices: [0, 1, 2, 3, 4, 5],
                         colors: Array(repeating: color, count: 6),
                         cameraPosition: frontalCamera)
        }
        _ = await map.integrate(doubled(color: red))
        // One frame, however densely welded, is one sighting: no anchors yet.
        #expect(await map.calibrationAnchors().isEmpty)
        _ = await map.integrate(doubled(color: red))
        #expect(await map.calibrationAnchors().count == 3)
    }

    /// Dense ground plane at y = -1.5 spanning 6-9 m ahead of a camera at
    /// the origin: every face's incidence cosine is 1.5/slantRange, i.e.
    /// 0.16-0.24 — the grazing far-field regime of a street scan.
    /// `duplicated` appends a second copy one centimetre away so a single
    /// frame also exercises dense same-frame welds into the same anchor
    /// cells.
    private func grazingGroundPlane(color: SIMD3<UInt8>,
                                    duplicated: Bool = false) -> MeshGenerator.DepthAnythingMeshSnapshot {
        let columns = 9, rows = 8
        var vertices: [SIMD3<Float>] = []
        for row in 0..<rows {
            for column in 0..<columns {
                let x = -1 + 2 * Float(column) / Float(columns - 1)
                let z = -6 - 3 * Float(row) / Float(rows - 1)
                vertices.append(SIMD3<Float>(x, -1.5, z))
            }
        }
        var indices: [UInt32] = []
        for row in 0..<(rows - 1) {
            for column in 0..<(columns - 1) {
                let a = UInt32(row * columns + column)
                let b = a + 1
                let c = a + UInt32(columns)
                let d = c + 1
                indices.append(contentsOf: [a, b, c, b, d, c])
            }
        }
        if duplicated {
            let base = UInt32(vertices.count)
            let shifted = vertices.map { $0 + SIMD3<Float>(0.01, 0, 0.01) }
            let shiftedIndices = indices.map { $0 + base }
            vertices.append(contentsOf: shifted)
            indices.append(contentsOf: shiftedIndices)
        }
        var descriptor = MeshDescriptor()
        descriptor.positions = MeshBuffers.Positions(vertices)
        descriptor.primitives = .triangles(indices)
        return .init(descriptor: descriptor, vertices: vertices, indices: indices,
                     colors: Array(repeating: color, count: vertices.count),
                     cameraPosition: .zero)
    }

    @Test("A grazing far-field ground plane matures anchor cells in three sightings")
    func grazingGroundPlaneMaturesWithinThreeFrames() async {
        let map = AccumulatedDepthMesh()
        _ = await map.integrate(grazingGroundPlane(color: red))
        #expect(await map.calibrationAnchors().isEmpty)
        _ = await map.integrate(grazingGroundPlane(color: red))
        // Floored sightings weigh 0.5: two frames reach 1.0 < 1.5, so
        // grazing geometry still cannot mature on two sightings...
        #expect(await map.calibrationAnchors().isEmpty)
        _ = await map.integrate(grazingGroundPlane(color: red))
        let anchors = await map.calibrationAnchors()
        // ...but the third crosses the threshold exactly (3 x 0.5 = 1.5).
        // The raw cosines here are 0.16-0.24, which without the incidence
        // floor would demand 7-10 sightings and starve the far field of
        // calibration references (all 72 plane cells mature here).
        #expect(anchors.count >= 60)
        #expect(anchors.allSatisfy { abs($0.y + 1.5) < 0.01 && $0.z <= -5.99 && $0.z >= -9.01 })
        // The deepest row (9 m slant range) matured too — the band the
        // street scan's supportCap horizon needs anchors in.
        #expect(anchors.contains { simd_length($0) > 8.5 })
    }

    @Test("One frame can never promote anchor cells, floored weights included")
    func singleFrameCannotPromoteRegardlessOfFloor() async {
        // Grazing regime: the incidence floor lifts each sighting to 0.5,
        // and the duplicated copy re-hits the same anchor cells within the
        // frame; once-per-cell-per-frame accrual caps the frame's deposit
        // at 0.5 < 1.5 per cell.
        let grazing = AccumulatedDepthMesh()
        _ = await grazing.integrate(grazingGroundPlane(color: red, duplicated: true))
        #expect(await grazing.calibrationAnchors().isEmpty)
        // Frontal regime: the certificate weight clamps at 1 < 1.5, so even
        // a perfect single sighting cannot fake a second frame.
        let frontal = AccumulatedDepthMesh()
        _ = await frontal.integrate(wall(z: 10, color: red))
        #expect(await frontal.calibrationAnchors().isEmpty)
    }

    @Test("Propagated frames refresh anchored color but never move anchored geometry")
    func fallbackFramesDoNotMoveAnchoredVertices() async throws {
        let map = AccumulatedDepthMesh()
        _ = await map.integrate(triangle(color: red, cameraPosition: frontalCamera))
        _ = await map.integrate(triangle(color: red, cameraPosition: frontalCamera))
        let anchored = await map.snapshot()

        _ = await map.integrate(triangle(x: 0.03, color: green, cameraPosition: frontalCamera),
                                calibrationSource: .meshPropagated)
        let after = await map.snapshot()
        #expect(after.vertices == anchored.vertices)
        #expect(after.colors != anchored.colors)
    }

    @Test("First LiDAR contact replaces fallback-born geometry outright")
    func lidarFirstContactReplacesFallbackGeometry() async throws {
        let map = AccumulatedDepthMesh()
        for _ in 0..<8 {
            _ = await map.integrate(triangle(color: green, cameraPosition: frontalCamera),
                                    calibrationSource: .meshPropagated)
        }
        _ = await map.integrate(triangle(x: 0.02, color: red, cameraPosition: frontalCamera))
        let mesh = await map.snapshot()
        // The reference-closure invariant: a LiDAR-touched position is a pure
        // combination of LiDAR observations, so the accumulated propagated
        // mass cannot hold the vertex at its old position even fractionally —
        // the reset makes the result exactly the LiDAR frame's geometry.
        #expect(mesh.vertices == [SIMD3<Float>(0.02, 0, -10),
                                  SIMD3<Float>(1.02, 0, -10),
                                  SIMD3<Float>(0.02, 1, -10)])
        #expect(mesh.colors == [red, red, red])
    }

    @Test("Anchor positions survive a fallback streak unchanged")
    func anchorSnapshotStableAcrossFallbackStreak() async {
        let map = AccumulatedDepthMesh()
        _ = await map.integrate(triangle(color: red, cameraPosition: frontalCamera))
        _ = await map.integrate(triangle(color: red, cameraPosition: frontalCamera))
        let before = await map.calibrationAnchors()
        for shift in [Float(0.01), 0.02, 0.03] {
            _ = await map.integrate(triangle(x: shift, color: green, cameraPosition: frontalCamera),
                                    calibrationSource: .meshPropagated)
        }
        let after = await map.calibrationAnchors()
        #expect(!before.isEmpty)
        #expect(before == after)
    }

    @Test("Mesh-propagated calibration does not compound scale error across a streak")
    func propagatedCalibrationDoesNotCompoundScaleError() async throws {
        let width = 96, height = 72
        let truth = DepthAnythingProcessor.MaximumLikelihoodCalibration(scale: 0.5, offset: 0.05)
        let imageResolution = CGSize(width: width, height: height)
        var intrinsics = matrix_identity_float3x3
        intrinsics[0][0] = 80
        intrinsics[1][1] = 80
        intrinsics[2][0] = Float(width) / 2
        intrinsics[2][1] = Float(height) / 2
        func depth(_ x: Int, _ y: Int) -> Float { 3 + Float(x) / Float(width - 1) }
        let relative = RelativeDepthMap(width: width, height: height, data: (0..<(width * height)).map {
            (1 / depth($0 % width, $0 / width) - truth.offset) / truth.scale
        })
        let wide = MeshGenerator.DepthAnythingMeshConfiguration(
            step: 1, minimumDepth: 0.1, maximumDepth: 100,
            maximumDepthDiscontinuity: 1_000, maximumTriangleCount: 100_000
        )
        func colored(_ mesh: MeshGenerator.DepthAnythingMeshSnapshot) -> MeshGenerator.DepthAnythingMeshSnapshot {
            .init(descriptor: mesh.descriptor, vertices: mesh.vertices, indices: mesh.indices,
                  colors: Array(repeating: SIMD3<UInt8>(200, 180, 160), count: mesh.vertices.count),
                  cameraPosition: mesh.cameraPosition, exposureOffset: mesh.exposureOffset)
        }

        // Weld coarser than the 2% drift (7 cm at this range) so propagated
        // geometry lands in the same voxels as the anchors it would corrupt
        // if the freeze rule leaked, while the 0.4 m anchor grid still yields
        // enough references for the fitter's own >50-sample gate.
        let map = AccumulatedDepthMesh(voxelSize: 0.1)
        for _ in 0..<3 {
            let mesh = try #require(MeshGenerator.createDepthAnythingMeshSnapshot(
                from: relative, calibration: truth, intrinsics: intrinsics,
                imageResolution: imageResolution, transform: matrix_identity_float4x4,
                configuration: wide
            ))
            _ = await map.integrate(colored(mesh))
        }
        let anchorsBefore = await map.calibrationAnchors()
        try #require(!anchorsBefore.isEmpty)

        var configuration = MeshDepthPropagation.Configuration()
        configuration.minimumSamples = 60
        configuration.minimumCoverage = 0.3
        let referenceRelative = relative.value(atX: width / 2, y: height / 2)
        var finalRatio: Float = 0
        for _ in 0..<20 {
            let anchors = await map.calibrationAnchors()
            let fit = try #require(MeshDepthPropagation.fitCalibration(
                anchors: anchors, relative: relative, intrinsics: intrinsics,
                imageResolution: imageResolution, transform: matrix_identity_float4x4,
                configuration: configuration
            ))
            let implied = try #require(DepthAnythingProcessor.calibratedMetricDepth(
                relativeDepth: referenceRelative, calibration: fit
            ))
            finalRatio = implied / depth(width / 2, height / 2)

            // Integrate deliberately 2%-too-far geometry as propagated. Were
            // fallback output able to re-enter the anchor set, each round
            // would fit against last round's error and compound 1.02^k.
            let drifted = DepthAnythingProcessor.MaximumLikelihoodCalibration(
                scale: fit.scale / 1.02, offset: fit.offset / 1.02
            )
            let mesh = try #require(MeshGenerator.createDepthAnythingMeshSnapshot(
                from: relative, calibration: drifted, intrinsics: intrinsics,
                imageResolution: imageResolution, transform: matrix_identity_float4x4,
                configuration: wide
            ))
            _ = await map.integrate(colored(mesh), calibrationSource: .meshPropagated)
            // A LiDAR frame elsewhere (nearby floor) lands between fallback
            // frames, invalidating the anchor cache. The next round's anchors
            // are re-read from vertex positions — the moment a freeze leak
            // would let last round's drifted geometry become this round's
            // reference and compound 1.02^k.
            _ = await map.integrate(triangle(x: 50, color: red,
                                             cameraPosition: SIMD3<Float>(50 + 1.0 / 3, 1.0 / 3, -9)))
        }
        #expect(abs(finalRatio - 1) < 0.02)
        // The floor patch appends its own anchors; the wall anchors that
        // served as calibration references must be byte-identical.
        let anchorsAfter = await map.calibrationAnchors()
        #expect(Array(anchorsAfter.prefix(anchorsBefore.count)) == anchorsBefore)
    }

    /// Dense wall at depth `z` filling the camera view, with full camera
    /// context so the frame participates in free-space carving.
    private func wall(z: Float, color: SIMD3<UInt8>,
                      source: DepthCalibrationSource = .lidar) -> MeshGenerator.DepthAnythingMeshSnapshot {
        let side = 48
        var vertices: [SIMD3<Float>] = []
        for row in 0..<side {
            for column in 0..<side {
                let x = -3 + 6 * Float(column) / Float(side - 1)
                let y = -3 + 6 * Float(row) / Float(side - 1)
                vertices.append(SIMD3<Float>(x, y, -z))
            }
        }
        var indices: [UInt32] = []
        for row in 0..<(side - 1) {
            for column in 0..<(side - 1) {
                let a = UInt32(row * side + column)
                let b = a + 1
                let c = a + UInt32(side)
                let d = c + 1
                indices.append(contentsOf: [a, b, c, b, d, c])
            }
        }
        var descriptor = MeshDescriptor()
        descriptor.positions = MeshBuffers.Positions(vertices)
        descriptor.primitives = .triangles(indices)
        var intrinsics = matrix_identity_float3x3
        intrinsics[0][0] = 100
        intrinsics[1][1] = 100
        intrinsics[2][0] = 50
        intrinsics[2][1] = 50
        return .init(descriptor: descriptor, vertices: vertices, indices: indices,
                     colors: Array(repeating: color, count: vertices.count),
                     cameraPosition: .zero,
                     cameraTransform: matrix_identity_float4x4,
                     intrinsics: intrinsics,
                     imageResolution: CGSize(width: 100, height: 100))
    }

    @Test("Observed free space carves phantoms out of the map")
    func carvingRemovesPhantomsInObservedFreeSpace() async {
        let map = AccumulatedDepthMesh()
        // Phantom surface at 5 m, born from a propagated frame (weight 0.25).
        _ = await map.integrate(triangle(z: -5, color: green),
                                calibrationSource: .meshPropagated)
        #expect(await map.snapshot().vertices.contains { $0.z == -5 })
        // Six frames observe a wall at 10 m through the phantom's position.
        for _ in 0..<6 {
            _ = await map.integrate(wall(z: 10, color: red))
        }
        let mesh = await map.snapshot()
        #expect(!mesh.vertices.contains { $0.z == -5 })
        #expect(mesh.vertices.contains { $0.z == -10 })
    }

    @Test("Occluded geometry behind a measured surface survives carving")
    func occludedGeometrySurvivesCarving() async {
        let map = AccumulatedDepthMesh()
        _ = await map.integrate(triangle(z: -15, color: green),
                                calibrationSource: .meshPropagated)
        for _ in 0..<6 {
            _ = await map.integrate(wall(z: 10, color: red))
        }
        #expect(await map.snapshot().vertices.contains { $0.z == -15 })
    }

    @Test("The carve margin protects surfaces near the measured depth")
    func carvingMarginProtectsNearbySurfaces() async {
        let map = AccumulatedDepthMesh()
        // 9 m sits inside the 1.35 m tolerance of a 10 m measurement; 6 m is
        // clearly in front of it.
        _ = await map.integrate(triangle(z: -9, color: green),
                                calibrationSource: .meshPropagated)
        _ = await map.integrate(triangle(z: -6, color: green),
                                calibrationSource: .meshPropagated)
        for _ in 0..<6 {
            _ = await map.integrate(wall(z: 10, color: red))
        }
        let mesh = await map.snapshot()
        #expect(mesh.vertices.contains { $0.z == -9 })
        #expect(!mesh.vertices.contains { $0.z == -6 })
    }

    @Test("The certificate drain fires at the calibration-inlier margin, inside the carve tolerance")
    func certificateDrainFiresInsideCarveTolerance() async {
        // Anchors matured at 9.2 m, then a frame measures the same rays'
        // surface at 10 m. The 0.8 m contradiction is INSIDE the carve
        // tolerance (max(0.3, 0.15*9.2) = 1.38 — geometry must survive:
        // it could be honest noise) but OUTSIDE the calibration-inlier
        // drain margin (max(0.3, 0.08*9.2) = 0.74): a fit consistent with
        // this frame would reject the stored depth, so the cells must stop
        // testifying. This is the exact gap a calibration-family-flip
        // ghost skin (11-17% of depth in front of the true surface) hides
        // in: without the drain, a matured ghost could never be demoted.
        let map = AccumulatedDepthMesh()
        _ = await map.integrate(triangle(z: -9.2, color: red))
        _ = await map.integrate(triangle(z: -9.2, color: red))
        #expect(await map.calibrationAnchors().count == 3)
        _ = await map.integrate(wall(z: 10, color: red))
        #expect(await map.calibrationAnchors().isEmpty,
                "A certificate contradicted at the inlier margin must drain")
        #expect(await map.snapshot().vertices.contains { $0.z == -9.2 },
                "The geometry itself stays under the carve tolerance's protection")
    }

    @Test("The certificate drain spares anchors inside the calibration-inlier margin")
    func certificateDrainSparesInlierAnchors() async {
        // 9.7 m against a 10 m measurement: the 0.3 m residual is honest
        // inter-frame noise (max(0.3, 0.08*9.7) = 0.78 margin), so the
        // anchors keep serving.
        let map = AccumulatedDepthMesh()
        _ = await map.integrate(triangle(z: -9.7, color: red))
        _ = await map.integrate(triangle(z: -9.7, color: red))
        #expect(await map.calibrationAnchors().count == 3)
        _ = await map.integrate(wall(z: 10, color: red))
        #expect(await map.calibrationAnchors().count == 3)
    }

    /// A frame that saw a small real surface plus beyond-cap content on the
    /// central rays: no geometry there, only clamped carrier points at the
    /// 12 m cap, exactly what MeshGenerator emits for a street-canyon
    /// opening or skyline.
    private func openBackgroundFrame(color: SIMD3<UInt8>) -> MeshGenerator.DepthAnythingMeshSnapshot {
        let vertices = [SIMD3<Float>(-0.9, 0, -2), SIMD3<Float>(-0.7, 0, -2), SIMD3<Float>(-0.9, 0.2, -2)]
        var descriptor = MeshDescriptor()
        descriptor.positions = MeshBuffers.Positions(vertices)
        descriptor.primitives = .triangles([0, 1, 2])
        var intrinsics = matrix_identity_float3x3
        intrinsics[0][0] = 100
        intrinsics[1][1] = 100
        intrinsics[2][0] = 50
        intrinsics[2][1] = 50
        var carriers: [SIMD3<Float>] = []
        for x in stride(from: Float(-4), through: 4, by: 0.5) {
            for y in stride(from: Float(-4), through: 4, by: 0.5) {
                carriers.append(SIMD3<Float>(x, y, -12))
            }
        }
        return .init(descriptor: descriptor, vertices: vertices, indices: [0, 1, 2],
                     colors: Array(repeating: color, count: 3),
                     cameraPosition: .zero,
                     cameraTransform: matrix_identity_float4x4,
                     intrinsics: intrinsics,
                     imageResolution: CGSize(width: 100, height: 100),
                     beyondCapCarriers: carriers)
    }

    @Test("Beyond-cap carriers carve phantoms on open-background rays")
    func beyondCapCarriersCarveOpenBackgroundPhantoms() async {
        let map = AccumulatedDepthMesh()
        // Phantom at 5 m on rays whose true content lies past the cap.
        _ = await map.integrate(triangle(z: -5, color: green),
                                calibrationSource: .meshPropagated)
        #expect(await map.snapshot().vertices.contains { $0.z == -5 })
        for _ in 0..<6 {
            _ = await map.integrate(openBackgroundFrame(color: red))
        }
        let mesh = await map.snapshot()
        #expect(!mesh.vertices.contains { $0.z == -5 },
                "The clamped carrier at the cap proves the phantom's ray empty")
        // Carriers are evidence, never geometry.
        #expect(!mesh.vertices.contains { $0.z < -10 })
    }

    @Test("Carved-out anchors stop serving as calibration references")
    func carvedAnchorsStopServingCalibration() async {
        let map = AccumulatedDepthMesh()
        _ = await map.integrate(triangle(z: -5, color: red, cameraPosition: frontalCamera))
        _ = await map.integrate(triangle(z: -5, color: red, cameraPosition: frontalCamera))
        #expect(await map.calibrationAnchors().count == 3)
        _ = await map.integrate(wall(z: 10, color: red))
        // One contradiction halves the anchors' LiDAR mass below the
        // two-sighting threshold; they must vanish from the reference set.
        #expect(await map.calibrationAnchors().isEmpty)
    }

    @Test("A carved voxel accepts fresh geometry afterwards")
    func carvedVoxelAcceptsFreshGeometry() async {
        let map = AccumulatedDepthMesh()
        _ = await map.integrate(triangle(z: -5, color: green),
                                calibrationSource: .meshPropagated)
        for _ in 0..<6 {
            _ = await map.integrate(wall(z: 10, color: red))
        }
        #expect(await map.snapshot().vertices.contains { $0.z == -5 } == false)
        let stats = await map.integrate(triangle(z: -5, color: red, cameraPosition: frontalCamera))
        #expect(await map.snapshot().vertices.contains { $0.z == -5 })
        #expect(stats.vertexCount > 0)
    }

    @Test("Auto-exposure drift is normalized against the scan's reference exposure")
    func exposureDriftDoesNotShiftAccumulatedColors() async throws {
        let gray = SIMD3<UInt8>(128, 128, 128)
        // The same surface recorded one EV darker: linear value halved, then
        // re-encoded with the 2.2 gamma the normalization assumes.
        let oneEVDarker = UInt8((255 * pow(pow(128.0 / 255, 2.2) / 2, 1 / 2.2)).rounded())
        let map = AccumulatedDepthMesh()
        _ = await map.integrate(triangle(color: gray, cameraPosition: frontalCamera, exposureOffset: 0))
        _ = await map.integrate(triangle(
            color: SIMD3<UInt8>(oneEVDarker, oneEVDarker, oneEVDarker),
            cameraPosition: frontalCamera, exposureOffset: -1
        ))
        let fused = try #require(await map.snapshot().colors.first)
        // Without normalization the average would sink to ~(128+93)/2 = 110.
        #expect(channelDistance(fused, gray) <= 2)

        let uncompensated = AccumulatedDepthMesh()
        _ = await uncompensated.integrate(triangle(color: gray, cameraPosition: frontalCamera))
        _ = await uncompensated.integrate(triangle(
            color: SIMD3<UInt8>(oneEVDarker, oneEVDarker, oneEVDarker),
            cameraPosition: frontalCamera
        ))
        let drifted = try #require(await uncompensated.snapshot().colors.first)
        #expect(channelDistance(drifted, gray) > 10)
    }

    @Test("Distant colored surfaces and earlier areas survive across depth frames")
    func retainsWholeSceneBeyondLiDARRange() async {
        let map = AccumulatedDepthMesh()
        _ = await map.integrate(triangle(color: red))
        _ = await map.integrate(triangle(x: 20, z: -15, color: green))
        let mesh = await map.snapshot()
        #expect(mesh.vertices.count == 6)
        #expect(mesh.indices.count == 6)
        #expect(mesh.vertices.allSatisfy { $0.z <= -10 })
        #expect(mesh.colors == [red, red, red, green, green, green])
    }

    @Test("Overlapping depth frames weld vertices and deduplicate faces regardless of winding")
    func overlapDoesNotGrowMesh() async {
        let map = AccumulatedDepthMesh()
        for _ in 0..<10 {
            _ = await map.integrate(triangle(x: 0.001, color: red))
            _ = await map.integrate(triangle(x: 0.002, color: red, indices: [2, 1, 0]))
        }
        let mesh = await map.snapshot()
        #expect(mesh.vertices.count == 3)
        #expect(mesh.indices.count == 3)
        #expect(mesh.colors == [red, red, red])
    }

    @Test("Missing colors, invalid indices and collapsed faces do not contaminate the map")
    func invalidFramesAreIgnored() async {
        let map = AccumulatedDepthMesh()
        let valid = triangle(color: red)
        let uncolored = MeshGenerator.DepthAnythingMeshSnapshot(
            descriptor: valid.descriptor, vertices: valid.vertices, indices: valid.indices, colors: []
        )
        _ = await map.integrate(uncolored)
        _ = await map.integrate(triangle(color: red, indices: [0, 1, .max]))
        _ = await map.integrate(triangle(color: red, indices: [0, 0, 1]))
        #expect(await map.snapshot().isEmpty)
        _ = await map.integrate(valid)
        #expect(await map.snapshot().indices.count == 3)
    }

    @Test("Capacity bounds preserve existing geometry instead of evicting the beginning of a scan")
    func capacityPreservesEarlierArea() async {
        let map = AccumulatedDepthMesh(maximumVertices: 3, maximumTriangles: 1)
        _ = await map.integrate(triangle(color: red))
        let stats = await map.integrate(triangle(x: 20, color: green))
        #expect(stats.reachedCapacity)
        let mesh = await map.snapshot()
        #expect(mesh.vertices.count == 3)
        #expect(mesh.colors == [red, red, red])
    }

    @Test("Export snapshots remain immutable when later frames update the same area")
    func snapshotIsStable() async {
        let map = AccumulatedDepthMesh()
        _ = await map.integrate(triangle(color: red))
        let first = await map.snapshot()
        _ = await map.integrate(triangle(color: green))
        let second = await map.snapshot()
        #expect(first.colors == [red, red, red])
        #expect(second.colors != first.colors)
        let nextScan = AccumulatedDepthMesh()
        #expect(await nextScan.snapshot().isEmpty)
    }

    @MainActor
    @Test("The final preview and export use accumulated Depth Anything colors")
    func finalPreviewAndExport() async throws {
        let controller = ARViewController()
        await controller.accumulateDepthAnythingMeshSnapshot(triangle(color: red))
        await controller.accumulateDepthAnythingMeshSnapshot(triangle(x: 20, color: green))
        let scene = try #require(await controller.makeStoppedInspectionScene())
        let source = try #require(scene.rootNode.childNodes.first?.geometry?.sources(for: .color).first)
        #expect(scene.rootNode.childNodes.first?.geometry?.firstMaterial?.lightingModel == .constant)
        #expect(scene.rootNode.childNodes.first?.geometry?.sources(for: .normal).isEmpty == true)
        #expect(source.vectorCount == 6)
        let renderedRed = source.data.withUnsafeBytes {
            $0.loadUnaligned(fromByteOffset: source.dataOffset, as: Float.self)
        }
        #expect(abs(renderedRed - Float(red.x) / 255) < 0.001)
        let artifact = try #require(await controller.currentFinalOverlayMeshArtifact())
        #expect(artifact.source == "accumulated_depth_anything_lidar_calibrated")
        #expect(artifact.colors == [red, red, red, green, green, green])
        #expect(artifact.vertices.allSatisfy { $0.z == -10 })
        controller.isScanning = true
        #expect(await controller.makeStoppedInspectionScene() == nil)
    }

    @Test("Calibrated 10-metre depth frames retain their own camera image colors")
    func farWallKeepsMatchingCaptureColors() async throws {
        let firstImage = try cameraImage(y: 70, cb: 90, cr: 220)
        let secondImage = try cameraImage(y: 160, cb: 180, cr: 80)
        // relative=.5, inverse depth scale=.2 -> metric depth=10 metres.
        let depth = RelativeDepthMap(width: 8, height: 8, data: Array(repeating: 0.5, count: 64))
        let calibration = DepthAnythingProcessor.MaximumLikelihoodCalibration(scale: 0.2, offset: 0)
        let intrinsics = simd_float3x3(columns: (
            SIMD3<Float>(8, 0, 0), SIMD3<Float>(0, 8, 0), SIMD3<Float>(4, 4, 1)
        ))
        func build(_ image: CVPixelBuffer, x: Float) throws -> MeshGenerator.DepthAnythingMeshSnapshot {
            var transform = matrix_identity_float4x4
            transform.columns.3.x = x
            return try #require(MeshGenerator.createDepthAnythingMeshSnapshot(
                from: depth, calibration: calibration, intrinsics: intrinsics,
                imageResolution: CGSize(width: 8, height: 8), transform: transform,
                configuration: .accumulatedScene, cameraImage: image
            ))
        }
        let first = try build(firstImage, x: 0)
        let second = try build(secondImage, x: 30)
        let firstColor = try #require(first.colors.first)
        let secondColor = try #require(second.colors.first)
        #expect(firstColor != secondColor)
        let map = AccumulatedDepthMesh()
        _ = await map.integrate(first)
        _ = await map.integrate(second)
        let mesh = await map.snapshot()
        #expect(!mesh.isEmpty)
        #expect(mesh.vertices.allSatisfy { abs($0.z + 10) < 0.001 })
        for (position, color) in zip(mesh.vertices, mesh.colors) {
            #expect(color == (position.x < 15 ? firstColor : secondColor))
        }
    }

    private func cameraImage(y: UInt8, cb: UInt8, cr: UInt8) throws -> CVPixelBuffer {
        var result: CVPixelBuffer?
        let status = CVPixelBufferCreate(kCFAllocatorDefault, 8, 8,
            kCVPixelFormatType_420YpCbCr8BiPlanarFullRange, nil, &result)
        try #require(status == kCVReturnSuccess)
        let buffer = try #require(result)
        CVPixelBufferLockBaseAddress(buffer, [])
        defer { CVPixelBufferUnlockBaseAddress(buffer, []) }
        let luma = try #require(CVPixelBufferGetBaseAddressOfPlane(buffer, 0)).assumingMemoryBound(to: UInt8.self)
        for row in 0..<8 {
            for col in 0..<8 { luma[row * CVPixelBufferGetBytesPerRowOfPlane(buffer, 0) + col] = y }
        }
        let chroma = try #require(CVPixelBufferGetBaseAddressOfPlane(buffer, 1)).assumingMemoryBound(to: UInt8.self)
        for row in 0..<4 {
            for col in 0..<4 {
                let index = row * CVPixelBufferGetBytesPerRowOfPlane(buffer, 1) + col * 2
                chroma[index] = cb
                chroma[index + 1] = cr
            }
        }
        return buffer
    }
}
