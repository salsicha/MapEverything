import CoreVideo
import Foundation
import Testing
import simd
@testable import MapEverything

private final class StreetScanFixtureBundle: NSObject {}

/// Replays the failing outdoor street scan (17 Depth Anything frames over
/// ~21 s of forward panning; LiDAR reaches only ~5.5-6.4 m) through the real
/// per-frame pipeline exactly as ARViewContainer runs it, TWICE — once with
/// the device build's 12 m integration ceiling and once with the current
/// 18 m ceiling — and instruments how far evidence (matured grazing anchors
/// via the anchor-incidence floor, chained-surface z-buffer visibility, and
/// fit-inlying pseudo-sample horizons) carries the integration cap past the
/// LiDAR p95 that used to pin it at 6.7-8.8 m.
///
/// IMPORTANT fixture caveat: the recording's published clouds were already
/// truncated at the on-device per-frame integrationDepthCap (z ceilings
/// 6.7-8.3 m, regressing mid-scan, 12 m only in the final frame), so the
/// fixture cannot contain scene content beyond each frame's recorded cap.
/// The suite therefore pins the cap trajectory / branch attribution and the
/// relative depth of the two runs; the achievable >12 m gain is real but
/// bounded by that pre-truncation (Euclidean reach beyond 12 m comes from
/// per-frame z near the recorded caps plus the frustum-corner factor and
/// the camera's ~4-8 m of forward travel).
struct StreetScanReplayTests {
    struct Frame {
        let t: Double
        let transform: simd_float4x4
        let scale: Float
        let offset: Float
        let relative: RelativeDepthMap
        let lidar: CVPixelBuffer
        let lidarData: Data
    }

    struct Fixture {
        let intrinsics: simd_float3x3
        let imageResolution: CGSize
        let frames: [Frame]
    }

    static func loadFixture() throws -> Fixture {
        let url = try #require(Bundle(for: StreetScanFixtureBundle.self)
            .url(forResource: "StreetScanReplayFrames", withExtension: "json"))
        let root = try #require(try JSONSerialization.jsonObject(with: Data(contentsOf: url)) as? [String: Any])
        let fx = (root["fx"] as! NSNumber).floatValue
        let fy = (root["fy"] as! NSNumber).floatValue
        let cx = (root["cx"] as! NSNumber).floatValue
        let cy = (root["cy"] as! NSNumber).floatValue
        var intrinsics = matrix_identity_float3x3
        intrinsics[0][0] = fx
        intrinsics[1][1] = fy
        intrinsics[2][0] = cx
        intrinsics[2][1] = cy
        let resolution = CGSize(width: (root["imageWidth"] as! NSNumber).doubleValue,
                                height: (root["imageHeight"] as! NSNumber).doubleValue)

        var frames: [Frame] = []
        var t0: Double?
        for entry in root["frames"] as! [[String: Any]] {
            let columns = (entry["transform"] as! [NSNumber]).map(\.floatValue)
            let transform = simd_float4x4(columns: (
                SIMD4<Float>(columns[0], columns[1], columns[2], columns[3]),
                SIMD4<Float>(columns[4], columns[5], columns[6], columns[7]),
                SIMD4<Float>(columns[8], columns[9], columns[10], columns[11]),
                SIMD4<Float>(columns[12], columns[13], columns[14], columns[15])
            ))
            let width = (entry["relativeWidth"] as! NSNumber).intValue
            let height = (entry["relativeHeight"] as! NSNumber).intValue
            let halfData = try #require(Data(base64Encoded: entry["relativeFloat16"] as! String))
            var values = [Float](repeating: .nan, count: width * height)
            halfData.withUnsafeBytes { (raw: UnsafeRawBufferPointer) in
                let halves = raw.bindMemory(to: Float16.self)
                for index in 0..<min(halves.count, values.count) {
                    values[index] = Float(halves[index])
                }
            }
            let lidarData = try #require(Data(base64Encoded: entry["lidarSamples"] as! String))
            let lidar = try makeLidarBuffer(samples: lidarData, sourceWidth: 518, sourceHeight: 392)
            let absolute = (entry["t"] as! NSNumber).doubleValue
            if t0 == nil { t0 = absolute }
            frames.append(Frame(
                t: absolute - (t0 ?? absolute),
                transform: transform,
                scale: (entry["scale"] as! NSNumber).floatValue,
                offset: (entry["offset"] as! NSNumber).floatValue,
                relative: RelativeDepthMap(width: width, height: height, data: values),
                lidar: lidar,
                lidarData: lidarData
            ))
        }
        return Fixture(intrinsics: intrinsics, imageResolution: resolution, frames: frames)
    }

    /// Sparse (px, py, z) triplets on the 518x392 grid scattered into a
    /// 256x192 DepthFloat32 buffer (unfilled pixels invalid at 0) — the same
    /// convention Scan6ReplayTests uses and makeLidarBuffer there implements.
    private static func makeLidarBuffer(samples: Data, sourceWidth: Int, sourceHeight: Int) throws -> CVPixelBuffer {
        let width = 256, height = 192
        var result: CVPixelBuffer?
        let status = CVPixelBufferCreate(kCFAllocatorDefault, width, height,
                                         kCVPixelFormatType_DepthFloat32, nil, &result)
        try #require(status == kCVReturnSuccess)
        let buffer = try #require(result)
        CVPixelBufferLockBaseAddress(buffer, [])
        defer { CVPixelBufferUnlockBaseAddress(buffer, []) }
        let base = try #require(CVPixelBufferGetBaseAddress(buffer)).assumingMemoryBound(to: Float32.self)
        let stride = CVPixelBufferGetBytesPerRow(buffer) / MemoryLayout<Float32>.stride
        for row in 0..<height { for col in 0..<width { base[row * stride + col] = 0 } }
        samples.withUnsafeBytes { (raw: UnsafeRawBufferPointer) in
            let floats = raw.bindMemory(to: Float32.self)
            for index in Swift.stride(from: 0, to: floats.count - 2, by: 3) {
                let px = Int((floats[index] / Float(sourceWidth) * Float(width)).rounded())
                let py = Int((floats[index + 1] / Float(sourceHeight) * Float(height)).rounded())
                guard px >= 0, px < width, py >= 0, py < height else { continue }
                let z = floats[index + 2]
                let existing = base[py * stride + px]
                if existing <= 0 || z < existing { base[py * stride + px] = z }
            }
        }
        return buffer
    }

    // MARK: - Instrumented replay

    static let histogramBands: [(label: String, range: Range<Float>)] = [
        ("0-8", 0..<8), ("8-12", 8..<12), ("12-15", 12..<15),
        ("15-18", 15..<18), ("18+", 18..<Float.greatestFiniteMagnitude)
    ]

    static func histogram(of vertices: [SIMD3<Float>], origin: SIMD3<Float>) -> (counts: [Int], maxReach: Float) {
        var counts = [Int](repeating: 0, count: histogramBands.count)
        var maxReach: Float = 0
        for vertex in vertices {
            let reach = simd_length(vertex - origin)
            maxReach = max(maxReach, reach)
            for (index, band) in histogramBands.enumerated() where band.range.contains(reach) {
                counts[index] += 1
                break
            }
        }
        return (counts, maxReach)
    }

    static func formatHistogram(_ counts: [Int]) -> String {
        zip(histogramBands, counts).map { "\($0.0.label)m:\($0.1)" }.joined(separator: " ")
    }

    struct RunResult {
        var caps: [Float] = []
        var capTimes: [Double] = []
        var sources: [DepthCalibrationSource] = []
        /// Per-frame integrated mesh vertices, for the far-field
        /// consistency metric (same-ray inter-frame spread).
        var framePoints: [[SIMD3<Float>]] = []
        /// Which evidence population set each kept frame's horizon:
        /// "lidarP95", "anchorShell", or "pseudoP95".
        var horizonBranches: [String] = []
        /// Frames whose joint (LiDAR + map pseudo-sample) fit engaged AND
        /// passed both agreement gates.
        var jointAccepted = 0
        /// Fitted offsets of the kept frames' calibrations.
        var offsets: [Float] = []
        var deepestVisibleAnchor: Float = 0
        var deepestInlyingPseudo: Float = 0
        var dropped = 0
        var tsdfVertexCount = 0
        var tsdfTriangleCount = 0
        var allocatedBlocks = 0
        var reachedCapacity = false
        var deviceBudgetBlocks = 0
        var deviceBudgetReachedCapacity = false
        var tsdfHistogram: [Int] = []
        var tsdfMaxReach: Float = 0
        var mapHistogram: [Int] = []
        var mapMaxReach: Float = 0
        var mapVertexCount = 0
        /// Same-ray inter-frame spread of the integrated per-frame meshes
        /// (Scan6ReplayTests.farFieldDisagreements), stratified at 11 m —
        /// the two-skin/consistency metric this suite lacked.
        var spreadNear: (median: Double, p90: Double, bins: Int) = (0, 0, 0)
        var spreadFar: (median: Double, p90: Double, bins: Int) = (0, 0, 0)
    }

    /// Replicates DepthCalibrationPipeline's joint-fit engagement (private
    /// there) so the replay can attribute each frame's horizon: balance the
    /// pseudo weights against the LiDAR total, fit the combined set, and
    /// accept only if both agreement gates hold — byte-for-byte the pipeline's
    /// own gain formula and tolerances (DepthCalibrationPipeline.swift:78-102).
    static func acceptedJointFit(
        lidarSamples: [DepthCalibrationSample],
        pseudoSamples: [DepthCalibrationSample],
        configuration: DepthCalibrationPipeline.Configuration
    ) -> DepthAnythingProcessor.MaximumLikelihoodCalibration? {
        guard lidarSamples.count > 50,
              pseudoSamples.count >= configuration.minimumJointPseudoSamples else { return nil }
        let lidarTotal = lidarSamples.reduce(0) { $0 + $1.weight }
        let pseudoTotal = pseudoSamples.reduce(0) { $0 + $1.weight }
        guard lidarTotal > 0, pseudoTotal > 0 else { return nil }
        let gain = configuration.pseudoWeightBalance * lidarTotal / pseudoTotal
        let balanced = pseudoSamples.map {
            DepthCalibrationSample(relative: $0.relative, depth: $0.depth, weight: $0.weight * gain)
        }
        guard let joint = RobustDepthCalibration.fit(lidarSamples + balanced),
              DepthCalibrationPipeline.agrees(joint, with: lidarSamples,
                                              toleranceFloor: 0.12, toleranceFraction: 0.15) != false,
              DepthCalibrationPipeline.agrees(joint, with: pseudoSamples,
                                              toleranceFloor: 0.3, toleranceFraction: 0.2) != false
        else { return nil }
        return joint
    }

    /// supportCap's anchor-shell horizon for attribution: the production
    /// shell walk behind the >= 30 population gate, PLUS the fit-agreement
    /// precondition a LiDAR-only frame must now clear — a fit the visible
    /// anchors contradict (agrees == false) inherits no anchor horizon.
    /// Joint-accepted frames carry the agreement by construction.
    static func anchorShellHorizon(
        _ anchorDepths: [Float],
        jointAccepted: Bool,
        lidarSamples: [DepthCalibrationSample],
        pseudoSamples: [DepthCalibrationSample]
    ) -> Float {
        guard anchorDepths.count >= 30 else { return 0 }
        if !jointAccepted {
            guard let lidarOnly = RobustDepthCalibration.fit(lidarSamples),
                  DepthCalibrationPipeline.agrees(lidarOnly, with: pseudoSamples,
                                                  toleranceFloor: 0.3, toleranceFraction: 0.2) != false
            else { return 0 }
        }
        return DepthCalibrationPipeline.shellSupportedHorizon(anchorDepths)
    }

    /// One full replay at the given integration ceiling, mirroring
    /// ARViewContainer's per-frame order exactly: calibrationAnchors ->
    /// DepthCalibrationPipeline.calibrate -> mesh snapshot capped at
    /// integrationDepthCap -> AccumulatedDepthMesh.integrate -> per-pixel
    /// metric depth -> TSDFVolume.integrate at the cap (with the device's
    /// propagated-source weight penalty). The same depth image also feeds a
    /// second TSDF at the previous device build's 24k block budget (the
    /// shipping default is now 32k) for the capacity comparison.
    static func replay(fixture: Fixture, ceiling: Float, tag: String) async throws -> RunResult {
        var configuration = DepthCalibrationPipeline.Configuration.default
        configuration.maximumIntegrationDepth = ceiling

        let map = AccumulatedDepthMesh()
        let volume = TSDFVolume(voxelSize: 0.06, maximumBlocks: 32_000)
        let deviceBudgetVolume = TSDFVolume(voxelSize: 0.06, maximumBlocks: 24_000)
        var run = RunResult()

        for (frameIndex, frame) in fixture.frames.enumerated() {
            let anchors = await map.calibrationAnchors()

            // Instrumentation: recompute the pipeline's evidence populations
            // from the same inputs calibrate() receives.
            let lidarSamples = DepthAnythingProcessor.lidarCalibrationSamples(
                relative: frame.relative, lidarDepthMap: frame.lidar, lidarConfidenceMap: nil
            )
            let lidarP95: Float = lidarSamples.isEmpty ? 0 : {
                let depths = lidarSamples.map(\.depth).sorted()
                return Float(depths[min(depths.count - 1, depths.count * 19 / 20)])
            }()
            var ungated = configuration.propagation
            ungated.minimumSamples = 1
            ungated.minimumCoverage = 0
            let pseudoSamples = MeshDepthPropagation.pseudoDepthSamples(
                anchors: anchors, relative: frame.relative, intrinsics: fixture.intrinsics,
                imageResolution: fixture.imageResolution, transform: frame.transform,
                configuration: ungated
            ) ?? []
            let anchorDepths = MeshDepthPropagation.visibleAnchorDepths(
                anchors: anchors, depthWidth: frame.relative.width,
                depthHeight: frame.relative.height, intrinsics: fixture.intrinsics,
                imageResolution: fixture.imageResolution, transform: frame.transform,
                configuration: configuration.propagation
            )

            guard let result = DepthCalibrationPipeline.calibrate(
                relative: frame.relative, lidarDepthMap: frame.lidar, lidarConfidenceMap: nil,
                anchors: anchors, intrinsics: fixture.intrinsics,
                imageResolution: fixture.imageResolution, transform: frame.transform,
                configuration: configuration
            ) else {
                run.dropped += 1
                // Forensics: which stage refused the frame?
                let lidarOnly = RobustDepthCalibration.fit(lidarSamples)
                let veto = pseudoSamples.count >= configuration.minimumVetoPseudoSamples
                    && lidarOnly.map {
                        DepthCalibrationPipeline.agrees($0, with: pseudoSamples,
                                                        toleranceFloor: 0.3, toleranceFraction: 0.2) == false
                    } == true
                print(String(
                    format: "%@ frame %2d t+%4.1f: DROPPED anchors=%d lidar=%d(p95=%.2f) pseudo=%d anchorDepths=%d lidarOnlyFit=%@ mapVeto=%@ recorded=(%.3f,%.3f)",
                    tag, frameIndex, frame.t, anchors.count, lidarSamples.count, lidarP95,
                    pseudoSamples.count, anchorDepths.count,
                    lidarOnly.map { String(format: "(%.3f,%.3f)", $0.scale, $0.offset) } ?? "nil",
                    veto ? "yes" : "no", frame.scale, frame.offset
                ))
                continue
            }
            run.caps.append(result.integrationDepthCap)
            run.capTimes.append(frame.t)
            run.sources.append(result.source)
            run.offsets.append(result.calibration.offset)

            // Horizon attribution: recompute each evidence population's own
            // horizon (LiDAR p95 / agreement-gated anchor shell / the
            // shell-supported fit-inlying pseudo horizon, exactly
            // supportCap's three inputs) and name the one the max picked.
            // The recomputed cap must reproduce the pipeline's.
            let joint = Self.acceptedJointFit(
                lidarSamples: lidarSamples, pseudoSamples: pseudoSamples,
                configuration: configuration
            )
            if joint != nil { run.jointAccepted += 1 }
            let inlyingDepths = joint.map {
                DepthCalibrationPipeline.fitInlyingPseudoDepths(calibration: $0,
                                                                pseudoSamples: pseudoSamples)
            } ?? []
            let anchorHorizon = Self.anchorShellHorizon(
                anchorDepths, jointAccepted: joint != nil,
                lidarSamples: lidarSamples, pseudoSamples: pseudoSamples
            )
            let pseudoP95 = DepthCalibrationPipeline.shellSupportedHorizon(inlyingDepths)
            let horizon = max(lidarP95, max(pseudoP95, anchorHorizon))
            let branch = horizon <= lidarP95 ? "lidarP95"
                : (pseudoP95 >= anchorHorizon ? "pseudoP95" : "anchorShell")
            run.horizonBranches.append(branch)
            run.deepestVisibleAnchor = max(run.deepestVisibleAnchor, anchorDepths.max() ?? 0)
            run.deepestInlyingPseudo = max(run.deepestInlyingPseudo, inlyingDepths.max() ?? 0)
            let reconstructedCap = min(configuration.maximumIntegrationDepth,
                                       max(DepthAnythingProcessor.minimumCalibratedDepth + 0.1,
                                           configuration.supportDepthFactor * horizon))
            let attributionConsistent = horizon > 0
                && abs(reconstructedCap - result.integrationDepthCap) < 0.01
            #expect(attributionConsistent,
                    "supportCap must decompose into its horizon populations at frame \(frameIndex)")

            // Mesh snapshot capped exactly like ARViewContainer's
            // cappedConfiguration (.accumulatedScene has no finite maximum,
            // so the cap is min(inf, integrationDepthCap)). The fixture's
            // relative maps are 2x-downsampled from the device's 518x392, so
            // step 1 here reproduces the device's .accumulatedScene step 2
            // vertex grid exactly.
            let meshConfiguration = MeshGenerator.DepthAnythingMeshConfiguration(
                step: 1,
                minimumDepth: MeshGenerator.DepthAnythingMeshConfiguration.accumulatedScene.minimumDepth,
                maximumDepth: min(MeshGenerator.DepthAnythingMeshConfiguration.accumulatedScene.maximumDepth,
                                  result.integrationDepthCap),
                maximumDepthDiscontinuity: MeshGenerator.DepthAnythingMeshConfiguration.accumulatedScene.maximumDepthDiscontinuity,
                maximumTriangleCount: MeshGenerator.DepthAnythingMeshConfiguration.accumulatedScene.maximumTriangleCount
            )
            var frameMaxZ: Float = 0
            if let mesh = MeshGenerator.createDepthAnythingMeshSnapshot(
                from: frame.relative, calibration: result.calibration, intrinsics: fixture.intrinsics,
                imageResolution: fixture.imageResolution, transform: frame.transform,
                configuration: meshConfiguration
            ) {
                let worldToCamera = frame.transform.inverse
                for vertex in mesh.vertices {
                    let camera = worldToCamera * SIMD4<Float>(vertex.x, vertex.y, vertex.z, 1)
                    frameMaxZ = max(frameMaxZ, -camera.z)
                }
                run.framePoints.append(mesh.vertices)
                // Full frame context, exactly what the device integrates:
                // with the camera model attached, free-space carving and the
                // anchor-certificate drain run here just as on device (the
                // earlier copy dropped the context and silently disabled
                // both), and the beyond-cap carriers carry the frame's
                // negative evidence for open-background rays.
                let colored = MeshGenerator.DepthAnythingMeshSnapshot(
                    descriptor: mesh.descriptor, vertices: mesh.vertices, indices: mesh.indices,
                    colors: Array(repeating: SIMD3<UInt8>(180, 180, 180), count: mesh.vertices.count),
                    cameraPosition: mesh.cameraPosition,
                    cameraTransform: mesh.cameraTransform,
                    intrinsics: mesh.intrinsics,
                    imageResolution: mesh.imageResolution,
                    beyondCapCarriers: mesh.beyondCapCarriers
                )
                _ = await map.integrate(colored, calibrationSource: result.source)
            }

            // Per-pixel metric depth exactly like integrateTSDF.
            var metric = [Float](repeating: .nan, count: frame.relative.width * frame.relative.height)
            frame.relative.withReadAccess { reader in
                for y in 0..<frame.relative.height {
                    for x in 0..<frame.relative.width {
                        if let depth = DepthAnythingProcessor.calibratedMetricDepth(
                            relativeDepth: reader.value(atX: x, y: y), calibration: result.calibration
                        ) {
                            metric[y * frame.relative.width + x] = depth
                        }
                    }
                }
            }
            let weightScale: Float = result.source == .meshPropagated
                ? AccumulatedDepthMesh.fallbackObservationWeightPenalty : 1
            _ = await volume.integrate(
                depth: metric, colors: nil, width: frame.relative.width, height: frame.relative.height,
                intrinsics: fixture.intrinsics, imageResolution: fixture.imageResolution,
                transform: frame.transform, maximumDepth: result.integrationDepthCap,
                weightScale: weightScale
            )
            _ = await deviceBudgetVolume.integrate(
                depth: metric, colors: nil, width: frame.relative.width, height: frame.relative.height,
                intrinsics: fixture.intrinsics, imageResolution: fixture.imageResolution,
                transform: frame.transform, maximumDepth: result.integrationDepthCap,
                weightScale: weightScale
            )

            print(String(
                format: "%@ frame %2d t+%4.1f: cap=%5.2f source=%@ lidar=%d(p95=%.2f) pseudo=%d anchorDepths=%d(max=%.2f shell=%.2f) joint=%@ inlying=%d(p95=%.2f) branch=%@%@ maxIntegratedZ=%5.2f recorded=(%.3f,%.3f) replayed=(%.3f,%.3f)",
                tag, frameIndex, frame.t, result.integrationDepthCap,
                result.source == .lidar ? "lidar/joint" : "propagated",
                lidarSamples.count, lidarP95, pseudoSamples.count,
                anchorDepths.count, anchorDepths.max() ?? 0, anchorHorizon,
                joint.map { String(format: "(%.3f,%.3f)", $0.scale, $0.offset) } ?? "no",
                inlyingDepths.count, pseudoP95, branch,
                attributionConsistent ? "" : "(ATTRIBUTION MISMATCH)", frameMaxZ,
                frame.scale, frame.offset, result.calibration.scale, result.calibration.offset
            ))
        }

        let fused = await volume.extractMesh()
        let statistics = await volume.statistics
        let deviceBudgetStatistics = await deviceBudgetVolume.statistics
        let snapshot = await map.snapshot()

        let origin4 = fixture.frames[0].transform.columns.3
        let origin = SIMD3<Float>(origin4.x, origin4.y, origin4.z)
        let tsdfHistogram = histogram(of: fused.vertices, origin: origin)
        let mapHistogram = histogram(of: snapshot.vertices, origin: origin)

        run.tsdfVertexCount = fused.vertices.count
        run.tsdfTriangleCount = fused.indices.count / 3
        run.allocatedBlocks = statistics.allocatedBlocks
        run.reachedCapacity = statistics.reachedCapacity
        run.deviceBudgetBlocks = deviceBudgetStatistics.allocatedBlocks
        run.deviceBudgetReachedCapacity = deviceBudgetStatistics.reachedCapacity
        run.tsdfHistogram = tsdfHistogram.counts
        run.tsdfMaxReach = tsdfHistogram.maxReach
        run.mapHistogram = mapHistogram.counts
        run.mapMaxReach = mapHistogram.maxReach
        run.mapVertexCount = snapshot.vertices.count

        // Far-field consistency: same-ray inter-frame spread of the
        // integrated meshes (the metric that sees a layered two-skin far
        // field regardless of which branch admitted it), binned from the
        // mean camera position. World-space vertices of the same surface
        // from different frames differ only by calibration error, so the
        // moving camera adds no parallax to the statistic.
        let meanOrigin = fixture.frames.reduce(SIMD3<Float>.zero) {
            $0 + SIMD3($1.transform.columns.3.x, $1.transform.columns.3.y, $1.transform.columns.3.z)
        } / Float(fixture.frames.count)
        let disagreements = Scan6ReplayTests.farFieldDisagreements(
            pointSets: run.framePoints, origin: meanOrigin)
        run.spreadNear = Scan6ReplayTests.spreadStatistics(
            disagreements.filter { $0.depth <= 11 }.map(\.difference))
        run.spreadFar = Scan6ReplayTests.spreadStatistics(
            disagreements.filter { $0.depth > 11 }.map(\.difference))

        print("\(tag) caps: " + run.caps.map { String(format: "%.2f", $0) }.joined(separator: " "))
        print("\(tag) sources: " + run.sources.map { $0 == .lidar ? "L" : "P" }.joined(separator: " ")
              + " dropped=\(run.dropped)")
        let tally = Dictionary(grouping: run.horizonBranches, by: { $0 }).mapValues(\.count)
        print("\(tag) horizon branches: lidarP95=\(tally["lidarP95"] ?? 0) "
              + "anchorShell=\(tally["anchorShell"] ?? 0) pseudoP95=\(tally["pseudoP95"] ?? 0) "
              + "jointAccepted=\(run.jointAccepted) "
              + String(format: "deepestVisibleAnchor=%.2f deepestInlyingPseudo=%.2f",
                       run.deepestVisibleAnchor, run.deepestInlyingPseudo))
        print("\(tag) offsets: " + run.offsets.map { String(format: "%.3f", $0) }.joined(separator: " ")
              + " nonzero=\(run.offsets.count(where: { $0 != 0 }))")
        print("\(tag) tsdf: \(run.tsdfVertexCount) vertices, \(run.tsdfTriangleCount) triangles, "
              + "\(run.allocatedBlocks) blocks (32k budget), capped=\(run.reachedCapacity)")
        print("\(tag) tsdf@24k(device budget): \(run.deviceBudgetBlocks) blocks, capped=\(run.deviceBudgetReachedCapacity)")
        print("\(tag) tsdf histogram (Euclidean from frame-0 camera): \(formatHistogram(run.tsdfHistogram)) "
              + String(format: "maxReach=%.2f", run.tsdfMaxReach))
        print("\(tag) vertex-map (\(run.mapVertexCount) vertices) histogram: \(formatHistogram(run.mapHistogram)) "
              + String(format: "maxReach=%.2f", run.mapMaxReach))
        print(String(format: "%@ far-field spread: 6-11m median %.3f p90 %.3f (%d bins), >11m median %.3f p90 %.3f (%d bins)",
                     tag, run.spreadNear.median, run.spreadNear.p90, run.spreadNear.bins,
                     run.spreadFar.median, run.spreadFar.p90, run.spreadFar.bins))
        return run
    }

    @Test("Matured far evidence lifts the street scan's cap past the old 12 m ceiling")
    func streetScanCeilingComparison() async throws {
        let fixture = try Self.loadFixture()
        try #require(fixture.frames.count == 17)

        let run12 = try await Self.replay(fixture: fixture, ceiling: 12, tag: "streetscan[12]")
        let run18 = try await Self.replay(fixture: fixture, ceiling: 18, tag: "streetscan[18]")

        // Frame retention, MEASURED post-fix: 5 of 17 drop, every one a
        // fixture artifact of the bag's sparse published LiDAR (truncated at
        // ~5 m): RobustDepthCalibration's 65% inlier gate refuses those
        // frames' LiDAR-only fits (the device, with dense untruncated LiDAR,
        // kept all 17), and the map fallback holds far fewer than its
        // 200-sample gate. Four are the t+11.9-17.3 pan frames the pre-fix
        // replay also dropped; t+7.4 joined them once the anchor map carried
        // real far evidence — its joint fit, which used to squeak past the
        // gates against a near-only pseudo population, now fails the same
        // 65% inlier gate it always deserved to fail (its LiDAR-only fit
        // was nil on this fixture from the start).
        #expect(run12.dropped == 5, "12 m run dropped \(run12.dropped)/17 frames")
        #expect(run18.dropped == 5, "18 m run dropped \(run18.dropped)/17 frames")
        #expect(run12.caps.count == 12, "12 m run kept \(run12.caps.count)/17 frames")
        #expect(run12.sources.allSatisfy { $0 == .lidar },
                "The source never leaves .lidar on this scan (no kept frame falls through to map-only calibration)")
        #expect(run18.sources.allSatisfy { $0 == .lidar })

        // Cap trajectory, MEASURED post-fix with device-faithful carving
        // (free-space carve + certificate drain active, as on device):
        // matured grazing anchors (incidence-floored certificate weights)
        // survive the chained z-buffer visibility, so the far-evidence
        // branches set the horizon on 6 of 12 kept frames (was 1 of 13 on
        // the LiDAR-pinned trajectory whose caps never left 6.7-8.8 m).
        // The 12 m run saturates its ceiling on four frames; the 18 m run
        // reaches 14.26 m. The old t+13 new-territory collapse below 7 m
        // is gone: the pan dips only to 8.33-9.06 m (the certificate drain
        // honestly suspends some mid-pan anchors; they requalify by
        // t+18.4), and no kept frame caps below 7 m at all.
        let maxCap12 = try #require(run12.caps.max())
        let maxCap18 = try #require(run18.caps.max())
        #expect(maxCap12 == 12, "The 12 m run must saturate its ceiling (got \(maxCap12))")
        #expect(run12.caps.allSatisfy { $0 <= 12 })
        #expect(maxCap18 > 12.5, "The 18 m run's horizon must clear the old ceiling (got \(maxCap18))")
        #expect(maxCap18 <= 18)
        #expect(run12.caps.allSatisfy { $0 >= 7 },
                "No kept frame may cap below 7 m any more (caps \(run12.caps))")
        let panCaps = zip(run12.capTimes, run12.caps).filter { $0.0 > 11 && $0.0 < 14 }.map(\.1)
        #expect(!panCaps.isEmpty && panCaps.allSatisfy { $0 >= 8 },
                "The t+13 pan must keep an evidence-supported cap of at least 8 m (caps \(run12.caps))")
        let final12 = try #require(run12.caps.last)
        #expect(final12 >= 8, "The cap must stay recovered after the pan (got \(final12))")

        // Horizon attribution: the far-evidence branches must do real work.
        // The anchor shell sets five horizons (to 8.13 m, vs LiDAR p95
        // 3.8-4.9) and the shell-supported fit-inlying-pseudo horizon sets
        // the final frame's (8.15 m -> cap 14.26) — all through the >= 30
        // population, >= 10-shell, and fit-agreement gates.
        let nonLidar12 = run12.horizonBranches.count(where: { $0 != "lidarP95" })
        let nonLidar18 = run18.horizonBranches.count(where: { $0 != "lidarP95" })
        #expect(nonLidar12 >= 6, "Far branches set \(nonLidar12)/12 horizons: \(run12.horizonBranches)")
        #expect(nonLidar18 >= 6, "Far branches set \(nonLidar18)/12 horizons: \(run18.horizonBranches)")
        #expect(run18.horizonBranches.contains("anchorShell"),
                "The anchor-shell branch must engage (branches \(run18.horizonBranches))")
        #expect(run18.deepestVisibleAnchor > 8,
                "Qualifying anchors must reach past 8 m (got \(run18.deepestVisibleAnchor); pre-fix max was 5.63)")

        // A ceiling only clips: below 12 m the two trajectories must agree
        // exactly, so any divergence is the 18 m run exercising headroom.
        try #require(run12.caps.count == run18.caps.count)
        #expect(zip(run12.caps, run18.caps).allSatisfy { $0 == $1 || $1 > 12 },
                "Caps may only diverge beyond the old ceiling: \(run12.caps) vs \(run18.caps)")

        // Fused output: the far field must actually reach the fused
        // surfaces. Pre-fix the TSDF stopped at 10.09 m Euclidean (vertex
        // map 10.61) with the 12-18 m band EMPTY; post-fix both runs carry
        // five-digit 12-18 m band populations out past 15 m, still without
        // saturating either block budget.
        #expect(run18.tsdfVertexCount > 5_000)
        #expect(run18.tsdfHistogram[1] > 0, "The 8-12 m band must survive fusion")
        #expect(run18.tsdfMaxReach > 15)
        #expect(run18.mapMaxReach > 15)
        #expect(!run18.reachedCapacity, "18 m run saturated the 32k block budget")
        #expect(!run18.deviceBudgetReachedCapacity,
                "18 m run saturated even the 24k device block budget")
        #expect(!run12.reachedCapacity && !run12.deviceBudgetReachedCapacity)

        // The ceiling comparison this suite was built for, HARD now that the
        // horizon fixes let the 18 m run truly integrate deeper than the
        // 12 m run. The margin is real but fixture-bounded: the bag's
        // published clouds were pre-truncated at the on-device caps (z
        // 6.7-8.3 m, 12 m only in the final frame), so the deepest per-frame
        // z content tops out at ~12.1 m and the >12 m Euclidean bands come
        // from that z plus the frustum-corner factor and camera travel.
        func band12to18(_ counts: [Int]) -> Int { counts[2] + counts[3] }
        #expect(band12to18(run12.tsdfHistogram) > 10_000,
                "Even the 12 m ceiling run must populate 12-18 m Euclidean (got \(band12to18(run12.tsdfHistogram)))")
        #expect(band12to18(run18.mapHistogram) > 3_000,
                "The vertex map must carry a 12-18 m band (got \(band12to18(run18.mapHistogram)))")
        #expect(run18.tsdfMaxReach > run12.tsdfMaxReach,
                "TSDF max reach: 18 m run \(run18.tsdfMaxReach) vs 12 m run \(run12.tsdfMaxReach)")
        #expect(band12to18(run18.tsdfHistogram) > band12to18(run12.tsdfHistogram),
                "TSDF 12-18 m band: 18 m run \(band12to18(run18.tsdfHistogram)) vs 12 m run \(band12to18(run12.tsdfHistogram))")
        #expect(run18.mapMaxReach > run12.mapMaxReach,
                "Vertex-map max reach: 18 m run \(run18.mapMaxReach) vs 12 m run \(run12.mapMaxReach)")
        #expect(band12to18(run18.mapHistogram) > band12to18(run12.mapHistogram),
                "Vertex-map 12-18 m band: 18 m run \(band12to18(run18.mapHistogram)) vs 12 m run \(band12to18(run12.mapHistogram))")

        // FAR-FIELD CONSISTENCY. The cap/branch/band pins above cannot see
        // a layered two-skin far field — a calibration-family-flip ghost
        // (measured 1.34-1.40 m same-ray displacement at 8-12 m on device)
        // could ride every one of them green. The same-ray inter-frame
        // spread can: with device-faithful carving and the certificate
        // drain active this replay measures 0.370 m median / 0.994 m p90
        // over 498 bins in the 6-11 m stratum (below even scan6's 0.396
        // pre-fix like-for-like median). A breach here is reintroduced
        // layering, whatever admitted it.
        try #require(run18.spreadNear.bins > 300, "The 6-11 m stratum must stay well populated")
        #expect(run18.spreadNear.median < 0.42,
                "6-11 m same-ray spread median regressed to \(run18.spreadNear.median) (measured 0.370 at fix time)")
        #expect(run18.spreadNear.p90 < 1.15,
                "6-11 m same-ray spread p90 regressed to \(run18.spreadNear.p90) (measured 0.994 at fix time)")
        // The >11 m band yields few same-ray bins on this fixture (the
        // deep caps only engage late in the scan), so it takes the same
        // fusible-band bound scan6's >11 m stratum carries: a ~1.4 m flip
        // skin pushes a bin past it, TSDF truncation (<= 0.8 m) cannot
        // absorb it, and with these few bins the p90 is effectively the
        // worst bin.
        #expect(run18.spreadFar.bins == 0 || run18.spreadFar.p90 < 2.2,
                ">11 m same-ray spread reached \(run18.spreadFar.p90) over \(run18.spreadFar.bins) bins (measured 1.15 at fix time)")
    }
}
