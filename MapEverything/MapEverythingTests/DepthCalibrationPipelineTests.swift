import CoreVideo
import Foundation
import Testing
import simd
@testable import MapEverything

/// Exercises the guarded far-evidence horizon: a JOINT-accepted calibration
/// whose pseudo-samples the fit predicts within RobustDepthCalibration's
/// inlier tolerance extends the integration cap to the support factor times
/// their p95 — the same contract LiDAR evidence gets — while a LiDAR-only
/// frame with the same visible anchors extends nothing, and pseudo-samples
/// the fit cannot predict are excluded from the horizon.
struct DepthCalibrationPipelineTests {
    private let imageResolution = CGSize(width: 256, height: 192)
    private var intrinsics: simd_float3x3 {
        simd_float3x3(columns: (
            SIMD3<Float>(200, 0, 0),
            SIMD3<Float>(0, 205, 0),
            SIMD3<Float>(126, 98, 1)
        ))
    }
    private var transform: simd_float4x4 {
        let angle: Float = 0.3
        return simd_float4x4(
            SIMD4<Float>(cos(angle), 0, -sin(angle), 0),
            SIMD4<Float>(0, 1, 0, 0),
            SIMD4<Float>(sin(angle), 0, cos(angle), 0),
            SIMD4<Float>(0.4, -1.1, 2.2, 1)
        )
    }

    private let truth = DepthAnythingProcessor.MaximumLikelihoodCalibration(scale: 0.5, offset: 0.05)
    private let gridWidth = 518
    private let gridHeight = 392

    /// Scene depth: 1.2 m at the left edge rising to 12.2 m at the right —
    /// LiDAR (valid below 5 m) sees the left third, the anchors sit at
    /// 6-9.5 m where only the map can testify.
    private func sceneDepth(gridX: Int) -> Float {
        1.2 + 11 * Float(gridX) / Float(gridWidth - 1)
    }

    private func relativeValue(forDepth depth: Float) -> Float {
        (1 / depth - truth.offset) / truth.scale
    }

    private var relativeMap: RelativeDepthMap {
        var data = [Float](repeating: 0, count: gridWidth * gridHeight)
        for y in 0..<gridHeight {
            for x in 0..<gridWidth {
                data[y * gridWidth + x] = relativeValue(forDepth: sceneDepth(gridX: x))
            }
        }
        return RelativeDepthMap(width: gridWidth, height: gridHeight, data: data)
    }

    /// World point whose image under the production projection is exactly
    /// relative-grid pixel (x, y) at `depth` (inverse of the pipeline's
    /// scaled pinhole model).
    private func worldPoint(gridX: Int, gridY: Int, depth: Float) -> SIMD3<Float> {
        let scaleX = Float(gridWidth) / Float(imageResolution.width)
        let scaleY = Float(gridHeight) / Float(imageResolution.height)
        let fx = intrinsics[0][0] * scaleX
        let fy = intrinsics[1][1] * scaleY
        let cx = intrinsics[2][0] * scaleX
        let cy = intrinsics[2][1] * scaleY
        let camera = SIMD4<Float>(
            ((Float(gridX) - cx) / fx) * depth,
            ((cy - Float(gridY)) / fy) * depth,
            -depth,
            1
        )
        let world = transform * camera
        return SIMD3<Float>(world.x, world.y, world.z)
    }

    /// 25 map anchors at 6.0-9.6 m: enough pseudo-samples for a joint fit at
    /// the tests' lowered gate, yet deliberately BELOW the >= 30 population
    /// supportCap's anchor-shell branch requires — any cap extension is
    /// attributable to the inlying-pseudo horizon alone.
    private var deepAnchors: [SIMD3<Float>] {
        (0..<25).map { index in
            let gridX = 226 + index * 7
            let gridY = 40 + index * 12
            return worldPoint(gridX: gridX, gridY: gridY, depth: sceneDepth(gridX: gridX))
        }
    }

    private func lidarBuffer() throws -> CVPixelBuffer {
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
        for y in 0..<height {
            for x in 0..<width {
                let gridX = min(gridWidth - 1, Int((Float(x) / Float(width) * Float(gridWidth)).rounded()))
                let depth = sceneDepth(gridX: gridX)
                base[y * stride + x] = depth < 4.9 ? depth : .nan
            }
        }
        return buffer
    }

    @Test("A joint-accepted fit with deep inlying pseudo-samples extends the integration cap")
    func jointAcceptedDeepPseudoSamplesRaiseCap() throws {
        var configuration = DepthCalibrationPipeline.Configuration.default
        configuration.minimumJointPseudoSamples = 20
        let result = try #require(DepthCalibrationPipeline.calibrate(
            relative: relativeMap, lidarDepthMap: try lidarBuffer(), lidarConfidenceMap: nil,
            anchors: deepAnchors, intrinsics: intrinsics, imageResolution: imageResolution,
            transform: transform, configuration: configuration
        ))
        #expect(result.source == .lidar)
        #expect(abs(result.calibration.scale - truth.scale) < 0.02)
        // Horizon = inlying-pseudo p95 (~9.4 m) >> LiDAR p95 (~4.8 m):
        // cap ≈ 1.75 * 9.4 ≈ 16.5, far beyond LiDAR's ~8.4 m reach.
        #expect(result.integrationDepthCap > 15,
                "Deep agreeing pseudo-samples must extend the cap (got \(result.integrationDepthCap))")
        #expect(result.integrationDepthCap <= configuration.maximumIntegrationDepth)
    }

    @Test("A LiDAR-only fit with the same visible anchors does not extend the cap")
    func lidarOnlySameAnchorsDoesNotRaiseCap() throws {
        var configuration = DepthCalibrationPipeline.Configuration.default
        // The joint branch cannot engage, so the identical frame — including
        // the same 25 visible deep anchors — falls to the LiDAR-only fit.
        configuration.minimumJointPseudoSamples = 10_000
        let result = try #require(DepthCalibrationPipeline.calibrate(
            relative: relativeMap, lidarDepthMap: try lidarBuffer(), lidarConfidenceMap: nil,
            anchors: deepAnchors, intrinsics: intrinsics, imageResolution: imageResolution,
            transform: transform, configuration: configuration
        ))
        #expect(result.source == .lidar)
        // Without a joint acceptance the pseudo evidence must extend nothing:
        // the horizon stays at LiDAR's p95 (~4.8 m -> cap ~8.4 m).
        #expect(result.integrationDepthCap < 10,
                "Visible anchors alone must not extend a LiDAR-only cap (got \(result.integrationDepthCap))")
        #expect(result.integrationDepthCap > 7)
    }

    @Test("Pseudo-samples the fit cannot predict within tolerance are excluded from the horizon")
    func disagreeingPseudoSamplesAreExcludedFromHorizon() {
        var samples: [DepthCalibrationSample] = []
        for index in 0..<30 {
            let depth = 4 + Float(index) * 0.1
            samples.append(DepthCalibrationSample(relative: Double(relativeValue(forDepth: depth)),
                                                  depth: Double(depth), weight: 1))
        }
        // Relative values imaging 8 m paired with a claimed 11 m depth:
        // residual 3 m >> max(0.06, 0.08 * 11) — pure disagreement.
        for _ in 0..<10 {
            samples.append(DepthCalibrationSample(relative: Double(relativeValue(forDepth: 8)),
                                                  depth: 11, weight: 1))
        }
        // Boundary pair at depth 10 (tolerance 0.8): predicted 10.7 is an
        // inlier, predicted 10.9 is not.
        samples.append(DepthCalibrationSample(relative: Double(relativeValue(forDepth: 10.7)),
                                              depth: 10, weight: 1))
        samples.append(DepthCalibrationSample(relative: Double(relativeValue(forDepth: 10.9)),
                                              depth: 10.0001, weight: 1))

        let inlying = DepthCalibrationPipeline.fitInlyingPseudoDepths(
            calibration: truth, pseudoSamples: samples
        )
        #expect(inlying.count == 31)
        #expect(inlying.filter { $0 > 10.5 }.isEmpty,
                "The 11 m disagreeing samples must be excluded (got \(inlying.sorted()))")
        #expect(inlying.contains { abs($0 - 10) < 0.01 })

        // And the excluded samples must not move the horizon: p95 of the
        // inlying set (~6.9 m) drives the cap, not the claimed 11 m.
        let cap = DepthCalibrationPipeline.supportCap(
            lidarSamples: [], anchorDepths: [], inlyingPseudoDepths: inlying
        )
        #expect(cap < 1.75 * 10.1)
        #expect(cap > 1.75 * 6.0)
    }

    @Test("The inlying-pseudo population is an independent horizon in supportCap")
    func supportCapTreatsInlyingPseudoDepthsAsOwnPopulation() {
        let lidar = (0..<100).map {
            DepthCalibrationSample(relative: 1, depth: 2 + Double($0) * 0.02, weight: 1)
        }
        let base = DepthCalibrationPipeline.supportCap(lidarSamples: lidar)
        let extended = DepthCalibrationPipeline.supportCap(
            lidarSamples: lidar, inlyingPseudoDepths: Array(repeating: 10, count: 20)
        )
        #expect(abs(base - 1.75 * 3.9) < 0.2)
        #expect(abs(extended - 17.5) < 0.01)
        // The defaulted parameter leaves every existing call site unchanged.
        #expect(DepthCalibrationPipeline.supportCap(lidarSamples: lidar, inlyingPseudoDepths: []) == base)
    }

    @Test("A lone deep inlying pseudo-sample cannot set the horizon")
    func loneDeepInlyingPseudoSampleCannotSetHorizon() {
        let lidar = (0..<100).map {
            DepthCalibrationSample(relative: 1, depth: 2 + Double($0) * 0.02, weight: 1)
        }
        // 28 corroborated near depths plus 1-2 deep strays (an isolated real
        // feature, or floor-matured flip ghosts): for n <= 20 the old p95
        // index was exactly the maximum, and even at n = 30 a thin deep tail
        // must not push the cap toward the ceiling. The shell bar demands
        // >= 10 members inside [0.7D, D] before D counts.
        let nearDepths = (0..<28).map { 5 + Float($0) * 0.04 }
        for strays in 1...2 {
            let population = nearDepths + Array(repeating: Float(14), count: strays)
            let cap = DepthCalibrationPipeline.supportCap(
                lidarSamples: lidar, inlyingPseudoDepths: population
            )
            // The horizon falls back to the deepest SUPPORTED depth (6.08 m,
            // whose shell [4.26, 6.08] holds all 28 near members), never 14.
            #expect(cap < 1.75 * 7,
                    "\(strays) stray deep inliers must not certify a deep cap (got \(cap))")
            #expect(cap > 1.75 * 5, "The supported near frontier still extends (got \(cap))")
        }
        // Ten corroborating members inside the deep shell DO certify it.
        let supported = nearDepths + Array(repeating: Float(14), count: 10)
        let cap = DepthCalibrationPipeline.supportCap(
            lidarSamples: lidar, inlyingPseudoDepths: supported
        )
        #expect(abs(cap - min(18, 1.75 * 14)) < 0.01)
    }

    /// 40 anchors whose claimed depths are 35% deeper than what the scene
    /// actually images at their pixels, placed as 20 z-buffer-cell PAIRS:
    /// the per-cell winner selection collapses them to ~20 pseudo-samples
    /// (below the 30-sample joint gate, and far below the 60-sample veto
    /// bar), while visibleAnchorDepths — no winner selection — still counts
    /// all 40, clearing supportCap's >= 30 anchor population gate with a
    /// fully populated shell at the claimed 10.2-15.7 m. Exactly the
    /// finding's regime: the frame is LiDAR-only, the map cannot veto it,
    /// yet every one of its pseudo-samples disagrees with the LiDAR fit
    /// beyond the max(0.3, 0.2*d) agreement tolerance (0.35d > 0.27d).
    private var contradictingDeepAnchors: [SIMD3<Float>] {
        (0..<20).flatMap { index -> [SIMD3<Float>] in
            let gridX = 300 + index * 10
            let gridY = 40 + index * 12
            let depth = sceneDepth(gridX: gridX) * 1.35
            return [worldPoint(gridX: gridX, gridY: gridY, depth: depth),
                    worldPoint(gridX: gridX + 1, gridY: gridY + 1, depth: depth)]
        }
    }

    @Test("A LiDAR-only fit contradicted by the visible anchors does not inherit their shell horizon")
    func contradictedLidarOnlyFitFallsBackToLidarHorizon() throws {
        let configuration = DepthCalibrationPipeline.Configuration.default
        let anchors = contradictingDeepAnchors
        let result = try #require(DepthCalibrationPipeline.calibrate(
            relative: relativeMap, lidarDepthMap: try lidarBuffer(), lidarConfidenceMap: nil,
            anchors: anchors, intrinsics: intrinsics, imageResolution: imageResolution,
            transform: transform, configuration: configuration
        ))
        // Joint fitting cannot engage (~20 pseudo-samples < 30) and the map
        // cannot veto (< 60), so the LiDAR-only fit is accepted —
        // correctly, LiDAR owns where it sees. Scale must be the LiDAR
        // family (0.5, recovered from the narrow 2-4.9 m support), never
        // the anchors' 0.5/1.35 = 0.37.
        #expect(result.source == .lidar)
        #expect(abs(result.calibration.scale - truth.scale) < 0.05)
        // But the same anchors that just testified AGAINST this calibration
        // must not hand it their 10.2-15.7 m shell horizon (pre-fix: the
        // 18 m ceiling): the frame writes only to the LiDAR evidence
        // horizon (~4.8 * 1.75).
        #expect(result.integrationDepthCap < 10,
                "A contradicted frame must not write to the disputed far field (got \(result.integrationDepthCap))")
        #expect(result.integrationDepthCap > 7)
    }

    @Test("A LiDAR-only fit the visible anchors corroborate keeps their shell horizon")
    func agreeingAnchorsStillExtendLidarOnlyHorizon() throws {
        var configuration = DepthCalibrationPipeline.Configuration.default
        // Joint fitting disabled: the frame is LiDAR-only by construction,
        // with 40 agreeing anchors at 6.9-11.8 m (shell populated).
        configuration.minimumJointPseudoSamples = 10_000
        let anchors = (0..<40).map { index -> SIMD3<Float> in
            let gridX = 270 + (index % 20) * 12
            let gridY = 30 + index * 8
            return worldPoint(gridX: gridX, gridY: gridY, depth: sceneDepth(gridX: gridX))
        }
        let result = try #require(DepthCalibrationPipeline.calibrate(
            relative: relativeMap, lidarDepthMap: try lidarBuffer(), lidarConfidenceMap: nil,
            anchors: anchors, intrinsics: intrinsics, imageResolution: imageResolution,
            transform: transform, configuration: configuration
        ))
        #expect(result.source == .lidar)
        #expect(result.integrationDepthCap > 12,
                "Corroborated anchors keep extending a LiDAR-only frame (got \(result.integrationDepthCap))")
    }
}
