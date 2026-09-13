import CoreVideo
import Foundation
import Testing
import simd
@testable import MapEverything

/// Exercises the mesh-propagated fallback calibration: anchors synthesized by
/// inverse-projecting known grid pixels at known depths must round-trip
/// through the projection, survive occlusion resolution, and recover the
/// ground-truth affine inverse-depth calibration — while every rejection gate
/// refuses to invent scale.
struct MeshDepthPropagationTests {
    /// Camera with distinct focal lengths, an off-center principal point, and
    /// a rotated + translated pose, so no convention error can hide.
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

    /// Inverse of the production projection: world point whose image under the
    /// scaled pinhole model is exactly relative-grid pixel (x, y) at `depth`.
    private func worldPoint(gridX: Int, gridY: Int, depth: Float,
                            depthWidth: Int, depthHeight: Int) -> SIMD3<Float> {
        let scaleX = Float(depthWidth) / Float(imageResolution.width)
        let scaleY = Float(depthHeight) / Float(imageResolution.height)
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

    /// Smoothly varying scene depth, spanning ~2–14 m across the frame.
    private func sceneDepth(gridX: Int, gridY: Int, width: Int, height: Int) -> Float {
        2 + 10 * Float(gridX) / Float(max(width - 1, 1)) + 2 * Float(gridY) / Float(max(height - 1, 1))
    }

    private func relativeValue(forDepth depth: Float) -> Float {
        (1 / depth - truth.offset) / truth.scale
    }

    private func relativeMap(width: Int, height: Int,
                             depthAt: (Int, Int) -> Float) -> RelativeDepthMap {
        var data = [Float](repeating: 0, count: width * height)
        for y in 0..<height {
            for x in 0..<width {
                data[y * width + x] = relativeValue(forDepth: depthAt(x, y))
            }
        }
        return RelativeDepthMap(width: width, height: height, data: data)
    }

    @Test("Projection lands anchors on the exact source pixel and depth")
    func projectionMatchesGridConvention() throws {
        let width = 128, height = 96
        let anchor = worldPoint(gridX: 37, gridY: 61, depth: 4.25, depthWidth: width, depthHeight: height)
        var configuration = MeshDepthPropagation.Configuration()
        configuration.minimumSamples = 1
        configuration.minimumCoverage = 0
        let visible = try #require(MeshDepthPropagation.visibleAnchors(
            anchors: [anchor], depthWidth: width, depthHeight: height,
            intrinsics: intrinsics, imageResolution: imageResolution,
            transform: transform, configuration: configuration
        ))
        let projected = try #require(visible.first)
        #expect(visible.count == 1)
        #expect(projected.gridX == 37)
        #expect(projected.gridY == 61)
        #expect(abs(projected.depth - 4.25) < 1e-3)
    }

    @Test("A full-frame anchor field recovers the ground-truth calibration")
    func recoversKnownCalibrationFromSyntheticScene() throws {
        let width = 518, height = 392
        var anchors: [SIMD3<Float>] = []
        for y in stride(from: 0, to: height, by: 8) {
            for x in stride(from: 0, to: width, by: 8) {
                anchors.append(worldPoint(
                    gridX: x, gridY: y,
                    depth: sceneDepth(gridX: x, gridY: y, width: width, height: height),
                    depthWidth: width, depthHeight: height
                ))
            }
        }
        let relative = relativeMap(width: width, height: height) {
            sceneDepth(gridX: $0, gridY: $1, width: width, height: height)
        }
        let fit = try #require(MeshDepthPropagation.fitCalibration(
            anchors: anchors, relative: relative, intrinsics: intrinsics,
            imageResolution: imageResolution, transform: transform
        ))
        #expect(abs(fit.scale - truth.scale) < 0.01)
        #expect(abs(fit.offset - truth.offset) < 0.01)
    }

    @Test("The z-buffer keeps a hidden back wall out of the fit")
    func occludedBackSurfaceDoesNotCorruptFit() throws {
        let width = 518, height = 392
        var anchors: [SIMD3<Float>] = []
        for y in stride(from: 0, to: height, by: 8) {
            for x in stride(from: 0, to: width, by: 8) {
                // A slightly tilted front wall (3–5 m) fully hides a back wall
                // at 9 m that projects onto the same pixels.
                let frontDepth = 3 + 2 * Float(x) / Float(width - 1)
                anchors.append(worldPoint(gridX: x, gridY: y, depth: frontDepth,
                                          depthWidth: width, depthHeight: height))
                anchors.append(worldPoint(gridX: x, gridY: y, depth: 9,
                                          depthWidth: width, depthHeight: height))
            }
        }
        let relative = relativeMap(width: width, height: height) { x, _ in
            3 + 2 * Float(x) / Float(width - 1)
        }
        let visible = try #require(MeshDepthPropagation.visibleAnchors(
            anchors: anchors, depthWidth: width, depthHeight: height,
            intrinsics: intrinsics, imageResolution: imageResolution, transform: transform
        ))
        #expect(visible.allSatisfy { $0.depth < 6 })

        let fit = try #require(MeshDepthPropagation.fitCalibration(
            anchors: anchors, relative: relative, intrinsics: intrinsics,
            imageResolution: imageResolution, transform: transform
        ))
        #expect(abs(fit.scale - truth.scale) < 0.02)
        #expect(abs(fit.offset - truth.offset) < 0.02)

        // Control documenting why the z-buffer exists: pairing both walls'
        // depths with the front wall's relative values must not produce the
        // front-wall calibration.
        let contaminated = anchors.enumerated().map { index, _ -> DepthCalibrationSample in
            let front = index % 2 == 0
            let depth: Float = front ? 4 : 9
            return DepthCalibrationSample(relative: Double(relativeValue(forDepth: 4)),
                                          depth: Double(depth), weight: 1)
        }
        let naive = RobustDepthCalibration.fit(contaminated)
        #expect(naive == nil || abs(naive!.scale - truth.scale) > 0.05)
    }

    @Test("A surface below the sampling range still occludes hidden anchors")
    func nearOccluderSuppressesHiddenAnchors() {
        let width = 128, height = 96
        var configuration = MeshDepthPropagation.Configuration()
        configuration.minimumSamples = 1
        configuration.minimumCoverage = 0
        // A scanned surface 0.2 m away cannot serve as a reference (below the
        // sampling range) but must still hide the geometry behind it — the
        // hidden anchor would otherwise pair with relative values that image
        // the near surface.
        let occluder = worldPoint(gridX: 64, gridY: 48, depth: 0.2, depthWidth: width, depthHeight: height)
        let hidden = worldPoint(gridX: 64, gridY: 48, depth: 5, depthWidth: width, depthHeight: height)
        let visible = MeshDepthPropagation.visibleAnchors(
            anchors: [occluder, hidden], depthWidth: width, depthHeight: height,
            intrinsics: intrinsics, imageResolution: imageResolution,
            transform: transform, configuration: configuration
        )
        #expect(visible == nil)
    }

    @Test("Too few anchors refuse to calibrate")
    func rejectsInsufficientSamples() {
        let width = 518, height = 392
        var anchors: [SIMD3<Float>] = []
        for y in stride(from: 0, to: height, by: 8) {
            for x in stride(from: 0, to: width, by: 64) where anchors.count < 120 {
                anchors.append(worldPoint(
                    gridX: x, gridY: y,
                    depth: sceneDepth(gridX: x, gridY: y, width: width, height: height),
                    depthWidth: width, depthHeight: height
                ))
            }
        }
        let relative = relativeMap(width: width, height: height) {
            sceneDepth(gridX: $0, gridY: $1, width: width, height: height)
        }
        #expect(MeshDepthPropagation.fitCalibration(
            anchors: anchors, relative: relative, intrinsics: intrinsics,
            imageResolution: imageResolution, transform: transform
        ) == nil)
    }

    @Test("Anchors confined to one corner cannot calibrate the whole frame")
    func rejectsPoorImageCoverage() {
        let width = 518, height = 392
        var anchors: [SIMD3<Float>] = []
        for y in stride(from: 0, to: height / 4, by: 2) {
            for x in stride(from: 0, to: width / 4, by: 2) {
                anchors.append(worldPoint(
                    gridX: x, gridY: y,
                    depth: sceneDepth(gridX: x, gridY: y, width: width, height: height),
                    depthWidth: width, depthHeight: height
                ))
            }
        }
        let relative = relativeMap(width: width, height: height) {
            sceneDepth(gridX: $0, gridY: $1, width: width, height: height)
        }
        #expect(MeshDepthPropagation.fitCalibration(
            anchors: anchors, relative: relative, intrinsics: intrinsics,
            imageResolution: imageResolution, transform: transform
        ) == nil)
    }

    @Test("Behind-camera and out-of-range anchors contribute nothing")
    func ignoresOutOfRangeAndBehindCameraAnchors() {
        let width = 518, height = 392
        var anchors: [SIMD3<Float>] = []
        for y in stride(from: 0, to: height, by: 4) {
            for x in stride(from: 0, to: width, by: 4) {
                anchors.append(worldPoint(gridX: x, gridY: y, depth: -3,
                                          depthWidth: width, depthHeight: height))
                anchors.append(worldPoint(gridX: x, gridY: y, depth: 0.15,
                                          depthWidth: width, depthHeight: height))
                anchors.append(worldPoint(gridX: x, gridY: y, depth: 60,
                                          depthWidth: width, depthHeight: height))
            }
        }
        let relative = relativeMap(width: width, height: height) {
            sceneDepth(gridX: $0, gridY: $1, width: width, height: height)
        }
        #expect(MeshDepthPropagation.fitCalibration(
            anchors: anchors, relative: relative, intrinsics: intrinsics,
            imageResolution: imageResolution, transform: transform
        ) == nil)
    }

    @Test("Sparse LiDAR returns veto a mesh calibration that contradicts them")
    func sparseLidarCrossCheckRejectsGrossScaleError() throws {
        let width = 518, height = 392
        var anchors: [SIMD3<Float>] = []
        for y in stride(from: 0, to: height, by: 8) {
            for x in stride(from: 0, to: width, by: 8) {
                anchors.append(worldPoint(
                    gridX: x, gridY: y,
                    depth: sceneDepth(gridX: x, gridY: y, width: width, height: height),
                    depthWidth: width, depthHeight: height
                ))
            }
        }
        let relative = relativeMap(width: width, height: height) {
            sceneDepth(gridX: $0, gridY: $1, width: width, height: height)
        }

        // LiDAR pixels seeing the same scene at HALF the mesh-implied depth:
        // the mesh calibration is wrong in absolute scale and must be vetoed.
        let disagreeing = try lidarBuffer(width: 64, height: 48) { nx, ny in
            let x = min(width - 1, Int((nx * Float(width)).rounded()))
            let y = min(height - 1, Int((ny * Float(height)).rounded()))
            let depth = sceneDepth(gridX: x, gridY: y, width: width, height: height) / 2
            return depth < 5 ? depth : .nan
        }
        #expect(MeshDepthPropagation.fitCalibration(
            anchors: anchors, relative: relative, intrinsics: intrinsics,
            imageResolution: imageResolution, transform: transform,
            lidarDepthMap: disagreeing
        ) == nil)

        // The same returns at the mesh-implied depths pass the cross-check.
        let agreeing = try lidarBuffer(width: 64, height: 48) { nx, ny in
            let x = min(width - 1, Int((nx * Float(width)).rounded()))
            let y = min(height - 1, Int((ny * Float(height)).rounded()))
            let depth = sceneDepth(gridX: x, gridY: y, width: width, height: height)
            return depth < 5 ? depth : .nan
        }
        let fit = try #require(MeshDepthPropagation.fitCalibration(
            anchors: anchors, relative: relative, intrinsics: intrinsics,
            imageResolution: imageResolution, transform: transform,
            lidarDepthMap: agreeing
        ))
        #expect(abs(fit.scale - truth.scale) < 0.01)
    }

    @Test("Fewer than ten valid LiDAR returns cannot judge the calibration")
    func tooFewLidarReturnsSkipTheCrossCheck() throws {
        // Everything invalid except a handful of wildly disagreeing pixels:
        // below the minimum, the cross-check must abstain rather than veto.
        let sparse = try lidarBuffer(width: 64, height: 48) { nx, ny in
            nx < 0.05 && ny < 0.2 ? 1.0 : .nan
        }
        let relative = relativeMap(width: 518, height: 392) {
            sceneDepth(gridX: $0, gridY: $1, width: 518, height: 392)
        }
        let agreement = DepthAnythingProcessor.sparseLiDARAgreement(
            calibration: truth, relative: relative,
            lidarDepthMap: sparse, lidarConfidenceMap: nil
        )
        #expect(agreement == nil)
    }

    private enum LidarBufferError: Error {
        case creationFailed
    }

    private func lidarBuffer(width: Int, height: Int,
                             depthAt: (Float, Float) -> Float) throws -> CVPixelBuffer {
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
                base[y * stride + x] = depthAt(Float(x) / Float(width), Float(y) / Float(height))
            }
        }
        return buffer
    }
}
