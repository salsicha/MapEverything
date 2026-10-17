import CoreVideo
import Foundation
import Testing
import simd
@testable import MapEverything

struct DA3CalibrationTests {
    @Test("Moderate DA3 shape error does not require an offset or discard the frame")
    func acceptsModerateShapeError() throws {
        let samples = (0..<300).map { i in
            let depth = 1 + Double(i) / 100
            let error = 1 + Double(i % 5 - 2) * 0.05
            return DepthCalibrationSample(relative: 3 / (depth * error), depth: depth, weight: 1)
        }
        let fit = try #require(RobustDepthCalibration.fitScaleOnly(samples))
        #expect(abs(fit.scale - 1 / 3) < 0.001)
        #expect(fit.offset == 0)
    }
    @Test("DA3 scale calibration works on a flat wall and rejects a minority of bad returns")
    func flatWallWithOutliers() throws {
        let samples = (0..<200).map { i in
            DepthCalibrationSample(relative: 1.5, depth: i % 5 == 0 ? 4.8 : 2,
                                   weight: i % 5 == 0 ? 100 : 1)
        }
        let fit = try #require(RobustDepthCalibration.fitScaleOnly(samples))
        #expect(abs(fit.scale - 1 / 3) < 0.00001)
        #expect(fit.offset == 0)
        let distant = try #require(DepthAnythingProcessor.calibratedMetricDepth(relativeDepth: 3 / 30, calibration: fit))
        #expect(abs(distant - 30) < 0.001)
    }

    @Test("DA3 refuses absent or incompatible scale references")
    func rejectsUnconstrainedScale() {
        #expect(RobustDepthCalibration.fitScaleOnly([]) == nil)
        let samples = (0..<300).map { i in
            DepthCalibrationSample(relative: 0.2 + Double(i) / 300,
                                   depth: 0.5 + Double((i * 137) % 293) / 70, weight: 1)
        }
        #expect(RobustDepthCalibration.fitScaleOnly(samples) == nil)
    }

    @Test("Partial near LiDAR scales the entire DA3 image without a LiDAR-range crop")
    func partialLiDARPreservesDistantSurface() throws {
        let width = 518, height = 392
        let relative = RelativeDepthMap(width: width, height: height, data: (0..<(width * height)).map {
            $0 % width < width / 2 ? 1.5 : 0.25 // DA3 depth * 3 = 2m or 12m.
        })
        var buffer: CVPixelBuffer?
        try #require(CVPixelBufferCreate(kCFAllocatorDefault, 64, 48, kCVPixelFormatType_DepthFloat32,
                                        nil, &buffer) == kCVReturnSuccess)
        let lidar = try #require(buffer)
        CVPixelBufferLockBaseAddress(lidar, [])
        let base = try #require(CVPixelBufferGetBaseAddress(lidar)).assumingMemoryBound(to: Float.self)
        let stride = CVPixelBufferGetBytesPerRow(lidar) / MemoryLayout<Float>.stride
        for y in 0..<48 {
            for x in 0..<64 { base[y * stride + x] = x < 32 ? 2 : .nan }
        }
        CVPixelBufferUnlockBaseAddress(lidar, [])
        let result = try #require(DepthCalibrationPipeline.calibrate(
            relative: relative, lidarDepthMap: lidar, lidarConfidenceMap: nil, anchors: [],
            intrinsics: matrix_identity_float3x3, imageResolution: .init(width: 1920, height: 1440),
            transform: matrix_identity_float4x4, configuration: .depthAnything3
        ))
        #expect(result.source == .lidar)
        #expect(result.calibration.offset == 0)
        #expect(result.integrationDepthCap > 12)
        let depth = try #require(DepthAnythingProcessor.calibratedMetricDepth(relativeDepth: 0.25, calibration: result.calibration))
        #expect(abs(depth - 12) < 0.001)
    }

    @Test("Recorded calibration identifies DA3 and its reciprocal scale-only contract")
    func modelMetadata() throws {
        let message = ROS2BridgeClient.makeDepthAnythingCalibrationMessage(
            calibration: .init(scale: 0.5, offset: 0), header: [:],
            relativeDepthSize: .init(width: 518, height: 392), imageResolution: .init(width: 1920, height: 1440),
            relativePointCloudTopic: "/test", frameID: "camera"
        )
        #expect(message["source"] as? String == "depth_anything_v3_lidar_calibrated")
        let encoded = try #require(message["metadata_json"] as? String)
        let metadata = try #require(JSONSerialization.jsonObject(with: Data(encoded.utf8)) as? [String: Any])
        #expect(metadata["calibration_fit_constraint"] as? String == "positive_scale_zero_offset")
        #expect(metadata["relative_depth_representation"] as? String == "reciprocal_da3_depth")
    }
}
