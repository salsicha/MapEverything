import Foundation
import Testing
import simd
import CoreVideo
@testable import MapEverything

private final class CalibrationFixtureBundle: NSObject {}

struct CalibratedSceneDepthTests {
    @Test("LiDAR-calibrated depth retains surfaces at 10, 30 and 50 metres")
    func distantMeshUsesCalibratedDepth() async throws {
        // Only the calibration references are in LiDAR range. The mesh itself
        // is exclusively distant DA pixels, with a known camera translation.
        let samples = (0..<400).map { index in
            let depth = 1 + Double(index) / 399 * 3.8
            return DepthCalibrationSample(relative: 1 / depth, depth: depth, weight: 1)
        }
        let fit = try #require(RobustDepthCalibration.fit(samples))
        #expect(abs(fit.scale - 1) < 0.001)
        #expect(abs(fit.offset) < 0.001)
        let intrinsics = simd_float3x3(columns: (.init(400, 0, 0), .init(0, 400, 0), .init(1, 1, 1)))
        var transform = matrix_identity_float4x4
        transform.columns.3 = .init(2, 0, 0, 1)
        for depth: Float in [10, 30, 50] {
            let relative = RelativeDepthMap(width: 3, height: 3, data: Array(repeating: 1 / depth, count: 9))
            let mesh = try #require(MeshGenerator.createDepthAnythingMeshSnapshot(
                from: relative, calibration: fit, intrinsics: intrinsics,
                imageResolution: .init(width: 3, height: 3), transform: transform
            ))
            #expect(mesh.indices.count == 24)
            #expect(mesh.vertices.allSatisfy { abs($0.z + depth) < 0.001 })
            let colored = MeshGenerator.DepthAnythingMeshSnapshot(
                descriptor: mesh.descriptor, vertices: mesh.vertices, indices: mesh.indices,
                colors: Array(repeating: .init(180, 100, 40), count: mesh.vertices.count)
            )
            let accumulator = AccumulatedDepthMesh(voxelSize: 0.01)
            _ = await accumulator.integrate(colored)
            let fullScene = await accumulator.snapshot()
            #expect(!fullScene.isEmpty)
            #expect(fullScene.vertices.allSatisfy { abs($0.z + depth) < 0.001 })
            #expect(fullScene.colors.allSatisfy { $0 == .init(180, 100, 40) })
        }
    }

    @Test("Calibration pairs use the same camera rays as depth unprojection")
    func matchesLiDARAndPredictionPixels() throws {
        let width = 518, height = 392
        let relative = RelativeDepthMap(width: width, height: height, data: (0..<(width * height)).map {
            0.3 + Float($0 % width) * 0.001 + Float($0 / width) * 0.002
        })
        let lidarWidth = 64, lidarHeight = 48
        var buffer: CVPixelBuffer?
        let status = CVPixelBufferCreate(kCFAllocatorDefault, lidarWidth, lidarHeight,
                                        kCVPixelFormatType_DepthFloat32, nil, &buffer)
        #expect(status == kCVReturnSuccess)
        let lidar = try #require(buffer)
        CVPixelBufferLockBaseAddress(lidar, [])
        let base = try #require(CVPixelBufferGetBaseAddress(lidar)).assumingMemoryBound(to: Float.self)
        let stride = CVPixelBufferGetBytesPerRow(lidar) / MemoryLayout<Float>.stride
        for y in 0..<lidarHeight {
            for x in 0..<lidarWidth {
                let rx = Int((Float(x) * Float(width) / Float(lidarWidth)).rounded())
                let ry = Int((Float(y) * Float(height) / Float(lidarHeight)).rounded())
                let r = relative.value(atX: rx, y: ry)
                base[y * stride + x] = 1 / (0.6 * r + 0.05)
            }
        }
        CVPixelBufferUnlockBaseAddress(lidar, [])
        let fit = try #require(DepthAnythingProcessor.maximumLikelihoodCalibration(relative: relative, lidarDepthMap: lidar))
        #expect(abs(fit.scale - 0.6) < 0.00001)
        #expect(abs(fit.offset - 0.05) < 0.00001)
    }

    @Test("Outdoor LiDAR pairs calibrate the full DA output without an inverse-depth pole")
    func recordedCalibrationPairs() throws {
        struct Fixture: Decodable {
            let frame: Int
            let samples: [[Double]]
            let distantRelative: Float
        }
        // Sanitized paired depth values from the 22:07 scan; no images or
        // locations. Confidence maps were not recorded, so replay uses the
        // depth-dependent measurement weights with confidence assumed to be 1.
        let url = try #require(Bundle(for: CalibrationFixtureBundle.self).url(forResource: "OutdoorCalibrationPairs", withExtension: "json"))
        let cases = try JSONDecoder().decode([Fixture].self, from: Data(contentsOf: url))
        for fixture in cases {
            let samples = fixture.samples.map { DepthCalibrationSample(relative: $0[0], depth: $0[1], weight: $0[2]) }
            let fit = try #require(RobustDepthCalibration.fit(samples), "Frame \(fixture.frame)")
            #expect(fit.scale > 0 && fit.offset >= 0)
            if fixture.frame >= 6 {
                let distant = try #require(DepthAnythingProcessor.calibratedMetricDepth(relativeDepth: fixture.distantRelative, calibration: fit))
                #expect(distant > 5 && distant < 40, "Frame \(fixture.frame): \(distant)m")
            }
        }
    }

    @Test("An incompatible calibration is rejected instead of cropping its far pixels")
    func rejectsIncompatibleModel() {
        // A strong positive relative-depth bias needs a negative offset. This
        // constrained model must not silently accept a poor replacement fit.
        let samples = (0..<400).map { index in
            let depth = 1 + Double(index) / 399 * 3.8
            return DepthCalibrationSample(relative: 1 / depth + 3, depth: depth, weight: 1)
        }
        #expect(RobustDepthCalibration.fit(samples) == nil)
    }

    @Test("Calibration metadata explicitly records that LiDAR does not limit output range")
    func recordsFullDepthPolicy() throws {
        let message = ROS2BridgeClient.makeDepthAnythingCalibrationMessage(
            calibration: .init(scale: 0.7, offset: 0), header: [:],
            relativeDepthSize: .init(width: 518, height: 392), imageResolution: .init(width: 1920, height: 1440),
            relativePointCloudTopic: "/test", frameID: "camera"
        )
        let json = try #require(message["metadata_json"] as? String)
        let metadata = try #require(JSONSerialization.jsonObject(with: Data(json.utf8)) as? [String: Any])
        #expect(metadata["depth_validity_filter"] as? String == "finite_metric_depth_only")
        #expect(metadata["lidar_limits_output_range"] as? Bool == false)
        #expect(metadata["inverse_error_variance"] == nil)
    }
}
