import CoreVideo
import Foundation
import Testing
import simd
@testable import MapEverything

private final class DA3WindowFixtureBundle: NSObject {}

struct DA3WindowTests {
    private struct Pixel: Decodable { let x: Int; let y: Int; let inverseDepth: Float }
    private struct Fixture: Decodable {
        let transforms: [[[Float]]]
        let poseEncoding: [Float]
        let pixels: [Pixel]
    }
    private func fixture() throws -> Fixture {
        let url = try #require(Bundle(for: DA3WindowFixtureBundle.self)
            .url(forResource: "DA3WindowSyntheticDepth", withExtension: "json"))
        return try JSONDecoder().decode(Fixture.self, from: Data(contentsOf: url))
    }
    private func camera(_ index: Int, _ fixture: Fixture) -> DA3WindowGeometry.Camera {
        let rows = fixture.transforms[index]
        let columns = (0..<4).map { c in SIMD4<Float>(rows[0][c], rows[1][c], rows[2][c], rows[3][c]) }
        return .init(timestamp: Double(index + 1),
                     transform: simd_float4x4(columns),
                     intrinsics: simd_float3x3(columns: (SIMD3(500, 0, 0), SIMD3(0, 500, 0), SIMD3(259, 196, 1))),
                     resolution: CGSize(width: 518, height: 392))
    }
    private func image(_ index: Int) throws -> CVPixelBuffer {
        var buffer: CVPixelBuffer?
        try #require(CVPixelBufferCreate(kCFAllocatorDefault, 518, 392, kCVPixelFormatType_32BGRA,
                     [kCVPixelBufferIOSurfacePropertiesKey: [:]] as CFDictionary, &buffer) == kCVReturnSuccess)
        let image = try #require(buffer)
        CVPixelBufferLockBaseAddress(image, [])
        defer { CVPixelBufferUnlockBaseAddress(image, []) }
        let base = try #require(CVPixelBufferGetBaseAddress(image)).assumingMemoryBound(to: UInt8.self)
        let stride = CVPixelBufferGetBytesPerRow(image)
        for y in 0..<392 {
            for x in 0..<518 {
                let offset = y * stride + x * 4
                base[offset] = UInt8((x + y + index * 13) % 256)
                base[offset + 1] = UInt8((y * 255 / 391 + index * 7) % 256)
                base[offset + 2] = UInt8((x * 255 / 517 + index * 11) % 256)
                base[offset + 3] = 255
            }
        }
        return image
    }

    @Test("Camera encoding matches official DA3 normalization, rotation, and XYZW quaternion")
    func poseConvention() throws {
        let f = try fixture()
        let actual = try #require(DA3WindowGeometry.poseEncoding((0..<3).map { camera($0, f) }))
        #expect(actual.count == 27)
        for (a, b) in zip(actual, f.poseEncoding) { #expect(abs(a - b) < 0.00001) }
    }

    @Test("Window keyframes reject stale, future, nonoverlapping and stationary views")
    func keyframeGates() throws {
        let f = try fixture(), a = camera(0, f), b = camera(1, f)
        func changed(time: Double = 2, x: Float = 0, angle: Float = 0) -> DA3WindowGeometry.Camera {
            var t = simd_float4x4(simd_quatf(angle: angle, axis: SIMD3(0, 1, 0)))
            t.columns.3.x = x
            return .init(timestamp: time, transform: t, intrinsics: a.intrinsics, resolution: a.resolution)
        }
        #expect(DA3WindowGeometry.isCompatible(a, with: b))
        #expect(DA3WindowGeometry.isKeyframe(b, after: a))
        #expect(!DA3WindowGeometry.isCompatible(a, with: a))
        #expect(!DA3WindowGeometry.isCompatible(b, with: a))
        #expect(!DA3WindowGeometry.isCompatible(a, with: changed(time: 7)))
        #expect(!DA3WindowGeometry.isCompatible(a, with: changed(x: 2)))
        #expect(!DA3WindowGeometry.isCompatible(a, with: changed(angle: .pi / 2)))
        #expect(!DA3WindowGeometry.isKeyframe(changed(x: 0.01), after: a))
        #expect(DA3WindowGeometry.isKeyframe(changed(angle: 0.1), after: a))
    }

    @Test("Multi-view is opt-in for a fresh installation")
    func defaultsOff() throws {
        let suite = "DA3WindowTests.\(UUID().uuidString)"
        let defaults = try #require(UserDefaults(suiteName: suite))
        defer { defaults.removePersistentDomain(forName: suite) }
        #expect(!defaults.bool(forKey: DA3WindowGeometry.enabledStorageKey))
    }

    @Test("Bundled window predicts only newest depth, preserves image orientation, and isolates scans")
    func modelAndLifecycle() throws {
        let f = try fixture(), processor = DA3WindowProcessor(), session = ScanWorkSession()
        let first = processor.inferNewest(image: try image(0), camera: camera(0, f), workSession: session)
        #expect(first.depth == nil && !first.failed)
        let second = processor.inferNewest(image: try image(1), camera: camera(1, f), workSession: session)
        #expect(second.depth == nil && !second.failed)
        let third = processor.inferNewest(image: try image(2), camera: camera(2, f), workSession: session)
        #expect(!third.failed)
        #expect(third.timestamps == [1, 2, 3])
        let depth = try #require(third.depth)
        #expect(depth.width == 518 && depth.height == 392)
        // Independent official PyTorch prediction, not a Core ML self-reference.
        for p in f.pixels {
            let actual = depth.value(atX: p.x, y: p.y)
            #expect(actual.isFinite && actual > 0)
            #expect(abs(actual - p.inverseDepth) / p.inverseDepth < 0.05)
        }
        let duplicate = processor.inferNewest(image: try image(2), camera: camera(2, f), workSession: session)
        #expect(duplicate.depth == nil)
        session.cancel()
        let cancelled = processor.inferNewest(image: try image(2), camera: camera(2, f), workSession: session)
        #expect(cancelled.depth == nil)
        let nextScan = DA3WindowProcessor().inferNewest(image: try image(2), camera: camera(2, f), workSession: ScanWorkSession())
        #expect(nextScan.depth == nil && !nextScan.failed)
    }

    @Test("Recording metadata distinguishes actual window output from single-frame fallback")
    func recordedMode() throws {
        for times in [[1.0], [1.0, 2.0, 3.0]] {
            let message = ROS2BridgeClient.makeDepthAnythingCalibrationMessage(
                calibration: .init(scale: 2, offset: 0), depthInputTimestamps: times,
                header: [:], relativeDepthSize: CGSize(width: 518, height: 392),
                imageResolution: CGSize(width: 1920, height: 1440),
                relativePointCloudTopic: "/mapping/pointcloud/depth_anything", frameID: "camera")
            let json = try #require(message["metadata_json"] as? String)
            let metadata = try #require(JSONSerialization.jsonObject(with: Data(json.utf8)) as? [String: Any])
            #expect(metadata["inference_mode"] as? String == (times.count == 3 ? "causal_window_3" : "single_frame"))
            #expect(metadata["input_frame_timestamps_uptime_s"] as? [Double] == times)
            #expect(metadata["pose_conditioned"] as? Bool == (times.count == 3))
        }
    }
}
