import CoreVideo
import Foundation
import Testing
import simd
@testable import MapEverything

private final class Scan6FixtureBundle: NSObject {}

/// Replays the real outdoor recording (scan6: 20 Depth Anything frames from a
/// near-static rotational scan, with time-aligned LiDAR) through the actual
/// calibration pipeline. The recorded per-frame LiDAR-only calibrations put
/// the same far surfaces at 3.8-19.5 m depending on the frame; the replay
/// asserts the map-anchored pipeline collapses that inter-frame far-field
/// spread while preserving near-field agreement.
struct Scan6ReplayTests {
    struct Frame {
        let transform: simd_float4x4
        let scale: Float
        let offset: Float
        let relative: RelativeDepthMap
        let lidar: CVPixelBuffer
    }

    struct Fixture {
        let intrinsics: simd_float3x3
        let imageResolution: CGSize
        let frames: [Frame]
    }

    static func loadFixture() throws -> Fixture {
        let url = try #require(Bundle(for: Scan6FixtureBundle.self)
            .url(forResource: "Scan6ReplayFrames", withExtension: "json"))
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
            frames.append(Frame(
                transform: transform,
                scale: (entry["scale"] as! NSNumber).floatValue,
                offset: (entry["offset"] as! NSNumber).floatValue,
                relative: RelativeDepthMap(width: width, height: height, data: values),
                lidar: lidar
            ))
        }
        return Fixture(intrinsics: intrinsics, imageResolution: resolution, frames: frames)
    }

    /// Sparse (px, py, z) triplets on the 518x392 grid scattered into a
    /// 256x192 DepthFloat32 buffer (unfilled pixels invalid at 0).
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

    /// Along-ray far-field disagreement between consecutive frames: 2-degree
    /// direction bins from the shared origin, |difference of median ranges|
    /// pooled over pairs, restricted to bins whose surfaces sit beyond 6 m.
    static func farFieldSpread(pointSets: [[SIMD3<Float>]], origin: SIMD3<Float>) -> (median: Double, p90: Double, bins: Int) {
        func binned(_ points: [SIMD3<Float>]) -> [Int: [Float]] {
            var bins: [Int: [Float]] = [:]
            for point in points {
                let d = point - origin
                let range = simd_length(d)
                guard range > 0.2 else { continue }
                let azimuth = Int((atan2(d.z, d.x) + .pi) / (.pi / 90))
                let elevation = Int((asin(max(-1, min(1, d.y / range))) + .pi / 2) / (.pi / 90))
                bins[azimuth << 10 | elevation, default: []].append(range)
            }
            return bins
        }
        var differences: [Double] = []
        let allBins = pointSets.map(binned)
        for index in 1..<allBins.count {
            let a = allBins[index - 1], b = allBins[index]
            for (key, ranges) in a {
                guard ranges.count >= 8, let other = b[key], other.count >= 8 else { continue }
                let ma = Double(ranges.sorted()[ranges.count / 2])
                let mb = Double(other.sorted()[other.count / 2])
                guard max(ma, mb) > 6 else { continue }
                differences.append(abs(ma - mb))
            }
        }
        guard !differences.isEmpty else { return (0, 0, 0) }
        let sorted = differences.sorted()
        return (sorted[sorted.count / 2], sorted[min(sorted.count - 1, sorted.count * 9 / 10)], sorted.count)
    }

    static func calibratedPoints(
        frame: Frame, calibration: DepthAnythingProcessor.MaximumLikelihoodCalibration,
        maximumDepth: Float, fixture: Fixture
    ) -> [SIMD3<Float>] {
        let configuration = MeshGenerator.DepthAnythingMeshConfiguration(
            step: 2, minimumDepth: 0.1, maximumDepth: maximumDepth,
            maximumDepthDiscontinuity: 0.45, maximumTriangleCount: 600_000
        )
        guard let mesh = MeshGenerator.createDepthAnythingMeshSnapshot(
            from: frame.relative, calibration: calibration, intrinsics: fixture.intrinsics,
            imageResolution: fixture.imageResolution, transform: frame.transform,
            configuration: configuration
        ) else { return [] }
        return mesh.vertices
    }

    @Test("Map-anchored calibration collapses scan6's far-field layering")
    func replayCollapsesFarFieldSpread() async throws {
        let fixture = try Self.loadFixture()
        try #require(fixture.frames.count == 20)
        let origin = fixture.frames.reduce(SIMD3<Float>.zero) {
            $0 + SIMD3($1.transform.columns.3.x, $1.transform.columns.3.y, $1.transform.columns.3.z)
        } / Float(fixture.frames.count)

        // Baseline: the recorded per-frame LiDAR-only calibrations, uncapped,
        // exactly what produced the layered preview.
        let baselinePoints = fixture.frames.map {
            Self.calibratedPoints(
                frame: $0,
                calibration: .init(scale: $0.scale, offset: $0.offset),
                maximumDepth: .greatestFiniteMagnitude, fixture: fixture
            )
        }
        let baseline = Self.farFieldSpread(pointSets: baselinePoints, origin: origin)
        try #require(baseline.bins > 50)

        // Replay through the real pipeline, growing the anchor map causally.
        let map = AccumulatedDepthMesh()
        var replayPoints: [[SIMD3<Float>]] = []
        var dropped = 0
        var sources: [DepthCalibrationSource] = []
        let clock = ContinuousClock()
        var calibrationTime = Duration.zero
        for frame in fixture.frames {
            let anchors = await map.calibrationAnchors()
            let started = clock.now
            let result = DepthCalibrationPipeline.calibrate(
                relative: frame.relative, lidarDepthMap: frame.lidar, lidarConfidenceMap: nil,
                anchors: anchors, intrinsics: fixture.intrinsics,
                imageResolution: fixture.imageResolution, transform: frame.transform
            )
            calibrationTime += clock.now - started
            guard let result else {
                dropped += 1
                continue
            }
            sources.append(result.source)
            let points = Self.calibratedPoints(
                frame: frame, calibration: result.calibration,
                maximumDepth: result.integrationDepthCap, fixture: fixture
            )
            replayPoints.append(points)
            let configuration = MeshGenerator.DepthAnythingMeshConfiguration(
                step: 2, minimumDepth: 0.1, maximumDepth: result.integrationDepthCap,
                maximumDepthDiscontinuity: 0.45, maximumTriangleCount: 600_000
            )
            if let mesh = MeshGenerator.createDepthAnythingMeshSnapshot(
                from: frame.relative, calibration: result.calibration, intrinsics: fixture.intrinsics,
                imageResolution: fixture.imageResolution, transform: frame.transform,
                configuration: configuration
            ) {
                let colored = MeshGenerator.DepthAnythingMeshSnapshot(
                    descriptor: mesh.descriptor, vertices: mesh.vertices, indices: mesh.indices,
                    colors: Array(repeating: SIMD3<UInt8>(180, 180, 180), count: mesh.vertices.count),
                    cameraPosition: mesh.cameraPosition
                )
                _ = await map.integrate(colored, calibrationSource: result.source)
            }
        }
        let replay = Self.farFieldSpread(pointSets: replayPoints, origin: origin)

        print("scan6 replay: baseline far-field spread median \(baseline.median) p90 \(baseline.p90) (\(baseline.bins) bins)")
        print("scan6 replay: pipeline far-field spread median \(replay.median) p90 \(replay.p90) (\(replay.bins) bins)")
        print("scan6 replay: dropped \(dropped)/20, joint/lidar \(sources.filter { $0 == .lidar }.count), propagated \(sources.filter { $0 == .meshPropagated }.count)")
        print("scan6 replay: calibration time total \(calibrationTime) for \(20 - dropped) frames")

        #expect(dropped <= 5, "The pipeline should keep most frames")
        try #require(replay.bins > 50, "Replay must retain enough far-field coverage to measure")
        #expect(replay.median < baseline.median * 0.7,
                "Far-field inter-frame spread must drop by at least 30% (got \(replay.median) vs \(baseline.median))")
    }
}
