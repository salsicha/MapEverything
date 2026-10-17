// Compile with the app's DA3WindowGeometry.swift and RobustDepthCalibration.swift.
// This shell type supplies only the existing result contract; the fit and camera
// math run directly from production source, not a reimplementation.
import Foundation
import simd

nonisolated enum DepthAnythingProcessor {
    struct MaximumLikelihoodCalibration { let scale: Float; let offset: Float }
}

@main enum ValidateWindowReplay {
    struct Frame: Decodable {
        let index: Int
        let session: Int
        let timestamp: Double
        let transform: [[Float]]
        let intrinsics: [[Float]]
        let inputIndices: [Int]
        let poseEncoding: [Float]?
        let samples: [[Double]] // inverse relative depth, metric LiDAR depth
        var camera: DA3WindowGeometry.Camera {
            let columns = (0..<4).map { c in SIMD4<Float>(transform[0][c], transform[1][c], transform[2][c], transform[3][c]) }
            let k = (0..<3).map { c in SIMD3<Float>(intrinsics[0][c], intrinsics[1][c], intrinsics[2][c]) }
            return .init(timestamp: timestamp, transform: simd_float4x4(columns),
                         intrinsics: simd_float3x3(k), resolution: CGSize(width: 518, height: 392))
        }
    }
    static func main() throws {
        let frames = try JSONDecoder().decode([Frame].self, from: Data(contentsOf: URL(fileURLWithPath: CommandLine.arguments[1])))
        var history: [Frame] = [], session = -1, fits = 0, lidarFrames = 0, windows = 0
        for f in frames {
            if f.session != session { history = []; session = f.session }
            history.removeAll { !DA3WindowGeometry.isCompatible($0.camera, with: f.camera) }
            let keyframe = history.last.map { DA3WindowGeometry.isKeyframe(f.camera, after: $0.camera) } ?? true
            var batch = keyframe ? history + [f] : []
            if keyframe { history = Array(batch.suffix(2)) }
            if batch.count != 3 { batch = [f] }
            precondition(batch.map(\.index) == f.inputIndices, "Swift/Python keyframe mismatch at \(f.index)")
            if let expected = f.poseEncoding {
                let actual = DA3WindowGeometry.poseEncoding(batch.map(\.camera))!
                precondition(actual.count == expected.count)
                for (a, b) in zip(actual, expected) {
                    precondition(abs(a - b) < 0.0001, "Camera encoding mismatch at \(f.index): \(a) != \(b)")
                }
                windows += 1
            }
            let samples = f.samples.map { DepthCalibrationSample(relative: $0[0], depth: $0[1], weight: 1) }
            if samples.count > 50 {
                lidarFrames += 1
                if let fit = RobustDepthCalibration.fitScaleOnly(samples) {
                    precondition(fit.scale > 0 && fit.offset == 0)
                    fits += 1
                } else {
                    print("Calibration rejected frame \(f.index)")
                }
            }
        }
        print("Validated \(frames.count) causal frame selections, \(windows) official pose encodings; accepted \(fits)/\(lidarFrames) LiDAR scale fits.")
        precondition(windows > 0 && fits == lidarFrames, "Unexpected calibration rejection")
    }
}
