import CoreVideo
import Foundation

/// Scene-brightness measurement for depth-mapping feedback. Depth Anything
/// is a passive RGB model: in a near-black night scene it cannot extend
/// depth beyond LiDAR range no matter how well calibration works (measured
/// on a recorded night scan: frames with median luma 3-7/255 produced zero
/// depth beyond 8 m, while the same scan's lit frames at luma 34-56 reached
/// 13-20 m). Surfacing that honestly beats the generic calibration guidance.
nonisolated enum SceneLuminance {

    /// Median luma of a camera pixel buffer on the full-range 0-255 scale,
    /// or nil when the buffer's layout is not one we can read. Biplanar
    /// YCbCr (ARKit's capturedImage) reads the Y plane directly; BGRA falls
    /// back to a Rec. 709 mix. Sampling is strided: exactness is not the
    /// point, a stable scene-level median is.
    static func medianLuma(of pixelBuffer: CVPixelBuffer, sampleStride: Int = 16) -> Float? {
        let stride = max(1, sampleStride)
        guard CVPixelBufferLockBaseAddress(pixelBuffer, .readOnly) == kCVReturnSuccess else { return nil }
        defer { CVPixelBufferUnlockBaseAddress(pixelBuffer, .readOnly) }

        var histogram = [Int](repeating: 0, count: 256)
        var total = 0
        let format = CVPixelBufferGetPixelFormatType(pixelBuffer)

        if CVPixelBufferGetPlaneCount(pixelBuffer) >= 2 {
            guard let base = CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 0) else { return nil }
            let width = CVPixelBufferGetWidthOfPlane(pixelBuffer, 0)
            let height = CVPixelBufferGetHeightOfPlane(pixelBuffer, 0)
            let rowBytes = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 0)
            let pointer = base.assumingMemoryBound(to: UInt8.self)
            for y in Swift.stride(from: 0, to: height, by: stride) {
                let row = pointer + y * rowBytes
                for x in Swift.stride(from: 0, to: width, by: stride) {
                    histogram[Int(row[x])] += 1
                    total += 1
                }
            }
            guard total > 0 else { return nil }
            var median = medianBin(histogram: histogram, total: total)
            if format == kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange {
                // Video-range black sits at 16; report on the full-range scale.
                median = max(0, (median - 16) * 255 / 219)
            }
            return median
        }

        if format == kCVPixelFormatType_32BGRA,
           let base = CVPixelBufferGetBaseAddress(pixelBuffer) {
            let width = CVPixelBufferGetWidth(pixelBuffer)
            let height = CVPixelBufferGetHeight(pixelBuffer)
            let rowBytes = CVPixelBufferGetBytesPerRow(pixelBuffer)
            let pointer = base.assumingMemoryBound(to: UInt8.self)
            for y in Swift.stride(from: 0, to: height, by: stride) {
                let row = pointer + y * rowBytes
                for x in Swift.stride(from: 0, to: width, by: stride) {
                    let pixel = row + x * 4
                    let luma = 0.0722 * Float(pixel[0]) + 0.7152 * Float(pixel[1]) + 0.2126 * Float(pixel[2])
                    histogram[min(255, Int(luma))] += 1
                    total += 1
                }
            }
            guard total > 0 else { return nil }
            return medianBin(histogram: histogram, total: total)
        }
        return nil
    }

    private static func medianBin(histogram: [Int], total: Int) -> Float {
        var seen = 0
        for (bin, count) in histogram.enumerated() {
            seen += count
            if seen * 2 >= total { return Float(bin) }
        }
        return 255
    }

    /// Hysteresis gate deciding when the scene is dark enough to blame for
    /// missing far-field depth. Thresholds sit between the measured night
    /// regimes (dark stretch median 3-7, workable streetlight 34-56):
    /// enter below 10, exit at 14, each after two consecutive frames so a
    /// single headlight sweep or dark doorway does not flap the message.
    struct DarknessGate {
        static let enterBelow: Float = 10
        static let exitAtOrAbove: Float = 14
        private static let framesToSwitch = 2

        private(set) var isDark = false
        private var streak = 0

        mutating func update(medianLuma: Float?) -> Bool {
            guard let medianLuma else { return isDark }
            if isDark {
                streak = medianLuma >= Self.exitAtOrAbove ? streak + 1 : 0
                if streak >= Self.framesToSwitch {
                    isDark = false
                    streak = 0
                }
            } else {
                streak = medianLuma < Self.enterBelow ? streak + 1 : 0
                if streak >= Self.framesToSwitch {
                    isDark = true
                    streak = 0
                }
            }
            return isDark
        }
    }
}
