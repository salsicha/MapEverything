import CoreVideo
import Testing
@testable import MapEverything

/// The dark-scene gate exists because a recorded night scan showed Depth
/// Anything collapsing to near-field-only output at median luma 3-7/255
/// while lit frames at 34-56 reached 13-20 m. These tests pin the median
/// measurement on the buffer layouts the app actually sees and the gate's
/// hysteresis around those measured regimes.
struct SceneLuminanceTests {

    private func makeBiplanarBuffer(
        width: Int, height: Int, format: OSType, luma: (Int, Int) -> UInt8
    ) throws -> CVPixelBuffer {
        var buffer: CVPixelBuffer?
        let status = CVPixelBufferCreate(nil, width, height, format, nil, &buffer)
        let pixelBuffer = try #require(status == kCVReturnSuccess ? buffer : nil)
        CVPixelBufferLockBaseAddress(pixelBuffer, [])
        defer { CVPixelBufferUnlockBaseAddress(pixelBuffer, []) }
        let base = try #require(CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 0))
        let rowBytes = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 0)
        let pointer = base.assumingMemoryBound(to: UInt8.self)
        for y in 0..<height {
            for x in 0..<width {
                pointer[y * rowBytes + x] = luma(x, y)
            }
        }
        return pixelBuffer
    }

    @Test("Full-range biplanar median reads the Y plane")
    func fullRangeMedian() throws {
        // Left half dark (5), right half lit (60): the median must land on
        // one of the two populations, not between them.
        let buffer = try makeBiplanarBuffer(
            width: 64, height: 48, format: kCVPixelFormatType_420YpCbCr8BiPlanarFullRange
        ) { x, _ in x < 32 ? 5 : 60 }
        let median = try #require(SceneLuminance.medianLuma(of: buffer, sampleStride: 1))
        #expect(median == 5 || median == 60)

        let dark = try makeBiplanarBuffer(
            width: 64, height: 48, format: kCVPixelFormatType_420YpCbCr8BiPlanarFullRange
        ) { _, _ in 4 }
        #expect(try #require(SceneLuminance.medianLuma(of: dark)) == 4)
    }

    @Test("Video-range black normalizes to the full-range scale")
    func videoRangeNormalization() throws {
        // Video-range black is Y=16; it must not read as luma 16 (which
        // would sit above the darkness threshold and mask a black scene).
        let black = try makeBiplanarBuffer(
            width: 64, height: 48, format: kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange
        ) { _, _ in 16 }
        #expect(try #require(SceneLuminance.medianLuma(of: black)) == 0)

        let mid = try makeBiplanarBuffer(
            width: 64, height: 48, format: kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange
        ) { _, _ in 126 }  // (126-16)*255/219 ~ 128
        let median = try #require(SceneLuminance.medianLuma(of: mid))
        #expect(abs(median - 128) < 2)
    }

    @Test("BGRA fallback mixes channels instead of failing")
    func bgraFallback() throws {
        var buffer: CVPixelBuffer?
        let status = CVPixelBufferCreate(nil, 32, 32, kCVPixelFormatType_32BGRA, nil, &buffer)
        let pixelBuffer = try #require(status == kCVReturnSuccess ? buffer : nil)
        CVPixelBufferLockBaseAddress(pixelBuffer, [])
        let base = try #require(CVPixelBufferGetBaseAddress(pixelBuffer))
        let rowBytes = CVPixelBufferGetBytesPerRow(pixelBuffer)
        let pointer = base.assumingMemoryBound(to: UInt8.self)
        for y in 0..<32 {
            for x in 0..<32 {
                let pixel = y * rowBytes + x * 4
                pointer[pixel] = 50; pointer[pixel + 1] = 50; pointer[pixel + 2] = 50
                pointer[pixel + 3] = 255
            }
        }
        CVPixelBufferUnlockBaseAddress(pixelBuffer, [])
        let median = try #require(SceneLuminance.medianLuma(of: pixelBuffer, sampleStride: 1))
        #expect(abs(median - 50) < 2)
    }

    @Test("The darkness gate needs two consecutive dark frames to enter")
    func gateEntersAfterTwoDarkFrames() {
        var gate = SceneLuminance.DarknessGate()
        #expect(gate.update(medianLuma: 4) == false)
        #expect(gate.update(medianLuma: 4) == true)
        // A lone dark frame between lit ones never flips the message on.
        var flicker = SceneLuminance.DarknessGate()
        #expect(flicker.update(medianLuma: 4) == false)
        #expect(flicker.update(medianLuma: 40) == false)
        #expect(flicker.update(medianLuma: 4) == false)
    }

    @Test("The gate exits only after two consecutive bright frames")
    func gateExitsWithHysteresis() {
        var gate = SceneLuminance.DarknessGate()
        _ = gate.update(medianLuma: 3)
        #expect(gate.update(medianLuma: 3) == true)
        // The 10-14 dead zone keeps the current state in both directions.
        #expect(gate.update(medianLuma: 12) == true)
        #expect(gate.update(medianLuma: 12) == true)
        // One bright frame (headlight sweep) is not enough...
        #expect(gate.update(medianLuma: 40) == true)
        #expect(gate.update(medianLuma: 3) == true)
        // ...two consecutive are.
        #expect(gate.update(medianLuma: 40) == true)
        #expect(gate.update(medianLuma: 40) == false)
    }

    @Test("Unreadable frames leave the gate unchanged")
    func nilLumaKeepsState() {
        var gate = SceneLuminance.DarknessGate()
        _ = gate.update(medianLuma: 3)
        _ = gate.update(medianLuma: 3)
        #expect(gate.update(medianLuma: nil) == true)
        var bright = SceneLuminance.DarknessGate()
        #expect(bright.update(medianLuma: nil) == false)
    }
}
