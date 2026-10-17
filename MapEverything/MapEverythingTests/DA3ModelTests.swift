import CoreVideo
import Foundation
import Testing
@testable import MapEverything

private final class DA3FixtureBundle: NSObject {}

struct DA3ModelTests {
    @Test("Bundled DA3 runs through Vision with correct RGB, orientation, and output semantics")
    func nativeCameraInference() throws {
        struct Pixel: Decodable { let x: Int; let y: Int; let inverseDepth: Float }
        struct Fixture: Decodable { let width: Int; let height: Int; let pixels: [Pixel] }
        let url = try #require(Bundle(for: DA3FixtureBundle.self).url(forResource: "DA3SyntheticDepth", withExtension: "json"))
        let reference = try JSONDecoder().decode(Fixture.self, from: Data(contentsOf: url))
        let processor = try #require(DepthAnythingProcessor(), "DA3 must be bundled and loadable")
        var buffer: CVPixelBuffer?
        let attrs = [kCVPixelBufferIOSurfacePropertiesKey: [:]] as CFDictionary
        try #require(CVPixelBufferCreate(kCFAllocatorDefault, reference.width, reference.height,
                                        kCVPixelFormatType_32BGRA, attrs, &buffer) == kCVReturnSuccess)
        let image = try #require(buffer)
        CVPixelBufferLockBaseAddress(image, [])
        let base = try #require(CVPixelBufferGetBaseAddress(image)).assumingMemoryBound(to: UInt8.self)
        let stride = CVPixelBufferGetBytesPerRow(image)
        for y in 0..<reference.height {
            for x in 0..<reference.width {
                let offset = y * stride + 4 * x
                base[offset] = UInt8((x + y) % 256)
                base[offset + 1] = UInt8(y * 255 / (reference.height - 1))
                base[offset + 2] = UInt8(x * 255 / (reference.width - 1))
                base[offset + 3] = 255
            }
        }
        CVPixelBufferUnlockBaseAddress(image, [])
        let depth = try #require(processor.inferRelativeDepth(from: image))
        #expect(depth.width == reference.width)
        #expect(depth.height == reference.height)
        // Independent official PyTorch predictions on this synthetic RGB
        // image. Checks detect flipped/transposed rays, swapped RGB, incorrect
        // normalization, and accidental direct-depth output into inverse APIs.
        for pixel in reference.pixels {
            let actual = depth.value(atX: pixel.x, y: pixel.y)
            #expect(actual.isFinite && actual > 0)
            #expect(abs(actual - pixel.inverseDepth) / pixel.inverseDepth < 0.05)
        }
    }
}
