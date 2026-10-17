import Foundation
import CoreML
import CoreImage
import CoreVideo
import simd

/// Owned by one scan work session. Serializes inference and retains only two
/// resized image copies, never historical ARFrames or camera capture buffers.
nonisolated final class DA3WindowProcessor: @unchecked Sendable {
    struct Result {
        let depth: RelativeDepthMap?
        let timestamps: [TimeInterval]
        let failed: Bool
    }
    private struct Keyframe {
        let camera: DA3WindowGeometry.Camera
        let image: CVPixelBuffer
    }
    private let lock = NSLock()
    private let context = CIContext(options: [.cacheIntermediates: false])
    private var history: [Keyframe] = []
    private var model: MLModel?
    private var failed = false
    private var lastTimestamp: TimeInterval = -.infinity

    func inferNewest(image: CVPixelBuffer, camera: DA3WindowGeometry.Camera,
                     workSession: ScanWorkSession) -> Result {
        lock.lock()
        defer { lock.unlock() }
        let fallback = Result(depth: nil, timestamps: [camera.timestamp], failed: failed)
        guard workSession.isActive, !failed, camera.timestamp > lastTimestamp else { return fallback }
        lastTimestamp = camera.timestamp
        history.removeAll { !DA3WindowGeometry.isCompatible($0.camera, with: camera) }
        // Do not spend multi-view compute on near-identical stationary frames.
        if let last = history.last, !DA3WindowGeometry.isKeyframe(camera, after: last.camera) {
            return fallback
        }
        do {
            let current = Keyframe(camera: camera, image: try resizedCopy(image))
            let window = history + [current]
            history = Array(window.suffix(DA3WindowGeometry.windowSize - 1))
            guard window.count == DA3WindowGeometry.windowSize,
                  let encoding = DA3WindowGeometry.poseEncoding(window.map(\.camera)),
                  workSession.isActive else { return fallback }
            let model = try loadModel()
            guard workSession.isActive else { return fallback }
            let poses = try MLMultiArray(shape: [1, 3, 9], dataType: .float32)
            for (index, value) in encoding.enumerated() { poses[index] = NSNumber(value: value) }
            var inputs: [String: MLFeatureValue] = ["poses": MLFeatureValue(multiArray: poses)]
            for (index, frame) in window.enumerated() {
                inputs["image\(index)"] = MLFeatureValue(pixelBuffer: frame.image)
            }
            let output = try model.prediction(from: MLDictionaryFeatureProvider(dictionary: inputs))
            guard let array = output.featureValue(for: "inverse_depth")?.multiArrayValue,
                  let depth = RelativeDepthMap(fromMultiArray: array, size: DA3WindowGeometry.width),
                  depth.width == DA3WindowGeometry.width, depth.height == DA3WindowGeometry.height else {
                throw WindowError.invalidOutput
            }
            return Result(depth: depth, timestamps: window.map { $0.camera.timestamp }, failed: false)
        } catch {
            // Retry on the next scan, not on every frame. Normal mode remains usable.
            failed = true
            history.removeAll()
            model = nil
            print("DA3 multi-view unavailable; using single-frame depth: \(error)")
            return Result(depth: nil, timestamps: [camera.timestamp], failed: true)
        }
    }

    private enum WindowError: Error { case missingModel, invalidOutput, allocationFailed }

    private func loadModel() throws -> MLModel {
        if let model { return model }
        guard let url = Bundle.main.url(forResource: "DepthAnything3WindowF16", withExtension: "mlmodelc") else {
            throw WindowError.missingModel
        }
        let config = MLModelConfiguration()
        config.computeUnits = .all
        let loaded = try MLModel(contentsOf: url, configuration: config)
        model = loaded
        return loaded
    }

    /// Resize in native sensor coordinates. The graph performs the clockwise
    /// rotation, RGB normalization, and inverse rotation of the output.
    private func resizedCopy(_ source: CVPixelBuffer) throws -> CVPixelBuffer {
        var buffer: CVPixelBuffer?
        let attributes = [kCVPixelBufferIOSurfacePropertiesKey: [:]] as CFDictionary
        guard CVPixelBufferCreate(kCFAllocatorDefault, DA3WindowGeometry.width, DA3WindowGeometry.height,
                                  kCVPixelFormatType_32BGRA, attributes, &buffer) == kCVReturnSuccess,
              let buffer else { throw WindowError.allocationFailed }
        let image = CIImage(cvPixelBuffer: source)
        let scaled = image.transformed(by: CGAffineTransform(
            scaleX: CGFloat(DA3WindowGeometry.width) / image.extent.width,
            y: CGFloat(DA3WindowGeometry.height) / image.extent.height))
        context.render(scaled, to: buffer, bounds: CGRect(x: 0, y: 0,
                       width: DA3WindowGeometry.width, height: DA3WindowGeometry.height),
                       colorSpace: CGColorSpace(name: CGColorSpace.sRGB))
        return buffer
    }
}
