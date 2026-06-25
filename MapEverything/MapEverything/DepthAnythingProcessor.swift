//
//  DepthAnythingProcessor.swift
//  MapEverything
//
//  Loads Apple's Depth Anything V2 Core ML model and fuses its dense
//  relative-depth output with sparse LiDAR depth to produce metric depth.
//
//  Setup: download the .mlpackage from
//    https://huggingface.co/apple/coreml-depth-anything-v2-small
//  and drag it into the Xcode project. Expected resource name:
//    "DepthAnythingV2SmallF16" (.mlmodelc after compilation).
//

import Foundation
import CoreML
import CoreVideo
import CoreImage
import Vision
import ARKit
import Accelerate

final class DepthAnythingProcessor {
    private let model: MLModel
    private let visionModel: VNCoreMLModel
    private let outputName: String
    private let inputSize: Int

    /// Loads the model from the app bundle. Returns nil if the model is missing,
    /// which lets the rest of the app continue working without depth enhancement.
    init?(modelResourceName: String = "DepthAnythingV2SmallF16") {
        let config = MLModelConfiguration()
        config.computeUnits = .cpuAndNeuralEngine

        var modelURL: URL?
        if let url = Bundle.main.url(forResource: modelResourceName, withExtension: "mlmodelc") {
            modelURL = url
        } else if let url = Bundle.main.url(forResource: modelResourceName, withExtension: "mlpackage") {
            if let compiledURL = try? MLModel.compileModel(at: url) {
                modelURL = compiledURL
            }
        }

        guard let resolvedURL = modelURL,
              let loadedModel = try? MLModel(contentsOf: resolvedURL, configuration: config),
              let vnModel = try? VNCoreMLModel(for: loadedModel) else {
            return nil
        }

        self.model = loadedModel
        self.visionModel = vnModel

        guard let inName = loadedModel.modelDescription.inputDescriptionsByName.keys.first,
              let outName = loadedModel.modelDescription.outputDescriptionsByName.keys.first else {
            return nil
        }
        self.outputName = outName

        if let imageConstraint = loadedModel.modelDescription.inputDescriptionsByName[inName]?.imageConstraint {
            self.inputSize = imageConstraint.pixelsWide
        } else {
            self.inputSize = 518
        }
    }

    /// Runs Depth Anything V2 on the given camera image and returns a dense relative-depth map.
    /// Uses Vision framework so preprocessing (resize, format conversion, normalization)
    /// is handled automatically based on the model's input constraints.
    func inferRelativeDepth(from cameraImage: CVPixelBuffer) -> RelativeDepthMap? {
        let request = VNCoreMLRequest(model: visionModel)
        request.imageCropAndScaleOption = .scaleFill

        let handler = VNImageRequestHandler(cvPixelBuffer: cameraImage, options: [:])
        do {
            try handler.perform([request])
        } catch {
            print("DepthAnythingProcessor: Vision request failed: \(error)")
            return nil
        }

        if let pixelObservations = request.results as? [VNPixelBufferObservation],
           let first = pixelObservations.first {
            return RelativeDepthMap(fromPixelBuffer: first.pixelBuffer)
        }

        if let featureObservations = request.results as? [VNCoreMLFeatureValueObservation] {
            for observation in featureObservations {
                let feature = observation.featureValue
                if let array = feature.multiArrayValue {
                    return RelativeDepthMap(fromMultiArray: array, size: inputSize)
                }
                if let buffer = feature.imageBufferValue {
                    return RelativeDepthMap(fromPixelBuffer: buffer)
                }
            }
        }

        print("DepthAnythingProcessor: no usable result from model. Result type: \(String(describing: request.results))")
        return nil
    }

    func warmUp() {
        var pixelBuffer: CVPixelBuffer?
        let attributes: [CFString: Any] = [
            kCVPixelBufferCGImageCompatibilityKey: true,
            kCVPixelBufferCGBitmapContextCompatibilityKey: true
        ]
        let status = CVPixelBufferCreate(
            kCFAllocatorDefault,
            inputSize,
            inputSize,
            kCVPixelFormatType_32BGRA,
            attributes as CFDictionary,
            &pixelBuffer
        )

        guard status == kCVReturnSuccess, let pixelBuffer else { return }
        CVPixelBufferLockBaseAddress(pixelBuffer, [])
        if let baseAddress = CVPixelBufferGetBaseAddress(pixelBuffer) {
            memset(baseAddress, 0, CVPixelBufferGetBytesPerRow(pixelBuffer) * inputSize)
        }
        CVPixelBufferUnlockBaseAddress(pixelBuffer, [])

        _ = inferRelativeDepth(from: pixelBuffer)
    }

    /// Fuses a Depth Anything relative depth map with LiDAR using a maximum-likelihood
    /// estimate. First calibrates monocular relative depth into metric depth using
    /// weighted least squares on valid LiDAR samples, then performs per-pixel
    /// inverse-variance fusion where LiDAR exists. This keeps LiDAR dominant at short
    /// range while preserving calibrated monocular depth outside the LiDAR return range.
    func fuseMaximumLikelihood(relative: RelativeDepthMap, lidarDepthMap: CVPixelBuffer) -> RelativeDepthMap? {
        CVPixelBufferLockBaseAddress(lidarDepthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(lidarDepthMap, .readOnly) }

        let lidarW = CVPixelBufferGetWidth(lidarDepthMap)
        let lidarH = CVPixelBufferGetHeight(lidarDepthMap)
        guard CVPixelBufferGetPixelFormatType(lidarDepthMap) == kCVPixelFormatType_DepthFloat32,
              let base = CVPixelBufferGetBaseAddress(lidarDepthMap)?.assumingMemoryBound(to: Float32.self)
        else { return nil }

        let floatsPerRow = CVPixelBufferGetBytesPerRow(lidarDepthMap) / MemoryLayout<Float32>.stride
        guard let calibration = calibrateRelativeDepth(
            relative: relative,
            lidarBase: base,
            lidarWidth: lidarW,
            lidarHeight: lidarH,
            lidarFloatsPerRow: floatsPerRow
        ) else { return nil }

        var fused = RelativeDepthMap(
            width: relative.width,
            height: relative.height,
            data: [Float](repeating: .nan, count: relative.width * relative.height)
        )

        for y in 0..<relative.height {
            let normalizedY = Float(y) / Float(max(relative.height - 1, 1))
            let lidarY = min(lidarH - 1, max(0, Int(normalizedY * Float(max(lidarH - 1, 1)))))

            for x in 0..<relative.width {
                let relativeDepth = relative.value(atX: x, y: y)
                guard relativeDepth.isFinite, relativeDepth > 0 else { continue }

                let monocularDepth = calibration.scale * relativeDepth + calibration.offset
                guard monocularDepth.isFinite, monocularDepth > 0.1, monocularDepth < 20.0 else { continue }

                let normalizedX = Float(x) / Float(max(relative.width - 1, 1))
                let lidarX = min(lidarW - 1, max(0, Int(normalizedX * Float(max(lidarW - 1, 1)))))
                let lidarDepth = base[lidarY * floatsPerRow + lidarX]

                if isValidLiDARDepth(lidarDepth) {
                    let lidarSigma = lidarStandardDeviation(depth: lidarDepth)
                    let monocularSigma = monocularStandardDeviation(depth: monocularDepth)
                    let lidarPrecision = 1.0 / (lidarSigma * lidarSigma)
                    let monocularPrecision = 1.0 / (monocularSigma * monocularSigma)
                    let maximumLikelihoodDepth = (
                        lidarDepth * lidarPrecision + monocularDepth * monocularPrecision
                    ) / (lidarPrecision + monocularPrecision)

                    fused.data[y * relative.width + x] = maximumLikelihoodDepth
                } else {
                    fused.data[y * relative.width + x] = monocularDepth
                }
            }
        }

        return fused
    }

    /// Fuses a Depth Anything relative depth map with the sparse LiDAR depth from an ARFrame
    /// to produce metric dense depth. Uses least-squares to solve `metric = a * relative + b`
    /// on valid LiDAR samples, then applies the same affine transform across the whole map.
    func fuseWithLiDAR(relative: RelativeDepthMap, lidarDepthMap: CVPixelBuffer) -> RelativeDepthMap? {
        CVPixelBufferLockBaseAddress(lidarDepthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(lidarDepthMap, .readOnly) }

        let lidarW = CVPixelBufferGetWidth(lidarDepthMap)
        let lidarH = CVPixelBufferGetHeight(lidarDepthMap)
        guard CVPixelBufferGetPixelFormatType(lidarDepthMap) == kCVPixelFormatType_DepthFloat32,
              let base = CVPixelBufferGetBaseAddress(lidarDepthMap)?.assumingMemoryBound(to: Float32.self)
        else { return nil }

        let floatsPerRow = CVPixelBufferGetBytesPerRow(lidarDepthMap) / MemoryLayout<Float32>.stride

        var sumX: Double = 0, sumY: Double = 0
        var sumXX: Double = 0, sumXY: Double = 0
        var n: Int = 0

        // Sample LiDAR pixels and the corresponding relative-depth pixel
        let step = 4
        for y in stride(from: 0, to: lidarH, by: step) {
            for x in stride(from: 0, to: lidarW, by: step) {
                let lidarDepth = base[y * floatsPerRow + x]
                guard lidarDepth > 0.2 && lidarDepth < 5.0 else { continue }

                // Map (x, y) in LiDAR space → (rx, ry) in relative-depth space
                let nx = Float(x) / Float(lidarW - 1)
                let ny = Float(y) / Float(lidarH - 1)
                let rx = Int(nx * Float(relative.width - 1))
                let ry = Int(ny * Float(relative.height - 1))
                let r = relative.value(atX: rx, y: ry)
                guard r.isFinite, r > 0 else { continue }

                let xv = Double(r)
                let yv = Double(lidarDepth)
                sumX += xv
                sumY += yv
                sumXX += xv * xv
                sumXY += xv * yv
                n += 1
            }
        }

        guard n > 50 else { return nil }

        let dn = Double(n)
        let denom = (dn * sumXX - sumX * sumX)
        guard abs(denom) > 1e-6 else { return nil }
        let a = Float((dn * sumXY - sumX * sumY) / denom)
        let b = Float((sumY * sumXX - sumX * sumXY) / denom)

        // Apply the affine transform to the whole relative depth map
        var fused = relative
        fused.applyAffine(a: a, b: b)
        return fused
    }

    private struct DepthCalibration {
        let scale: Float
        let offset: Float
    }

    private func calibrateRelativeDepth(
        relative: RelativeDepthMap,
        lidarBase: UnsafePointer<Float32>,
        lidarWidth: Int,
        lidarHeight: Int,
        lidarFloatsPerRow: Int
    ) -> DepthCalibration? {
        var sumW: Double = 0
        var sumX: Double = 0
        var sumY: Double = 0
        var sumXX: Double = 0
        var sumXY: Double = 0
        var sampleCount = 0

        let step = 4
        for y in stride(from: 0, to: lidarHeight, by: step) {
            for x in stride(from: 0, to: lidarWidth, by: step) {
                let lidarDepth = lidarBase[y * lidarFloatsPerRow + x]
                guard isValidLiDARDepth(lidarDepth) else { continue }

                let nx = Float(x) / Float(max(lidarWidth - 1, 1))
                let ny = Float(y) / Float(max(lidarHeight - 1, 1))
                let rx = Int(nx * Float(max(relative.width - 1, 1)))
                let ry = Int(ny * Float(max(relative.height - 1, 1)))
                let r = relative.value(atX: rx, y: ry)
                guard r.isFinite, r > 0 else { continue }

                let sigma = lidarStandardDeviation(depth: lidarDepth)
                let weight = Double(1.0 / (sigma * sigma))
                let xv = Double(r)
                let yv = Double(lidarDepth)

                sumW += weight
                sumX += weight * xv
                sumY += weight * yv
                sumXX += weight * xv * xv
                sumXY += weight * xv * yv
                sampleCount += 1
            }
        }

        guard sampleCount > 50 else { return nil }

        let denom = (sumW * sumXX - sumX * sumX)
        guard abs(denom) > 1e-6 else { return nil }

        let scale = Float((sumW * sumXY - sumX * sumY) / denom)
        let offset = Float((sumY * sumXX - sumX * sumXY) / denom)
        guard scale.isFinite, offset.isFinite else { return nil }

        return DepthCalibration(scale: scale, offset: offset)
    }

    private func isValidLiDARDepth(_ depth: Float) -> Bool {
        depth.isFinite && depth > 0.2 && depth < 5.0
    }

    private func lidarStandardDeviation(depth: Float) -> Float {
        max(0.015, 0.012 + 0.004 * depth * depth)
    }

    private func monocularStandardDeviation(depth: Float) -> Float {
        max(0.15, 0.10 + 0.08 * depth)
    }

}

/// Dense depth map stored as a flat Float32 buffer.
struct RelativeDepthMap {
    var data: [Float]
    let width: Int
    let height: Int

    init(width: Int, height: Int, data: [Float]) {
        self.width = width
        self.height = height
        self.data = data
    }

    init?(fromMultiArray array: MLMultiArray, size: Int) {
        // The output is typically [1, H, W] or [H, W]
        let shape = array.shape.map { $0.intValue }
        let count = shape.reduce(1, *)
        guard count > 0 else { return nil }

        // Infer height/width: prefer last two dims if rank >= 2
        let h: Int
        let w: Int
        if shape.count >= 2 {
            h = shape[shape.count - 2]
            w = shape[shape.count - 1]
        } else {
            h = size; w = size
        }

        var floats = [Float](repeating: 0, count: h * w)
        let dataPointer = array.dataPointer

        switch array.dataType {
        case .float32:
            let typed = dataPointer.assumingMemoryBound(to: Float.self)
            for i in 0..<(h * w) { floats[i] = typed[i] }
        case .float16:
            // Float16 → Float32 conversion via vImage
            let typed = dataPointer.assumingMemoryBound(to: UInt16.self)
            var srcBuffer = vImage_Buffer(
                data: UnsafeMutableRawPointer(mutating: typed),
                height: vImagePixelCount(h),
                width: vImagePixelCount(w),
                rowBytes: w * 2
            )
            var dstBuffer = floats.withUnsafeMutableBufferPointer { ptr -> vImage_Buffer in
                vImage_Buffer(
                    data: ptr.baseAddress,
                    height: vImagePixelCount(h),
                    width: vImagePixelCount(w),
                    rowBytes: w * 4
                )
            }
            _ = vImageConvert_Planar16FtoPlanarF(&srcBuffer, &dstBuffer, 0)
        case .double:
            let typed = dataPointer.assumingMemoryBound(to: Double.self)
            for i in 0..<(h * w) { floats[i] = Float(typed[i]) }
        default:
            return nil
        }

        self.width = w
        self.height = h
        self.data = floats
    }

    init?(fromPixelBuffer buffer: CVPixelBuffer) {
        CVPixelBufferLockBaseAddress(buffer, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(buffer, .readOnly) }
        let w = CVPixelBufferGetWidth(buffer)
        let h = CVPixelBufferGetHeight(buffer)
        let format = CVPixelBufferGetPixelFormatType(buffer)
        guard let baseAddress = CVPixelBufferGetBaseAddress(buffer) else { return nil }
        let bytesPerRow = CVPixelBufferGetBytesPerRow(buffer)
        var data = [Float](repeating: 0, count: w * h)

        switch format {
        case kCVPixelFormatType_DepthFloat32, kCVPixelFormatType_OneComponent32Float:
            let ptr = baseAddress.assumingMemoryBound(to: Float.self)
            let floatsPerRow = bytesPerRow / MemoryLayout<Float>.stride
            for y in 0..<h {
                for x in 0..<w {
                    data[y * w + x] = ptr[y * floatsPerRow + x]
                }
            }
        case kCVPixelFormatType_DepthFloat16, kCVPixelFormatType_OneComponent16Half:
            var src = vImage_Buffer(data: baseAddress, height: vImagePixelCount(h),
                                    width: vImagePixelCount(w), rowBytes: bytesPerRow)
            var dst = data.withUnsafeMutableBufferPointer { ptr -> vImage_Buffer in
                vImage_Buffer(data: ptr.baseAddress, height: vImagePixelCount(h),
                              width: vImagePixelCount(w), rowBytes: w * 4)
            }
            guard vImageConvert_Planar16FtoPlanarF(&src, &dst, 0) == kvImageNoError else { return nil }
        default:
            return nil
        }

        self.width = w
        self.height = h
        self.data = data
    }

    @inline(__always)
    func value(atX x: Int, y: Int) -> Float {
        return data[y * width + x]
    }

    mutating func applyAffine(a: Float, b: Float) {
        for i in 0..<data.count {
            data[i] = a * data[i] + b
        }
    }
}
