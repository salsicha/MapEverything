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
import simd

final class DepthAnythingProcessor {
    private let model: MLModel
    private let relativeDepthRequest: VNCoreMLRequest
    private let inferenceLock = NSLock()
    private let outputName: String
    private let inputSize: Int

    struct MaximumLikelihoodCalibration {
        let scale: Float
        let offset: Float
    }

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

        let request = VNCoreMLRequest(model: vnModel)
        request.imageCropAndScaleOption = .scaleFill

        self.model = loadedModel
        self.relativeDepthRequest = request

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
        inferenceLock.lock()
        defer { inferenceLock.unlock() }

        let handler = VNImageRequestHandler(cvPixelBuffer: cameraImage, options: [:])
        do {
            try handler.perform([relativeDepthRequest])
        } catch {
            print("DepthAnythingProcessor: Vision request failed: \(error)")
            return nil
        }

        if let pixelObservations = relativeDepthRequest.results as? [VNPixelBufferObservation],
           let first = pixelObservations.first {
            return RelativeDepthMap(fromPixelBuffer: first.pixelBuffer)
        }

        if let featureObservations = relativeDepthRequest.results as? [VNCoreMLFeatureValueObservation] {
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

        print("DepthAnythingProcessor: no usable result from model. Result type: \(String(describing: relativeDepthRequest.results))")
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

    /// Calibrates a Depth Anything relative depth map into metric depth using LiDAR.
    /// LiDAR samples are used only to estimate the affine metric transform; every
    /// valid Depth Anything pixel is then kept on the calibrated monocular surface.
    func fuseMaximumLikelihood(
        relative: RelativeDepthMap,
        lidarDepthMap: CVPixelBuffer,
        lidarConfidenceMap: CVPixelBuffer? = nil
    ) -> RelativeDepthMap? {
        CVPixelBufferLockBaseAddress(lidarDepthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(lidarDepthMap, .readOnly) }
        if let lidarConfidenceMap {
            CVPixelBufferLockBaseAddress(lidarConfidenceMap, .readOnly)
        }
        defer {
            if let lidarConfidenceMap {
                CVPixelBufferUnlockBaseAddress(lidarConfidenceMap, .readOnly)
            }
        }

        let lidarW = CVPixelBufferGetWidth(lidarDepthMap)
        let lidarH = CVPixelBufferGetHeight(lidarDepthMap)
        guard CVPixelBufferGetPixelFormatType(lidarDepthMap) == kCVPixelFormatType_DepthFloat32,
              let base = CVPixelBufferGetBaseAddress(lidarDepthMap)?.assumingMemoryBound(to: Float32.self)
        else { return nil }

        let floatsPerRow = CVPixelBufferGetBytesPerRow(lidarDepthMap) / MemoryLayout<Float32>.stride
        let confidenceSampling = Self.lidarConfidenceSampling(lidarConfidenceMap)
        guard let calibration = Self.calibrateRelativeDepth(
            relative: relative,
            lidarBase: base,
            lidarWidth: lidarW,
            lidarHeight: lidarH,
            lidarFloatsPerRow: floatsPerRow,
            confidenceSampling: confidenceSampling
        ) else { return nil }

        var fusedData = [Float](repeating: .nan, count: relative.width * relative.height)
        relative.withReadAccess { relativeReader in
            for y in 0..<relative.height {
                let normalizedY = Float(y) / Float(max(relative.height - 1, 1))
                let lidarY = min(lidarH - 1, max(0, Int(normalizedY * Float(max(lidarH - 1, 1)))))

                for x in 0..<relative.width {
                    let relativeDepth = relativeReader.value(atX: x, y: y)

                    let normalizedX = Float(x) / Float(max(relative.width - 1, 1))
                    let lidarX = min(lidarW - 1, max(0, Int(normalizedX * Float(max(lidarW - 1, 1)))))
                    if let depth = Self.maximumLikelihoodMetricDepth(
                        relativeDepth: relativeDepth,
                        lidarDepth: base[lidarY * floatsPerRow + lidarX],
                        calibration: calibration
                    ) {
                        fusedData[y * relative.width + x] = depth
                    }
                }
            }
        }

        return RelativeDepthMap(width: relative.width, height: relative.height, data: fusedData)
    }

    static func maximumLikelihoodCalibration(
        relative: RelativeDepthMap,
        lidarDepthMap: CVPixelBuffer,
        lidarConfidenceMap: CVPixelBuffer? = nil
    ) -> MaximumLikelihoodCalibration? {
        CVPixelBufferLockBaseAddress(lidarDepthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(lidarDepthMap, .readOnly) }
        if let lidarConfidenceMap {
            CVPixelBufferLockBaseAddress(lidarConfidenceMap, .readOnly)
        }
        defer {
            if let lidarConfidenceMap {
                CVPixelBufferUnlockBaseAddress(lidarConfidenceMap, .readOnly)
            }
        }

        let lidarW = CVPixelBufferGetWidth(lidarDepthMap)
        let lidarH = CVPixelBufferGetHeight(lidarDepthMap)
        guard CVPixelBufferGetPixelFormatType(lidarDepthMap) == kCVPixelFormatType_DepthFloat32,
              let base = CVPixelBufferGetBaseAddress(lidarDepthMap)?.assumingMemoryBound(to: Float32.self)
        else { return nil }

        let floatsPerRow = CVPixelBufferGetBytesPerRow(lidarDepthMap) / MemoryLayout<Float32>.stride
        let confidenceSampling = lidarConfidenceSampling(lidarConfidenceMap)
        return calibrateRelativeDepth(
            relative: relative,
            lidarBase: base,
            lidarWidth: lidarW,
            lidarHeight: lidarH,
            lidarFloatsPerRow: floatsPerRow,
            confidenceSampling: confidenceSampling
        )
    }

    static func maximumLikelihoodMetricDepth(
        relativeDepth: Float,
        lidarDepth: Float,
        calibration: MaximumLikelihoodCalibration
    ) -> Float? {
        calibratedMetricDepth(relativeDepth: relativeDepth, calibration: calibration)
    }

    static func calibratedMetricDepth(
        relativeDepth: Float,
        calibration: MaximumLikelihoodCalibration
    ) -> Float? {
        guard relativeDepth.isFinite, relativeDepth > 0 else { return nil }

        let monocularDepth = calibration.scale * relativeDepth + calibration.offset
        guard monocularDepth.isFinite, monocularDepth > 0.1 else { return nil }
        return monocularDepth
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

    private static func calibrateRelativeDepth(
        relative: RelativeDepthMap,
        lidarBase: UnsafePointer<Float32>,
        lidarWidth: Int,
        lidarHeight: Int,
        lidarFloatsPerRow: Int,
        confidenceSampling: LiDARConfidenceSampling
    ) -> MaximumLikelihoodCalibration? {
        relative.withReadAccess { relativeReader in
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
                    guard Self.isValidLiDARDepth(lidarDepth) else { continue }

                    let nx = Float(x) / Float(max(lidarWidth - 1, 1))
                    let ny = Float(y) / Float(max(lidarHeight - 1, 1))
                    let confidenceWeight = Self.lidarConfidenceWeight(
                        confidenceSampling.value(normalizedX: nx, normalizedY: ny)
                    )
                    guard confidenceWeight > 0 else { continue }

                    let rx = Int(nx * Float(max(relative.width - 1, 1)))
                    let ry = Int(ny * Float(max(relative.height - 1, 1)))
                    let r = relativeReader.value(atX: rx, y: ry)
                    guard r.isFinite, r > 0 else { continue }

                    let sigma = Self.lidarStandardDeviation(depth: lidarDepth)
                    let weight = Double(confidenceWeight / (sigma * sigma))
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

            return MaximumLikelihoodCalibration(scale: scale, offset: offset)
        }
    }

    private struct LiDARConfidenceSampling {
        let base: UnsafePointer<UInt8>?
        let width: Int
        let height: Int
        let bytesPerRow: Int

        func value(normalizedX: Float, normalizedY: Float) -> UInt8? {
            guard let base, width > 0, height > 0 else { return nil }
            let x = min(width - 1, max(0, Int(normalizedX * Float(max(width - 1, 1)))))
            let y = min(height - 1, max(0, Int(normalizedY * Float(max(height - 1, 1)))))
            return base[y * bytesPerRow + x]
        }
    }

    private static func lidarConfidenceSampling(_ confidenceMap: CVPixelBuffer?) -> LiDARConfidenceSampling {
        guard let confidenceMap,
              CVPixelBufferGetPixelFormatType(confidenceMap) == kCVPixelFormatType_OneComponent8,
              let base = CVPixelBufferGetBaseAddress(confidenceMap)?.assumingMemoryBound(to: UInt8.self)
        else {
            return LiDARConfidenceSampling(base: nil, width: 0, height: 0, bytesPerRow: 0)
        }

        return LiDARConfidenceSampling(
            base: base,
            width: CVPixelBufferGetWidth(confidenceMap),
            height: CVPixelBufferGetHeight(confidenceMap),
            bytesPerRow: CVPixelBufferGetBytesPerRow(confidenceMap)
        )
    }

    private static func lidarConfidenceWeight(_ confidence: UInt8?) -> Float {
        guard let confidence else { return 1.0 }
        switch confidence {
        case 0:
            return 0.20
        case 1:
            return 0.65
        default:
            return 1.0
        }
    }

    private static func isValidLiDARDepth(_ depth: Float) -> Bool {
        depth.isFinite && depth > 0.2 && depth < 5.0
    }

    private static func lidarStandardDeviation(depth: Float) -> Float {
        max(0.015, 0.012 + 0.004 * depth * depth)
    }

    private static func monocularStandardDeviation(depth: Float) -> Float {
        max(0.15, 0.10 + 0.08 * depth)
    }

}

final class DepthAnythingCalibrationCache {
    private struct Entry {
        let calibration: DepthAnythingProcessor.MaximumLikelihoodCalibration
        let timestamp: TimeInterval
        let cameraTransform: simd_float4x4
        let relativeWidth: Int
        let relativeHeight: Int
        let lidarWidth: Int
        let lidarHeight: Int
        let lidarConfidenceWidth: Int?
        let lidarConfidenceHeight: Int?
    }

    private let lock = NSLock()
    private let maxAge: TimeInterval
    private let maxTranslationMeters: Float
    private let maxRotationRadians: Float
    private var entry: Entry?

    init(
        maxAge: TimeInterval = 1.25,
        maxTranslationMeters: Float = 0.20,
        maxRotationRadians: Float = 0.25
    ) {
        self.maxAge = maxAge
        self.maxTranslationMeters = maxTranslationMeters
        self.maxRotationRadians = maxRotationRadians
    }

    func reset() {
        lock.lock()
        entry = nil
        lock.unlock()
    }

    func calibration(
        relative: RelativeDepthMap,
        lidarDepthMap: CVPixelBuffer,
        lidarConfidenceMap: CVPixelBuffer? = nil,
        timestamp: TimeInterval,
        cameraTransform: simd_float4x4
    ) -> DepthAnythingProcessor.MaximumLikelihoodCalibration? {
        let lidarWidth = CVPixelBufferGetWidth(lidarDepthMap)
        let lidarHeight = CVPixelBufferGetHeight(lidarDepthMap)
        let lidarConfidenceWidth = lidarConfidenceMap.map(CVPixelBufferGetWidth)
        let lidarConfidenceHeight = lidarConfidenceMap.map(CVPixelBufferGetHeight)

        lock.lock()
        if let entry,
           isReusable(
               entry,
               relative: relative,
               lidarWidth: lidarWidth,
               lidarHeight: lidarHeight,
               lidarConfidenceWidth: lidarConfidenceWidth,
               lidarConfidenceHeight: lidarConfidenceHeight,
               timestamp: timestamp,
               cameraTransform: cameraTransform
           ) {
            let calibration = entry.calibration
            lock.unlock()
            return calibration
        }
        lock.unlock()

        guard let calibration = DepthAnythingProcessor.maximumLikelihoodCalibration(
            relative: relative,
            lidarDepthMap: lidarDepthMap,
            lidarConfidenceMap: lidarConfidenceMap
        ) else { return nil }

        let nextEntry = Entry(
            calibration: calibration,
            timestamp: timestamp,
            cameraTransform: cameraTransform,
            relativeWidth: relative.width,
            relativeHeight: relative.height,
            lidarWidth: lidarWidth,
            lidarHeight: lidarHeight,
            lidarConfidenceWidth: lidarConfidenceWidth,
            lidarConfidenceHeight: lidarConfidenceHeight
        )

        lock.lock()
        entry = nextEntry
        lock.unlock()

        return calibration
    }

    private func isReusable(
        _ entry: Entry,
        relative: RelativeDepthMap,
        lidarWidth: Int,
        lidarHeight: Int,
        lidarConfidenceWidth: Int?,
        lidarConfidenceHeight: Int?,
        timestamp: TimeInterval,
        cameraTransform: simd_float4x4
    ) -> Bool {
        guard timestamp >= entry.timestamp,
              timestamp - entry.timestamp <= maxAge,
              entry.relativeWidth == relative.width,
              entry.relativeHeight == relative.height,
              entry.lidarWidth == lidarWidth,
              entry.lidarHeight == lidarHeight,
              entry.lidarConfidenceWidth == lidarConfidenceWidth,
              entry.lidarConfidenceHeight == lidarConfidenceHeight else {
            return false
        }

        let translationDelta = simd_length(Self.position(cameraTransform) - Self.position(entry.cameraTransform))
        guard translationDelta <= maxTranslationMeters else { return false }

        return Self.rotationDeltaRadians(cameraTransform, entry.cameraTransform) <= maxRotationRadians
    }

    private static func position(_ transform: simd_float4x4) -> SIMD3<Float> {
        SIMD3<Float>(transform.columns.3.x, transform.columns.3.y, transform.columns.3.z)
    }

    private static func rotationDeltaRadians(_ lhs: simd_float4x4, _ rhs: simd_float4x4) -> Float {
        let lhsForward = normalized(-SIMD3<Float>(lhs.columns.2.x, lhs.columns.2.y, lhs.columns.2.z))
        let rhsForward = normalized(-SIMD3<Float>(rhs.columns.2.x, rhs.columns.2.y, rhs.columns.2.z))
        let lhsUp = normalized(SIMD3<Float>(lhs.columns.1.x, lhs.columns.1.y, lhs.columns.1.z))
        let rhsUp = normalized(SIMD3<Float>(rhs.columns.1.x, rhs.columns.1.y, rhs.columns.1.z))

        guard let lhsForward, let rhsForward, let lhsUp, let rhsUp else { return .greatestFiniteMagnitude }

        return max(angleRadians(lhsForward, rhsForward), angleRadians(lhsUp, rhsUp))
    }

    private static func normalized(_ value: SIMD3<Float>) -> SIMD3<Float>? {
        let length = simd_length(value)
        guard length > 1e-5 else { return nil }
        return value / length
    }

    private static func angleRadians(_ lhs: SIMD3<Float>, _ rhs: SIMD3<Float>) -> Float {
        let cosine = min(1, max(-1, simd_dot(lhs, rhs)))
        return acos(cosine)
    }
}

/// Dense depth map view. Depth Anything outputs stay backed by their native
/// MLMultiArray or CVPixelBuffer until a caller explicitly asks for `data`.
struct RelativeDepthMap {
    private enum Storage {
        case array([Float])
        case multiArray(MultiArrayStorage)
        case pixelBuffer(PixelBufferStorage)
    }

    private struct MultiArrayStorage {
        let array: MLMultiArray
        let dataType: MLMultiArrayDataType
        let yStride: Int
        let xStride: Int
    }

    private struct PixelBufferStorage {
        let buffer: CVPixelBuffer
        let format: OSType
    }

    private var storage: Storage
    let width: Int
    let height: Int
    var data: [Float] {
        get {
            var values = [Float](repeating: .nan, count: width * height)
            withReadAccess { reader in
                for y in 0..<height {
                    for x in 0..<width {
                        values[y * width + x] = reader.value(atX: x, y: y)
                    }
                }
            }
            return values
        }
        set {
            storage = .array(newValue)
        }
    }

    init(width: Int, height: Int, data: [Float]) {
        self.width = width
        self.height = height
        self.storage = .array(data)
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
        guard h > 0, w > 0 else { return nil }

        let strides = array.strides.map { $0.intValue }
        let yStride = strides.count >= 2 ? strides[strides.count - 2] : w
        let xStride = strides.count >= 1 ? strides[strides.count - 1] : 1
        guard [.float32, .float16, .double].contains(array.dataType) else { return nil }

        self.width = w
        self.height = h
        self.storage = .multiArray(
            MultiArrayStorage(
                array: array,
                dataType: array.dataType,
                yStride: yStride,
                xStride: xStride
            )
        )
    }

    init?(fromPixelBuffer buffer: CVPixelBuffer) {
        let w = CVPixelBufferGetWidth(buffer)
        let h = CVPixelBufferGetHeight(buffer)
        let format = CVPixelBufferGetPixelFormatType(buffer)
        switch format {
        case kCVPixelFormatType_DepthFloat32, kCVPixelFormatType_OneComponent32Float:
            break
        case kCVPixelFormatType_DepthFloat16, kCVPixelFormatType_OneComponent16Half:
            break
        default:
            return nil
        }

        self.width = w
        self.height = h
        self.storage = .pixelBuffer(PixelBufferStorage(buffer: buffer, format: format))
    }

    @inline(__always)
    func value(atX x: Int, y: Int) -> Float {
        withReadAccess { reader in
            reader.value(atX: x, y: y)
        }
    }

    func withReadAccess<Result>(_ body: (RelativeDepthMapReader) -> Result) -> Result {
        switch storage {
        case .array(let values):
            return values.withUnsafeBufferPointer { buffer in
                body(
                    RelativeDepthMapReader(
                        width: width,
                        height: height,
                        storage: .array(buffer)
                    )
                )
            }
        case .multiArray(let multiArray):
            let pointer = multiArray.array.dataPointer
            switch multiArray.dataType {
            case .float32:
                return body(
                    RelativeDepthMapReader(
                        width: width,
                        height: height,
                        storage: .multiArrayFloat32(
                            pointer.assumingMemoryBound(to: Float.self),
                            yStride: multiArray.yStride,
                            xStride: multiArray.xStride
                        )
                    )
                )
            case .float16:
                return body(
                    RelativeDepthMapReader(
                        width: width,
                        height: height,
                        storage: .multiArrayFloat16(
                            pointer.assumingMemoryBound(to: UInt16.self),
                            yStride: multiArray.yStride,
                            xStride: multiArray.xStride
                        )
                    )
                )
            case .double:
                return body(
                    RelativeDepthMapReader(
                        width: width,
                        height: height,
                        storage: .multiArrayDouble(
                            pointer.assumingMemoryBound(to: Double.self),
                            yStride: multiArray.yStride,
                            xStride: multiArray.xStride
                        )
                    )
                )
            default:
                return body(RelativeDepthMapReader(width: width, height: height, storage: .empty))
            }
        case .pixelBuffer(let pixelBuffer):
            CVPixelBufferLockBaseAddress(pixelBuffer.buffer, .readOnly)
            defer { CVPixelBufferUnlockBaseAddress(pixelBuffer.buffer, .readOnly) }
            guard let baseAddress = CVPixelBufferGetBaseAddress(pixelBuffer.buffer) else {
                return body(RelativeDepthMapReader(width: width, height: height, storage: .empty))
            }

            let bytesPerRow = CVPixelBufferGetBytesPerRow(pixelBuffer.buffer)
            switch pixelBuffer.format {
            case kCVPixelFormatType_DepthFloat32, kCVPixelFormatType_OneComponent32Float:
                return body(
                    RelativeDepthMapReader(
                        width: width,
                        height: height,
                        storage: .pixelFloat32(
                            baseAddress.assumingMemoryBound(to: Float.self),
                            floatsPerRow: bytesPerRow / MemoryLayout<Float>.stride
                        )
                    )
                )
            case kCVPixelFormatType_DepthFloat16, kCVPixelFormatType_OneComponent16Half:
                return body(
                    RelativeDepthMapReader(
                        width: width,
                        height: height,
                        storage: .pixelFloat16(
                            baseAddress.assumingMemoryBound(to: UInt16.self),
                            valuesPerRow: bytesPerRow / MemoryLayout<UInt16>.stride
                        )
                    )
                )
            default:
                return body(RelativeDepthMapReader(width: width, height: height, storage: .empty))
            }
        }
    }

    mutating func applyAffine(a: Float, b: Float) {
        var transformed = [Float](repeating: .nan, count: width * height)
        withReadAccess { reader in
            for y in 0..<height {
                for x in 0..<width {
                    transformed[y * width + x] = a * reader.value(atX: x, y: y) + b
                }
            }
        }
        storage = .array(transformed)
    }
}

struct RelativeDepthMapReader {
    enum Storage {
        case array(UnsafeBufferPointer<Float>)
        case multiArrayFloat32(UnsafePointer<Float>, yStride: Int, xStride: Int)
        case multiArrayFloat16(UnsafePointer<UInt16>, yStride: Int, xStride: Int)
        case multiArrayDouble(UnsafePointer<Double>, yStride: Int, xStride: Int)
        case pixelFloat32(UnsafePointer<Float>, floatsPerRow: Int)
        case pixelFloat16(UnsafePointer<UInt16>, valuesPerRow: Int)
        case empty
    }

    let width: Int
    let height: Int
    let storage: Storage

    @inline(__always)
    func value(atX x: Int, y: Int) -> Float {
        guard x >= 0, x < width, y >= 0, y < height else { return .nan }
        switch storage {
        case .array(let values):
            return values[y * width + x]
        case .multiArrayFloat32(let pointer, let yStride, let xStride):
            return pointer[y * yStride + x * xStride]
        case .multiArrayFloat16(let pointer, let yStride, let xStride):
            return Float(Float16(bitPattern: pointer[y * yStride + x * xStride]))
        case .multiArrayDouble(let pointer, let yStride, let xStride):
            return Float(pointer[y * yStride + x * xStride])
        case .pixelFloat32(let pointer, let floatsPerRow):
            return pointer[y * floatsPerRow + x]
        case .pixelFloat16(let pointer, let valuesPerRow):
            return Float(Float16(bitPattern: pointer[y * valuesPerRow + x]))
        case .empty:
            return .nan
        }
    }
}
