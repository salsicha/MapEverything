//
//  PointCloudProcessor.swift
//  MapEverything
//
//  Created by Alex Moran on 5/8/26.
//

import Foundation
import ARKit
import CoreVideo

public struct ColoredPoint: Equatable, Sendable {
    public let position: SIMD3<Float>
    public let color: SIMD3<UInt8>
    
    public init(position: SIMD3<Float>, color: SIMD3<UInt8> = SIMD3<UInt8>(255, 255, 255)) {
        self.position = position
        self.color = color
    }
}

public struct ColoredSurfel: Equatable, Sendable {
    public let position: SIMD3<Float>
    public let normal: SIMD3<Float>
    public let color: SIMD3<UInt8>
    public let radius: Float
    public let confidence: Float
    public let observationCount: UInt32

    public init(
        position: SIMD3<Float>,
        normal: SIMD3<Float> = SIMD3<Float>(0, 1, 0),
        color: SIMD3<UInt8> = SIMD3<UInt8>(255, 255, 255),
        radius: Float = 0.03,
        confidence: Float = 0,
        observationCount: UInt32 = 1
    ) {
        self.position = position
        self.normal = normal
        self.color = color
        self.radius = radius
        self.confidence = confidence
        self.observationCount = observationCount
    }

    var coloredPoint: ColoredPoint {
        ColoredPoint(position: position, color: color)
    }
}

private struct PointProjectionSample: Sendable {
    let depthX: Int
    let depthY: Int
    let depthIndex: Int
    let lidarIndex: Int
    let cameraXFactor: Float
    let cameraYFactor: Float
    let yIndex: Int
    let uvIndex: Int
}

private struct PointProjectionTable: Sendable {
    let samples: [PointProjectionSample]

    static let empty = PointProjectionTable(samples: [])
}

private struct PointProjectionTableKey: Hashable, Sendable {
    let depthWidth: Int
    let depthHeight: Int
    let depthFloatsPerRow: Int
    let imageWidth: Int
    let imageHeight: Int
    let yBytesPerRow: Int
    let cbcrBytesPerRow: Int
    let lidarWidth: Int
    let lidarHeight: Int
    let lidarFloatsPerRow: Int
    let step: Int
    let fx: Int
    let fy: Int
    let cx: Int
    let cy: Int

    init(
        depthWidth: Int,
        depthHeight: Int,
        depthFloatsPerRow: Int,
        imageWidth: Int,
        imageHeight: Int,
        yBytesPerRow: Int,
        cbcrBytesPerRow: Int,
        lidarWidth: Int,
        lidarHeight: Int,
        lidarFloatsPerRow: Int,
        step: Int,
        fx: Float,
        fy: Float,
        cx: Float,
        cy: Float
    ) {
        self.depthWidth = depthWidth
        self.depthHeight = depthHeight
        self.depthFloatsPerRow = depthFloatsPerRow
        self.imageWidth = imageWidth
        self.imageHeight = imageHeight
        self.yBytesPerRow = yBytesPerRow
        self.cbcrBytesPerRow = cbcrBytesPerRow
        self.lidarWidth = lidarWidth
        self.lidarHeight = lidarHeight
        self.lidarFloatsPerRow = lidarFloatsPerRow
        self.step = step
        self.fx = Self.quantized(fx)
        self.fy = Self.quantized(fy)
        self.cx = Self.quantized(cx)
        self.cy = Self.quantized(cy)
    }

    private static func quantized(_ value: Float) -> Int {
        guard value.isFinite else { return Int.min }
        return Int((value * 100).rounded())
    }
}

private final class PointProjectionTableCache: @unchecked Sendable {
    private let lock = NSLock()
    private let capacity: Int
    private var entries: [PointProjectionTableKey: PointProjectionTable] = [:]
    private var keysByUse: [PointProjectionTableKey] = []

    init(capacity: Int = 8) {
        self.capacity = capacity
    }

    func table(for key: PointProjectionTableKey, build: () -> PointProjectionTable) -> PointProjectionTable {
        lock.lock()
        if let table = entries[key] {
            markRecentlyUsed(key)
            lock.unlock()
            return table
        }
        lock.unlock()

        let table = build()

        lock.lock()
        if let existing = entries[key] {
            markRecentlyUsed(key)
            lock.unlock()
            return existing
        }

        entries[key] = table
        keysByUse.append(key)
        while keysByUse.count > capacity {
            let expired = keysByUse.removeFirst()
            entries[expired] = nil
        }
        lock.unlock()

        return table
    }

    private func markRecentlyUsed(_ key: PointProjectionTableKey) {
        keysByUse.removeAll { $0 == key }
        keysByUse.append(key)
    }
}

struct PointCloudProcessor {
    private static let projectionTableCache = PointProjectionTableCache()

    private static func projectionTable(
        depthWidth: Int,
        depthHeight: Int,
        depthFloatsPerRow: Int = 0,
        imageWidth: Int,
        imageHeight: Int,
        yBytesPerRow: Int,
        cbcrBytesPerRow: Int,
        lidarWidth: Int = 0,
        lidarHeight: Int = 0,
        lidarFloatsPerRow: Int = 0,
        intrinsics: simd_float3x3,
        imageResolution resolution: CGSize,
        step: Int
    ) -> PointProjectionTable {
        guard depthWidth > 0,
              depthHeight > 0,
              imageWidth > 0,
              imageHeight > 0,
              step > 0,
              resolution.width > 0,
              resolution.height > 0 else {
            return .empty
        }

        let scaleX = Float(depthWidth) / Float(resolution.width)
        let scaleY = Float(depthHeight) / Float(resolution.height)
        let fx = intrinsics[0][0] * scaleX
        let fy = intrinsics[1][1] * scaleY
        let cx = intrinsics[2][0] * scaleX
        let cy = intrinsics[2][1] * scaleY
        guard fx.isFinite, fy.isFinite, cx.isFinite, cy.isFinite, abs(fx) > 1e-5, abs(fy) > 1e-5 else {
            return .empty
        }

        let normalizedDepthFloatsPerRow = depthFloatsPerRow > 0 ? depthFloatsPerRow : depthWidth
        let key = PointProjectionTableKey(
            depthWidth: depthWidth,
            depthHeight: depthHeight,
            depthFloatsPerRow: normalizedDepthFloatsPerRow,
            imageWidth: imageWidth,
            imageHeight: imageHeight,
            yBytesPerRow: yBytesPerRow,
            cbcrBytesPerRow: cbcrBytesPerRow,
            lidarWidth: lidarWidth,
            lidarHeight: lidarHeight,
            lidarFloatsPerRow: lidarFloatsPerRow,
            step: step,
            fx: fx,
            fy: fy,
            cx: cx,
            cy: cy
        )

        return projectionTableCache.table(for: key) {
            buildProjectionTable(
                depthWidth: depthWidth,
                depthHeight: depthHeight,
                depthFloatsPerRow: normalizedDepthFloatsPerRow,
                imageWidth: imageWidth,
                imageHeight: imageHeight,
                yBytesPerRow: yBytesPerRow,
                cbcrBytesPerRow: cbcrBytesPerRow,
                lidarWidth: lidarWidth,
                lidarHeight: lidarHeight,
                lidarFloatsPerRow: lidarFloatsPerRow,
                step: step,
                fx: fx,
                fy: fy,
                cx: cx,
                cy: cy
            )
        }
    }

    private static func buildProjectionTable(
        depthWidth: Int,
        depthHeight: Int,
        depthFloatsPerRow: Int,
        imageWidth: Int,
        imageHeight: Int,
        yBytesPerRow: Int,
        cbcrBytesPerRow: Int,
        lidarWidth: Int,
        lidarHeight: Int,
        lidarFloatsPerRow: Int,
        step: Int,
        fx: Float,
        fy: Float,
        cx: Float,
        cy: Float
    ) -> PointProjectionTable {
        let sampleColumns = (depthWidth + step - 1) / step
        let sampleRows = (depthHeight + step - 1) / step
        var samples: [PointProjectionSample] = []
        samples.reserveCapacity(sampleColumns * sampleRows)

        let maxDepthX = max(depthWidth - 1, 1)
        let maxDepthY = max(depthHeight - 1, 1)
        let maxLiDARX = max(lidarWidth - 1, 1)
        let maxLiDARY = max(lidarHeight - 1, 1)
        let hasLiDARLookup = lidarWidth > 0 && lidarHeight > 0 && lidarFloatsPerRow > 0

        for y in stride(from: 0, to: depthHeight, by: step) {
            let normalizedY = Float(y) / Float(maxDepthY)
            let imageY = min(imageHeight - 1, max(0, Int((Float(y) / Float(depthHeight)) * Float(imageHeight))))
            let lidarY = hasLiDARLookup ? min(lidarHeight - 1, max(0, Int(normalizedY * Float(maxLiDARY)))) : 0

            for x in stride(from: 0, to: depthWidth, by: step) {
                let normalizedX = Float(x) / Float(maxDepthX)
                let imageX = min(imageWidth - 1, max(0, Int((Float(x) / Float(depthWidth)) * Float(imageWidth))))
                let lidarX = hasLiDARLookup ? min(lidarWidth - 1, max(0, Int(normalizedX * Float(maxLiDARX)))) : 0

                samples.append(
                    PointProjectionSample(
                        depthX: x,
                        depthY: y,
                        depthIndex: y * depthFloatsPerRow + x,
                        lidarIndex: hasLiDARLookup ? lidarY * lidarFloatsPerRow + lidarX : -1,
                        cameraXFactor: (Float(x) - cx) / fx,
                        cameraYFactor: (cy - Float(y)) / fy,
                        yIndex: imageY * yBytesPerRow + imageX,
                        uvIndex: (imageY / 2) * cbcrBytesPerRow + (imageX / 2) * 2
                    )
                )
            }
        }

        return PointProjectionTable(samples: samples)
    }

    @inline(__always)
    private static func sampleColor(
        for sample: PointProjectionSample,
        yPlane: UnsafePointer<UInt8>?,
        cbcrPlane: UnsafePointer<UInt8>?
    ) -> SIMD3<UInt8> {
        guard let yPlane, let cbcrPlane else {
            return SIMD3<UInt8>(255, 255, 255)
        }

        let yVal = Float(yPlane[sample.yIndex])
        let cbVal = Float(cbcrPlane[sample.uvIndex]) - 128.0
        let crVal = Float(cbcrPlane[sample.uvIndex + 1]) - 128.0
        let r = yVal + 1.402 * crVal
        let g = yVal - 0.344136 * cbVal - 0.714136 * crVal
        let b = yVal + 1.772 * cbVal

        return SIMD3<UInt8>(
            UInt8(max(0, min(255, r))),
            UInt8(max(0, min(255, g))),
            UInt8(max(0, min(255, b)))
        )
    }

    
    /// Processes raw feature points and scene depth to create a structured point cloud.
    /// - Parameters:
    ///   - frame: The current ARFrame containing raw depth and feature points.
    ///   - transform: The camera transform to convert points to world coordinates.
    /// - Returns: An array of ColoredPoint representing the processed point cloud with RGB data.
    func processPointCloud(frame: ARFrame, transform: simd_float4x4) -> [ColoredPoint] {
        guard let sceneDepth = frame.smoothedSceneDepth ?? frame.sceneDepth else { return [] }
        return processPointCloud(
            depthMap: sceneDepth.depthMap,
            cameraImage: frame.capturedImage,
            intrinsics: frame.camera.intrinsics,
            imageResolution: frame.camera.imageResolution,
            transform: transform
        )
    }

    func processPointCloud(
        depthMap: CVPixelBuffer,
        cameraImage pixelBuffer: CVPixelBuffer,
        intrinsics: simd_float3x3,
        imageResolution resolution: CGSize,
        transform: simd_float4x4
    ) -> [ColoredPoint] {
        var processedPoints: [ColoredPoint] = []

        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }
        
        let depthWidth = CVPixelBufferGetWidth(depthMap)
        let depthHeight = CVPixelBufferGetHeight(depthMap)
        
        // Verify the depth map is in Float32 format before binding memory to prevent EXC_BAD_ACCESS crashes
        let pixelFormat = CVPixelBufferGetPixelFormatType(depthMap)
        guard pixelFormat == kCVPixelFormatType_DepthFloat32 else {
            print("Warning: Unsupported depth format. Expected Float32, got \(pixelFormat)")
            return processedPoints
        }
        
        guard let depthPointer = CVPixelBufferGetBaseAddress(depthMap)?.assumingMemoryBound(to: Float32.self) else { return processedPoints }
        
        // Calculate actual memory stride to prevent EXC_BAD_ACCESS crashes on padded GPU buffers
        let floatsPerRow = CVPixelBufferGetBytesPerRow(depthMap) / MemoryLayout<Float32>.stride
        
        CVPixelBufferLockBaseAddress(pixelBuffer, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(pixelBuffer, .readOnly) }
        
        let imgWidth = CVPixelBufferGetWidth(pixelBuffer)
        let imgHeight = CVPixelBufferGetHeight(pixelBuffer)
        
        let yPlane = CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 0)?.assumingMemoryBound(to: UInt8.self)
        let cbcrPlane = CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 1)?.assumingMemoryBound(to: UInt8.self)
        let yPerRow = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 0)
        let cbcrPerRow = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 1)
        
        let step = 4 // Process every 4th depth pixel for performance
        let table = Self.projectionTable(
            depthWidth: depthWidth,
            depthHeight: depthHeight,
            depthFloatsPerRow: floatsPerRow,
            imageWidth: imgWidth,
            imageHeight: imgHeight,
            yBytesPerRow: yPerRow,
            cbcrBytesPerRow: cbcrPerRow,
            intrinsics: intrinsics,
            imageResolution: resolution,
            step: step
        )
        processedPoints.reserveCapacity(table.samples.count)

        for sample in table.samples {
            let depth = depthPointer[sample.depthIndex]
            guard depth > 0.1 && depth < 5.0 else { continue }

            let pointCamera = simd_float4(
                sample.cameraXFactor * depth,
                sample.cameraYFactor * depth,
                -depth,
                1.0
            )

            let pointWorld = simd_mul(transform, pointCamera)
            let point3D = simd_float3(pointWorld.x, pointWorld.y, pointWorld.z)
            let color = Self.sampleColor(for: sample, yPlane: yPlane, cbcrPlane: cbcrPlane)

            processedPoints.append(ColoredPoint(position: point3D, color: color))
        }
        
        return processedPoints
    }
    
    /// Generates a dense point cloud from a fused metric depth map (from Depth Anything + LiDAR).
    /// - Parameters:
    ///   - frame: The current ARFrame providing camera intrinsics and color image.
    ///   - transform: Camera transform to world space.
    ///   - depthMap: Dense metric depth aligned to the camera image (Float32, size from DA V2).
    /// - Returns: ColoredPoints in world space using RGB sampled from the camera frame.
    func processPointCloudEnhanced(frame: ARFrame, transform: simd_float4x4, depthMap: RelativeDepthMap) -> [ColoredPoint] {
        processPointCloudEnhanced(
            cameraImage: frame.capturedImage,
            intrinsics: frame.camera.intrinsics,
            imageResolution: frame.camera.imageResolution,
            transform: transform,
            depthMap: depthMap
        )
    }

    func processPointCloudEnhanced(
        cameraImage pixelBuffer: CVPixelBuffer,
        intrinsics: simd_float3x3,
        imageResolution resolution: CGSize,
        transform: simd_float4x4,
        depthMap: RelativeDepthMap
    ) -> [ColoredPoint] {
        var processed: [ColoredPoint] = []

        CVPixelBufferLockBaseAddress(pixelBuffer, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(pixelBuffer, .readOnly) }

        let imgWidth = CVPixelBufferGetWidth(pixelBuffer)
        let imgHeight = CVPixelBufferGetHeight(pixelBuffer)

        let yPlane = CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 0)?.assumingMemoryBound(to: UInt8.self)
        let cbcrPlane = CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 1)?.assumingMemoryBound(to: UInt8.self)
        let yPerRow = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 0)
        let cbcrPerRow = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 1)

        let depthW = depthMap.width
        let depthH = depthMap.height

        let step = 6 // Denser than LiDAR-only since DA V2 is high-resolution
        let table = Self.projectionTable(
            depthWidth: depthW,
            depthHeight: depthH,
            imageWidth: imgWidth,
            imageHeight: imgHeight,
            yBytesPerRow: yPerRow,
            cbcrBytesPerRow: cbcrPerRow,
            intrinsics: intrinsics,
            imageResolution: resolution,
            step: step
        )
        processed.reserveCapacity(table.samples.count)

        depthMap.withReadAccess { depthReader in
            for sample in table.samples {
                let depth = depthReader.value(atX: sample.depthX, y: sample.depthY)
                guard depth.isFinite, depth > 0.1 && depth < 8.0 else { continue }

                let pointCamera = simd_float4(
                    sample.cameraXFactor * depth,
                    sample.cameraYFactor * depth,
                    -depth,
                    1.0
                )
                let pointWorld = simd_mul(transform, pointCamera)
                let point3D = simd_float3(pointWorld.x, pointWorld.y, pointWorld.z)
                let color = Self.sampleColor(for: sample, yPlane: yPlane, cbcrPlane: cbcrPlane)

                processed.append(ColoredPoint(position: point3D, color: color))
            }
        }

        return processed
    }

    /// Generates colored points from Depth Anything relative depth calibrated into metric scale.
    /// LiDAR is not sampled per output point; any LiDAR contribution is limited to the calibration.
    func processDepthAnythingPointCloud(
        cameraImage pixelBuffer: CVPixelBuffer,
        intrinsics: simd_float3x3,
        imageResolution resolution: CGSize,
        transform: simd_float4x4,
        relativeDepthMap: RelativeDepthMap,
        calibration: DepthAnythingProcessor.MaximumLikelihoodCalibration
    ) -> [ColoredPoint] {
        var processed: [ColoredPoint] = []

        CVPixelBufferLockBaseAddress(pixelBuffer, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(pixelBuffer, .readOnly) }

        let imgWidth = CVPixelBufferGetWidth(pixelBuffer)
        let imgHeight = CVPixelBufferGetHeight(pixelBuffer)

        let yPlane = CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 0)?.assumingMemoryBound(to: UInt8.self)
        let cbcrPlane = CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 1)?.assumingMemoryBound(to: UInt8.self)
        let yPerRow = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 0)
        let cbcrPerRow = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 1)

        let depthWidth = relativeDepthMap.width
        let depthHeight = relativeDepthMap.height

        let step = 6
        let table = Self.projectionTable(
            depthWidth: depthWidth,
            depthHeight: depthHeight,
            imageWidth: imgWidth,
            imageHeight: imgHeight,
            yBytesPerRow: yPerRow,
            cbcrBytesPerRow: cbcrPerRow,
            intrinsics: intrinsics,
            imageResolution: resolution,
            step: step
        )
        processed.reserveCapacity(table.samples.count)

        relativeDepthMap.withReadAccess { relativeReader in
            for sample in table.samples {
                let relativeDepth = relativeReader.value(atX: sample.depthX, y: sample.depthY)
                let depth = calibration.scale * relativeDepth + calibration.offset
                guard depth.isFinite, depth > 0.1 && depth < 8.0 else { continue }

                let pointCamera = simd_float4(
                    sample.cameraXFactor * depth,
                    sample.cameraYFactor * depth,
                    -depth,
                    1.0
                )
                let pointWorld = simd_mul(transform, pointCamera)
                let point3D = simd_float3(pointWorld.x, pointWorld.y, pointWorld.z)
                let color = Self.sampleColor(for: sample, yPlane: yPlane, cbcrPlane: cbcrPlane)

                processed.append(ColoredPoint(position: point3D, color: color))
            }
        }

        return processed
    }

    /// Generates a camera-frame point cloud from raw Depth Anything relative depth.
    /// The coordinates are intentionally not metric; pair this with
    /// `/mapping/depth_anything/calibration` to reconstruct calibrated depth.
    func processRelativeDepthAnythingPointCloud(
        cameraImage pixelBuffer: CVPixelBuffer,
        intrinsics: simd_float3x3,
        imageResolution resolution: CGSize,
        relativeDepthMap: RelativeDepthMap
    ) -> [ColoredPoint] {
        var processed: [ColoredPoint] = []

        CVPixelBufferLockBaseAddress(pixelBuffer, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(pixelBuffer, .readOnly) }

        let imgWidth = CVPixelBufferGetWidth(pixelBuffer)
        let imgHeight = CVPixelBufferGetHeight(pixelBuffer)

        let yPlane = CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 0)?.assumingMemoryBound(to: UInt8.self)
        let cbcrPlane = CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 1)?.assumingMemoryBound(to: UInt8.self)
        let yPerRow = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 0)
        let cbcrPerRow = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 1)

        let depthWidth = relativeDepthMap.width
        let depthHeight = relativeDepthMap.height

        let step = 6
        let table = Self.projectionTable(
            depthWidth: depthWidth,
            depthHeight: depthHeight,
            imageWidth: imgWidth,
            imageHeight: imgHeight,
            yBytesPerRow: yPerRow,
            cbcrBytesPerRow: cbcrPerRow,
            intrinsics: intrinsics,
            imageResolution: resolution,
            step: step
        )
        processed.reserveCapacity(table.samples.count)

        relativeDepthMap.withReadAccess { relativeReader in
            for sample in table.samples {
                let relativeDepth = relativeReader.value(atX: sample.depthX, y: sample.depthY)
                guard relativeDepth.isFinite,
                      relativeDepth > 0.000_001,
                      relativeDepth < 10_000 else {
                    continue
                }

                let point = SIMD3<Float>(
                    sample.cameraXFactor * relativeDepth,
                    sample.cameraYFactor * relativeDepth,
                    -relativeDepth
                )
                let color = Self.sampleColor(for: sample, yPlane: yPlane, cbcrPlane: cbcrPlane)
                processed.append(ColoredPoint(position: point, color: color))
            }
        }

        return processed
    }

    /// Generates colored points by fusing Depth Anything relative depth with LiDAR
    /// directly at each sampled output point, avoiding a full intermediate fused map.
    func processFusedPointCloud(
        cameraImage pixelBuffer: CVPixelBuffer,
        intrinsics: simd_float3x3,
        imageResolution resolution: CGSize,
        transform: simd_float4x4,
        relativeDepthMap: RelativeDepthMap,
        lidarDepthMap: CVPixelBuffer,
        lidarConfidenceMap: CVPixelBuffer? = nil,
        calibration cachedCalibration: DepthAnythingProcessor.MaximumLikelihoodCalibration? = nil
    ) -> [ColoredPoint] {
        let calibration: DepthAnythingProcessor.MaximumLikelihoodCalibration
        if let cachedCalibration {
            calibration = cachedCalibration
        } else {
            guard let computedCalibration = DepthAnythingProcessor.maximumLikelihoodCalibration(
                relative: relativeDepthMap,
                lidarDepthMap: lidarDepthMap,
                lidarConfidenceMap: lidarConfidenceMap
            ) else { return [] }
            calibration = computedCalibration
        }

        var processed: [ColoredPoint] = []

        CVPixelBufferLockBaseAddress(lidarDepthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(lidarDepthMap, .readOnly) }

        let lidarWidth = CVPixelBufferGetWidth(lidarDepthMap)
        let lidarHeight = CVPixelBufferGetHeight(lidarDepthMap)
        guard CVPixelBufferGetPixelFormatType(lidarDepthMap) == kCVPixelFormatType_DepthFloat32,
              let lidarBase = CVPixelBufferGetBaseAddress(lidarDepthMap)?.assumingMemoryBound(to: Float32.self)
        else { return [] }

        let lidarFloatsPerRow = CVPixelBufferGetBytesPerRow(lidarDepthMap) / MemoryLayout<Float32>.stride

        CVPixelBufferLockBaseAddress(pixelBuffer, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(pixelBuffer, .readOnly) }

        let imgWidth = CVPixelBufferGetWidth(pixelBuffer)
        let imgHeight = CVPixelBufferGetHeight(pixelBuffer)

        let yPlane = CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 0)?.assumingMemoryBound(to: UInt8.self)
        let cbcrPlane = CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 1)?.assumingMemoryBound(to: UInt8.self)
        let yPerRow = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 0)
        let cbcrPerRow = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 1)

        let depthWidth = relativeDepthMap.width
        let depthHeight = relativeDepthMap.height

        let step = 6
        let table = Self.projectionTable(
            depthWidth: depthWidth,
            depthHeight: depthHeight,
            imageWidth: imgWidth,
            imageHeight: imgHeight,
            yBytesPerRow: yPerRow,
            cbcrBytesPerRow: cbcrPerRow,
            lidarWidth: lidarWidth,
            lidarHeight: lidarHeight,
            lidarFloatsPerRow: lidarFloatsPerRow,
            intrinsics: intrinsics,
            imageResolution: resolution,
            step: step
        )
        processed.reserveCapacity(table.samples.count)

        relativeDepthMap.withReadAccess { relativeReader in
            for sample in table.samples {
                let lidarDepth = lidarBase[sample.lidarIndex]
                let relativeDepth = relativeReader.value(atX: sample.depthX, y: sample.depthY)

                guard let depth = DepthAnythingProcessor.maximumLikelihoodMetricDepth(
                    relativeDepth: relativeDepth,
                    lidarDepth: lidarDepth,
                    calibration: calibration
                ), depth > 0.1, depth < 8.0 else { continue }

                let pointCamera = simd_float4(
                    sample.cameraXFactor * depth,
                    sample.cameraYFactor * depth,
                    -depth,
                    1.0
                )
                let pointWorld = simd_mul(transform, pointCamera)
                let point3D = simd_float3(pointWorld.x, pointWorld.y, pointWorld.z)
                let color = Self.sampleColor(for: sample, yPlane: yPlane, cbcrPlane: cbcrPlane)

                processed.append(ColoredPoint(position: point3D, color: color))
            }
        }

        return processed
    }

    /// Applies a Voxel Grid filter to downsample the point cloud.
    /// - Parameters:
    ///   - points: The input point cloud.
    ///   - voxelSize: The size of the voxel grid.
    /// - Returns: A downsampled point cloud.
    func voxelGridFilter(points: [ColoredPoint], voxelSize: Float) -> [ColoredPoint] {
        var voxelMap: [SIMD3<Int>: ColoredPoint] = [:]
        
        for point in points {
            let voxelIndex = SIMD3<Int>(
                Int(floor(point.position.x / voxelSize)),
                Int(floor(point.position.y / voxelSize)),
                Int(floor(point.position.z / voxelSize))
            )
            if voxelMap[voxelIndex] == nil {
                voxelMap[voxelIndex] = point
            }
        }
        return Array(voxelMap.values)
    }
    
    /// Removes outliers from the point cloud.
    /// - Parameter points: The input point cloud.
    /// - Parameter maxDistance: Distance from origin to filter out early.
    /// - Returns: A point cloud with outliers removed.
    func removeOutliers(points: [ColoredPoint], maxDistance: Float = 20.0) -> [ColoredPoint] {
        // 1. Initial Bounding Box Pass to cut off obvious floating artifacts
        let boundingBoxFiltered = points.filter { simd_length($0.position) < maxDistance }
        
        // 2. Fast Radius Outlier Removal (ROR) using an O(N) Spatial Grid
        let searchRadius: Float = 0.2 // 20cm search radius
        var grid: [SIMD3<Int>: Int] = [:]
        
        // Populate the spatial hash grid
        for point in boundingBoxFiltered {
            let index = SIMD3<Int>(
                Int(floor(point.position.x / searchRadius)),
                Int(floor(point.position.y / searchRadius)),
                Int(floor(point.position.z / searchRadius))
            )
            grid[index, default: 0] += 1
        }
        
        let minNeighbors = 3
        var cleanedPoints: [ColoredPoint] = []
        cleanedPoints.reserveCapacity(boundingBoxFiltered.count)
        
        // Filter out points with too few neighbors in adjacent voxels
        for point in boundingBoxFiltered {
            let index = SIMD3<Int>(
                Int(floor(point.position.x / searchRadius)),
                Int(floor(point.position.y / searchRadius)),
                Int(floor(point.position.z / searchRadius))
            )
            
            var neighborCount = 0
            for dx in -1...1 {
                for dy in -1...1 {
                    for dz in -1...1 {
                        let neighborIndex = SIMD3<Int>(index.x + dx, index.y + dy, index.z + dz)
                        neighborCount += grid[neighborIndex] ?? 0
                    }
                }
            }
            
            // Point itself is included in neighborCount, so > minNeighbors is correct
            if neighborCount > minNeighbors {
                cleanedPoints.append(point)
            }
        }
        
        return cleanedPoints
    }
}

/// Manages the accumulation and processing of point clouds off the main AR thread.
actor PointCloudManager {
    private var voxelMap: [SIMD3<Int>: ColoredPoint] = [:]
    private var voxelSize: Float = 0.05
    private let processor = PointCloudProcessor()
    
    func setVoxelSize(_ size: Float) {
        self.voxelSize = size
    }
    
    func addAndFilter(newPoints: [ColoredPoint]) -> Int {
        for point in newPoints {
            let voxelIndex = SIMD3<Int>(
                Int(floor(point.position.x / voxelSize)),
                Int(floor(point.position.y / voxelSize)),
                Int(floor(point.position.z / voxelSize))
            )
            if voxelMap[voxelIndex] == nil {
                voxelMap[voxelIndex] = point
            }
        }
        return voxelMap.count
    }
    
    func getCleanedPoints(maxDistance: Float = 20.0) -> [ColoredPoint] {
        let points = Array(voxelMap.values)
        return processor.removeOutliers(points: points, maxDistance: maxDistance)
    }
    
    func clear() {
        voxelMap.removeAll()
    }
}

/// Incrementally fuses colored RGB-D samples into a bounded surfel map.
actor ColoredSurfelMap {
    private struct SurfelAccumulator {
        var position: SIMD3<Float>
        var normal: SIMD3<Float>
        var color: SIMD3<Float>
        var radius: Float
        var confidence: Float
        var observationCount: UInt32
        var updatedAt: TimeInterval

        var surfel: ColoredSurfel {
            ColoredSurfel(
                position: position,
                normal: normal,
                color: SIMD3<UInt8>(
                    UInt8(max(0, min(255, color.x.rounded()))),
                    UInt8(max(0, min(255, color.y.rounded()))),
                    UInt8(max(0, min(255, color.z.rounded())))
                ),
                radius: radius,
                confidence: confidence,
                observationCount: observationCount
            )
        }
    }

    private struct RemovalCandidate {
        let key: SIMD3<Int>
        let updatedAt: TimeInterval
    }

    private struct BinaryHeap<Element> {
        private(set) var elements: [Element] = []
        private let hasHigherPriority: (Element, Element) -> Bool

        var count: Int {
            elements.count
        }

        var root: Element? {
            elements.first
        }

        init(hasHigherPriority: @escaping (Element, Element) -> Bool) {
            self.hasHigherPriority = hasHigherPriority
        }

        mutating func insert(_ element: Element) {
            elements.append(element)
            siftUp(from: elements.count - 1)
        }

        mutating func replaceRoot(with element: Element) {
            guard !elements.isEmpty else {
                insert(element)
                return
            }

            elements[0] = element
            siftDown(from: 0)
        }

        private mutating func siftUp(from index: Int) {
            var child = index
            while child > 0 {
                let parent = (child - 1) / 2
                guard hasHigherPriority(elements[child], elements[parent]) else { return }
                elements.swapAt(child, parent)
                child = parent
            }
        }

        private mutating func siftDown(from index: Int) {
            var parent = index

            while true {
                let left = parent * 2 + 1
                let right = left + 1
                var candidate = parent

                if left < elements.count,
                   hasHigherPriority(elements[left], elements[candidate]) {
                    candidate = left
                }

                if right < elements.count,
                   hasHigherPriority(elements[right], elements[candidate]) {
                    candidate = right
                }

                guard candidate != parent else { return }
                elements.swapAt(parent, candidate)
                parent = candidate
            }
        }
    }

    private var surfels: [SIMD3<Int>: SurfelAccumulator] = [:]
    private var voxelSize: Float
    private var maxSurfels: Int
    private let maxFusionWeight: Float = 64

    init(voxelSize: Float = 0.04, maxSurfels: Int = 300_000) {
        self.voxelSize = voxelSize
        self.maxSurfels = maxSurfels
    }

    func configure(voxelSize: Float? = nil, maxSurfels: Int? = nil) {
        if let voxelSize {
            self.voxelSize = max(0.01, voxelSize)
        }
        if let maxSurfels {
            self.maxSurfels = max(1_000, maxSurfels)
            trimIfNeeded()
        }
    }

    func fuse(
        points: [ColoredPoint],
        observerPosition: SIMD3<Float>,
        timestamp: TimeInterval
    ) -> Int {
        guard !points.isEmpty else { return surfels.count }

        for point in points {
            guard point.position.x.isFinite,
                  point.position.y.isFinite,
                  point.position.z.isFinite else {
                continue
            }

            let key = voxelIndex(for: point.position)
            let incomingNormal = estimatedNormal(point: point.position, observerPosition: observerPosition)
            let incomingColor = SIMD3<Float>(
                Float(point.color.x),
                Float(point.color.y),
                Float(point.color.z)
            )

            if var existing = surfels[key] {
                let oldWeight = min(Float(existing.observationCount), maxFusionWeight)
                let newWeight = oldWeight + 1
                existing.position = ((existing.position * oldWeight) + point.position) / newWeight
                existing.color = ((existing.color * oldWeight) + incomingColor) / newWeight
                existing.normal = normalized((existing.normal * oldWeight) + incomingNormal)
                existing.radius = min(max(existing.radius, voxelSize * 0.75), voxelSize * 2)
                existing.confidence = min(1, existing.confidence + 0.04)
                existing.observationCount = min(UInt32.max, existing.observationCount + 1)
                existing.updatedAt = timestamp
                surfels[key] = existing
            } else {
                surfels[key] = SurfelAccumulator(
                    position: point.position,
                    normal: incomingNormal,
                    color: incomingColor,
                    radius: voxelSize * 0.75,
                    confidence: 0.2,
                    observationCount: 1,
                    updatedAt: timestamp
                )
            }
        }

        trimIfNeeded()
        return surfels.count
    }

    func snapshot(maxCount: Int? = nil) -> [ColoredSurfel] {
        guard let maxCount else {
            return surfels.values.map(\.surfel)
        }

        let limit = max(0, maxCount)
        guard limit > 0 else { return [] }
        guard surfels.count > limit else {
            return surfels.values.map(\.surfel)
        }

        var selected = BinaryHeap<SurfelAccumulator> {
            Self.hasLowerSnapshotPriority($0, than: $1)
        }
        for surfel in surfels.values {
            if selected.count < limit {
                selected.insert(surfel)
            } else if let lowestSelected = selected.root,
                      Self.hasHigherSnapshotPriority(surfel, than: lowestSelected) {
                selected.replaceRoot(with: surfel)
            }
        }

        return selected.elements.map(\.surfel)
    }

    func clear() {
        surfels.removeAll()
    }

    private func voxelIndex(for position: SIMD3<Float>) -> SIMD3<Int> {
        SIMD3<Int>(
            Int(floor(position.x / voxelSize)),
            Int(floor(position.y / voxelSize)),
            Int(floor(position.z / voxelSize))
        )
    }

    private func estimatedNormal(point: SIMD3<Float>, observerPosition: SIMD3<Float>) -> SIMD3<Float> {
        normalized(observerPosition - point)
    }

    private func normalized(_ value: SIMD3<Float>) -> SIMD3<Float> {
        let length = simd_length(value)
        guard length.isFinite, length > 0.000_001 else {
            return SIMD3<Float>(0, 1, 0)
        }
        return value / length
    }

    private static func hasHigherSnapshotPriority(_ lhs: SurfelAccumulator, than rhs: SurfelAccumulator) -> Bool {
        if lhs.confidence == rhs.confidence {
            return lhs.updatedAt > rhs.updatedAt
        }
        return lhs.confidence > rhs.confidence
    }

    private static func hasLowerSnapshotPriority(_ lhs: SurfelAccumulator, than rhs: SurfelAccumulator) -> Bool {
        if lhs.confidence == rhs.confidence {
            return lhs.updatedAt < rhs.updatedAt
        }
        return lhs.confidence < rhs.confidence
    }

    private static func isNewerRemovalCandidate(_ lhs: RemovalCandidate, than rhs: RemovalCandidate) -> Bool {
        lhs.updatedAt > rhs.updatedAt
    }

    private func trimIfNeeded() {
        guard surfels.count > maxSurfels else { return }
        let removeCount = surfels.count - maxSurfels

        var oldest = BinaryHeap<RemovalCandidate> {
            Self.isNewerRemovalCandidate($0, than: $1)
        }
        for (key, surfel) in surfels {
            let candidate = RemovalCandidate(key: key, updatedAt: surfel.updatedAt)
            if oldest.count < removeCount {
                oldest.insert(candidate)
            } else if let newestRemovalCandidate = oldest.root,
                      candidate.updatedAt < newestRemovalCandidate.updatedAt {
                oldest.replaceRoot(with: candidate)
            }
        }

        for candidate in oldest.elements {
            surfels.removeValue(forKey: candidate.key)
        }
    }
}
