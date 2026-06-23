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

struct PointCloudProcessor {
    
    /// Processes raw feature points and scene depth to create a structured point cloud.
    /// - Parameters:
    ///   - frame: The current ARFrame containing raw depth and feature points.
    ///   - transform: The camera transform to convert points to world coordinates.
    /// - Returns: An array of ColoredPoint representing the processed point cloud with RGB data.
    func processPointCloud(frame: ARFrame, transform: simd_float4x4) -> [ColoredPoint] {
        var processedPoints: [ColoredPoint] = []
        
        // Extract dense LiDAR depth map instead of sparse visual features
        guard let sceneDepth = frame.smoothedSceneDepth ?? frame.sceneDepth else { return processedPoints }
        let depthMap = sceneDepth.depthMap
        
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
        
        let pixelBuffer = frame.capturedImage
        CVPixelBufferLockBaseAddress(pixelBuffer, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(pixelBuffer, .readOnly) }
        
        let imgWidth = CVPixelBufferGetWidth(pixelBuffer)
        let imgHeight = CVPixelBufferGetHeight(pixelBuffer)
        
        let yPlane = CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 0)?.assumingMemoryBound(to: UInt8.self)
        let cbcrPlane = CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 1)?.assumingMemoryBound(to: UInt8.self)
        let yPerRow = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 0)
        let cbcrPerRow = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 1)
        
        let intrinsics = frame.camera.intrinsics
        let resolution = frame.camera.imageResolution
        let scaleX = Float(depthWidth) / Float(resolution.width)
        let scaleY = Float(depthHeight) / Float(resolution.height)
        let fx = intrinsics[0][0] * scaleX
        let fy = intrinsics[1][1] * scaleY
        let cx = intrinsics[2][0] * scaleX
        let cy = intrinsics[2][1] * scaleY
        
        let step = 4 // Process every 4th depth pixel for performance
        
        for y in stride(from: 0, to: depthHeight, by: step) {
            for x in stride(from: 0, to: depthWidth, by: step) {
                let depth = depthPointer[y * floatsPerRow + x]
                guard depth > 0.1 && depth < 5.0 else { continue }
                
                // Unproject 2D depth pixel to 3D camera space
                let xCamera = (Float(x) - cx) * depth / fx
                // Camera Y-axis points UP, pixel Y-axis points DOWN. Must invert to prevent upside-down point clouds!
                let yCamera = (cy - Float(y)) * depth / fy
                let pointCamera = simd_float4(xCamera, yCamera, -depth, 1.0)
                
                // Transform to world space
                let pointWorld = simd_mul(transform, pointCamera)
                let point3D = simd_float3(pointWorld.x, pointWorld.y, pointWorld.z)
                
                let imgX = Int((Float(x) / Float(depthWidth)) * Float(imgWidth))
                let imgY = Int((Float(y) / Float(depthHeight)) * Float(imgHeight))
                
                var color = SIMD3<UInt8>(255, 255, 255)
                
                if imgX >= 0 && imgX < imgWidth && imgY >= 0 && imgY < imgHeight, let yP = yPlane, let cbcrP = cbcrPlane {
                    let yIndex = imgY * yPerRow + imgX
                    let uvIndex = (imgY / 2) * cbcrPerRow + (imgX / 2) * 2
                    
                    let yVal = Float(yP[yIndex])
                    let cbVal = Float(cbcrP[uvIndex]) - 128.0
                    let crVal = Float(cbcrP[uvIndex + 1]) - 128.0
                    
                    // YCbCr to RGB conversion
                    let r = yVal + 1.402 * crVal
                    let g = yVal - 0.344136 * cbVal - 0.714136 * crVal
                    let b = yVal + 1.772 * cbVal
                    
                    color = SIMD3<UInt8>(
                        UInt8(max(0, min(255, r))),
                        UInt8(max(0, min(255, g))),
                        UInt8(max(0, min(255, b)))
                    )
                }
                
                processedPoints.append(ColoredPoint(position: point3D, color: color))
            }
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
        var processed: [ColoredPoint] = []

        let pixelBuffer = frame.capturedImage
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

        let intrinsics = frame.camera.intrinsics
        let resolution = frame.camera.imageResolution
        let scaleX = Float(depthW) / Float(resolution.width)
        let scaleY = Float(depthH) / Float(resolution.height)
        let fx = intrinsics[0][0] * scaleX
        let fy = intrinsics[1][1] * scaleY
        let cx = intrinsics[2][0] * scaleX
        let cy = intrinsics[2][1] * scaleY

        let step = 6 // Denser than LiDAR-only since DA V2 is high-resolution
        processed.reserveCapacity((depthW / step) * (depthH / step))

        for y in stride(from: 0, to: depthH, by: step) {
            for x in stride(from: 0, to: depthW, by: step) {
                let depth = depthMap.value(atX: x, y: y)
                guard depth.isFinite, depth > 0.1 && depth < 8.0 else { continue }

                let xCamera = (Float(x) - cx) * depth / fx
                let yCamera = (cy - Float(y)) * depth / fy
                let pointCamera = simd_float4(xCamera, yCamera, -depth, 1.0)
                let pointWorld = simd_mul(transform, pointCamera)
                let point3D = simd_float3(pointWorld.x, pointWorld.y, pointWorld.z)

                let imgX = Int((Float(x) / Float(depthW)) * Float(imgWidth))
                let imgY = Int((Float(y) / Float(depthH)) * Float(imgHeight))

                var color = SIMD3<UInt8>(255, 255, 255)
                if imgX >= 0, imgX < imgWidth, imgY >= 0, imgY < imgHeight,
                   let yP = yPlane, let cbcrP = cbcrPlane {
                    let yIndex = imgY * yPerRow + imgX
                    let uvIndex = (imgY / 2) * cbcrPerRow + (imgX / 2) * 2
                    let yVal = Float(yP[yIndex])
                    let cbVal = Float(cbcrP[uvIndex]) - 128.0
                    let crVal = Float(cbcrP[uvIndex + 1]) - 128.0
                    let r = yVal + 1.402 * crVal
                    let g = yVal - 0.344136 * cbVal - 0.714136 * crVal
                    let b = yVal + 1.772 * cbVal
                    color = SIMD3<UInt8>(
                        UInt8(max(0, min(255, r))),
                        UInt8(max(0, min(255, g))),
                        UInt8(max(0, min(255, b)))
                    )
                }

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
        let values = surfels.values
            .sorted { lhs, rhs in
                if lhs.confidence == rhs.confidence {
                    return lhs.updatedAt > rhs.updatedAt
                }
                return lhs.confidence > rhs.confidence
            }

        if let maxCount, values.count > maxCount {
            return values.prefix(maxCount).map(\.surfel)
        }
        return values.map(\.surfel)
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

    private func trimIfNeeded() {
        guard surfels.count > maxSurfels else { return }
        let removeCount = surfels.count - maxSurfels
        let oldestKeys = surfels
            .sorted { lhs, rhs in lhs.value.updatedAt < rhs.value.updatedAt }
            .prefix(removeCount)
            .map(\.key)
        for key in oldestKeys {
            surfels.removeValue(forKey: key)
        }
    }
}
