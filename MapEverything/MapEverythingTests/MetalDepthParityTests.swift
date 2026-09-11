//
//  MetalDepthParityTests.swift
//  MapEverythingTests
//

import Testing
import Foundation
import CoreVideo
import simd
@testable import MapEverything

struct MetalDepthParityTests {
    /// The kernel binds camera planes through CVMetalTextureCache, which
    /// needs an IOSurface-backed, Metal-compatible buffer.
    private func makeMetalCompatibleYCbCrBuffer(width: Int, height: Int) throws -> CVPixelBuffer {
        var buffer: CVPixelBuffer?
        let attributes: [CFString: Any] = [
            kCVPixelBufferMetalCompatibilityKey: true,
            kCVPixelBufferIOSurfacePropertiesKey: [:] as CFDictionary
        ]
        let status = CVPixelBufferCreate(
            kCFAllocatorDefault,
            width,
            height,
            kCVPixelFormatType_420YpCbCr8BiPlanarFullRange,
            attributes as CFDictionary,
            &buffer
        )
        let pixelBuffer = try #require(buffer)
        try #require(status == kCVReturnSuccess)

        CVPixelBufferLockBaseAddress(pixelBuffer, [])
        defer { CVPixelBufferUnlockBaseAddress(pixelBuffer, []) }

        let yBase = try #require(CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 0))
            .assumingMemoryBound(to: UInt8.self)
        let yStride = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 0)
        for y in 0..<height {
            for x in 0..<width {
                yBase[y * yStride + x] = UInt8((x * 3 + y * 7) % 256)
            }
        }

        let cbcrBase = try #require(CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 1))
            .assumingMemoryBound(to: UInt8.self)
        let cbcrStride = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 1)
        for y in 0..<(height / 2) {
            for x in 0..<(width / 2) {
                cbcrBase[y * cbcrStride + x * 2] = UInt8((x * 5 + 40) % 256)
                cbcrBase[y * cbcrStride + x * 2 + 1] = UInt8((y * 11 + 90) % 256)
            }
        }

        return pixelBuffer
    }

    @Test("Metal kernel matches the CPU depth pipeline within float tolerance")
    func testMetalMatchesCPUPath() throws {
        guard let metal = DepthPointCloudMetalProcessor() else {
            // No Metal device in this environment; the CPU fallback is the
            // only live path, so there is nothing to compare.
            return
        }

        // Odd sizes exercise the clamping and half-resolution chroma paths.
        let depthWidth = 37
        let depthHeight = 23
        var depthData = [Float](repeating: 0, count: depthWidth * depthHeight)
        for index in 0..<depthData.count {
            switch index % 7 {
            case 0: depthData[index] = .nan            // rejected: not finite
            case 1: depthData[index] = -0.5            // rejected: non-positive
            case 2: depthData[index] = 0               // rejected: non-positive
            case 3: depthData[index] = 1e-9            // rejected: depth beyond max
            case 4: depthData[index] = 25.0            // rejected: depth below min
            default: depthData[index] = 0.05 + Float(index % 40) * 0.02
            }
        }
        let relativeDepthMap = RelativeDepthMap(width: depthWidth, height: depthHeight, data: depthData)
        let calibration = DepthAnythingProcessor.MaximumLikelihoodCalibration(scale: 0.5, offset: 0.001)

        let imageWidth = 64
        let imageHeight = 48
        let cameraImage = try makeMetalCompatibleYCbCrBuffer(width: imageWidth, height: imageHeight)

        let intrinsics = simd_float3x3(columns: (
            SIMD3<Float>(500, 0, 0),
            SIMD3<Float>(0, 510, 0),
            SIMD3<Float>(310, 245, 1)
        ))
        let resolution = CGSize(width: imageWidth, height: imageHeight)

        let angle: Float = 0.35
        let rotation = simd_float4x4(
            SIMD4<Float>(cos(angle), 0, -sin(angle), 0),
            SIMD4<Float>(0, 1, 0, 0),
            SIMD4<Float>(sin(angle), 0, cos(angle), 0),
            SIMD4<Float>(0.3, -1.2, 2.05, 1)
        )

        let processor = PointCloudProcessor()
        let cpuPoints = processor.processDepthAnythingPointCloudCPU(
            cameraImage: cameraImage,
            intrinsics: intrinsics,
            imageResolution: resolution,
            transform: rotation,
            relativeDepthMap: relativeDepthMap,
            calibration: calibration
        )
        let gpuPoints = try #require(metal.processDepthAnythingPointCloud(
            cameraImage: cameraImage,
            intrinsics: intrinsics,
            imageResolution: resolution,
            transform: rotation,
            relativeDepthMap: relativeDepthMap,
            calibration: calibration
        ))

        // The validity gates must agree exactly, preserving count and order.
        try #require(cpuPoints.count == gpuPoints.count)
        #expect(cpuPoints.count > 100)

        for (cpu, gpu) in zip(cpuPoints, gpuPoints) {
            let tolerance = max(0.001, 0.0001 * simd_length(cpu.position))
            #expect(simd_distance(cpu.position, gpu.position) < tolerance)

            // Unorm texture reads can differ from byte reads by one LSB at
            // truncation boundaries.
            #expect(abs(Int(cpu.color.x) - Int(gpu.color.x)) <= 1)
            #expect(abs(Int(cpu.color.y) - Int(gpu.color.y)) <= 1)
            #expect(abs(Int(cpu.color.z) - Int(gpu.color.z)) <= 1)
        }
    }

    @Test("Metal path rejects buffers it cannot bind, forcing CPU fallback")
    func testMetalRejectsNonPlanarBuffer() throws {
        guard let metal = DepthPointCloudMetalProcessor() else { return }

        var buffer: CVPixelBuffer?
        CVPixelBufferCreate(kCFAllocatorDefault, 16, 16, kCVPixelFormatType_32BGRA, nil, &buffer)
        let bgraBuffer = try #require(buffer)

        let result = metal.processDepthAnythingPointCloud(
            cameraImage: bgraBuffer,
            intrinsics: matrix_identity_float3x3,
            imageResolution: CGSize(width: 16, height: 16),
            transform: matrix_identity_float4x4,
            relativeDepthMap: RelativeDepthMap(width: 8, height: 8, data: [Float](repeating: 0.5, count: 64)),
            calibration: DepthAnythingProcessor.MaximumLikelihoodCalibration(scale: 0.5, offset: 0.15)
        )
        #expect(result == nil)
    }
}
