//
//  ColoredMeshGenerationTests.swift
//  MapEverythingTests
//

import Testing
import Foundation
import CoreVideo
import simd
@testable import MapEverything

struct ColoredMeshGenerationTests {
    private func makeYCbCrBuffer(width: Int, height: Int) throws -> CVPixelBuffer {
        var buffer: CVPixelBuffer?
        let status = CVPixelBufferCreate(
            kCFAllocatorDefault,
            width,
            height,
            kCVPixelFormatType_420YpCbCr8BiPlanarFullRange,
            nil,
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
                yBase[y * yStride + x] = UInt8((x * 9 + y * 5) % 256)
            }
        }

        let cbcrBase = try #require(CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 1))
            .assumingMemoryBound(to: UInt8.self)
        let cbcrStride = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 1)
        for y in 0..<(height / 2) {
            for x in 0..<(width / 2) {
                cbcrBase[y * cbcrStride + x * 2] = UInt8((x * 7 + 60) % 256)
                cbcrBase[y * cbcrStride + x * 2 + 1] = UInt8((y * 13 + 110) % 256)
            }
        }

        return pixelBuffer
    }

    private var testCalibration: DepthAnythingProcessor.MaximumLikelihoodCalibration {
        DepthAnythingProcessor.MaximumLikelihoodCalibration(scale: 0.5, offset: 0.15)
    }

    private var testIntrinsics: simd_float3x3 {
        simd_float3x3(columns: (
            SIMD3<Float>(400, 0, 0),
            SIMD3<Float>(0, 410, 0),
            SIMD3<Float>(150, 120, 1)
        ))
    }

    // Wide bounds so the mesh accepts exactly the calibratedMetricDepth set,
    // matching the point-cloud path one to one.
    private var testConfiguration: MeshGenerator.DepthAnythingMeshConfiguration {
        MeshGenerator.DepthAnythingMeshConfiguration(
            step: 1,
            minimumDepth: DepthAnythingProcessor.minimumCalibratedDepth,
            maximumDepth: DepthAnythingProcessor.maximumCalibratedDepth,
            maximumDepthDiscontinuity: 1_000,
            maximumTriangleCount: 100_000
        )
    }

    private func makeDepthMap(width: Int, height: Int) -> RelativeDepthMap {
        var data = [Float](repeating: 0, count: width * height)
        for index in 0..<data.count {
            // A few invalid holes; the rest smooth so triangles survive the
            // discontinuity gate.
            data[index] = index % 11 == 0 ? .nan : 0.4 + Float(index % 17) * 0.01
        }
        return RelativeDepthMap(width: width, height: height, data: data)
    }

    @Test("Mesh vertices carry one color each, matching the point-cloud colors")
    func testMeshColorsMatchPointCloud() throws {
        let depthMap = makeDepthMap(width: 24, height: 18)
        let cameraImage = try makeYCbCrBuffer(width: 48, height: 36)
        let resolution = CGSize(width: 48, height: 36)
        let transform = matrix_identity_float4x4

        let snapshot = try #require(MeshGenerator.createDepthAnythingMeshSnapshot(
            from: depthMap,
            calibration: testCalibration,
            intrinsics: testIntrinsics,
            imageResolution: resolution,
            transform: transform,
            configuration: testConfiguration,
            cameraImage: cameraImage
        ))

        #expect(snapshot.colors.count == snapshot.vertices.count)
        #expect(snapshot.vertices.count > 50)

        // Same acceptance gates and same ordering as the CPU point cloud, so
        // vertices and points correspond index-for-index.
        let points = PointCloudProcessor().processDepthAnythingPointCloudCPU(
            cameraImage: cameraImage,
            intrinsics: testIntrinsics,
            imageResolution: resolution,
            transform: transform,
            relativeDepthMap: depthMap,
            calibration: testCalibration
        )
        try #require(points.count == snapshot.vertices.count)

        for (point, (vertex, color)) in zip(points, zip(snapshot.vertices, snapshot.colors)) {
            #expect(simd_distance(point.position, vertex) < 1e-5)
            #expect(point.color == color)
        }
    }

    @Test("No camera buffer produces an uncolored but otherwise identical mesh")
    func testNilCameraBufferKeepsMeshUncolored() throws {
        let depthMap = makeDepthMap(width: 24, height: 18)
        let resolution = CGSize(width: 48, height: 36)

        let uncolored = try #require(MeshGenerator.createDepthAnythingMeshSnapshot(
            from: depthMap,
            calibration: testCalibration,
            intrinsics: testIntrinsics,
            imageResolution: resolution,
            transform: matrix_identity_float4x4,
            configuration: testConfiguration
        ))
        let colored = try #require(MeshGenerator.createDepthAnythingMeshSnapshot(
            from: depthMap,
            calibration: testCalibration,
            intrinsics: testIntrinsics,
            imageResolution: resolution,
            transform: matrix_identity_float4x4,
            configuration: testConfiguration,
            cameraImage: try makeYCbCrBuffer(width: 48, height: 36)
        ))

        #expect(uncolored.colors.isEmpty)
        #expect(uncolored.vertices == colored.vertices)
        #expect(uncolored.indices == colored.indices)
    }

    @Test("Non-planar camera buffers are ignored instead of crashing")
    func testNonPlanarBufferYieldsUncoloredMesh() throws {
        var buffer: CVPixelBuffer?
        CVPixelBufferCreate(kCFAllocatorDefault, 32, 32, kCVPixelFormatType_32BGRA, nil, &buffer)
        let bgraBuffer = try #require(buffer)

        let snapshot = try #require(MeshGenerator.createDepthAnythingMeshSnapshot(
            from: makeDepthMap(width: 24, height: 18),
            calibration: testCalibration,
            intrinsics: testIntrinsics,
            imageResolution: CGSize(width: 32, height: 32),
            transform: matrix_identity_float4x4,
            configuration: testConfiguration,
            cameraImage: bgraBuffer
        ))
        #expect(snapshot.colors.isEmpty)
        #expect(snapshot.vertices.count > 3)
    }
}
