import CoreVideo
import Foundation
import RealityKit
import SceneKit
import Testing
import simd
@testable import MapEverything

struct AccumulatedDepthMeshTests {
    @Test("A frame queued before tracking loss cannot enter the accumulated scene")
    func cancelledTrackingFrameIsRejected() async {
        let map = AccumulatedDepthMesh()
        let session = ScanWorkSession()
        session.cancel()
        _ = await map.integrate(triangle(color: red), workSession: session)
        #expect(await map.snapshot().isEmpty)
    }

    private let red = SIMD3<UInt8>(220, 30, 20)
    private let green = SIMD3<UInt8>(20, 210, 40)

    private func triangle(x: Float = 0, z: Float = -10, color: SIMD3<UInt8>, indices: [UInt32] = [0, 1, 2]) -> MeshGenerator.DepthAnythingMeshSnapshot {
        let vertices = [SIMD3<Float>(x, 0, z), SIMD3<Float>(x + 1, 0, z), SIMD3<Float>(x, 1, z)]
        var descriptor = MeshDescriptor()
        descriptor.positions = MeshBuffers.Positions(vertices)
        descriptor.primitives = .triangles(indices)
        return .init(descriptor: descriptor, vertices: vertices, indices: indices, colors: Array(repeating: color, count: 3))
    }

    @Test("Distant colored surfaces and earlier areas survive across depth frames")
    func retainsWholeSceneBeyondLiDARRange() async {
        let map = AccumulatedDepthMesh()
        _ = await map.integrate(triangle(color: red))
        _ = await map.integrate(triangle(x: 20, z: -15, color: green))
        let mesh = await map.snapshot()
        #expect(mesh.vertices.count == 6)
        #expect(mesh.indices.count == 6)
        #expect(mesh.vertices.allSatisfy { $0.z <= -10 })
        #expect(mesh.colors == [red, red, red, green, green, green])
    }

    @Test("Overlapping depth frames weld vertices and deduplicate faces regardless of winding")
    func overlapDoesNotGrowMesh() async {
        let map = AccumulatedDepthMesh()
        for _ in 0..<10 {
            _ = await map.integrate(triangle(x: 0.001, color: red))
            _ = await map.integrate(triangle(x: 0.002, color: red, indices: [2, 1, 0]))
        }
        let mesh = await map.snapshot()
        #expect(mesh.vertices.count == 3)
        #expect(mesh.indices.count == 3)
        #expect(mesh.colors == [red, red, red])
    }

    @Test("Missing colors, invalid indices and collapsed faces do not contaminate the map")
    func invalidFramesAreIgnored() async {
        let map = AccumulatedDepthMesh()
        let valid = triangle(color: red)
        let uncolored = MeshGenerator.DepthAnythingMeshSnapshot(
            descriptor: valid.descriptor, vertices: valid.vertices, indices: valid.indices, colors: []
        )
        _ = await map.integrate(uncolored)
        _ = await map.integrate(triangle(color: red, indices: [0, 1, .max]))
        _ = await map.integrate(triangle(color: red, indices: [0, 0, 1]))
        #expect(await map.snapshot().isEmpty)
        _ = await map.integrate(valid)
        #expect(await map.snapshot().indices.count == 3)
    }

    @Test("Capacity bounds preserve existing geometry instead of evicting the beginning of a scan")
    func capacityPreservesEarlierArea() async {
        let map = AccumulatedDepthMesh(maximumVertices: 3, maximumTriangles: 1)
        _ = await map.integrate(triangle(color: red))
        let stats = await map.integrate(triangle(x: 20, color: green))
        #expect(stats.reachedCapacity)
        let mesh = await map.snapshot()
        #expect(mesh.vertices.count == 3)
        #expect(mesh.colors == [red, red, red])
    }

    @Test("Export snapshots remain immutable when later frames update the same area")
    func snapshotIsStable() async {
        let map = AccumulatedDepthMesh()
        _ = await map.integrate(triangle(color: red))
        let first = await map.snapshot()
        _ = await map.integrate(triangle(color: green))
        let second = await map.snapshot()
        #expect(first.colors == [red, red, red])
        #expect(second.colors != first.colors)
        let nextScan = AccumulatedDepthMesh()
        #expect(await nextScan.snapshot().isEmpty)
    }

    @MainActor
    @Test("The final preview and export use accumulated Depth Anything colors")
    func finalPreviewAndExport() async throws {
        let controller = ARViewController()
        await controller.accumulateDepthAnythingMeshSnapshot(triangle(color: red))
        await controller.accumulateDepthAnythingMeshSnapshot(triangle(x: 20, color: green))
        let scene = try #require(await controller.makeStoppedInspectionScene())
        let source = try #require(scene.rootNode.childNodes.first?.geometry?.sources(for: .color).first)
        #expect(source.vectorCount == 6)
        let renderedRed = source.data.withUnsafeBytes {
            $0.loadUnaligned(fromByteOffset: source.dataOffset, as: Float.self)
        }
        #expect(abs(renderedRed - Float(red.x) / 255) < 0.001)
        let artifact = try #require(await controller.currentFinalOverlayMeshArtifact())
        #expect(artifact.source == "accumulated_depth_anything_lidar_calibrated")
        #expect(artifact.colors == [red, red, red, green, green, green])
        #expect(artifact.vertices.allSatisfy { $0.z == -10 })
        controller.isScanning = true
        #expect(await controller.makeStoppedInspectionScene() == nil)
    }

    @Test("Calibrated 10-metre depth frames retain their own camera image colors")
    func farWallKeepsMatchingCaptureColors() async throws {
        let firstImage = try cameraImage(y: 70, cb: 90, cr: 220)
        let secondImage = try cameraImage(y: 160, cb: 180, cr: 80)
        // relative=.5, inverse depth scale=.2 -> metric depth=10 metres.
        let depth = RelativeDepthMap(width: 8, height: 8, data: Array(repeating: 0.5, count: 64))
        let calibration = DepthAnythingProcessor.MaximumLikelihoodCalibration(scale: 0.2, offset: 0)
        let intrinsics = simd_float3x3(columns: (
            SIMD3<Float>(8, 0, 0), SIMD3<Float>(0, 8, 0), SIMD3<Float>(4, 4, 1)
        ))
        func build(_ image: CVPixelBuffer, x: Float) throws -> MeshGenerator.DepthAnythingMeshSnapshot {
            var transform = matrix_identity_float4x4
            transform.columns.3.x = x
            return try #require(MeshGenerator.createDepthAnythingMeshSnapshot(
                from: depth, calibration: calibration, intrinsics: intrinsics,
                imageResolution: CGSize(width: 8, height: 8), transform: transform,
                configuration: .accumulatedScene, cameraImage: image
            ))
        }
        let first = try build(firstImage, x: 0)
        let second = try build(secondImage, x: 30)
        let firstColor = try #require(first.colors.first)
        let secondColor = try #require(second.colors.first)
        #expect(firstColor != secondColor)
        let map = AccumulatedDepthMesh()
        _ = await map.integrate(first)
        _ = await map.integrate(second)
        let mesh = await map.snapshot()
        #expect(!mesh.isEmpty)
        #expect(mesh.vertices.allSatisfy { abs($0.z + 10) < 0.001 })
        for (position, color) in zip(mesh.vertices, mesh.colors) {
            #expect(color == (position.x < 15 ? firstColor : secondColor))
        }
    }

    private func cameraImage(y: UInt8, cb: UInt8, cr: UInt8) throws -> CVPixelBuffer {
        var result: CVPixelBuffer?
        let status = CVPixelBufferCreate(kCFAllocatorDefault, 8, 8,
            kCVPixelFormatType_420YpCbCr8BiPlanarFullRange, nil, &result)
        try #require(status == kCVReturnSuccess)
        let buffer = try #require(result)
        CVPixelBufferLockBaseAddress(buffer, [])
        defer { CVPixelBufferUnlockBaseAddress(buffer, []) }
        let luma = try #require(CVPixelBufferGetBaseAddressOfPlane(buffer, 0)).assumingMemoryBound(to: UInt8.self)
        for row in 0..<8 {
            for col in 0..<8 { luma[row * CVPixelBufferGetBytesPerRowOfPlane(buffer, 0) + col] = y }
        }
        let chroma = try #require(CVPixelBufferGetBaseAddressOfPlane(buffer, 1)).assumingMemoryBound(to: UInt8.self)
        for row in 0..<4 {
            for col in 0..<4 {
                let index = row * CVPixelBufferGetBytesPerRowOfPlane(buffer, 1) + col * 2
                chroma[index] = cb
                chroma[index + 1] = cr
            }
        }
        return buffer
    }
}
