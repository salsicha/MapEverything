import Foundation
import CoreGraphics
import Testing
import simd
@testable import MapEverything

struct DepthMeshEdgeTests {
    private func mesh(depths: [Float], width: Int = 2, height: Int = 2, step: Int = 1) -> MeshGenerator.DepthAnythingMeshSnapshot? {
        let relative = RelativeDepthMap(width: width, height: height, data: depths.map { 1 / $0 })
        let intrinsics = simd_float3x3(columns: (
            SIMD3<Float>(400, 0, 0), SIMD3<Float>(0, 400, 0),
            SIMD3<Float>(Float(width) / 2, Float(height) / 2, 1)
        ))
        let configuration = MeshGenerator.DepthAnythingMeshConfiguration(
            step: step, minimumDepth: 0.1, maximumDepth: 100,
            maximumDepthDiscontinuity: 0.45, maximumTriangleCount: 1000
        )
        return MeshGenerator.createDepthAnythingMeshSnapshot(
            from: relative, calibration: .init(scale: 1, offset: 0),
            intrinsics: intrinsics, imageResolution: CGSize(width: width, height: height),
            transform: matrix_identity_float4x4, configuration: configuration
        )
    }

    @Test("Adjacent pixels cannot create metre-long spikes at outdoor distances", arguments: [Float(2), 10, 50])
    func rejectsStretchedTriangle(depth: Float) throws {
        // A 15% jump passed the old 18%-of-range rule at every distance.
        let result = try #require(mesh(depths: [depth, depth, depth, depth * 1.15]))
        #expect(result.indices == [0, 2, 1])
    }

    @Test("Flat distant walls remain meshed without imposing a LiDAR range limit", arguments: [Float(2), 10, 50])
    func preservesDistantSurface(depth: Float) throws {
        let result = try #require(mesh(depths: Array(repeating: depth, count: 4)))
        #expect(result.indices.count == 6)
    }

    @Test("Gently sloped surfaces retain both triangles")
    func preservesSlope() throws {
        let result = try #require(mesh(depths: [10, 10.01, 10.01, 10.02]))
        #expect(result.indices.count == 6)
    }

    @Test("Edge limits account for the shorter final cell in a downsampled depth grid")
    func respectsBorderPixelSpacing() throws {
        var depths = Array(repeating: Float(10), count: 8)
        depths[7] = 10.35
        let result = try #require(mesh(depths: depths, width: 4, height: 2, step: 2))
        #expect(result.indices.count == 9)
        #expect(!result.indices.contains(5))
    }

    @Test("Coarser sampling receives a proportionate edge allowance")
    func respectsSamplingStep() throws {
        var depths = Array(repeating: Float(10), count: 9)
        depths[8] = 10.3
        let result = try #require(mesh(depths: depths, width: 3, height: 3, step: 2))
        #expect(result.indices.count == 6)
    }
}
