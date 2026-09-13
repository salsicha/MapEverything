import CoreGraphics
import Foundation
import Testing
import simd
@testable import MapEverything

/// Structural checks on the sparse TSDF: fronto-parallel planes must come
/// back flat and single-sheeted, sub-truncation disagreement must fuse to
/// one surface (the whole point of stage 5), observed free space must erode
/// stale geometry, and occluded geometry must survive.
struct TSDFVolumeTests {
    private let width = 128
    private let height = 96
    private var intrinsics: simd_float3x3 {
        var matrix = matrix_identity_float3x3
        matrix[0][0] = 100
        matrix[1][1] = 100
        matrix[2][0] = 64
        matrix[2][1] = 48
        return matrix
    }
    private var resolution: CGSize { CGSize(width: width, height: height) }

    private func plane(z: Float) -> [Float] {
        [Float](repeating: z, count: width * height)
    }

    private func colors(_ value: SIMD3<UInt8>) -> [SIMD3<UInt8>] {
        [SIMD3<UInt8>](repeating: value, count: width * height)
    }

    private func integrate(_ volume: TSDFVolume, depth: [Float],
                           color: SIMD3<UInt8> = SIMD3(200, 150, 100),
                           times: Int = 1) async {
        for _ in 0..<times {
            _ = await volume.integrate(
                depth: depth, colors: colors(color), width: width, height: height,
                intrinsics: intrinsics, imageResolution: resolution,
                transform: matrix_identity_float4x4, maximumDepth: 12
            )
        }
    }

    @Test("A fronto-parallel plane reconstructs flat, single-sheeted, and colored")
    func planeReconstructsFlat() async throws {
        let volume = TSDFVolume(voxelSize: 0.06)
        await integrate(volume, depth: plane(z: 3), times: 4)
        let mesh = await volume.extractMesh()
        try #require(!mesh.isEmpty)
        #expect(mesh.vertices.allSatisfy { abs($0.z + 3) < 0.09 })
        // Single sheet: no two vertices on the same lateral position at
        // different depths beyond a voxel.
        let zs = mesh.vertices.map(\.z)
        #expect((zs.max()! - zs.min()!) < 0.13)
        // Winding: triangle normals face the camera (+z side of the plane).
        var facing = 0
        var total = 0
        for offset in stride(from: 0, to: mesh.indices.count, by: 3) {
            let a = mesh.vertices[Int(mesh.indices[offset])]
            let b = mesh.vertices[Int(mesh.indices[offset + 1])]
            let c = mesh.vertices[Int(mesh.indices[offset + 2])]
            let normal = simd_cross(b - a, c - a)
            total += 1
            if normal.z > 0 { facing += 1 }
        }
        #expect(Double(facing) >= Double(total) * 0.95)
        let color = mesh.colors[mesh.colors.count / 2]
        #expect(abs(Int(color.x) - 200) <= 8 && abs(Int(color.y) - 150) <= 8 && abs(Int(color.z) - 100) <= 8)
    }

    @Test("Frames disagreeing within the truncation band fuse into one surface")
    func disagreeingFramesFuseToOneSurface() async throws {
        let volume = TSDFVolume(voxelSize: 0.06)
        for _ in 0..<3 {
            await integrate(volume, depth: plane(z: 3.0))
            await integrate(volume, depth: plane(z: 3.1))
        }
        let mesh = await volume.extractMesh()
        try #require(!mesh.isEmpty)
        // One surface near the weighted middle - never two sheets 10 cm apart.
        let zs = mesh.vertices.map(\.z).sorted()
        #expect(zs.first! > -3.2 && zs.last! < -2.95)
        #expect((zs.last! - zs.first!) < 0.13)
    }

    @Test("Observed free space erodes a stale surface")
    func freeSpaceErodesStaleSurfaces() async throws {
        let volume = TSDFVolume(voxelSize: 0.06)
        await integrate(volume, depth: plane(z: 3), times: 3)
        try #require(await !volume.extractMesh().isEmpty)
        await integrate(volume, depth: plane(z: 6), times: 30)
        let mesh = await volume.extractMesh()
        #expect(!mesh.vertices.contains { abs($0.z + 3) < 0.3 },
                "The 3 m shell sits in space every later frame observed to be empty")
        #expect(mesh.vertices.contains { abs($0.z + 6) < 0.15 })
    }

    @Test("Geometry occluded behind a measured surface survives")
    func occludedSurfaceSurvives() async throws {
        let volume = TSDFVolume(voxelSize: 0.06)
        await integrate(volume, depth: plane(z: 3), times: 3)
        await integrate(volume, depth: plane(z: 1.5), times: 10)
        let mesh = await volume.extractMesh()
        #expect(mesh.vertices.contains { abs($0.z + 3) < 0.12 })
        #expect(mesh.vertices.contains { abs($0.z + 1.5) < 0.12 })
    }

    @Test("The block budget caps allocation without corrupting the surface")
    func capacityCapStopsAllocation() async throws {
        let volume = TSDFVolume(voxelSize: 0.06, maximumBlocks: 64)
        await integrate(volume, depth: plane(z: 3), times: 2)
        let statistics = await volume.statistics
        #expect(statistics.allocatedBlocks <= 64)
        #expect(statistics.reachedCapacity)
        let mesh = await volume.extractMesh()
        #expect(mesh.vertices.allSatisfy { abs($0.z + 3) < 0.09 })
    }

    @Test("Extraction is cached until new integration dirties blocks")
    func extractionIsCached() async {
        let volume = TSDFVolume(voxelSize: 0.06)
        await integrate(volume, depth: plane(z: 3), times: 2)
        let first = await volume.extractMesh()
        let second = await volume.extractMesh()
        #expect(first.vertices.count == second.vertices.count)
        #expect(first.indices.count == second.indices.count)
        await integrate(volume, depth: plane(z: 3), times: 1)
        let third = await volume.extractMesh()
        #expect(abs(third.vertices.count - first.vertices.count) < first.vertices.count / 4)
    }
}
