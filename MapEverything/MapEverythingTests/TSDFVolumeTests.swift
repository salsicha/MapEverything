import CoreGraphics
import Foundation
import Testing
import simd
@testable import MapEverything

/// Structural checks on the sparse TSDF: fronto-parallel planes must come
/// back flat and single-sheeted, sub-truncation disagreement must fuse to
/// one surface (the whole point of stage 5), observed free space must erode
/// stale geometry, occluded geometry must survive, and a truncated
/// observation must mesh out to its frontier without skinning unobserved
/// space.
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

    @Test("A measurement beyond the integration cap still erodes stale in-cap surface")
    func beyondCapMeasurementErodesInCapSurface() async throws {
        let volume = TSDFVolume(voxelSize: 0.06)
        // A spurious curtain at 3 m (three sightings, full weight), then
        // thirty correcting frames that each measure the ray's true content
        // at 25 m — beyond the 12 m cap. Pre-fix those rays were skipped
        // outright, so no free-space evidence ever reached the curtain and
        // it survived at full weight forever; the beyond-cap measurement
        // proves 0-11.4 m of the ray empty and must erode it.
        await integrate(volume, depth: plane(z: 3), times: 3)
        try #require(await !volume.extractMesh().isEmpty)
        await integrate(volume, depth: plane(z: 25), times: 30)
        let mesh = await volume.extractMesh()
        #expect(!mesh.vertices.contains { abs($0.z + 3) < 0.3 },
                "The 3 m curtain sits on rays whose content was measured past the cap")
        // The clamped carrier is a bound, not a surface: nothing may
        // materialize near the cap (or anywhere else) from those frames.
        #expect(!mesh.vertices.contains { -$0.z > 4 },
                "A beyond-cap measurement must never write surface")
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

    // MARK: - Frontier extraction

    /// A wall tilted in x by 0.1 per meter of depth (x = 1 + 0.1 d) seen
    /// from the origin: depth recedes smoothly along the wall, and a 7 m
    /// `maximumDepth` truncates observation mid-wall — the street far field
    /// in miniature. Rays that meet the wall beyond the cap carry their true
    /// beyond-cap depth, so integrate() writes no SURFACE for them — only
    /// clamped-carrier free-space updates to their in-cap prefix (all
    /// positive, so they can never skin) — and the deepest observed band
    /// ends in cells with one or two surface-unobserved corners (the
    /// occluded band behind, low-weight free space to the side).
    private func recedingWallDepth() -> [Float] {
        var depth = [Float](repeating: .nan, count: width * height)
        for px in 75..<width {
            let d = 100 / (Float(px) - 74)  // ray px meets the wall here
            for py in 0..<height {
                depth[py * width + px] = d
            }
        }
        return depth
    }

    private func recedingWallMesh() async -> TSDFMesh {
        let volume = TSDFVolume(voxelSize: 0.06)
        for _ in 0..<4 {
            _ = await volume.integrate(
                depth: recedingWallDepth(), colors: nil, width: width, height: height,
                intrinsics: intrinsics, imageResolution: resolution,
                transform: matrix_identity_float4x4, maximumDepth: 7
            )
        }
        return await volume.extractMesh()
    }

    /// Vertices actually referenced by triangles — the skinned surface.
    private func skinnedVertices(_ mesh: TSDFMesh) -> [SIMD3<Float>] {
        Set(mesh.indices).map { mesh.vertices[Int($0)] }
    }

    @Test("Frontier cells with unobserved corners still mesh out to the truncated observation edge")
    func frontierReachesTruncatedObservationEdge() async throws {
        let mesh = await recedingWallMesh()
        try #require(!mesh.isEmpty)
        let skinned = skinnedVertices(mesh)
        let reach = skinned.map { -$0.z }.max() ?? 0
        // The deepest observed surface sample sits at 6.67 m (the px 89
        // ray). The old all-8-corners rule stalled at 6.42 m with ZERO
        // skinned vertices beyond 6.45 m — every deeper band cell has one
        // or two never-observed corners (dropped rays to the side, the
        // occluded band behind) — while the >= 6-observed rule reaches
        // 6.51 m with ~345 skinned vertices beyond 6.45 m.
        #expect(reach > 6.47, "frontier band must mesh strictly deeper than the all-8 rule's 6.42 m")
        #expect(skinned.filter { -$0.z > 6.45 }.count >= 100,
                "the recovered frontier is a band of surface, not a stray vertex")
        // The whole deepest 0.4 m of skinned surface stays well populated.
        #expect(skinned.filter { -$0.z > reach - 0.4 }.count >= 200)
    }

    @Test("Unobserved space beyond the frontier grows no surface")
    func unobservedSpaceBeyondFrontierStaysEmpty() async throws {
        let mesh = await recedingWallMesh()
        try #require(!mesh.isEmpty)
        // The wall genuinely continues into the dropped-ray wedge (7.14 m
        // at the first dropped ray, out to ~100 m): none of it may mesh.
        // Checked over ALL vertices, referenced by triangles or not.
        #expect(!mesh.vertices.contains { -$0.z > 6.9 },
                "no vertex beyond the deepest observed band (measured reach 6.51 m)")
        // Every vertex hugs the observed wall plane x = 1 + 0.1 d
        // (measured max deviation 0.028 m): no skin floats in free or
        // unobserved space.
        #expect(mesh.vertices.allSatisfy { abs($0.x - (1 - 0.1 * $0.z)) < 0.3 })
        // And every vertex stays inside the observed-SURFACE ray wedge
        // (columns px >= 89). The wedge px <= 88 now carries beyond-cap
        // free-space evidence, but free-space-only voxels are all-positive
        // and must never form surface.
        #expect(mesh.vertices.allSatisfy { v in
            let d = -v.z
            return d > 0 && 64 + 100 * v.x / d > 87
        })
    }
}
