import Foundation
import simd

/// Surface mesh extracted from the TSDF volume.
nonisolated struct TSDFMesh: Sendable {
    let vertices: [SIMD3<Float>]
    let colors: [SIMD3<UInt8>]
    let indices: [UInt32]

    var isEmpty: Bool { indices.isEmpty }
}

/// Sparse voxel-hashed truncated signed distance field with incremental
/// surface-net extraction (docs/consistent-scene-plan.md, stage 5). Depth
/// frames integrate projectively: voxels inside the truncation band average
/// toward the measured surface, voxels observed well in front of it take
/// free-space updates, and occluded voxels are never touched. Residual
/// sub-truncation disagreement between calibrated frames therefore fuses
/// into ONE surface, and stale geometry in observed free space erodes.
actor TSDFVolume {
    struct Statistics: Sendable {
        let allocatedBlocks: Int
        let reachedCapacity: Bool
    }

    /// 8 bytes, CHISEL-style packing: distance and weight as Float16 plus a
    /// running color average.
    private struct Voxel {
        var sdf: Float16 = 0
        var weight: Float16 = 0
        var red: UInt8 = 0
        var green: UInt8 = 0
        var blue: UInt8 = 0
        var colorWeight: UInt8 = 0
    }

    private final class Block {
        var voxels: [Voxel]
        var dirty = true
        var cachedMesh: TSDFMesh?

        init(voxelCount: Int) {
            voxels = [Voxel](repeating: Voxel(), count: voxelCount)
        }
    }

    static let blockEdge = 8
    /// Truncation distance grows with range like the depth noise does, with
    /// a floor of two voxels so surfaces never alias away.
    static func truncation(depth: Float, voxelSize: Float) -> Float {
        min(0.8, max(2.5 * voxelSize, 0.05 * depth))
    }

    private let voxelSize: Float
    private let maximumBlocks: Int
    private var blocks: [SIMD3<Int32>: Block] = [:]
    private var reachedCapacity = false
    private var cachedFullMesh: TSDFMesh?

    /// ~32k blocks x 4 KB of voxels ≈ 130 MB ceiling at the default 6 cm —
    /// sized for walking street scans at the 18 m integration ceiling.
    init(voxelSize: Float = 0.06, maximumBlocks: Int = 32_000) {
        self.voxelSize = voxelSize.isFinite ? max(0.02, voxelSize) : 0.06
        self.maximumBlocks = max(64, maximumBlocks)
    }

    var statistics: Statistics {
        Statistics(allocatedBlocks: blocks.count, reachedCapacity: reachedCapacity)
    }

    /// Integrates one metric depth image (row-major, NaN or <=0 = invalid)
    /// with optional per-pixel colors sampled from the same frame.
    func integrate(
        depth: [Float],
        colors: [SIMD3<UInt8>]?,
        width: Int,
        height: Int,
        intrinsics: simd_float3x3,
        imageResolution: CGSize,
        transform: simd_float4x4,
        maximumDepth: Float,
        weightScale: Float = 1,
        workSession: ScanWorkSession? = nil
    ) -> Statistics {
        guard !Task.isCancelled, workSession?.isActive != false else { return statistics }
        guard depth.count == width * height, width > 1, height > 1,
              imageResolution.width > 0, imageResolution.height > 0 else { return statistics }
        if let colors { guard colors.count == depth.count else { return statistics } }
        let scaleX = Float(width) / Float(imageResolution.width)
        let scaleY = Float(height) / Float(imageResolution.height)
        let fx = intrinsics[0][0] * scaleX
        let fy = intrinsics[1][1] * scaleY
        let cx = intrinsics[2][0] * scaleX
        let cy = intrinsics[2][1] * scaleY
        guard fx.isFinite, fy.isFinite, abs(fx) > 1e-5, abs(fy) > 1e-5 else { return statistics }
        let worldToCamera = transform.inverse
        guard worldToCamera.columns.0.x.isFinite else { return statistics }

        // Pass 1: allocate blocks along each measurement's truncation band.
        var touched: Set<SIMD3<Int32>> = []
        let origin = SIMD3<Float>(transform.columns.3.x, transform.columns.3.y, transform.columns.3.z)
        let blockSize = voxelSize * Float(Self.blockEdge)
        let sampleStride = 2
        for py in stride(from: 0, to: height, by: sampleStride) {
            for px in stride(from: 0, to: width, by: sampleStride) {
                let z = depth[py * width + px]
                guard z.isFinite, z > 0.05, z <= maximumDepth else { continue }
                let camera = SIMD3<Float>(((Float(px) - cx) / fx) * z, ((cy - Float(py)) / fy) * z, -z)
                let world = transform * SIMD4<Float>(camera.x, camera.y, camera.z, 1)
                let surface = SIMD3<Float>(world.x, world.y, world.z)
                let ray = simd_normalize(surface - origin)
                let tau = Self.truncation(depth: z, voxelSize: voxelSize)
                var offset = -tau
                while offset <= tau {
                    let point = surface + ray * offset
                    touched.insert(blockCoordinate(of: point))
                    offset += blockSize
                }
                touched.insert(blockCoordinate(of: surface))
                touched.insert(blockCoordinate(of: surface + ray * tau))
            }
        }
        for coordinate in touched where blocks[coordinate] == nil {
            guard blocks.count < maximumBlocks else {
                reachedCapacity = true
                break
            }
            blocks[coordinate] = Block(voxelCount: Self.blockEdge * Self.blockEdge * Self.blockEdge)
        }

        // Pass 2: projective update of every voxel in the touched blocks AND
        // every already-allocated block inside this frame's frustum — free
        // space is only observed by revisiting blocks the new surface does
        // not touch, which is what lets stale geometry erode.
        let edge = Self.blockEdge
        var updateSet = touched
        let blockDiagonal = blockSize * 0.87
        for (coordinate, _) in blocks where !updateSet.contains(coordinate) {
            let center = (SIMD3<Float>(coordinate) + SIMD3<Float>(repeating: 0.5)) * blockSize
            let camera = worldToCamera * SIMD4<Float>(center.x, center.y, center.z, 1)
            let depthAtCenter = -camera.z
            guard depthAtCenter > -blockDiagonal, depthAtCenter < maximumDepth + blockDiagonal else { continue }
            let projectionDepth = max(depthAtCenter, 0.05)
            let px = cx + fx * camera.x / projectionDepth
            let py = cy - fy * camera.y / projectionDepth
            let margin = abs(fx) * blockDiagonal / max(projectionDepth, 0.3) + 8
            guard px > -margin, px < Float(width) + margin,
                  py > -margin, py < Float(height) + margin else { continue }
            updateSet.insert(coordinate)
        }
        for coordinate in updateSet {
            guard let block = blocks[coordinate] else { continue }
            var changed = false
            var index = 0
            for vz in 0..<edge {
                for vy in 0..<edge {
                    for vx in 0..<edge {
                        defer { index += 1 }
                        let world = SIMD3<Float>(
                            (Float(coordinate.x) * Float(edge) + Float(vx) + 0.5) * voxelSize,
                            (Float(coordinate.y) * Float(edge) + Float(vy) + 0.5) * voxelSize,
                            (Float(coordinate.z) * Float(edge) + Float(vz) + 0.5) * voxelSize
                        )
                        let camera = worldToCamera * SIMD4<Float>(world.x, world.y, world.z, 1)
                        let voxelDepth = -camera.z
                        guard voxelDepth > 0.05 else { continue }
                        let ix = Int((cx + fx * camera.x / voxelDepth).rounded())
                        let iy = Int((cy - fy * camera.y / voxelDepth).rounded())
                        guard ix >= 0, ix < width, iy >= 0, iy < height else { continue }
                        let measured = depth[iy * width + ix]
                        guard measured.isFinite, measured > 0.05 else { continue }
                        // A measurement beyond the integration cap writes no
                        // surface, but it is still negative evidence for the
                        // capped part of its ray: the content lies PAST the
                        // cap, so every in-cap voxel clear of the CLAMPED
                        // carrier's truncation band is observed free space.
                        // Without this, a spurious in-cap surface on an
                        // open-background ray (street-canyon opening,
                        // skyline) could never erode — each correcting
                        // frame measures beyond the cap and used to skip
                        // the ray entirely, making the mistake permanent.
                        let carrier = min(measured, maximumDepth)
                        let tau = Self.truncation(depth: carrier, voxelSize: voxelSize)
                        let signed = carrier - voxelDepth
                        if signed < -tau { continue }   // occluded: unobserved
                        // The clamped carrier is a bound, not a surface:
                        // voxels inside its band stay unobserved.
                        if measured > maximumDepth, signed <= tau { continue }

                        let observationWeight = min(1, max(0.08, 4 / (carrier * carrier))) * max(0.05, min(1, weightScale))
                        let update: Float
                        let weightGain: Float
                        if signed > tau {
                            // Observed free space: pull gently positive so
                            // stale surfaces here erode.
                            update = tau
                            weightGain = observationWeight * 0.25
                        } else {
                            update = signed
                            weightGain = observationWeight
                        }
                        var voxel = block.voxels[index]
                        let previousWeight = Float(voxel.weight)
                        let total = min(previousWeight + weightGain, 64)
                        let blended = (Float(voxel.sdf) * previousWeight + update * weightGain)
                            / (previousWeight + weightGain)
                        // Clamp only to the global band: re-clamping to
                        // this frame's tau would let one low-weight
                        // observation discard accumulated evidence.
                        voxel.sdf = Float16(max(-0.8, min(0.8, blended)))
                        voxel.weight = Float16(total)
                        if let colors, abs(signed) <= tau * 0.5 {
                            let color = colors[iy * width + ix]
                            let cw = Float(voxel.colorWeight)
                            let gain = min(cw + 1, 32)
                            voxel.red = UInt8(((Float(voxel.red) * cw + Float(color.x)) / (cw + 1)).rounded())
                            voxel.green = UInt8(((Float(voxel.green) * cw + Float(color.y)) / (cw + 1)).rounded())
                            voxel.blue = UInt8(((Float(voxel.blue) * cw + Float(color.z)) / (cw + 1)).rounded())
                            voxel.colorWeight = UInt8(gain)
                        }
                        block.voxels[index] = voxel
                        changed = true
                    }
                }
            }
            if changed {
                block.dirty = true
                block.cachedMesh = nil
                cachedFullMesh = nil
                // Border voxels influence neighbor cells; their meshes must
                // re-extract too.
                for dz in -1...1 { for dy in -1...1 { for dx in -1...1 where dx != 0 || dy != 0 || dz != 0 {
                    let neighbor = coordinate &+ SIMD3<Int32>(Int32(dx), Int32(dy), Int32(dz))
                    blocks[neighbor]?.cachedMesh = nil
                    blocks[neighbor]?.dirty = true
                } } }
            }
        }
        return statistics
    }

    /// Extracts the full surface via naive surface nets, re-meshing only
    /// dirty blocks. One vertex per sign-changing cell (>= 6 of 8 corners
    /// observed) placed at the mean of its observed edge zero-crossings;
    /// quads across every observed sign-changing lattice edge, wound by the
    /// sign, yield a consistent, hole-free surface that reaches the
    /// observation frontier without skinning unobserved space.
    func extractMesh() -> TSDFMesh {
        if let cachedFullMesh { return cachedFullMesh }
        var vertices: [SIMD3<Float>] = []
        var colors: [SIMD3<UInt8>] = []
        var indices: [UInt32] = []
        for (coordinate, block) in blocks {
            if block.cachedMesh == nil {
                block.cachedMesh = extract(block: coordinate)
                block.dirty = false
            }
            guard let mesh = block.cachedMesh, !mesh.isEmpty else { continue }
            let base = UInt32(vertices.count)
            vertices.append(contentsOf: mesh.vertices)
            colors.append(contentsOf: mesh.colors)
            indices.append(contentsOf: mesh.indices.map { $0 + base })
        }
        let result = TSDFMesh(vertices: vertices, colors: colors, indices: indices)
        cachedFullMesh = result
        return result
    }

    // MARK: - Surface nets

    private struct Sample {
        let sdf: Float
        let valid: Bool
        let color: SIMD3<Float>
    }

    private func sample(_ voxel: SIMD3<Int32>) -> Sample {
        let edge = Int32(Self.blockEdge)
        var blockCoordinate = voxel / edge
        if voxel.x < 0 && voxel.x % edge != 0 { blockCoordinate.x -= 1 }
        if voxel.y < 0 && voxel.y % edge != 0 { blockCoordinate.y -= 1 }
        if voxel.z < 0 && voxel.z % edge != 0 { blockCoordinate.z -= 1 }
        guard let block = blocks[blockCoordinate] else {
            return Sample(sdf: 0, valid: false, color: .zero)
        }
        let local = voxel &- blockCoordinate &* edge
        let index = (Int(local.z) * Self.blockEdge + Int(local.y)) * Self.blockEdge + Int(local.x)
        let voxelData = block.voxels[index]
        return Sample(
            sdf: Float(voxelData.sdf),
            // Any genuine surface observation counts (a single far-field
            // sighting carries 0.08); free-space-only voxels stay below
            // this and, being all-positive, can never form a surface.
            valid: Float(voxelData.weight) > 0.06,
            color: SIMD3<Float>(Float(voxelData.red), Float(voxelData.green), Float(voxelData.blue))
        )
    }

    private static let cellCorners: [SIMD3<Int32>] = [
        SIMD3(0, 0, 0), SIMD3(1, 0, 0), SIMD3(0, 1, 0), SIMD3(1, 1, 0),
        SIMD3(0, 0, 1), SIMD3(1, 0, 1), SIMD3(0, 1, 1), SIMD3(1, 1, 1),
    ]
    private static let cellEdges: [(Int, Int)] = [
        (0, 1), (2, 3), (4, 5), (6, 7),
        (0, 2), (1, 3), (4, 6), (5, 7),
        (0, 4), (1, 5), (2, 6), (3, 7),
    ]

    /// A cell's surface-net vertex, or nil when the cell has no valid sign
    /// change. Cell (x,y,z) spans voxels (x..x+1, y..y+1, z..z+1).
    ///
    /// Frontier cells mesh too: up to two never-observed corners are
    /// tolerated so a truncated observation extracts out to its frontier
    /// instead of losing the whole deepest band. Nothing is fabricated for
    /// unobserved corners — crossings come only from edges whose BOTH
    /// endpoints are observed — and with >= 6 observed corners the observed
    /// corners always form a connected subgraph of the cube (Q3 is
    /// 3-connected), so an observed sign change is exactly an observed edge
    /// crossing. Quad emission is unchanged (quads only across observed
    /// sign-changing lattice edges), so no skin ever spans unobserved
    /// space. Pure function of the lattice: neighboring blocks derive
    /// identical vertices at their seams.
    private func cellVertex(_ cell: SIMD3<Int32>) -> (position: SIMD3<Float>, color: SIMD3<UInt8>)? {
        var samples: [Sample] = []
        samples.reserveCapacity(8)
        var observed = 0
        for corner in Self.cellCorners {
            let sampleValue = sample(cell &+ corner)
            if sampleValue.valid { observed += 1 }
            samples.append(sampleValue)
        }
        guard observed >= 6 else { return nil }
        var crossings = 0
        var position = SIMD3<Float>.zero
        var color = SIMD3<Float>.zero
        for (a, b) in Self.cellEdges {
            guard samples[a].valid, samples[b].valid else { continue }
            let sa = samples[a].sdf, sb = samples[b].sdf
            guard (sa < 0) != (sb < 0) else { continue }
            let t = sa / (sa - sb)
            let pa = SIMD3<Float>(Self.cellCorners[a]), pb = SIMD3<Float>(Self.cellCorners[b])
            position += pa + (pb - pa) * t
            color += samples[a].color + (samples[b].color - samples[a].color) * t
            crossings += 1
        }
        guard crossings > 0 else { return nil }
        position /= Float(crossings)
        color /= Float(crossings)
        let world = (SIMD3<Float>(cell) + position + SIMD3<Float>(repeating: 0.5)) * voxelSize
        return (world, SIMD3<UInt8>(UInt8(clamping: Int(color.x)),
                                    UInt8(clamping: Int(color.y)),
                                    UInt8(clamping: Int(color.z))))
    }

    private func extract(block coordinate: SIMD3<Int32>) -> TSDFMesh {
        let edge = Int32(Self.blockEdge)
        let baseVoxel = coordinate &* edge
        var vertices: [SIMD3<Float>] = []
        var colors: [SIMD3<UInt8>] = []
        var indices: [UInt32] = []
        var cellIndex: [SIMD3<Int32>: UInt32] = [:]

        func vertexIndex(_ cell: SIMD3<Int32>) -> UInt32? {
            if let existing = cellIndex[cell] { return existing }
            guard let vertex = cellVertex(cell) else { return nil }
            let index = UInt32(vertices.count)
            vertices.append(vertex.position)
            colors.append(vertex.color)
            cellIndex[cell] = index
            return index
        }

        // Each lattice edge owned by this block that crosses the surface
        // stitches the four surrounding cells into a quad. Owning edges
        // whose base voxel lies in the block keeps blocks seam-consistent.
        let axes: [(direction: SIMD3<Int32>, quad: [SIMD3<Int32>])] = [
            (SIMD3(1, 0, 0), [SIMD3(0, -1, -1), SIMD3(0, 0, -1), SIMD3(0, 0, 0), SIMD3(0, -1, 0)]),
            (SIMD3(0, 1, 0), [SIMD3(-1, 0, -1), SIMD3(-1, 0, 0), SIMD3(0, 0, 0), SIMD3(0, 0, -1)]),
            (SIMD3(0, 0, 1), [SIMD3(-1, -1, 0), SIMD3(0, -1, 0), SIMD3(0, 0, 0), SIMD3(-1, 0, 0)]),
        ]
        for vz in 0..<edge {
            for vy in 0..<edge {
                for vx in 0..<edge {
                    let voxel = baseVoxel &+ SIMD3<Int32>(vx, vy, vz)
                    let here = sample(voxel)
                    guard here.valid else { continue }
                    for (direction, quad) in axes {
                        let neighbor = sample(voxel &+ direction)
                        guard neighbor.valid, (here.sdf < 0) != (neighbor.sdf < 0) else { continue }
                        var quadIndices: [UInt32] = []
                        quadIndices.reserveCapacity(4)
                        var complete = true
                        for offset in quad {
                            guard let index = vertexIndex(voxel &+ offset) else {
                                complete = false
                                break
                            }
                            quadIndices.append(index)
                        }
                        guard complete else { continue }
                        // Wind toward the positive (outside) side.
                        let flip = here.sdf < 0
                        let (a, b, c, d) = (quadIndices[0], quadIndices[1], quadIndices[2], quadIndices[3])
                        if flip {
                            indices.append(contentsOf: [a, b, c, a, c, d])
                        } else {
                            indices.append(contentsOf: [a, c, b, a, d, c])
                        }
                    }
                }
            }
        }
        return TSDFMesh(vertices: vertices, colors: colors, indices: indices)
    }

    private func blockCoordinate(of point: SIMD3<Float>) -> SIMD3<Int32> {
        let blockSize = voxelSize * Float(Self.blockEdge)
        return SIMD3<Int32>(Int32(floor(point.x / blockSize)),
                            Int32(floor(point.y / blockSize)),
                            Int32(floor(point.z / blockSize)))
    }
}
