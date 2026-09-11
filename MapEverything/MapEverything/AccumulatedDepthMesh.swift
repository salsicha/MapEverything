import Foundation
import simd

nonisolated struct ColoredSceneMesh: Sendable {
    let vertices: [SIMD3<Float>]
    let indices: [UInt32]
    let colors: [SIMD3<UInt8>]

    var isEmpty: Bool { indices.isEmpty }
}

/// Fuses world-space, already-colored Depth Anything triangles while scanning.
/// Camera buffers and individual frame meshes are not retained.
actor AccumulatedDepthMesh {
    struct Statistics: Sendable {
        let vertexCount: Int
        let triangleCount: Int
        let reachedCapacity: Bool
    }

    private struct Vertex {
        var position: SIMD3<Float>
        var color: SIMD3<Float>
        var weight: Float
    }

    private struct Face: Hashable {
        let low: UInt32
        let middle: UInt32
        let high: UInt32

        init(_ a: UInt32, _ b: UInt32, _ c: UInt32) {
            low = min(a, min(b, c))
            high = max(a, max(b, c))
            middle = a + b + c - low - high
        }
    }

    private let voxelSize: Float
    private let maximumVertices: Int
    private let maximumTriangles: Int
    private var vertexIDs: [SIMD3<Int>: UInt32] = [:]
    private var vertices: [Vertex] = []
    private var faces: Set<Face> = []
    private var indices: [UInt32] = []
    private var cachedSnapshot: ColoredSceneMesh?
    private var reachedCapacity = false

    init(voxelSize: Float = 0.05, maximumVertices: Int = 2_000_000, maximumTriangles: Int = 4_000_000) {
        self.voxelSize = voxelSize.isFinite ? max(0.01, voxelSize) : 0.05
        self.maximumVertices = max(0, min(maximumVertices, 2_000_000))
        self.maximumTriangles = max(0, min(maximumTriangles, 4_000_000))
    }

    func integrate(_ mesh: MeshGenerator.DepthAnythingMeshSnapshot, workSession: ScanWorkSession? = nil) -> Statistics {
        // Recheck after the actor hop: tracking can be lost while a completed
        // inference waits to enter the accumulator.
        guard !Task.isCancelled, workSession?.isActive != false else { return statistics }
        // Never fabricate colors from another frame or silently import an
        // uncolored mesh. The producer supplies one camera color per vertex.
        guard mesh.colors.count == mesh.vertices.count else { return statistics }
        var remap = [UInt32?](repeating: nil, count: mesh.vertices.count)
        let keys = mesh.vertices.map(key)
        for offset in stride(from: 0, to: mesh.indices.count - mesh.indices.count % 3, by: 3) {
            let local = (Int(mesh.indices[offset]), Int(mesh.indices[offset + 1]), Int(mesh.indices[offset + 2]))
            guard local.0 < keys.count, local.1 < keys.count, local.2 < keys.count,
                  let a = keys[local.0], let b = keys[local.1], let c = keys[local.2],
                  a != b, b != c, a != c else { continue }
            let area = simd_length_squared(simd_cross(mesh.vertices[local.1] - mesh.vertices[local.0],
                                                      mesh.vertices[local.2] - mesh.vertices[local.0]))
            guard area.isFinite, area > 1e-12 else { continue }
            let newCount = [a, b, c].reduce(0) { $0 + (vertexIDs[$1] == nil ? 1 : 0) }
            guard vertices.count + newCount <= maximumVertices, faces.count < maximumTriangles else {
                reachedCapacity = true
                break
            }
            let ia = fuse(local.0, key: a, mesh: mesh, remap: &remap)
            let ib = fuse(local.1, key: b, mesh: mesh, remap: &remap)
            let ic = fuse(local.2, key: c, mesh: mesh, remap: &remap)
            if faces.insert(Face(ia, ib, ic)).inserted {
                indices.append(contentsOf: [ia, ib, ic])
            }
            cachedSnapshot = nil
        }
        return statistics
    }

    func snapshot() -> ColoredSceneMesh {
        if let cachedSnapshot { return cachedSnapshot }
        let result = ColoredSceneMesh(
            vertices: vertices.map(\.position), indices: indices,
            colors: vertices.map {
                SIMD3<UInt8>(UInt8(clamping: Int($0.color.x.rounded())),
                             UInt8(clamping: Int($0.color.y.rounded())),
                             UInt8(clamping: Int($0.color.z.rounded())))
            }
        )
        cachedSnapshot = result
        return result
    }

    private var statistics: Statistics {
        Statistics(vertexCount: vertices.count, triangleCount: faces.count, reachedCapacity: reachedCapacity)
    }

    private func fuse(_ local: Int, key: SIMD3<Int>, mesh: MeshGenerator.DepthAnythingMeshSnapshot,
                      remap: inout [UInt32?]) -> UInt32 {
        if let id = remap[local] { return id }
        let rgb = mesh.colors[local]
        let color = SIMD3<Float>(Float(rgb.x), Float(rgb.y), Float(rgb.z))
        let id: UInt32
        if let existing = vertexIDs[key] {
            id = existing
            let index = Int(id)
            let weight = min(vertices[index].weight, 7)
            vertices[index].position = (vertices[index].position * weight + mesh.vertices[local]) / (weight + 1)
            vertices[index].color = (vertices[index].color * weight + color) / (weight + 1)
            vertices[index].weight = weight + 1
        } else {
            id = UInt32(vertices.count)
            vertexIDs[key] = id
            vertices.append(Vertex(position: mesh.vertices[local], color: color, weight: 1))
        }
        remap[local] = id
        return id
    }

    private func key(_ position: SIMD3<Float>) -> SIMD3<Int>? {
        guard position.x.isFinite, position.y.isFinite, position.z.isFinite,
              abs(position.x) < 1_000_000, abs(position.y) < 1_000_000, abs(position.z) < 1_000_000 else { return nil }
        return SIMD3<Int>(Int(floor(position.x / voxelSize)), Int(floor(position.y / voxelSize)),
                          Int(floor(position.z / voxelSize)))
    }
}
