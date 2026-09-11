import Foundation
import simd

/// Scan-local camera samples survive anchor retirement and topology changes.
/// All spatial matching runs on this actor, away from the UI thread.
actor SceneMeshColorizer {
    private struct Sample {
        var position: SIMD3<Float>
        var color: SIMD3<Float>
        var weight: Float
    }

    private let cellSize: Float = 0.06
    private let maximumSamples: Int
    private var samples: [SIMD3<Int>: Sample] = [:]
    nonisolated static let unobservedColor = SIMD3<UInt8>(180, 180, 180)

    init(maximumSamples: Int = 1_000_000) {
        self.maximumSamples = max(0, maximumSamples)
    }

    func integrate(_ points: [ColoredPoint]) {
        for point in points {
            guard let key = key(for: point.position) else { continue }
            let color = SIMD3<Float>(Float(point.color.x), Float(point.color.y), Float(point.color.z))
            if var sample = samples[key] {
                // Bound the weight so later views can improve old observations.
                let weight = min(sample.weight, 7)
                sample.position = (sample.position * weight + point.position) / (weight + 1)
                sample.color = (sample.color * weight + color) / (weight + 1)
                sample.weight = weight + 1
                samples[key] = sample
            } else if samples.count < maximumSamples {
                samples[key] = Sample(position: point.position, color: color, weight: 1)
            }
        }
    }

    func colorize(_ meshes: [SafeARMesh], refresh: Bool = true) -> [SafeARMesh] {
        meshes.map { mesh in
            if !refresh, mesh.colors.count == mesh.vertices.count { return mesh }
            var result = mesh
            result.colors = mesh.vertices.map { vertex in
                let world = mesh.transform * SIMD4<Float>(vertex.x, vertex.y, vertex.z, 1)
                return color(at: SIMD3<Float>(world.x, world.y, world.z))
            }
            return result
        }
    }

    private func color(at position: SIMD3<Float>) -> SIMD3<UInt8> {
        guard let center = key(for: position) else { return Self.unobservedColor }
        // Limit transfer to nearby measured surfaces, rather than projecting
        // camera pixels onto hidden geometry through foreground objects.
        var bestDistance = cellSize * cellSize
        var bestColor: SIMD3<Float>?
        for x in -1...1 {
            for y in -1...1 {
                for z in -1...1 {
                    guard let sample = samples[center &+ SIMD3<Int>(x, y, z)] else { continue }
                    let distance = simd_length_squared(sample.position - position)
                    if distance <= bestDistance {
                        bestDistance = distance
                        bestColor = sample.color
                    }
                }
            }
        }
        guard let color = bestColor else { return Self.unobservedColor }
        return SIMD3<UInt8>(
            UInt8(clamping: Int(color.x.rounded())),
            UInt8(clamping: Int(color.y.rounded())),
            UInt8(clamping: Int(color.z.rounded()))
        )
    }

    private func key(for position: SIMD3<Float>) -> SIMD3<Int>? {
        // Keep conversion and neighbor offsets safe for malformed input.
        guard position.x.isFinite, position.y.isFinite, position.z.isFinite,
              abs(position.x) < 1_000_000, abs(position.y) < 1_000_000,
              abs(position.z) < 1_000_000 else { return nil }
        return SIMD3<Int>(Int(floor(position.x / cellSize)),
                          Int(floor(position.y / cellSize)),
                          Int(floor(position.z / cellSize)))
    }
}
