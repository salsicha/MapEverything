import Foundation
import SceneKit
import Testing
import simd
@testable import MapEverything

struct SceneMeshColorizerTests {
    private let red = SIMD3<UInt8>(220, 30, 20)
    private let green = SIMD3<UInt8>(20, 210, 40)

    private func mesh(id: UUID = UUID(), x: Float = 0) -> SafeARMesh {
        var transform = matrix_identity_float4x4
        transform.columns.3.x = x
        return SafeARMesh(identifier: id,
                          vertices: [SIMD3<Float>(0, 0, 0), SIMD3<Float>(1, 0, 0), SIMD3<Float>(0, 1, 0)],
                          indices: [0, 1, 2], transform: transform)
    }

    private func points(for mesh: SafeARMesh, color: SIMD3<UInt8>) -> [ColoredPoint] {
        mesh.vertices.map {
            let world = mesh.transform * SIMD4<Float>($0.x, $0.y, $0.z, 1)
            return ColoredPoint(position: SIMD3<Float>(world.x, world.y, world.z), color: color)
        }
    }

    @Test("Colors from earlier camera views survive remeshing and new scan areas")
    func retainedColorsAndRemeshing() async throws {
        let colorizer = SceneMeshColorizer()
        let first = mesh(x: -10)
        let second = mesh(x: 20)
        await colorizer.integrate(points(for: first, color: red))
        await colorizer.integrate(points(for: second, color: green))
        var revised = mesh(id: first.identifier, x: -10)
        revised = SafeARMesh(identifier: revised.identifier,
                             vertices: [SIMD3<Float>(0, 1, 0), SIMD3<Float>(0.01, 0, 0), SIMD3<Float>(1, 0, 0)],
                             indices: [0, 1, 2], transform: revised.transform)
        let colored = await colorizer.colorize([revised, second])
        #expect(colored[0].colors == [red, red, red])
        #expect(colored[1].colors == [green, green, green])
        #expect(colored[0].vertices == revised.vertices)
        #expect(colored[0].revision == revised.revision)
    }

    @Test("Missing samples stay neutral and distant surfaces do not supply colors")
    func missingAndInvalidSamples() async {
        let colorizer = SceneMeshColorizer()
        await colorizer.integrate([
            ColoredPoint(position: SIMD3<Float>(0, 0, 0.2), color: red),
            ColoredPoint(position: SIMD3<Float>(.nan, 0, 0), color: red),
            ColoredPoint(position: SIMD3<Float>(.infinity, 0, 0), color: red)
        ])
        let colored = await colorizer.colorize([mesh()])
        #expect(colored[0].colors == Array(repeating: SceneMeshColorizer.unobservedColor, count: 3))
    }

    @Test("Stable mesh colors improve when new camera coverage arrives")
    func stableMeshRefresh() async {
        let colorizer = SceneMeshColorizer()
        let original = mesh()
        let unobserved = await colorizer.colorize([original])
        await colorizer.integrate(points(for: original, color: red))
        let refreshed = await colorizer.colorize(unobserved)
        #expect(refreshed[0].colors == [red, red, red])
    }

    @Test("The bounded cache retains earlier colors when its capacity is reached")
    func boundedCache() async {
        let colorizer = SceneMeshColorizer(maximumSamples: 3)
        let first = mesh()
        await colorizer.integrate(points(for: first, color: red))
        await colorizer.integrate(points(for: mesh(x: 20), color: green))
        let colored = await colorizer.colorize([first])
        #expect(colored[0].colors == [red, red, red])
    }

    @Test("Stopping finishes pending geometry while preserving already colored chunks")
    func finishPendingColors() async {
        let colorizer = SceneMeshColorizer()
        let first = mesh()
        let second = mesh(x: 20)
        await colorizer.integrate(points(for: first, color: red))
        let complete = await colorizer.colorize([first])
        await colorizer.integrate(points(for: second, color: green))
        let finished = await colorizer.colorize([complete[0], second], refresh: false)
        #expect(finished[0].colors == [red, red, red])
        #expect(finished[1].colors == [green, green, green])
    }

    @Test("A new scan's color cache starts empty")
    func freshScanHasNoOldColors() async {
        let old = SceneMeshColorizer()
        await old.integrate(points(for: mesh(), color: red))
        let fresh = SceneMeshColorizer()
        let colored = await fresh.colorize([mesh()])
        #expect(colored[0].colors == Array(repeating: SceneMeshColorizer.unobservedColor, count: 3))
    }

    @MainActor
    @Test("Full preview and saved mesh carry matching camera colors across distant areas")
    func previewAndExportColors() async throws {
        let controller = ARViewController()
        let colorizer = SceneMeshColorizer()
        let first = mesh()
        let second = mesh(x: 20)
        controller.updateSceneMeshes([first, second])
        await colorizer.integrate(points(for: first, color: red) + points(for: second, color: green))
        controller.applyColoredSceneMeshes(await colorizer.colorize([first, second]))
        let scene = try #require(controller.makeStoppedInspectionScene())
        #expect(scene.rootNode.childNodes.count == 2)
        for node in scene.rootNode.childNodes {
            let source = try #require(node.geometry?.sources(for: .color).first)
            #expect(source.vectorCount == 3)
            let expected = node.boundingBox.min.x < 10 ? red : green
            let actualRed = source.data.withUnsafeBytes {
                $0.loadUnaligned(fromByteOffset: source.dataOffset, as: Float.self)
            }
            #expect(abs(actualRed - Float(expected.x) / 255) < 0.001)
        }
        let artifact = try #require(controller.currentFinalOverlayMeshArtifact())
        #expect(artifact.hasVertexColors)
        for (vertex, color) in zip(artifact.vertices, artifact.colors) {
            #expect(color == (vertex.x < 10 ? red : green))
        }
        #expect(artifact.objString().contains("v 20.000000"))
    }

    @MainActor
    @Test("Late coloring cannot replace refined geometry or populate a new scan")
    func staleResultsAreRejected() async throws {
        let controller = ARViewController()
        let colorizer = SceneMeshColorizer()
        let old = mesh()
        controller.updateSceneMeshes([old])
        await colorizer.integrate(points(for: old, color: red))
        let lateResult = await colorizer.colorize([old])
        controller.updateSceneMeshes([mesh(id: old.identifier, x: 20)])
        controller.applyColoredSceneMeshes(lateResult)
        #expect(controller.currentFinalOverlayMeshArtifact()?.vertices.map(\.x).min() == 20)
        controller.isScanning = true
        controller.applyColoredSceneMeshes(lateResult)
        #expect(controller.makeStoppedInspectionScene() == nil)
    }
}
