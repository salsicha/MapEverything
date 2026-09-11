import ARKit
import RealityKit
import SceneKit
import Testing
import simd
@testable import MapEverything

@MainActor
struct StoppedScenePreviewTests {
    @Test("Stopped preview and mesh export include earlier areas, not just the last depth frame")
    func previewIncludesWholeScan() throws {
        let controller = ARViewController()
        controller.updateSceneMeshes([mesh(x: 0)])
        controller.updateSceneMeshes([mesh(x: 10)])
        controller.updateSceneMeshes([mesh(x: 20)])
        controller.updateDepthAnythingMeshSnapshot(with: depthSnapshot(x: 100))

        let scene = try #require(controller.makeStoppedInspectionScene())
        #expect(scene.rootNode.childNodes.count == 3)
        #expect(scene.rootNode.boundingBox.min.x == 0)
        #expect(scene.rootNode.boundingBox.max.x == 21)

        let artifact = try #require(controller.currentFinalOverlayMeshArtifact())
        #expect(artifact.source == "arkit_scene_reconstruction")
        #expect(artifact.vertices.count == 9)
        #expect(artifact.triangleCount == 3)
        #expect(artifact.vertices.map(\.x).min() == 0)
        #expect(artifact.vertices.map(\.x).max() == 21)
    }

    @Test("Updated anchors replace old geometry without discarding other scan areas")
    func updatedAnchorReplacesGeometry() throws {
        let controller = ARViewController()
        let firstID = UUID()
        controller.updateSceneMeshes([mesh(id: firstID, x: 0), mesh(x: 20)])
        controller.updateSceneMeshes([mesh(id: firstID, x: 5)])

        let artifact = try #require(controller.currentFinalOverlayMeshArtifact())
        #expect(artifact.vertices.count == 6)
        #expect(artifact.vertices.map(\.x).min() == 5)
        #expect(artifact.vertices.map(\.x).max() == 21)
    }

    @Test("Starting a new scan clears both accumulated geometry and the depth fallback")
    func newScanClearsPreview() {
        let controller = ARViewController()
        controller.updateSceneMeshes([mesh(x: 20)])
        controller.updateDepthAnythingMeshSnapshot(with: depthSnapshot(x: 100))
        controller.isScanning = true
        #expect(controller.makeStoppedInspectionScene() == nil)
        #expect(controller.currentFinalOverlayMeshArtifact() == nil)
    }

    @Test("Depth snapshots remain available as a fallback without creating a camera overlay")
    func depthFallbackDoesNotCoverCamera() throws {
        let controller = ARViewController()
        let view = ARView(frame: .zero, cameraMode: .nonAR, automaticallyConfigureSession: false)
        controller.arView = view
        controller.updateDepthAnythingMeshSnapshot(with: depthSnapshot(x: 4))
        #expect(view.scene.anchors.isEmpty)
        let scene = try #require(controller.makeStoppedInspectionScene())
        #expect(scene.rootNode.boundingBox.min.x == 4)
        #expect(scene.rootNode.boundingBox.max.x == 5)
    }

    private func mesh(id: UUID = UUID(), x: Float) -> SafeARMesh {
        var transform = matrix_identity_float4x4
        transform.columns.3.x = x
        return SafeARMesh(
            identifier: id,
            vertices: [SIMD3<Float>(0, 0, 0), SIMD3<Float>(1, 0, 0), SIMD3<Float>(0, 1, 0)],
            indices: [0, 1, 2], transform: transform
        )
    }

    private func depthSnapshot(x: Float) -> MeshGenerator.DepthAnythingMeshSnapshot {
        let vertices = [SIMD3<Float>(x, 0, 0), SIMD3<Float>(x + 1, 0, 0), SIMD3<Float>(x, 1, 0)]
        var descriptor = MeshDescriptor()
        descriptor.positions = MeshBuffers.Positions(vertices)
        descriptor.primitives = .triangles([0, 1, 2])
        return MeshGenerator.DepthAnythingMeshSnapshot(
            descriptor: descriptor, vertices: vertices, indices: [0, 1, 2], colors: []
        )
    }
}
