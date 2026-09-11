import ARKit
import RealityKit
import SceneKit
import SwiftUI
import Testing
import simd
@testable import MapEverything

@MainActor
struct StoppedScenePreviewTests {
    @Test("A reused preview view switches to the second scan and reframes its camera")
    func reusedViewDisplaysNewScan() throws {
        let controller = ARViewController()
        controller.updateSceneMeshes([mesh(x: 0)])
        let firstScene = try #require(controller.makeStoppedInspectionScene())
        let view = SCNView()
        TopDownSceneView(scene: firstScene).updateScene(in: view)
        let firstCamera = try #require(view.pointOfView)

        controller.isScanning = true
        controller.updateSceneMeshes([mesh(x: 20)])
        let secondScene = try #require(controller.makeStoppedInspectionScene())
        TopDownSceneView(scene: secondScene).updateScene(in: view)

        #expect(view.scene === secondScene)
        #expect(view.pointOfView !== firstCamera)
        #expect(view.pointOfView?.position.x == 20.5)
        #expect(secondScene.rootNode.childNode(withName: "inspection_floor", recursively: false)?.position.x == 20.5)
        #expect(controller.currentFinalOverlayMeshArtifact()?.vertices.map(\.x).min() == 20)
    }

    @Test("Updating the same preview preserves the user's camera navigation")
    func sameScenePreservesCamera() throws {
        let controller = ARViewController()
        controller.updateSceneMeshes([mesh(x: 0)])
        let scene = try #require(controller.makeStoppedInspectionScene())
        let view = SCNView()
        let preview = TopDownSceneView(scene: scene)
        preview.updateScene(in: view)
        let camera = try #require(view.pointOfView)
        camera.position.x = 42
        let nodeCount = scene.rootNode.childNodes.count
        preview.updateScene(in: view)
        #expect(view.pointOfView === camera)
        #expect(camera.position.x == 42)
        #expect(scene.rootNode.childNodes.count == nodeCount)
    }

    @Test("Queued preview callbacks cannot replace the newest stopped scene")
    func onlyLatestPreviewCallbackIsApplied() async {
        let state = PreviewState()
        let coordinator = makeCoordinator(state)
        let oldScene = SCNScene()
        let newScene = SCNScene()
        coordinator.didUpdateStoppedInspectionScene(oldScene)
        coordinator.didUpdateStoppedInspectionScene(nil)
        coordinator.didUpdateStoppedInspectionScene(newScene)
        await drainMainQueue()
        #expect(state.scene === newScene)
        #expect(state.assignmentCount == 1)
    }

    @Test("An old stopped preview is discarded if a new scan has started")
    func oldPreviewCannotAppearDuringScan() async {
        let state = PreviewState()
        let coordinator = makeCoordinator(state)
        coordinator.didUpdateStoppedInspectionScene(SCNScene())
        state.isScanning = true
        await drainMainQueue()
        #expect(state.scene == nil)
        #expect(state.assignmentCount == 0)
    }

    private final class PreviewState {
        var isScanning = false
        var scene: SCNScene?
        var assignmentCount = 0
    }

    private func makeCoordinator(_ state: PreviewState) -> ARViewContainer.Coordinator {
        ARViewContainer.Coordinator(ARViewContainer(
            visualizationMode: .constant(.solidMesh),
            isScanning: Binding(get: { state.isScanning }, set: { state.isScanning = $0 }),
            stoppedInspectionScene: Binding(
                get: { state.scene },
                set: { state.scene = $0; state.assignmentCount += 1 }
            ),
            isPreparingMapper: .constant(false), isDepthAnythingReady: .constant(true),
            pointCount: .constant(0), trackingFeedback: .constant(""), errorMessage: .constant(nil),
            maxPointLimit: 2_000_000, voxelSize: 0.05, boundingBoxSize: 20
        ))
    }

    private func drainMainQueue() async {
        await withCheckedContinuation { continuation in
            DispatchQueue.main.async { continuation.resume() }
        }
    }

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
