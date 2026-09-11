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
    func reusedViewDisplaysNewScan() async throws {
        let controller = ARViewController()
        await controller.accumulateDepthAnythingMeshSnapshot(depthSnapshot(x: 0))
        let firstScene = try #require(await controller.makeStoppedInspectionScene())
        let view = SCNView()
        TopDownSceneView(scene: firstScene).updateScene(in: view)
        let firstCamera = try #require(view.pointOfView)

        controller.isScanning = true
        await controller.accumulateDepthAnythingMeshSnapshot(depthSnapshot(x: 20))
        let secondScene = try #require(await controller.makeStoppedInspectionScene())
        TopDownSceneView(scene: secondScene).updateScene(in: view)

        #expect(view.scene === secondScene)
        #expect(view.pointOfView !== firstCamera)
        #expect(view.pointOfView?.position.x == 20.5)
        #expect(secondScene.rootNode.childNode(withName: "inspection_floor", recursively: false)?.position.x == 20.5)
        #expect(await controller.currentFinalOverlayMeshArtifact()?.vertices.map(\.x).min() == 20)
    }

    @Test("Updating the same preview preserves the user's camera navigation")
    func sameScenePreservesCamera() async throws {
        let controller = ARViewController()
        await controller.accumulateDepthAnythingMeshSnapshot(depthSnapshot(x: 0))
        let scene = try #require(await controller.makeStoppedInspectionScene())
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
    func previewIncludesWholeScan() async throws {
        let controller = ARViewController()
        await controller.accumulateDepthAnythingMeshSnapshot(depthSnapshot(x: 0))
        await controller.accumulateDepthAnythingMeshSnapshot(depthSnapshot(x: 10))
        await controller.accumulateDepthAnythingMeshSnapshot(depthSnapshot(x: 20))

        let scene = try #require(await controller.makeStoppedInspectionScene())
        #expect(scene.rootNode.childNodes.count == 1)
        #expect(scene.rootNode.boundingBox.min.x == 0)
        #expect(scene.rootNode.boundingBox.max.x == 21)

        let artifact = try #require(await controller.currentFinalOverlayMeshArtifact())
        #expect(artifact.source == "accumulated_depth_anything_lidar_calibrated")
        #expect(artifact.vertices.count == 9)
        #expect(artifact.triangleCount == 3)
        #expect(artifact.vertices.map(\.x).min() == 0)
        #expect(artifact.vertices.map(\.x).max() == 21)
    }

    @Test("Repeated depth frames weld together without discarding earlier areas")
    func repeatedFrameDoesNotDuplicateGeometry() async throws {
        let controller = ARViewController()
        await controller.accumulateDepthAnythingMeshSnapshot(depthSnapshot(x: 0))
        await controller.accumulateDepthAnythingMeshSnapshot(depthSnapshot(x: 20))
        await controller.accumulateDepthAnythingMeshSnapshot(depthSnapshot(x: 0))

        let artifact = try #require(await controller.currentFinalOverlayMeshArtifact())
        #expect(artifact.vertices.count == 6)
        #expect(artifact.vertices.map(\.x).min() == 0)
        #expect(artifact.vertices.map(\.x).max() == 21)
    }

    @Test("Starting a new scan clears the accumulated Depth Anything scene")
    func newScanClearsPreview() async {
        let controller = ARViewController()
        await controller.accumulateDepthAnythingMeshSnapshot(depthSnapshot(x: 20))
        controller.isScanning = true
        #expect(await controller.makeStoppedInspectionScene() == nil)
        #expect(await controller.currentFinalOverlayMeshArtifact() == nil)
    }

    @Test("Depth frames accumulate without creating a solid camera overlay")
    func depthAccumulationDoesNotCoverCamera() async throws {
        let controller = ARViewController()
        let view = ARView(frame: .zero, cameraMode: .nonAR, automaticallyConfigureSession: false)
        controller.arView = view
        await controller.accumulateDepthAnythingMeshSnapshot(depthSnapshot(x: 4))
        #expect(view.scene.anchors.isEmpty)
        let scene = try #require(await controller.makeStoppedInspectionScene())
        #expect(scene.rootNode.boundingBox.min.x == 4)
        #expect(scene.rootNode.boundingBox.max.x == 5)
    }

    private func depthSnapshot(x: Float) -> MeshGenerator.DepthAnythingMeshSnapshot {
        let vertices = [SIMD3<Float>(x, 0, 0), SIMD3<Float>(x + 1, 0, 0), SIMD3<Float>(x, 1, 0)]
        var descriptor = MeshDescriptor()
        descriptor.positions = MeshBuffers.Positions(vertices)
        descriptor.primitives = .triangles([0, 1, 2])
        return MeshGenerator.DepthAnythingMeshSnapshot(
            descriptor: descriptor, vertices: vertices, indices: [0, 1, 2],
            colors: Array(repeating: SIMD3<UInt8>(210, 40, 30), count: 3)
        )
    }
}
