//
//  ARViewContainer.swift
//  MapEverything
//
//  Created by Alex Moran on 5/8/26.
//

import SwiftUI
import RealityKit
import ARKit
import SceneKit

struct MeshRebuildThrottle {
    let minimumInterval: TimeInterval
    private var lastScheduledAt: [UUID: TimeInterval] = [:]
    private var activeTokens: [UUID: UUID] = [:]

    init(minimumInterval: TimeInterval) {
        self.minimumInterval = minimumInterval
    }

    mutating func begin(anchorID: UUID, now: TimeInterval, force: Bool = false) -> UUID? {
        if !force {
            guard activeTokens[anchorID] == nil else { return nil }
            if let lastScheduled = lastScheduledAt[anchorID],
               now - lastScheduled < minimumInterval {
                return nil
            }
        }

        let token = UUID()
        activeTokens[anchorID] = token
        lastScheduledAt[anchorID] = now
        return token
    }

    mutating func finish(anchorID: UUID, token: UUID) -> Bool {
        guard activeTokens[anchorID] == token else { return false }
        activeTokens.removeValue(forKey: anchorID)
        return true
    }

    mutating func removeAnchor(_ anchorID: UUID) {
        activeTokens.removeValue(forKey: anchorID)
        lastScheduledAt.removeValue(forKey: anchorID)
    }

    mutating func removeAll() {
        activeTokens.removeAll()
        lastScheduledAt.removeAll()
    }
}

struct ARViewContainer: UIViewControllerRepresentable {
    @Binding var visualizationMode: VisualizationMode
    @Binding var isScanning: Bool
    @Binding var stoppedInspectionScene: SCNScene?
    @Binding var isPreparingMapper: Bool
    @Binding var isDepthAnythingReady: Bool
    @Binding var pointCount: Int
    @Binding var trackingFeedback: String
    @Binding var errorMessage: String?
    var maxPointLimit: Int
    var voxelSize: Float
    var boundingBoxSize: Float
    
    func makeCoordinator() -> Coordinator {
        Coordinator(self)
    }
    
    func makeUIViewController(context: Context) -> ARViewController {
        let vc = ARViewController()
        vc.delegate = context.coordinator
        return vc
    }
    
    func updateUIViewController(_ uiViewController: ARViewController, context: Context) {
        // Vital: Update the coordinator's parent to the newest SwiftUI struct to prevent UI update failures
        context.coordinator.parent = self
        
        uiViewController.updateVisualizationMode(visualizationMode)
        uiViewController.isScanning = isScanning
        uiViewController.maxPointLimit = maxPointLimit
        uiViewController.voxelSize = voxelSize
        uiViewController.boundingBoxSize = boundingBoxSize
    }
    
    class Coordinator: NSObject, ARViewControllerDelegate {
        var parent: ARViewContainer
        
        init(_ parent: ARViewContainer) {
            self.parent = parent
        }
        
        func didUpdatePointCount(_ count: Int) {
            DispatchQueue.main.async {
                self.parent.pointCount = count
            }
        }
        
        func didUpdateTrackingFeedback(_ text: String) {
            DispatchQueue.main.async {
                self.parent.trackingFeedback = text
            }
        }
        
        func didFailWithError(_ error: Error) {
            DispatchQueue.main.async {
                self.parent.errorMessage = error.localizedDescription
            }
        }
        
        func didReachScanLimit(limit: Int) {
            DispatchQueue.main.async {
                self.parent.isScanning = false
                self.parent.errorMessage = "Scan limit reached (\(limit) points) to prevent device crash. Please save your scan."
            }
        }

        func didUpdateStoppedInspectionScene(_ scene: SCNScene?) {
            DispatchQueue.main.async {
                self.parent.stoppedInspectionScene = scene
            }
        }

        func didUpdateMapperPreparation(isPreparing: Bool, depthAnythingReady: Bool) {
            DispatchQueue.main.async {
                self.parent.isPreparingMapper = isPreparing
                self.parent.isDepthAnythingReady = depthAnythingReady
            }
        }
    }
}

protocol ARViewControllerDelegate: AnyObject {
    func didUpdatePointCount(_ count: Int)
    func didUpdateTrackingFeedback(_ text: String)
    func didFailWithError(_ error: Error)
    func didReachScanLimit(limit: Int)
    func didUpdateStoppedInspectionScene(_ scene: SCNScene?)
    func didUpdateMapperPreparation(isPreparing: Bool, depthAnythingReady: Bool)
}

class ARViewController: UIViewController, ARSessionDelegate {
    private struct SurfelPreviewMesh {
        let descriptor: MeshDescriptor
        let colorAtlas: CGImage
    }

    private struct DepthAnythingMappingFrame {
        let calibratedPoints: [ColoredPoint]
        let relativePoints: [ColoredPoint]
        let calibration: DepthAnythingProcessor.MaximumLikelihoodCalibration
        let relativeDepthSize: CGSize
        let meshSnapshot: MeshGenerator.DepthAnythingMeshSnapshot?
    }

    var arView: ARView? // Using RealityKit's ARView
    
    weak var delegate: ARViewControllerDelegate?
    var isScanning: Bool = false {
        didSet {
            if oldValue != isScanning {
                if isScanning {
                    delegate?.didUpdateStoppedInspectionScene(nil)
                    depthAnythingCalibrationCache.reset()
                    resumeWorldTrackingSession()
                } else {
                    depthAnythingCalibrationCache.reset()
                    freezeCurrentMeshForInspection()
                    cancelMeshUpdateTasks()
                    arView?.session.pause()
                    updateVisualizationMode(currentMode)
                }
            }
        }
    }
    private let pointManager = PointCloudManager()
    private let surfelMap = ColoredSurfelMap()
    private let pointCloudProcessor = PointCloudProcessor()
    private var depthAnythingProcessor: DepthAnythingProcessor?
    private let depthAnythingCalibrationCache = DepthAnythingCalibrationCache()
    private var depthAnythingPreloadTask: Task<Void, Never>?
    private var lastEnhancedFrameTime: TimeInterval = 0
    private let enhancedFrameInterval: TimeInterval = 0.5 // Run Depth Anything at ~2 fps
    private var isRunningEnhancedInference = false

    var currentMode: VisualizationMode = .none
    private var lastPointProcessingTime: TimeInterval = 0
    private var pointProcessingInterval: TimeInterval = 0.2 // Process points up to 5 times a second to avoid starving ARKit capture buffers
    private var lastCameraImagePublishTime: TimeInterval = 0
    private let cameraImagePublishInterval: TimeInterval = 0.5 // Publish camera images at up to 2Hz
    private var isPublishingCameraImage = false
    private var lastPointCloudPublishTime: TimeInterval = 0
    private let pointCloudPublishInterval: TimeInterval = 0.2 // Publish ROS point clouds at up to 5Hz
    private let maxDisplayedSurfels = 12_000
    private let surfelVisualizationInterval: TimeInterval = 0.5
    private var lastSurfelVisualizationTime: TimeInterval = 0
    private var isProcessingFrame = false
    var maxPointLimit: Int = 2_000_000
    var boundingBoxSize: Float = 20.0
    var voxelSize: Float = 0.05 {
        didSet {
            Task { await pointManager.setVoxelSize(voxelSize) }
            Task { await surfelMap.configure(voxelSize: max(0.02, voxelSize * 0.8)) }
        }
    }
    private var meshEntities: [UUID: ModelEntity] = [:]
    private var lastMapPublishTime: TimeInterval = 0
    private let meshSnapshotPublishConfiguration = MeshSnapshotPublishConfiguration.default
    private var anchorEntities: [UUID: AnchorEntity] = [:]
    private var meshUpdateTasks: [UUID: Task<Void, Never>] = [:]
    private var meshRebuildThrottle = MeshRebuildThrottle(minimumInterval: 0.35)
    
    private var liveSurfelAnchor: AnchorEntity?
    private var liveSurfelEntity: ModelEntity?
    private var liveSurfelUpdateTask: Task<Void, Never>?
    private var liveDepthMeshAnchor: AnchorEntity?
    private var liveDepthMeshEntity: ModelEntity?
    private var liveDepthMeshUpdateTask: Task<Void, Never>?
    private var latestDepthAnythingMeshSnapshot: MeshGenerator.DepthAnythingMeshSnapshot?
    private let depthMeshVisualizationInterval: TimeInterval = 0.5
    private var lastDepthMeshVisualizationTime: TimeInterval = 0
    private var coachingOverlay: ARCoachingOverlayView?
    
    override func viewDidLoad() {
        super.viewDidLoad()
        setupViews()
        preloadDepthAnythingIfNeeded()
        
        NotificationCenter.default.addObserver(self, selector: #selector(thermalStateChanged), name: ProcessInfo.thermalStateDidChangeNotification, object: nil)
    }
    
    @objc private func thermalStateChanged() {
        let state = ProcessInfo.processInfo.thermalState
        DispatchQueue.main.async {
            switch state {
            case .nominal, .fair:
                self.pointProcessingInterval = 0.1 // 10Hz
            case .serious:
                self.pointProcessingInterval = 0.25 // 4Hz - Cooldown mode
                self.delegate?.didUpdateTrackingFeedback("Device Heating Up (Throttling scan...)")
            case .critical:
                self.pointProcessingInterval = 0.5 // 2Hz - Emergency mode
                self.delegate?.didUpdateTrackingFeedback("Device Too Hot! Save scan soon.")
            @unknown default:
                break
            }
        }
    }
    
    deinit {
        depthAnythingPreloadTask?.cancel()
        arView?.session.pause()
        NotificationCenter.default.removeObserver(self)
    }
    
    private func setupViews() {
        // Explicitly pause hardware sensors before deallocating views to prevent massive battery drain
        arView?.session.pause()
        
        arView?.removeFromSuperview()
        arView = nil
        liveSurfelUpdateTask?.cancel()
        liveSurfelUpdateTask = nil
        liveSurfelEntity = nil
        liveSurfelAnchor = nil
        liveDepthMeshUpdateTask?.cancel()
        liveDepthMeshUpdateTask = nil
        liveDepthMeshEntity = nil
        liveDepthMeshAnchor = nil
        latestDepthAnythingMeshSnapshot = nil
        coachingOverlay = nil
        
        let av = ARView(frame: .zero, cameraMode: .ar, automaticallyConfigureSession: false)
        self.view.addSubview(av)

        av.translatesAutoresizingMaskIntoConstraints = false
        NSLayoutConstraint.activate([
            av.topAnchor.constraint(equalTo: self.view.topAnchor),
            av.bottomAnchor.constraint(equalTo: self.view.bottomAnchor),
            av.leadingAnchor.constraint(equalTo: self.view.leadingAnchor),
            av.trailingAnchor.constraint(equalTo: self.view.trailingAnchor)
        ])

        av.session.delegate = self

        guard let configuration = makeWorldTrackingConfiguration() else {
            arView = av
            return
        }

        let overlay = ARCoachingOverlayView()
        overlay.autoresizingMask = [.flexibleWidth, .flexibleHeight]
        overlay.session = av.session
        overlay.goal = .anyPlane
        av.addSubview(overlay)
        coachingOverlay = overlay

        arView = av
        if isScanning {
            av.session.run(configuration)
        }
    }

    private func resumeWorldTrackingSession() {
        guard let arView else { return }
        guard let configuration = makeWorldTrackingConfiguration() else { return }
        depthAnythingCalibrationCache.reset()
        clearLiveMeshEntities()
        arView.session.run(configuration, options: [.resetTracking, .removeExistingAnchors])
        updateVisualizationMode(currentMode)
    }

    private func makeWorldTrackingConfiguration() -> ARWorldTrackingConfiguration? {
        guard ARWorldTrackingConfiguration.isSupported else {
            DispatchQueue.main.async {
                self.delegate?.didFailWithError(NSError(domain: "MapEverything", code: 0, userInfo: [NSLocalizedDescriptionKey: "AR World Tracking is not supported on this device."]))
            }
            return nil
        }

        let configuration = ARWorldTrackingConfiguration()
        configuration.worldAlignment = .gravityAndHeading
        configuration.planeDetection = [.horizontal, .vertical]
        configuration.environmentTexturing = .automatic
        if ARWorldTrackingConfiguration.supportsSceneReconstruction(.mesh) {
            configuration.sceneReconstruction = .mesh
        }
        if ARWorldTrackingConfiguration.supportsFrameSemantics(.smoothedSceneDepth) {
            configuration.frameSemantics.insert(.smoothedSceneDepth)
        } else if ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) {
            configuration.frameSemantics.insert(.sceneDepth)
        } else {
            DispatchQueue.main.async {
                self.delegate?.didFailWithError(NSError(domain: "MapEverything", code: 1, userInfo: [NSLocalizedDescriptionKey: "LiDAR sensor not detected. Point cloud scanning requires a LiDAR-equipped device (iPhone/iPad Pro)."]))
            }
        }

        return configuration
    }

    private func preloadDepthAnythingIfNeeded() {
        guard depthAnythingProcessor == nil, depthAnythingPreloadTask == nil else {
            delegate?.didUpdateMapperPreparation(
                isPreparing: depthAnythingPreloadTask != nil,
                depthAnythingReady: depthAnythingProcessor != nil
            )
            return
        }

        delegate?.didUpdateMapperPreparation(isPreparing: true, depthAnythingReady: false)
        depthAnythingPreloadTask = Task.detached(priority: .utility) { [weak self] in
            try? await Task.sleep(nanoseconds: 350_000_000)
            guard !Task.isCancelled else { return }

            let processor = DepthAnythingProcessor()
            processor?.warmUp()

            await MainActor.run { [weak self] in
                guard let self else { return }
                self.depthAnythingProcessor = processor
                self.depthAnythingPreloadTask = nil
                self.delegate?.didUpdateMapperPreparation(
                    isPreparing: false,
                    depthAnythingReady: processor != nil
                )
            }
        }
    }

    private func clearLiveMeshEntities() {
        cancelMeshUpdateTasks()
        anchorEntities.values.forEach { arView?.scene.removeAnchor($0) }
        meshEntities.removeAll()
        anchorEntities.removeAll()
        clearDepthAnythingMeshEntity()
    }

    private func clearDepthAnythingMeshEntity() {
        liveDepthMeshUpdateTask?.cancel()
        liveDepthMeshUpdateTask = nil
        liveDepthMeshEntity?.removeFromParent()
        liveDepthMeshEntity = nil
        if let liveDepthMeshAnchor {
            arView?.scene.removeAnchor(liveDepthMeshAnchor)
        }
        liveDepthMeshAnchor = nil
        latestDepthAnythingMeshSnapshot = nil
    }

    private func freezeCurrentMeshForInspection() {
        if let latestDepthAnythingMeshSnapshot,
           let scene = makeInspectionScene(from: latestDepthAnythingMeshSnapshot) {
            delegate?.didUpdateStoppedInspectionScene(scene)
            return
        }

        guard let anchors = arView?.session.currentFrame?.anchors.compactMap({ $0 as? ARMeshAnchor }),
              !anchors.isEmpty else {
            delegate?.didUpdateStoppedInspectionScene(nil)
            return
        }

        let safeMeshes = MeshGenerator.extractSafeMeshes(from: anchors)
        delegate?.didUpdateStoppedInspectionScene(makeInspectionScene(from: safeMeshes))
    }

    private func makeInspectionScene(from safeMeshes: [SafeARMesh]) -> SCNScene? {
        let scene = SCNScene()
        var hasGeometry = false

        for mesh in safeMeshes {
            guard !mesh.vertices.isEmpty, mesh.indices.count >= 3 else { continue }

            let worldVertices = mesh.vertices.map { vertex in
                let transformed = simd_mul(mesh.transform, SIMD4<Float>(vertex.x, vertex.y, vertex.z, 1))
                return SCNVector3(transformed.x, transformed.y, transformed.z)
            }

            scene.rootNode.addChildNode(SCNNode(
                geometry: makeLitInspectionGeometry(
                    vertices: worldVertices,
                    indices: mesh.indices,
                    tint: .systemCyan
                )
            ))
            hasGeometry = true
        }

        return hasGeometry ? scene : nil
    }

    private func makeInspectionScene(from snapshot: MeshGenerator.DepthAnythingMeshSnapshot) -> SCNScene? {
        guard !snapshot.vertices.isEmpty, snapshot.indices.count >= 3 else { return nil }

        let scene = SCNScene()
        let worldVertices = snapshot.vertices.map { SCNVector3($0.x, $0.y, $0.z) }
        scene.rootNode.addChildNode(SCNNode(
            geometry: makeLitInspectionGeometry(
                vertices: worldVertices,
                indices: snapshot.indices,
                tint: .systemTeal
            )
        ))
        return scene
    }

    private func makeLitInspectionGeometry(
        vertices: [SCNVector3],
        indices: [UInt32],
        tint: UIColor
    ) -> SCNGeometry {
        let vertexSource = SCNGeometrySource(vertices: vertices)
        let normalSource = SCNGeometrySource(normals: inspectionNormals(for: vertices, indices: indices))
        let indexData = Data(bytes: indices, count: indices.count * MemoryLayout<UInt32>.size)
        let element = SCNGeometryElement(
            data: indexData,
            primitiveType: .triangles,
            primitiveCount: indices.count / 3,
            bytesPerIndex: MemoryLayout<UInt32>.size
        )
        let geometry = SCNGeometry(sources: [vertexSource, normalSource], elements: [element])
        let material = SCNMaterial()
        material.diffuse.contents = tint.withAlphaComponent(0.9)
        material.ambient.contents = tint.withAlphaComponent(0.24)
        material.specular.contents = UIColor.white.withAlphaComponent(0.36)
        material.emission.contents = tint.withAlphaComponent(0.035)
        material.shininess = 0.42
        material.lightingModel = .blinn
        material.isDoubleSided = true
        geometry.materials = [material]
        return geometry
    }

    private func inspectionNormals(for vertices: [SCNVector3], indices: [UInt32]) -> [SCNVector3] {
        guard !vertices.isEmpty else { return [] }
        var accumulated = Array(repeating: SIMD3<Float>(0, 0, 0), count: vertices.count)

        for triangleStart in stride(from: 0, to: indices.count - 2, by: 3) {
            let i0 = Int(indices[triangleStart])
            let i1 = Int(indices[triangleStart + 1])
            let i2 = Int(indices[triangleStart + 2])
            guard vertices.indices.contains(i0),
                  vertices.indices.contains(i1),
                  vertices.indices.contains(i2) else { continue }

            let v0 = SIMD3<Float>(vertices[i0].x, vertices[i0].y, vertices[i0].z)
            let v1 = SIMD3<Float>(vertices[i1].x, vertices[i1].y, vertices[i1].z)
            let v2 = SIMD3<Float>(vertices[i2].x, vertices[i2].y, vertices[i2].z)
            let normal = simd_cross(v1 - v0, v2 - v0)
            guard simd_length_squared(normal) > 0.000001 else { continue }

            let unitNormal = simd_normalize(normal)
            accumulated[i0] += unitNormal
            accumulated[i1] += unitNormal
            accumulated[i2] += unitNormal
        }

        return accumulated.map { normal in
            let normalized = simd_length_squared(normal) > 0.000001
                ? simd_normalize(normal)
                : SIMD3<Float>(0, 1, 0)
            return SCNVector3(normalized.x, normalized.y, normalized.z)
        }
    }

    private func cancelMeshUpdateTasks() {
        meshUpdateTasks.values.forEach { $0.cancel() }
        meshUpdateTasks.removeAll()
        meshRebuildThrottle.removeAll()
    }

    private func finishMeshRebuild(anchorID: UUID, token: UUID) {
        if meshRebuildThrottle.finish(anchorID: anchorID, token: token) {
            meshUpdateTasks.removeValue(forKey: anchorID)
        }
    }

    var isDepthAnythingModelAvailable: Bool {
        depthAnythingProcessor != nil
    }

    @MainActor
    private func shouldUseDepthAnythingMappingDepth() -> Bool {
        guard depthAnythingProcessor != nil else { return false }
        if ProcessInfo.processInfo.thermalState == .critical { return false }
        return true
    }

    private func processDepthAnythingMappingFrame(
        timestamp: TimeInterval,
        cameraImage: CVPixelBuffer,
        lidarDepthMap: CVPixelBuffer,
        lidarConfidenceMap: CVPixelBuffer?,
        intrinsics: simd_float3x3,
        imageResolution: CGSize,
        transform: simd_float4x4,
        shouldBuildMesh: Bool
    ) async -> DepthAnythingMappingFrame? {
        // Rate-limit so the model only runs at ~enhancedFrameInterval. A skipped
        // frame leaves the current Depth Anything mesh in place instead of falling
        // back to LiDAR-derived visual mapping.
        let canRun: Bool = await MainActor.run {
            guard timestamp - lastEnhancedFrameTime >= enhancedFrameInterval,
                  !isRunningEnhancedInference else { return false }
            lastEnhancedFrameTime = timestamp
            isRunningEnhancedInference = true
            return true
        }
        guard canRun else { return nil }

        defer {
            Task { @MainActor in self.isRunningEnhancedInference = false }
        }

        guard let processor = depthAnythingProcessor else { return nil }

        guard let relative = processor.inferRelativeDepth(from: cameraImage) else { return nil }
        guard let calibration = depthAnythingCalibrationCache.calibration(
            relative: relative,
            lidarDepthMap: lidarDepthMap,
            lidarConfidenceMap: lidarConfidenceMap,
            timestamp: timestamp,
            cameraTransform: transform
        ) else { return nil }

        let calibratedPoints = pointCloudProcessor.processDepthAnythingPointCloud(
            cameraImage: cameraImage,
            intrinsics: intrinsics,
            imageResolution: imageResolution,
            transform: transform,
            relativeDepthMap: relative,
            calibration: calibration
        )
        let relativePoints = pointCloudProcessor.processRelativeDepthAnythingPointCloud(
            cameraImage: cameraImage,
            intrinsics: intrinsics,
            imageResolution: imageResolution,
            relativeDepthMap: relative
        )

        let meshSnapshot = shouldBuildMesh
            ? MeshGenerator.createDepthAnythingMeshSnapshot(
                from: relative,
                calibration: calibration,
                intrinsics: intrinsics,
                imageResolution: imageResolution,
                transform: transform
            )
            : nil

        guard !calibratedPoints.isEmpty || !relativePoints.isEmpty || meshSnapshot != nil else { return nil }
        return DepthAnythingMappingFrame(
            calibratedPoints: calibratedPoints,
            relativePoints: relativePoints,
            calibration: calibration,
            relativeDepthSize: CGSize(width: CGFloat(relative.width), height: CGFloat(relative.height)),
            meshSnapshot: meshSnapshot
        )
    }

    private func updateLiveSurfelVisualization(with surfels: [ColoredSurfel]) {
        liveSurfelUpdateTask?.cancel()

        guard currentMode == .surfels else { return }
        guard !surfels.isEmpty else {
            liveSurfelEntity?.removeFromParent()
            liveSurfelEntity = nil
            return
        }

        let displayedSurfels = Array(surfels.prefix(maxDisplayedSurfels))
        liveSurfelUpdateTask = Task { [weak self] in
            guard let self = self else { return }

            do {
                guard let previewMesh = Self.makeSurfelPreviewMesh(from: displayedSurfels) else { return }
                let resource = try await MeshResource(from: [previewMesh.descriptor])
                let texture = try TextureResource.generate(
                    from: previewMesh.colorAtlas,
                    options: TextureResource.CreateOptions(semantic: .color, mipmapsMode: .none)
                )
                var material = UnlitMaterial()
                material.color = UnlitMaterial.BaseColor(
                    tint: .white,
                    texture: UnlitMaterial.Texture(texture)
                )
                let entity = ModelEntity(mesh: resource, materials: [material])

                guard !Task.isCancelled else { return }

                await MainActor.run {
                    guard !Task.isCancelled else { return }

                    self.liveSurfelEntity?.removeFromParent()

                    let anchor: AnchorEntity
                    if let existingAnchor = self.liveSurfelAnchor {
                        anchor = existingAnchor
                    } else {
                        anchor = AnchorEntity(world: .zero)
                        self.liveSurfelAnchor = anchor
                        self.arView?.scene.addAnchor(anchor)
                    }

                    anchor.addChild(entity)
                    anchor.isEnabled = self.currentMode == .surfels
                    self.liveSurfelEntity = entity
                }
            } catch {
                print("Failed to create surfel preview mesh: \(error)")
            }
        }
    }

    private func updateLiveDepthMeshVisualization(with snapshot: MeshGenerator.DepthAnythingMeshSnapshot?) {
        liveDepthMeshUpdateTask?.cancel()

        guard currentMode == .solidMesh else { return }
        guard let snapshot else {
            liveDepthMeshEntity?.removeFromParent()
            liveDepthMeshEntity = nil
            latestDepthAnythingMeshSnapshot = nil
            return
        }

        latestDepthAnythingMeshSnapshot = snapshot
        let descriptor = snapshot.descriptor
        liveDepthMeshUpdateTask = Task { [weak self] in
            guard let self = self else { return }

            do {
                let meshResource = try await MeshResource(from: [descriptor])
                let material = UnlitMaterial(color: UIColor.systemTeal.withAlphaComponent(0.72))
                let entity = ModelEntity(mesh: meshResource, materials: [material])

                guard !Task.isCancelled else { return }

                await MainActor.run {
                    guard !Task.isCancelled else { return }

                    self.liveDepthMeshEntity?.removeFromParent()

                    let anchor: AnchorEntity
                    if let existingAnchor = self.liveDepthMeshAnchor {
                        anchor = existingAnchor
                    } else {
                        anchor = AnchorEntity(world: .zero)
                        self.liveDepthMeshAnchor = anchor
                        self.arView?.scene.addAnchor(anchor)
                    }

                    anchor.addChild(entity)
                    anchor.isEnabled = self.currentMode == .solidMesh
                    self.liveDepthMeshEntity = entity
                }
            } catch {
                print("Failed to create Depth Anything mesh preview: \(error)")
            }
        }
    }

    private static func makeSurfelPreviewMesh(from surfels: [ColoredSurfel]) -> SurfelPreviewMesh? {
        var positions: [SIMD3<Float>] = []
        var normals: [SIMD3<Float>] = []
        var textureCoordinates: [SIMD2<Float>] = []
        var indices: [UInt32] = []
        let atlasSide = max(1, Int(ceil(sqrt(Double(max(surfels.count, 1))))))
        var atlasPixels = [UInt8](repeating: 0, count: atlasSide * atlasSide * 4)

        positions.reserveCapacity(surfels.count * 3)
        normals.reserveCapacity(surfels.count * 3)
        textureCoordinates.reserveCapacity(surfels.count * 3)
        indices.reserveCapacity(surfels.count * 3)

        for (surfelIndex, surfel) in surfels.enumerated() {
            guard surfel.position.x.isFinite,
                  surfel.position.y.isFinite,
                  surfel.position.z.isFinite else {
                continue
            }

            let normal = normalized(surfel.normal, fallback: SIMD3<Float>(0, 1, 0))
            let helper = abs(simd_dot(normal, SIMD3<Float>(0, 1, 0))) < 0.9
                ? SIMD3<Float>(0, 1, 0)
                : SIMD3<Float>(1, 0, 0)
            let tangent = normalized(simd_cross(helper, normal), fallback: SIMD3<Float>(1, 0, 0))
            let bitangent = normalized(simd_cross(normal, tangent), fallback: SIMD3<Float>(0, 0, 1))
            let radius = min(max(surfel.radius, 0.008), 0.045)
            let base = UInt32(positions.count)
            let atlasX = surfelIndex % atlasSide
            let atlasY = surfelIndex / atlasSide
            let atlasOffset = (atlasY * atlasSide + atlasX) * 4
            atlasPixels[atlasOffset] = surfel.color.x
            atlasPixels[atlasOffset + 1] = surfel.color.y
            atlasPixels[atlasOffset + 2] = surfel.color.z
            atlasPixels[atlasOffset + 3] = 255
            let uv = SIMD2<Float>(
                (Float(atlasX) + 0.5) / Float(atlasSide),
                (Float(atlasY) + 0.5) / Float(atlasSide)
            )

            positions.append(surfel.position + tangent * radius)
            positions.append(surfel.position - tangent * radius * 0.5 + bitangent * radius * 0.86)
            positions.append(surfel.position - tangent * radius * 0.5 - bitangent * radius * 0.86)
            normals.append(contentsOf: [normal, normal, normal])
            textureCoordinates.append(contentsOf: [uv, uv, uv])
            indices.append(contentsOf: [base, base + 1, base + 2])
        }

        guard !positions.isEmpty,
              let colorAtlas = makeSurfelColorAtlas(
                pixels: atlasPixels,
                width: atlasSide,
                height: atlasSide
              ) else {
            return nil
        }

        var descriptor = MeshDescriptor()
        descriptor.positions = MeshBuffers.Positions(positions)
        descriptor.normals = MeshBuffers.Normals(normals)
        descriptor.textureCoordinates = MeshBuffers.TextureCoordinates(textureCoordinates)
        descriptor.primitives = .triangles(indices)
        return SurfelPreviewMesh(descriptor: descriptor, colorAtlas: colorAtlas)
    }

    private static func makeSurfelColorAtlas(pixels: [UInt8], width: Int, height: Int) -> CGImage? {
        let bytesPerPixel = 4
        let bytesPerRow = width * bytesPerPixel
        let bitmapInfo = CGBitmapInfo(rawValue:
            CGImageAlphaInfo.premultipliedLast.rawValue
            | CGBitmapInfo.byteOrder32Big.rawValue
        )

        guard let provider = CGDataProvider(data: Data(pixels) as CFData) else { return nil }
        return CGImage(
            width: width,
            height: height,
            bitsPerComponent: 8,
            bitsPerPixel: bytesPerPixel * 8,
            bytesPerRow: bytesPerRow,
            space: CGColorSpaceCreateDeviceRGB(),
            bitmapInfo: bitmapInfo,
            provider: provider,
            decode: nil,
            shouldInterpolate: false,
            intent: .defaultIntent
        )
    }

    private static func normalized(_ value: SIMD3<Float>, fallback: SIMD3<Float>) -> SIMD3<Float> {
        let length = simd_length(value)
        guard length.isFinite, length > 0.000_001 else { return fallback }
        return value / length
    }
    
    func updateVisualizationMode(_ mode: VisualizationMode) {
        currentMode = mode
        
        arView?.debugOptions.remove([.showFeaturePoints, .showSceneUnderstanding])
        meshEntities.values.forEach { $0.isEnabled = false }
        liveSurfelAnchor?.isEnabled = false
        liveDepthMeshAnchor?.isEnabled = false
        guard isScanning else { return }
        
        switch mode {
        case .solidMesh:
            liveDepthMeshAnchor?.isEnabled = true
        case .surfels:
            liveSurfelAnchor?.isEnabled = true
        case .wireframe:
            arView?.debugOptions.insert(.showSceneUnderstanding)
        case .none:
            break
        }
    }
    
    // MARK: - ARSessionDelegate
    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        guard isScanning else { return }

        MapGeoreferencer.shared.updateMapPose(frame.camera.transform, timestamp: frame.timestamp)
        
        if ROS2BridgeClient.shared.isConnected {
            ROS2BridgeClient.shared.publishPose(frame.camera.transform, timestamp: frame.timestamp)
            ROS2BridgeClient.shared.publishOdometry(frame.camera.transform, timestamp: frame.timestamp)
            ROS2BridgeClient.shared.publishTF(frame.camera.transform, timestamp: frame.timestamp)
        }
        
        // Only accumulate visual mapping data when spatial tracking is highly accurate to prevent garbage data.
        guard case .normal = frame.camera.trackingState else { return }
        
        // Throttle processing to prevent CPU overload and memory churn
        guard frame.timestamp - lastPointProcessingTime > pointProcessingInterval else { return }
        guard !isProcessingFrame else { return }
        
        lastPointProcessingTime = frame.timestamp
        isProcessingFrame = true
        
        let transform = frame.camera.transform
        let timestamp = frame.timestamp
        guard let sceneDepth = frame.smoothedSceneDepth ?? frame.sceneDepth else {
            isProcessingFrame = false
            return
        }
        let cameraImage = frame.capturedImage
        let lidarDepthMap = sceneDepth.depthMap
        let lidarConfidenceMap = sceneDepth.confidenceMap
        let intrinsics = frame.camera.intrinsics
        let imageResolution = frame.camera.imageResolution
        let cameraPosition = SIMD3<Float>(
            transform.columns.3.x,
            transform.columns.3.y,
            transform.columns.3.z
        )
        let topicRegistry = ROS2TopicRegistry.shared
        let shouldUseDepthAnythingDepth = shouldUseDepthAnythingMappingDepth()
        let shouldPublishPointCloud = topicRegistry.isStreamEnabled(.pointCloud)
            && timestamp - lastPointCloudPublishTime >= pointCloudPublishInterval
        let shouldPublishCameraImage = topicRegistry.isStreamEnabled(.camera)
            && timestamp - lastCameraImagePublishTime >= cameraImagePublishInterval
            && !isPublishingCameraImage
        let shouldRefreshSurfelVisualization = currentMode == .surfels
            && timestamp - lastSurfelVisualizationTime >= surfelVisualizationInterval
        let shouldRefreshDepthMeshVisualization = currentMode == .solidMesh
            && timestamp - lastDepthMeshVisualizationTime >= depthMeshVisualizationInterval

        if shouldPublishCameraImage, ROS2BridgeClient.shared.isConnected {
            lastCameraImagePublishTime = timestamp
            isPublishingCameraImage = true
            Task.detached(priority: .utility) { [weak self, cameraImage, intrinsics, imageResolution, timestamp] in
                defer {
                    Task { @MainActor in
                        self?.isPublishingCameraImage = false
                    }
                }
                ROS2BridgeClient.shared.publishImage(
                    pixelBuffer: cameraImage,
                    intrinsics: intrinsics,
                    imageResolution: imageResolution,
                    timestamp: timestamp
                )
            }
        }

        Task.detached(priority: .userInitiated) { [weak self] in
            guard let self = self else { return }

            let mappingFrame: DepthAnythingMappingFrame?
            if shouldUseDepthAnythingDepth {
                mappingFrame = await self.processDepthAnythingMappingFrame(
                    timestamp: timestamp,
                    cameraImage: cameraImage,
                    lidarDepthMap: lidarDepthMap,
                    lidarConfidenceMap: lidarConfidenceMap,
                    intrinsics: intrinsics,
                    imageResolution: imageResolution,
                    transform: transform,
                    shouldBuildMesh: shouldRefreshDepthMeshVisualization
                )
            } else {
                mappingFrame = nil
            }

            let depthAnythingPointCloud = mappingFrame?.relativePoints ?? []
            let newPoints = mappingFrame?.calibratedPoints ?? []
            let lidarPointCloud: [ColoredPoint]
            if shouldPublishPointCloud {
                lidarPointCloud = autoreleasepool {
                    self.pointCloudProcessor.processPointCloud(
                        depthMap: lidarDepthMap,
                        cameraImage: cameraImage,
                        intrinsics: intrinsics,
                        imageResolution: imageResolution,
                        transform: transform
                    )
                }
            } else {
                lidarPointCloud = []
            }

            if shouldPublishPointCloud {
                // Downsample the network payload to a sparse 10cm grid to prevent saturating the Wi-Fi bandwidth.
                let sparseLiDARPoints = self.pointCloudProcessor.voxelGridFilter(points: lidarPointCloud, voxelSize: 0.1)
                let sparseDepthAnythingPoints = self.pointCloudProcessor.voxelGridFilter(points: depthAnythingPointCloud, voxelSize: 0.1)

                if !sparseLiDARPoints.isEmpty {
                    ROS2BridgeClient.shared.publishLiDARPointCloud(sparseLiDARPoints, timestamp: timestamp)
                }
                if !sparseDepthAnythingPoints.isEmpty {
                    ROS2BridgeClient.shared.publishDepthAnythingPointCloud(sparseDepthAnythingPoints, timestamp: timestamp)
                }
                if let mappingFrame {
                    ROS2BridgeClient.shared.publishDepthAnythingCalibration(
                        mappingFrame.calibration,
                        relativeDepthSize: mappingFrame.relativeDepthSize,
                        imageResolution: imageResolution,
                        timestamp: timestamp
                    )
                }
                if !sparseLiDARPoints.isEmpty || !sparseDepthAnythingPoints.isEmpty || mappingFrame != nil {
                    await MainActor.run {
                        self.lastPointCloudPublishTime = timestamp
                    }
                }
            }

            if let meshSnapshot = mappingFrame?.meshSnapshot {
                await MainActor.run {
                    self.lastDepthMeshVisualizationTime = timestamp
                    self.updateLiveDepthMeshVisualization(with: meshSnapshot)
                }
            }

            if !newPoints.isEmpty {
                _ = await self.surfelMap.fuse(
                    points: newPoints,
                    observerPosition: cameraPosition,
                    timestamp: timestamp
                )
                if shouldRefreshSurfelVisualization {
                    let surfelSnapshot = await self.surfelMap.snapshot(maxCount: self.maxDisplayedSurfels)
                    let previewSurfels = Array(surfelSnapshot.prefix(self.maxDisplayedSurfels))
                    await MainActor.run {
                        self.lastSurfelVisualizationTime = timestamp
                        self.updateLiveSurfelVisualization(with: previewSurfels)
                    }
                }
                let count = await self.pointManager.addAndFilter(newPoints: newPoints)
                
                await MainActor.run {
                    self.delegate?.didUpdatePointCount(count)
                    if count >= self.maxPointLimit {
                        self.delegate?.didReachScanLimit(limit: self.maxPointLimit)
                    }
                    self.isProcessingFrame = false
                }
            } else {
                await MainActor.run {
                    self.isProcessingFrame = false
                }
            }
        }
    }
    
    func session(_ session: ARSession, didAdd anchors: [ARAnchor]) {
        if isScanning {
            publishMapToROS2IfNeeded(anchors: anchors)
        }
    }
    
    func session(_ session: ARSession, didUpdate anchors: [ARAnchor]) {
        guard isScanning else { return }
        publishMapToROS2IfNeeded(anchors: anchors)
    }
    
    private func publishMapToROS2IfNeeded(anchors: [ARAnchor]) {
        guard ROS2TopicRegistry.shared.isStreamEnabled(.mesh) else { return }
        
        let currentTime = Date().timeIntervalSince1970
        if currentTime - lastMapPublishTime > meshSnapshotPublishConfiguration.publishInterval {
            lastMapPublishTime = currentTime
            
            let timestamp = ProcessInfo.processInfo.systemUptime
            let meshAnchors = anchors.compactMap { $0 as? ARMeshAnchor }
            let safeMeshes = MeshGenerator.extractSafeMeshes(from: meshAnchors)
            Task.detached(priority: .background) {
                if !safeMeshes.isEmpty {
                    ROS2BridgeClient.shared.publishMap(safeMeshes: safeMeshes, timestamp: timestamp)
                }
            }
        }
    }
    
    func session(_ session: ARSession, didRemove anchors: [ARAnchor]) {
        for anchor in anchors {
            if let meshAnchor = anchor as? ARMeshAnchor {
                meshUpdateTasks[meshAnchor.identifier]?.cancel()
                meshUpdateTasks.removeValue(forKey: meshAnchor.identifier)
                meshRebuildThrottle.removeAnchor(meshAnchor.identifier)
                
                if let anchorEntity = anchorEntities[meshAnchor.identifier] {
                    arView?.scene.removeAnchor(anchorEntity)
                }
                meshEntities.removeValue(forKey: meshAnchor.identifier)
                anchorEntities.removeValue(forKey: meshAnchor.identifier)
            }
        }
    }
    
    func session(_ session: ARSession, cameraDidChangeTrackingState camera: ARCamera) {
        switch camera.trackingState {
        case .notAvailable:
            delegate?.didUpdateTrackingFeedback("Tracking Unavailable")
        case .limited(let reason):
            switch reason {
            case .excessiveMotion:
                delegate?.didUpdateTrackingFeedback("Move Slower")
                let generator = UINotificationFeedbackGenerator()
                generator.notificationOccurred(.warning)
            case .insufficientFeatures:
                delegate?.didUpdateTrackingFeedback("More Light / Features Needed")
            case .initializing, .relocalizing:
                delegate?.didUpdateTrackingFeedback("Calibrating...")
            @unknown default:
                delegate?.didUpdateTrackingFeedback("Tracking Limited")
            }
        case .normal:
            delegate?.didUpdateTrackingFeedback("")
        }
    }
    
    func session(_ session: ARSession, didFailWithError error: Error) {
        print("AR Session Failed: \(error.localizedDescription)")
        delegate?.didFailWithError(error)
    }
    
    func sessionWasInterrupted(_ session: ARSession) {
        // Inform the user that the session has been interrupted, for example, by taking the phone away from the face
        print("AR Session Interrupted")
    }
    
    func sessionInterruptionEnded(_ session: ARSession) {
        print("AR Session Interruption Ended")
        // Resume the session without resetting tracking so the user doesn't lose their scan
        guard isScanning else { return }
        if let configuration = session.configuration {
            session.run(configuration)
        }
    }

}
