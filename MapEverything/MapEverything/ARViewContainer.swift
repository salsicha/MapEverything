//
//  ARViewContainer.swift
//  MapEverything
//
//  Created by Alex Moran on 5/8/26.
//

import SwiftUI
import RealityKit
import ARKit
import RoomPlan

struct ARViewContainer: UIViewControllerRepresentable {
    @Binding var visualizationMode: VisualizationMode
    @Binding var isScanning: Bool
    @Binding var appMode: AppMode
    @Binding var measurementText: String
    @Binding var pointCount: Int
    @Binding var trackingFeedback: String
    @Binding var errorMessage: String?
    @Binding var isProcessing: Bool
    var maxPointLimit: Int
    var voxelSize: Float
    var boundingBoxSize: Float
    var useRoomPlan: Bool
    var useImperialUnits: Bool
    var onSave: ((EnvironmentModel) -> Void)?
    var onFloorplanExported: ((URL) -> Void)?
    @Binding var triggerSaveName: String?
    @Binding var triggerClear: Bool
    @Binding var triggerExportFloorplan: Bool
    @Binding var triggerUndoMeasurement: Bool
    @Binding var environmentToLoad: EnvironmentModel?
    
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
        uiViewController.appMode = appMode
        uiViewController.maxPointLimit = maxPointLimit
        uiViewController.voxelSize = voxelSize
        uiViewController.boundingBoxSize = boundingBoxSize
        uiViewController.useRoomPlan = useRoomPlan
        uiViewController.useImperialUnits = useImperialUnits
        
        if let name = triggerSaveName {
            triggerSaveName = nil
            DispatchQueue.main.async {
                self.isProcessing = true
                uiViewController.saveCurrentScan(name: name) { model in
                    DispatchQueue.main.async {
                        if let model = model {
                            self.onSave?(model)
                            UINotificationFeedbackGenerator().notificationOccurred(.success)
                        } else {
                            UINotificationFeedbackGenerator().notificationOccurred(.error)
                        }
                        self.isProcessing = false
                    }
                }
            }
        }
        
        if triggerClear {
            triggerClear = false
            DispatchQueue.main.async {
                uiViewController.clearScan()
            }
        }

        if triggerUndoMeasurement {
            triggerUndoMeasurement = false
            DispatchQueue.main.async {
                uiViewController.undoLastMeasurement()
            }
        }

        if triggerExportFloorplan {
            triggerExportFloorplan = false
            DispatchQueue.main.async {
                uiViewController.exportFloorplan { url in
                    DispatchQueue.main.async {
                        if let url = url {
                            self.onFloorplanExported?(url)
                        } else {
                            self.errorMessage = "No walls detected yet. Scan the room to detect walls before exporting."
                        }
                    }
                }
            }
        }

        if let env = environmentToLoad {
            environmentToLoad = nil
            DispatchQueue.main.async {
                self.isProcessing = true
                uiViewController.loadEnvironment(env) {
                    DispatchQueue.main.async {
                        self.isProcessing = false
                    }
                }
            }
        }
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
        
        func didUpdateMeasurement(_ text: String) {
            DispatchQueue.main.async {
                self.parent.measurementText = text
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
    }
}

protocol ARViewControllerDelegate: AnyObject {
    func didUpdatePointCount(_ count: Int)
    func didUpdateMeasurement(_ text: String)
    func didUpdateTrackingFeedback(_ text: String)
    func didFailWithError(_ error: Error)
    func didReachScanLimit(limit: Int)
}

class ARViewController: UIViewController, ARSessionDelegate, RoomCaptureViewDelegate, RoomCaptureSessionDelegate {
    var arView: ARView? // Using RealityKit's ARView
    var roomCaptureView: RoomCaptureView?
    var capturedRoom: CapturedRoom?
    
    weak var delegate: ARViewControllerDelegate?
    var isScanning: Bool = false {
        didSet {
            if oldValue != isScanning {
                if useRoomPlan {
                    if isScanning {
                        roomCaptureView?.captureSession.run(configuration: RoomCaptureSession.Configuration())
                    } else {
                        roomCaptureView?.captureSession.stop()
                    }
                }
            }
        }
    }
    var appMode: AppMode = .scan
    
    private let pointManager = PointCloudManager()
    private let pointCloudProcessor = PointCloudProcessor()
    private let adaptiveMappingController = AdaptiveMappingModeController.shared
    private lazy var depthAnythingProcessor: DepthAnythingProcessor? = DepthAnythingProcessor()
    private var lastEnhancedFrameTime: TimeInterval = 0
    private let enhancedFrameInterval: TimeInterval = 0.5 // Run Depth Anything at ~2 fps
    private var isRunningEnhancedInference = false

    var currentMode: VisualizationMode = .none
    private var lastPointProcessingTime: TimeInterval = 0
    private var pointProcessingInterval: TimeInterval = 0.1 // Process points 10 times a second
    private var lastPointCloudPublishTime: TimeInterval = 0
    private let pointCloudPublishInterval: TimeInterval = 0.2 // Publish ROS point clouds at up to 5Hz
    private var isProcessingFrame = false
    var maxPointLimit: Int = 2_000_000
    var boundingBoxSize: Float = 20.0
    var voxelSize: Float = 0.05 {
        didSet {
            Task { await pointManager.setVoxelSize(voxelSize) }
        }
    }
    var useRoomPlan: Bool = false {
        didSet {
            if oldValue != useRoomPlan {
                setupViews()
                if useRoomPlan, isScanning {
                    roomCaptureView?.captureSession.run(configuration: RoomCaptureSession.Configuration())
                }
            }
        }
    }
    var useImperialUnits: Bool = false
    
    private var meshEntities: [UUID: ModelEntity] = [:]
    private var lastMapPublishTime: TimeInterval = 0
    private let meshSnapshotPublishConfiguration = MeshSnapshotPublishConfiguration.default
    private var anchorEntities: [UUID: AnchorEntity] = [:]
    private var meshUpdateTasks: [UUID: Task<Void, Never>] = [:]
    
    private var measurementNodes: [ModelEntity] = []
    private var measurementLines: [ModelEntity] = []
    private var loadedPointCloudEntity: Entity?
    private var coachingOverlay: ARCoachingOverlayView?
    
    override func viewDidLoad() {
        super.viewDidLoad()
        setupViews()
        
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
        arView?.session.pause()
        roomCaptureView?.captureSession.stop()
        NotificationCenter.default.removeObserver(self)
    }
    
    private func setupViews() {
        // Explicitly pause hardware sensors before deallocating views to prevent massive battery drain
        arView?.session.pause()
        roomCaptureView?.captureSession.stop()
        
        arView?.removeFromSuperview()
        roomCaptureView?.removeFromSuperview()
        arView = nil
        roomCaptureView = nil
        coachingOverlay = nil
        
        if useRoomPlan {
            let rcv = RoomCaptureView(frame: view.bounds)
            rcv.delegate = self
            rcv.captureSession.delegate = self
            view.addSubview(rcv)
            rcv.translatesAutoresizingMaskIntoConstraints = false
            NSLayoutConstraint.activate([
                rcv.topAnchor.constraint(equalTo: self.view.topAnchor),
                rcv.bottomAnchor.constraint(equalTo: self.view.bottomAnchor),
                rcv.leadingAnchor.constraint(equalTo: self.view.leadingAnchor),
                rcv.trailingAnchor.constraint(equalTo: self.view.trailingAnchor)
            ])
            roomCaptureView = rcv
        } else {
            let av = ARView(frame: .zero)
            self.view.addSubview(av)
            
            av.translatesAutoresizingMaskIntoConstraints = false
            NSLayoutConstraint.activate([
                av.topAnchor.constraint(equalTo: self.view.topAnchor),
                av.bottomAnchor.constraint(equalTo: self.view.bottomAnchor),
                av.leadingAnchor.constraint(equalTo: self.view.leadingAnchor),
                av.trailingAnchor.constraint(equalTo: self.view.trailingAnchor)
            ])
            
            av.session.delegate = self

            guard ARWorldTrackingConfiguration.isSupported else {
                DispatchQueue.main.async {
                    self.delegate?.didFailWithError(NSError(domain: "MapEverything", code: 0, userInfo: [NSLocalizedDescriptionKey: "AR World Tracking is not supported on this device."]))
                }
                arView = av
                return
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
            
            let overlay = ARCoachingOverlayView()
            overlay.autoresizingMask = [.flexibleWidth, .flexibleHeight]
            overlay.session = av.session
            overlay.goal = .anyPlane
            av.addSubview(overlay)
            coachingOverlay = overlay
            
            let tapGesture = UITapGestureRecognizer(target: self, action: #selector(handleTap(_:)))
            av.addGestureRecognizer(tapGesture)
            
            let longPressGesture = UILongPressGestureRecognizer(target: self, action: #selector(handleLongPress(_:)))
            av.addGestureRecognizer(longPressGesture)
            
            av.session.run(configuration)
            arView = av
        }
    }

    // MARK: - Data Management
    func saveCurrentScan(name: String, completion: @escaping (EnvironmentModel?) -> Void) {
        var bgTask: UIBackgroundTaskIdentifier = .invalid
        bgTask = UIApplication.shared.beginBackgroundTask(withName: "SaveEnvironment") {
            UIApplication.shared.endBackgroundTask(bgTask)
            bgTask = .invalid
        }
        let finalizeSave: (EnvironmentModel?) -> Void = { model in
            completion(model)
            if bgTask != .invalid {
                UIApplication.shared.endBackgroundTask(bgTask)
            }
        }
        
        let filename = UUID().uuidString
        
        let processSnapshot: (@escaping (String?) -> Void) -> Void = { [weak self] snapshotCompletion in
            guard let self = self else { snapshotCompletion(nil); return }
            
            if self.useRoomPlan, let rcv = self.roomCaptureView {
                let renderer = UIGraphicsImageRenderer(bounds: rcv.bounds)
                let snapshotImage = renderer.image { _ in
                    rcv.drawHierarchy(in: rcv.bounds, afterScreenUpdates: true)
                }
                DispatchQueue.global(qos: .userInitiated).async {
                    if let docDir = FileManager.default.cloudDocumentsURL, let jpegData = snapshotImage.jpegData(compressionQuality: 0.7) {
                        let thumbName = "\(filename)_thumb.jpg"
                        try? jpegData.write(to: docDir.appendingPathComponent(thumbName))
                        DispatchQueue.main.async { snapshotCompletion(thumbName) }
                    } else {
                        DispatchQueue.main.async { snapshotCompletion(nil) }
                    }
                }
            } else if let av = self.arView {
                av.snapshot(saveToHDR: false) { image in
                    DispatchQueue.global(qos: .userInitiated).async {
                        if let image = image, let docDir = FileManager.default.cloudDocumentsURL, let jpegData = image.jpegData(compressionQuality: 0.7) {
                            let thumbName = "\(filename)_thumb.jpg"
                            try? jpegData.write(to: docDir.appendingPathComponent(thumbName))
                            DispatchQueue.main.async { snapshotCompletion(thumbName) }
                        } else {
                            DispatchQueue.main.async { snapshotCompletion(nil) }
                        }
                    }
                }
            } else {
                snapshotCompletion(nil)
            }
        }
        
        processSnapshot { savedThumbnailPath in
            if self.useRoomPlan {
                guard let capturedRoom = self.capturedRoom else {
                    finalizeSave(nil)
                    return
                }
                
                let usdzPath = "\(filename)_room.usdz"
                
                Task.detached(priority: .userInitiated) {
                    if let docDir = FileManager.default.cloudDocumentsURL {
                        let fileURL = docDir.appendingPathComponent(usdzPath)
                        do {
                            try capturedRoom.export(to: fileURL, exportOptions: .parametric)
                            
                            var savedBlueprintPath: String? = nil
                            if let pdfURL = BlueprintExporter.exportToPDF(capturedRoom: capturedRoom, filename: "\(filename)_blueprint") {
                                savedBlueprintPath = pdfURL.lastPathComponent
                            }
                            
                            let model = EnvironmentModel(name: name, filePathToPointCloudData: nil, arWorldMapPath: nil, meshPath: usdzPath, blueprintPath: savedBlueprintPath, videoPath: nil, thumbnailPath: savedThumbnailPath)
                            await MainActor.run { finalizeSave(model) }
                        } catch {
                            print("Failed to export room plan: \(error)")
                            await MainActor.run { finalizeSave(nil) }
                        }
                    } else {
                        await MainActor.run { finalizeSave(nil) }
                    }
                }
                return
            }

            Task {
                let cleanedPoints = await self.pointManager.getCleanedPoints(maxDistance: self.boundingBoxSize)
                
                let savedPath = await Task.detached(priority: .userInitiated) {
                    PointCloudStorageManager.shared.saveBinaryPLY(points: cleanedPoints, to: filename)
                }.value
                
                guard let savedPath = savedPath else {
                    finalizeSave(nil)
                    return
                }
                
                guard let arView = self.arView else { finalizeSave(nil); return }
                let frame = arView.session.currentFrame
                let meshAnchors = frame?.anchors.compactMap { $0 as? ARMeshAnchor } ?? []
                let safeMeshes = MeshGenerator.extractSafeMeshes(from: meshAnchors) // Must happen synchronously to prevent EXC_BAD_ACCESS in background task
                
                arView.session.getCurrentWorldMap { worldMap, error in
                    Task.detached(priority: .userInitiated) {
                        var savedMeshPath: String? = nil
                        var savedObjPath: String? = nil
                        var savedBlueprintPath: String? = nil
                        var savedVideoPath: String? = nil
                        
                        if !safeMeshes.isEmpty {
                            if let exportURLs = MeshExporter.exportToOBJAndUSDZ(safeMeshes: safeMeshes, filename: "\(filename)_mesh") {
                                savedMeshPath = exportURLs.usdzURL?.lastPathComponent
                                savedObjPath = exportURLs.objURL.lastPathComponent
                                if let videoURL = await VideoExporter.exportFlythrough(objURL: exportURLs.objURL, filename: "\(filename)_video") {
                                    savedVideoPath = videoURL.lastPathComponent
                                }
                            }
                            if let pdfURL = BlueprintExporter.exportToPDF(safeMeshes: safeMeshes, filename: "\(filename)_blueprint") {
                                savedBlueprintPath = pdfURL.lastPathComponent
                            }
                        }
                        
                        var savedMapPath: String? = nil
                        if let map = worldMap {
                            if let data = try? NSKeyedArchiver.archivedData(withRootObject: map, requiringSecureCoding: true) {
                                let mapFilename = "\(filename)_map.arworldmap"
                                if let docDir = FileManager.default.cloudDocumentsURL {
                                    let fileURL = docDir.appendingPathComponent(mapFilename)
                                    try? data.write(to: fileURL)
                                    savedMapPath = mapFilename
                                }
                            }
                        }
                        
                        let model = EnvironmentModel(name: name, filePathToPointCloudData: savedPath, arWorldMapPath: savedMapPath, meshPath: savedMeshPath, objPath: savedObjPath, blueprintPath: savedBlueprintPath, videoPath: savedVideoPath, thumbnailPath: savedThumbnailPath)
                        await MainActor.run {
                            finalizeSave(model)
                        }
                    }
                }
            }
        }
    }
    
    func clearScan() {
        if useRoomPlan {
            capturedRoom = nil
            roomCaptureView?.captureSession.stop()
            delegate?.didUpdatePointCount(0)
            delegate?.didUpdateMeasurement("Room cleared.")
            
            // Fully recreate the RoomCaptureView to visually wipe the scanned geometry from the screen
            DispatchQueue.main.async {
                self.setupViews()
            }
            return
        }

        Task { [weak self] in
            guard let self = self else { return }
            await pointManager.clear()
            delegate?.didUpdatePointCount(0)
            
            measurementNodes.forEach { $0.anchor?.removeFromParent() }
            measurementNodes.removeAll()
            measurementLines.forEach { $0.anchor?.removeFromParent() }
            measurementLines.removeAll()
            
            loadedPointCloudEntity?.removeFromParent()
            loadedPointCloudEntity = nil
            
            meshUpdateTasks.values.forEach { $0.cancel() }
            meshUpdateTasks.removeAll()
            
            if let configuration = arView?.session.configuration as? ARWorldTrackingConfiguration {
                configuration.initialWorldMap = nil // ensure it doesn't re-load the map on clear
                arView?.session.run(configuration, options: [.resetTracking, .removeExistingAnchors])
            }
        }
    }
    
    func loadEnvironment(_ env: EnvironmentModel, completion: @escaping () -> Void) {
        clearScan()
        guard let docDir = FileManager.default.cloudDocumentsURL else {
            completion()
            return
        }
        
        if let mapPath = env.arWorldMapPath {
            let fileURL = docDir.appendingPathComponent(mapPath)
            if let data = try? Data(contentsOf: fileURL),
               let worldMap = try? NSKeyedUnarchiver.unarchivedObject(ofClass: ARWorldMap.self, from: data) {
                if let configuration = arView?.session.configuration as? ARWorldTrackingConfiguration {
                    configuration.initialWorldMap = worldMap
                    arView?.session.run(configuration, options: [.resetTracking, .removeExistingAnchors])
                    
                    // Request the user to scan the room to relocalize the map
                    coachingOverlay?.goal = .tracking
                    coachingOverlay?.setActive(true, animated: true)
                }
            }
        }
        
        if let plyPath = env.filePathToPointCloudData {
            Task.detached(priority: .userInitiated) { [weak self] in
                let loadedPoints = PointCloudStorageManager.shared.loadBinaryPLY(from: plyPath)
                await MainActor.run {
                    if let points = loadedPoints {
                        self?.visualizeLoadedPointCloud(points, completion: completion)
                    } else {
                        completion()
                    }
                }
            }
        } else if let meshPath = env.meshPath {
            // Load and visualize the saved USDZ parametric model for RoomPlan scans
            Task.detached(priority: .userInitiated) { [weak self] in
                guard let self = self else { return }
                
                let usdzURL = docDir.appendingPathComponent(meshPath)
                do {
                    try await MainActor.run {
                        let entity = try Entity.loadModel(contentsOf: usdzURL)
                        let anchor = AnchorEntity(world: .zero)
                        anchor.addChild(entity)
                        self.arView?.scene.addAnchor(anchor)
                        self.loadedPointCloudEntity = entity
                        completion()
                    }
                } catch {
                    print("Failed to load USDZ: \(error)")
                    await MainActor.run { completion() }
                }
            }
        } else {
            completion()
        }
    }
    
    var isDepthAnythingModelAvailable: Bool {
        depthAnythingProcessor != nil
    }

    private func evaluateAdaptiveMappingPolicy(frame: ARFrame) {
        let depthConfidence: Double
        if frame.smoothedSceneDepth != nil {
            depthConfidence = 1.0
        } else if frame.sceneDepth != nil {
            depthConfidence = 0.75
        } else {
            depthConfidence = 0.15
        }

        adaptiveMappingController.update(
            input: makeAdaptiveMappingInput(
                roomPlanObjectCount: capturedRoomObjectCount,
                lidarDepthConfidence: depthConfidence
            )
        )
    }

    private func evaluateAdaptiveMappingPolicy(room: CapturedRoom) {
        adaptiveMappingController.update(
            input: makeAdaptiveMappingInput(
                roomPlanObjectCount: capturedRoomElementCount(room),
                lidarDepthConfidence: fallbackLiDARDepthConfidence
            )
        )
    }

    private func makeAdaptiveMappingInput(
        roomPlanObjectCount: Int,
        lidarDepthConfidence: Double
    ) -> AdaptiveMappingModeInput {
        let localization = IndoorLocalizationManager.shared
        let horizontalAccuracy = localization.lastHorizontalAccuracy >= 0
            ? localization.lastHorizontalAccuracy
            : nil

        return AdaptiveMappingModeInput(
            roomPlanAvailable: RoomCaptureSession.isSupported,
            roomPlanObjectCount: roomPlanObjectCount,
            indoorRegistrationQuality: localization.lastIndoorRegistrationQuality,
            globalRegistrationQuality: localization.lastGlobalRegistrationQuality,
            gpsHorizontalAccuracyMeters: horizontalAccuracy,
            lidarDepthConfidence: lidarDepthConfidence,
            depthAnythingAvailable: isDepthAnythingModelAvailable,
            thermalState: ProcessInfo.processInfo.thermalState,
            operatorOverride: adaptiveMappingController.operatorOverride
        )
    }

    private var capturedRoomObjectCount: Int {
        capturedRoom.map(capturedRoomElementCount) ?? 0
    }

    private var fallbackLiDARDepthConfidence: Double {
        if ARWorldTrackingConfiguration.supportsFrameSemantics(.smoothedSceneDepth) {
            return 0.75
        }
        if ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) {
            return 0.55
        }
        return 0.15
    }

    private func capturedRoomElementCount(_ room: CapturedRoom) -> Int {
        room.walls.count + room.doors.count + room.windows.count + room.objects.count
    }

    @MainActor
    private func shouldUseFusedMappingDepth() -> Bool {
        depthAnythingProcessor != nil
    }

    private func processFusedMappingFrame(frame: ARFrame, transform: simd_float4x4) async -> [ColoredPoint]? {
        // Rate-limit so the model only runs at ~enhancedFrameInterval. Returning an
        // empty array means "skip this frame" rather than falling back to LiDAR-only.
        let now = frame.timestamp
        let canRun: Bool = await MainActor.run {
            guard now - lastEnhancedFrameTime >= enhancedFrameInterval,
                  !isRunningEnhancedInference else { return false }
            lastEnhancedFrameTime = now
            isRunningEnhancedInference = true
            return true
        }
        guard canRun else { return [] }

        defer {
            Task { @MainActor in self.isRunningEnhancedInference = false }
        }

        guard let processor = depthAnythingProcessor else { return nil }
        guard let sceneDepth = frame.smoothedSceneDepth ?? frame.sceneDepth else { return nil }

        guard let relative = processor.inferRelativeDepth(from: frame.capturedImage) else { return nil }
        guard let fused = processor.fuseMaximumLikelihood(relative: relative, lidarDepthMap: sceneDepth.depthMap) else { return nil }

        return pointCloudProcessor.processPointCloudEnhanced(frame: frame, transform: transform, depthMap: fused)
    }

    func exportFloorplan(completion: @escaping (URL?) -> Void) {
        let walls: [BlueprintExporter.WallSegment]

        if useRoomPlan, let room = capturedRoom {
            walls = BlueprintExporter.extractWallSegments(from: room)
        } else if let anchors = arView?.session.currentFrame?.anchors {
            walls = BlueprintExporter.extractWallSegments(from: anchors)
        } else {
            completion(nil)
            return
        }

        guard !walls.isEmpty else {
            completion(nil)
            return
        }

        let filename = "floorplan_\(UUID().uuidString)"
        let url = BlueprintExporter.exportDimensionedFloorplan(
            walls: walls,
            filename: filename,
            useImperialUnits: useImperialUnits
        )
        completion(url)
    }

    private func visualizeLoadedPointCloud(_ points: [ColoredPoint], completion: @escaping () -> Void) {
        loadedPointCloudEntity?.removeFromParent()
        
        let pointSize: Float = 0.003
        var positions: [SIMD3<Float>] = []
        var indices: [UInt32] = []
        positions.reserveCapacity(points.count * 3)
        indices.reserveCapacity(points.count * 3)
        for (i, pt) in points.enumerated() {
            let p = pt.position
            let base = UInt32(i * 3)
            positions.append(p + SIMD3<Float>(-pointSize, 0, 0))
            positions.append(p + SIMD3<Float>(pointSize, 0, 0))
            positions.append(p + SIMD3<Float>(0, pointSize, 0))
            indices.append(contentsOf: [base, base + 1, base + 2])
        }
        var desc = MeshDescriptor()
        desc.positions = MeshBuffers.Positions(positions)
        desc.primitives = .triangles(indices)
        
        Task { [weak self] in
            guard let self = self else { return }
            
            do {
                let resource = try await MeshResource(from: [desc])
                let material = UnlitMaterial(color: .white)
                let entity = ModelEntity(mesh: resource, materials: [material])
                
                await MainActor.run {
                    let anchor = AnchorEntity(world: .zero)
                    anchor.addChild(entity)
                    self.arView?.scene.addAnchor(anchor)
                    self.loadedPointCloudEntity = entity
                    completion()
                }
            } catch {
                print("Failed to generate point cloud mesh: \(error)")
                await MainActor.run { completion() }
            }
        }
    }
    
    func updateVisualizationMode(_ mode: VisualizationMode) {
        currentMode = mode
        
        arView?.debugOptions.remove([.showFeaturePoints, .showSceneUnderstanding])
        meshEntities.values.forEach { $0.isEnabled = false }
        
        switch mode {
        case .none: break
        case .pointCloud:
            arView?.debugOptions.insert(.showFeaturePoints)
        case .wireframe:
            arView?.debugOptions.insert(.showSceneUnderstanding)
        case .solidMesh:
            meshEntities.values.forEach { $0.isEnabled = true }
        }
    }
    
    @objc func handleTap(_ sender: UITapGestureRecognizer) {
        guard appMode != .scan else { return }
        guard let arView = arView else { return }
        
        let location = sender.location(in: arView)
        guard let result = arView.raycast(from: location, allowing: .estimatedPlane, alignment: .any).first else { return }
        
        let generator = UIImpactFeedbackGenerator(style: .medium)
        generator.impactOccurred()
        
        if appMode == .measure {
            let arAnchor = ARAnchor(name: "measurement", transform: result.worldTransform)
            arView.session.add(anchor: arAnchor)
        } else if appMode == .homeRemodel {
            let arAnchor = ARAnchor(name: "remodel", transform: result.worldTransform)
            arView.session.add(anchor: arAnchor)
        } else if appMode == .landscape {
            let arAnchor = ARAnchor(name: "landscape", transform: result.worldTransform)
            arView.session.add(anchor: arAnchor)
        }
    }
    
    @objc func handleLongPress(_ sender: UILongPressGestureRecognizer) {
        guard sender.state == .began else { return }
        guard let arView = arView else { return }
        let location = sender.location(in: arView)
        
        if let entity = arView.entity(at: location) as? ModelEntity {
            if entity == loadedPointCloudEntity { return }
            
            let generator = UIImpactFeedbackGenerator(style: .heavy)
            generator.impactOccurred()
            
            if let anchorEntity = entity.anchor {
                if let id = anchorEntity.anchorIdentifier,
                   let arAnchor = arView.session.currentFrame?.anchors.first(where: { $0.identifier == id }) {
                    arView.session.remove(anchor: arAnchor)
                }
                anchorEntity.removeFromParent()
                
                if let index = measurementNodes.firstIndex(of: entity) {
                    measurementNodes.forEach { $0.anchor?.removeFromParent() }
                    measurementNodes.removeAll()
                    measurementLines.forEach { $0.anchor?.removeFromParent() }
                    measurementLines.removeAll()
                    delegate?.didUpdateMeasurement("Measurement cleared.")
                }
            }
        }
    }
    
    // MARK: - Node Spawning
    func undoLastMeasurement() {
        guard !measurementNodes.isEmpty else {
            delegate?.didUpdateMeasurement("")
            return
        }

        let lastNode = measurementNodes.removeLast()
        lastNode.anchor?.removeFromParent()

        if let lastLine = measurementLines.popLast() {
            lastLine.anchor?.removeFromParent()
        }

        if measurementNodes.count >= 2 {
            if measurementNodes.count > 2 {
                let area = calculatePolygonArea(nodes: measurementNodes)
                if useImperialUnits {
                    delegate?.didUpdateMeasurement(String(format: "Area: %.2f sq ft", area * 10.7639))
                } else {
                    delegate?.didUpdateMeasurement(String(format: "Area: %.2f sq m", area))
                }
            } else {
                let p1 = measurementNodes[0].position(relativeTo: nil)
                let p2 = measurementNodes[1].position(relativeTo: nil)
                let d = simd_distance(p1, p2)
                if useImperialUnits {
                    delegate?.didUpdateMeasurement(String(format: "Distance: %.2f ft", d * 3.28084))
                } else {
                    delegate?.didUpdateMeasurement(String(format: "Distance: %.2f m", d))
                }
            }
        } else if measurementNodes.count == 1 {
            delegate?.didUpdateMeasurement("Point 1 placed. Tap to continue path.")
        } else {
            delegate?.didUpdateMeasurement("")
        }
    }

    private func spawnMeasurementNode(for anchor: ARAnchor) {
        let sphere = MeshResource.generateSphere(radius: 0.02)
        let material = SimpleMaterial(color: .red, isMetallic: false)
        let entity = ModelEntity(mesh: sphere, materials: [material])
        
        let anchorEntity = AnchorEntity(anchor: anchor)
        anchorEntity.addChild(entity)
        arView?.scene.addAnchor(anchorEntity)
        
        measurementNodes.append(entity)
        
        if measurementNodes.count > 1 {
            let p1 = measurementNodes[measurementNodes.count - 2].position(relativeTo: nil)
            let p2 = measurementNodes[measurementNodes.count - 1].position(relativeTo: nil)
            let distance = simd_distance(p1, p2)
            
            // Draw connecting line
            let midpoint = (p1 + p2) / 2
            let direction = normalize(p2 - p1)
            let defaultUp = SIMD3<Float>(0, 1, 0)
            
            // Prevent NaN singularity crash when vector points straight down
            let rotation: simd_quatf
            if simd_dot(defaultUp, direction) < -0.9999 {
                rotation = simd_quatf(angle: .pi, axis: SIMD3<Float>(1, 0, 0))
            } else {
                rotation = simd_quatf(from: defaultUp, to: direction)
            }
            
            let cylinder = MeshResource.generateCylinder(height: distance, radius: 0.005)
            let lineMaterial = SimpleMaterial(color: .yellow, isMetallic: false)
            let lineEntity = ModelEntity(mesh: cylinder, materials: [lineMaterial])
            lineEntity.position = midpoint
            lineEntity.orientation = rotation
            
            let lineAnchor = AnchorEntity(world: midpoint)
            lineAnchor.addChild(lineEntity)
            arView?.scene.addAnchor(lineAnchor)
            measurementLines.append(lineEntity)
            
            if measurementNodes.count > 2 {
                let area = calculatePolygonArea(nodes: measurementNodes)
                if useImperialUnits {
                    delegate?.didUpdateMeasurement(String(format: "Area: %.2f sq ft", area * 10.7639))
                } else {
                    delegate?.didUpdateMeasurement(String(format: "Area: %.2f sq m", area))
                }
            } else {
                if useImperialUnits {
                    delegate?.didUpdateMeasurement(String(format: "Distance: %.2f ft", distance * 3.28084))
                } else {
                    delegate?.didUpdateMeasurement(String(format: "Distance: %.2f m", distance))
                }
            }
        } else {
            delegate?.didUpdateMeasurement("Point 1 placed. Tap to continue path.")
        }
    }
    
    private func calculatePolygonArea(nodes: [ModelEntity]) -> Float {
        var crossSum = simd_float3(0, 0, 0)
        let n = nodes.count
        for i in 0..<n {
            let p1 = nodes[i].position(relativeTo: nil)
            let p2 = nodes[(i + 1) % n].position(relativeTo: nil) // Connect back to origin mathematically
            crossSum += simd_cross(p1, p2)
        }
        return 0.5 * simd_length(crossSum)
    }
    
    private func spawnRemodelNode(for anchor: ARAnchor) {
        let entity: ModelEntity
        if let usdzModel = try? Entity.loadModel(named: "remodel") {
            entity = usdzModel
        } else {
            let box = MeshResource.generateBox(size: 0.5)
            let material = SimpleMaterial(color: UIColor.systemBlue.withAlphaComponent(0.6), isMetallic: false)
            entity = ModelEntity(mesh: box, materials: [material])
            entity.position.y += 0.25
        }
        
        let anchorEntity = AnchorEntity(anchor: anchor)
        anchorEntity.addChild(entity)
        arView?.scene.addAnchor(anchorEntity)
    }
    
    private func spawnLandscapeNode(for anchor: ARAnchor) {
        let entity: ModelEntity
        if let usdzModel = try? Entity.loadModel(named: "landscape") {
            entity = usdzModel
        } else {
            let cone = MeshResource.generateCone(height: 1.5, radius: 0.5)
            let material = SimpleMaterial(color: .systemGreen, isMetallic: false)
            entity = ModelEntity(mesh: cone, materials: [material])
            entity.position.y += 0.75
        }
        
        let anchorEntity = AnchorEntity(anchor: anchor)
        anchorEntity.addChild(entity)
        arView?.scene.addAnchor(anchorEntity)
    }

    // MARK: - ARSessionDelegate
    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        guard isScanning else { return }

        MapGeoreferencer.shared.updateMapPose(frame.camera.transform, timestamp: frame.timestamp)
        evaluateAdaptiveMappingPolicy(frame: frame)
        
        if ROS2BridgeClient.shared.isConnected {
            ROS2BridgeClient.shared.publishPose(frame.camera.transform, timestamp: frame.timestamp)
            ROS2BridgeClient.shared.publishOdometry(frame.camera.transform, timestamp: frame.timestamp)
            ROS2BridgeClient.shared.publishTF(frame.camera.transform, timestamp: frame.timestamp)
        }
        
        // Only accumulate LiDAR points if spatial tracking is highly accurate to prevent garbage data
        guard case .normal = frame.camera.trackingState else { return }
        
        // Throttle processing to prevent CPU overload and memory churn
        guard frame.timestamp - lastPointProcessingTime > pointProcessingInterval else { return }
        guard !isProcessingFrame else { return }
        
        lastPointProcessingTime = frame.timestamp
        isProcessingFrame = true
        
        let transform = frame.camera.transform
        let timestamp = frame.timestamp
        let shouldPublishPointCloud = timestamp - lastPointCloudPublishTime >= pointCloudPublishInterval
        
        Task.detached(priority: .userInitiated) { [weak self] in
            guard let self = self else { return }
            
            if ROS2BridgeClient.shared.isConnected {
                ROS2BridgeClient.shared.publishImage(frame: frame, timestamp: timestamp)
            }
            
            let shouldUseFusedDepth = await self.shouldUseFusedMappingDepth()
            let newPoints: [ColoredPoint]
            if shouldUseFusedDepth {
                if let fusedPoints = await self.processFusedMappingFrame(frame: frame, transform: transform) {
                    newPoints = fusedPoints
                } else {
                    newPoints = self.pointCloudProcessor.processPointCloud(frame: frame, transform: transform)
                }
            } else {
                newPoints = self.pointCloudProcessor.processPointCloud(frame: frame, transform: transform)
            }

            if !newPoints.isEmpty {
                if shouldPublishPointCloud {
                    // Downsample the network payload to a sparse 10cm grid to prevent saturating the Wi-Fi bandwidth
                    let sparsePoints = self.pointCloudProcessor.voxelGridFilter(points: newPoints, voxelSize: 0.1)
                    ROS2BridgeClient.shared.publishPointCloud(sparsePoints, timestamp: timestamp)
                    await MainActor.run {
                        self.lastPointCloudPublishTime = timestamp
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
        publishMapToROS2IfNeeded()
        
        for anchor in anchors {
            if let meshAnchor = anchor as? ARMeshAnchor {
                let desc = MeshGenerator.createDescriptor(from: meshAnchor.geometry)
                
                // Create and add the tracking anchor immediately on the main thread
                let anchorEntity = AnchorEntity(anchor: meshAnchor)
                self.anchorEntities[meshAnchor.identifier] = anchorEntity
                self.arView?.scene.addAnchor(anchorEntity)
                
                let isSolid = (currentMode == .solidMesh)
                let identifier = meshAnchor.identifier
                
                meshUpdateTasks[identifier]?.cancel()
                
                let task = Task { [weak self] in
                    guard let self = self else { return }
                    
                    do {
                        let meshResource = try await MeshResource(from: [desc])
                        let material = SimpleMaterial(color: UIColor.systemBlue.withAlphaComponent(0.4), isMetallic: false)
                        let modelEntity = ModelEntity(mesh: meshResource, materials: [material])
                        modelEntity.isEnabled = isSolid
                        
                        guard !Task.isCancelled else { return }
                        
                        await MainActor.run {
                            guard !Task.isCancelled else { return }
                            // Ensure the user hasn't cleared the scan while we were generating
                            if let targetAnchor = self.anchorEntities[identifier] {
                                self.meshEntities[identifier] = modelEntity
                                targetAnchor.addChild(modelEntity)
                            }
                        }
                    } catch {
                        print("Failed to create mesh entity: \(error)")
                    }
                }
                meshUpdateTasks[identifier] = task
            } else if anchor.name == "measurement" {
                spawnMeasurementNode(for: anchor)
            } else if anchor.name == "remodel" {
                spawnRemodelNode(for: anchor)
            } else if anchor.name == "landscape" {
                spawnLandscapeNode(for: anchor)
            }
        }
    }
    
    func session(_ session: ARSession, didUpdate anchors: [ARAnchor]) {
        publishMapToROS2IfNeeded()
        
        for anchor in anchors {
            if let meshAnchor = anchor as? ARMeshAnchor,
               let entity = meshEntities[meshAnchor.identifier] {
                let desc = MeshGenerator.createDescriptor(from: meshAnchor.geometry)
                
                meshUpdateTasks[meshAnchor.identifier]?.cancel()
                
                let task = Task { [weak entity] in
                    guard let entity = entity else { return }
                    do {
                        let meshResource = try await MeshResource(from: [desc])
                        guard !Task.isCancelled else { return }

                        await MainActor.run {
                            guard !Task.isCancelled else { return }
                            if var modelComponent = entity.components[ModelComponent.self] as? ModelComponent {
                                modelComponent.mesh = meshResource
                                entity.components.set(modelComponent)
                            }
                        }
                    } catch {
                        print("Failed to update mesh entity: \(error)")
                    }
                }
                meshUpdateTasks[meshAnchor.identifier] = task
            }
        }
    }
    
    private func publishMapToROS2IfNeeded() {
        guard ROS2TopicRegistry.shared.isStreamEnabled(.mesh) else { return }
        
        let currentTime = Date().timeIntervalSince1970
        if currentTime - lastMapPublishTime > meshSnapshotPublishConfiguration.publishInterval {
            lastMapPublishTime = currentTime
            
            let timestamp = arView?.session.currentFrame?.timestamp ?? ProcessInfo.processInfo.systemUptime
            let meshAnchors = arView?.session.currentFrame?.anchors.compactMap { $0 as? ARMeshAnchor } ?? []
            Task.detached(priority: .background) {
                if !meshAnchors.isEmpty {
                    ROS2BridgeClient.shared.publishMap(meshAnchors: meshAnchors, timestamp: timestamp)
                }
            }
        }
    }
    
    func session(_ session: ARSession, didRemove anchors: [ARAnchor]) {
        for anchor in anchors {
            if let meshAnchor = anchor as? ARMeshAnchor {
                meshUpdateTasks[meshAnchor.identifier]?.cancel()
                meshUpdateTasks.removeValue(forKey: meshAnchor.identifier)
                
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
        if let configuration = session.configuration {
            session.run(configuration)
        }
    }

    // MARK: - RoomCaptureViewDelegate
    func captureView(shouldPresent roomDataForProcessing: CapturedRoomData, error: Error?) -> Bool {
        if let error = error {
            print("RoomCaptureView processing error: \(error)")
            delegate?.didFailWithError(error)
            return false
        }
        return true
    }

    func captureView(didPresent processedResult: CapturedRoom, error: Error?) {
        if let error = error {
            print("RoomCaptureView result error: \(error)")
            delegate?.didFailWithError(error)
        } else {
            self.capturedRoom = processedResult
            evaluateAdaptiveMappingPolicy(room: processedResult)
            delegate?.didUpdateTrackingFeedback("Room captured successfully!")
            
            let objectCount = capturedRoomElementCount(processedResult)
            delegate?.didUpdatePointCount(objectCount)
        }
    }
    
    // MARK: - RoomCaptureSessionDelegate
    func captureSession(_ session: RoomCaptureSession, didUpdate room: CapturedRoom) {
        evaluateAdaptiveMappingPolicy(room: room)
        let objectCount = capturedRoomElementCount(room)
        delegate?.didUpdatePointCount(objectCount)
        
        if ROS2BridgeClient.shared.isConnected {
            ROS2BridgeClient.shared.publishRoomPlan(room, timestamp: ProcessInfo.processInfo.systemUptime)
        }
    }
}
