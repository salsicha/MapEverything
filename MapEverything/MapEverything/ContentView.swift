//
//  ContentView.swift
//  MapEverything
//
//  Created by Alex Moran on 5/2/26.
//

import SwiftUI
import SwiftData
import AVFoundation
import SceneKit

enum VisualizationMode: String, CaseIterable, Identifiable {
    case none = "None"
    case pointCloud = "Point Cloud"
    case wireframe = "Wireframe"
    case solidMesh = "Solid Mesh"
    var id: Self { self }

    var iconName: String {
        switch self {
        case .none: return "eye.slash"
        case .pointCloud: return "circle.dotted"
        case .wireframe: return "square.grid.3x3"
        case .solidMesh: return "cube.fill"
        }
    }
}

enum AppMode: String, CaseIterable, Identifiable {
    case scan = "Scan"
    case enhancedScan = "AI Depth"
    case measure = "Measure"
    case homeRemodel = "Remodel"
    case landscape = "Landscape"
    var id: Self { self }

    var iconName: String {
        switch self {
        case .scan: return "viewfinder"
        case .enhancedScan: return "sparkles.rectangle.stack"
        case .measure: return "ruler"
        case .homeRemodel: return "sofa"
        case .landscape: return "leaf"
        }
    }

    var hint: String {
        switch self {
        case .scan: return "Walk around to capture geometry"
        case .enhancedScan: return "Depth Anything V2 + LiDAR fusion for dense scans"
        case .measure: return "Tap to place points • Long-press to clear"
        case .homeRemodel: return "Tap surfaces to place furniture"
        case .landscape: return "Tap surfaces to place plants"
        }
    }
}

struct ContentView: View {
    @Environment(\.modelContext) private var modelContext
    
    @ObservedObject private var ros2Client = ROS2BridgeClient.shared
    @ObservedObject private var mappingSession = MappingSessionManager.shared
    @ObservedObject private var adaptiveMapping = AdaptiveMappingModeController.shared
    @ObservedObject private var localBagRecorder = LocalROS2BagRecorder.shared
    
    // App Settings
    @AppStorage("maxPointLimit") private var maxPointLimit: Int = 2_000_000
    @AppStorage("voxelSize") private var voxelSize: Double = 0.05
    @AppStorage("boundingBoxSize") private var boundingBoxSize: Double = 20.0
    @AppStorage("ros2Enabled") private var ros2Enabled: Bool = false
    @AppStorage("ros2WebSocketURL") private var ros2WebSocketURL: String = "ws://192.168.1.100:9090"
    @AppStorage("useImperialUnits") private var useImperialUnits: Bool = false
    
    @State private var visualizationMode: VisualizationMode = .solidMesh
    @State private var isScanning = false
    @State private var pointCount = 0
    
    @State private var trackingFeedback: String = ""
    @State private var appMode: AppMode = .scan
    @State private var measurementText: String = ""
    
    @State private var showSaveDialog = false
    @State private var scanName = ""
    @State private var triggerSaveName: String? = nil
    @State private var triggerClear = false
    @State private var environmentToLoad: EnvironmentModel? = nil
    @State private var errorMessage: String? = nil
    @State private var isProcessing: Bool = false
    
    @State private var triggerExportFloorplan = false
    @State private var triggerUndoMeasurement = false
    @State private var floorplanURL: URL?

    @State private var showGallery = false
    @State private var showSettings = false
    @State private var showClearConfirmation = false
    @State private var hasCameraPermission = false
    
    var body: some View {
        Group {
            if hasCameraPermission {
                mainScannerView
            } else {
                permissionDeniedView
            }
        }
        .onAppear(perform: checkCameraPermission)
    }
    
    private var mainScannerView: some View {
        NavigationStack {
            ZStack(alignment: .bottom) {
                ARViewContainer(
                    visualizationMode: $visualizationMode,
                    isScanning: $isScanning,
                    appMode: $appMode,
                    measurementText: $measurementText,
                    pointCount: $pointCount,
                    trackingFeedback: $trackingFeedback,
                    errorMessage: $errorMessage,
                    isProcessing: $isProcessing,
                    maxPointLimit: maxPointLimit,
                    voxelSize: Float(voxelSize),
                    boundingBoxSize: Float(boundingBoxSize),
                    useRoomPlan: adaptiveMapping.usesRoomPlanCapture,
                    useImperialUnits: useImperialUnits,
                    onSave: { newModel in
                        modelContext.insert(newModel)
                    },
                    onFloorplanExported: { url in
                        floorplanURL = url
                    },
                    triggerSaveName: $triggerSaveName,
                    triggerClear: $triggerClear,
                    triggerExportFloorplan: $triggerExportFloorplan,
                    triggerUndoMeasurement: $triggerUndoMeasurement,
                    environmentToLoad: $environmentToLoad
                )
                .edgesIgnoringSafeArea(.all)

                VStack {
                    recorderDiagnosticsPanel
                    Spacer()
                }
                .padding(.top, 54)
                .padding(.horizontal, 14)
                .allowsHitTesting(false)

                VStack(spacing: 12) {
                    visualizationModeControl

                    Button(action: toggleMapping) {
                        HStack(spacing: 10) {
                            Image(systemName: isScanning ? "stop.fill" : "play.fill")
                                .font(.headline.weight(.bold))
                            Text(isScanning ? "Stop" : "Start")
                                .font(.headline.weight(.semibold))
                        }
                        .foregroundColor(.white)
                        .frame(width: 168, height: 56)
                        .background(isScanning ? Color.red : Color.green)
                        .clipShape(Capsule())
                        .shadow(color: Color.black.opacity(0.35), radius: 12, x: 0, y: 6)
                    }
                    .accessibilityLabel(isScanning ? "Stop Mapping" : "Start Mapping")
                }
                .padding(.bottom, 28)
            }
            .navigationBarHidden(true)
            .alert("AR Session Error", isPresented: Binding<Bool>(
                get: { errorMessage != nil },
                set: { if !$0 { errorMessage = nil } }
            ), presenting: errorMessage) { _ in
                Button("OK", role: .cancel) { }
            } message: { msg in
                Text(msg)
            }
            .onAppear {
                appMode = .scan
                visualizationMode = .solidMesh
                mappingSession.configure(recorderURL: ros2WebSocketURL)
            }
            .onChange(of: isScanning) { scanning in
                UIApplication.shared.isIdleTimerDisabled = scanning
            }
        }
    }

    private var recorderDiagnosticsPanel: some View {
        let queueStats = ros2Client.publishQueueStats
        let localBufferStats = ros2Client.localSampleBufferStats
        let localBagStats = localBagRecorder.stats
        let lastError = queueStats.lastError ?? mappingSession.lastError

        return VStack(alignment: .leading, spacing: 8) {
            HStack(spacing: 8) {
                Circle()
                    .fill(ros2Client.isConnected ? Color.green : Color.red)
                    .frame(width: 8, height: 8)
                Text("Recorder")
                    .font(.caption.weight(.semibold))
                Spacer(minLength: 10)
                Text(ros2Client.isConnected ? "Connected" : mappingSession.state.label)
                    .font(.caption2.weight(.medium))
                    .foregroundColor(.secondary)
            }

            HStack(spacing: 12) {
                Label("\(queueStats.depth)/\(queueStats.capacity)", systemImage: "tray.full")
                Label("\(queueStats.droppedMessages)", systemImage: "exclamationmark.triangle")
                Label("\(localBufferStats.pointCloudSamples + localBufferStats.meshSamples)", systemImage: "externaldrive")
                if localBagStats.isEnabled {
                    Label("\(localBagStats.messageCount)", systemImage: "externaldrive.badge.plus")
                }
                Label(adaptiveMapping.activeMode.displayName, systemImage: adaptiveMapping.usesRoomPlanCapture ? "square.3.layers.3d" : "sparkles.rectangle.stack")
            }
            .font(.caption2.monospacedDigit())
            .foregroundColor(.secondary)

            if let lastError, !lastError.isEmpty {
                Text(lastError)
                    .font(.caption2)
                    .foregroundColor(.orange)
                    .lineLimit(2)
                    .fixedSize(horizontal: false, vertical: true)
            }
        }
        .padding(.horizontal, 12)
        .padding(.vertical, 10)
        .frame(maxWidth: 280, alignment: .leading)
        .background(.ultraThinMaterial)
        .clipShape(RoundedRectangle(cornerRadius: 8, style: .continuous))
        .overlay(
            RoundedRectangle(cornerRadius: 8, style: .continuous)
                .stroke(Color.white.opacity(0.18), lineWidth: 1)
        )
        .shadow(color: Color.black.opacity(0.25), radius: 8, x: 0, y: 4)
    }

    private var visualizationModeControl: some View {
        Picker("Visualization", selection: $visualizationMode) {
            ForEach(VisualizationMode.allCases) { mode in
                Image(systemName: mode.iconName)
                    .tag(mode)
                    .accessibilityLabel(mode.rawValue)
            }
        }
        .pickerStyle(.segmented)
        .frame(width: 252)
        .padding(8)
        .background(.ultraThinMaterial)
        .clipShape(RoundedRectangle(cornerRadius: 8, style: .continuous))
        .overlay(
            RoundedRectangle(cornerRadius: 8, style: .continuous)
                .stroke(Color.white.opacity(0.18), lineWidth: 1)
        )
        .accessibilityLabel("Visualization")
    }

    private func toggleMapping() {
        UIImpactFeedbackGenerator(style: .medium).impactOccurred()
        isScanning.toggle()

        if isScanning {
            mappingSession.start(recorderURL: ros2WebSocketURL)
        } else {
            mappingSession.stop()
        }
    }
    
    private var saveDialogView: some View {
        NavigationStack {
            Form {
                Section("Scan Name") {
                    TextField("My Living Room", text: $scanName)
                        .submitLabel(.done)
                }

                Section("Will be saved") {
                    Label("\(pointCount) \(adaptiveMapping.usesRoomPlanCapture ? "elements" : "points")", systemImage: adaptiveMapping.usesRoomPlanCapture ? "square.3.layers.3d" : "circle.dotted")
                    if !adaptiveMapping.usesRoomPlanCapture {
                        Label("3D mesh (USDZ + OBJ)", systemImage: "cube")
                        Label("Blueprint PDF", systemImage: "doc.richtext")
                        Label("Flythrough video", systemImage: "video")
                    } else {
                        Label("Parametric room model", systemImage: "cube")
                        Label("Blueprint PDF", systemImage: "doc.richtext")
                    }
                    Label("AR world map", systemImage: "map")
                }

                Section {
                    Text("Export may take 30–60 seconds for large scans.")
                        .font(.caption)
                        .foregroundColor(.secondary)
                }
            }
            .navigationTitle("Save Scan")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .navigationBarLeading) {
                    Button("Cancel") {
                        showSaveDialog = false
                        scanName = ""
                    }
                }
                ToolbarItem(placement: .navigationBarTrailing) {
                    Button("Save") {
                        triggerSaveName = scanName.isEmpty ? "Untitled Scan" : scanName
                        showSaveDialog = false
                        scanName = ""
                    }
                    .fontWeight(.semibold)
                }
            }
        }
        .presentationDetents([.medium])
    }
}

extension ContentView {
    private var permissionDeniedView: some View {
        VStack(spacing: 20) {
            Image(systemName: "camera.fill")
                .font(.system(size: 60))
                .foregroundColor(.gray)
            Text("Camera Access Required")
                .font(.title2)
                .fontWeight(.bold)
            Text("MapEverything needs camera and LiDAR access to capture 3D environments.")
                .multilineTextAlignment(.center)
                .foregroundColor(.secondary)
                .padding(.horizontal)
            Button("Open Settings") {
                if let url = URL(string: UIApplication.openSettingsURLString) {
                    UIApplication.shared.open(url)
                }
            }
            .padding()
            .background(Color.blue)
            .foregroundColor(.white)
            .cornerRadius(10)
        }
        .padding()
    }
    
    private func checkCameraPermission() {
        switch AVCaptureDevice.authorizationStatus(for: .video) {
        case .authorized:
            hasCameraPermission = true
        case .notDetermined:
            AVCaptureDevice.requestAccess(for: .video) { granted in
                DispatchQueue.main.async { self.hasCameraPermission = granted }
            }
        case .denied, .restricted:
            hasCameraPermission = false
        @unknown default:
            hasCameraPermission = false
        }
    }
}

struct EnvironmentGalleryView: View {
    @Environment(\.modelContext) private var modelContext
    @Query(sort: \EnvironmentModel.creationDate, order: .reverse) private var environments: [EnvironmentModel]
    @Environment(\.dismiss) private var dismiss
    
    var onLoad: (EnvironmentModel) -> Void
    
    struct FilePreviewItem: Identifiable {
        let id = UUID()
        let url: URL
    }

    @State private var previewEnvironment: EnvironmentModel?
    @State private var filePreview: FilePreviewItem?

    @State private var pendingDeletionOffsets: IndexSet?
    @State private var showingDeleteConfirmation = false
    @State private var floorplanURL: URL?
    @State private var generatingFloorplanFor: UUID?
    @State private var searchText: String = ""
    @State private var renamingEnvironment: EnvironmentModel?
    @State private var renameText: String = ""
    @AppStorage("useImperialUnits") private var useImperialUnits: Bool = false

    private var filteredEnvironments: [EnvironmentModel] {
        guard !searchText.isEmpty else { return environments }
        return environments.filter { $0.name.localizedCaseInsensitiveContains(searchText) }
    }

    private func fileURL(for path: String?) -> URL? {
        guard let path = path, let docDir = FileManager.default.cloudDocumentsURL else { return nil }
        let url = docDir.appendingPathComponent(path)
        return FileManager.default.fileExists(atPath: url.path) ? url : nil
    }

    var body: some View {
        NavigationStack {
            List {
                if environments.isEmpty {
                    ContentUnavailableView(
                        "No Saved Scans",
                        systemImage: "cube.transparent",
                        description: Text("Scans you save will appear here.")
                    )
                } else if filteredEnvironments.isEmpty {
                    ContentUnavailableView.search(text: searchText)
                } else {
                    ForEach(filteredEnvironments) { env in
                        VStack(alignment: .leading, spacing: 8) {
                            Button {
                                previewEnvironment = env
                            } label: {
                                TopDownThumbnailView(environment: env)
                                    .frame(maxWidth: .infinity)
                                    .frame(height: 160)
                                    .clipped()
                                    .cornerRadius(8)
                            }
                            .buttonStyle(.plain)

                            HStack {
                                VStack(alignment: .leading) {
                                    Text(env.name)
                                        .font(.headline)
                                    Text(env.creationDate, style: .date)
                                        .font(.caption)
                                        .foregroundColor(.secondary)
                                }
                                Spacer()
                            }

                            HStack(spacing: 16) {
                                Button {
                                    previewEnvironment = env
                                } label: {
                                    Label("Preview", systemImage: "eye.fill")
                                        .font(.caption)
                                        .padding(.horizontal, 10)
                                        .padding(.vertical, 6)
                                        .background(Color.blue.opacity(0.15))
                                        .foregroundColor(.blue)
                                        .cornerRadius(8)
                                }
                                .buttonStyle(.borderless)

                                Button {
                                    onLoad(env)
                                    dismiss()
                                } label: {
                                    Label("Load", systemImage: "icloud.and.arrow.down")
                                        .font(.caption)
                                        .padding(.horizontal, 10)
                                        .padding(.vertical, 6)
                                        .background(Color.green.opacity(0.15))
                                        .foregroundColor(.green)
                                        .cornerRadius(8)
                                }
                                .buttonStyle(.borderless)

                                Button {
                                    generateFloorplan(for: env)
                                } label: {
                                    if generatingFloorplanFor == env.id {
                                        ProgressView()
                                            .frame(width: 32, height: 20)
                                    } else {
                                        Label("Floorplan", systemImage: "ruler")
                                            .font(.caption)
                                            .padding(.horizontal, 10)
                                            .padding(.vertical, 6)
                                            .background(Color.indigo.opacity(0.15))
                                            .foregroundColor(.indigo)
                                            .cornerRadius(8)
                                    }
                                }
                                .buttonStyle(.borderless)
                                .disabled(generatingFloorplanFor != nil)

                                Spacer()

                                if let url = fileURL(for: env.meshPath) ?? fileURL(for: env.objPath) {
                                    Button {
                                        filePreview = FilePreviewItem(url: url)
                                    } label: {
                                        Image(systemName: "cube.transparent")
                                            .frame(width: 32, height: 32)
                                            .foregroundColor(.purple)
                                    }
                                    .buttonStyle(.borderless)
                                }

                                if let url = fileURL(for: env.blueprintPath) {
                                    Button {
                                        filePreview = FilePreviewItem(url: url)
                                    } label: {
                                        Image(systemName: "map")
                                            .frame(width: 32, height: 32)
                                            .foregroundColor(.orange)
                                    }
                                    .buttonStyle(.borderless)
                                }

                                if let url = fileURL(for: env.videoPath) {
                                    Button {
                                        filePreview = FilePreviewItem(url: url)
                                    } label: {
                                        Image(systemName: "video.fill")
                                            .frame(width: 32, height: 32)
                                            .foregroundColor(.red)
                                    }
                                    .buttonStyle(.borderless)
                                }

                                if let url = fileURL(for: env.filePathToPointCloudData) {
                                    ShareLink(item: url) {
                                        Image(systemName: "square.and.arrow.up")
                                            .frame(width: 32, height: 32)
                                            .foregroundColor(.blue)
                                    }
                                    .buttonStyle(.borderless)
                                }
                            }
                        }
                        .padding(.vertical, 4)
                        .swipeActions(edge: .leading) {
                            Button {
                                renameText = env.name
                                renamingEnvironment = env
                            } label: {
                                Label("Rename", systemImage: "pencil")
                            }
                            .tint(.blue)
                        }
                    }
                    .onDelete { offsets in
                        let mapped = IndexSet(offsets.compactMap { idx -> Int? in
                            let env = filteredEnvironments[idx]
                            return environments.firstIndex(where: { $0.id == env.id })
                        })
                        pendingDeletionOffsets = mapped
                        showingDeleteConfirmation = true
                    }
                }
            }
            .navigationTitle("Saved Scans")
            .searchable(text: $searchText, prompt: "Search scans")
            .alert("Rename Scan", isPresented: Binding<Bool>(
                get: { renamingEnvironment != nil },
                set: { if !$0 { renamingEnvironment = nil } }
            )) {
                TextField("Name", text: $renameText)
                Button("Save") {
                    if let env = renamingEnvironment {
                        let trimmed = renameText.trimmingCharacters(in: .whitespacesAndNewlines)
                        if !trimmed.isEmpty {
                            env.name = trimmed
                        }
                    }
                    renamingEnvironment = nil
                }
                Button("Cancel", role: .cancel) {
                    renamingEnvironment = nil
                }
            }
            .alert("Delete Scan?", isPresented: $showingDeleteConfirmation) {
                Button("Delete", role: .destructive) {
                    if let offsets = pendingDeletionOffsets {
                        deleteEnvironments(offsets: offsets)
                    }
                }
                Button("Cancel", role: .cancel) {
                    pendingDeletionOffsets = nil
                }
            } message: {
                Text("This action cannot be undone. All associated 3D models, blueprints, and videos will be permanently removed.")
            }
            .sheet(item: $previewEnvironment) { env in
                ScanPreviewSheet(environment: env)
            }
            .sheet(item: $filePreview) { item in
                QuickLookPreview(url: item.url)
                    .edgesIgnoringSafeArea(.all)
            }
            .sheet(isPresented: Binding<Bool>(
                get: { floorplanURL != nil },
                set: { if !$0 { floorplanURL = nil } }
            )) {
                if let url = floorplanURL {
                    FloorplanShareView(url: url)
                }
            }
            .toolbar {
                ToolbarItem(placement: .navigationBarTrailing) {
                    Button("Done") {
                        dismiss()
                    }
                }
            }
        }
    }
    
    private func deleteEnvironments(offsets: IndexSet) {
        for index in offsets {
            let env = environments[index]
            
            // Dispatch heavy file I/O to a background thread to prevent UI swipe hitching
            if let docDir = FileManager.default.cloudDocumentsURL {
                let paths = [env.filePathToPointCloudData, env.arWorldMapPath, env.meshPath, env.objPath, env.blueprintPath, env.videoPath, env.thumbnailPath].compactMap { $0 }
                DispatchQueue.global(qos: .background).async {
                    for path in paths {
                        let fileURL = docDir.appendingPathComponent(path)
                        try? FileManager.default.removeItem(at: fileURL)
                    }
                }
            }
            
            withAnimation {
                modelContext.delete(env)
            }
        }
    }

    private func generateFloorplan(for env: EnvironmentModel) {
        generatingFloorplanFor = env.id
        let imperial = useImperialUnits
        Task.detached(priority: .userInitiated) {
            guard let docDir = FileManager.default.cloudDocumentsURL else {
                await MainActor.run { generatingFloorplanFor = nil }
                return
            }

            var walls: [BlueprintExporter.WallSegment] = []

            // Try mesh/USDZ first — extract vertical surfaces as wall segments
            if let meshPath = env.meshPath ?? env.objPath {
                let url = docDir.appendingPathComponent(meshPath)
                if let scene = try? SCNScene(url: url) {
                    walls = Self.extractWallsFromScene(scene)
                }
            }

            // Fall back to point cloud — detect walls via vertical slice analysis
            if walls.isEmpty, let plyPath = env.filePathToPointCloudData,
               let points = PointCloudStorageManager.shared.loadBinaryPLY(from: plyPath) {
                walls = Self.extractWallsFromPointCloud(points)
            }

            let filename = "floorplan_\(env.name.replacingOccurrences(of: " ", with: "_"))_\(UUID().uuidString.prefix(6))"
            let url = BlueprintExporter.exportDimensionedFloorplan(walls: walls, filename: filename, useImperialUnits: imperial)

            await MainActor.run {
                generatingFloorplanFor = nil
                floorplanURL = url
            }
        }
    }

    private static func extractWallsFromScene(_ scene: SCNScene) -> [BlueprintExporter.WallSegment] {
        var walls: [BlueprintExporter.WallSegment] = []
        scene.rootNode.enumerateChildNodes { node, _ in
            guard let geometry = node.geometry else { return }
            guard let posSource = geometry.sources.first(where: { $0.semantic == .vertex }) else { return }

            let count = posSource.vectorCount
            guard count >= 3 else { return }
            let data = posSource.data
            let dataStride = posSource.dataStride
            let offset = posSource.dataOffset
            let transform = node.worldTransform

            var vertices: [SIMD3<Float>] = []
            vertices.reserveCapacity(count)

            data.withUnsafeBytes { rawBuffer in
                guard let base = rawBuffer.baseAddress else { return }
                for i in 0..<count {
                    let ptr = base + i * dataStride + offset
                    let x = ptr.load(as: Float.self)
                    let y = ptr.load(fromByteOffset: 4, as: Float.self)
                    let z = ptr.load(fromByteOffset: 8, as: Float.self)
                    let tx = transform.m11 * x + transform.m21 * y + transform.m31 * z + transform.m41
                    let ty = transform.m12 * x + transform.m22 * y + transform.m32 * z + transform.m42
                    let tz = transform.m13 * x + transform.m23 * y + transform.m33 * z + transform.m43
                    vertices.append(SIMD3<Float>(tx, ty, tz))
                }
            }

            // Find the bounding box height — if tall and thin, it's likely a wall
            guard !vertices.isEmpty else { return }
            var minY: Float = .greatestFiniteMagnitude, maxY: Float = -.greatestFiniteMagnitude
            var minX: Float = .greatestFiniteMagnitude, maxX: Float = -.greatestFiniteMagnitude
            var minZ: Float = .greatestFiniteMagnitude, maxZ: Float = -.greatestFiniteMagnitude
            for v in vertices {
                minY = min(minY, v.y); maxY = max(maxY, v.y)
                minX = min(minX, v.x); maxX = max(maxX, v.x)
                minZ = min(minZ, v.z); maxZ = max(maxZ, v.z)
            }
            let height = maxY - minY
            let spanX = maxX - minX
            let spanZ = maxZ - minZ
            let horizontalSpan = max(spanX, spanZ)
            guard height > 0.5, horizontalSpan > 0.3 else { return }

            let start = CGPoint(x: CGFloat(minX), y: CGFloat(minZ))
            let end = CGPoint(x: CGFloat(maxX), y: CGFloat(maxZ))
            let length = Float(hypot(end.x - start.x, end.y - start.y))
            walls.append(BlueprintExporter.WallSegment(startPoint: start, endPoint: end, lengthMeters: length))
        }
        return walls
    }

    private static func extractWallsFromPointCloud(_ points: [ColoredPoint]) -> [BlueprintExporter.WallSegment] {
        guard points.count > 100 else { return [] }

        // Simple approach: bin points into a 2D grid (top-down), find occupied cells,
        // then detect linear runs of cells as wall segments.
        var minX: Float = .greatestFiniteMagnitude, maxX: Float = -.greatestFiniteMagnitude
        var minZ: Float = .greatestFiniteMagnitude, maxZ: Float = -.greatestFiniteMagnitude
        for p in points {
            minX = min(minX, p.position.x); maxX = max(maxX, p.position.x)
            minZ = min(minZ, p.position.z); maxZ = max(maxZ, p.position.z)
        }
        let rangeX = maxX - minX, rangeZ = maxZ - minZ
        guard rangeX > 0.5, rangeZ > 0.5 else { return [] }

        let cellSize: Float = 0.1
        let gridW = Int(rangeX / cellSize) + 1
        let gridH = Int(rangeZ / cellSize) + 1
        var density = [Int](repeating: 0, count: gridW * gridH)

        for p in points {
            let gx = min(Int((p.position.x - minX) / cellSize), gridW - 1)
            let gz = min(Int((p.position.z - minZ) / cellSize), gridH - 1)
            density[gz * gridW + gx] += 1
        }

        // Find the density threshold (top 15% cells are likely walls/surfaces)
        let sorted = density.filter { $0 > 0 }.sorted()
        guard !sorted.isEmpty else { return [] }
        let threshold = sorted[max(0, Int(Double(sorted.count) * 0.85))]
        guard threshold > 0 else { return [] }

        // Collect dense cells as wall candidates
        var wallCells: [(x: Int, z: Int)] = []
        for gz in 0..<gridH {
            for gx in 0..<gridW {
                if density[gz * gridW + gx] >= threshold {
                    wallCells.append((x: gx, z: gz))
                }
            }
        }

        // Simple line segment extraction: scan rows and columns for runs of dense cells
        var walls: [BlueprintExporter.WallSegment] = []

        // Scan rows (horizontal walls)
        for gz in 0..<gridH {
            var runStart: Int? = nil
            for gx in 0..<gridW {
                let isDense = density[gz * gridW + gx] >= threshold
                if isDense && runStart == nil {
                    runStart = gx
                } else if !isDense, let start = runStart {
                    let length = Float(gx - start) * cellSize
                    if length >= 0.5 {
                        let s = CGPoint(x: CGFloat(minX + Float(start) * cellSize), y: CGFloat(minZ + Float(gz) * cellSize))
                        let e = CGPoint(x: CGFloat(minX + Float(gx) * cellSize), y: CGFloat(minZ + Float(gz) * cellSize))
                        walls.append(BlueprintExporter.WallSegment(startPoint: s, endPoint: e, lengthMeters: length))
                    }
                    runStart = nil
                }
            }
            if let start = runStart {
                let length = Float(gridW - start) * cellSize
                if length >= 0.5 {
                    let s = CGPoint(x: CGFloat(minX + Float(start) * cellSize), y: CGFloat(minZ + Float(gz) * cellSize))
                    let e = CGPoint(x: CGFloat(minX + Float(gridW) * cellSize), y: CGFloat(minZ + Float(gz) * cellSize))
                    walls.append(BlueprintExporter.WallSegment(startPoint: s, endPoint: e, lengthMeters: length))
                }
            }
        }

        // Scan columns (vertical walls)
        for gx in 0..<gridW {
            var runStart: Int? = nil
            for gz in 0..<gridH {
                let isDense = density[gz * gridW + gx] >= threshold
                if isDense && runStart == nil {
                    runStart = gz
                } else if !isDense, let start = runStart {
                    let length = Float(gz - start) * cellSize
                    if length >= 0.5 {
                        let s = CGPoint(x: CGFloat(minX + Float(gx) * cellSize), y: CGFloat(minZ + Float(start) * cellSize))
                        let e = CGPoint(x: CGFloat(minX + Float(gx) * cellSize), y: CGFloat(minZ + Float(gz) * cellSize))
                        walls.append(BlueprintExporter.WallSegment(startPoint: s, endPoint: e, lengthMeters: length))
                    }
                    runStart = nil
                }
            }
            if let start = runStart {
                let length = Float(gridH - start) * cellSize
                if length >= 0.5 {
                    let s = CGPoint(x: CGFloat(minX + Float(gx) * cellSize), y: CGFloat(minZ + Float(start) * cellSize))
                    let e = CGPoint(x: CGFloat(minX + Float(gx) * cellSize), y: CGFloat(minZ + Float(gridH) * cellSize))
                    walls.append(BlueprintExporter.WallSegment(startPoint: s, endPoint: e, lengthMeters: length))
                }
            }
        }

        return walls
    }
}

struct SettingsView: View {
    @ObservedObject private var mappingSession = MappingSessionManager.shared
    @ObservedObject private var ros2Client = ROS2BridgeClient.shared
    @ObservedObject private var adaptiveMapping = AdaptiveMappingModeController.shared
    @ObservedObject private var localBagRecorder = LocalROS2BagRecorder.shared
    @AppStorage("maxPointLimit") private var maxPointLimit: Int = 2_000_000
    @AppStorage("voxelSize") private var voxelSize: Double = 0.05
    @AppStorage("boundingBoxSize") private var boundingBoxSize: Double = 20.0
    @AppStorage("ros2Enabled") private var ros2Enabled: Bool = false
    @AppStorage("ros2WebSocketURL") private var ros2WebSocketURL: String = "ws://192.168.1.100:9090"
    @AppStorage(LocalROS2BagRecorderConfiguration.enabledStorageKey) private var localROS2BagStorageEnabled: Bool = false
    @AppStorage(LocalROS2BagRecorderConfiguration.chunkSizeMBStorageKey) private var localROS2BagChunkSizeMB: Int = LocalROS2BagRecorderConfiguration.defaultChunkSizeMB
    @State private var showLocalBagBrowser = false
    @AppStorage("bleBeaconServiceUUIDs") private var bleBeaconServiceUUIDs: String = ""
    @AppStorage("bleBeaconPeripheralIDs") private var bleBeaconPeripheralIDs: String = ""
    @AppStorage("bleBeaconLocalNamePrefixes") private var bleBeaconLocalNamePrefixes: String = ""
    @AppStorage("bleBeaconAllowDuplicateAdvertisements") private var bleBeaconAllowDuplicateAdvertisements: Bool = true
    @AppStorage("useImperialUnits") private var useImperialUnits: Bool = false
    @AppStorage(GeoTileProviderConfigurationStore.key("enabled", .copernicusDataSpace)) private var copernicusEnabled: Bool = false
    @AppStorage(GeoTileProviderConfigurationStore.key("endpointURL", .copernicusDataSpace)) private var copernicusEndpointURL: String = GeoTileOptionalProviderID.copernicusDataSpace.defaultEndpointURL
    @AppStorage(GeoTileProviderConfigurationStore.key("credentialReference", .copernicusDataSpace)) private var copernicusCredentialReference: String = ""
    @AppStorage(GeoTileProviderConfigurationStore.key("hasCredentialMaterial", .copernicusDataSpace)) private var copernicusHasCredentialMaterial: Bool = false
    @AppStorage(GeoTileProviderConfigurationStore.key("recordingAllowed", .copernicusDataSpace)) private var copernicusRecordingAllowed: Bool = false
    @AppStorage(GeoTileProviderConfigurationStore.key("attributionOverride", .copernicusDataSpace)) private var copernicusAttributionOverride: String = ""
    @AppStorage(GeoTileProviderConfigurationStore.key("enabled", .openTopography)) private var openTopographyEnabled: Bool = false
    @AppStorage(GeoTileProviderConfigurationStore.key("endpointURL", .openTopography)) private var openTopographyEndpointURL: String = GeoTileOptionalProviderID.openTopography.defaultEndpointURL
    @AppStorage(GeoTileProviderConfigurationStore.key("credentialReference", .openTopography)) private var openTopographyCredentialReference: String = ""
    @AppStorage(GeoTileProviderConfigurationStore.key("hasCredentialMaterial", .openTopography)) private var openTopographyHasCredentialMaterial: Bool = false
    @AppStorage(GeoTileProviderConfigurationStore.key("recordingAllowed", .openTopography)) private var openTopographyRecordingAllowed: Bool = false
    @AppStorage(GeoTileProviderConfigurationStore.key("attributionOverride", .openTopography)) private var openTopographyAttributionOverride: String = ""
    @AppStorage(GeoTileProviderConfigurationStore.key("enabled", .usgsEROS)) private var usgsEROSEnabled: Bool = false
    @AppStorage(GeoTileProviderConfigurationStore.key("endpointURL", .usgsEROS)) private var usgsEROSEndpointURL: String = GeoTileOptionalProviderID.usgsEROS.defaultEndpointURL
    @AppStorage(GeoTileProviderConfigurationStore.key("credentialReference", .usgsEROS)) private var usgsEROSCredentialReference: String = ""
    @AppStorage(GeoTileProviderConfigurationStore.key("hasCredentialMaterial", .usgsEROS)) private var usgsEROSHasCredentialMaterial: Bool = false
    @AppStorage(GeoTileProviderConfigurationStore.key("recordingAllowed", .usgsEROS)) private var usgsEROSRecordingAllowed: Bool = false
    @AppStorage(GeoTileProviderConfigurationStore.key("attributionOverride", .usgsEROS)) private var usgsEROSAttributionOverride: String = ""
    @AppStorage(GeoTileProviderConfigurationStore.key("enabled", .commercialImagery)) private var commercialImageryEnabled: Bool = false
    @AppStorage(GeoTileProviderConfigurationStore.key("endpointURL", .commercialImagery)) private var commercialImageryEndpointURL: String = GeoTileOptionalProviderID.commercialImagery.defaultEndpointURL
    @AppStorage(GeoTileProviderConfigurationStore.key("credentialReference", .commercialImagery)) private var commercialImageryCredentialReference: String = ""
    @AppStorage(GeoTileProviderConfigurationStore.key("hasCredentialMaterial", .commercialImagery)) private var commercialImageryHasCredentialMaterial: Bool = false
    @AppStorage(GeoTileProviderConfigurationStore.key("recordingAllowed", .commercialImagery)) private var commercialImageryRecordingAllowed: Bool = false
    @AppStorage(GeoTileProviderConfigurationStore.key("attributionOverride", .commercialImagery)) private var commercialImageryAttributionOverride: String = ""
    @AppStorage(GeoTileProviderConfigurationStore.key("enabled", .commercialTerrain)) private var commercialTerrainEnabled: Bool = false
    @AppStorage(GeoTileProviderConfigurationStore.key("endpointURL", .commercialTerrain)) private var commercialTerrainEndpointURL: String = GeoTileOptionalProviderID.commercialTerrain.defaultEndpointURL
    @AppStorage(GeoTileProviderConfigurationStore.key("credentialReference", .commercialTerrain)) private var commercialTerrainCredentialReference: String = ""
    @AppStorage(GeoTileProviderConfigurationStore.key("hasCredentialMaterial", .commercialTerrain)) private var commercialTerrainHasCredentialMaterial: Bool = false
    @AppStorage(GeoTileProviderConfigurationStore.key("recordingAllowed", .commercialTerrain)) private var commercialTerrainRecordingAllowed: Bool = false
    @AppStorage(GeoTileProviderConfigurationStore.key("attributionOverride", .commercialTerrain)) private var commercialTerrainAttributionOverride: String = ""
    @Environment(\.dismiss) private var dismiss
    
    var body: some View {
        NavigationStack {
            Form {
                Section(header: Text("Scanning Mode")) {
                    Picker("Mapping Mode", selection: adaptiveMappingOverrideBinding) {
                        ForEach(AdaptiveMappingOperatorOverride.allCases, id: \.self) { override in
                            Text(override.displayName).tag(override)
                        }
                    }
                    Label(adaptiveMapping.activeMode.displayName, systemImage: adaptiveMapping.usesRoomPlanCapture ? "square.3.layers.3d" : "sparkles.rectangle.stack")
                        .font(.caption)
                    Label("Confidence \(Int(adaptiveMapping.recommendation.confidence * 100))%", systemImage: "gauge.with.dots.needle.67percent")
                        .font(.caption)
                        .foregroundColor(.secondary)
                }
                Section(header: Text("Preferences")) {
                    Toggle("Use Imperial Units (ft / sq ft)", isOn: $useImperialUnits)
                }
                Section(header: Text("LiDAR Resolution")) {
                    Picker("Voxel Density", selection: $voxelSize) {
                        Text("High (2cm)").tag(0.02)
                        Text("Medium (5cm)").tag(0.05)
                        Text("Low (10cm)").tag(0.10)
                    }
                    .pickerStyle(SegmentedPickerStyle())
                    Text("Higher density captures greater detail but consumes more memory.")
                        .font(.caption)
                        .foregroundColor(.secondary)
                }
                Section(header: Text("Scanning Limits")) {
                    Stepper("Max Points: \(maxPointLimit)", value: $maxPointLimit, in: 100_000...5_000_000, step: 100_000)
                }
                Section(header: Text("Processing")) {
                    Stepper("Bounding Box Size: \(Int(boundingBoxSize))m", value: $boundingBoxSize, in: 5...100, step: 5)
                    Text("Points further than this distance from the origin will be ignored during export.")
                        .font(.caption)
                        .foregroundColor(.secondary)
                }

                Section {
                    Label("Changes to voxel density, max points, and bounding box take effect on the next scan.", systemImage: "info.circle")
                        .font(.caption)
                        .foregroundColor(.secondary)
                }
                Section(header: Text("BLE Beacon Filters")) {
                    TextField("Service UUIDs", text: $bleBeaconServiceUUIDs)
                        .autocapitalization(.none)
                    TextField("Peripheral UUIDs", text: $bleBeaconPeripheralIDs)
                        .autocapitalization(.none)
                    TextField("Local name prefixes", text: $bleBeaconLocalNamePrefixes)
                        .autocapitalization(.none)
                    Toggle("Allow Duplicate Advertisements", isOn: $bleBeaconAllowDuplicateAdvertisements)
                }
                Section(header: Text("Optional Geospatial Providers")) {
                    optionalProviderSlot(
                        GeoTileOptionalProviderID.copernicusDataSpace,
                        isEnabled: $copernicusEnabled,
                        endpointURL: $copernicusEndpointURL,
                        credentialReference: $copernicusCredentialReference,
                        hasCredentialMaterial: $copernicusHasCredentialMaterial,
                        recordingAllowed: $copernicusRecordingAllowed,
                        attributionOverride: $copernicusAttributionOverride
                    )
                    optionalProviderSlot(
                        GeoTileOptionalProviderID.openTopography,
                        isEnabled: $openTopographyEnabled,
                        endpointURL: $openTopographyEndpointURL,
                        credentialReference: $openTopographyCredentialReference,
                        hasCredentialMaterial: $openTopographyHasCredentialMaterial,
                        recordingAllowed: $openTopographyRecordingAllowed,
                        attributionOverride: $openTopographyAttributionOverride
                    )
                    optionalProviderSlot(
                        GeoTileOptionalProviderID.usgsEROS,
                        isEnabled: $usgsEROSEnabled,
                        endpointURL: $usgsEROSEndpointURL,
                        credentialReference: $usgsEROSCredentialReference,
                        hasCredentialMaterial: $usgsEROSHasCredentialMaterial,
                        recordingAllowed: $usgsEROSRecordingAllowed,
                        attributionOverride: $usgsEROSAttributionOverride
                    )
                    optionalProviderSlot(
                        GeoTileOptionalProviderID.commercialImagery,
                        isEnabled: $commercialImageryEnabled,
                        endpointURL: $commercialImageryEndpointURL,
                        credentialReference: $commercialImageryCredentialReference,
                        hasCredentialMaterial: $commercialImageryHasCredentialMaterial,
                        recordingAllowed: $commercialImageryRecordingAllowed,
                        attributionOverride: $commercialImageryAttributionOverride
                    )
                    optionalProviderSlot(
                        GeoTileOptionalProviderID.commercialTerrain,
                        isEnabled: $commercialTerrainEnabled,
                        endpointURL: $commercialTerrainEndpointURL,
                        credentialReference: $commercialTerrainCredentialReference,
                        hasCredentialMaterial: $commercialTerrainHasCredentialMaterial,
                        recordingAllowed: $commercialTerrainRecordingAllowed,
                        attributionOverride: $commercialTerrainAttributionOverride
                    )
                }
                Section(header: Text("ROS2 Recorder")) {
                    Toggle("Enable Mapping Stream", isOn: $ros2Enabled)
                    if ros2Enabled {
                        TextField("WebSocket URL (rosbridge)", text: $ros2WebSocketURL)
                            .autocapitalization(.none)
                            .keyboardType(.URL)
                        Label("Session: \(mappingSession.state.label)", systemImage: "dot.radiowaves.left.and.right")
                            .font(.caption)
                            .foregroundColor(.secondary)
                        Label(
                            "Queue: \(ros2Client.publishQueueStats.depth)/\(ros2Client.publishQueueStats.capacity) pending",
                            systemImage: "tray.full"
                        )
                        .font(.caption)
                        .foregroundColor(.secondary)
                        Label(
                            "Dropped: \(ros2Client.publishQueueStats.droppedMessages)  Retried: \(ros2Client.publishQueueStats.retriedMessages)",
                            systemImage: "exclamationmark.arrow.triangle.2.circlepath"
                        )
                        .font(.caption)
                        .foregroundColor(.secondary)
                        if let lastError = ros2Client.publishQueueStats.lastError {
                            Label(lastError, systemImage: "exclamationmark.triangle")
                                .font(.caption)
                                .foregroundColor(.orange)
                        }
                    }
                }
                Section(header: Text("Local ROS2 Bag Storage")) {
                    Toggle("Save Local SQLite Bag", isOn: $localROS2BagStorageEnabled)
                    Button {
                        showLocalBagBrowser = true
                    } label: {
                        Label("Manage Local Bags", systemImage: "externaldrive")
                    }
                    if localROS2BagStorageEnabled {
                        Stepper(
                            "Chunk Size: \(localROS2BagChunkSizeMB) MB",
                            value: $localROS2BagChunkSizeMB,
                            in: LocalROS2BagRecorderConfiguration.minimumChunkSizeMB...LocalROS2BagRecorderConfiguration.maximumChunkSizeMB,
                            step: 8
                        )
                        Label(
                            localBagRecorder.stats.isRecording ? "Recording \(localBagRecorder.stats.messageCount) messages" : "Starts with the next mapping session",
                            systemImage: localBagRecorder.stats.isRecording ? "record.circle" : "externaldrive"
                        )
                        .font(.caption)
                        .foregroundColor(.secondary)
                        if let bagURL = localBagRecorder.stats.bagDirectoryURL {
                            Text(bagURL.lastPathComponent)
                                .font(.caption2.monospaced())
                                .foregroundColor(.secondary)
                        }
                    }
                }
            }
            .navigationTitle("Settings")
            .sheet(isPresented: $showLocalBagBrowser) {
                LocalROS2BagBrowserView()
            }
            .toolbar {
                ToolbarItem(placement: .navigationBarTrailing) {
                    Button("Done") {
                        dismiss()
                    }
                }
            }
        }
    }

    private var adaptiveMappingOverrideBinding: Binding<AdaptiveMappingOperatorOverride> {
        Binding(
            get: { adaptiveMapping.operatorOverride },
            set: { adaptiveMapping.setOperatorOverride($0) }
        )
    }

    @ViewBuilder
    private func optionalProviderSlot(
        _ providerID: GeoTileOptionalProviderID,
        isEnabled: Binding<Bool>,
        endpointURL: Binding<String>,
        credentialReference: Binding<String>,
        hasCredentialMaterial: Binding<Bool>,
        recordingAllowed: Binding<Bool>,
        attributionOverride: Binding<String>
    ) -> some View {
        VStack(alignment: .leading, spacing: 8) {
            Toggle(providerID.displayName, isOn: isEnabled)
            if isEnabled.wrappedValue {
                TextField("Endpoint URL", text: endpointURL)
                    .autocapitalization(.none)
                    .keyboardType(.URL)
                TextField("Credential reference (not secret)", text: credentialReference)
                    .autocapitalization(.none)
                Toggle("\(providerID.credentialLabel) configured", isOn: hasCredentialMaterial)
                Toggle("Recording rights confirmed", isOn: recordingAllowed)
                TextField("Attribution override", text: attributionOverride)
                    .autocapitalization(.none)
                Text(providerSlotStatus(providerID, hasCredentialMaterial: hasCredentialMaterial.wrappedValue, recordingAllowed: recordingAllowed.wrappedValue))
                    .font(.caption)
                    .foregroundColor(.secondary)
            }
        }
        .padding(.vertical, 4)
    }

    private func providerSlotStatus(
        _ providerID: GeoTileOptionalProviderID,
        hasCredentialMaterial: Bool,
        recordingAllowed: Bool
    ) -> String {
        let credentialState = hasCredentialMaterial ? "credentials present" : "credentials missing"
        let recordingState = recordingAllowed ? "recording allowed" : "recording disabled"
        return "\(providerID.credentialLabel); \(credentialState); \(recordingState)"
    }
}

struct LocalROS2BagBrowserView: View {
    @ObservedObject private var recorder = LocalROS2BagRecorder.shared
    @Environment(\.dismiss) private var dismiss
    @State private var sessions: [LocalROS2BagSession] = []
    @State private var pendingDeletion: LocalROS2BagSession?
    @State private var errorMessage: String?

    var body: some View {
        NavigationStack {
            List {
                if sessions.isEmpty {
                    ContentUnavailableView(
                        "No Local Bags",
                        systemImage: "externaldrive",
                        description: Text("Recorded local bags appear here.")
                    )
                } else {
                    ForEach(sessions) { session in
                        NavigationLink {
                            LocalROS2BagSessionDetailView(
                                session: session,
                                isActive: isActive(session)
                            )
                        } label: {
                            localBagRow(session)
                        }
                        .swipeActions {
                            if !isActive(session) {
                                Button(role: .destructive) {
                                    pendingDeletion = session
                                } label: {
                                    Label("Delete", systemImage: "trash")
                                }
                            }
                        }
                    }
                }
            }
            .navigationTitle("Local Bags")
            .toolbar {
                ToolbarItem(placement: .navigationBarLeading) {
                    Button("Done") {
                        dismiss()
                    }
                }
                ToolbarItem(placement: .navigationBarTrailing) {
                    Button {
                        reload()
                    } label: {
                        Image(systemName: "arrow.clockwise")
                    }
                    .accessibilityLabel("Refresh")
                }
            }
            .onAppear(perform: reload)
            .refreshable {
                reload()
            }
            .alert("Delete Local Bag?", isPresented: Binding<Bool>(
                get: { pendingDeletion != nil },
                set: { if !$0 { pendingDeletion = nil } }
            )) {
                Button("Delete", role: .destructive) {
                    deletePendingSession()
                }
                Button("Cancel", role: .cancel) {
                    pendingDeletion = nil
                }
            }
            .alert("Local Bag Error", isPresented: Binding<Bool>(
                get: { errorMessage != nil },
                set: { if !$0 { errorMessage = nil } }
            )) {
                Button("OK", role: .cancel) { }
            } message: {
                Text(errorMessage ?? "")
            }
        }
    }

    private func localBagRow(_ session: LocalROS2BagSession) -> some View {
        VStack(alignment: .leading, spacing: 4) {
            HStack {
                Text(session.name)
                    .font(.headline)
                    .lineLimit(1)
                if isActive(session) {
                    Image(systemName: "record.circle.fill")
                        .foregroundColor(.red)
                }
            }
            HStack(spacing: 10) {
                Label("\(session.chunkCount)", systemImage: "cylinder.split.1x2")
                Label(session.byteCountLabel, systemImage: "doc")
                if let modifiedAt = session.modifiedAt {
                    Text(modifiedAt, style: .date)
                }
            }
            .font(.caption)
            .foregroundColor(.secondary)
        }
    }

    private func reload() {
        do {
            sessions = try recorder.listBagSessions()
        } catch {
            errorMessage = error.localizedDescription
        }
    }

    private func deletePendingSession() {
        guard let pendingDeletion else { return }
        do {
            try recorder.deleteBagSession(pendingDeletion)
            self.pendingDeletion = nil
            reload()
        } catch {
            errorMessage = error.localizedDescription
        }
    }

    private func isActive(_ session: LocalROS2BagSession) -> Bool {
        guard recorder.stats.isRecording,
              let activeURL = recorder.stats.bagDirectoryURL else {
            return false
        }

        return activeURL.standardizedFileURL.path == session.directoryURL.standardizedFileURL.path
    }
}

struct LocalROS2BagSessionDetailView: View {
    let session: LocalROS2BagSession
    let isActive: Bool

    var body: some View {
        List {
            Section("Bag") {
                LabeledContent("Files", value: "\(session.files.count)")
                LabeledContent("Chunks", value: "\(session.chunkCount)")
                LabeledContent("Size", value: session.byteCountLabel)
                if let modifiedAt = session.modifiedAt {
                    LabeledContent("Modified") {
                        Text(modifiedAt, style: .date)
                    }
                }
                if isActive {
                    Label("Recording", systemImage: "record.circle.fill")
                        .foregroundColor(.red)
                }
            }

            Section("Files") {
                ForEach(session.files) { file in
                    HStack(spacing: 12) {
                        Image(systemName: file.kind.iconName)
                            .foregroundColor(.blue)
                            .frame(width: 24)
                        VStack(alignment: .leading, spacing: 2) {
                            Text(file.name)
                                .font(.body)
                                .lineLimit(1)
                            Text("\(file.kind.displayName) • \(file.byteCountLabel)")
                                .font(.caption)
                                .foregroundColor(.secondary)
                        }
                        Spacer()
                        ShareLink(item: file.url) {
                            Image(systemName: "square.and.arrow.up")
                                .frame(width: 32, height: 32)
                        }
                        .buttonStyle(.borderless)
                    }
                }
            }
        }
        .navigationTitle(session.name)
        .navigationBarTitleDisplayMode(.inline)
    }
}

struct FloorplanShareView: View {
    let url: URL
    @Environment(\.dismiss) private var dismiss
    @State private var pdfImage: UIImage?

    var body: some View {
        NavigationStack {
            VStack {
                if let pdfImage = pdfImage {
                    Image(uiImage: pdfImage)
                        .resizable()
                        .scaledToFit()
                        .padding()
                } else {
                    ProgressView("Rendering preview...")
                        .frame(maxHeight: .infinity)
                }

                HStack(spacing: 20) {
                    ShareLink(item: url) {
                        Label("Share PDF", systemImage: "square.and.arrow.up")
                            .font(.headline)
                            .padding()
                            .frame(maxWidth: .infinity)
                            .background(Color.indigo)
                            .foregroundColor(.white)
                            .cornerRadius(12)
                    }
                }
                .padding(.horizontal)
                .padding(.bottom)
            }
            .navigationTitle("Floor Plan")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .navigationBarTrailing) {
                    Button("Done") { dismiss() }
                }
            }
        }
        .onAppear(perform: renderPreview)
    }

    private func renderPreview() {
        let scale: CGFloat = 2.0
        Task.detached(priority: .userInitiated) {
            guard let doc = CGPDFDocument(url as CFURL),
                  let page = doc.page(at: 1) else { return }
            let box = page.getBoxRect(.mediaBox)
            let size = CGSize(width: box.width * scale, height: box.height * scale)
            UIGraphicsBeginImageContextWithOptions(size, true, 1.0)
            guard let ctx = UIGraphicsGetCurrentContext() else { return }
            ctx.setFillColor(UIColor.white.cgColor)
            ctx.fill(CGRect(origin: .zero, size: size))
            ctx.translateBy(x: 0, y: size.height)
            ctx.scaleBy(x: scale, y: -scale)
            ctx.drawPDFPage(page)
            let img = UIGraphicsGetImageFromCurrentImageContext()
            UIGraphicsEndImageContext()
            await MainActor.run { pdfImage = img }
        }
    }
}

struct LocalAsyncImage: View {
    let url: URL
    @State private var image: UIImage?
    
    var body: some View {
        Group {
            if let image = image {
                Image(uiImage: image)
                    .resizable()
                    .scaledToFill()
            } else {
                Rectangle()
                    .fill(Color.gray.opacity(0.2))
                    .overlay(ProgressView())
            }
        }
        .onAppear(perform: load)
    }
    
    private func load() {
        DispatchQueue.global(qos: .userInitiated).async {
            if let loadedImage = UIImage(contentsOfFile: url.path) {
                DispatchQueue.main.async {
                    self.image = loadedImage
                }
            }
        }
    }
}

struct TopDownThumbnailView: View {
    let environment: EnvironmentModel
    @State private var image: UIImage?

    var body: some View {
        ZStack {
            Color(white: 0.12)
            if let image = image {
                Image(uiImage: image)
                    .resizable()
                    .scaledToFit()
            } else {
                ProgressView()
                    .tint(.white)
            }
        }
        .onAppear(perform: generate)
    }

    private func generate() {
        Task.detached(priority: .userInitiated) {
            guard let docDir = FileManager.default.cloudDocumentsURL else { return }

            var projected: [(x: Float, z: Float, r: UInt8, g: UInt8, b: UInt8)] = []

            if let plyPath = environment.filePathToPointCloudData,
               let points = PointCloudStorageManager.shared.loadBinaryPLY(from: plyPath) {
                let stride = max(1, points.count / 200_000)
                projected.reserveCapacity(points.count / stride)
                for i in Swift.stride(from: 0, to: points.count, by: stride) {
                    let p = points[i]
                    projected.append((x: p.position.x, z: p.position.z, r: p.color.x, g: p.color.y, b: p.color.z))
                }
            } else if let meshPath = environment.meshPath ?? environment.objPath {
                let url = docDir.appendingPathComponent(meshPath)
                if let scene = try? SCNScene(url: url) {
                    projected = Self.extractProjectedVertices(from: scene)
                }
            }

            guard !projected.isEmpty else { return }
            let rendered = Self.renderTopDown(points: projected, imageSize: 600)
            await MainActor.run { self.image = rendered }
        }
    }

    private static func extractProjectedVertices(from scene: SCNScene) -> [(x: Float, z: Float, r: UInt8, g: UInt8, b: UInt8)] {
        var result: [(x: Float, z: Float, r: UInt8, g: UInt8, b: UInt8)] = []
        scene.rootNode.enumerateChildNodes { node, _ in
            guard let geometry = node.geometry else { return }
            guard let posSource = geometry.sources.first(where: { $0.semantic == .vertex }) else { return }

            let count = posSource.vectorCount
            let data = posSource.data
            let dataStride = posSource.dataStride
            let offset = posSource.dataOffset
            let transform = node.worldTransform

            data.withUnsafeBytes { rawBuffer in
                guard let base = rawBuffer.baseAddress else { return }
                for i in 0..<count {
                    let ptr = base + i * dataStride + offset
                    let x = ptr.load(as: Float.self)
                    let y = ptr.load(fromByteOffset: 4, as: Float.self)
                    let z = ptr.load(fromByteOffset: 8, as: Float.self)

                    let tx = transform.m11 * x + transform.m21 * y + transform.m31 * z + transform.m41
                    let tz = transform.m13 * x + transform.m23 * y + transform.m33 * z + transform.m43

                    result.append((x: tx, z: tz, r: 90, g: 160, b: 255))
                }
            }
        }
        return result
    }

    private static func renderTopDown(points: [(x: Float, z: Float, r: UInt8, g: UInt8, b: UInt8)], imageSize: Int) -> UIImage? {
        var minX: Float = .greatestFiniteMagnitude, maxX: Float = -.greatestFiniteMagnitude
        var minZ: Float = .greatestFiniteMagnitude, maxZ: Float = -.greatestFiniteMagnitude
        for p in points {
            minX = min(minX, p.x); maxX = max(maxX, p.x)
            minZ = min(minZ, p.z); maxZ = max(maxZ, p.z)
        }

        let rangeX = maxX - minX, rangeZ = maxZ - minZ
        guard rangeX > 0 || rangeZ > 0 else { return nil }

        let pad: Float = 0.08
        let pMinX = minX - rangeX * pad, pMaxX = maxX + rangeX * pad
        let pMinZ = minZ - rangeZ * pad, pMaxZ = maxZ + rangeZ * pad
        let pRangeX = pMaxX - pMinX, pRangeZ = pMaxZ - pMinZ

        let aspect = pRangeX / pRangeZ
        let width: Int, height: Int
        if aspect > 1 {
            width = imageSize; height = max(1, Int(Float(imageSize) / aspect))
        } else {
            height = imageSize; width = max(1, Int(Float(imageSize) * aspect))
        }

        // Accumulation buffer: sum of r,g,b and count per pixel
        var accR = [Float](repeating: 0, count: width * height)
        var accG = [Float](repeating: 0, count: width * height)
        var accB = [Float](repeating: 0, count: width * height)
        var accN = [Int](repeating: 0, count: width * height)

        let brushRadius = 1
        for p in points {
            let nx = (p.x - pMinX) / pRangeX
            let nz = (p.z - pMinZ) / pRangeZ
            let cx = Int(nx * Float(width - 1))
            let cy = Int(nz * Float(height - 1))

            for dy in -brushRadius...brushRadius {
                for dx in -brushRadius...brushRadius {
                    let px = cx + dx, py = cy + dy
                    guard px >= 0, px < width, py >= 0, py < height else { continue }
                    let idx = py * width + px
                    accR[idx] += Float(p.r)
                    accG[idx] += Float(p.g)
                    accB[idx] += Float(p.b)
                    accN[idx] += 1
                }
            }
        }

        var pixels = [UInt8](repeating: 0, count: width * height * 4)
        for i in 0..<(width * height) {
            let idx = i * 4
            if accN[i] > 0 {
                let n = Float(accN[i])
                pixels[idx]     = UInt8(min(255, accR[i] / n))
                pixels[idx + 1] = UInt8(min(255, accG[i] / n))
                pixels[idx + 2] = UInt8(min(255, accB[i] / n))
            } else {
                pixels[idx] = 25; pixels[idx + 1] = 25; pixels[idx + 2] = 30
            }
            pixels[idx + 3] = 255
        }

        let colorSpace = CGColorSpaceCreateDeviceRGB()
        guard let provider = CGDataProvider(data: Data(pixels) as CFData),
              let cgImage = CGImage(
                  width: width, height: height,
                  bitsPerComponent: 8, bitsPerPixel: 32,
                  bytesPerRow: width * 4,
                  space: colorSpace,
                  bitmapInfo: CGBitmapInfo(rawValue: CGImageAlphaInfo.premultipliedLast.rawValue),
                  provider: provider, decode: nil,
                  shouldInterpolate: true, intent: .defaultIntent
              ) else { return nil }

        return UIImage(cgImage: cgImage)
    }
}

struct ScanPreviewSheet: View {
    let environment: EnvironmentModel
    @Environment(\.dismiss) private var dismiss
    @State private var sceneLoadState: SceneLoadState = .loading

    enum SceneLoadState {
        case loading
        case loaded(SCNScene)
        case failed(String)
    }

    var body: some View {
        NavigationStack {
            ZStack {
                Color(UIColor.systemBackground).edgesIgnoringSafeArea(.all)

                switch sceneLoadState {
                case .loading:
                    ProgressView("Loading 3D model...")
                case .loaded(let scene):
                    TopDownSceneView(scene: scene)
                        .edgesIgnoringSafeArea(.bottom)
                case .failed(let message):
                    VStack(spacing: 12) {
                        Image(systemName: "cube.transparent")
                            .font(.system(size: 48))
                            .foregroundColor(.secondary)
                        Text(message)
                            .foregroundColor(.secondary)
                            .multilineTextAlignment(.center)
                    }
                    .padding()
                }
            }
            .navigationTitle(environment.name)
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .navigationBarTrailing) {
                    Button("Done") { dismiss() }
                }
            }
        }
        .onAppear(perform: loadScene)
    }

    private func loadScene() {
        guard let docDir = FileManager.default.cloudDocumentsURL else {
            sceneLoadState = .failed("Could not access documents directory.")
            return
        }

        Task.detached(priority: .userInitiated) {
            if let path = environment.meshPath {
                let url = docDir.appendingPathComponent(path)
                if FileManager.default.fileExists(atPath: url.path),
                   let scene = try? SCNScene(url: url) {
                    await MainActor.run { sceneLoadState = .loaded(scene) }
                    return
                }
            }

            if let path = environment.objPath {
                let url = docDir.appendingPathComponent(path)
                if FileManager.default.fileExists(atPath: url.path),
                   let scene = try? SCNScene(url: url) {
                    await MainActor.run { sceneLoadState = .loaded(scene) }
                    return
                }
            }

            if let path = environment.filePathToPointCloudData {
                if let points = PointCloudStorageManager.shared.loadBinaryPLY(from: path) {
                    let scene = Self.buildPointCloudScene(from: points)
                    await MainActor.run { sceneLoadState = .loaded(scene) }
                    return
                }
            }

            await MainActor.run { sceneLoadState = .failed("No 3D model found for this scan.") }
        }
    }

    private static func buildPointCloudScene(from points: [ColoredPoint]) -> SCNScene {
        let scene = SCNScene()

        let stride = max(1, points.count / 500_000)
        var positions: [SCNVector3] = []
        var colors: [CGFloat] = []
        positions.reserveCapacity(points.count / stride)
        colors.reserveCapacity(points.count / stride * 4)

        for i in Swift.stride(from: 0, to: points.count, by: stride) {
            let p = points[i]
            positions.append(SCNVector3(p.position.x, p.position.y, p.position.z))
            colors.append(CGFloat(p.color.x) / 255.0)
            colors.append(CGFloat(p.color.y) / 255.0)
            colors.append(CGFloat(p.color.z) / 255.0)
            colors.append(1.0)
        }

        let positionSource = SCNGeometrySource(vertices: positions)
        let colorData = Data(bytes: colors.map { Float($0) }, count: colors.count * MemoryLayout<Float>.size)
        let colorSource = SCNGeometrySource(
            data: colorData,
            semantic: .color,
            vectorCount: positions.count,
            usesFloatComponents: true,
            componentsPerVector: 4,
            bytesPerComponent: MemoryLayout<Float>.size,
            dataOffset: 0,
            dataStride: MemoryLayout<Float>.size * 4
        )

        let indices = (0..<Int32(positions.count)).map { $0 }
        let indexData = Data(bytes: indices, count: indices.count * MemoryLayout<Int32>.size)
        let element = SCNGeometryElement(
            data: indexData,
            primitiveType: .point,
            primitiveCount: positions.count,
            bytesPerIndex: MemoryLayout<Int32>.size
        )
        element.pointSize = 2
        element.minimumPointScreenSpaceRadius = 2
        element.maximumPointScreenSpaceRadius = 5

        let geometry = SCNGeometry(sources: [positionSource, colorSource], elements: [element])
        let material = SCNMaterial()
        material.lightingModel = .constant
        geometry.materials = [material]

        let node = SCNNode(geometry: geometry)
        scene.rootNode.addChildNode(node)

        return scene
    }
}

struct TopDownSceneView: UIViewRepresentable {
    let scene: SCNScene

    func makeUIView(context: Context) -> SCNView {
        let scnView = SCNView()
        scnView.scene = scene
        scnView.allowsCameraControl = true
        scnView.autoenablesDefaultLighting = true
        scnView.backgroundColor = .secondarySystemBackground

        let (minBound, maxBound) = scene.rootNode.boundingBox
        let center = SCNVector3(
            (minBound.x + maxBound.x) / 2,
            (minBound.y + maxBound.y) / 2,
            (minBound.z + maxBound.z) / 2
        )
        let sizeX = maxBound.x - minBound.x
        let sizeY = maxBound.y - minBound.y
        let sizeZ = maxBound.z - minBound.z
        let maxDim = max(sizeX, max(sizeY, sizeZ))

        let cameraNode = SCNNode()
        let camera = SCNCamera()
        camera.usesOrthographicProjection = true
        camera.orthographicScale = Double(max(sizeX, sizeZ) * 0.7)
        camera.zNear = 0.01
        camera.zFar = Double(maxDim * 10)
        cameraNode.camera = camera
        cameraNode.position = SCNVector3(center.x, center.y + maxDim * 2, center.z)
        cameraNode.look(at: center)

        scene.rootNode.addChildNode(cameraNode)
        scnView.pointOfView = cameraNode

        let floor = SCNFloor()
        floor.reflectivity = 0
        let floorMaterial = SCNMaterial()
        floorMaterial.diffuse.contents = UIColor.systemGray5
        floor.materials = [floorMaterial]
        let floorNode = SCNNode(geometry: floor)
        floorNode.position = SCNVector3(0, minBound.y - 0.001, 0)
        scene.rootNode.addChildNode(floorNode)

        return scnView
    }

    func updateUIView(_ uiView: SCNView, context: Context) {}
}

#Preview {
    ContentView()
        .modelContainer(for: EnvironmentModel.self, inMemory: true)
}
