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
import UIKit

enum VisualizationMode: String, CaseIterable, Identifiable {
    case solidMesh = "Solid Mesh"
    case surfels = "Surfels"
    case wireframe = "Wireframe"
    case none = "None"
    var id: Self { self }

    var iconName: String {
        switch self {
        case .solidMesh: return "cube.fill"
        case .surfels: return "circle.dotted"
        case .wireframe: return "square.grid.3x3"
        case .none: return "eye.slash"
        }
    }
}

struct ContentView: View {
    private static let topControlsHorizontalPadding: CGFloat = 14
    private static let actionRailTrailingPadding: CGFloat = 28
    private static let topControlsTopPadding: CGFloat = 54
    private static let topControlsGap: CGFloat = 8
    private static let actionRailButtonSize: CGFloat = 56
    private static let recorderPanelPreferredWidth: CGFloat = 248

    @ObservedObject private var ros2Client = ROS2BridgeClient.shared
    @ObservedObject private var mappingSession = MappingSessionManager.shared
    @ObservedObject private var localBagRecorder = LocalROS2BagRecorder.shared
    
    // App Settings
    @AppStorage("maxPointLimit") private var maxPointLimit: Int = 2_000_000
    @AppStorage("voxelSize") private var voxelSize: Double = 0.05
    @AppStorage("boundingBoxSize") private var boundingBoxSize: Double = 20.0
    @AppStorage("ros2Enabled") private var ros2Enabled: Bool = false
    @AppStorage("ros2WebSocketURL") private var ros2WebSocketURL: String = "ws://192.168.1.100:9090"
    @AppStorage(LocalROS2BagRecorderConfiguration.enabledStorageKey) private var localROS2BagStorageEnabled: Bool = false
    
    @State private var visualizationMode: VisualizationMode = .solidMesh
    @State private var isScanning = false
    @State private var pointCount = 0
    
    @State private var trackingFeedback: String = ""
    @State private var errorMessage: String? = nil

    @State private var showLocalBagBrowser = false
    @State private var hasCameraPermission = false
    @State private var stoppedInspectionScene: SCNScene?
    @State private var isPreparingMapper = true
    @State private var isDepthAnythingReady = false
    @State private var shouldMountARView = false
    @State private var rosBridgeHostInput = ""
    private let checksCameraPermission: Bool

    init(checksCameraPermission: Bool = true, previewHasCameraPermission: Bool? = nil) {
        self.checksCameraPermission = checksCameraPermission
        _hasCameraPermission = State(initialValue: previewHasCameraPermission ?? false)
    }
    
    var body: some View {
        Group {
            if hasCameraPermission {
                mainScannerView
            } else {
                permissionDeniedView
            }
        }
        .onAppear {
            guard checksCameraPermission else { return }
            checkCameraPermission()
        }
    }
    
    private var mainScannerView: some View {
        NavigationStack {
            ZStack(alignment: .bottom) {
                if shouldMountARView {
                    ARViewContainer(
                        visualizationMode: $visualizationMode,
                        isScanning: $isScanning,
                        stoppedInspectionScene: $stoppedInspectionScene,
                        isPreparingMapper: $isPreparingMapper,
                        isDepthAnythingReady: $isDepthAnythingReady,
                        pointCount: $pointCount,
                        trackingFeedback: $trackingFeedback,
                        errorMessage: $errorMessage,
                        maxPointLimit: maxPointLimit,
                        voxelSize: Float(voxelSize),
                        boundingBoxSize: Float(boundingBoxSize)
                    )
                    .edgesIgnoringSafeArea(.all)
                } else {
                    Color.black
                        .ignoresSafeArea()
                }

                mapperStartupOverlay
                stoppedMapInspectionOverlay
                rosPublishingPanel
                topControlsOverlay
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
                visualizationMode = .solidMesh
                mappingSession.configure(
                    recorderURL: ros2WebSocketURL,
                    remoteStreamingEnabled: ros2Enabled
                )
                syncROSBridgeHostInput()
                mountARViewAfterStartupPaint()
            }
            .onChange(of: ros2WebSocketURL) { _ in
                syncROSBridgeHostInput()
            }
            .onChange(of: isScanning) { scanning in
                UIApplication.shared.isIdleTimerDisabled = scanning
            }
            .onChange(of: ros2Enabled) { enabled in
                commitROSBridgeHostInput(reconnectIfActive: false)
                mappingSession.configure(
                    recorderURL: ros2WebSocketURL,
                    remoteStreamingEnabled: enabled
                )
                if isScanning {
                    mappingSession.restart(
                        recorderURL: ros2WebSocketURL,
                        remoteStreamingEnabled: enabled
                    )
                }
            }
            .sheet(isPresented: $showLocalBagBrowser) {
                LocalROS2BagBrowserView()
            }
        }
    }

    private var scannerActionRail: some View {
        VStack(spacing: 10) {
            startButton
            localBagButton
            shareLocalBagsButton
        }
    }

    private var topControlsOverlay: some View {
        GeometryReader { proxy in
            let leadingInset = Self.topControlsHorizontalPadding + proxy.safeAreaInsets.leading
            let trailingInset = Self.actionRailTrailingPadding + proxy.safeAreaInsets.trailing
            let topInset = Self.topControlsTopPadding
            let availableWidth = max(
                0,
                proxy.size.width - leadingInset - trailingInset
            )
            let recorderWidth = min(
                Self.recorderPanelPreferredWidth,
                max(
                    0,
                    availableWidth - Self.actionRailButtonSize - Self.topControlsGap
                )
            )

            ZStack(alignment: .topLeading) {
                recorderDiagnosticsPanel(width: recorderWidth)
                    .allowsHitTesting(false)
                    .padding(.top, topInset)
                    .padding(.leading, leadingInset)
                    .frame(
                        width: proxy.size.width,
                        height: proxy.size.height,
                        alignment: .topLeading
                    )

                scannerActionRail
                    .frame(width: Self.actionRailButtonSize, alignment: .topTrailing)
                    .fixedSize(horizontal: true, vertical: false)
                    .padding(.top, topInset)
                    .padding(.trailing, trailingInset)
                    .frame(
                        width: proxy.size.width,
                        height: proxy.size.height,
                        alignment: .topTrailing
                    )
            }
            .frame(width: proxy.size.width, height: proxy.size.height, alignment: .topLeading)
        }
    }

    @ViewBuilder
    private var mapperStartupOverlay: some View {
        if !isScanning, stoppedInspectionScene == nil {
            ZStack {
                Color.black.opacity(0.82)
                    .ignoresSafeArea()

                VStack(spacing: 12) {
                    Image(systemName: isPreparingMapper ? "cube.transparent" : "viewfinder")
                        .font(.system(size: 44, weight: .semibold))
                    if isPreparingMapper {
                        ProgressView()
                            .tint(.white)
                    }
                    Text(isPreparingMapper ? "Preparing Mapper" : readinessLabel)
                        .font(.headline.weight(.semibold))
                    if isPreparingMapper {
                        Text("Warming Depth Anything")
                            .font(.caption)
                            .foregroundColor(.white.opacity(0.72))
                    }
                }
                .foregroundColor(.white)
                .shadow(color: .black.opacity(0.45), radius: 14, x: 0, y: 6)
            }
            .allowsHitTesting(false)
        }
    }

    private var readinessLabel: String {
        isDepthAnythingReady ? "Ready" : "Ready Without Depth Model"
    }

    @ViewBuilder
    private var stoppedMapInspectionOverlay: some View {
        if !isScanning, let scene = stoppedInspectionScene {
            GeometryReader { proxy in
                TopDownSceneView(scene: scene)
                    .frame(
                        width: min(proxy.size.width - 32, 560),
                        height: min(proxy.size.height * 0.58, 520)
                    )
                    .clipShape(RoundedRectangle(cornerRadius: 8, style: .continuous))
                    .overlay(
                        RoundedRectangle(cornerRadius: 8, style: .continuous)
                            .stroke(Color.white.opacity(0.2), lineWidth: 1)
                    )
                    .shadow(color: Color.black.opacity(0.35), radius: 18, x: 0, y: 8)
                    .position(x: proxy.size.width / 2, y: proxy.size.height / 2)
            }
            .transition(.opacity.combined(with: .scale(scale: 0.98)))
        }
    }

    private var startButton: some View {
        Button(action: toggleMapping) {
            actionRailIcon(
                systemName: isScanning ? "stop.fill" : "play.fill",
                foregroundColor: .white,
                backgroundColor: isScanning ? .red : .green
            )
        }
        .accessibilityLabel(isScanning ? "Stop Mapping" : "Start Mapping")
        .buttonStyle(.plain)
        .disabled(!isScanning && isPreparingMapper)
        .opacity(!isScanning && isPreparingMapper ? 0.55 : 1)
    }

    private var localBagButton: some View {
        Button(action: toggleLocalBagStorage) {
            actionRailIcon(
                systemName: localROS2BagStorageEnabled ? "externaldrive.badge.checkmark" : "externaldrive.badge.plus",
                foregroundColor: localROS2BagStorageEnabled ? .white : .primary,
                backgroundColor: localROS2BagStorageEnabled ? .blue : nil
            )
        }
        .accessibilityLabel(localROS2BagStorageEnabled ? "Disable Save Local" : "Enable Save Local")
        .buttonStyle(.plain)
    }

    private var shareLocalBagsButton: some View {
        Button {
            showLocalBagBrowser = true
        } label: {
            actionRailIcon(
                systemName: "square.and.arrow.up",
                foregroundColor: .primary,
                backgroundColor: nil
            )
        }
        .accessibilityLabel("Share Local Bags")
        .buttonStyle(.plain)
    }

    private func actionRailIcon(
        systemName: String,
        foregroundColor: Color,
        backgroundColor: Color?
    ) -> some View {
        Image(systemName: systemName)
            .font(.title3.weight(.bold))
            .foregroundColor(foregroundColor)
            .frame(width: Self.actionRailButtonSize, height: Self.actionRailButtonSize)
            .background(backgroundColor ?? Color.clear)
            .background(.ultraThinMaterial)
            .clipShape(Circle())
            .overlay(
                Circle()
                    .stroke(Color.white.opacity(0.18), lineWidth: 1)
            )
            .shadow(color: Color.black.opacity(0.35), radius: 10, x: 0, y: 5)
    }

    private var rosPublishingPanel: some View {
        let topics = publishedTopicDefinitions

        return VStack(alignment: .leading, spacing: 8) {
            HStack(spacing: 10) {
                Toggle(isOn: $ros2Enabled) {
                    Text("ROS")
                        .font(.caption.weight(.semibold))
                }
                .toggleStyle(.switch)
                .fixedSize()

                HStack(spacing: 6) {
                    Image(systemName: "network")
                        .font(.caption.weight(.semibold))
                        .foregroundColor(.secondary)
                    TextField("ROS bridge IP", text: $rosBridgeHostInput)
                        .textInputAutocapitalization(.never)
                        .autocorrectionDisabled()
                        .keyboardType(.numbersAndPunctuation)
                        .submitLabel(.done)
                        .font(.caption.monospacedDigit())
                        .onSubmit {
                            commitROSBridgeHostInput(reconnectIfActive: true)
                        }
                    Button {
                        commitROSBridgeHostInput(reconnectIfActive: true)
                    } label: {
                        Image(systemName: "checkmark.circle.fill")
                            .font(.caption.weight(.semibold))
                    }
                    .buttonStyle(.plain)
                    .accessibilityLabel("Apply ROS bridge address")
                }
                .padding(.horizontal, 9)
                .padding(.vertical, 7)
                .frame(minWidth: 170, maxWidth: 260)
                .background(Color.black.opacity(0.18))
                .clipShape(RoundedRectangle(cornerRadius: 8, style: .continuous))
                .overlay(
                    RoundedRectangle(cornerRadius: 8, style: .continuous)
                        .stroke(Color.white.opacity(0.14), lineWidth: 1)
                )

                Spacer(minLength: 6)

                Label(ros2Client.isConnected ? "Connected" : mappingSession.state.label, systemImage: ros2Client.isConnected ? "dot.radiowaves.left.and.right" : "link")
                    .font(.caption2.weight(.medium))
                    .foregroundColor(ros2Client.isConnected ? .green : .secondary)
                    .lineLimit(1)
            }

            HStack(spacing: 8) {
                Text(isScanning ? "Publishing" : "Topics")
                    .font(.caption2.weight(.semibold))
                    .foregroundColor(.secondary)
                ScrollView(.horizontal, showsIndicators: false) {
                    HStack(spacing: 6) {
                        ForEach(topics) { definition in
                            topicChip(definition)
                        }
                    }
                }
            }
        }
        .padding(.horizontal, 12)
        .padding(.vertical, 10)
        .frame(maxWidth: 720, alignment: .leading)
        .background(.ultraThinMaterial)
        .clipShape(RoundedRectangle(cornerRadius: 8, style: .continuous))
        .overlay(
            RoundedRectangle(cornerRadius: 8, style: .continuous)
                .stroke(Color.white.opacity(0.18), lineWidth: 1)
        )
        .shadow(color: Color.black.opacity(0.3), radius: 12, x: 0, y: 5)
        .padding(.horizontal, 12)
        .padding(.bottom, 12)
    }

    private var publishedTopicDefinitions: [ROS2TopicDefinition] {
        ROS2TopicRegistry.shared.advertisedTopics()
    }

    private func topicChip(_ definition: ROS2TopicDefinition) -> some View {
        Text(definition.topic)
            .font(.caption2.monospaced())
            .lineLimit(1)
            .padding(.horizontal, 8)
            .padding(.vertical, 4)
            .background(Color.black.opacity(0.18))
            .clipShape(Capsule())
            .overlay(
                Capsule()
                    .stroke(Color.white.opacity(0.14), lineWidth: 1)
            )
    }

    private func recorderDiagnosticsPanel(width: CGFloat) -> some View {
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

            HStack(spacing: 10) {
                diagnosticsMetric("\(queueStats.depth)/\(queueStats.capacity)", systemImage: "tray.full", minWidth: 56)
                diagnosticsMetric("\(queueStats.droppedMessages)", systemImage: "exclamationmark.triangle", minWidth: 34)
                diagnosticsMetric(
                    "\(localBufferStats.pointCloudSamples + localBufferStats.meshSamples)",
                    systemImage: "externaldrive",
                    minWidth: 66
                )
                if localBagStats.isEnabled {
                    diagnosticsMetric(
                        "\(localBagStats.messageCount)",
                        systemImage: "externaldrive.badge.plus",
                        minWidth: 66
                    )
                }
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
        .frame(width: width, alignment: .leading)
        .background(.ultraThinMaterial)
        .clipShape(RoundedRectangle(cornerRadius: 8, style: .continuous))
        .overlay(
            RoundedRectangle(cornerRadius: 8, style: .continuous)
                .stroke(Color.white.opacity(0.18), lineWidth: 1)
        )
        .shadow(color: Color.black.opacity(0.25), radius: 8, x: 0, y: 4)
    }

    private func diagnosticsMetric(
        _ value: String,
        systemImage: String,
        minWidth: CGFloat
    ) -> some View {
        HStack(spacing: 5) {
            Image(systemName: systemImage)
                .frame(width: 14)
            Text(value)
                .lineLimit(1)
                .minimumScaleFactor(0.8)
                .fixedSize(horizontal: true, vertical: false)
                .layoutPriority(1)
        }
        .frame(minWidth: minWidth, alignment: .leading)
    }

    private func toggleMapping() {
        UIImpactFeedbackGenerator(style: .medium).impactOccurred()

        if isScanning {
            isScanning = false
            mappingSession.stop()
        } else {
            guard shouldMountARView, !isPreparingMapper else { return }
            commitROSBridgeHostInput(reconnectIfActive: false)
            stoppedInspectionScene = nil
            isScanning = true
            mappingSession.start(
                recorderURL: ros2WebSocketURL,
                remoteStreamingEnabled: ros2Enabled
            )
        }
    }

    private func toggleLocalBagStorage() {
        UIImpactFeedbackGenerator(style: .light).impactOccurred()
        localROS2BagStorageEnabled.toggle()
        mappingSession.refreshLocalBagRecording()
    }

    private func syncROSBridgeHostInput() {
        let host = rosBridgeHost(from: ros2WebSocketURL)
        guard !host.isEmpty, rosBridgeHostInput != host else { return }
        rosBridgeHostInput = host
    }

    private func commitROSBridgeHostInput(reconnectIfActive: Bool) {
        let updatedURL = rosBridgeURL(fromHostInput: rosBridgeHostInput)
        guard !updatedURL.isEmpty else { return }

        ros2WebSocketURL = updatedURL
        syncROSBridgeHostInput()
        mappingSession.configure(
            recorderURL: updatedURL,
            remoteStreamingEnabled: ros2Enabled
        )

        if reconnectIfActive, isScanning, ros2Enabled {
            mappingSession.restart(
                recorderURL: updatedURL,
                remoteStreamingEnabled: true
            )
        }
    }

    private func rosBridgeURL(fromHostInput input: String) -> String {
        let current = rosBridgeEndpoint(from: ros2WebSocketURL)
        guard let typed = rosBridgeEndpoint(from: input) else {
            return ros2WebSocketURL
        }

        let scheme = typed.scheme ?? current?.scheme ?? "ws"
        let host = typed.host ?? current?.host ?? "192.168.1.100"
        let port = typed.port ?? current?.port ?? 9090
        let path = typed.path ?? current?.path ?? ""

        var components = URLComponents()
        components.scheme = scheme
        components.host = host
        components.port = port
        components.path = path == "/" ? "" : path
        return components.string ?? "\(scheme)://\(host):\(port)"
    }

    private func rosBridgeHost(from urlString: String) -> String {
        rosBridgeEndpoint(from: urlString)?.host ?? ""
    }

    private func rosBridgeEndpoint(from rawValue: String) -> (scheme: String?, host: String?, port: Int?, path: String?)? {
        let trimmed = rawValue.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !trimmed.isEmpty else { return nil }

        if trimmed.contains("://"),
           let components = URLComponents(string: trimmed),
           let host = components.host {
            return (
                scheme: components.scheme,
                host: host,
                port: components.port,
                path: components.path.isEmpty ? nil : components.path
            )
        }

        let parts = trimmed.split(separator: "/", maxSplits: 1, omittingEmptySubsequences: false)
        let hostPort = String(parts[0])
        let path = parts.count > 1 ? "/" + String(parts[1]) : nil
        let colonCount = hostPort.filter { $0 == ":" }.count

        if colonCount == 1 {
            let endpointParts = hostPort.split(separator: ":", maxSplits: 1, omittingEmptySubsequences: false)
            let host = String(endpointParts[0])
            let port = endpointParts.count > 1 ? Int(endpointParts[1]) : nil
            return (scheme: nil, host: host, port: port, path: path)
        }

        return (scheme: nil, host: hostPort, port: nil, path: path)
    }

    private func mountARViewAfterStartupPaint() {
        guard !shouldMountARView else { return }

        DispatchQueue.main.asyncAfter(deadline: .now() + 0.25) {
            shouldMountARView = true
        }
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

struct LocalROS2BagBrowserView: View {
    @ObservedObject private var recorder = LocalROS2BagRecorder.shared
    @Environment(\.dismiss) private var dismiss
    @State private var sessions: [LocalROS2BagSession] = []
    @State private var pendingDeletion: LocalROS2BagSession?
    @State private var errorMessage: String?
    @State private var previewScanTask: Task<Void, Never>?

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
                        Task { await reload() }
                    } label: {
                        Image(systemName: "arrow.clockwise")
                    }
                    .accessibilityLabel("Refresh")
                }
            }
            .task {
                await reload()
            }
            .refreshable {
                await reload()
            }
            .onDisappear {
                previewScanTask?.cancel()
                previewScanTask = nil
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
        HStack(spacing: 12) {
            LocalROS2BagSessionThumbnail(session: session)

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

                if let preview = session.preview {
                    Text("\(preview.messageCountLabel) • \(preview.durationLabel) • \(preview.topicSummary)")
                        .font(.caption2)
                        .foregroundColor(.secondary)
                        .lineLimit(1)
                }
            }
        }
    }

    @MainActor
    private func reload() async {
        previewScanTask?.cancel()
        previewScanTask = nil

        do {
            let loadedSessions = try await recorder.listBagSessionsAsync(previewLoadingMode: .cachedOnly)
            guard !Task.isCancelled else { return }
            sessions = loadedSessions
            scanMissingPreviews(for: loadedSessions)
        } catch {
            errorMessage = error.localizedDescription
        }
    }

    @MainActor
    private func scanMissingPreviews(for loadedSessions: [LocalROS2BagSession]) {
        let missingPreviews = loadedSessions.filter { $0.preview == nil }
        guard !missingPreviews.isEmpty else { return }

        previewScanTask = Task {
            for session in missingPreviews {
                guard !Task.isCancelled else { return }

                do {
                    let scannedSession = try await recorder.bagSessionWithPreviewScan(session)
                    guard !Task.isCancelled else { return }

                    await MainActor.run {
                        if let index = sessions.firstIndex(where: { $0.id == scannedSession.id }) {
                            sessions[index] = scannedSession
                        }
                    }
                } catch {
                    await MainActor.run {
                        errorMessage = error.localizedDescription
                    }
                }
            }
        }
    }

    private func deletePendingSession() {
        guard let pendingDeletion else { return }
        do {
            try recorder.deleteBagSession(pendingDeletion)
            self.pendingDeletion = nil
            Task { await reload() }
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
            if let preview = session.preview {
                Section("Preview") {
                    HStack(spacing: 14) {
                        LocalROS2BagSessionThumbnail(session: session, size: 88)
                        VStack(alignment: .leading, spacing: 6) {
                            LabeledContent("Messages", value: preview.messageCount.formatted())
                            LabeledContent("Duration", value: preview.durationLabel)
                            LabeledContent("Topics", value: "\(preview.topicNames.count)")
                        }
                    }
                    Text(preview.topicSummary)
                        .font(.caption)
                        .foregroundColor(.secondary)
                        .lineLimit(2)
                }
            }

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
                if !session.files.isEmpty {
                    ShareLink(items: session.files.map(\.url)) {
                        Label("Share Bag Files", systemImage: "square.and.arrow.up")
                    }
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

struct LocalROS2BagSessionThumbnail: View {
    let session: LocalROS2BagSession
    var size: CGFloat = 56

    var body: some View {
        Group {
            if let image = thumbnailImage {
                Image(uiImage: image)
                    .resizable()
                    .scaledToFill()
            } else {
                ZStack {
                    LinearGradient(
                        colors: [Color(.systemBlue), Color(.systemTeal), Color(.systemGray3)],
                        startPoint: .topLeading,
                        endPoint: .bottomTrailing
                    )
                    Image(systemName: "map")
                        .font(.system(size: size * 0.36, weight: .semibold))
                        .foregroundColor(.white)
                }
            }
        }
        .frame(width: size, height: size)
        .clipShape(RoundedRectangle(cornerRadius: 8, style: .continuous))
        .overlay(
            RoundedRectangle(cornerRadius: 8, style: .continuous)
                .stroke(Color.primary.opacity(0.08), lineWidth: 1)
        )
        .accessibilityHidden(true)
    }

    private var thumbnailImage: UIImage? {
        guard let url = session.preview?.thumbnailURL(relativeTo: session.directoryURL) else { return nil }
        return UIImage(contentsOfFile: url.path)
    }
}

struct TopDownSceneView: UIViewRepresentable {
    let scene: SCNScene

    func makeUIView(context: Context) -> SCNView {
        let scnView = SCNView()
        scnView.scene = scene
        scnView.allowsCameraControl = true
        scnView.autoenablesDefaultLighting = false
        scnView.backgroundColor = .secondarySystemBackground

        removeExistingInspectionViewerNodes(from: scene)

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
        let sceneRadius = max(maxDim, 1.0)

        let cameraNode = SCNNode()
        cameraNode.name = "inspection_camera"
        let camera = SCNCamera()
        camera.usesOrthographicProjection = true
        camera.orthographicScale = Double(max(max(sizeX, sizeZ) * 0.7, sceneRadius * 0.45))
        camera.zNear = 0.01
        camera.zFar = Double(sceneRadius * 10)
        cameraNode.camera = camera
        cameraNode.position = SCNVector3(center.x, center.y + sceneRadius * 2, center.z)
        cameraNode.look(at: center)

        scene.rootNode.addChildNode(cameraNode)
        scnView.pointOfView = cameraNode
        addInspectionLighting(to: scene, center: center, radius: sceneRadius)

        let floorSize = CGFloat(max(sceneRadius * 2.2, 1.0))
        let floor = SCNPlane(width: floorSize, height: floorSize)
        let floorMaterial = SCNMaterial()
        floorMaterial.diffuse.contents = UIColor.systemGray5
        floor.materials = [floorMaterial]
        let floorNode = SCNNode(geometry: floor)
        floorNode.name = "inspection_floor"
        floorNode.position = SCNVector3(0, minBound.y - 0.001, 0)
        floorNode.eulerAngles.x = -.pi / 2
        scene.rootNode.addChildNode(floorNode)

        return scnView
    }

    func updateUIView(_ uiView: SCNView, context: Context) {}

    private func removeExistingInspectionViewerNodes(from scene: SCNScene) {
        [
            "inspection_camera",
            "inspection_floor",
            "inspection_ambient_light",
            "inspection_key_light",
            "inspection_fill_light"
        ].forEach { name in
            scene.rootNode.childNode(withName: name, recursively: false)?.removeFromParentNode()
        }
    }

    private func addInspectionLighting(to scene: SCNScene, center: SCNVector3, radius: Float) {
        let ambient = SCNLight()
        ambient.type = .ambient
        ambient.intensity = 260
        ambient.color = UIColor(white: 0.86, alpha: 1.0)
        let ambientNode = SCNNode()
        ambientNode.name = "inspection_ambient_light"
        ambientNode.light = ambient
        scene.rootNode.addChildNode(ambientNode)

        let key = SCNLight()
        key.type = .directional
        key.intensity = 820
        key.castsShadow = true
        key.shadowRadius = 8
        key.shadowSampleCount = 8
        key.color = UIColor(white: 1.0, alpha: 1.0)
        let keyNode = SCNNode()
        keyNode.name = "inspection_key_light"
        keyNode.light = key
        keyNode.position = SCNVector3(
            center.x - radius * 0.6,
            center.y + radius * 1.4,
            center.z + radius * 0.8
        )
        keyNode.look(at: center)
        scene.rootNode.addChildNode(keyNode)

        let fill = SCNLight()
        fill.type = .directional
        fill.intensity = 260
        fill.color = UIColor(white: 0.78, alpha: 1.0)
        let fillNode = SCNNode()
        fillNode.name = "inspection_fill_light"
        fillNode.light = fill
        fillNode.position = SCNVector3(
            center.x + radius * 0.9,
            center.y + radius * 0.9,
            center.z - radius * 0.7
        )
        fillNode.look(at: center)
        scene.rootNode.addChildNode(fillNode)
    }
}

#if DEBUG
private enum MapEverythingPreviewSupport {
    static let modelContainer: ModelContainer = {
        let schema = MapEverythingModelSchema.schema
        let configuration = ModelConfiguration(schema: schema, isStoredInMemoryOnly: true)

        do {
            return try ModelContainer(for: schema, configurations: [configuration])
        } catch {
            fatalError("Could not create preview ModelContainer: \(error)")
        }
    }()
}

#Preview("MapEverything Canvas") {
    ContentView(checksCameraPermission: false, previewHasCameraPermission: false)
        .modelContainer(MapEverythingPreviewSupport.modelContainer)
}
#endif
