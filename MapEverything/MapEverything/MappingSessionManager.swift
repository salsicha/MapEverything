//
//  MappingSessionManager.swift
//  MapEverything
//

import Foundation
import Combine

enum MappingSensorStream: String, CaseIterable, Identifiable, Codable, Hashable {
    case pose
    case tf
    case imu
    case camera
    case pointCloud
    case mesh
    case gps
    case radio
    case indoorLocalization
    case satelliteImagery
    case dem
    case diagnostics
    case session

    var id: String { rawValue }

    var displayName: String {
        switch self {
        case .pose: return "Pose"
        case .tf: return "TF"
        case .imu: return "IMU"
        case .camera: return "Camera"
        case .pointCloud: return "Point Cloud"
        case .mesh: return "Mesh"
        case .gps: return "GPS"
        case .radio: return "Radio"
        case .indoorLocalization: return "Indoor Localization"
        case .satelliteImagery: return "Satellite Imagery"
        case .dem: return "DEM"
        case .diagnostics: return "Diagnostics"
        case .session: return "Session"
        }
    }
}

enum MappingSessionState: Equatable {
    case idle
    case connecting
    case active
    case failed(String)

    var label: String {
        switch self {
        case .idle: return "Idle"
        case .connecting: return "Connecting"
        case .active: return "Active"
        case .failed: return "Failed"
        }
    }
}

struct MappingSessionSnapshot {
    let event: String
    let sessionID: UUID?
    let state: String
    let recorderURL: String
    let enabledStreams: [String]
    let startedAt: Date?
    let endedAt: Date?
    let lastError: String?
}

@MainActor
final class MappingSessionManager: ObservableObject {
    static let shared = MappingSessionManager()

    @Published private(set) var sessionID: UUID?
    @Published private(set) var state: MappingSessionState = .idle
    @Published private(set) var recorderURL: String
    @Published private(set) var enabledStreams: Set<MappingSensorStream>
    @Published private(set) var startedAt: Date?
    @Published private(set) var endedAt: Date?
    @Published private(set) var lastError: String?

    private let bridge: ROS2BridgeClient
    private let geoTilePublisher: GeoTilePublisher
    private let indoorLocalizationManager: IndoorLocalizationManager
    private let currentWiFiTelemetryManager: CurrentWiFiTelemetryManager
    private let bleBeaconTelemetryManager: BLEBeaconTelemetryManager
    private let networkPathDiagnosticsManager: NetworkPathDiagnosticsManager
    private let recorderEndpointProbeManager: RecorderEndpointProbeManager
    private let radioObservationPublisher: RadioObservationPublisher

    var isActive: Bool {
        state == .active
    }

    var sessionMetadata: [String: String] {
        var metadata: [String: String] = [
            "recorder_url": recorderURL,
            "bridge_transport": ROS2BridgeTransportProfile.current.kind.rawValue,
            "bridge_transport_decision": ROS2BridgeTransportProfile.current.decision,
            "state": state.label,
            "active_mapping_mode": AdaptiveMappingModeController.shared.activeMode.rawValue,
            "adaptive_mapping_confidence": String(format: "%.3f", AdaptiveMappingModeController.shared.recommendation.confidence),
            "adaptive_mapping_operator_override": AdaptiveMappingModeController.shared.operatorOverride.rawValue,
            "adaptive_mapping_reasons": AdaptiveMappingModeController.shared.recommendation.reasons.map(\.rawValue).joined(separator: ","),
            "enabled_streams": enabledStreams
                .map(\.rawValue)
                .sorted()
                .joined(separator: ",")
        ]

        if let sessionID {
            metadata["session_id"] = sessionID.uuidString
        }
        if let startedAt {
            metadata["started_at"] = ISO8601DateFormatter().string(from: startedAt)
        }
        if let endedAt {
            metadata["ended_at"] = ISO8601DateFormatter().string(from: endedAt)
        }
        if let lastError {
            metadata["last_error"] = lastError
        }

        return metadata
    }

    init(
        bridge: ROS2BridgeClient? = nil,
        geoTilePublisher: GeoTilePublisher? = nil,
        indoorLocalizationManager: IndoorLocalizationManager? = nil,
        currentWiFiTelemetryManager: CurrentWiFiTelemetryManager? = nil,
        bleBeaconTelemetryManager: BLEBeaconTelemetryManager? = nil,
        networkPathDiagnosticsManager: NetworkPathDiagnosticsManager? = nil,
        recorderEndpointProbeManager: RecorderEndpointProbeManager? = nil,
        radioObservationPublisher: RadioObservationPublisher? = nil,
        recorderURL: String = "ws://192.168.1.100:9090",
        enabledStreams: Set<MappingSensorStream>? = nil
    ) {
        self.bridge = bridge ?? ROS2BridgeClient.shared
        self.geoTilePublisher = geoTilePublisher ?? GeoTilePublisher.shared
        self.indoorLocalizationManager = indoorLocalizationManager ?? IndoorLocalizationManager.shared
        self.currentWiFiTelemetryManager = currentWiFiTelemetryManager ?? CurrentWiFiTelemetryManager.shared
        self.bleBeaconTelemetryManager = bleBeaconTelemetryManager ?? BLEBeaconTelemetryManager.shared
        self.networkPathDiagnosticsManager = networkPathDiagnosticsManager ?? NetworkPathDiagnosticsManager.shared
        self.recorderEndpointProbeManager = recorderEndpointProbeManager ?? RecorderEndpointProbeManager.shared
        self.radioObservationPublisher = radioObservationPublisher ?? RadioObservationPublisher.shared
        self.recorderURL = recorderURL
        self.enabledStreams = enabledStreams ?? Self.defaultStreams
    }

    func configure(recorderURL: String? = nil, enabledStreams: Set<MappingSensorStream>? = nil) {
        if let recorderURL {
            self.recorderURL = recorderURL.trimmingCharacters(in: .whitespacesAndNewlines)
        }
        if let enabledStreams {
            self.enabledStreams = enabledStreams
        }
        ROS2TopicRegistry.shared.configure(enabledStreams: self.enabledStreams)
    }

    func start(recorderURL: String? = nil) {
        configure(recorderURL: recorderURL)

        if isActive {
            return
        }

        guard isValidRecorderURL(self.recorderURL) else {
            fail("Invalid ROS2 recorder WebSocket URL.")
            return
        }

        if sessionID == nil || endedAt != nil {
            sessionID = UUID()
            startedAt = Date()
        } else if startedAt == nil {
            startedAt = Date()
        }

        endedAt = nil
        lastError = nil
        state = .connecting

        MapGeoreferencer.shared.reset()
        bridge.connect(to: self.recorderURL)
        geoTilePublisher.start()
        indoorLocalizationManager.start()
        currentWiFiTelemetryManager.start()
        bleBeaconTelemetryManager.start()
        networkPathDiagnosticsManager.start()
        recorderEndpointProbeManager.start(recorderURL: self.recorderURL)
        radioObservationPublisher.start(sessionID: sessionID)
        state = .active
        publishSessionMetadata(event: "started")
    }

    func stop() {
        endedAt = Date()
        state = .idle
        publishSessionMetadata(event: "stopped")
        geoTilePublisher.stop()
        indoorLocalizationManager.stop()
        currentWiFiTelemetryManager.stop()
        bleBeaconTelemetryManager.stop()
        networkPathDiagnosticsManager.stop()
        recorderEndpointProbeManager.stop()
        radioObservationPublisher.stop()
        bridge.disconnect(after: 0.25)
    }

    func restart(recorderURL: String? = nil) {
        stop()
        start(recorderURL: recorderURL)
    }

    func setStream(_ stream: MappingSensorStream, isEnabled: Bool) {
        if isEnabled {
            enabledStreams.insert(stream)
        } else {
            enabledStreams.remove(stream)
        }
        ROS2TopicRegistry.shared.setStream(stream, isEnabled: isEnabled)
        publishSessionMetadata(event: "streams_updated")
    }

    func isStreamEnabled(_ stream: MappingSensorStream) -> Bool {
        enabledStreams.contains(stream)
    }

    func resetSession() {
        if isActive {
            stop()
        }

        sessionID = nil
        startedAt = nil
        endedAt = nil
        lastError = nil
        enabledStreams = Self.defaultStreams
    }

    private func fail(_ message: String) {
        lastError = message
        state = .failed(message)
        publishSessionMetadata(event: "failed")
    }

    private func publishSessionMetadata(event: String) {
        guard ROS2TopicRegistry.shared.isStreamEnabled(.session) else { return }

        let snapshot = MappingSessionSnapshot(
            event: event,
            sessionID: sessionID,
            state: state.label,
            recorderURL: recorderURL,
            enabledStreams: enabledStreams.map(\.rawValue).sorted(),
            startedAt: startedAt,
            endedAt: endedAt,
            lastError: lastError
        )

        bridge.publishSessionMetadata(snapshot, timestamp: ProcessInfo.processInfo.systemUptime)
    }

    func publishSessionUpdate(event: String) {
        publishSessionMetadata(event: event)
    }

    private func isValidRecorderURL(_ value: String) -> Bool {
        guard let url = URL(string: value),
              let scheme = url.scheme?.lowercased(),
              ["ws", "wss"].contains(scheme),
              url.host != nil else {
            return false
        }
        return true
    }

    private static let defaultStreams: Set<MappingSensorStream> = [
        .pose,
        .tf,
        .imu,
        .camera,
        .pointCloud,
        .mesh,
        .gps,
        .radio,
        .satelliteImagery,
        .dem,
        .diagnostics,
        .session,
        .indoorLocalization
    ]
}
