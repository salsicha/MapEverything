//
//  RadioObservationPublisher.swift
//  MapEverything
//

import Foundation

struct RadioObservationMessage {
    let timestamp: Date
    let frameID: String
    let deduplicationKey: String
    let fields: [String: Any]

    init(
        timestamp: Date,
        frameID: String = "iphone_camera",
        sessionID: String,
        channelID: RadioTelemetryChannelID,
        observationKind: String,
        sourceAPI: String,
        sourceID: String = "",
        radioType: String,
        tags: [String] = [],
        metadata: [String: Any] = [:],
        values: [String: Any] = [:]
    ) {
        self.timestamp = timestamp
        self.frameID = frameID
        self.deduplicationKey = [
            channelID.rawValue,
            observationKind,
            sourceID,
            String(format: "%.6f", timestamp.timeIntervalSince1970)
        ].joined(separator: ":")

        var fields = Self.defaultFields
        fields["session_id"] = sessionID
        fields["schema_version"] = RadioObservationMessageSchema.shared.schemaVersion
        fields["channel_id"] = channelID.rawValue
        fields["observation_kind"] = observationKind
        fields["source_api"] = sourceAPI
        fields["source_id"] = sourceID
        fields["radio_type"] = radioType
        fields["tags"] = tags
        fields["metadata_json"] = Self.metadataJSONString(metadata)

        for (key, value) in values {
            fields[key] = Self.sanitizedJSONValue(value)
        }

        self.fields = fields
    }

    private static let defaultFields: [String: Any] = [
        "session_id": "",
        "schema_version": RadioObservationMessageSchema.shared.schemaVersion,
        "channel_id": "",
        "observation_kind": "",
        "source_api": "",
        "source_id": "",
        "radio_type": "",
        "tags": [String](),
        "map_position_available": false,
        "map_position": ["x": 0.0, "y": 0.0, "z": 0.0],
        "geodetic_position_available": false,
        "latitude": 0.0,
        "longitude": 0.0,
        "altitude": 0.0,
        "ssid": "",
        "bssid": "",
        "signal_strength_normalized": 0.0,
        "is_secure": false,
        "peripheral_id": "",
        "local_name": "",
        "service_uuids": [String](),
        "is_connectable": false,
        "network_status": "",
        "interface_types": [String](),
        "is_expensive": false,
        "is_constrained": false,
        "frequency_hz": 0.0,
        "rssi_dbm": 0.0,
        "snr_db": 0.0,
        "quality": 0.0,
        "round_trip_time_ms": 0.0,
        "throughput_bytes_per_second": 0.0,
        "success": false,
        "error": "",
        "metadata_json": "{}"
    ]

    nonisolated private static func metadataJSONString(_ metadata: [String: Any]) -> String {
        let sanitizedMetadata = sanitizedJSONValue(metadata)
        guard JSONSerialization.isValidJSONObject(sanitizedMetadata),
              let data = try? JSONSerialization.data(withJSONObject: sanitizedMetadata, options: [.sortedKeys]),
              let jsonString = String(data: data, encoding: .utf8) else {
            return "{}"
        }
        return jsonString
    }

    nonisolated private static func sanitizedJSONValue(_ value: Any) -> Any {
        switch value {
        case let value as [String: Any]:
            return value.reduce(into: [String: Any]()) { result, entry in
                result[entry.key] = sanitizedJSONValue(entry.value)
            }
        case let value as [Any]:
            return value.map(sanitizedJSONValue)
        case let value as Date:
            return ISO8601DateFormatter().string(from: value)
        case let value as UUID:
            return value.uuidString
        case let value as Data:
            return value.base64EncodedString()
        case let value as Double:
            return value.isFinite ? value : 0.0
        case let value as Float:
            return value.isFinite ? Double(value) : 0.0
        case let value as CGFloat:
            let doubleValue = Double(value)
            return doubleValue.isFinite ? doubleValue : 0.0
        case let value as NSNumber:
            if CFGetTypeID(value) == CFBooleanGetTypeID() {
                return value.boolValue
            }
            return value.doubleValue.isFinite ? value : 0.0
        case let value as String:
            return value
        case let value as Bool:
            return value
        case let value as Int:
            return value
        case let value as UInt:
            return value
        case let value as Int64:
            return value
        case let value as UInt64:
            return value
        default:
            return String(describing: value)
        }
    }
}

struct RadioObservationTransientBuffer {
    let capacity: Int
    private(set) var observations: [RadioObservationMessage] = []
    private var observationKeys = Set<String>()

    init(capacity: Int = 200) {
        self.capacity = max(1, capacity)
    }

    var count: Int {
        observations.count
    }

    mutating func enqueue(_ observation: RadioObservationMessage) {
        guard observationKeys.insert(observation.deduplicationKey).inserted else { return }

        observations.append(observation)
        while observations.count > capacity {
            let dropped = observations.removeFirst()
            observationKeys.remove(dropped.deduplicationKey)
        }
    }

    mutating func enqueue(contentsOf newObservations: [RadioObservationMessage]) {
        newObservations.forEach { enqueue($0) }
    }

    mutating func flush() -> [RadioObservationMessage] {
        let flushedObservations = observations
        observations.removeAll()
        observationKeys.removeAll()
        return flushedObservations
    }

    mutating func removeAll() {
        observations.removeAll()
        observationKeys.removeAll()
    }
}

@MainActor
final class RadioObservationPublisher {
    struct Configuration {
        let publishInterval: TimeInterval
        let maxBufferedObservations: Int

        static let `default` = Configuration(
            publishInterval: 0.5,
            maxBufferedObservations: 200
        )
    }

    static let shared = RadioObservationPublisher()

    private let bridge: ROS2BridgeClient
    private let currentWiFiTelemetryManager: CurrentWiFiTelemetryManager
    private let bleBeaconTelemetryManager: BLEBeaconTelemetryManager
    private let networkPathDiagnosticsManager: NetworkPathDiagnosticsManager
    private let recorderEndpointProbeManager: RecorderEndpointProbeManager
    private let configuration: Configuration

    private var isRunning = false
    private var sessionID = ""
    private var publishTimer: DispatchSourceTimer?
    private var publishedObservationKeys = Set<String>()
    private var transientBuffer: RadioObservationTransientBuffer

    init(
        bridge: ROS2BridgeClient? = nil,
        currentWiFiTelemetryManager: CurrentWiFiTelemetryManager? = nil,
        bleBeaconTelemetryManager: BLEBeaconTelemetryManager? = nil,
        networkPathDiagnosticsManager: NetworkPathDiagnosticsManager? = nil,
        recorderEndpointProbeManager: RecorderEndpointProbeManager? = nil,
        configuration: Configuration? = nil
    ) {
        self.bridge = bridge ?? ROS2BridgeClient.shared
        self.currentWiFiTelemetryManager = currentWiFiTelemetryManager ?? CurrentWiFiTelemetryManager.shared
        self.bleBeaconTelemetryManager = bleBeaconTelemetryManager ?? BLEBeaconTelemetryManager.shared
        self.networkPathDiagnosticsManager = networkPathDiagnosticsManager ?? NetworkPathDiagnosticsManager.shared
        self.recorderEndpointProbeManager = recorderEndpointProbeManager ?? RecorderEndpointProbeManager.shared
        self.configuration = configuration ?? Configuration.default
        self.transientBuffer = RadioObservationTransientBuffer(
            capacity: self.configuration.maxBufferedObservations
        )
    }

    func start(sessionID: UUID?) {
        self.sessionID = sessionID?.uuidString ?? ""
        publishedObservationKeys.removeAll()
        transientBuffer.removeAll()

        guard !isRunning else {
            publishFreshObservations()
            return
        }

        isRunning = true
        schedulePublishTimer()
        publishFreshObservations()
    }

    func stop() {
        isRunning = false
        publishTimer?.cancel()
        publishTimer = nil
        publishedObservationKeys.removeAll()
        transientBuffer.removeAll()
    }

    private func schedulePublishTimer() {
        publishTimer?.cancel()

        let timer = DispatchSource.makeTimerSource(queue: .main)
        timer.schedule(deadline: .now() + configuration.publishInterval, repeating: configuration.publishInterval)
        timer.setEventHandler { [weak self] in
            self?.publishFreshObservations()
        }
        publishTimer = timer
        timer.resume()
    }

    private func publishFreshObservations() {
        guard isRunning, ROS2TopicRegistry.shared.isStreamEnabled(.radio) else { return }

        let observations = currentObservations()
        guard bridge.hasActivePublishTarget else {
            transientBuffer.enqueue(
                contentsOf: observations.filter { !publishedObservationKeys.contains($0.deduplicationKey) }
            )
            return
        }

        flushBufferedObservations()

        for observation in observations {
            guard publishedObservationKeys.insert(observation.deduplicationKey).inserted else {
                continue
            }
            bridge.publishRadioObservation(observation)
        }
    }

    private func flushBufferedObservations() {
        guard bridge.hasActivePublishTarget else { return }

        for observation in transientBuffer.flush() {
            guard publishedObservationKeys.insert(observation.deduplicationKey).inserted else {
                continue
            }
            bridge.publishRadioObservation(observation)
        }
    }

    private func currentObservations() -> [RadioObservationMessage] {
        var observations: [RadioObservationMessage] = []

        if let sample = currentWiFiTelemetryManager.lastSample {
            observations.append(wifiObservation(from: sample))
        }

        let bleSamples = bleBeaconTelemetryManager.lastSamplesByPeripheral.values
            .sorted { $0.timestamp < $1.timestamp }
        observations.append(contentsOf: bleSamples.map(bleObservation(from:)))

        if let sample = networkPathDiagnosticsManager.lastSample {
            observations.append(networkPathObservation(from: sample))
        }

        if let sample = recorderEndpointProbeManager.lastSample {
            observations.append(recorderProbeObservation(from: sample))
        }

        return observations
    }

    private func wifiObservation(from sample: CurrentWiFiTelemetrySample) -> RadioObservationMessage {
        return RadioObservationMessage(
            timestamp: sample.timestamp,
            sessionID: sessionID,
            channelID: .currentWiFiNetwork,
            observationKind: "wifi_current_network",
            sourceAPI: "NetworkExtension.NEHotspotNetwork.fetchCurrent",
            sourceID: sample.bssid.isEmpty ? sample.ssid : sample.bssid,
            radioType: "wifi",
            tags: ["wifi", "current_network"],
            metadata: [
                "did_auto_join": sample.didAutoJoin,
                "did_just_join": sample.didJustJoin,
                "signal_quality_label": sample.qualityLabel,
                "timestamp": ISO8601DateFormatter().string(from: sample.timestamp)
            ],
            values: [
                "ssid": sample.ssid,
                "bssid": sample.bssid,
                "signal_strength_normalized": sample.signalStrength,
                "is_secure": sample.isSecure,
                "quality": sample.signalStrength,
                "success": true
            ]
        )
    }

    private func bleObservation(from sample: BLEBeaconTelemetrySample) -> RadioObservationMessage {
        return RadioObservationMessage(
            timestamp: sample.timestamp,
            sessionID: sessionID,
            channelID: .bleAdvertisement,
            observationKind: "ble_advertisement",
            sourceAPI: "CoreBluetooth.CBCentralManager",
            sourceID: sample.peripheralID.uuidString,
            radioType: "ble",
            tags: ["ble", "advertisement"],
            metadata: [
                "advertisement_data": sample.advertisementData,
                "timestamp": ISO8601DateFormatter().string(from: sample.timestamp)
            ],
            values: [
                "peripheral_id": sample.peripheralID.uuidString,
                "local_name": sample.localName,
                "service_uuids": sample.serviceUUIDs,
                "is_connectable": sample.advertisementData["is_connectable"] as? Bool ?? false,
                "rssi_dbm": Double(sample.rssiDBM),
                "success": true
            ]
        )
    }

    private func networkPathObservation(from sample: NetworkPathDiagnosticsSample) -> RadioObservationMessage {
        let quality: Double
        switch sample.status {
        case "satisfied":
            quality = 1.0
        case "requires_connection":
            quality = 0.5
        default:
            quality = 0.0
        }

        return RadioObservationMessage(
            timestamp: sample.timestamp,
            sessionID: sessionID,
            channelID: .networkPath,
            observationKind: "network_path_state",
            sourceAPI: "Network.NWPathMonitor",
            sourceID: sample.interfaceTypes.joined(separator: ","),
            radioType: "network_path",
            tags: ["network_path"],
            metadata: [
                "available_interfaces": sample.availableInterfaces,
                "supports_ipv4": sample.supportsIPv4,
                "supports_ipv6": sample.supportsIPv6,
                "supports_dns": sample.supportsDNS,
                "unsatisfied_reason": sample.unsatisfiedReason,
                "timestamp": ISO8601DateFormatter().string(from: sample.timestamp)
            ],
            values: [
                "network_status": sample.status,
                "interface_types": sample.interfaceTypes,
                "is_expensive": sample.isExpensive,
                "is_constrained": sample.isConstrained,
                "quality": quality,
                "success": true
            ]
        )
    }

    private func recorderProbeObservation(from sample: RecorderEndpointProbeSample) -> RadioObservationMessage {
        return RadioObservationMessage(
            timestamp: sample.timestamp,
            sessionID: sessionID,
            channelID: .recorderLatencyProbe,
            observationKind: "recorder_endpoint_probe",
            sourceAPI: "URLSessionWebSocketTask.sendPing and rosbridge publish",
            sourceID: sample.recorderURL,
            radioType: "recorder_probe",
            tags: ["recorder", "latency", "throughput"],
            metadata: [
                "throughput_payload_bytes": sample.throughputPayloadBytes,
                "throughput_message_count": sample.throughputMessageCount,
                "throughput_elapsed_ms": sample.throughputElapsedMilliseconds,
                "timestamp": ISO8601DateFormatter().string(from: sample.timestamp)
            ],
            values: [
                "round_trip_time_ms": sample.roundTripTimeMilliseconds,
                "throughput_bytes_per_second": sample.throughputBytesPerSecond,
                "quality": sample.success ? 1.0 : 0.0,
                "success": sample.success,
                "error": sample.error
            ]
        )
    }
}
