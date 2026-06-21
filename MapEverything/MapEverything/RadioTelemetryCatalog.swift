//
//  RadioTelemetryCatalog.swift
//  MapEverything
//

import Foundation

enum RadioTelemetryChannelID: String, CaseIterable, Codable, Hashable {
    case currentWiFiNetwork = "current_wifi_network"
    case bleAdvertisement = "ble_advertisement"
    case networkPath = "network_path"
    case recorderLatencyProbe = "recorder_latency_probe"
    case externalAdapter = "external_adapter"
}

struct RadioTelemetryChannelDefinition: Identifiable, Codable, Hashable {
    let id: RadioTelemetryChannelID
    let displayName: String
    let sourceAPI: String
    let defaultRateHz: Double?
    let requiredCapabilities: [String]
    let observationFields: [String]
    let limitations: [String]
    let isPublicAPIBacked: Bool

    var rosMessage: [String: Any] {
        [
            "id": id.rawValue,
            "display_name": displayName,
            "source_api": sourceAPI,
            "default_rate_hz": defaultRateHz.map { String($0) } ?? "",
            "required_capabilities": requiredCapabilities,
            "observation_fields": observationFields,
            "limitations": limitations,
            "public_api_backed": isPublicAPIBacked
        ]
    }
}

final class RadioTelemetryCatalog {
    static let shared = RadioTelemetryCatalog()

    let channels: [RadioTelemetryChannelDefinition]

    init(channels: [RadioTelemetryChannelDefinition] = RadioTelemetryCatalog.defaultChannels) {
        self.channels = channels
    }

    var rosMessage: [[String: Any]] {
        channels.map(\.rosMessage)
    }

    private static let defaultChannels: [RadioTelemetryChannelDefinition] = [
        RadioTelemetryChannelDefinition(
            id: .currentWiFiNetwork,
            displayName: "Current Wi-Fi Network",
            sourceAPI: "NetworkExtension.NEHotspotNetwork.fetchCurrent",
            defaultRateHz: 0.5,
            requiredCapabilities: [
                "Location permission",
                "Access WiFi Information entitlement"
            ],
            observationFields: [
                "ssid",
                "bssid",
                "signal_strength_normalized",
                "is_secure",
                "did_auto_join",
                "did_just_join",
                "timestamp"
            ],
            limitations: [
                "Reports only the current associated Wi-Fi network.",
                "Does not provide broad Wi-Fi scan results.",
                "Signal strength is normalized by iOS rather than raw RSSI dBm."
            ],
            isPublicAPIBacked: true
        ),
        RadioTelemetryChannelDefinition(
            id: .bleAdvertisement,
            displayName: "BLE Advertisement RSSI",
            sourceAPI: "CoreBluetooth.CBCentralManager",
            defaultRateHz: 2.0,
            requiredCapabilities: [
                "Bluetooth permission",
                "Configured service UUIDs or known peripheral filters"
            ],
            observationFields: [
                "peripheral_id",
                "local_name",
                "service_uuids",
                "rssi_dbm",
                "advertisement_data",
                "timestamp"
            ],
            limitations: [
                "Scanning should be scoped to configured beacons or peripherals.",
                "Background scan behavior is limited by iOS policy."
            ],
            isPublicAPIBacked: true
        ),
        RadioTelemetryChannelDefinition(
            id: .networkPath,
            displayName: "Network Path State",
            sourceAPI: "Network.NWPathMonitor",
            defaultRateHz: 1.0,
            requiredCapabilities: [
                "Local network permission when connecting to a local recorder"
            ],
            observationFields: [
                "status",
                "interface_types",
                "available_interfaces",
                "is_expensive",
                "is_constrained",
                "supports_ipv4",
                "supports_ipv6",
                "supports_dns",
                "unsatisfied_reason",
                "timestamp"
            ],
            limitations: [
                "Reports path and interface characteristics, not RF signal power."
            ],
            isPublicAPIBacked: true
        ),
        RadioTelemetryChannelDefinition(
            id: .recorderLatencyProbe,
            displayName: "Recorder Latency Probe",
            sourceAPI: "URLSessionWebSocketTask and Network.framework",
            defaultRateHz: 0.5,
            requiredCapabilities: [
                "Reachable ROS2 recorder endpoint"
            ],
            observationFields: [
                "recorder_url",
                "round_trip_time_ms",
                "success",
                "error",
                "timestamp"
            ],
            limitations: [
                "Measures application-path latency to the recorder, not raw link-layer latency."
            ],
            isPublicAPIBacked: true
        ),
        RadioTelemetryChannelDefinition(
            id: .externalAdapter,
            displayName: "External Radio Adapter",
            sourceAPI: "ExternalAccessory, BLE, local network, or companion ROS2 node",
            defaultRateHz: nil,
            requiredCapabilities: [
                "Configured external sensor or adapter integration"
            ],
            observationFields: [
                "adapter_id",
                "radio_type",
                "frequency_hz",
                "rssi_dbm",
                "snr_db",
                "quality",
                "metadata",
                "timestamp"
            ],
            limitations: [
                "Requires hardware-specific integration.",
                "Used for cellular, spectrum, or router metrics unavailable through normal iOS APIs."
            ],
            isPublicAPIBacked: false
        )
    ]
}
