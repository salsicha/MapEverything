//
//  RadioObservationMessageSchema.swift
//  MapEverything
//

import Foundation

struct RadioObservationMessageField: Identifiable, Codable, Hashable {
    let name: String
    let type: String
    let description: String
    let unsetValue: String

    var id: String { name }

    var rosMessage: [String: Any] {
        [
            "name": name,
            "type": type,
            "description": description,
            "unset_value": unsetValue
        ]
    }
}

final class RadioObservationMessageSchema {
    static let shared = RadioObservationMessageSchema()

    let messageType = "reconstructor_msgs/msg/RadioObservation"
    let packageName = "reconstructor_msgs"
    let messageName = "RadioObservation"
    let topic = "/mapping/radio"
    let schemaVersion = 1
    let dependencies = [
        "std_msgs/msg/Header",
        "geometry_msgs/msg/Point"
    ]
    let supportedChannelIDs = RadioTelemetryChannelID.allCases.map(\.rawValue).sorted()
    let unsetNumericValue = "0.0"
    let unsetStringValue = ""
    let unsetArrayValue = "[]"
    let unsetBooleanValue = "false"
    let fields: [RadioObservationMessageField]

    private init() {
        fields = Self.defaultFields
    }

    var messageDefinition: String {
        """
        std_msgs/Header header
        string session_id
        uint32 schema_version
        string channel_id
        string observation_kind
        string source_api
        string source_id
        string radio_type
        string[] tags

        bool map_position_available
        geometry_msgs/Point map_position
        bool geodetic_position_available
        float64 latitude
        float64 longitude
        float64 altitude

        string ssid
        string bssid
        float64 signal_strength_normalized
        bool is_secure

        string peripheral_id
        string local_name
        string[] service_uuids
        bool is_connectable

        string network_status
        string[] interface_types
        bool is_expensive
        bool is_constrained

        float64 frequency_hz
        float64 rssi_dbm
        float64 snr_db
        float64 quality

        float64 round_trip_time_ms
        float64 throughput_bytes_per_second
        bool success
        string error

        string metadata_json
        """
    }

    var rosMessage: [String: Any] {
        [
            "package": packageName,
            "message_name": messageName,
            "message_type": messageType,
            "topic": topic,
            "schema_version": schemaVersion,
            "dependencies": dependencies,
            "supported_channel_ids": supportedChannelIDs,
            "unset_values": [
                "numeric": unsetNumericValue,
                "string": unsetStringValue,
                "array": unsetArrayValue,
                "boolean": unsetBooleanValue
            ],
            "fields": fields.map(\.rosMessage),
            "msg_definition": messageDefinition
        ]
    }

    private static let defaultFields: [RadioObservationMessageField] = [
        RadioObservationMessageField(
            name: "header",
            type: "std_msgs/Header",
            description: "Observation timestamp and reporting frame; use earth for geodetic samples or map/base_link for local samples.",
            unsetValue: "required"
        ),
        RadioObservationMessageField(
            name: "session_id",
            type: "string",
            description: "Mapping session UUID that ties the radio observation to /mapping/session.",
            unsetValue: ""
        ),
        RadioObservationMessageField(
            name: "schema_version",
            type: "uint32",
            description: "RadioObservation schema version emitted by MapEverything.",
            unsetValue: "1"
        ),
        RadioObservationMessageField(
            name: "channel_id",
            type: "string",
            description: "Radio telemetry channel identifier, such as current_wifi_network, ble_advertisement, network_path, recorder_latency_probe, or external_adapter.",
            unsetValue: ""
        ),
        RadioObservationMessageField(
            name: "observation_kind",
            type: "string",
            description: "Specific observation subtype, such as wifi_current_network, ble_advertisement, network_path_state, recorder_latency, recorder_throughput, or external_sample.",
            unsetValue: ""
        ),
        RadioObservationMessageField(
            name: "source_api",
            type: "string",
            description: "Apple public API, probe mechanism, or external adapter that produced the observation.",
            unsetValue: ""
        ),
        RadioObservationMessageField(
            name: "source_id",
            type: "string",
            description: "Stable source identity when available, such as BSSID, peripheral UUID, recorder URL, interface type, or adapter ID.",
            unsetValue: ""
        ),
        RadioObservationMessageField(
            name: "radio_type",
            type: "string",
            description: "Radio family or measurement group: wifi, ble, network_path, recorder_probe, cellular, spectrum, or external.",
            unsetValue: ""
        ),
        RadioObservationMessageField(
            name: "tags",
            type: "string[]",
            description: "Free-form labels for recorder-side filtering and experiment annotation.",
            unsetValue: "[]"
        ),
        RadioObservationMessageField(
            name: "map_position_available",
            type: "bool",
            description: "True when map_position contains a synchronized position in the frame named by header.frame_id.",
            unsetValue: "false"
        ),
        RadioObservationMessageField(
            name: "map_position",
            type: "geometry_msgs/Point",
            description: "Optional local map position at observation time.",
            unsetValue: "x/y/z = 0.0"
        ),
        RadioObservationMessageField(
            name: "geodetic_position_available",
            type: "bool",
            description: "True when latitude, longitude, and altitude contain a synchronized WGS84 position.",
            unsetValue: "false"
        ),
        RadioObservationMessageField(
            name: "latitude",
            type: "float64",
            description: "Optional WGS84 latitude in degrees.",
            unsetValue: "0.0"
        ),
        RadioObservationMessageField(
            name: "longitude",
            type: "float64",
            description: "Optional WGS84 longitude in degrees.",
            unsetValue: "0.0"
        ),
        RadioObservationMessageField(
            name: "altitude",
            type: "float64",
            description: "Optional WGS84 altitude in meters.",
            unsetValue: "0.0"
        ),
        RadioObservationMessageField(
            name: "ssid",
            type: "string",
            description: "Current associated Wi-Fi SSID when available.",
            unsetValue: ""
        ),
        RadioObservationMessageField(
            name: "bssid",
            type: "string",
            description: "Current associated Wi-Fi BSSID when available.",
            unsetValue: ""
        ),
        RadioObservationMessageField(
            name: "signal_strength_normalized",
            type: "float64",
            description: "iOS-normalized current Wi-Fi signal strength in the 0.0 to 1.0 range.",
            unsetValue: "0.0"
        ),
        RadioObservationMessageField(
            name: "is_secure",
            type: "bool",
            description: "Current Wi-Fi security state when reported by iOS.",
            unsetValue: "false"
        ),
        RadioObservationMessageField(
            name: "peripheral_id",
            type: "string",
            description: "CoreBluetooth peripheral identifier for BLE advertisement samples.",
            unsetValue: ""
        ),
        RadioObservationMessageField(
            name: "local_name",
            type: "string",
            description: "BLE advertised local name or peripheral name.",
            unsetValue: ""
        ),
        RadioObservationMessageField(
            name: "service_uuids",
            type: "string[]",
            description: "BLE service UUIDs observed in advertisement, overflow, solicited, or service data fields.",
            unsetValue: "[]"
        ),
        RadioObservationMessageField(
            name: "is_connectable",
            type: "bool",
            description: "BLE advertisement connectability flag when present.",
            unsetValue: "false"
        ),
        RadioObservationMessageField(
            name: "network_status",
            type: "string",
            description: "Network.framework path status such as satisfied, unsatisfied, or requires_connection.",
            unsetValue: ""
        ),
        RadioObservationMessageField(
            name: "interface_types",
            type: "string[]",
            description: "Active or available Network.framework interface labels.",
            unsetValue: "[]"
        ),
        RadioObservationMessageField(
            name: "is_expensive",
            type: "bool",
            description: "Network.framework expensive path flag.",
            unsetValue: "false"
        ),
        RadioObservationMessageField(
            name: "is_constrained",
            type: "bool",
            description: "Network.framework constrained path flag.",
            unsetValue: "false"
        ),
        RadioObservationMessageField(
            name: "frequency_hz",
            type: "float64",
            description: "Carrier, channel, or adapter frequency in hertz when known.",
            unsetValue: "0.0"
        ),
        RadioObservationMessageField(
            name: "rssi_dbm",
            type: "float64",
            description: "Raw received signal strength in dBm for BLE or external adapters.",
            unsetValue: "0.0"
        ),
        RadioObservationMessageField(
            name: "snr_db",
            type: "float64",
            description: "Signal-to-noise ratio in dB for external adapters.",
            unsetValue: "0.0"
        ),
        RadioObservationMessageField(
            name: "quality",
            type: "float64",
            description: "Normalized or provider-defined radio quality score.",
            unsetValue: "0.0"
        ),
        RadioObservationMessageField(
            name: "round_trip_time_ms",
            type: "float64",
            description: "Recorder endpoint WebSocket ping/pong round-trip time in milliseconds.",
            unsetValue: "0.0"
        ),
        RadioObservationMessageField(
            name: "throughput_bytes_per_second",
            type: "float64",
            description: "Bounded recorder endpoint upload write-rate probe in bytes per second.",
            unsetValue: "0.0"
        ),
        RadioObservationMessageField(
            name: "success",
            type: "bool",
            description: "True when the probe or source measurement succeeded.",
            unsetValue: "false"
        ),
        RadioObservationMessageField(
            name: "error",
            type: "string",
            description: "Source, permission, probe, or adapter error text.",
            unsetValue: ""
        ),
        RadioObservationMessageField(
            name: "metadata_json",
            type: "string",
            description: "Compact JSON object for channel-specific fields that do not belong in the stable schema.",
            unsetValue: "{}"
        )
    ]
}
