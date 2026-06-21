//
//  ROS2BridgeTransport.swift
//  Reconstructor
//

import Foundation

enum ROS2BridgeTransportKind: String, Codable {
    case rosbridgeWebSocket = "rosbridge_websocket"
}

struct ROS2BridgeTransportProfile: Codable, Equatable {
    let kind: ROS2BridgeTransportKind
    let displayName: String
    let messageEncoding: String
    let urlSchemes: [String]
    let supportsNativeDDS: Bool
    let supportsBinaryHighRateStreams: Bool
    let decision: String
    let rationale: String
    let upgradePath: String

    var rosMessage: [String: Any] {
        [
            "kind": kind.rawValue,
            "display_name": displayName,
            "message_encoding": messageEncoding,
            "url_schemes": urlSchemes,
            "supports_native_dds": supportsNativeDDS,
            "supports_binary_high_rate_streams": supportsBinaryHighRateStreams,
            "decision": decision,
            "rationale": rationale,
            "upgrade_path": upgradePath
        ]
    }

    var diagnosticValues: [String: String] {
        [
            "transport_kind": kind.rawValue,
            "transport_display_name": displayName,
            "message_encoding": messageEncoding,
            "url_schemes": urlSchemes.joined(separator: ","),
            "supports_native_dds": String(supportsNativeDDS),
            "supports_binary_high_rate_streams": String(supportsBinaryHighRateStreams),
            "transport_decision": decision,
            "transport_rationale": rationale,
            "transport_upgrade_path": upgradePath
        ]
    }

    static let current = ROS2BridgeTransportProfile(
        kind: .rosbridgeWebSocket,
        displayName: "rosbridge WebSocket",
        messageEncoding: "rosbridge JSON envelopes over WebSocket data frames",
        urlSchemes: ["ws", "wss"],
        supportsNativeDDS: false,
        supportsBinaryHighRateStreams: false,
        decision: "Continue using rosbridge for this build.",
        rationale: "No native ROS2/DDS iOS client or recorder-side binary receiver is integrated in the project. High-rate streams are managed with compression, throttling, backpressure, and transient retry buffers.",
        upgradePath: "Revisit a native binary bridge after measured rosbridge throughput fails field requirements and a maintained iOS client or companion ROS2 binary receiver is selected."
    )
}
