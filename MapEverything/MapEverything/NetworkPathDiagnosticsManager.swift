//
//  NetworkPathDiagnosticsManager.swift
//  MapEverything
//

import Combine
import Foundation
import Network

struct NetworkPathDiagnosticsSample: Equatable {
    let status: String
    let interfaceTypes: [String]
    let availableInterfaces: [String]
    let isExpensive: Bool
    let isConstrained: Bool
    let supportsIPv4: Bool
    let supportsIPv6: Bool
    let supportsDNS: Bool
    let unsatisfiedReason: String
    let timestamp: Date

    var rosMessage: [String: Any] {
        [
            "status": status,
            "interface_types": interfaceTypes,
            "available_interfaces": availableInterfaces,
            "is_expensive": isExpensive,
            "is_constrained": isConstrained,
            "supports_ipv4": supportsIPv4,
            "supports_ipv6": supportsIPv6,
            "supports_dns": supportsDNS,
            "unsatisfied_reason": unsatisfiedReason,
            "timestamp": ISO8601DateFormatter().string(from: timestamp)
        ]
    }
}

final class NetworkPathDiagnosticsManager: ObservableObject {
    static let shared = NetworkPathDiagnosticsManager()

    @Published private(set) var isRunning = false
    @Published private(set) var lastSample: NetworkPathDiagnosticsSample?
    @Published private(set) var lastError: String?

    private let monitorQueue = DispatchQueue(label: "com.mapeverything.networkPathMonitor", qos: .utility)
    private var monitor: NWPathMonitor?

    var sessionMetadata: [String: Any] {
        var metadata: [String: Any] = [
            "source_api": "Network.NWPathMonitor",
            "running": isRunning,
            "requires_local_network_permission_for_local_recorders": true,
            "last_error": lastError ?? "",
            "limitations": [
                "reports_path_characteristics_not_rf_signal_power",
                "local_network_permission_is_prompted_by_local_endpoint_connections"
            ]
        ]

        if let lastSample {
            metadata["last_sample"] = lastSample.rosMessage
        }

        return metadata
    }

    var diagnosticLevel: Int {
        guard isRunning else { return 1 }
        guard let lastSample else { return 1 }

        switch lastSample.status {
        case "satisfied":
            return 0
        case "requires_connection":
            return 1
        default:
            return 2
        }
    }

    var diagnosticMessage: String {
        guard isRunning else {
            return "Network path diagnostics are not running"
        }
        guard let lastSample else {
            return lastError ?? "Waiting for Network.framework path update"
        }

        switch lastSample.status {
        case "satisfied":
            let interfaces = lastSample.interfaceTypes.isEmpty
                ? "unknown interface"
                : lastSample.interfaceTypes.joined(separator: ",")
            return "Network path satisfied via \(interfaces)"
        case "requires_connection":
            return "Network path requires a connection"
        default:
            if lastSample.unsatisfiedReason.isEmpty {
                return "Network path is unsatisfied"
            }
            return "Network path is unsatisfied: \(lastSample.unsatisfiedReason)"
        }
    }

    var diagnosticValues: [String: String] {
        var values: [String: String] = [
            "running": String(isRunning),
            "source_api": "Network.NWPathMonitor",
            "last_error": lastError ?? "",
            "requires_local_network_permission_for_local_recorders": "true"
        ]

        if let lastSample {
            values["status"] = lastSample.status
            values["interface_types"] = lastSample.interfaceTypes.joined(separator: ",")
            values["available_interfaces"] = lastSample.availableInterfaces.joined(separator: ",")
            values["is_expensive"] = String(lastSample.isExpensive)
            values["is_constrained"] = String(lastSample.isConstrained)
            values["supports_ipv4"] = String(lastSample.supportsIPv4)
            values["supports_ipv6"] = String(lastSample.supportsIPv6)
            values["supports_dns"] = String(lastSample.supportsDNS)
            values["unsatisfied_reason"] = lastSample.unsatisfiedReason
            values["sample_timestamp"] = ISO8601DateFormatter().string(from: lastSample.timestamp)
        }

        return values
    }

    func start() {
        guard !isRunning else { return }

        isRunning = true
        lastError = nil

        let monitor = NWPathMonitor()
        monitor.pathUpdateHandler = { [weak self] path in
            DispatchQueue.main.async {
                self?.handlePathUpdate(path)
            }
        }
        self.monitor = monitor
        monitor.start(queue: monitorQueue)
    }

    func stop() {
        isRunning = false
        monitor?.cancel()
        monitor = nil
    }

    private func handlePathUpdate(_ path: NWPath) {
        guard isRunning else { return }

        lastSample = NetworkPathDiagnosticsSample(
            status: label(for: path.status),
            interfaceTypes: activeInterfaceTypes(for: path),
            availableInterfaces: path.availableInterfaces.map { label(for: $0.type) },
            isExpensive: path.isExpensive,
            isConstrained: path.isConstrained,
            supportsIPv4: path.supportsIPv4,
            supportsIPv6: path.supportsIPv6,
            supportsDNS: path.supportsDNS,
            unsatisfiedReason: label(for: path.unsatisfiedReason),
            timestamp: Date()
        )
        lastError = nil
    }

    private func activeInterfaceTypes(for path: NWPath) -> [String] {
        [
            NWInterface.InterfaceType.wifi,
            .cellular,
            .wiredEthernet,
            .loopback,
            .other
        ]
        .filter { path.usesInterfaceType($0) }
        .map(label(for:))
    }

    private func label(for status: NWPath.Status) -> String {
        switch status {
        case .satisfied:
            return "satisfied"
        case .unsatisfied:
            return "unsatisfied"
        case .requiresConnection:
            return "requires_connection"
        @unknown default:
            return "unknown"
        }
    }

    private func label(for interfaceType: NWInterface.InterfaceType) -> String {
        switch interfaceType {
        case .wifi:
            return "wifi"
        case .cellular:
            return "cellular"
        case .wiredEthernet:
            return "wired_ethernet"
        case .loopback:
            return "loopback"
        case .other:
            return "other"
        @unknown default:
            return "unknown"
        }
    }

    private func label(for reason: NWPath.UnsatisfiedReason) -> String {
        switch reason {
        case .notAvailable:
            return "not_available"
        case .cellularDenied:
            return "cellular_denied"
        case .wifiDenied:
            return "wifi_denied"
        case .localNetworkDenied:
            return "local_network_denied"
        case .vpnInactive:
            return "vpn_inactive"
        @unknown default:
            return "unknown"
        }
    }
}
