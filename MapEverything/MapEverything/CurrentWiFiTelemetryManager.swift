//
//  CurrentWiFiTelemetryManager.swift
//  MapEverything
//

import Combine
import CoreLocation
import Foundation
import NetworkExtension

struct CurrentWiFiTelemetrySample: Equatable {
    let ssid: String
    let bssid: String
    let signalStrength: Double
    let isSecure: Bool
    let didAutoJoin: Bool
    let didJustJoin: Bool
    let timestamp: Date

    var qualityLabel: String {
        switch signalStrength {
        case 0.75...:
            return "high"
        case 0.45..<0.75:
            return "medium"
        case 0.20..<0.45:
            return "low"
        default:
            return "poor"
        }
    }

    var rosMessage: [String: Any] {
        [
            "ssid": ssid,
            "bssid": bssid,
            "signal_strength_normalized": signalStrength,
            "signal_quality_label": qualityLabel,
            "is_secure": isSecure,
            "did_auto_join": didAutoJoin,
            "did_just_join": didJustJoin,
            "timestamp": ISO8601DateFormatter().string(from: timestamp)
        ]
    }
}

final class CurrentWiFiTelemetryManager: NSObject, ObservableObject, CLLocationManagerDelegate {
    struct Configuration {
        let fetchInterval: TimeInterval

        static let `default` = Configuration(fetchInterval: 2.0)
    }

    static let shared = CurrentWiFiTelemetryManager()

    @Published private(set) var isRunning = false
    @Published private(set) var entitlementConfigured: Bool
    @Published private(set) var locationAuthorizationStatusLabel = "not_determined"
    @Published private(set) var accuracyAuthorizationLabel = "unknown"
    @Published private(set) var lastSample: CurrentWiFiTelemetrySample?
    @Published private(set) var lastFetchAt: Date?
    @Published private(set) var lastError: String?

    private let configuration: Configuration
    private let locationManager: CLLocationManager
    private var fetchTimer: DispatchSourceTimer?

    var sessionMetadata: [String: Any] {
        var metadata: [String: Any] = [
            "source_api": "NetworkExtension.NEHotspotNetwork.fetchCurrent",
            "requires_access_wifi_information_entitlement": true,
            "access_wifi_information_entitlement_configured": entitlementConfigured,
            "requires_location_permission": true,
            "location_authorization": locationAuthorizationStatusLabel,
            "accuracy_authorization": accuracyAuthorizationLabel,
            "running": isRunning,
            "last_fetch_at": lastFetchAt.map { ISO8601DateFormatter().string(from: $0) } ?? "",
            "last_error": lastError ?? "",
            "limitations": [
                "current_associated_network_only",
                "normalized_signal_strength_only",
                "no_broad_wifi_scans"
            ]
        ]

        if let lastSample {
            metadata["last_sample"] = lastSample.rosMessage
        }

        return metadata
    }

    var diagnosticLevel: Int {
        if !isRunning { return 1 }
        if !entitlementConfigured { return 1 }
        if locationAuthorizationStatusLabel == "denied"
            || locationAuthorizationStatusLabel == "restricted" {
            return 1
        }
        if locationAuthorizationStatusLabel == "not_determined" { return 1 }
        if lastSample == nil { return 1 }
        return 0
    }

    var diagnosticMessage: String {
        if !isRunning {
            return "Current Wi-Fi telemetry is not running"
        }
        if !entitlementConfigured {
            return "Access WiFi Information entitlement is not configured"
        }
        if locationAuthorizationStatusLabel == "denied"
            || locationAuthorizationStatusLabel == "restricted" {
            return "Location permission is required for current Wi-Fi signal quality"
        }
        if locationAuthorizationStatusLabel == "not_determined" {
            return "Waiting for location permission before reading current Wi-Fi signal quality"
        }
        if let lastSample {
            return "Current Wi-Fi signal quality available: \(lastSample.qualityLabel)"
        }
        return lastError ?? "Current Wi-Fi network unavailable"
    }

    var diagnosticValues: [String: String] {
        var values: [String: String] = [
            "running": String(isRunning),
            "source_api": "NetworkExtension.NEHotspotNetwork.fetchCurrent",
            "access_wifi_information_entitlement_configured": String(entitlementConfigured),
            "location_authorization": locationAuthorizationStatusLabel,
            "accuracy_authorization": accuracyAuthorizationLabel,
            "fetch_interval_seconds": String(format: "%.1f", configuration.fetchInterval),
            "last_fetch_at": lastFetchAt.map { ISO8601DateFormatter().string(from: $0) } ?? "",
            "last_error": lastError ?? "",
            "limitations": "current_associated_network_only,normalized_signal_strength_only,no_broad_wifi_scans"
        ]

        if let lastSample {
            values["ssid"] = lastSample.ssid
            values["bssid"] = lastSample.bssid
            values["signal_strength_normalized"] = String(format: "%.3f", lastSample.signalStrength)
            values["signal_quality_label"] = lastSample.qualityLabel
            values["is_secure"] = String(lastSample.isSecure)
            values["did_auto_join"] = String(lastSample.didAutoJoin)
            values["did_just_join"] = String(lastSample.didJustJoin)
            values["sample_timestamp"] = ISO8601DateFormatter().string(from: lastSample.timestamp)
        }

        return values
    }

    init(
        configuration: Configuration = .default,
        locationManager: CLLocationManager = CLLocationManager(),
        entitlementConfigured: Bool = true
    ) {
        self.configuration = configuration
        self.locationManager = locationManager
        self.entitlementConfigured = entitlementConfigured
        super.init()

        locationManager.delegate = self
        updateAuthorizationState()
    }

    func start() {
        guard !isRunning else { return }

        isRunning = true
        lastError = nil
        updateAuthorizationState()
        requestLocationPermissionIfNeeded()
        scheduleFetchTimer()
        fetchCurrentNetworkIfPossible()
    }

    func stop() {
        isRunning = false
        fetchTimer?.cancel()
        fetchTimer = nil
    }

    func locationManagerDidChangeAuthorization(_ manager: CLLocationManager) {
        updateAuthorizationState()
        if isRunning {
            fetchCurrentNetworkIfPossible()
        }
    }

    private func scheduleFetchTimer() {
        fetchTimer?.cancel()

        let timer = DispatchSource.makeTimerSource(queue: .main)
        timer.schedule(deadline: .now() + configuration.fetchInterval, repeating: configuration.fetchInterval)
        timer.setEventHandler { [weak self] in
            self?.fetchCurrentNetworkIfPossible()
        }
        fetchTimer = timer
        timer.resume()
    }

    private func fetchCurrentNetworkIfPossible() {
        guard isRunning else { return }

        updateAuthorizationState()

        guard entitlementConfigured else {
            lastSample = nil
            lastError = "Access WiFi Information entitlement is required for NEHotspotNetwork.fetchCurrent."
            return
        }

        guard hasLocationPermission else {
            lastSample = nil
            lastError = "Location permission is required for current Wi-Fi signal quality."
            requestLocationPermissionIfNeeded()
            return
        }

        #if targetEnvironment(simulator)
        lastFetchAt = Date()
        lastSample = nil
        lastError = "Current Wi-Fi telemetry is unavailable in the iOS Simulator."
        #else
        NEHotspotNetwork.fetchCurrent { [weak self] network in
            DispatchQueue.main.async {
                guard let self else { return }

                let timestamp = Date()
                self.lastFetchAt = timestamp

                guard let network else {
                    self.lastSample = nil
                    self.lastError = "No current Wi-Fi network was reported by iOS."
                    return
                }

                self.lastSample = CurrentWiFiTelemetrySample(
                    ssid: network.ssid,
                    bssid: network.bssid,
                    signalStrength: network.signalStrength,
                    isSecure: network.isSecure,
                    didAutoJoin: network.didAutoJoin,
                    didJustJoin: network.didJustJoin,
                    timestamp: timestamp
                )
                self.lastError = nil
            }
        }
        #endif
    }

    private var hasLocationPermission: Bool {
        switch locationManager.authorizationStatus {
        case .authorizedAlways, .authorizedWhenInUse:
            return true
        case .denied, .restricted, .notDetermined:
            return false
        @unknown default:
            return false
        }
    }

    private func requestLocationPermissionIfNeeded() {
        updateAuthorizationState()

        switch locationManager.authorizationStatus {
        case .notDetermined:
            locationManager.requestWhenInUseAuthorization()
        case .denied, .restricted:
            lastError = "Location permission is required for current Wi-Fi signal quality."
        case .authorizedAlways, .authorizedWhenInUse:
            break
        @unknown default:
            break
        }
    }

    private func updateAuthorizationState() {
        locationAuthorizationStatusLabel = label(for: locationManager.authorizationStatus)
        accuracyAuthorizationLabel = label(for: locationManager.accuracyAuthorization)
    }

    private func label(for status: CLAuthorizationStatus) -> String {
        switch status {
        case .notDetermined:
            return "not_determined"
        case .restricted:
            return "restricted"
        case .denied:
            return "denied"
        case .authorizedAlways:
            return "authorized_always"
        case .authorizedWhenInUse:
            return "authorized_when_in_use"
        @unknown default:
            return "unknown"
        }
    }

    private func label(for authorization: CLAccuracyAuthorization) -> String {
        switch authorization {
        case .fullAccuracy:
            return "full_accuracy"
        case .reducedAccuracy:
            return "reduced_accuracy"
        @unknown default:
            return "unknown"
        }
    }
}
