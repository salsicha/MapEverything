//
//  IndoorLocalizationManager.swift
//  Reconstructor
//

import Foundation
import CoreLocation
import Combine

struct IndoorLocalizationSample {
    let location: CLLocation
    let heading: CLHeading?
    let indoorRegistrationQuality: Double
    let globalRegistrationQuality: Double
    let indoorQualityLabel: String
    let globalQualityLabel: String
    let timestamp: Date
}

final class IndoorLocalizationManager: NSObject, ObservableObject, CLLocationManagerDelegate {
    private static let preciseLocationPurposeKey = "RoboticsMappingPreciseLocation"

    struct Configuration {
        let publishInterval: TimeInterval
        let gpsFixPublishInterval: TimeInterval
        let distanceFilter: CLLocationDistance

        static let `default` = Configuration(
            publishInterval: 1.0,
            gpsFixPublishInterval: 1.0,
            distanceFilter: 1.0
        )
    }

    static let shared = IndoorLocalizationManager()

    @Published private(set) var isRunning = false
    @Published private(set) var lastPublishedAt: Date?
    @Published private(set) var lastGPSFixPublishedAt: Date?
    @Published private(set) var lastError: String?
    @Published private(set) var lastIndoorRegistrationQuality: Double = 0
    @Published private(set) var lastGlobalRegistrationQuality: Double = 0
    @Published private(set) var lastHorizontalAccuracy: Double = -1
    @Published private(set) var lastVerticalAccuracy: Double = -1
    @Published private(set) var authorizationStatusLabel = "not_determined"
    @Published private(set) var accuracyAuthorizationLabel = "unknown"
    @Published private(set) var isPreciseLocationAuthorized = false
    @Published private(set) var isHeadingAvailable = false
    @Published private(set) var lastHeadingAccuracy: Double = -1

    private let configuration: Configuration
    private let locationManager: CLLocationManager
    private let bridge: ROS2BridgeClient
    private let topicRegistry: ROS2TopicRegistry

    private var latestLocation: CLLocation?
    private var latestHeading: CLHeading?
    private var hasRequestedTemporaryFullAccuracy = false

    var lastLocationAgeSeconds: TimeInterval? {
        latestLocation.map { abs($0.timestamp.timeIntervalSinceNow) }
    }

    var lastHeadingAgeSeconds: TimeInterval? {
        latestHeading.map { abs($0.timestamp.timeIntervalSinceNow) }
    }

    init(
        configuration: Configuration = .default,
        locationManager: CLLocationManager = CLLocationManager(),
        bridge: ROS2BridgeClient = .shared,
        topicRegistry: ROS2TopicRegistry = .shared
    ) {
        self.configuration = configuration
        self.locationManager = locationManager
        self.bridge = bridge
        self.topicRegistry = topicRegistry
        super.init()

        locationManager.delegate = self
        locationManager.desiredAccuracy = kCLLocationAccuracyBestForNavigation
        locationManager.distanceFilter = configuration.distanceFilter
        locationManager.headingFilter = 1.0
        locationManager.pausesLocationUpdatesAutomatically = false
        isHeadingAvailable = CLLocationManager.headingAvailable()
        updateAuthorizationState()
    }

    func start() {
        guard !isRunning else { return }

        isRunning = true
        lastError = nil
        prepareLocationServices()
    }

    func stop() {
        isRunning = false
        locationManager.stopUpdatingLocation()
        if CLLocationManager.headingAvailable() {
            locationManager.stopUpdatingHeading()
        }
    }

    func locationManagerDidChangeAuthorization(_ manager: CLLocationManager) {
        updateAuthorizationState()

        switch manager.authorizationStatus {
        case .authorizedAlways, .authorizedWhenInUse:
            requestTemporaryFullAccuracyIfNeeded()
            startLocationUpdates()
        case .denied, .restricted:
            lastError = "Location permission is required for indoor and global registration."
        case .notDetermined:
            break
        @unknown default:
            break
        }
    }

    func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
        guard let location = locations.last else { return }
        latestLocation = location
        publishIfNeeded()
    }

    func locationManager(_ manager: CLLocationManager, didUpdateHeading newHeading: CLHeading) {
        latestHeading = newHeading
        lastHeadingAccuracy = newHeading.headingAccuracy
        publishIfNeeded()
    }

    func locationManager(_ manager: CLLocationManager, didFailWithError error: Error) {
        lastError = "Indoor localization update failed: \(error.localizedDescription)"
    }

    private func prepareLocationServices() {
        updateAuthorizationState()

        switch locationManager.authorizationStatus {
        case .notDetermined:
            locationManager.requestWhenInUseAuthorization()
        case .authorizedAlways, .authorizedWhenInUse:
            requestTemporaryFullAccuracyIfNeeded()
            startLocationUpdates()
        case .denied, .restricted:
            lastError = "Location permission is required for indoor and global registration."
        @unknown default:
            break
        }
    }

    private func startLocationUpdates() {
        locationManager.startUpdatingLocation()
        isHeadingAvailable = CLLocationManager.headingAvailable()
        if isHeadingAvailable {
            locationManager.startUpdatingHeading()
        } else {
            lastHeadingAccuracy = -1
        }
    }

    private func requestTemporaryFullAccuracyIfNeeded() {
        updateAuthorizationState()
        guard locationManager.authorizationStatus == .authorizedAlways
                || locationManager.authorizationStatus == .authorizedWhenInUse else { return }
        guard locationManager.accuracyAuthorization == .reducedAccuracy else { return }
        guard !hasRequestedTemporaryFullAccuracy else { return }

        hasRequestedTemporaryFullAccuracy = true
        locationManager.requestTemporaryFullAccuracyAuthorization(
            withPurposeKey: Self.preciseLocationPurposeKey
        ) { [weak self] error in
            guard let self else { return }
            self.updateAuthorizationState()
            if let error {
                self.lastError = "Precise location request failed: \(error.localizedDescription)"
            }
        }
    }

    private func updateAuthorizationState() {
        authorizationStatusLabel = label(for: locationManager.authorizationStatus)
        accuracyAuthorizationLabel = label(for: locationManager.accuracyAuthorization)
        isPreciseLocationAuthorized = locationManager.accuracyAuthorization == .fullAccuracy
    }

    private func publishIfNeeded() {
        guard isRunning,
              bridge.isConnected,
              let location = latestLocation else { return }

        let now = Date()
        var didPublish = false

        if topicRegistry.isStreamEnabled(.gps),
           shouldPublish(lastPublishedAt: lastGPSFixPublishedAt, interval: configuration.gpsFixPublishInterval) {
            bridge.publishNavSatFix(location, timestamp: ProcessInfo.processInfo.systemUptime)
            bridge.publishGPSMetadata(location)
            lastGPSFixPublishedAt = now
            lastHorizontalAccuracy = location.horizontalAccuracy
            lastVerticalAccuracy = location.verticalAccuracy
            didPublish = true
        }

        if topicRegistry.isStreamEnabled(.indoorLocalization),
           shouldPublish(lastPublishedAt: lastPublishedAt, interval: configuration.publishInterval) {
            let sample = makeSample(location: location, heading: latestHeading)
            bridge.publishIndoorLocalization(sample, timestamp: ProcessInfo.processInfo.systemUptime)
            lastPublishedAt = now
            lastIndoorRegistrationQuality = sample.indoorRegistrationQuality
            lastGlobalRegistrationQuality = sample.globalRegistrationQuality
            lastHorizontalAccuracy = location.horizontalAccuracy
            lastVerticalAccuracy = location.verticalAccuracy
            didPublish = true
        }

        if didPublish {
            lastError = nil
        }
    }

    private func shouldPublish(lastPublishedAt: Date?, interval: TimeInterval) -> Bool {
        guard let lastPublishedAt else { return true }
        return Date().timeIntervalSince(lastPublishedAt) >= interval
    }

    private func makeSample(location: CLLocation, heading: CLHeading?) -> IndoorLocalizationSample {
        let indoorScore = indoorScore(location: location, heading: heading)
        let globalScore = globalScore(location: location, heading: heading)

        return IndoorLocalizationSample(
            location: location,
            heading: heading,
            indoorRegistrationQuality: indoorScore,
            globalRegistrationQuality: globalScore,
            indoorQualityLabel: qualityLabel(indoorScore),
            globalQualityLabel: qualityLabel(globalScore),
            timestamp: Date()
        )
    }

    private func indoorScore(location: CLLocation, heading: CLHeading?) -> Double {
        let horizontalScore = accuracyScore(location.horizontalAccuracy, excellent: 3, unusable: 25)
        let floorScore = location.floor == nil ? 0.35 : 1.0
        let headingScore = heading.map { accuracyScore($0.headingAccuracy, excellent: 8, unusable: 60) } ?? 0.25
        let freshnessScore = freshnessScore(location.timestamp)
        let sourceScore = sourceScore(location.sourceInformation)

        return clamp(
            horizontalScore * 0.38
            + floorScore * 0.24
            + headingScore * 0.18
            + freshnessScore * 0.10
            + sourceScore * 0.10
        )
    }

    private func globalScore(location: CLLocation, heading: CLHeading?) -> Double {
        let horizontalScore = accuracyScore(location.horizontalAccuracy, excellent: 5, unusable: 60)
        let verticalScore = accuracyScore(location.verticalAccuracy, excellent: 8, unusable: 80)
        let headingScore = heading.map { accuracyScore($0.headingAccuracy, excellent: 10, unusable: 90) } ?? 0.3
        let coordinateScore = location.coordinate.latitude.isFinite && location.coordinate.longitude.isFinite ? 1.0 : 0.0
        let sourceScore = sourceScore(location.sourceInformation)

        return clamp(
            horizontalScore * 0.34
            + verticalScore * 0.20
            + headingScore * 0.16
            + coordinateScore * 0.20
            + sourceScore * 0.10
        )
    }

    private func accuracyScore(_ accuracy: Double, excellent: Double, unusable: Double) -> Double {
        guard accuracy.isFinite, accuracy >= 0 else { return 0 }
        guard accuracy > excellent else { return 1 }
        guard accuracy < unusable else { return 0 }
        return 1.0 - ((accuracy - excellent) / (unusable - excellent))
    }

    private func freshnessScore(_ timestamp: Date) -> Double {
        let age = abs(timestamp.timeIntervalSinceNow)
        if age <= 1 { return 1 }
        if age >= 10 { return 0 }
        return 1.0 - ((age - 1) / 9.0)
    }

    private func sourceScore(_ sourceInformation: CLLocationSourceInformation?) -> Double {
        guard let sourceInformation else { return 1 }
        if sourceInformation.isSimulatedBySoftware { return 0.2 }
        if sourceInformation.isProducedByAccessory { return 0.8 }
        return 1
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

    private func qualityLabel(_ score: Double) -> String {
        switch score {
        case 0.8...:
            return "high"
        case 0.5..<0.8:
            return "medium"
        case 0.25..<0.5:
            return "low"
        default:
            return "unusable"
        }
    }

    private func clamp(_ value: Double) -> Double {
        min(max(value, 0), 1)
    }
}
