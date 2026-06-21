//
//  GeoTilePublisher.swift
//  MapEverything
//

import Foundation
import CoreLocation
import Combine

final class GeoTilePublisher: NSObject, ObservableObject, CLLocationManagerDelegate {
    struct Configuration {
        let publishInterval: TimeInterval
        let locationDistanceFilter: CLLocationDistance

        static let `default` = Configuration(
            publishInterval: 180,
            locationDistanceFilter: 25
        )
    }

    static let shared = GeoTilePublisher()

    @Published private(set) var isRunning = false
    @Published private(set) var lastPublishedAt: Date?
    @Published private(set) var lastError: String?

    private let configuration: Configuration
    private let locationManager: CLLocationManager
    private let cache: GeoTileCache
    private let bridge: ROS2BridgeClient
    private let topicRegistry: ROS2TopicRegistry
    private let providers: [GeoTileProvider]

    private var latestLocation: CLLocation?
    private var publishTask: Task<Void, Never>?
    private var isPublishing = false

    init(
        configuration: Configuration = .default,
        locationManager: CLLocationManager = CLLocationManager(),
        cache: GeoTileCache = GeoTileCache(),
        bridge: ROS2BridgeClient = .shared,
        topicRegistry: ROS2TopicRegistry = .shared,
        providers: [GeoTileProvider] = [.defaultSatellite, .defaultDEM]
    ) {
        self.configuration = configuration
        self.locationManager = locationManager
        self.cache = cache
        self.bridge = bridge
        self.topicRegistry = topicRegistry
        self.providers = providers
        super.init()

        locationManager.delegate = self
        locationManager.desiredAccuracy = kCLLocationAccuracyHundredMeters
        locationManager.distanceFilter = configuration.locationDistanceFilter
    }

    func start() {
        guard !isRunning else { return }

        isRunning = true
        lastError = nil
        prepareLocationUpdates()

        publishTask = Task(priority: .background) { [weak self] in
            await self?.runPublishLoop()
        }
    }

    func stop() {
        isRunning = false
        publishTask?.cancel()
        publishTask = nil
        locationManager.stopUpdatingLocation()
    }

    func locationManagerDidChangeAuthorization(_ manager: CLLocationManager) {
        switch manager.authorizationStatus {
        case .authorizedAlways, .authorizedWhenInUse:
            manager.startUpdatingLocation()
        case .denied, .restricted:
            lastError = "Location permission is required to fetch localized satellite and DEM tiles."
        case .notDetermined:
            break
        @unknown default:
            break
        }
    }

    func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
        guard let location = locations.last else { return }
        latestLocation = location

        if isRunning, lastPublishedAt == nil {
            Task(priority: .background) { [weak self] in
                await self?.publishTilesIfAvailable()
            }
        }
    }

    func locationManager(_ manager: CLLocationManager, didFailWithError error: Error) {
        lastError = "Location update failed: \(error.localizedDescription)"
    }

    private func prepareLocationUpdates() {
        switch locationManager.authorizationStatus {
        case .notDetermined:
            locationManager.requestWhenInUseAuthorization()
        case .authorizedAlways, .authorizedWhenInUse:
            locationManager.startUpdatingLocation()
        case .denied, .restricted:
            lastError = "Location permission is required to fetch localized satellite and DEM tiles."
        @unknown default:
            break
        }
    }

    private func runPublishLoop() async {
        while !Task.isCancelled {
            await publishTilesIfAvailable()

            do {
                try await Task.sleep(nanoseconds: UInt64(configuration.publishInterval * 1_000_000_000))
            } catch {
                return
            }
        }
    }

    private func publishTilesIfAvailable() async {
        guard !isPublishing else { return }
        guard bridge.isConnected else { return }
        guard let location = latestLocation else { return }

        let activeProviders = providers.filter { provider in
            switch provider.kind {
            case .satelliteImagery:
                return topicRegistry.isStreamEnabled(.satelliteImagery)
            case .dem:
                return topicRegistry.isStreamEnabled(.dem)
            }
        }
        guard !activeProviders.isEmpty else { return }

        isPublishing = true
        defer { isPublishing = false }

        let now = Date()
        for provider in activeProviders {
            do {
                let payload = try await loadTile(
                    provider: provider,
                    location: location,
                    date: now
                )
                publish(payload: payload, timestamp: ProcessInfo.processInfo.systemUptime)
                lastPublishedAt = Date()
                lastError = nil
            } catch {
                lastError = "GeoTile fetch failed for \(provider.name): \(error.localizedDescription)"
            }
        }
    }

    private func loadTile(provider: GeoTileProvider, location: CLLocation, date: Date) async throws -> GeoTilePayload {
        let coordinate = GeoTileCoordinate.webMercator(
            latitude: location.coordinate.latitude,
            longitude: location.coordinate.longitude,
            zoom: provider.zoom
        )
        let time = provider.tileTime(for: date)
        guard let sourceURL = provider.makeURL(coordinate, time) else {
            throw URLError(.badURL)
        }

        let bounds = GeoTileBounds.webMercatorBounds(for: coordinate)
        let deviceLocation = GeoTileDeviceLocation(
            latitude: location.coordinate.latitude,
            longitude: location.coordinate.longitude,
            altitude: location.altitude,
            horizontalAccuracy: location.horizontalAccuracy,
            verticalAccuracy: location.verticalAccuracy,
            timestamp: location.timestamp,
            pixel: GeoTilePixelCoordinate.webMercator(
                latitude: location.coordinate.latitude,
                longitude: location.coordinate.longitude,
                coordinate: coordinate,
                tileSizePixels: provider.tileSizePixels
            )
        )

        if let cachedData = cache.load(provider: provider, coordinate: coordinate, time: time) {
            return GeoTilePayload(
                provider: provider,
                coordinate: coordinate,
                bounds: bounds,
                deviceLocation: deviceLocation,
                time: time,
                data: cachedData,
                sourceURL: sourceURL,
                fetchedAt: date,
                isCached: true
            )
        }

        let (data, response) = try await URLSession.shared.data(from: sourceURL)
        guard let httpResponse = response as? HTTPURLResponse,
              200..<300 ~= httpResponse.statusCode else {
            throw URLError(.badServerResponse)
        }

        try cache.store(data, provider: provider, coordinate: coordinate, time: time)

        return GeoTilePayload(
            provider: provider,
            coordinate: coordinate,
            bounds: bounds,
            deviceLocation: deviceLocation,
            time: time,
            data: data,
            sourceURL: sourceURL,
            fetchedAt: date,
            isCached: false
        )
    }

    private func publish(payload: GeoTilePayload, timestamp: TimeInterval) {
        switch payload.provider.kind {
        case .satelliteImagery:
            bridge.publishSatelliteTile(payload, timestamp: timestamp)
            bridge.publishGeoTileInfo(payload, timestamp: timestamp)
        case .dem:
            bridge.publishDEMTile(payload, timestamp: timestamp)
        }
    }
}
