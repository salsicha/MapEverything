//
//  MapGeoreferencer.swift
//  Reconstructor
//

import Foundation
import CoreLocation
import simd

struct MapGeoreferenceSnapshot {
    let origin: MapGeoreferencer.Origin
    let enuMeters: SIMD3<Double>
    let mapPositionMeters: SIMD3<Double>
    let locationTimestamp: Date

    var rosMessage: [String: Any] {
        [
            "origin_established": true,
            "origin": origin.rosMessage,
            "enu_meters": [
                "east": enuMeters.x,
                "north": enuMeters.y,
                "up": enuMeters.z
            ],
            "map_position_meters": [
                "x": mapPositionMeters.x,
                "y": mapPositionMeters.y,
                "z": mapPositionMeters.z
            ],
            "map_frame": "map",
            "map_axis_convention": "x=east,y=up,z=-north",
            "timestamp": ISO8601DateFormatter().string(from: locationTimestamp)
        ]
    }
}

struct EarthToMapTransform {
    let translationMeters: SIMD3<Double>
    let rotation: simd_quatd
}

final class MapGeoreferencer {
    struct Origin {
        let latitude: Double
        let longitude: Double
        let altitude: Double
        let horizontalAccuracy: Double
        let verticalAccuracy: Double
        let timestamp: Date
        let ecefMeters: SIMD3<Double>
        let mapPositionMeters: SIMD3<Double>
        let mapPoseTimestamp: TimeInterval

        var rosMessage: [String: Any] {
            [
                "latitude": latitude,
                "longitude": longitude,
                "altitude": altitude,
                "horizontal_accuracy": horizontalAccuracy,
                "vertical_accuracy": verticalAccuracy,
                "timestamp": ISO8601DateFormatter().string(from: timestamp),
                "ecef_meters": [
                    "x": ecefMeters.x,
                    "y": ecefMeters.y,
                    "z": ecefMeters.z
                ],
                "map_position_meters": [
                    "x": mapPositionMeters.x,
                    "y": mapPositionMeters.y,
                    "z": mapPositionMeters.z
                ],
                "map_pose_timestamp": mapPoseTimestamp
            ]
        }
    }

    private struct MapPose {
        let position: SIMD3<Double>
        let timestamp: TimeInterval
    }

    static let shared = MapGeoreferencer()

    private let maximumOriginHorizontalAccuracy: Double
    private var origin: Origin?
    private var latestMapPose: MapPose?

    init(maximumOriginHorizontalAccuracy: Double = 25) {
        self.maximumOriginHorizontalAccuracy = maximumOriginHorizontalAccuracy
    }

    func reset() {
        origin = nil
        latestMapPose = nil
    }

    func updateMapPose(_ transform: simd_float4x4, timestamp: TimeInterval) {
        latestMapPose = MapPose(
            position: SIMD3<Double>(
                Double(transform.columns.3.x),
                Double(transform.columns.3.y),
                Double(transform.columns.3.z)
            ),
            timestamp: timestamp
        )
    }

    func snapshot(for location: CLLocation) -> MapGeoreferenceSnapshot? {
        guard isUsableCoordinate(location) else { return nil }

        if origin == nil {
            establishOrigin(from: location)
        }

        guard let origin else { return nil }

        let ecef = Self.ecefMeters(
            latitude: location.coordinate.latitude,
            longitude: location.coordinate.longitude,
            altitude: location.altitude
        )
        let enu = Self.enuMeters(
            ecef: ecef,
            originECEF: origin.ecefMeters,
            originLatitude: origin.latitude,
            originLongitude: origin.longitude
        )
        let mapPosition = origin.mapPositionMeters + SIMD3<Double>(
            enu.x,
            enu.z,
            -enu.y
        )

        return MapGeoreferenceSnapshot(
            origin: origin,
            enuMeters: enu,
            mapPositionMeters: mapPosition,
            locationTimestamp: location.timestamp
        )
    }

    var unavailableMessage: [String: Any] {
        [
            "origin_established": false,
            "map_frame": "map",
            "map_axis_convention": "x=east,y=up,z=-north",
            "reason": originUnavailableReason()
        ]
    }

    func earthToMapTransform() -> EarthToMapTransform? {
        guard let origin else { return nil }

        let rotation = Self.ecefFromMapRotation(
            latitude: origin.latitude,
            longitude: origin.longitude
        )
        let mapOriginECEF = origin.ecefMeters - simd_mul(rotation, origin.mapPositionMeters)

        return EarthToMapTransform(
            translationMeters: mapOriginECEF,
            rotation: simd_quatd(rotation)
        )
    }

    private func establishOrigin(from location: CLLocation) {
        guard let latestMapPose else { return }
        guard location.horizontalAccuracy.isFinite,
              location.horizontalAccuracy >= 0,
              location.horizontalAccuracy <= maximumOriginHorizontalAccuracy else { return }

        origin = Origin(
            latitude: location.coordinate.latitude,
            longitude: location.coordinate.longitude,
            altitude: location.altitude,
            horizontalAccuracy: location.horizontalAccuracy,
            verticalAccuracy: location.verticalAccuracy,
            timestamp: location.timestamp,
            ecefMeters: Self.ecefMeters(
                latitude: location.coordinate.latitude,
                longitude: location.coordinate.longitude,
                altitude: location.altitude
            ),
            mapPositionMeters: latestMapPose.position,
            mapPoseTimestamp: latestMapPose.timestamp
        )
    }

    private func originUnavailableReason() -> String {
        if latestMapPose == nil {
            return "waiting_for_arkit_map_pose"
        }
        return "waiting_for_accurate_gps_origin"
    }

    private func isUsableCoordinate(_ location: CLLocation) -> Bool {
        location.coordinate.latitude.isFinite
        && location.coordinate.longitude.isFinite
        && location.altitude.isFinite
    }

    private static func ecefMeters(latitude: Double, longitude: Double, altitude: Double) -> SIMD3<Double> {
        let semiMajorAxis = 6_378_137.0
        let firstEccentricitySquared = 6.694_379_990_14e-3
        let latitudeRadians = latitude * .pi / 180.0
        let longitudeRadians = longitude * .pi / 180.0
        let sinLatitude = sin(latitudeRadians)
        let cosLatitude = cos(latitudeRadians)
        let primeVerticalRadius = semiMajorAxis / sqrt(1 - firstEccentricitySquared * sinLatitude * sinLatitude)

        return SIMD3<Double>(
            (primeVerticalRadius + altitude) * cosLatitude * cos(longitudeRadians),
            (primeVerticalRadius + altitude) * cosLatitude * sin(longitudeRadians),
            (primeVerticalRadius * (1 - firstEccentricitySquared) + altitude) * sinLatitude
        )
    }

    private static func enuMeters(
        ecef: SIMD3<Double>,
        originECEF: SIMD3<Double>,
        originLatitude: Double,
        originLongitude: Double
    ) -> SIMD3<Double> {
        let delta = ecef - originECEF
        let latitudeRadians = originLatitude * .pi / 180.0
        let longitudeRadians = originLongitude * .pi / 180.0
        let sinLatitude = sin(latitudeRadians)
        let cosLatitude = cos(latitudeRadians)
        let sinLongitude = sin(longitudeRadians)
        let cosLongitude = cos(longitudeRadians)

        let east = -sinLongitude * delta.x + cosLongitude * delta.y
        let north = -sinLatitude * cosLongitude * delta.x
            - sinLatitude * sinLongitude * delta.y
            + cosLatitude * delta.z
        let up = cosLatitude * cosLongitude * delta.x
            + cosLatitude * sinLongitude * delta.y
            + sinLatitude * delta.z

        return SIMD3<Double>(east, north, up)
    }

    private static func ecefFromMapRotation(latitude: Double, longitude: Double) -> simd_double3x3 {
        let latitudeRadians = latitude * .pi / 180.0
        let longitudeRadians = longitude * .pi / 180.0
        let sinLatitude = sin(latitudeRadians)
        let cosLatitude = cos(latitudeRadians)
        let sinLongitude = sin(longitudeRadians)
        let cosLongitude = cos(longitudeRadians)

        let east = SIMD3<Double>(-sinLongitude, cosLongitude, 0)
        let north = SIMD3<Double>(
            -sinLatitude * cosLongitude,
            -sinLatitude * sinLongitude,
            cosLatitude
        )
        let up = SIMD3<Double>(
            cosLatitude * cosLongitude,
            cosLatitude * sinLongitude,
            sinLatitude
        )

        return simd_double3x3(columns: (east, up, -north))
    }
}
