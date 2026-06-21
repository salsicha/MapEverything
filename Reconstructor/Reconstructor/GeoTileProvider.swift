//
//  GeoTileProvider.swift
//  Reconstructor
//

import Foundation
import CoreLocation

enum GeoTileLayerKind: String {
    case satelliteImagery = "satellite_imagery"
    case dem = "dem"
}

struct GeoTileCoordinate: Hashable {
    let z: Int
    let x: Int
    let y: Int

    static func webMercator(latitude: CLLocationDegrees, longitude: CLLocationDegrees, zoom: Int) -> GeoTileCoordinate {
        let clampedLatitude = min(max(latitude, -85.05112878), 85.05112878)
        let latRadians = clampedLatitude * .pi / 180.0
        let tileCount = pow(2.0, Double(zoom))
        let rawX = (longitude + 180.0) / 360.0 * tileCount
        let rawY = (1.0 - log(tan(latRadians) + 1.0 / cos(latRadians)) / .pi) / 2.0 * tileCount

        return GeoTileCoordinate(
            z: zoom,
            x: min(max(Int(floor(rawX)), 0), Int(tileCount) - 1),
            y: min(max(Int(floor(rawY)), 0), Int(tileCount) - 1)
        )
    }
}

struct GeoTilePixelCoordinate {
    let x: Double
    let y: Double
    let width: Int
    let height: Int

    var rosMessage: [String: Any] {
        [
            "x": x,
            "y": y,
            "width": width,
            "height": height,
            "origin": "upper_left",
            "units": "pixels"
        ]
    }

    static func webMercator(
        latitude: CLLocationDegrees,
        longitude: CLLocationDegrees,
        coordinate: GeoTileCoordinate,
        tileSizePixels: Int
    ) -> GeoTilePixelCoordinate {
        let clampedLatitude = min(max(latitude, -85.05112878), 85.05112878)
        let latRadians = clampedLatitude * .pi / 180.0
        let tileCount = pow(2.0, Double(coordinate.z))
        let rawTileX = (longitude + 180.0) / 360.0 * tileCount
        let rawTileY = (1.0 - log(tan(latRadians) + 1.0 / cos(latRadians)) / .pi) / 2.0 * tileCount
        let pixelX = (rawTileX - Double(coordinate.x)) * Double(tileSizePixels)
        let pixelY = (rawTileY - Double(coordinate.y)) * Double(tileSizePixels)

        return GeoTilePixelCoordinate(
            x: min(max(pixelX, 0), Double(tileSizePixels)),
            y: min(max(pixelY, 0), Double(tileSizePixels)),
            width: tileSizePixels,
            height: tileSizePixels
        )
    }
}

struct GeoTileDeviceLocation {
    let latitude: Double
    let longitude: Double
    let altitude: Double
    let horizontalAccuracy: Double
    let verticalAccuracy: Double
    let timestamp: Date
    let pixel: GeoTilePixelCoordinate

    var rosMessage: [String: Any] {
        [
            "latitude": latitude,
            "longitude": longitude,
            "altitude": altitude,
            "horizontal_accuracy": horizontalAccuracy,
            "vertical_accuracy": verticalAccuracy,
            "timestamp": ISO8601DateFormatter().string(from: timestamp),
            "pixel": pixel.rosMessage
        ]
    }
}

struct GeoTileBounds {
    let west: Double
    let south: Double
    let east: Double
    let north: Double

    var rosMessage: [String: Any] {
        [
            "west": west,
            "south": south,
            "east": east,
            "north": north
        ]
    }

    static func webMercatorBounds(for coordinate: GeoTileCoordinate) -> GeoTileBounds {
        let tileCount = pow(2.0, Double(coordinate.z))
        let west = Double(coordinate.x) / tileCount * 360.0 - 180.0
        let east = Double(coordinate.x + 1) / tileCount * 360.0 - 180.0
        let north = mercatorLatitude(tileY: Double(coordinate.y), tileCount: tileCount)
        let south = mercatorLatitude(tileY: Double(coordinate.y + 1), tileCount: tileCount)

        return GeoTileBounds(west: west, south: south, east: east, north: north)
    }

    private static func mercatorLatitude(tileY: Double, tileCount: Double) -> Double {
        let radians = atan(sinh(.pi * (1.0 - 2.0 * tileY / tileCount)))
        return radians * 180.0 / .pi
    }
}

struct GeoTilePayload {
    let provider: GeoTileProvider
    let coordinate: GeoTileCoordinate
    let bounds: GeoTileBounds
    let deviceLocation: GeoTileDeviceLocation
    let time: String?
    let data: Data
    let sourceURL: URL
    let fetchedAt: Date
    let isCached: Bool
}

struct GeoTileProvider {
    let kind: GeoTileLayerKind
    let name: String
    let layer: String
    let zoom: Int
    let crs: String
    let format: String
    let mimeType: String
    let fileExtension: String
    let encoding: String
    let tileSizePixels: Int
    let attribution: String
    let license: String
    let dateOffsetDays: Int?
    let makeURL: (GeoTileCoordinate, String?) -> URL?

    func tileTime(for date: Date) -> String? {
        guard let dateOffsetDays else { return nil }
        let calendar = Calendar(identifier: .gregorian)
        let tileDate = calendar.date(byAdding: .day, value: dateOffsetDays, to: date) ?? date
        let formatter = DateFormatter()
        formatter.calendar = calendar
        formatter.locale = Locale(identifier: "en_US_POSIX")
        formatter.timeZone = TimeZone(secondsFromGMT: 0)
        formatter.dateFormat = "yyyy-MM-dd"
        return formatter.string(from: tileDate)
    }

    static let defaultSatellite = GeoTileProvider(
        kind: .satelliteImagery,
        name: "NASA GIBS",
        layer: "MODIS_Terra_CorrectedReflectance_TrueColor",
        zoom: 9,
        crs: "EPSG:3857",
        format: "jpeg",
        mimeType: "image/jpeg",
        fileExtension: "jpg",
        encoding: "wmts_jpeg",
        tileSizePixels: 256,
        attribution: "NASA Global Imagery Browse Services (GIBS)",
        license: "NASA Earthdata open-data guidance; retain GIBS attribution and layer/date metadata",
        dateOffsetDays: -2
    ) { coordinate, time in
        guard let time else { return nil }
        return URL(string: "https://gibs.earthdata.nasa.gov/wmts/epsg3857/best/MODIS_Terra_CorrectedReflectance_TrueColor/default/\(time)/GoogleMapsCompatible_Level9/\(coordinate.z)/\(coordinate.y)/\(coordinate.x).jpg")
    }

    static let defaultDEM = GeoTileProvider(
        kind: .dem,
        name: "Mapzen Terrain Tiles",
        layer: "terrarium",
        zoom: 12,
        crs: "EPSG:3857",
        format: "png",
        mimeType: "image/png",
        fileExtension: "png",
        encoding: "terrarium_png",
        tileSizePixels: 256,
        attribution: "Mapzen Terrain Tiles; source data includes USGS, SRTM, and other regional DEM sources",
        license: "Mapzen Terrain Tiles attribution requirements; underlying source attribution varies by region",
        dateOffsetDays: nil
    ) { coordinate, _ in
        URL(string: "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/\(coordinate.z)/\(coordinate.x)/\(coordinate.y).png")
    }
}

final class GeoTileCache {
    private let rootURL: URL

    init(fileManager: FileManager = .default) {
        let baseURL = fileManager.urls(for: .cachesDirectory, in: .userDomainMask).first
            ?? fileManager.temporaryDirectory
        rootURL = baseURL.appendingPathComponent("GeoTiles", isDirectory: true)
    }

    func load(provider: GeoTileProvider, coordinate: GeoTileCoordinate, time: String?) -> Data? {
        try? Data(contentsOf: fileURL(provider: provider, coordinate: coordinate, time: time))
    }

    func store(_ data: Data, provider: GeoTileProvider, coordinate: GeoTileCoordinate, time: String?) throws {
        let url = fileURL(provider: provider, coordinate: coordinate, time: time)
        try FileManager.default.createDirectory(at: url.deletingLastPathComponent(), withIntermediateDirectories: true)
        try data.write(to: url, options: .atomic)
    }

    private func fileURL(provider: GeoTileProvider, coordinate: GeoTileCoordinate, time: String?) -> URL {
        let safeProvider = provider.name.sanitizedPathComponent
        let safeLayer = provider.layer.sanitizedPathComponent
        let timeComponent = time?.sanitizedPathComponent ?? "static"

        return rootURL
            .appendingPathComponent(safeProvider, isDirectory: true)
            .appendingPathComponent(safeLayer, isDirectory: true)
            .appendingPathComponent(timeComponent, isDirectory: true)
            .appendingPathComponent(String(coordinate.z), isDirectory: true)
            .appendingPathComponent(String(coordinate.x), isDirectory: true)
            .appendingPathComponent("\(coordinate.y).\(provider.fileExtension)")
    }
}

private extension String {
    var sanitizedPathComponent: String {
        let allowed = CharacterSet.alphanumerics.union(CharacterSet(charactersIn: "-_."))
        return unicodeScalars.map { scalar in
            allowed.contains(scalar) ? Character(scalar) : "_"
        }.reduce(into: "") { result, character in
            result.append(character)
        }
    }
}
