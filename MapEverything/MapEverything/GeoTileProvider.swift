//
//  GeoTileProvider.swift
//  MapEverything
//

import Foundation
import CoreLocation

enum GeoTileLayerKind: String {
    case satelliteImagery = "satellite_imagery"
    case dem = "dem"
}

enum GeoTileCredentialRequirement: String, Codable, Hashable {
    case none
    case userAPIKey = "user_api_key"
    case userLogin = "user_login"
    case commercialAccount = "commercial_account"
}

struct GeoTileGeographicRegion: Hashable {
    let name: String
    let south: CLLocationDegrees
    let west: CLLocationDegrees
    let north: CLLocationDegrees
    let east: CLLocationDegrees

    func contains(_ coordinate: CLLocationCoordinate2D) -> Bool {
        guard coordinate.latitude >= south, coordinate.latitude <= north else { return false }

        let longitude = Self.normalizedLongitude(coordinate.longitude)
        let normalizedWest = Self.normalizedLongitude(west)
        let normalizedEast = Self.normalizedLongitude(east)

        if normalizedWest <= normalizedEast {
            return longitude >= normalizedWest && longitude <= normalizedEast
        } else {
            return longitude >= normalizedWest || longitude <= normalizedEast
        }
    }

    private static func normalizedLongitude(_ longitude: CLLocationDegrees) -> CLLocationDegrees {
        var normalized = longitude
        while normalized < -180 { normalized += 360 }
        while normalized > 180 { normalized -= 360 }
        return normalized
    }
}

enum GeoTileCoverage {
    case global
    case geographicRegions([GeoTileGeographicRegion])

    var isGlobal: Bool {
        if case .global = self {
            return true
        }
        return false
    }

    func contains(_ coordinate: CLLocationCoordinate2D) -> Bool {
        switch self {
        case .global:
            return true
        case .geographicRegions(let regions):
            return regions.contains { $0.contains(coordinate) }
        }
    }

    static let usgs3DEP = GeoTileCoverage.geographicRegions([
        GeoTileGeographicRegion(name: "Conterminous United States", south: 24, west: -125, north: 50, east: -66),
        GeoTileGeographicRegion(name: "Alaska", south: 51, west: 172, north: 72, east: -129),
        GeoTileGeographicRegion(name: "Hawaii", south: 18, west: -161, north: 23, east: -154),
        GeoTileGeographicRegion(name: "Puerto Rico and US Virgin Islands", south: 17, west: -68, north: 19, east: -64),
        GeoTileGeographicRegion(name: "Guam and Northern Mariana Islands", south: 13, west: 144, north: 21, east: 147),
        GeoTileGeographicRegion(name: "American Samoa", south: -15, west: -172, north: -11, east: -168)
    ])
}

struct GeoTileSourcePolicy: Codable, Hashable {
    let recordableByDefault: Bool
    let transientCacheOnly: Bool
    let attributionURL: String
    let credentialRequirement: GeoTileCredentialRequirement

    var requiresCredentials: Bool {
        credentialRequirement != .none
    }

    var rosMessage: [String: Any] {
        [
            "recordable_by_default": recordableByDefault,
            "transient_cache_only": transientCacheOnly,
            "attribution_url": attributionURL,
            "credential_requirement": credentialRequirement.rawValue,
            "requires_credentials": requiresCredentials
        ]
    }
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

struct GeoTileProjectedBounds {
    let minX: Double
    let minY: Double
    let maxX: Double
    let maxY: Double

    var arcGISBBoxParameter: String {
        [minX, minY, maxX, maxY]
            .map(Self.formatArcGISNumber)
            .joined(separator: ",")
    }

    static func webMercatorBounds(for coordinate: GeoTileCoordinate) -> GeoTileProjectedBounds {
        let originShift = Double.pi * 6_378_137.0
        let tileCount = pow(2.0, Double(coordinate.z))
        let tileWidth = originShift * 2.0 / tileCount
        let minX = -originShift + Double(coordinate.x) * tileWidth
        let maxX = -originShift + Double(coordinate.x + 1) * tileWidth
        let maxY = originShift - Double(coordinate.y) * tileWidth
        let minY = originShift - Double(coordinate.y + 1) * tileWidth

        return GeoTileProjectedBounds(minX: minX, minY: minY, maxX: maxX, maxY: maxY)
    }

    nonisolated private static func formatArcGISNumber(_ value: Double) -> String {
        String(format: "%.6f", locale: Locale(identifier: "en_US_POSIX"), value)
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
    let sourcePolicy: GeoTileSourcePolicy
    let coverage: GeoTileCoverage
    let dateOffsetDays: Int?
    let makeURL: (GeoTileCoordinate, String?) -> URL?

    var selectionKey: String {
        "\(kind.rawValue)|\(name)|\(layer)"
    }

    func supports(location: CLLocation) -> Bool {
        coverage.contains(location.coordinate)
    }

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
        sourcePolicy: GeoTileSourcePolicy(
            recordableByDefault: true,
            transientCacheOnly: false,
            attributionURL: "https://www.earthdata.nasa.gov/engage/open-data-services-software-policies/data-information-guidance",
            credentialRequirement: .none
        ),
        coverage: .global,
        dateOffsetDays: -2
    ) { coordinate, time in
        guard let time else { return nil }
        return URL(string: "https://gibs.earthdata.nasa.gov/wmts/epsg3857/best/MODIS_Terra_CorrectedReflectance_TrueColor/default/\(time)/GoogleMapsCompatible_Level9/\(coordinate.z)/\(coordinate.y)/\(coordinate.x).jpg")
    }

    static let usgs3DEPDEM = GeoTileProvider(
        kind: .dem,
        name: "USGS 3DEP",
        layer: "3DEPElevation",
        zoom: 12,
        crs: "EPSG:3857",
        format: "tiff",
        mimeType: "image/tiff",
        fileExtension: "tif",
        encoding: "usgs_3dep_float32_tiff",
        tileSizePixels: 256,
        attribution: "USGS 3D Elevation Program (3DEP) through The National Map",
        license: "USGS public data; retain USGS 3DEP and The National Map attribution",
        sourcePolicy: GeoTileSourcePolicy(
            recordableByDefault: true,
            transientCacheOnly: false,
            attributionURL: "https://www.usgs.gov/information-policies-and-instructions/copyrights-and-credits",
            credentialRequirement: .none
        ),
        coverage: .usgs3DEP,
        dateOffsetDays: nil
    ) { coordinate, _ in
        let bounds = GeoTileProjectedBounds.webMercatorBounds(for: coordinate)
        var components = URLComponents(string: "https://elevation.nationalmap.gov/arcgis/rest/services/3DEPElevation/ImageServer/exportImage")
        components?.queryItems = [
            URLQueryItem(name: "bbox", value: bounds.arcGISBBoxParameter),
            URLQueryItem(name: "bboxSR", value: "3857"),
            URLQueryItem(name: "imageSR", value: "3857"),
            URLQueryItem(name: "size", value: "256,256"),
            URLQueryItem(name: "format", value: "tiff"),
            URLQueryItem(name: "pixelType", value: "F32"),
            URLQueryItem(name: "interpolation", value: "RSP_BilinearInterpolation"),
            URLQueryItem(name: "f", value: "image")
        ]
        return components?.url
    }

    static let mapzenTerrainTiles = GeoTileProvider(
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
        sourcePolicy: GeoTileSourcePolicy(
            recordableByDefault: true,
            transientCacheOnly: false,
            attributionURL: "https://github.com/tilezen/joerd/blob/master/docs/attribution.md",
            credentialRequirement: .none
        ),
        coverage: .global,
        dateOffsetDays: nil
    ) { coordinate, _ in
        URL(string: "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/\(coordinate.z)/\(coordinate.x)/\(coordinate.y).png")
    }

    static let defaultDEM = usgs3DEPDEM
}

enum GeoTileProviderSelection {
    static func demCandidates(for location: CLLocation, providers: [GeoTileProvider]) -> [GeoTileProvider] {
        var candidates: [GeoTileProvider] = []
        var seenProviderKeys: Set<String> = []

        func appendIfNeeded(_ provider: GeoTileProvider) {
            guard seenProviderKeys.insert(provider.selectionKey).inserted else { return }
            candidates.append(provider)
        }

        providers
            .filter { $0.kind == .dem && !$0.coverage.isGlobal && $0.supports(location: location) }
            .forEach(appendIfNeeded)

        providers
            .filter { $0.kind == .dem && $0.coverage.isGlobal }
            .forEach(appendIfNeeded)

        return candidates
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

    func relativePath(provider: GeoTileProvider, coordinate: GeoTileCoordinate, time: String?) -> String {
        [
            provider.name.sanitizedPathComponent,
            provider.layer.sanitizedPathComponent,
            time?.sanitizedPathComponent ?? "static",
            String(coordinate.z),
            String(coordinate.x),
            "\(coordinate.y).\(provider.fileExtension)"
        ].joined(separator: "/")
    }

    private func fileURL(provider: GeoTileProvider, coordinate: GeoTileCoordinate, time: String?) -> URL {
        relativePath(provider: provider, coordinate: coordinate, time: time)
            .split(separator: "/")
            .reduce(rootURL) { url, component in
                url.appendingPathComponent(String(component), isDirectory: false)
            }
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
