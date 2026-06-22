//
//  MappingFileCleanupManager.swift
//  MapEverything
//

import Foundation

struct MappingFileCleanupResult: Equatable {
    let removedFiles: [String]
    let skippedFiles: [String]

    var removedCount: Int {
        removedFiles.count
    }
}

enum MappingFileCleanupManager {
    static let managedExtensions: Set<String> = [
        "ply",
        "arexperience",
        "worldmap",
        "usdz",
        "obj",
        "pdf",
        "mp4",
        "jpg",
        "jpeg",
        "png",
        "tif",
        "tiff",
        "json",
        "bag",
        "mcap",
        "db3"
    ]

    static let sessionDirectoryNames: Set<String> = [
        "Sessions",
        "MappingSessions",
        "ROS2Sessions"
    ]

    static func referencedDocumentPaths(
        environments: [EnvironmentModel],
        sessions: [MappingSessionModel] = [],
        geoTiles: [GeoTileModel] = []
    ) -> Set<String> {
        var paths = Set<String>()

        for environment in environments {
            [
                environment.filePathToPointCloudData,
                environment.arWorldMapPath,
                environment.meshPath,
                environment.objPath,
                environment.blueprintPath,
                environment.videoPath,
                environment.thumbnailPath
            ]
            .compactMap { $0 }
            .forEach { paths.insert($0) }
        }

        sessions.compactMap(\.sessionDirectoryPath).forEach { paths.insert($0) }
        geoTiles.map(\.cachePath).forEach { paths.insert($0) }

        return paths
    }

    @discardableResult
    static func removeOrphanedDocumentFiles(
        in documentDirectory: URL,
        referencedPaths: Set<String>,
        fileManager: FileManager = .default
    ) -> MappingFileCleanupResult {
        guard let enumerator = fileManager.enumerator(
            at: documentDirectory,
            includingPropertiesForKeys: [.isRegularFileKey],
            options: [.skipsHiddenFiles]
        ) else {
            return MappingFileCleanupResult(removedFiles: [], skippedFiles: [])
        }

        var removed: [String] = []
        var skipped: [String] = []

        for case let fileURL as URL in enumerator {
            guard isManagedDocumentFile(fileURL) else { continue }

            let relativePath = relativePath(for: fileURL, baseURL: documentDirectory)
            guard !referencedPaths.contains(relativePath) else {
                skipped.append(relativePath)
                continue
            }

            do {
                try fileManager.removeItem(at: fileURL)
                removed.append(relativePath)
            } catch {
                skipped.append(relativePath)
            }
        }

        return MappingFileCleanupResult(
            removedFiles: removed.sorted(),
            skippedFiles: skipped.sorted()
        )
    }

    @discardableResult
    static func removeOrphanedCacheFiles(
        in cacheDirectory: URL,
        referencedPaths: Set<String>,
        fileManager: FileManager = .default
    ) -> MappingFileCleanupResult {
        guard let enumerator = fileManager.enumerator(
            at: cacheDirectory,
            includingPropertiesForKeys: [.isRegularFileKey],
            options: [.skipsHiddenFiles]
        ) else {
            return MappingFileCleanupResult(removedFiles: [], skippedFiles: [])
        }

        var removed: [String] = []
        var skipped: [String] = []

        for case let fileURL as URL in enumerator {
            guard isManagedDocumentFile(fileURL) || fileURL.pathComponents.contains("GeoTiles") else { continue }

            let relativePath = relativePath(for: fileURL, baseURL: cacheDirectory)
            let geoTileRelativePath = relativePath.removingGeoTilesPrefix
            guard !referencedPaths.contains(relativePath),
                  !referencedPaths.contains(geoTileRelativePath) else {
                skipped.append(relativePath)
                continue
            }

            do {
                try fileManager.removeItem(at: fileURL)
                removed.append(relativePath)
            } catch {
                skipped.append(relativePath)
            }
        }

        return MappingFileCleanupResult(
            removedFiles: removed.sorted(),
            skippedFiles: skipped.sorted()
        )
    }

    static func isManagedDocumentFile(_ url: URL) -> Bool {
        managedExtensions.contains(url.pathExtension.lowercased())
    }

    private static func relativePath(for fileURL: URL, baseURL: URL) -> String {
        let basePath = baseURL.standardizedFileURL.path
        let filePath = fileURL.standardizedFileURL.path
        guard filePath.hasPrefix(basePath) else {
            return fileURL.lastPathComponent
        }

        var relative = String(filePath.dropFirst(basePath.count))
        if relative.hasPrefix("/") {
            relative.removeFirst()
        }
        return relative
    }
}

private extension String {
    var removingGeoTilesPrefix: String {
        guard hasPrefix("GeoTiles/") else { return self }
        return String(dropFirst("GeoTiles/".count))
    }
}
