//
//  LocalROS2BagRecorder.swift
//  MapEverything
//

import Foundation
import Combine
import SQLite3
#if canImport(UIKit)
import UIKit
#endif

struct LocalROS2BagRecorderConfiguration: Equatable {
    static let enabledStorageKey = "localROS2BagStorageEnabled"
    static let chunkSizeMBStorageKey = "localROS2BagChunkSizeMB"
    static let defaultChunkSizeMB = 64
    static let minimumChunkSizeMB = 8
    static let maximumChunkSizeMB = 512

    let isEnabled: Bool
    let maxChunkBytes: Int

    var chunkSizeMB: Int {
        max(1, maxChunkBytes / 1_048_576)
    }

    static func load(from userDefaults: UserDefaults = .standard) -> LocalROS2BagRecorderConfiguration {
        let storedChunkSize = userDefaults.integer(forKey: chunkSizeMBStorageKey)
        let chunkSizeMB = storedChunkSize > 0 ? storedChunkSize : defaultChunkSizeMB
        let clampedChunkSizeMB = min(max(chunkSizeMB, minimumChunkSizeMB), maximumChunkSizeMB)

        return LocalROS2BagRecorderConfiguration(
            isEnabled: userDefaults.bool(forKey: enabledStorageKey),
            maxChunkBytes: clampedChunkSizeMB * 1_048_576
        )
    }
}

struct LocalROS2BagRecorderStats: Equatable {
    let isEnabled: Bool
    let isRecording: Bool
    let bagDirectoryURL: URL?
    let currentChunkURL: URL?
    let chunkCount: Int
    let messageCount: Int
    let currentChunkBytes: Int
    let maxChunkBytes: Int
    let startedAt: Date?
    let stoppedAt: Date?
    let lastError: String?
    let lastErrorAt: Date?

    init(
        isEnabled: Bool = false,
        isRecording: Bool = false,
        bagDirectoryURL: URL? = nil,
        currentChunkURL: URL? = nil,
        chunkCount: Int = 0,
        messageCount: Int = 0,
        currentChunkBytes: Int = 0,
        maxChunkBytes: Int = 0,
        startedAt: Date? = nil,
        stoppedAt: Date? = nil,
        lastError: String? = nil,
        lastErrorAt: Date? = nil
    ) {
        self.isEnabled = isEnabled
        self.isRecording = isRecording
        self.bagDirectoryURL = bagDirectoryURL
        self.currentChunkURL = currentChunkURL
        self.chunkCount = chunkCount
        self.messageCount = messageCount
        self.currentChunkBytes = currentChunkBytes
        self.maxChunkBytes = maxChunkBytes
        self.startedAt = startedAt
        self.stoppedAt = stoppedAt
        self.lastError = lastError
        self.lastErrorAt = lastErrorAt
    }

    var rosMessage: [String: Any] {
        [
            "enabled": isEnabled,
            "recording": isRecording,
            "serialization_format": LocalROS2BagRecorder.serializationFormat,
            "storage_identifier": LocalROS2BagRecorder.storageIdentifier,
            "bag_directory": bagDirectoryURL?.path ?? "",
            "current_chunk": currentChunkURL?.lastPathComponent ?? "",
            "chunk_count": chunkCount,
            "message_count": messageCount,
            "current_chunk_bytes": currentChunkBytes,
            "max_chunk_bytes": maxChunkBytes,
            "started_at": startedAt.map { ISO8601DateFormatter().string(from: $0) } ?? "",
            "stopped_at": stoppedAt.map { ISO8601DateFormatter().string(from: $0) } ?? "",
            "last_error": lastError ?? "",
            "last_error_at": lastErrorAt.map { ISO8601DateFormatter().string(from: $0) } ?? ""
        ]
    }

    var diagnosticValues: [String: String] {
        [
            "enabled": String(isEnabled),
            "recording": String(isRecording),
            "serialization_format": LocalROS2BagRecorder.serializationFormat,
            "storage_identifier": LocalROS2BagRecorder.storageIdentifier,
            "bag_directory": bagDirectoryURL?.path ?? "",
            "current_chunk": currentChunkURL?.lastPathComponent ?? "",
            "chunk_count": String(chunkCount),
            "message_count": String(messageCount),
            "current_chunk_bytes": String(currentChunkBytes),
            "max_chunk_bytes": String(maxChunkBytes),
            "started_at": startedAt.map { ISO8601DateFormatter().string(from: $0) } ?? "",
            "stopped_at": stoppedAt.map { ISO8601DateFormatter().string(from: $0) } ?? "",
            "last_error": lastError ?? ""
        ]
    }
}

struct LocalROS2BagFile: Identifiable, Hashable {
    enum Kind: String {
        case metadata
        case sqliteChunk
        case other

        var displayName: String {
            switch self {
            case .metadata: return "Metadata"
            case .sqliteChunk: return "SQLite Chunk"
            case .other: return "File"
            }
        }

        var iconName: String {
            switch self {
            case .metadata: return "doc.text"
            case .sqliteChunk: return "cylinder.split.1x2"
            case .other: return "doc"
            }
        }
    }

    let url: URL
    let relativePath: String
    let kind: Kind
    let byteCount: Int
    let modifiedAt: Date?

    var id: String {
        url.path
    }

    var name: String {
        url.lastPathComponent
    }

    var byteCountLabel: String {
        ByteCountFormatter.string(fromByteCount: Int64(byteCount), countStyle: .file)
    }
}

struct LocalROS2BagSession: Identifiable, Hashable {
    let directoryURL: URL
    let files: [LocalROS2BagFile]
    let byteCount: Int
    let modifiedAt: Date?
    let preview: LocalROS2BagSessionPreview?

    var id: String {
        directoryURL.path
    }

    var name: String {
        directoryURL.lastPathComponent
    }

    var chunkCount: Int {
        files.filter { $0.kind == .sqliteChunk }.count
    }

    var byteCountLabel: String {
        ByteCountFormatter.string(fromByteCount: Int64(byteCount), countStyle: .file)
    }
}

struct LocalROS2BagSessionPreview: Codable, Hashable {
    static let currentSchemaVersion = 1
    static let cacheFileName = ".mapeverything-preview.json"
    static let thumbnailFileName = ".mapeverything-thumbnail.jpg"

    let schemaVersion: Int
    let cacheKey: String
    let generatedAt: Date
    let messageCount: Int
    let topicNames: [String]
    let startedAtNanoseconds: Int64?
    let endedAtNanoseconds: Int64?
    let thumbnailRelativePath: String?

    init(
        cacheKey: String,
        generatedAt: Date = Date(),
        messageCount: Int,
        topicNames: [String],
        startedAtNanoseconds: Int64?,
        endedAtNanoseconds: Int64?,
        thumbnailRelativePath: String?
    ) {
        self.schemaVersion = Self.currentSchemaVersion
        self.cacheKey = cacheKey
        self.generatedAt = generatedAt
        self.messageCount = messageCount
        self.topicNames = topicNames.sorted()
        self.startedAtNanoseconds = startedAtNanoseconds
        self.endedAtNanoseconds = endedAtNanoseconds
        self.thumbnailRelativePath = thumbnailRelativePath
    }

    var durationNanoseconds: Int64 {
        guard let startedAtNanoseconds, let endedAtNanoseconds else { return 0 }
        return max(0, endedAtNanoseconds - startedAtNanoseconds)
    }

    var durationLabel: String {
        let seconds = Double(durationNanoseconds) / 1_000_000_000.0
        guard seconds > 0 else { return "0s" }
        let formatter = DateComponentsFormatter()
        formatter.allowedUnits = seconds >= 3600 ? [.hour, .minute] : [.minute, .second]
        formatter.unitsStyle = .abbreviated
        formatter.maximumUnitCount = 2
        return formatter.string(from: seconds) ?? "\(Int(seconds))s"
    }

    var messageCountLabel: String {
        "\(messageCount.formatted()) msgs"
    }

    var topicSummary: String {
        guard !topicNames.isEmpty else { return "No topics" }
        if topicNames.count <= 2 {
            return topicNames.joined(separator: ", ")
        }
        return "\(topicNames[0]), \(topicNames[1]) +\(topicNames.count - 2)"
    }

    func thumbnailURL(relativeTo directoryURL: URL) -> URL? {
        guard let thumbnailRelativePath else { return nil }
        return directoryURL.appendingPathComponent(thumbnailRelativePath)
    }
}

enum LocalROS2BagPreviewLoadingMode {
    case cachedOnly
    case scanIfNeeded
}

final class LocalROS2BagRecorder: ObservableObject {
    static let shared = LocalROS2BagRecorder()
    static let serializationFormat = "rosbridge_json"
    static let storageIdentifier = "sqlite3"

    @Published private(set) var stats = LocalROS2BagRecorderStats()

    private struct TopicInfo {
        let id: Int64
        let name: String
        let type: String
        var messageCount: Int
    }

    private struct BagFileInfo {
        let relativePath: String
        let startingTimeNanoseconds: Int64
        var endingTimeNanoseconds: Int64
        var messageCount: Int
        var byteCount: Int

        var durationNanoseconds: Int64 {
            max(0, endingTimeNanoseconds - startingTimeNanoseconds)
        }
    }

    private struct PendingWrite {
        let topic: String
        let messageType: String
        let timestampNanoseconds: Int64
        let data: Data
    }

    private static let writeBatchMaxMessages = 32
    private static let writeBatchMaxBytes = 1_048_576
    private static let writeBatchFlushDelay: TimeInterval = 0.25
    private static let insertMessageSQL = "INSERT INTO messages(id, topic_id, timestamp, data) VALUES (?, ?, ?, ?)"

    private let queue = DispatchQueue(label: "com.mapeverything.localROS2BagRecorder", qos: .utility)
    private let previewQueue = DispatchQueue(label: "com.mapeverything.localROS2BagPreviewScanner", qos: .utility)
    private let fileManager: FileManager
    private let baseDirectoryURL: URL?
    private var database: OpaquePointer?
    private var bagDirectoryURL: URL?
    private var currentChunkURL: URL?
    private var currentChunkIndex = 0
    private var currentChunkBytes = 0
    private var currentChunkMessageCount = 0
    private var currentChunkStartNanoseconds: Int64?
    private var currentChunkEndNanoseconds: Int64?
    private var nextTopicID: Int64 = 1
    private var nextMessageID: Int64 = 1
    private var topicsByName: [String: TopicInfo] = [:]
    private var topicsInsertedInCurrentChunk: Set<String> = []
    private var bagFiles: [BagFileInfo] = []
    private var configuration = LocalROS2BagRecorderConfiguration(isEnabled: false, maxChunkBytes: 0)
    private var startedAt: Date?
    private var stoppedAt: Date?
    private var lastError: String?
    private var lastErrorAt: Date?
    private var acceptsRecords = false
    private var pendingWrites: [PendingWrite] = []
    private var pendingWriteBytes = 0
    private var pendingFlushWorkItem: DispatchWorkItem?

    init(fileManager: FileManager = .default, baseDirectoryURL: URL? = nil) {
        self.fileManager = fileManager
        self.baseDirectoryURL = baseDirectoryURL
    }

    var sessionMetadata: [String: Any] {
        stats.rosMessage
    }

    var diagnosticLevel: Int {
        if stats.lastError != nil { return 1 }
        return 0
    }

    var diagnosticMessage: String {
        if let lastError = stats.lastError {
            return lastError
        }
        if stats.isRecording {
            return "Local SQLite rosbag recording active"
        }
        if stats.isEnabled {
            return "Local SQLite rosbag recording enabled"
        }
        return "Local SQLite rosbag recording disabled"
    }

    var isAcceptingRecords: Bool {
        queue.sync {
            acceptsRecords && configuration.isEnabled && database != nil
        }
    }

    func start(sessionID: UUID?, configuration: LocalROS2BagRecorderConfiguration = .load()) {
        queue.sync {
            self.acceptsRecords = false
            self.closeCurrentBag(writeMetadata: true)
            self.configuration = configuration
            self.startedAt = nil
            self.stoppedAt = nil
            self.lastError = nil
            self.lastErrorAt = nil

            guard configuration.isEnabled else {
                self.publishStats(isRecording: false)
                return
            }

            do {
                self.resetBagState()
                let rootURL = try self.storageRootURL()
                try self.fileManager.createDirectory(at: rootURL, withIntermediateDirectories: true)

                let bagName = self.bagName(sessionID: sessionID, date: Date())
                let directoryURL = rootURL.appendingPathComponent(bagName, isDirectory: true)
                try self.fileManager.createDirectory(at: directoryURL, withIntermediateDirectories: true)
                self.bagDirectoryURL = directoryURL
                self.startedAt = Date()
                try self.openChunk(index: 0)
                self.acceptsRecords = true
                self.writeMetadata()
                self.publishStats(isRecording: true)
            } catch {
                self.recordFailure("Failed to start local rosbag: \(error.localizedDescription)")
            }
        }
    }

    func stop() {
        queue.async {
            self.acceptsRecords = false
            self.stoppedAt = Date()
            self.closeCurrentBag(writeMetadata: true)
            self.publishStats(isRecording: false)
        }
    }

    func stopAndWait() {
        queue.sync {
            self.acceptsRecords = false
            self.stoppedAt = Date()
            self.closeCurrentBag(writeMetadata: true)
            self.publishStats(isRecording: false)
        }
    }

    func flushAndWait() {
        queue.sync {
            do {
                try self.flushPendingWrites(publishStatsAfterFlush: true)
            } catch {
                self.recordFailure("Failed to flush local rosbag: \(error.localizedDescription)")
            }
        }
    }

    func listBagSessions() throws -> [LocalROS2BagSession] {
        try listBagSessions(previewLoadingMode: .cachedOnly)
    }

    func listBagSessions(previewLoadingMode: LocalROS2BagPreviewLoadingMode) throws -> [LocalROS2BagSession] {
        let rootURL = try storageRootURL()
        guard fileManager.fileExists(atPath: rootURL.path) else { return [] }

        let directoryURLs = try fileManager.contentsOfDirectory(
            at: rootURL,
            includingPropertiesForKeys: [.isDirectoryKey, .contentModificationDateKey],
            options: [.skipsHiddenFiles]
        )

        return try directoryURLs.compactMap { url in
            let values = try url.resourceValues(forKeys: [.isDirectoryKey])
            guard values.isDirectory == true else { return nil }
            return try bagSession(directoryURL: url, previewLoadingMode: previewLoadingMode)
        }
        .sorted { lhs, rhs in
            (lhs.modifiedAt ?? .distantPast) > (rhs.modifiedAt ?? .distantPast)
        }
    }

    func listBagSessionsAsync(
        previewLoadingMode: LocalROS2BagPreviewLoadingMode = .cachedOnly
    ) async throws -> [LocalROS2BagSession] {
        try await withCheckedThrowingContinuation { continuation in
            previewQueue.async {
                do {
                    continuation.resume(returning: try self.listBagSessions(previewLoadingMode: previewLoadingMode))
                } catch {
                    continuation.resume(throwing: error)
                }
            }
        }
    }

    func bagSessionWithPreviewScan(_ session: LocalROS2BagSession) async throws -> LocalROS2BagSession {
        try await withCheckedThrowingContinuation { continuation in
            previewQueue.async {
                do {
                    continuation.resume(
                        returning: try self.bagSession(
                            directoryURL: session.directoryURL,
                            previewLoadingMode: .scanIfNeeded
                        )
                    )
                } catch {
                    continuation.resume(throwing: error)
                }
            }
        }
    }

    func deleteBagSession(_ session: LocalROS2BagSession) throws {
        let isActiveSession = queue.sync {
            (self.acceptsRecords || self.database != nil)
                && self.bagDirectoryURL?.standardizedFileURL.path == session.directoryURL.standardizedFileURL.path
        }

        if isActiveSession {
            throw NSError(
                domain: "LocalROS2BagRecorder",
                code: 3,
                userInfo: [NSLocalizedDescriptionKey: "Stop recording before deleting the active local bag."]
            )
        }

        try fileManager.removeItem(at: session.directoryURL)
    }

    func recordPublishedTopic(topic: String, messageType: String, msg: [String: Any]) {
        guard acceptsRecords else { return }

        let payload: [String: Any] = [
            "op": "publish",
            "topic": topic,
            "msg": msg
        ]
        guard JSONSerialization.isValidJSONObject(payload),
              let data = try? JSONSerialization.data(withJSONObject: payload, options: []) else {
            recordEncodingFailure(topic: topic)
            return
        }

        let timestamp = Self.timestampNanoseconds(from: msg) ?? Self.nowNanoseconds()
        recordEncodedPublishPayload(data, topic: topic, messageType: messageType, timestampNanoseconds: timestamp)
    }

    func recordEncodedPublishPayload(
        _ data: Data,
        topic: String,
        messageType: String,
        timestampNanoseconds: Int64 = LocalROS2BagRecorder.nowNanoseconds()
    ) {
        queue.async {
            guard self.acceptsRecords, self.configuration.isEnabled, self.database != nil else { return }

            do {
                try self.enqueuePendingWrite(
                    PendingWrite(
                        topic: topic,
                        messageType: messageType,
                        timestampNanoseconds: timestampNanoseconds,
                        data: data
                    )
                )
            } catch {
                self.recordFailure("Failed to record \(topic): \(error.localizedDescription)")
            }
        }
    }

    private func storageRootURL() throws -> URL {
        if let baseDirectoryURL {
            return baseDirectoryURL
        }
        guard let documentsURL = fileManager.urls(for: .documentDirectory, in: .userDomainMask).first else {
            throw NSError(domain: "LocalROS2BagRecorder", code: 1, userInfo: [NSLocalizedDescriptionKey: "Documents directory unavailable"])
        }
        return documentsURL.appendingPathComponent("ROS2Bags", isDirectory: true)
    }

    private func bagSession(
        directoryURL: URL,
        previewLoadingMode: LocalROS2BagPreviewLoadingMode
    ) throws -> LocalROS2BagSession {
        let fileURLs = try fileManager.contentsOfDirectory(
            at: directoryURL,
            includingPropertiesForKeys: [.fileSizeKey, .contentModificationDateKey],
            options: [.skipsHiddenFiles]
        )
        .filter { url in
            url.pathExtension == "db3" || url.lastPathComponent == "metadata.yaml"
        }
        .sorted { lhs, rhs in
            lhs.lastPathComponent < rhs.lastPathComponent
        }

        let files = try fileURLs.map { url in
            let values = try url.resourceValues(forKeys: [.fileSizeKey, .contentModificationDateKey])
            let kind: LocalROS2BagFile.Kind
            if url.lastPathComponent == "metadata.yaml" {
                kind = .metadata
            } else if url.pathExtension == "db3" {
                kind = .sqliteChunk
            } else {
                kind = .other
            }

            return LocalROS2BagFile(
                url: url,
                relativePath: "\(directoryURL.lastPathComponent)/\(url.lastPathComponent)",
                kind: kind,
                byteCount: values.fileSize ?? 0,
                modifiedAt: values.contentModificationDate
            )
        }

        let directoryValues = try directoryURL.resourceValues(forKeys: [.contentModificationDateKey])
        let preview = preview(for: directoryURL, files: files, loadingMode: previewLoadingMode)
        return LocalROS2BagSession(
            directoryURL: directoryURL,
            files: files,
            byteCount: files.reduce(0) { $0 + $1.byteCount },
            modifiedAt: directoryValues.contentModificationDate ?? files.compactMap(\.modifiedAt).max(),
            preview: preview
        )
    }

    private func preview(
        for directoryURL: URL,
        files: [LocalROS2BagFile],
        loadingMode: LocalROS2BagPreviewLoadingMode
    ) -> LocalROS2BagSessionPreview? {
        let cacheKey = previewCacheKey(files: files)
        let cacheURL = directoryURL.appendingPathComponent(LocalROS2BagSessionPreview.cacheFileName)

        if let cached = readCachedPreview(at: cacheURL),
           cached.schemaVersion == LocalROS2BagSessionPreview.currentSchemaVersion,
           cached.cacheKey == cacheKey {
            return cached
        }

        guard loadingMode == .scanIfNeeded else { return nil }

        let preview = buildPreview(for: directoryURL, files: files, cacheKey: cacheKey)
        writeCachedPreview(preview, to: cacheURL)
        return preview
    }

    private func previewCacheKey(files: [LocalROS2BagFile]) -> String {
        files
            .sorted { $0.relativePath < $1.relativePath }
            .map { file in
                let modifiedMilliseconds = Int64((file.modifiedAt?.timeIntervalSince1970 ?? 0) * 1000)
                return "\(file.relativePath):\(file.byteCount):\(modifiedMilliseconds)"
            }
            .joined(separator: "|")
    }

    private func readCachedPreview(at url: URL) -> LocalROS2BagSessionPreview? {
        guard let data = try? Data(contentsOf: url) else { return nil }
        let decoder = JSONDecoder()
        decoder.dateDecodingStrategy = .iso8601
        return try? decoder.decode(LocalROS2BagSessionPreview.self, from: data)
    }

    private func writeCachedPreview(_ preview: LocalROS2BagSessionPreview, to url: URL) {
        let encoder = JSONEncoder()
        encoder.dateEncodingStrategy = .iso8601
        encoder.outputFormatting = [.prettyPrinted, .sortedKeys]
        guard let data = try? encoder.encode(preview) else { return }
        try? data.write(to: url, options: [.atomic])
    }

    private func buildPreview(
        for directoryURL: URL,
        files: [LocalROS2BagFile],
        cacheKey: String
    ) -> LocalROS2BagSessionPreview {
        var messageCount = 0
        var topicNames = Set<String>()
        var startedAtNanoseconds: Int64?
        var endedAtNanoseconds: Int64?
        var thumbnailSourceData: Data?

        for file in files where file.kind == .sqliteChunk {
            guard let chunkPreview = sqlitePreview(from: file.url) else { continue }
            messageCount += chunkPreview.messageCount
            topicNames.formUnion(chunkPreview.topicNames)

            if let start = chunkPreview.startedAtNanoseconds {
                startedAtNanoseconds = min(startedAtNanoseconds ?? start, start)
            }
            if let end = chunkPreview.endedAtNanoseconds {
                endedAtNanoseconds = max(endedAtNanoseconds ?? end, end)
            }
            if thumbnailSourceData == nil {
                thumbnailSourceData = chunkPreview.thumbnailSourceData
            }
        }

        let thumbnailRelativePath = thumbnailSourceData.flatMap {
            writeThumbnail(from: $0, in: directoryURL)
        }

        return LocalROS2BagSessionPreview(
            cacheKey: cacheKey,
            messageCount: messageCount,
            topicNames: Array(topicNames),
            startedAtNanoseconds: startedAtNanoseconds,
            endedAtNanoseconds: endedAtNanoseconds,
            thumbnailRelativePath: thumbnailRelativePath
        )
    }

    private struct SQLitePreview {
        let messageCount: Int
        let topicNames: Set<String>
        let startedAtNanoseconds: Int64?
        let endedAtNanoseconds: Int64?
        let thumbnailSourceData: Data?
    }

    private func sqlitePreview(from url: URL) -> SQLitePreview? {
        var database: OpaquePointer?
        guard sqlite3_open_v2(url.path, &database, SQLITE_OPEN_READONLY | SQLITE_OPEN_FULLMUTEX, nil) == SQLITE_OK else {
            if let database {
                sqlite3_close(database)
            }
            return nil
        }
        defer { sqlite3_close(database) }

        let messageCount = Int(sqliteInt64(
            database: database,
            sql: "SELECT COUNT(*) FROM messages"
        ) ?? 0)
        let start = sqliteInt64(
            database: database,
            sql: "SELECT MIN(timestamp) FROM messages"
        )
        let end = sqliteInt64(
            database: database,
            sql: "SELECT MAX(timestamp) FROM messages"
        )
        let topics = sqliteTopicNames(database: database)
        let thumbnailData = sqliteFirstCameraImageData(database: database)

        return SQLitePreview(
            messageCount: messageCount,
            topicNames: topics,
            startedAtNanoseconds: start,
            endedAtNanoseconds: end,
            thumbnailSourceData: thumbnailData
        )
    }

    private func sqliteInt64(database: OpaquePointer?, sql: String) -> Int64? {
        var statement: OpaquePointer?
        guard sqlite3_prepare_v2(database, sql, -1, &statement, nil) == SQLITE_OK else { return nil }
        defer { sqlite3_finalize(statement) }

        guard sqlite3_step(statement) == SQLITE_ROW,
              sqlite3_column_type(statement, 0) != SQLITE_NULL else { return nil }
        return sqlite3_column_int64(statement, 0)
    }

    private func sqliteTopicNames(database: OpaquePointer?) -> Set<String> {
        var statement: OpaquePointer?
        let sql = "SELECT DISTINCT name FROM topics ORDER BY name"
        guard sqlite3_prepare_v2(database, sql, -1, &statement, nil) == SQLITE_OK else { return [] }
        defer { sqlite3_finalize(statement) }

        var names = Set<String>()
        while sqlite3_step(statement) == SQLITE_ROW {
            guard let value = sqlite3_column_text(statement, 0) else { continue }
            names.insert(String(cString: value))
        }
        return names
    }

    private func sqliteFirstCameraImageData(database: OpaquePointer?) -> Data? {
        var statement: OpaquePointer?
        let sql = """
        SELECT messages.data
        FROM messages
        JOIN topics ON messages.topic_id = topics.id
        WHERE topics.name = ?
        ORDER BY messages.timestamp ASC
        LIMIT 1
        """
        guard sqlite3_prepare_v2(database, sql, -1, &statement, nil) == SQLITE_OK else { return nil }
        defer { sqlite3_finalize(statement) }

        sqlite3_bind_text(statement, 1, ROS2TopicRegistry.shared.topic(.cameraCompressed), -1, sqliteTransientDestructor)

        guard sqlite3_step(statement) == SQLITE_ROW,
              let bytes = sqlite3_column_blob(statement, 0) else { return nil }

        let byteCount = Int(sqlite3_column_bytes(statement, 0))
        let payloadData = Data(bytes: bytes, count: byteCount)
        return cameraImageData(fromRosbridgePayload: payloadData)
    }

    private func cameraImageData(fromRosbridgePayload payloadData: Data) -> Data? {
        guard let payload = try? JSONSerialization.jsonObject(with: payloadData) as? [String: Any],
              let msg = payload["msg"] as? [String: Any],
              let format = (msg["format"] as? String)?.lowercased(),
              format.contains("jpeg") || format.contains("jpg"),
              let encoded = msg["data"] as? String else { return nil }

        return Data(base64Encoded: encoded)
    }

    private func writeThumbnail(from imageData: Data, in directoryURL: URL) -> String? {
        #if canImport(UIKit)
        guard let image = UIImage(data: imageData) else { return nil }

        let maxSide: CGFloat = 160
        let sourceSize = image.size
        guard sourceSize.width > 0, sourceSize.height > 0 else { return nil }

        let scale = min(maxSide / sourceSize.width, maxSide / sourceSize.height, 1)
        let thumbnailSize = CGSize(
            width: max(1, floor(sourceSize.width * scale)),
            height: max(1, floor(sourceSize.height * scale))
        )
        let format = UIGraphicsImageRendererFormat()
        format.scale = 1
        format.opaque = true

        let renderer = UIGraphicsImageRenderer(size: thumbnailSize, format: format)
        let thumbnailData = renderer.jpegData(withCompressionQuality: 0.72) { context in
            UIColor.systemBackground.setFill()
            context.fill(CGRect(origin: .zero, size: thumbnailSize))
            image.draw(in: CGRect(origin: .zero, size: thumbnailSize))
        }

        let thumbnailURL = directoryURL.appendingPathComponent(LocalROS2BagSessionPreview.thumbnailFileName)
        do {
            try thumbnailData.write(to: thumbnailURL, options: [.atomic])
            return LocalROS2BagSessionPreview.thumbnailFileName
        } catch {
            return nil
        }
        #else
        return nil
        #endif
    }

    private func bagName(sessionID: UUID?, date: Date) -> String {
        let formatter = DateFormatter()
        formatter.calendar = Calendar(identifier: .gregorian)
        formatter.locale = Locale(identifier: "en_US_POSIX")
        formatter.timeZone = TimeZone(secondsFromGMT: 0)
        formatter.dateFormat = "yyyyMMdd_HHmmss"

        let sessionSuffix = sessionID?.uuidString.prefix(8).lowercased() ?? UUID().uuidString.prefix(8).lowercased()
        return "mapeverything_\(formatter.string(from: date))_\(sessionSuffix)"
    }

    private func resetBagState() {
        cancelPendingFlush()
        pendingWrites.removeAll()
        pendingWriteBytes = 0
        database = nil
        bagDirectoryURL = nil
        currentChunkURL = nil
        currentChunkIndex = 0
        currentChunkBytes = 0
        currentChunkMessageCount = 0
        currentChunkStartNanoseconds = nil
        currentChunkEndNanoseconds = nil
        nextTopicID = 1
        nextMessageID = 1
        topicsByName.removeAll()
        topicsInsertedInCurrentChunk.removeAll()
        bagFiles.removeAll()
    }

    private func openChunk(index: Int) throws {
        guard let bagDirectoryURL else {
            throw NSError(domain: "LocalROS2BagRecorder", code: 2, userInfo: [NSLocalizedDescriptionKey: "Bag directory unavailable"])
        }

        let chunkURL = bagDirectoryURL.appendingPathComponent("mapeverything_\(index).db3")
        currentChunkURL = chunkURL
        currentChunkIndex = index
        currentChunkBytes = 0
        currentChunkMessageCount = 0
        currentChunkStartNanoseconds = nil
        currentChunkEndNanoseconds = nil
        topicsInsertedInCurrentChunk.removeAll()

        if sqlite3_open_v2(chunkURL.path, &database, SQLITE_OPEN_CREATE | SQLITE_OPEN_READWRITE | SQLITE_OPEN_FULLMUTEX, nil) != SQLITE_OK {
            throw SQLiteError(database: database, fallback: "Unable to open SQLite chunk")
        }

        try execute("PRAGMA synchronous=NORMAL")
        try execute("CREATE TABLE IF NOT EXISTS topics(id INTEGER PRIMARY KEY, name TEXT NOT NULL, type TEXT NOT NULL, serialization_format TEXT NOT NULL, offered_qos_profiles TEXT NOT NULL)")
        try execute("CREATE TABLE IF NOT EXISTS messages(id INTEGER PRIMARY KEY, topic_id INTEGER NOT NULL, timestamp INTEGER NOT NULL, data BLOB NOT NULL)")
        try execute("CREATE INDEX IF NOT EXISTS timestamp_idx ON messages (timestamp ASC)")
    }

    private func rotateChunk(startingAt timestampNanoseconds: Int64) throws {
        finalizeCurrentChunk()
        closeDatabase()
        try openChunk(index: currentChunkIndex + 1)
        currentChunkStartNanoseconds = timestampNanoseconds
    }

    private func closeCurrentBag(writeMetadata: Bool) {
        do {
            try flushPendingWrites(publishStatsAfterFlush: false)
        } catch {
            recordFailure("Failed to flush local rosbag: \(error.localizedDescription)")
            return
        }
        finalizeCurrentChunk()
        closeDatabase()
        if writeMetadata {
            self.writeMetadata()
        }
    }

    private func finalizeCurrentChunk() {
        guard let currentChunkURL,
              currentChunkMessageCount > 0,
              !bagFiles.contains(where: { $0.relativePath == currentChunkURL.lastPathComponent }) else {
            return
        }

        let fileSize = (try? fileManager.attributesOfItem(atPath: currentChunkURL.path)[.size] as? NSNumber)?.intValue ?? currentChunkBytes
        let start = currentChunkStartNanoseconds ?? Self.nowNanoseconds()
        let end = currentChunkEndNanoseconds ?? start
        bagFiles.append(
            BagFileInfo(
                relativePath: currentChunkURL.lastPathComponent,
                startingTimeNanoseconds: start,
                endingTimeNanoseconds: end,
                messageCount: currentChunkMessageCount,
                byteCount: fileSize
            )
        )
    }

    private func closeDatabase() {
        if let database {
            sqlite3_close(database)
        }
        database = nil
    }

    private func enqueuePendingWrite(_ write: PendingWrite) throws {
        pendingWrites.append(write)
        pendingWriteBytes += write.data.count

        if pendingWrites.count >= Self.writeBatchMaxMessages
            || pendingWriteBytes >= Self.writeBatchMaxBytes {
            try flushPendingWrites(publishStatsAfterFlush: true)
        } else {
            schedulePendingFlush()
        }
    }

    private func schedulePendingFlush() {
        guard pendingFlushWorkItem == nil else { return }

        let workItem = DispatchWorkItem { [weak self] in
            guard let self else { return }
            do {
                try self.flushPendingWrites(publishStatsAfterFlush: true)
            } catch {
                self.recordFailure("Failed to flush local rosbag: \(error.localizedDescription)")
            }
        }
        pendingFlushWorkItem = workItem
        queue.asyncAfter(deadline: .now() + Self.writeBatchFlushDelay, execute: workItem)
    }

    private func cancelPendingFlush() {
        pendingFlushWorkItem?.cancel()
        pendingFlushWorkItem = nil
    }

    private func flushPendingWrites(publishStatsAfterFlush: Bool) throws {
        cancelPendingFlush()
        guard !pendingWrites.isEmpty else { return }
        guard configuration.isEnabled, database != nil else {
            pendingWrites.removeAll()
            pendingWriteBytes = 0
            return
        }

        let writes = pendingWrites
        pendingWrites.removeAll()
        pendingWriteBytes = 0

        var transactionOpen = false
        var messageStatement: OpaquePointer?

        func beginTransaction() throws {
            try execute("BEGIN IMMEDIATE TRANSACTION")
            transactionOpen = true
            messageStatement = try prepare(Self.insertMessageSQL)
        }

        func closeStatement() {
            if let messageStatement {
                sqlite3_finalize(messageStatement)
            }
            messageStatement = nil
        }

        func commitTransaction() throws {
            closeStatement()
            try execute("COMMIT")
            transactionOpen = false
        }

        func rollbackTransaction() {
            closeStatement()
            if transactionOpen {
                try? execute("ROLLBACK")
                transactionOpen = false
            }
        }

        do {
            for write in writes {
                if currentChunkMessageCount > 0,
                   currentChunkBytes + write.data.count > configuration.maxChunkBytes {
                    if transactionOpen {
                        try commitTransaction()
                    }
                    try rotateChunk(startingAt: write.timestampNanoseconds)
                }

                if !transactionOpen {
                    try beginTransaction()
                }

                let topicID = try ensureTopic(write.topic, messageType: write.messageType)
                try insertMessage(
                    id: nextMessageID,
                    topicID: topicID,
                    timestampNanoseconds: write.timestampNanoseconds,
                    data: write.data,
                    statement: messageStatement
                )
                nextMessageID += 1
                currentChunkMessageCount += 1
                currentChunkBytes += write.data.count
                currentChunkStartNanoseconds = currentChunkStartNanoseconds ?? write.timestampNanoseconds
                currentChunkEndNanoseconds = write.timestampNanoseconds

                if var topicInfo = topicsByName[write.topic] {
                    topicInfo.messageCount += 1
                    topicsByName[write.topic] = topicInfo
                }
            }

            if transactionOpen {
                try commitTransaction()
            }

            if publishStatsAfterFlush {
                publishStats(isRecording: true)
            }
        } catch {
            rollbackTransaction()
            throw error
        }
    }

    private func ensureTopic(_ topic: String, messageType: String) throws -> Int64 {
        let topicInfo: TopicInfo
        if let existing = topicsByName[topic] {
            topicInfo = existing
        } else {
            topicInfo = TopicInfo(id: nextTopicID, name: topic, type: messageType, messageCount: 0)
            topicsByName[topic] = topicInfo
            nextTopicID += 1
        }

        if !topicsInsertedInCurrentChunk.contains(topic) {
            try insertTopic(topicInfo)
            topicsInsertedInCurrentChunk.insert(topic)
        }

        return topicInfo.id
    }

    private func insertTopic(_ topic: TopicInfo) throws {
        let sql = "INSERT OR IGNORE INTO topics(id, name, type, serialization_format, offered_qos_profiles) VALUES (?, ?, ?, ?, ?)"
        let statement = try prepare(sql)
        defer { sqlite3_finalize(statement) }

        sqlite3_bind_int64(statement, 1, topic.id)
        sqlite3_bind_text(statement, 2, topic.name, -1, sqliteTransientDestructor)
        sqlite3_bind_text(statement, 3, topic.type, -1, sqliteTransientDestructor)
        sqlite3_bind_text(statement, 4, Self.serializationFormat, -1, sqliteTransientDestructor)
        sqlite3_bind_text(statement, 5, "", -1, sqliteTransientDestructor)

        try stepDone(statement)
    }

    private func insertMessage(
        id: Int64,
        topicID: Int64,
        timestampNanoseconds: Int64,
        data: Data,
        statement: OpaquePointer?
    ) throws {
        sqlite3_bind_int64(statement, 1, id)
        sqlite3_bind_int64(statement, 2, topicID)
        sqlite3_bind_int64(statement, 3, timestampNanoseconds)
        _ = data.withUnsafeBytes { rawBuffer in
            sqlite3_bind_blob(statement, 4, rawBuffer.baseAddress, Int32(data.count), sqliteTransientDestructor)
        }

        try stepDone(statement)
        sqlite3_reset(statement)
        sqlite3_clear_bindings(statement)
    }

    private func execute(_ sql: String) throws {
        guard sqlite3_exec(database, sql, nil, nil, nil) == SQLITE_OK else {
            throw SQLiteError(database: database, fallback: "SQLite exec failed")
        }
    }

    private func prepare(_ sql: String) throws -> OpaquePointer? {
        var statement: OpaquePointer?
        guard sqlite3_prepare_v2(database, sql, -1, &statement, nil) == SQLITE_OK else {
            throw SQLiteError(database: database, fallback: "SQLite prepare failed")
        }
        return statement
    }

    private func stepDone(_ statement: OpaquePointer?) throws {
        guard sqlite3_step(statement) == SQLITE_DONE else {
            throw SQLiteError(database: database, fallback: "SQLite step failed")
        }
    }

    private func writeMetadata() {
        guard let bagDirectoryURL else { return }

        let metadataURL = bagDirectoryURL.appendingPathComponent("metadata.yaml")
        let metadata = metadataYAML()
        do {
            try metadata.write(to: metadataURL, atomically: true, encoding: .utf8)
        } catch {
            recordFailure("Failed to write metadata.yaml: \(error.localizedDescription)")
        }
    }

    private func metadataYAML() -> String {
        let sortedTopics = topicsByName.values.sorted { $0.name < $1.name }
        let sortedFiles = bagFiles.sorted { $0.relativePath < $1.relativePath }
        let messageCount = sortedTopics.reduce(0) { $0 + $1.messageCount }
        let start = sortedFiles.map(\.startingTimeNanoseconds).min() ?? Self.nowNanoseconds()
        let end = sortedFiles.map(\.endingTimeNanoseconds).max() ?? start
        let duration = max(0, end - start)

        var lines: [String] = [
            "rosbag2_bagfile_information:",
            "  version: 8",
            "  storage_identifier: \(Self.storageIdentifier)",
            "  duration:",
            "    nanoseconds: \(duration)",
            "  starting_time:",
            "    nanoseconds_since_epoch: \(start)",
            "  message_count: \(messageCount)",
            "  topics_with_message_count:"
        ]

        if sortedTopics.isEmpty {
            lines.append("    []")
        } else {
            for topic in sortedTopics {
                lines.append("    - topic_metadata:")
                lines.append("        name: \(yamlString(topic.name))")
                lines.append("        type: \(yamlString(topic.type))")
                lines.append("        serialization_format: \(yamlString(Self.serializationFormat))")
                lines.append("        offered_qos_profiles: ''")
                lines.append("      message_count: \(topic.messageCount)")
            }
        }

        lines.append("  compression_format: ''")
        lines.append("  compression_mode: ''")
        lines.append("  relative_file_paths:")
        if sortedFiles.isEmpty {
            lines.append("    []")
        } else {
            for file in sortedFiles {
                lines.append("    - \(yamlString(file.relativePath))")
            }
        }

        lines.append("  files:")
        if sortedFiles.isEmpty {
            lines.append("    []")
        } else {
            for file in sortedFiles {
                lines.append("    - path: \(yamlString(file.relativePath))")
                lines.append("      starting_time:")
                lines.append("        nanoseconds_since_epoch: \(file.startingTimeNanoseconds)")
                lines.append("      duration:")
                lines.append("        nanoseconds: \(file.durationNanoseconds)")
                lines.append("      message_count: \(file.messageCount)")
            }
        }

        lines.append("  custom_data:")
        lines.append("    mapeverything.serialization_format: \(yamlString(Self.serializationFormat))")
        lines.append("    mapeverything.serialization_note: \(yamlString("Messages are stored as rosbridge publish JSON payloads, not native ROS2 CDR bytes."))")

        return lines.joined(separator: "\n") + "\n"
    }

    private func yamlString(_ value: String) -> String {
        "'\(value.replacingOccurrences(of: "'", with: "''"))'"
    }

    private func publishStats(isRecording: Bool) {
        let hasOpenUnfinalizedChunk = database != nil
            && currentChunkURL != nil
            && !bagFiles.contains(where: { $0.relativePath == currentChunkURL?.lastPathComponent })
        let stats = LocalROS2BagRecorderStats(
            isEnabled: configuration.isEnabled,
            isRecording: isRecording && configuration.isEnabled && database != nil,
            bagDirectoryURL: bagDirectoryURL,
            currentChunkURL: currentChunkURL,
            chunkCount: bagFiles.count + (hasOpenUnfinalizedChunk ? 1 : 0),
            messageCount: topicsByName.values.reduce(0) { $0 + $1.messageCount },
            currentChunkBytes: currentChunkBytes,
            maxChunkBytes: configuration.maxChunkBytes,
            startedAt: startedAt,
            stoppedAt: stoppedAt,
            lastError: lastError,
            lastErrorAt: lastErrorAt
        )

        DispatchQueue.main.async {
            self.stats = stats
        }
    }

    private func recordEncodingFailure(topic: String) {
        queue.async {
            self.recordFailure("Failed to encode local rosbag payload for \(topic).")
        }
    }

    private func recordFailure(_ message: String) {
        cancelPendingFlush()
        pendingWrites.removeAll()
        pendingWriteBytes = 0
        lastError = message
        lastErrorAt = Date()
        closeDatabase()
        publishStats(isRecording: false)
    }

    static func timestampNanoseconds(from msg: [String: Any]) -> Int64? {
        if let header = msg["header"] as? [String: Any],
           let timestamp = timestampNanoseconds(fromHeader: header) {
            return timestamp
        }

        if let transforms = msg["transforms"] as? [[String: Any]],
           let firstHeader = transforms.first?["header"] as? [String: Any] {
            return timestampNanoseconds(fromHeader: firstHeader)
        }

        return nil
    }

    private static func timestampNanoseconds(fromHeader header: [String: Any]) -> Int64? {
        guard let stamp = header["stamp"] as? [String: Any],
              let sec = integerValue(stamp["sec"]),
              let nanosec = integerValue(stamp["nanosec"]) else {
            return nil
        }
        return sec * 1_000_000_000 + nanosec
    }

    private static func integerValue(_ value: Any?) -> Int64? {
        switch value {
        case let value as Int:
            return Int64(value)
        case let value as Int64:
            return value
        case let value as UInt64:
            return value <= UInt64(Int64.max) ? Int64(value) : nil
        case let value as Double:
            return value.isFinite ? Int64(value) : nil
        case let value as NSNumber:
            return value.int64Value
        default:
            return nil
        }
    }

    static func nowNanoseconds(date: Date = Date()) -> Int64 {
        Int64(date.timeIntervalSince1970 * 1_000_000_000)
    }
}

private let sqliteTransientDestructor = unsafeBitCast(-1, to: sqlite3_destructor_type.self)

private struct SQLiteError: LocalizedError {
    let message: String

    init(database: OpaquePointer?, fallback: String) {
        if let database, let sqliteMessage = sqlite3_errmsg(database) {
            message = String(cString: sqliteMessage)
        } else {
            message = fallback
        }
    }

    var errorDescription: String? {
        message
    }
}
