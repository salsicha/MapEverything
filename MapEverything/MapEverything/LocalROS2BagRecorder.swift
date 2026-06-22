//
//  LocalROS2BagRecorder.swift
//  MapEverything
//

import Foundation
import Combine
import SQLite3

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

    private let queue = DispatchQueue(label: "com.mapeverything.localROS2BagRecorder", qos: .utility)
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
        queue.sync {}
    }

    func listBagSessions() throws -> [LocalROS2BagSession] {
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
            return try bagSession(directoryURL: url)
        }
        .sorted { lhs, rhs in
            (lhs.modifiedAt ?? .distantPast) > (rhs.modifiedAt ?? .distantPast)
        }
    }

    func deleteBagSession(_ session: LocalROS2BagSession) throws {
        if stats.isRecording,
           let activeURL = stats.bagDirectoryURL,
           activeURL.standardizedFileURL.path == session.directoryURL.standardizedFileURL.path {
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
            guard self.configuration.isEnabled, self.database != nil else { return }

            do {
                if self.currentChunkMessageCount > 0,
                   self.currentChunkBytes + data.count > self.configuration.maxChunkBytes {
                    try self.rotateChunk(startingAt: timestampNanoseconds)
                }

                let topicID = try self.ensureTopic(topic, messageType: messageType)
                try self.insertMessage(
                    id: self.nextMessageID,
                    topicID: topicID,
                    timestampNanoseconds: timestampNanoseconds,
                    data: data
                )
                self.nextMessageID += 1
                self.currentChunkMessageCount += 1
                self.currentChunkBytes += data.count
                self.currentChunkStartNanoseconds = self.currentChunkStartNanoseconds ?? timestampNanoseconds
                self.currentChunkEndNanoseconds = timestampNanoseconds

                if var topicInfo = self.topicsByName[topic] {
                    topicInfo.messageCount += 1
                    self.topicsByName[topic] = topicInfo
                }

                self.publishStats(isRecording: true)
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

    private func bagSession(directoryURL: URL) throws -> LocalROS2BagSession {
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
        return LocalROS2BagSession(
            directoryURL: directoryURL,
            files: files,
            byteCount: files.reduce(0) { $0 + $1.byteCount },
            modifiedAt: directoryValues.contentModificationDate ?? files.compactMap(\.modifiedAt).max()
        )
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

    private func insertMessage(id: Int64, topicID: Int64, timestampNanoseconds: Int64, data: Data) throws {
        let sql = "INSERT INTO messages(id, topic_id, timestamp, data) VALUES (?, ?, ?, ?)"
        let statement = try prepare(sql)
        defer { sqlite3_finalize(statement) }

        sqlite3_bind_int64(statement, 1, id)
        sqlite3_bind_int64(statement, 2, topicID)
        sqlite3_bind_int64(statement, 3, timestampNanoseconds)
        _ = data.withUnsafeBytes { rawBuffer in
            sqlite3_bind_blob(statement, 4, rawBuffer.baseAddress, Int32(data.count), sqliteTransientDestructor)
        }

        try stepDone(statement)
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
