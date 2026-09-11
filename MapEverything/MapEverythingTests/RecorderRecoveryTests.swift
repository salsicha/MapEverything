import Foundation
import SQLite3
import Testing
@testable import MapEverything

struct RecorderRecoveryTests {
    private func withRecorder(
        maxChunkBytes: Int = 8 * 1_048_576,
        _ body: (LocalROS2BagRecorder, URL, UUID) throws -> Void
    ) throws {
        let root = FileManager.default.temporaryDirectory
            .appendingPathComponent("RecorderRecovery-\(UUID().uuidString)")
        let recorder = LocalROS2BagRecorder(baseDirectoryURL: root)
        defer {
            recorder.stopAndWait()
            try? FileManager.default.removeItem(at: root)
        }
        let id = UUID()
        recorder.start(sessionID: id, configuration: .init(isEnabled: true, maxChunkBytes: maxChunkBytes))
        let directory = try #require(recorder.currentArtifactDirectoryURL)
        try body(recorder, directory, id)
    }

    private func record(_ recorder: LocalROS2BagRecorder, timestamp: Int64) {
        recorder.recordEncodedPublishPayload(
            Data("{}".utf8), topic: "/test", messageType: "std_msgs/msg/String",
            timestampNanoseconds: timestamp
        )
    }

    private func open(_ url: URL) throws -> OpaquePointer {
        var database: OpaquePointer?
        let result = sqlite3_open(url.path, &database)
        guard result == SQLITE_OK, let database else {
            sqlite3_close(database)
            throw Failure.sqlite
        }
        sqlite3_busy_timeout(database, 2_000)
        return database
    }

    private func execute(_ sql: String, on database: OpaquePointer) throws {
        guard sqlite3_exec(database, sql, nil, nil, nil) == SQLITE_OK else { throw Failure.sqlite }
    }

    private func timestamps(_ url: URL) throws -> [Int64] {
        let database = try open(url)
        defer { sqlite3_close(database) }
        var statement: OpaquePointer?
        guard sqlite3_prepare_v2(database, "SELECT timestamp FROM messages ORDER BY id", -1, &statement, nil) == SQLITE_OK else {
            throw Failure.sqlite
        }
        defer { sqlite3_finalize(statement) }
        var values: [Int64] = []
        while sqlite3_step(statement) == SQLITE_ROW {
            values.append(sqlite3_column_int64(statement, 0))
        }
        return values
    }

    @Test("A transient database lock retains and retries the pending messages")
    func transientLockPreservesBatch() throws {
        try withRecorder { recorder, directory, _ in
            let chunk = directory.appendingPathComponent("mapeverything_0.db3")
            record(recorder, timestamp: 1)
            recorder.flushAndWait()
            let blocker = try open(chunk)
            defer { sqlite3_close(blocker) }
            try execute("BEGIN EXCLUSIVE", on: blocker)
            record(recorder, timestamp: 2)
            recorder.flushAndWait() // Times out once, retaining the batch.
            try execute("ROLLBACK", on: blocker)
            record(recorder, timestamp: 3)
            recorder.stopAndWait()
            #expect(try timestamps(chunk) == [1, 2, 3])
        }
    }

    @Test("A failed flush retries automatically without another sensor sample")
    func automaticRetryPreservesBatch() throws {
        try withRecorder { recorder, directory, _ in
            let chunk = directory.appendingPathComponent("mapeverything_0.db3")
            let database = try open(chunk)
            defer { sqlite3_close(database) }
            try execute("CREATE TRIGGER reject_insert BEFORE INSERT ON messages BEGIN SELECT RAISE(ABORT, 'test'); END", on: database)
            record(recorder, timestamp: 1)
            recorder.flushAndWait()
            try execute("DROP TRIGGER reject_insert", on: database)
            // No explicit flush or stop until the retry has persisted the row.
            let deadline = Date().addingTimeInterval(3)
            while try timestamps(chunk).isEmpty, Date() < deadline {
                Thread.sleep(forTimeInterval: 0.02)
            }
            #expect(try timestamps(chunk) == [1])
        }
    }

    @Test("Retrying after chunk rotation does not replay the committed prefix")
    func retryAfterPartialCommit() throws {
        try withRecorder(maxChunkBytes: 2) { recorder, directory, _ in
            let nextChunk = directory.appendingPathComponent("mapeverything_1.db3")
            let database = try open(nextChunk)
            defer { sqlite3_close(database) }
            try execute("CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER NOT NULL, timestamp INTEGER NOT NULL, data BLOB NOT NULL)", on: database)
            try execute("CREATE TRIGGER reject_insert BEFORE INSERT ON messages BEGIN SELECT RAISE(ABORT, 'test'); END", on: database)
            record(recorder, timestamp: 1)
            record(recorder, timestamp: 2)
            recorder.flushAndWait() // Commits chunk 0, then fails in chunk 1.
            try execute("DROP TRIGGER reject_insert", on: database)
            recorder.stopAndWait()
            #expect(try timestamps(directory.appendingPathComponent("mapeverything_0.db3")) == [1])
            #expect(try timestamps(nextChunk) == [2])
            let metadata = try String(contentsOf: directory.appendingPathComponent("metadata.yaml"), encoding: .utf8)
            #expect(metadata.contains("  message_count: 2\n"))
        }
    }

    @Test("Persistent write errors stop recording after bounded retries")
    func persistentFailureStopsRecording() throws {
        try withRecorder { recorder, directory, _ in
            let database = try open(directory.appendingPathComponent("mapeverything_0.db3"))
            defer { sqlite3_close(database) }
            try execute("CREATE TRIGGER reject_insert BEFORE INSERT ON messages BEGIN SELECT RAISE(ABORT, 'test'); END", on: database)
            record(recorder, timestamp: 1)
            recorder.flushAndWait()
            let deadline = Date().addingTimeInterval(3)
            while recorder.isAcceptingRecords, Date() < deadline {
                Thread.sleep(forTimeInterval: 0.02)
            }
            #expect(!recorder.isAcceptingRecords)
        }
    }

    @Test("Rapid restarts for one session preserve every recording segment")
    func rapidRestartsUseUniqueDirectories() throws {
        try withRecorder { recorder, firstDirectory, id in
            var directories = [firstDirectory]
            record(recorder, timestamp: 0)
            for timestamp in 1...5 {
                recorder.start(sessionID: id, configuration: .init(isEnabled: true, maxChunkBytes: 8 * 1_048_576))
                directories.append(try #require(recorder.currentArtifactDirectoryURL))
                record(recorder, timestamp: Int64(timestamp))
            }
            recorder.stopAndWait()
            #expect(Set(directories).count == 6)
            for (timestamp, directory) in directories.enumerated() {
                #expect(try timestamps(directory.appendingPathComponent("mapeverything_0.db3")) == [Int64(timestamp)])
            }
        }
    }

    private enum Failure: Error { case sqlite }
}
