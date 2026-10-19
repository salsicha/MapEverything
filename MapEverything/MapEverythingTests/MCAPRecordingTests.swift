import Foundation
import Testing
import UIKit
@testable import MapEverything

struct MCAPRecordingTests {
    @Test("App-loaded recording configuration defaults to MCAP")
    func defaultFormat() throws {
        let name = "MCAPDefault.\(UUID())"
        let defaults = try #require(UserDefaults(suiteName: name))
        defer { defaults.removePersistentDomain(forName: name) }
        #expect(LocalROS2BagRecorderConfiguration.load(from: defaults).storageFormat == .mcap)
        #expect(LocalROS2BagRecorderConfiguration(isEnabled: true, maxChunkBytes: 1024).storageFormat == .mcap)
    }

    @Test("MCAP rotation preserves payloads, independent chunks, preview and thumbnails")
    func recordingRoundTrip() async throws {
        let root = FileManager.default.temporaryDirectory.appendingPathComponent("MCAP-\(UUID())")
        let recorder = LocalROS2BagRecorder(baseDirectoryURL: root)
        defer { recorder.stopAndWait(); try? FileManager.default.removeItem(at: root) }
        recorder.start(sessionID: UUID(), configuration: .init(isEnabled: true, maxChunkBytes: 1))
        let directory = try #require(recorder.currentArtifactDirectoryURL)
        let jpeg = try #require(UIGraphicsImageRenderer(size: CGSize(width: 8, height: 8)).image { ctx in
            UIColor.blue.setFill(); ctx.fill(CGRect(x: 0, y: 0, width: 8, height: 8))
        }.jpegData(compressionQuality: 0.8))
        let cameraTopic = ROS2TopicRegistry.shared.topic(.cameraCompressed)
        let payload = try JSONSerialization.data(withJSONObject: ["op": "publish", "topic": cameraTopic,
            "msg": ["format": "jpeg", "data": jpeg.base64EncodedString()]])
        for time: Int64 in [100, 200, 300] {
            recorder.recordEncodedPublishPayload(payload, topic: cameraTopic,
                messageType: "sensor_msgs/msg/CompressedImage", timestampNanoseconds: time)
        }
        recorder.flushAndWait()
        // Every flushed file is sealed even before stop, with no WAL sidecars.
        var times: [Int64] = []
        for i in 0..<3 {
            let url = directory.appendingPathComponent("mapeverything_\(i).mcap")
            let bytes = try Data(contentsOf: url)
            #expect(bytes.prefix(8) == LocalMCAPWriter.magic)
            #expect(bytes.suffix(8) == LocalMCAPWriter.magic)
            try LocalMCAPReader.scan(url) { message in
                #expect(message.data == payload)
                #expect(message.type == "sensor_msgs/msg/CompressedImage")
                times.append(message.timestamp)
            }
        }
        #expect(times == [100, 200, 300])
        recorder.stopAndWait()
        let metadata = try String(contentsOf: directory.appendingPathComponent("metadata.yaml"), encoding: .utf8)
        #expect(metadata.contains("storage_identifier: mcap"))
        #expect(metadata.contains("message_count: 3"))
        #expect(metadata.contains("serialization_format: 'rosbridge_json'"))
        let session = try #require(recorder.listBagSessions().first)
        #expect(session.chunkCount == 3)
        #expect(session.files.filter { $0.kind == .mcapChunk }.count == 3)
        let loaded = try await recorder.bagSessionWithPreviewScan(session)
        #expect(loaded.preview?.messageCount == 3)
        #expect(loaded.preview?.thumbnailRelativePath != nil)
    }

    @Test("A failed MCAP batch rolls back without damaging earlier committed messages")
    func recorderFailurePreservesCommittedPrefix() throws {
        let root = FileManager.default.temporaryDirectory.appendingPathComponent("MCAPFailure-\(UUID())")
        let recorder = LocalROS2BagRecorder(baseDirectoryURL: root)
        defer { recorder.stopAndWait(); try? FileManager.default.removeItem(at: root) }
        recorder.start(sessionID: UUID(), configuration: .init(isEnabled: true, maxChunkBytes: 1_048_576))
        let directory = try #require(recorder.currentArtifactDirectoryURL)
        recorder.recordEncodedPublishPayload(Data("{}".utf8), topic: "/test", messageType: "std_msgs/msg/String", timestampNanoseconds: 100)
        recorder.flushAndWait()
        // An invalid unsigned MCAP timestamp fails after begin(), exercising
        // the real recorder rollback/retry path without a test-only writer.
        recorder.recordEncodedPublishPayload(Data("{}".utf8), topic: "/test", messageType: "std_msgs/msg/String", timestampNanoseconds: -1)
        recorder.stopAndWait()
        let file = directory.appendingPathComponent("mapeverything_0.mcap")
        var times: [Int64] = []
        try LocalMCAPReader.scan(file) { times.append($0.timestamp) }
        #expect(times == [100])
        #expect(try Data(contentsOf: file).suffix(8) == LocalMCAPWriter.magic)
        let metadata = try String(contentsOf: directory.appendingPathComponent("metadata.yaml"), encoding: .utf8)
        #expect(metadata.contains("message_count: 1"))
    }

    @Test("MCAP rollback retains committed data and permits a retry without duplicates")
    func rollbackAndRetry() throws {
        let url = FileManager.default.temporaryDirectory.appendingPathComponent("rollback-\(UUID()).mcap")
        defer { try? FileManager.default.removeItem(at: url) }
        let writer = try LocalMCAPWriter(url: url)
        try writer.begin()
        try writer.channel(id: 1, topic: "/test", type: "std_msgs/msg/String")
        try writer.message(id: 1, channel: 1, timestamp: 100, data: Data("{}".utf8))
        try writer.commit()
        try writer.begin()
        try writer.message(id: 2, channel: 1, timestamp: 150, data: Data("{}".utf8))
        try writer.rollback()
        try writer.begin()
        try writer.message(id: 2, channel: 1, timestamp: 200, data: Data("{}".utf8))
        try writer.commit()
        var times: [Int64] = []
        try LocalMCAPReader.scan(url) { times.append($0.timestamp) }
        #expect(times == [100, 200])
    }

    @Test("Interrupted MCAP tail preserves complete records and rejects non-MCAP input")
    func interruptedTail() throws {
        let url = FileManager.default.temporaryDirectory.appendingPathComponent("tail-\(UUID()).mcap")
        defer { try? FileManager.default.removeItem(at: url) }
        let writer = try LocalMCAPWriter(url: url)
        try writer.begin()
        try writer.channel(id: 1, topic: "/test", type: "std_msgs/msg/String")
        try writer.message(id: 1, channel: 1, timestamp: 100, data: Data("{}".utf8))
        try writer.commit()
        try writer.begin()
        // Simulate termination before a complete next record/footer.
        let append = try FileHandle(forWritingTo: url)
        try append.seekToEnd(); try append.write(contentsOf: Data([0x05, 0x22])); try append.close()
        var count = 0
        try LocalMCAPReader.scan(url) { _ in count += 1 }
        #expect(count == 1)
        let bad = url.appendingPathExtension("bad")
        defer { try? FileManager.default.removeItem(at: bad) }
        try Data("not MCAP".utf8).write(to: bad)
        #expect(throws: MCAPError.self) { try LocalMCAPReader.scan(bad) { _ in } }
    }
}
