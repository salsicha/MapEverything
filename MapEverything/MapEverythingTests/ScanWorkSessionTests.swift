import Foundation
import SQLite3
import Testing
import simd
@testable import MapEverything

@MainActor
struct ScanWorkSessionTests {
    @Test("Delayed frame publications cannot enter a restarted recording")
    func delayedFrameCannotWriteToNewBag() async throws {
        let root = FileManager.default.temporaryDirectory
            .appendingPathComponent("ScanWorkSession-\(UUID().uuidString)")
        let recorder = LocalROS2BagRecorder(baseDirectoryURL: root)
        defer {
            recorder.stopAndWait()
            try? FileManager.default.removeItem(at: root)
        }
        let configuration = LocalROS2BagRecorderConfiguration(isEnabled: true, maxChunkBytes: 8 * 1_048_576)
        let client = ROS2BridgeClient(
            topicRegistry: ROS2TopicRegistry(enabledStreams: [.pose, .pointCloud]),
            localBagRecorder: recorder
        )
        let oldSession = ScanWorkSession()
        recorder.start(sessionID: UUID(), configuration: configuration)
        let firstDirectory = try #require(recorder.currentArtifactDirectoryURL)
        oldSession.withPublishing {
            publishFrame(client, x: 1)
        }

        let started = Gate()
        let finishProcessing = Gate()
        let delayedFrame = Task { @MainActor in
            await started.open()
            await finishProcessing.wait()
            // Simulates inference finishing after stop/start. This exercises
            // both the normal send and the point-cloud buffer/record path.
            oldSession.withPublishing {
                publishFrame(client, x: 999)
            }
        }
        await started.wait()
        oldSession.cancel()
        recorder.stopAndWait()
        recorder.start(sessionID: UUID(), configuration: configuration)
        let secondDirectory = try #require(recorder.currentArtifactDirectoryURL)
        await finishProcessing.open()
        await delayedFrame.value
        let newSession = ScanWorkSession()
        newSession.withPublishing {
            publishFrame(client, x: 2)
        }
        recorder.stopAndWait()

        #expect(try messageCount(in: firstDirectory) == 2)
        #expect(try messageCount(in: secondDirectory) == 2)
    }

    @Test("Cancelled frames are not replay-buffered during a connection outage")
    func cancelledFrameCannotEnterReconnectBuffer() async throws {
        let root = FileManager.default.temporaryDirectory
            .appendingPathComponent("ScanWorkBuffer-\(UUID().uuidString)")
        let recorder = LocalROS2BagRecorder(baseDirectoryURL: root)
        let socket = MockROSBridgeSocket()
        let client = ROS2BridgeClient(
            topicRegistry: ROS2TopicRegistry(enabledStreams: [.pointCloud]),
            localBagRecorder: recorder,
            socketFactory: { _ in socket },
            reconnectDelay: 60
        )
        defer { client.disconnect() }
        client.connect(to: "ws://127.0.0.1:9090")
        socket.failReceive(NSError(domain: "test", code: 1))
        for _ in 0..<100 where client.isConnected {
            try await Task.sleep(for: .milliseconds(10))
        }
        try #require(!client.isConnected)
        let oldSession = ScanWorkSession()
        oldSession.cancel()
        oldSession.withPublishing {
            client.publishLiDARPointCloud([ColoredPoint(position: SIMD3<Float>(999, 0, 0))], timestamp: 1)
        }
        let currentSession = ScanWorkSession()
        currentSession.withPublishing {
            client.publishLiDARPointCloud([ColoredPoint(position: SIMD3<Float>(2, 0, 0))], timestamp: 2)
        }
        // Wait for the asynchronous buffer/stats queues instead of assuming a
        // fixed scheduling delay while the rest of the suite is running.
        for _ in 0..<300 where client.localSampleBufferStats.lastBufferedAt == nil {
            try await Task.sleep(for: .milliseconds(10))
        }
        #expect(client.localSampleBufferStats.pointCloudSamples == 1)

        // Restart before a graceful close completes. Previously the newer
        // connection superseded that close and replayed the old scan's buffer.
        client.disconnect(after: 0.25)
        client.connect(to: "ws://127.0.0.1:9090")
        socket.succeedPong()
        for _ in 0..<300 where client.localSampleBufferStats.pointCloudSamples != 0 {
            try await Task.sleep(for: .milliseconds(10))
        }
        #expect(client.localSampleBufferStats.pointCloudSamples == 0)
        #expect(client.localSampleBufferStats.replayedSamples == 0)
    }

    private func publishFrame(_ client: ROS2BridgeClient, x: Float) {
        var transform = matrix_identity_float4x4
        transform.columns.3.x = x
        client.publishPose(transform, timestamp: 1)
        client.publishLiDARPointCloud([ColoredPoint(position: SIMD3<Float>(x, 0, 0))], timestamp: 1)
    }

    private func messageCount(in directory: URL) throws -> Int {
        var database: OpaquePointer?
        let result = sqlite3_open_v2(
            directory.appendingPathComponent("mapeverything_0.db3").path,
            &database, SQLITE_OPEN_READONLY, nil
        )
        defer { sqlite3_close(database) }
        try #require(result == SQLITE_OK)
        var statement: OpaquePointer?
        try #require(sqlite3_prepare_v2(database, "SELECT COUNT(*) FROM messages", -1, &statement, nil) == SQLITE_OK)
        defer { sqlite3_finalize(statement) }
        try #require(sqlite3_step(statement) == SQLITE_ROW)
        return Int(sqlite3_column_int(statement, 0))
    }

    private actor Gate {
        private var isOpen = false
        private var waiters: [CheckedContinuation<Void, Never>] = []

        func wait() async {
            guard !isOpen else { return }
            await withCheckedContinuation { waiters.append($0) }
        }

        func open() {
            isOpen = true
            waiters.forEach { $0.resume() }
            waiters.removeAll()
        }
    }
}
