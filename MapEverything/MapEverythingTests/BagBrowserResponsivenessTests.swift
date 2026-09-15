import Foundation
import Testing
@testable import MapEverything

// Simulate a long export without allocating a million-vertex mesh. The timeout
// lets the old blocking implementation fail an assertion instead of hanging CI.
// It must be far larger than any continuous main-actor occupancy a parallel
// run can produce: probes queued on the main actor compete with every other
// @MainActor test (including CPU-bound sweeps), so a responsive
// implementation may wait many seconds for one free slot. A blocking
// regression holds its caller for the whole window and fails
// deterministically after it.
nonisolated private final class BagWriterStall: @unchecked Sendable {
    private let lock = NSLock()
    private let releaseSignal = DispatchSemaphore(value: 0)
    private var finished = false
    var hasFinished: Bool { lock.withLock { finished } }

    init(queue: DispatchQueue) {
        let entered = DispatchSemaphore(value: 0)
        queue.async {
            entered.signal()
            _ = self.releaseSignal.wait(timeout: .now() + 120)
            self.lock.withLock { self.finished = true }
        }
        entered.wait()
    }

    func release() { releaseSignal.signal() }
}

struct BagBrowserResponsivenessTests {
    @Test("Reading the export destination never waits for a busy writer")
    @MainActor
    func destinationRemainsResponsive() throws {
        let root = FileManager.default.temporaryDirectory.appendingPathComponent(UUID().uuidString)
        let writer = DispatchQueue(label: "test.bag.destination")
        let recorder = LocalROS2BagRecorder(baseDirectoryURL: root, recordingQueue: writer)
        recorder.start(sessionID: UUID(), configuration: .init(isEnabled: true, maxChunkBytes: 8_388_608))
        let expected = try #require(recorder.currentArtifactDirectoryURL)
        let stall = BagWriterStall(queue: writer)
        defer {
            stall.release()
            recorder.stopAndWait()
            try? FileManager.default.removeItem(at: root)
        }
        #expect(recorder.currentArtifactDirectoryURL == expected)
        #expect(!stall.hasFinished, "Reading the destination waited for the export to finish")
    }

    @Test("The UI can handle another event while a preview waits for the writer")
    @MainActor
    func previewDoesNotBlockMainActor() async throws {
        let root = FileManager.default.temporaryDirectory.appendingPathComponent(UUID().uuidString)
        let writer = DispatchQueue(label: "test.bag.preview.writer")
        let recorder = LocalROS2BagRecorder(baseDirectoryURL: root, recordingQueue: writer)
        recorder.start(sessionID: UUID(), configuration: .init(isEnabled: true, maxChunkBytes: 8_388_608))
        recorder.stopAndWait()
        let session = try #require(recorder.listBagSessions().first)
        let stall = BagWriterStall(queue: writer)
        defer {
            stall.release()
            recorder.stopAndWait()
            try? FileManager.default.removeItem(at: root)
        }

        // Queue main-actor work immediately before entering the async API: it
        // can only run if that API actually suspends the caller instead of
        // holding the main thread. A Task (not DispatchQueue.main.async) so a
        // failure attributes to this test rather than recording as an
        // unassociated issue.
        let scan = Task { @MainActor in
            _ = Task { @MainActor in
                #expect(!stall.hasFinished, "The preview blocked UI events behind the writer")
                stall.release()
            }
            return try await recorder.bagSessionWithPreviewScan(session)
        }
        let loaded = try await scan.value
        #expect(loaded.id == session.id)
    }

    @Test("Cached bag listing is independent of slow thumbnail scans")
    func listingDoesNotQueueBehindPreview() async throws {
        let root = FileManager.default.temporaryDirectory.appendingPathComponent(UUID().uuidString)
        let previewQueue = DispatchQueue(label: "test.bag.preview.scanner")
        let recorder = LocalROS2BagRecorder(baseDirectoryURL: root, previewQueue: previewQueue)
        recorder.start(sessionID: UUID(), configuration: .init(isEnabled: true, maxChunkBytes: 8_388_608))
        recorder.stopAndWait()
        let stall = BagWriterStall(queue: previewQueue)
        defer {
            stall.release()
            try? FileManager.default.removeItem(at: root)
        }
        let sessions = try await recorder.listBagSessionsAsync()
        #expect(sessions.count == 1)
        #expect(!stall.hasFinished, "The initial list waited behind thumbnail generation")
    }

    @Test("Asynchronous stop and final exports notify the open bag browser")
    @MainActor
    func finalizationRefreshesLibrary() async throws {
        let root = FileManager.default.temporaryDirectory.appendingPathComponent(UUID().uuidString)
        let writer = DispatchQueue(label: "test.bag.finalization")
        let recorder = LocalROS2BagRecorder(baseDirectoryURL: root, recordingQueue: writer)
        defer {
            recorder.stopAndWait()
            try? FileManager.default.removeItem(at: root)
        }
        recorder.start(sessionID: UUID(), configuration: .init(isEnabled: true, maxChunkBytes: 8_388_608))
        let active = try #require(try await recorder.listBagSessionsAsync().first)
        #expect(try await recorder.bagSessionWithPreviewScan(active).preview == nil)
        let beforeStop = recorder.libraryRevision
        recorder.stop()
        await withCheckedContinuation { continuation in writer.async { continuation.resume() } }
        #expect(recorder.libraryRevision > beforeStop)
        let stopped = try await recorder.bagSessionWithPreviewScan(active)
        #expect(stopped.preview != nil)

        let beforeExport = recorder.libraryRevision
        recorder.recordFinalOverlayMesh(LocalOverlayMeshArtifact(
            source: "test", coordinateFrame: "map", capturedAt: Date(),
            vertices: [.init(0, 0, 0), .init(1, 0, 0), .init(0, 1, 0)],
            indices: [0, 1, 2], metadata: [:]
        ), in: active.directoryURL)
        await withCheckedContinuation { continuation in writer.async { continuation.resume() } }
        #expect(recorder.libraryRevision > beforeExport)
        let refreshed = try #require(try await recorder.listBagSessionsAsync().first)
        #expect(refreshed.files.contains { $0.kind == .overlayMesh })
        #expect(refreshed.preview == nil, "The earlier file listing must not keep a stale preview cache")
        try await recorder.deleteBagSessionAsync(refreshed)
        #expect(try await recorder.listBagSessionsAsync().isEmpty)
    }

    @Test("Deleting one bag preserves the other saved bags")
    func deletingOneBagPreservesTheOthers() async throws {
        let root = FileManager.default.temporaryDirectory.appendingPathComponent(UUID().uuidString)
        let recorder = LocalROS2BagRecorder(baseDirectoryURL: root)
        defer { try? FileManager.default.removeItem(at: root) }

        recorder.start(sessionID: UUID(), configuration: .init(isEnabled: true, maxChunkBytes: 8_388_608))
        recorder.stopAndWait()
        recorder.start(sessionID: UUID(), configuration: .init(isEnabled: true, maxChunkBytes: 8_388_608))
        recorder.stopAndWait()

        let sessions = try await recorder.listBagSessionsAsync()
        #expect(sessions.count == 2)

        let deletedSession = try #require(sessions.first)
        try await recorder.deleteBagSessionAsync(deletedSession)

        let remainingSessions = try await recorder.listBagSessionsAsync()
        #expect(remainingSessions.count == 1)
        #expect(remainingSessions.first?.id != deletedSession.id)
    }
}
