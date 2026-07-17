//
//  SessionHistoryNotesTests.swift
//  MapEverythingTests
//

import Testing
import Foundation
import SwiftData
@testable import MapEverything

struct SessionHistoryNotesTests {

    @MainActor
    private func makeInMemoryContext() throws -> ModelContext {
        let schema = MapEverythingModelSchema.schema
        let configuration = ModelConfiguration(schema: schema, isStoredInMemoryOnly: true)
        let container = try ModelContainer(for: schema, configurations: [configuration])
        return ModelContext(container)
    }

    private func makeSnapshot(
        sessionID: UUID,
        event: String = "start",
        state: String = "Active",
        lastError: String? = nil
    ) -> MappingSessionSnapshot {
        MappingSessionSnapshot(
            event: event,
            sessionID: sessionID,
            state: state,
            recorderURL: "ws://192.168.1.100:9090",
            enabledStreams: ["camera", "lidar"],
            startedAt: Date(timeIntervalSince1970: 1_000),
            endedAt: nil,
            lastError: lastError
        )
    }

    @Test("Session notes persist through a save and fetch")
    @MainActor
    func testNotesPersistAcrossFetch() throws {
        let context = try makeInMemoryContext()
        let sessionID = UUID()

        let record = MappingSessionModel(sessionID: sessionID)
        #expect(record.notes.isEmpty)
        record.notes = "Scanned the loading dock; rerun with lidar only."
        context.insert(record)
        try context.save()

        let target: UUID? = sessionID
        let descriptor = FetchDescriptor<MappingSessionModel>(
            predicate: #Predicate { $0.sessionID == target }
        )
        let fetched = try context.fetch(descriptor)

        #expect(fetched.count == 1)
        #expect(fetched.first?.notes == "Scanned the loading dock; rerun with lidar only.")
    }

    @Test("History store upsert does not clobber user-authored notes")
    @MainActor
    func testUpsertPreservesNotes() throws {
        let context = try makeInMemoryContext()
        let store = MappingSessionHistoryStore(context: context)
        let sessionID = UUID()

        // Create the record the way production does: through the upsert path.
        store.record(makeSnapshot(sessionID: sessionID), sessionDirectoryPath: "/tmp/bag-1")

        let target: UUID? = sessionID
        let descriptor = FetchDescriptor<MappingSessionModel>(
            predicate: #Predicate { $0.sessionID == target }
        )
        let created = try #require(try context.fetch(descriptor).first)
        #expect(created.notes.isEmpty)

        created.notes = "Do not delete; shared with the mapping team."
        try context.save()

        // A later snapshot for the same session updates via update(from:).
        store.record(
            makeSnapshot(
                sessionID: sessionID,
                event: "stop",
                state: "Idle",
                lastError: "recorder unreachable"
            ),
            metadataJSON: #"{"event":"stop"}"#
        )

        let updated = try #require(try context.fetch(descriptor).first)
        #expect(updated.state == "Idle")
        #expect(updated.lastError == "recorder unreachable")
        #expect(updated.metadataJSON == #"{"event":"stop"}"#)
        #expect(updated.notes == "Do not delete; shared with the mapping team.")
    }

    @Test("Diagnostics pruning keeps the most recent files per payload kind")
    func testStaleFileURLsKeepsMostRecentPerKind() {
        func url(day: Int, kind: String) -> URL {
            URL(fileURLWithPath: String(
                format: "/tmp/Diagnostics/2026-07-%02dT00:00:00.000Z-%@.json", day, kind
            ))
        }

        let diagnostics = (1...25).map { url(day: $0, kind: "diagnostic") }
        let metrics = (1...5).map { url(day: $0, kind: "metric") }
        let unrelated = [URL(fileURLWithPath: "/tmp/Diagnostics/readme.json")]

        let stale = MetricKitDiagnostics.staleFileURLs(
            in: (diagnostics + metrics + unrelated).shuffled()
        )

        // Only the 5 oldest diagnostics fall out; metrics are under the limit
        // and files that match neither kind are never deleted.
        #expect(Set(stale) == Set(diagnostics.prefix(5)))
    }

    @Test("Diagnostics pruning honors an explicit retention limit")
    func testStaleFileURLsExplicitLimit() {
        func url(day: Int, kind: String) -> URL {
            URL(fileURLWithPath: String(
                format: "/tmp/Diagnostics/2026-07-%02dT00:00:00.000Z-%@.json", day, kind
            ))
        }

        let diagnostics = (1...4).map { url(day: $0, kind: "diagnostic") }
        let metrics = (1...3).map { url(day: $0, kind: "metric") }

        let stale = MetricKitDiagnostics.staleFileURLs(
            in: (diagnostics + metrics).shuffled(),
            keepingMostRecent: 2
        )

        #expect(Set(stale) == Set(diagnostics.prefix(2) + metrics.prefix(1)))
        #expect(MetricKitDiagnostics.staleFileURLs(in: [], keepingMostRecent: 2).isEmpty)
    }
}
