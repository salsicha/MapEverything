//
//  ColoredMeshPublishTests.swift
//  MapEverythingTests
//

import Testing
import Foundation
import SQLite3
import simd
import RealityKit
@testable import MapEverything

struct ColoredMeshPublishTests {
    private func makeQuadSnapshot() -> MeshGenerator.DepthAnythingMeshSnapshot {
        let vertices = [
            SIMD3<Float>(0, 0, 0),
            SIMD3<Float>(1, 0, 0),
            SIMD3<Float>(1, 1, 0),
            SIMD3<Float>(0, 1, 0)
        ]
        let colors = [
            SIMD3<UInt8>(255, 0, 0),
            SIMD3<UInt8>(0, 255, 0),
            SIMD3<UInt8>(0, 0, 255),
            SIMD3<UInt8>(128, 64, 32)
        ]
        var descriptor = MeshDescriptor()
        descriptor.positions = MeshBuffers.Positions(vertices)
        descriptor.primitives = .triangles([0, 1, 2, 0, 2, 3])
        return MeshGenerator.DepthAnythingMeshSnapshot(
            descriptor: descriptor,
            vertices: vertices,
            indices: [0, 1, 2, 0, 2, 3],
            colors: colors
        )
    }

    /// A grid mesh large enough that its expanded marker JSON far exceeds a
    /// small payload budget, forcing the fitting loop to trim.
    private func makeGridSnapshot(gridSize: Int) -> MeshGenerator.DepthAnythingMeshSnapshot {
        var vertices: [SIMD3<Float>] = []
        var colors: [SIMD3<UInt8>] = []
        for row in 0..<gridSize {
            for col in 0..<gridSize {
                vertices.append(SIMD3<Float>(Float(col) * 0.05, Float(row) * 0.05, -1.0))
                colors.append(SIMD3<UInt8>(UInt8((row * 7) % 256), UInt8((col * 7) % 256), 99))
            }
        }
        var indices: [UInt32] = []
        for row in 0..<(gridSize - 1) {
            for col in 0..<(gridSize - 1) {
                let upperLeft = UInt32(row * gridSize + col)
                let upperRight = upperLeft + 1
                let lowerLeft = upperLeft + UInt32(gridSize)
                indices.append(contentsOf: [upperLeft, lowerLeft, upperRight])
                indices.append(contentsOf: [upperRight, lowerLeft, lowerLeft + 1])
            }
        }
        var descriptor = MeshDescriptor()
        descriptor.positions = MeshBuffers.Positions(vertices)
        descriptor.primitives = .triangles(indices)
        return MeshGenerator.DepthAnythingMeshSnapshot(
            descriptor: descriptor,
            vertices: vertices,
            indices: indices,
            colors: colors
        )
    }

    /// Raw sizes of the recorded rosbridge blobs for a topic — the exact
    /// bytes the payload fitter is supposed to bound.
    private func recordedPayloadSizes(chunkURL: URL, topic: String) throws -> [Int] {
        var database: OpaquePointer?
        guard sqlite3_open_v2(chunkURL.path, &database, SQLITE_OPEN_READONLY, nil) == SQLITE_OK else {
            return []
        }
        defer { sqlite3_close(database) }

        let sql = """
        SELECT length(messages.data) FROM messages
        JOIN topics ON topics.id = messages.topic_id
        WHERE topics.name = ?
        ORDER BY messages.timestamp ASC
        """
        var statement: OpaquePointer?
        guard sqlite3_prepare_v2(database, sql, -1, &statement, nil) == SQLITE_OK else { return [] }
        defer { sqlite3_finalize(statement) }
        let transient = unsafeBitCast(-1, to: sqlite3_destructor_type.self)
        sqlite3_bind_text(statement, 1, topic, -1, transient)

        var sizes: [Int] = []
        while sqlite3_step(statement) == SQLITE_ROW {
            sizes.append(Int(sqlite3_column_int64(statement, 0)))
        }
        return sizes
    }

    /// Reads every recorded rosbridge publish payload for a topic out of the
    /// bag chunk (the recorder stores the full {op, topic, msg} JSON blob).
    private func recordedMessages(chunkURL: URL, topic: String) throws -> [[String: Any]] {
        var database: OpaquePointer?
        guard sqlite3_open_v2(chunkURL.path, &database, SQLITE_OPEN_READONLY, nil) == SQLITE_OK else {
            return []
        }
        defer { sqlite3_close(database) }

        let sql = """
        SELECT messages.data FROM messages
        JOIN topics ON topics.id = messages.topic_id
        WHERE topics.name = ?
        ORDER BY messages.timestamp ASC
        """
        var statement: OpaquePointer?
        guard sqlite3_prepare_v2(database, sql, -1, &statement, nil) == SQLITE_OK else { return [] }
        defer { sqlite3_finalize(statement) }
        let transient = unsafeBitCast(-1, to: sqlite3_destructor_type.self)
        sqlite3_bind_text(statement, 1, topic, -1, transient)

        var payloads: [[String: Any]] = []
        while sqlite3_step(statement) == SQLITE_ROW {
            guard let blob = sqlite3_column_blob(statement, 0) else { continue }
            let byteCount = Int(sqlite3_column_bytes(statement, 0))
            let data = Data(bytes: blob, count: byteCount)
            if let object = try? JSONSerialization.jsonObject(with: data) as? [String: Any] {
                payloads.append(object)
            }
        }
        return payloads
    }

    private func makeRecordingBridge() throws -> (ROS2BridgeClient, LocalROS2BagRecorder, URL) {
        let rootURL = FileManager.default.temporaryDirectory
            .appendingPathComponent("ColoredMeshPublish-\(UUID().uuidString)", isDirectory: true)
        try FileManager.default.createDirectory(at: rootURL, withIntermediateDirectories: true)

        let recorder = LocalROS2BagRecorder(fileManager: .default, baseDirectoryURL: rootURL)
        recorder.start(
            sessionID: UUID(),
            configuration: LocalROS2BagRecorderConfiguration(isEnabled: true, maxChunkBytes: 8 * 1_048_576)
        )
        let registry = ROS2TopicRegistry(enabledStreams: [.mesh])
        let bridge = ROS2BridgeClient(topicRegistry: registry, localBagRecorder: recorder)
        return (bridge, recorder, rootURL)
    }

    private func chunkURL(in rootURL: URL) throws -> URL {
        let bagDirectory = try #require(
            try FileManager.default.contentsOfDirectory(
                at: rootURL,
                includingPropertiesForKeys: [.isDirectoryKey]
            ).first { url in
                (try? url.resourceValues(forKeys: [.isDirectoryKey]).isDirectory) == true
            }
        )
        return bagDirectory.appendingPathComponent("mapeverything_0.db3")
    }

    @Test("Publishing the colored DA mesh records a colored marker and snapshot")
    func testColoredMeshRecordsMarkerAndSnapshot() throws {
        let (bridge, recorder, rootURL) = try makeRecordingBridge()
        defer { try? FileManager.default.removeItem(at: rootURL) }

        bridge.publishDepthAnythingMesh(makeQuadSnapshot(), timestamp: 42.0)
        recorder.flushAndWait()
        recorder.stopAndWait()

        let chunk = try chunkURL(in: rootURL)

        // Marker: one TRIANGLE_LIST with per-point colors matching vertex RGB.
        let markerPayloads = try recordedMessages(chunkURL: chunk, topic: "/mapping/map")
        let markerPayload = try #require(markerPayloads.first)
        let markerMsg = try #require(markerPayload["msg"] as? [String: Any])
        let markers = try #require(markerMsg["markers"] as? [[String: Any]])
        let marker = try #require(markers.first)
        #expect(marker["ns"] as? String == "depth_mesh")
        #expect(marker["id"] as? Int == 0)
        #expect(marker["type"] as? Int == 11)
        let points = try #require(marker["points"] as? [[String: Any]])
        let markerColors = try #require(marker["colors"] as? [[String: Any]])
        #expect(points.count == markerColors.count)
        #expect(points.count == 6) // two triangles

        // First vertex of the first triangle is index 0 -> (255,0,0).
        let firstColor = markerColors[0]
        #expect(abs((firstColor["r"] as? Double ?? 0) - 1.0) < 1e-6)
        #expect(abs((firstColor["g"] as? Double ?? 1) - 0.0) < 1e-6)
        #expect(abs((firstColor["b"] as? Double ?? 1) - 0.0) < 1e-6)
        #expect(abs((firstColor["a"] as? Double ?? 0) - 1.0) < 1e-6)

        // Snapshot: schema v3 colored encoding.
        let snapshotPayloads = try recordedMessages(chunkURL: chunk, topic: "/mapping/mesh_snapshot")
        let snapshotPayload = try #require(snapshotPayloads.first)
        let snapshotMsg = try #require(snapshotPayload["msg"] as? [String: Any])
        #expect(snapshotMsg["schema_version"] as? Int == 3)
        #expect(snapshotMsg["vertex_encoding"] as? String == "float32_xyz_rgb8_le_base64")
        #expect(snapshotMsg["vertex_stride_bytes"] as? Int == 15)
        #expect((snapshotMsg["metadata_json"] as? String)?.contains("has_vertex_colors") ?? false)
    }

    @Test("clearDepthMeshMarker records a DELETE for the overlay marker")
    func testClearDepthMeshMarkerRecordsDelete() throws {
        let (bridge, recorder, rootURL) = try makeRecordingBridge()
        defer { try? FileManager.default.removeItem(at: rootURL) }

        bridge.clearDepthMeshMarker(timestamp: 5.0)
        recorder.flushAndWait()
        recorder.stopAndWait()

        let chunk = try chunkURL(in: rootURL)
        let markerPayloads = try recordedMessages(chunkURL: chunk, topic: "/mapping/map")
        let markerMsg = try #require(markerPayloads.first?["msg"] as? [String: Any])
        let marker = try #require((markerMsg["markers"] as? [[String: Any]])?.first)
        #expect(marker["ns"] as? String == "depth_mesh")
        #expect(marker["id"] as? Int == 0)
        #expect(marker["action"] as? Int == 2) // DELETE
    }

    @Test("Fitted marker and snapshot never exceed the payload budget with real headers")
    func testFittedPayloadsRespectBudgetWithRealHeaders() throws {
        // Regression guard for the placeholder-header fitting bug: candidates
        // must be measured with the same epoch-digit header the published
        // marker carries, or fitted payloads land just over the cap.
        let rootURL = FileManager.default.temporaryDirectory
            .appendingPathComponent("ColoredMeshBudget-\(UUID().uuidString)", isDirectory: true)
        try FileManager.default.createDirectory(at: rootURL, withIntermediateDirectories: true)
        defer { try? FileManager.default.removeItem(at: rootURL) }

        let recorder = LocalROS2BagRecorder(fileManager: .default, baseDirectoryURL: rootURL)
        recorder.start(
            sessionID: UUID(),
            configuration: LocalROS2BagRecorderConfiguration(isEnabled: true, maxChunkBytes: 8 * 1_048_576)
        )
        let maxPayloadBytes = 8_000
        let bridge = ROS2BridgeClient(
            topicRegistry: ROS2TopicRegistry(enabledStreams: [.mesh]),
            localBagRecorder: recorder,
            meshSnapshotConfiguration: MeshSnapshotPublishConfiguration(
                publishInterval: 2.0,
                maxPayloadBytes: maxPayloadBytes,
                maxTrianglePoints: 12_000
            )
        )

        // 10x10 grid -> 162 triangles -> ~486 marker points with colors,
        // roughly 60 KB of JSON before fitting.
        bridge.publishDepthAnythingMesh(makeGridSnapshot(gridSize: 10), timestamp: 42.0)
        recorder.flushAndWait()
        recorder.stopAndWait()

        let chunk = try chunkURL(in: rootURL)
        let markerSizes = try recordedPayloadSizes(chunkURL: chunk, topic: "/mapping/map")
        let snapshotSizes = try recordedPayloadSizes(chunkURL: chunk, topic: "/mapping/mesh_snapshot")

        // Both messages must have been published (fitting trims, not drops)...
        #expect(markerSizes.count == 1)
        #expect(snapshotSizes.count == 1)
        // ...and every recorded rosbridge blob must fit the budget exactly as
        // the fitter measured it.
        for size in markerSizes + snapshotSizes {
            #expect(size <= maxPayloadBytes)
            #expect(size > 0)
        }

        // The fitted marker must still be triangle-aligned with colors in
        // lockstep.
        let markerMsg = try #require(
            try recordedMessages(chunkURL: chunk, topic: "/mapping/map").first?["msg"] as? [String: Any]
        )
        let marker = try #require((markerMsg["markers"] as? [[String: Any]])?.first)
        let points = try #require(marker["points"] as? [[String: Any]])
        let markerColors = try #require(marker["colors"] as? [[String: Any]])
        #expect(points.count % 3 == 0)
        #expect(points.count > 0)
        #expect(points.count == markerColors.count)
    }

    private func publishedMarkerSize(
        snapshot: MeshGenerator.DepthAnythingMeshSnapshot,
        budget: Int
    ) throws -> (markerSizes: [Int], snapshotSizes: [Int]) {
        let rootURL = FileManager.default.temporaryDirectory
            .appendingPathComponent("MeshBudgetSweep-\(UUID().uuidString)", isDirectory: true)
        try FileManager.default.createDirectory(at: rootURL, withIntermediateDirectories: true)
        defer { try? FileManager.default.removeItem(at: rootURL) }

        let recorder = LocalROS2BagRecorder(fileManager: .default, baseDirectoryURL: rootURL)
        recorder.start(
            sessionID: UUID(),
            configuration: LocalROS2BagRecorderConfiguration(isEnabled: true, maxChunkBytes: 8 * 1_048_576)
        )
        let bridge = ROS2BridgeClient(
            topicRegistry: ROS2TopicRegistry(enabledStreams: [.mesh]),
            localBagRecorder: recorder,
            meshSnapshotConfiguration: MeshSnapshotPublishConfiguration(
                publishInterval: 2.0,
                maxPayloadBytes: budget,
                maxTrianglePoints: 12_000
            )
        )
        bridge.publishDepthAnythingMesh(snapshot, timestamp: 42.0)
        recorder.flushAndWait()
        recorder.stopAndWait()

        let chunk = try chunkURL(in: rootURL)
        return (
            try recordedPayloadSizes(chunkURL: chunk, topic: "/mapping/map"),
            try recordedPayloadSizes(chunkURL: chunk, topic: "/mapping/mesh_snapshot")
        )
    }

    @Test("Marker fitting honors budgets bracketing the untrimmed marker size")
    func testMarkerBudgetSweep() throws {
        // Deterministic regression pin for candidate/final byte mismatches in
        // the marker fitter (e.g. measuring candidates with a placeholder
        // header). The violation window for such bugs sits at budgets just
        // above the size the fitter MEASURES for the untrimmed marker: the
        // loop accepts it unchanged, then publishes something larger. So:
        // measure the real untrimmed size with a huge budget, then sweep
        // budgets bracketing it at finer granularity than any realistic
        // header/placeholder delta. Any mismatch makes a bracketed publish
        // exceed its budget; the correct implementation trims instead.
        let snapshot = makeGridSnapshot(gridSize: 10)
        let untrimmed = try publishedMarkerSize(snapshot: snapshot, budget: 10_000_000)
        let untrimmedSize = try #require(untrimmed.markerSizes.first)
        #expect(untrimmedSize > 10_000) // fixture sanity: real content

        for budget in stride(from: untrimmedSize - 40, through: untrimmedSize + 8, by: 4) {
            let sizes = try publishedMarkerSize(snapshot: snapshot, budget: budget)
            for size in sizes.markerSizes + sizes.snapshotSizes {
                #expect(size <= budget, "recorded payload \(size) exceeds budget \(budget)")
            }
            #expect(sizes.markerSizes.count == 1)
        }
    }

    @Test("Uncolored snapshot falls back to the legacy vertex encoding")
    func testUncoloredSnapshotUsesLegacyEncoding() throws {
        let (bridge, recorder, rootURL) = try makeRecordingBridge()
        defer { try? FileManager.default.removeItem(at: rootURL) }

        let quad = makeQuadSnapshot()
        let uncolored = MeshGenerator.DepthAnythingMeshSnapshot(
            descriptor: quad.descriptor,
            vertices: quad.vertices,
            indices: quad.indices,
            colors: []
        )
        bridge.publishDepthAnythingMesh(uncolored, timestamp: 7.0)
        recorder.flushAndWait()
        recorder.stopAndWait()

        let chunk = try chunkURL(in: rootURL)
        let snapshotMsg = try #require(
            try recordedMessages(chunkURL: chunk, topic: "/mapping/mesh_snapshot").first?["msg"] as? [String: Any]
        )
        #expect(snapshotMsg["vertex_encoding"] as? String == "float32_xyz_le_base64")
        #expect(snapshotMsg["vertex_stride_bytes"] as? Int == 12)

        let markerMsg = try #require(
            try recordedMessages(chunkURL: chunk, topic: "/mapping/map").first?["msg"] as? [String: Any]
        )
        let marker = try #require((markerMsg["markers"] as? [[String: Any]])?.first)
        // No per-point colors when the mesh is uncolored.
        #expect(marker["colors"] == nil)
    }
}
