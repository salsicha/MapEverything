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
