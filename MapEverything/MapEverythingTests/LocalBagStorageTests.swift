//
//  LocalBagStorageTests.swift
//  MapEverythingTests
//

import Testing
import Foundation
import simd
import SQLite3
@testable import MapEverything

struct LocalBagStorageTests {

    @Test("LAS writer emits a LAS 1.2 format-2 file that round-trips points")
    func testLASWriterHeaderAndPointRoundtrip() throws {
        let inputPoints = [
            ColoredPoint(position: SIMD3<Float>(1.25, -2.5, 3.75), color: SIMD3<UInt8>(10, 20, 30)),
            ColoredPoint(position: SIMD3<Float>(-4.5, 6.0, -7.25), color: SIMD3<UInt8>(255, 0, 128)),
            ColoredPoint(position: SIMD3<Float>(.nan, 0, .infinity), color: SIMD3<UInt8>(1, 2, 3))
        ]

        let data = try #require(LASPointCloudWriter.lasData(points: inputPoints, sourceID: 7))

        // Public header block.
        #expect(String(data: data.prefix(4), encoding: .ascii) == "LASF")
        #expect(data[24] == 1)                              // version major
        #expect(data[25] == 2)                              // version minor
        #expect(uint16(from: data, at: 94) == 227)          // header size
        #expect(uint32(from: data, at: 96) == 227)          // offset to point data
        #expect(data[104] == 2)                             // point data record format
        #expect(uint16(from: data, at: 105) == 26)          // point record length
        #expect(uint32(from: data, at: 107) == 2)           // non-finite point skipped
        #expect(uint32(from: data, at: 111) == 2)           // points-by-return first entry
        #expect((115..<131).allSatisfy { data[$0] == 0 })   // remaining by-return entries
        #expect(data.count == 227 + 2 * 26)

        let scale = SIMD3<Double>(
            float64(from: data, at: 131),
            float64(from: data, at: 139),
            float64(from: data, at: 147)
        )
        let offset = SIMD3<Double>(
            float64(from: data, at: 155),
            float64(from: data, at: 163),
            float64(from: data, at: 171)
        )
        #expect(scale.x > 0 && scale.y > 0 && scale.z > 0)
        #expect(abs(offset.x - -4.5) < 0.0001)              // offsets are the min bounds
        #expect(abs(offset.y - -2.5) < 0.0001)
        #expect(abs(offset.z - -7.25) < 0.0001)
        #expect(abs(float64(from: data, at: 179) - 1.25) < 0.0001)   // max X
        #expect(abs(float64(from: data, at: 187) - -4.5) < 0.0001)   // min X
        #expect(abs(float64(from: data, at: 211) - 3.75) < 0.0001)   // max Z
        #expect(abs(float64(from: data, at: 219) - -7.25) < 0.0001)  // min Z

        // First point record decodes back through scale/offset.
        let record = 227
        let x = Double(int32(from: data, at: record)) * scale.x + offset.x
        let y = Double(int32(from: data, at: record + 4)) * scale.y + offset.y
        let z = Double(int32(from: data, at: record + 8)) * scale.z + offset.z
        #expect(abs(x - 1.25) < 0.001)
        #expect(abs(y - -2.5) < 0.001)
        #expect(abs(z - 3.75) < 0.001)
        #expect(uint16(from: data, at: record + 12) == 0)   // intensity
        #expect(data[record + 14] == 0b0000_1001)           // return 1 of 1
        #expect(data[record + 15] == 0)                     // classification
        #expect(uint16(from: data, at: record + 18) == 7)   // point source ID
        #expect(uint16(from: data, at: record + 20) == UInt16(10) * 257)
        #expect(uint16(from: data, at: record + 22) == UInt16(20) * 257)
        #expect(uint16(from: data, at: record + 24) == UInt16(30) * 257)

        // Second record round-trips RGB including channel extremes.
        let second = record + 26
        #expect(uint16(from: data, at: second + 20) == 65535)
        #expect(uint16(from: data, at: second + 22) == 0)
        #expect(uint16(from: data, at: second + 24) == UInt16(128) * 257)

        #expect(LASPointCloudWriter.lasData(points: []) == nil)
        #expect(LASPointCloudWriter.lasData(points: [
            ColoredPoint(position: SIMD3<Float>(.nan, .nan, .nan))
        ]) == nil)
    }

    @Test("Local ROS2 bag recorder writes a LAS artifact next to the PLY")
    func testLocalROS2BagRecorderWritesLASArtifact() throws {
        let fileManager = FileManager.default
        let rootURL = fileManager.temporaryDirectory
            .appendingPathComponent("MapEverythingLocalBagLAS-\(UUID().uuidString)", isDirectory: true)
        try fileManager.createDirectory(at: rootURL, withIntermediateDirectories: true)
        defer { try? fileManager.removeItem(at: rootURL) }

        let recorder = LocalROS2BagRecorder(fileManager: fileManager, baseDirectoryURL: rootURL)
        recorder.start(
            sessionID: UUID(),
            configuration: LocalROS2BagRecorderConfiguration(isEnabled: true, maxChunkBytes: 8 * 1_048_576)
        )
        let targetDirectoryURL = try #require(recorder.currentArtifactDirectoryURL)
        recorder.stopAndWait()

        recorder.recordFinalPointCloud(
            LocalPointCloudArtifact(
                source: "unit_test_points",
                coordinateFrame: "map",
                capturedAt: Date(timeIntervalSince1970: 1_700_000_051),
                points: [
                    LocalPointCloudArtifact.Point(
                        position: SIMD3<Float>(0.25, 0.5, 0.75),
                        color: SIMD3<UInt8>(10, 20, 30)
                    ),
                    LocalPointCloudArtifact.Point(
                        position: SIMD3<Float>(-1.0, 2.0, -3.0),
                        color: SIMD3<UInt8>(40, 50, 60)
                    )
                ],
                metadata: [:]
            ),
            in: targetDirectoryURL
        )
        recorder.flushAndWait()

        let lasURL = targetDirectoryURL.appendingPathComponent(LocalPointCloudArtifact.lasFileName)
        let lasData = try Data(contentsOf: lasURL)
        #expect(String(data: lasData.prefix(4), encoding: .ascii) == "LASF")
        #expect(uint32(from: lasData, at: 107) == 2)
        #expect(lasData.count == 227 + 2 * 26)

        let session = try #require(try recorder.listBagSessions().first)
        #expect(session.files.contains { $0.name == LocalPointCloudArtifact.plyFileName && $0.kind == .pointCloud })
        #expect(session.files.contains { $0.name == LocalPointCloudArtifact.lasFileName && $0.kind == .pointCloud })
    }

    @Test("Disk-space guard leaves a normal recording session intact")
    func testDiskSpaceGuardAllowsNormalRecording() throws {
        let fileManager = FileManager.default
        let rootURL = fileManager.temporaryDirectory
            .appendingPathComponent("MapEverythingLocalBagDiskGuard-\(UUID().uuidString)", isDirectory: true)
        try fileManager.createDirectory(at: rootURL, withIntermediateDirectories: true)
        defer { try? fileManager.removeItem(at: rootURL) }

        let recorder = LocalROS2BagRecorder(fileManager: fileManager, baseDirectoryURL: rootURL)
        recorder.start(
            sessionID: UUID(),
            configuration: LocalROS2BagRecorderConfiguration(isEnabled: true, maxChunkBytes: 8 * 1_048_576)
        )
        #expect(recorder.isAcceptingRecords)

        // 64 successful flushes cross the periodic free-space re-check; with a
        // healthy disk the recorder must keep accepting records afterwards.
        for sequence in 1...64 {
            recorder.recordPublishedTopic(
                topic: "/mapping/status",
                messageType: "diagnostic_msgs/msg/DiagnosticArray",
                msg: [
                    "header": [
                        "stamp": ["sec": 1_700_000_000 + sequence, "nanosec": sequence],
                        "frame_id": "map"
                    ],
                    "status": []
                ]
            )
            recorder.flushAndWait()
        }
        #expect(recorder.isAcceptingRecords)

        recorder.stopAndWait()

        let bagDirectory = try #require(
            try fileManager.contentsOfDirectory(
                at: rootURL,
                includingPropertiesForKeys: [.isDirectoryKey]
            ).first { url in
                (try? url.resourceValues(forKeys: [.isDirectoryKey]).isDirectory) == true
            }
        )
        let chunkURL = bagDirectory.appendingPathComponent("mapeverything_0.db3")
        #expect(try sqliteInteger(url: chunkURL, sql: "SELECT COUNT(*) FROM messages") == 64)

        let metadata = try String(
            contentsOf: bagDirectory.appendingPathComponent("metadata.yaml"),
            encoding: .utf8
        )
        #expect(metadata.contains("message_count: 64"))
    }

    private func uint16(from data: Data, at offset: Int) -> UInt16 {
        guard offset + 1 < data.count else { return 0 }
        return UInt16(data[offset]) | (UInt16(data[offset + 1]) << 8)
    }

    private func uint32(from data: Data, at offset: Int) -> UInt32 {
        guard offset + 3 < data.count else { return 0 }
        return UInt32(data[offset])
            | (UInt32(data[offset + 1]) << 8)
            | (UInt32(data[offset + 2]) << 16)
            | (UInt32(data[offset + 3]) << 24)
    }

    private func int32(from data: Data, at offset: Int) -> Int32 {
        Int32(bitPattern: uint32(from: data, at: offset))
    }

    private func float64(from data: Data, at offset: Int) -> Double {
        guard offset + 7 < data.count else { return 0 }
        var bitPattern: UInt64 = 0
        for index in (0..<8).reversed() {
            bitPattern = (bitPattern << 8) | UInt64(data[offset + index])
        }
        return Double(bitPattern: bitPattern)
    }

    private func sqliteInteger(url: URL, sql: String) throws -> Int {
        var database: OpaquePointer?
        guard sqlite3_open_v2(url.path, &database, SQLITE_OPEN_READONLY, nil) == SQLITE_OK else {
            sqlite3_close(database)
            throw LocalBagStorageTestError.sqlite("Unable to open SQLite database")
        }
        defer { sqlite3_close(database) }

        var statement: OpaquePointer?
        guard sqlite3_prepare_v2(database, sql, -1, &statement, nil) == SQLITE_OK else {
            throw LocalBagStorageTestError.sqlite("Unable to prepare SQLite statement")
        }
        defer { sqlite3_finalize(statement) }

        guard sqlite3_step(statement) == SQLITE_ROW else {
            throw LocalBagStorageTestError.sqlite("SQLite query returned no rows")
        }

        return Int(sqlite3_column_int64(statement, 0))
    }

    private enum LocalBagStorageTestError: Error {
        case sqlite(String)
    }
}
