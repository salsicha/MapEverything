//
//  ColoredMeshSchemaTests.swift
//  MapEverythingTests
//

import Testing
import Foundation
import simd
@testable import MapEverything

struct ColoredMeshSchemaTests {
    private let header: [String: Any] = [
        "stamp": ["sec": 1, "nanosec": 2],
        "frame_id": "map"
    ]

    private func floatLE(_ data: Data, _ offset: Int) -> Float {
        let bits = UInt32(data[offset])
            | (UInt32(data[offset + 1]) << 8)
            | (UInt32(data[offset + 2]) << 16)
            | (UInt32(data[offset + 3]) << 24)
        return Float(bitPattern: bits)
    }

    private func uint32LE(_ data: Data, _ offset: Int) -> UInt32 {
        UInt32(data[offset])
            | (UInt32(data[offset + 1]) << 8)
            | (UInt32(data[offset + 2]) << 16)
            | (UInt32(data[offset + 3]) << 24)
    }

    @Test("Colored grid mesh encodes 15-byte xyz+rgb vertices that decode back")
    func testColoredVertexRoundtrip() throws {
        // A single quad: two triangles, four vertices.
        let vertices = [
            SIMD3<Float>(0, 0, 0),
            SIMD3<Float>(1, 0, 0),
            SIMD3<Float>(1, 1, 0),
            SIMD3<Float>(0, 1, 0)
        ]
        let colors = [
            SIMD3<UInt8>(10, 20, 30),
            SIMD3<UInt8>(200, 100, 50),
            SIMD3<UInt8>(255, 0, 128),
            SIMD3<UInt8>(1, 2, 3)
        ]
        let indices: [UInt32] = [0, 1, 2, 0, 2, 3]

        let message = MeshSnapshotMessageBuilder.makeColoredGridMeshMessage(
            header: header,
            snapshotID: "quad",
            source: "calibrated_depthanything_grid",
            frameID: "map",
            vertices: vertices,
            indices: indices,
            colors: colors
        )

        #expect(message["schema_version"] as? Int == 3)
        #expect(message["vertex_encoding"] as? String == "float32_xyz_rgb8_le_base64")
        #expect(message["vertex_stride_bytes"] as? Int == 15)
        #expect(message["index_encoding"] as? String == "uint32_le_base64")
        #expect(message["triangle_count"] as? Int == 2)
        try #require(JSONSerialization.isValidJSONObject(message))

        let vertexCount = try #require(message["vertex_count"] as? Int)
        #expect(vertexCount == 4)
        let vertexDataText = try #require(message["vertex_data"] as? String)
        let vertexData = try #require(Data(base64Encoded: vertexDataText))
        #expect(vertexData.count == vertexCount * 15)

        let indexDataText = try #require(message["index_data"] as? String)
        let indexData = try #require(Data(base64Encoded: indexDataText))
        #expect(indexData.count == indices.count * 4)

        // Vertices are compacted in first-use order; for indices 0,1,2,3 that
        // preserves the original order.
        for vertex in 0..<vertexCount {
            let base = vertex * 15
            let x = floatLE(vertexData, base)
            let y = floatLE(vertexData, base + 4)
            let z = floatLE(vertexData, base + 8)
            #expect(simd_distance(SIMD3<Float>(x, y, z), vertices[vertex]) < 1e-6)
            #expect(vertexData[base + 12] == colors[vertex].x)
            #expect(vertexData[base + 13] == colors[vertex].y)
            #expect(vertexData[base + 14] == colors[vertex].z)
        }

        for triangle in 0..<indices.count {
            #expect(uint32LE(indexData, triangle * 4) == indices[triangle])
        }
    }

    @Test("Empty colors fall back to the legacy xyz encoding at schema v3")
    func testEmptyColorsFallBackToLegacyEncoding() throws {
        let vertices = [
            SIMD3<Float>(0, 0, 0),
            SIMD3<Float>(1, 0, 0),
            SIMD3<Float>(1, 1, 0)
        ]
        let message = MeshSnapshotMessageBuilder.makeColoredGridMeshMessage(
            header: header,
            snapshotID: "tri",
            source: "calibrated_depthanything_grid",
            frameID: "map",
            vertices: vertices,
            indices: [0, 1, 2],
            colors: []
        )

        #expect(message["schema_version"] as? Int == 3)
        #expect(message["vertex_encoding"] as? String == "float32_xyz_le_base64")
        #expect(message["vertex_stride_bytes"] as? Int == 12)
        let vertexDataText = try #require(message["vertex_data"] as? String)
        let vertexData = try #require(Data(base64Encoded: vertexDataText))
        #expect(vertexData.count == 3 * 12)
    }

    @Test("Colored mesh fitting keeps the payload under the byte budget")
    func testColoredFittingRespectsPayloadBudget() throws {
        // Grid of vertices/triangles large enough to exceed a tight cap.
        var vertices: [SIMD3<Float>] = []
        var colors: [SIMD3<UInt8>] = []
        let gridSize = 40
        for row in 0..<gridSize {
            for col in 0..<gridSize {
                vertices.append(SIMD3<Float>(Float(col) * 0.05, Float(row) * 0.05, Float((row + col) % 5) * 0.02))
                colors.append(SIMD3<UInt8>(UInt8((row * 6) % 256), UInt8((col * 6) % 256), UInt8((row + col) % 256)))
            }
        }
        var indices: [UInt32] = []
        for row in 0..<(gridSize - 1) {
            for col in 0..<(gridSize - 1) {
                let upperLeft = UInt32(row * gridSize + col)
                let upperRight = upperLeft + 1
                let lowerLeft = upperLeft + UInt32(gridSize)
                let lowerRight = lowerLeft + 1
                indices.append(contentsOf: [upperLeft, lowerLeft, upperRight])
                indices.append(contentsOf: [upperRight, lowerLeft, lowerRight])
            }
        }

        let maxPayloadBytes = 12_000
        let message = MeshSnapshotMessageBuilder.makeColoredGridMeshMessage(
            header: header,
            snapshotID: "grid",
            source: "calibrated_depthanything_grid",
            frameID: "map",
            vertices: vertices,
            indices: indices,
            colors: colors,
            maxPayloadBytes: maxPayloadBytes
        )

        let encodedBytes = try #require(
            MeshSnapshotMessageBuilder.encodedPublishPayloadByteCount(
                topic: MeshSnapshotMessageSchema.shared.topic,
                msg: message
            )
        )
        #expect(message["is_truncated"] as? Bool == true)
        #expect((message["published_payload_bytes"] as? Int ?? 0) <= maxPayloadBytes)
        #expect(encodedBytes <= maxPayloadBytes)
        #expect((message["triangle_count"] as? Int ?? 0) > 0)
        #expect(message["vertex_stride_bytes"] as? Int == 15)

        let vertexCount = try #require(message["vertex_count"] as? Int)
        let vertexDataText = try #require(message["vertex_data"] as? String)
        let vertexData = try #require(Data(base64Encoded: vertexDataText))
        #expect(vertexData.count == vertexCount * 15)
    }
}
