//
//  MeshSnapshotMessageSchema.swift
//  MapEverything
//

import Foundation
import simd

struct MeshSnapshotMessageField: Identifiable, Codable, Hashable {
    let name: String
    let type: String
    let description: String
    let unsetValue: String

    var id: String { name }

    var rosMessage: [String: Any] {
        [
            "name": name,
            "type": type,
            "description": description,
            "unset_value": unsetValue
        ]
    }
}

final class MeshSnapshotMessageSchema {
    static let shared = MeshSnapshotMessageSchema()

    let messageType = "mapeverything_msgs/msg/MeshSnapshot"
    let packageName = "mapeverything_msgs"
    let messageName = "MeshSnapshot"
    let topic = "/mapping/mesh_snapshot"
    let schemaVersion = 2
    let dependencies = [
        "std_msgs/msg/Header"
    ]
    let fields: [MeshSnapshotMessageField]

    private init() {
        fields = Self.defaultFields
    }

    var messageDefinition: String {
        """
        std_msgs/Header header
        uint32 schema_version
        string snapshot_id
        string source
        string frame_id
        uint32 anchor_count
        uint32 vertex_count
        uint32 triangle_count
        uint32 original_vertex_count
        uint32 original_triangle_count
        bool is_truncated
        uint32 original_payload_bytes
        uint32 published_payload_bytes
        string compression
        string vertex_encoding
        uint32 vertex_stride_bytes
        uint8[] vertex_data
        string index_encoding
        uint32 index_stride_bytes
        uint8[] index_data
        string metadata_json
        """
    }

    var rosMessage: [String: Any] {
        [
            "package": packageName,
            "message_name": messageName,
            "message_type": messageType,
            "topic": topic,
            "schema_version": schemaVersion,
            "dependencies": dependencies,
            "fields": fields.map(\.rosMessage),
            "msg_definition": messageDefinition
        ]
    }

    private static let defaultFields: [MeshSnapshotMessageField] = [
        MeshSnapshotMessageField(
            name: "header",
            type: "std_msgs/Header",
            description: "Snapshot timestamp and map frame for the mesh vertices.",
            unsetValue: "required"
        ),
        MeshSnapshotMessageField(
            name: "schema_version",
            type: "uint32",
            description: "MeshSnapshot schema version emitted by MapEverything.",
            unsetValue: "2"
        ),
        MeshSnapshotMessageField(
            name: "snapshot_id",
            type: "string",
            description: "Unique identifier for this mesh snapshot publication.",
            unsetValue: ""
        ),
        MeshSnapshotMessageField(
            name: "source",
            type: "string",
            description: "Mesh source such as arkit_mesh or roomplan.",
            unsetValue: ""
        ),
        MeshSnapshotMessageField(
            name: "frame_id",
            type: "string",
            description: "Coordinate frame for vertices, normally map.",
            unsetValue: ""
        ),
        MeshSnapshotMessageField(
            name: "anchor_count",
            type: "uint32",
            description: "Number of AR mesh anchors represented by the snapshot.",
            unsetValue: "0"
        ),
        MeshSnapshotMessageField(
            name: "vertex_count",
            type: "uint32",
            description: "Number of vertices included after payload-size fitting.",
            unsetValue: "0"
        ),
        MeshSnapshotMessageField(
            name: "triangle_count",
            type: "uint32",
            description: "Number of triangles included after payload-size fitting.",
            unsetValue: "0"
        ),
        MeshSnapshotMessageField(
            name: "original_vertex_count",
            type: "uint32",
            description: "Number of source triangle-list vertices before payload fitting.",
            unsetValue: "0"
        ),
        MeshSnapshotMessageField(
            name: "original_triangle_count",
            type: "uint32",
            description: "Number of source triangles before payload fitting.",
            unsetValue: "0"
        ),
        MeshSnapshotMessageField(
            name: "is_truncated",
            type: "bool",
            description: "True when the source mesh was reduced to satisfy payload limits.",
            unsetValue: "false"
        ),
        MeshSnapshotMessageField(
            name: "original_payload_bytes",
            type: "uint32",
            description: "Estimated rosbridge publish payload bytes before fitting.",
            unsetValue: "0"
        ),
        MeshSnapshotMessageField(
            name: "published_payload_bytes",
            type: "uint32",
            description: "Estimated rosbridge publish payload bytes after fitting.",
            unsetValue: "0"
        ),
        MeshSnapshotMessageField(
            name: "compression",
            type: "string",
            description: "Payload compression or encoding used for this structured snapshot.",
            unsetValue: "none"
        ),
        MeshSnapshotMessageField(
            name: "vertex_encoding",
            type: "string",
            description: "Encoding for vertex_data, currently float32_xyz_le_base64.",
            unsetValue: ""
        ),
        MeshSnapshotMessageField(
            name: "vertex_stride_bytes",
            type: "uint32",
            description: "Bytes per packed vertex in vertex_data.",
            unsetValue: "0"
        ),
        MeshSnapshotMessageField(
            name: "vertex_data",
            type: "uint8[]",
            description: "Base64 rosbridge payload containing packed little-endian float32 xyz vertices.",
            unsetValue: "[]"
        ),
        MeshSnapshotMessageField(
            name: "index_encoding",
            type: "string",
            description: "Encoding for index_data, currently uint32_le_base64.",
            unsetValue: ""
        ),
        MeshSnapshotMessageField(
            name: "index_stride_bytes",
            type: "uint32",
            description: "Bytes per packed triangle index in index_data.",
            unsetValue: "0"
        ),
        MeshSnapshotMessageField(
            name: "index_data",
            type: "uint8[]",
            description: "Base64 rosbridge payload containing packed little-endian uint32 triangle indices.",
            unsetValue: "[]"
        ),
        MeshSnapshotMessageField(
            name: "metadata_json",
            type: "string",
            description: "JSON object for recorder-side diagnostics and app-specific details.",
            unsetValue: "{}"
        )
    ]
}

enum MeshSnapshotMessageBuilder {
    static func makeSafeMeshMessage(
        header: [String: Any],
        snapshotID: String,
        source: String,
        frameID: String,
        safeMeshes: [SafeARMesh],
        maxTrianglePoints: Int? = nil,
        maxPayloadBytes: Int? = nil,
        compression: String = "mesh_snapshot_binary_base64",
        metadata: [String: Any] = [:],
        topic: String = MeshSnapshotMessageSchema.shared.topic
    ) -> [String: Any] {
        let sourceVertexCount = safeMeshes.reduce(0) { $0 + $1.vertices.count }
        let sourceTriangleCount = safeMeshes.reduce(0) { $0 + ($1.indices.count / 3) }
        let pointBudgetTriangleLimit = maxTrianglePoints.map { max(0, $0 / 3) }
        let initialTriangleLimit = min(sourceTriangleCount, pointBudgetTriangleLimit ?? sourceTriangleCount)

        let originalGeometry = packedSafeMeshGeometry(
            safeMeshes,
            maxTriangleCount: initialTriangleLimit
        )
        let originalMessage = basePackedMessage(
            header: header,
            snapshotID: snapshotID,
            source: source,
            frameID: frameID,
            anchorCount: safeMeshes.count,
            vertexCount: originalGeometry.vertexCount,
            triangleCount: originalGeometry.triangleCount,
            originalVertexCount: sourceVertexCount,
            originalTriangleCount: sourceTriangleCount,
            isTruncated: originalGeometry.triangleCount < sourceTriangleCount,
            originalPayloadBytes: 0,
            publishedPayloadBytes: 0,
            compression: compression,
            vertexData: originalGeometry.vertexData,
            indexData: originalGeometry.indexData,
            metadata: metadata
        )
        let originalPayloadBytes = encodedPublishPayloadByteCount(topic: topic, msg: originalMessage) ?? 0
        let fittedGeometry = fittedSafeMeshGeometry(
            safeMeshes,
            initialTriangleLimit: originalGeometry.triangleCount,
            sourceVertexCount: sourceVertexCount,
            sourceTriangleCount: sourceTriangleCount,
            header: header,
            snapshotID: snapshotID,
            source: source,
            frameID: frameID,
            originalPayloadBytes: originalPayloadBytes,
            maxPayloadBytes: maxPayloadBytes,
            compression: compression,
            metadata: metadata,
            topic: topic
        )

        var message = basePackedMessage(
            header: header,
            snapshotID: snapshotID,
            source: source,
            frameID: frameID,
            anchorCount: safeMeshes.count,
            vertexCount: fittedGeometry.vertexCount,
            triangleCount: fittedGeometry.triangleCount,
            originalVertexCount: sourceVertexCount,
            originalTriangleCount: sourceTriangleCount,
            isTruncated: fittedGeometry.triangleCount < sourceTriangleCount,
            originalPayloadBytes: originalPayloadBytes,
            publishedPayloadBytes: 0,
            compression: compression,
            vertexData: fittedGeometry.vertexData,
            indexData: fittedGeometry.indexData,
            metadata: metadata
        )

        for _ in 0..<3 {
            let byteCount = encodedPublishPayloadByteCount(topic: topic, msg: message) ?? 0
            if message["published_payload_bytes"] as? Int == byteCount {
                break
            }
            message["published_payload_bytes"] = byteCount
        }

        return message
    }

    static func makeTriangleListMessage(
        header: [String: Any],
        snapshotID: String,
        source: String,
        frameID: String,
        anchorCount: Int,
        trianglePoints: [[String: Float]],
        originalTrianglePointCount: Int? = nil,
        maxPayloadBytes: Int? = nil,
        compression: String = "mesh_snapshot_binary_base64",
        metadata: [String: Any] = [:],
        topic: String = MeshSnapshotMessageSchema.shared.topic
    ) -> [String: Any] {
        let originalPointCount = max(0, originalTrianglePointCount ?? trianglePoints.count)
        let originalMessage = baseMessage(
            header: header,
            snapshotID: snapshotID,
            source: source,
            frameID: frameID,
            anchorCount: anchorCount,
            trianglePoints: trianglePoints,
            originalPointCount: originalPointCount,
            isTruncated: false,
            originalPayloadBytes: 0,
            publishedPayloadBytes: 0,
            compression: compression,
            metadata: metadata
        )
        let originalPayloadBytes = encodedPublishPayloadByteCount(
            topic: topic,
            msg: originalMessage
        ) ?? 0

        let fittedPoints = fittedTrianglePoints(
            trianglePoints,
            header: header,
            snapshotID: snapshotID,
            source: source,
            frameID: frameID,
            anchorCount: anchorCount,
            originalPointCount: originalPointCount,
            originalPayloadBytes: originalPayloadBytes,
            maxPayloadBytes: maxPayloadBytes,
            compression: compression,
            metadata: metadata,
            topic: topic
        )
        let isTruncated = fittedPoints.count < originalPointCount
        var message = baseMessage(
            header: header,
            snapshotID: snapshotID,
            source: source,
            frameID: frameID,
            anchorCount: anchorCount,
            trianglePoints: fittedPoints,
            originalPointCount: originalPointCount,
            isTruncated: isTruncated,
            originalPayloadBytes: originalPayloadBytes,
            publishedPayloadBytes: 0,
            compression: compression,
            metadata: metadata
        )

        for _ in 0..<3 {
            let byteCount = encodedPublishPayloadByteCount(topic: topic, msg: message) ?? 0
            if message["published_payload_bytes"] as? Int == byteCount {
                break
            }
            message["published_payload_bytes"] = byteCount
        }

        return message
    }

    static func encodedPublishPayloadByteCount(topic: String, msg: [String: Any]) -> Int? {
        let payload: [String: Any] = [
            "op": "publish",
            "topic": topic,
            "msg": msg
        ]
        return try? JSONSerialization.data(withJSONObject: payload, options: []).count
    }

    private struct PackedMeshGeometry {
        var vertexData = Data()
        var indexData = Data()
        var vertexCount = 0
        var triangleCount = 0
    }

    private static func fittedSafeMeshGeometry(
        _ safeMeshes: [SafeARMesh],
        initialTriangleLimit: Int,
        sourceVertexCount: Int,
        sourceTriangleCount: Int,
        header: [String: Any],
        snapshotID: String,
        source: String,
        frameID: String,
        originalPayloadBytes: Int,
        maxPayloadBytes: Int?,
        compression: String,
        metadata: [String: Any],
        topic: String
    ) -> PackedMeshGeometry {
        guard let maxPayloadBytes, maxPayloadBytes > 0 else {
            return packedSafeMeshGeometry(safeMeshes, maxTriangleCount: initialTriangleLimit)
        }

        var triangleLimit = initialTriangleLimit
        while triangleLimit > 0 {
            let geometry = packedSafeMeshGeometry(safeMeshes, maxTriangleCount: triangleLimit)
            let candidate = basePackedMessage(
                header: header,
                snapshotID: snapshotID,
                source: source,
                frameID: frameID,
                anchorCount: safeMeshes.count,
                vertexCount: geometry.vertexCount,
                triangleCount: geometry.triangleCount,
                originalVertexCount: sourceVertexCount,
                originalTriangleCount: sourceTriangleCount,
                isTruncated: geometry.triangleCount < sourceTriangleCount,
                originalPayloadBytes: originalPayloadBytes,
                publishedPayloadBytes: maxPayloadBytes,
                compression: compression,
                vertexData: geometry.vertexData,
                indexData: geometry.indexData,
                metadata: metadata
            )

            guard let byteCount = encodedPublishPayloadByteCount(topic: topic, msg: candidate),
                  byteCount > maxPayloadBytes else {
                return geometry
            }

            let estimatedCount = Int(Double(triangleLimit) * Double(maxPayloadBytes) / Double(byteCount))
            triangleLimit = max(0, min(triangleLimit - 1, estimatedCount))
        }

        return PackedMeshGeometry()
    }

    private static func packedSafeMeshGeometry(
        _ safeMeshes: [SafeARMesh],
        maxTriangleCount: Int
    ) -> PackedMeshGeometry {
        guard maxTriangleCount > 0 else { return PackedMeshGeometry() }

        var geometry = PackedMeshGeometry()
        geometry.vertexData.reserveCapacity(maxTriangleCount * 3 * 3 * MemoryLayout<Float32>.size)
        geometry.indexData.reserveCapacity(maxTriangleCount * 3 * MemoryLayout<UInt32>.size)

        for mesh in safeMeshes {
            guard geometry.triangleCount < maxTriangleCount else { break }
            var remappedIndices: [UInt32: UInt32] = [:]
            remappedIndices.reserveCapacity(min(mesh.vertices.count, maxTriangleCount * 3))

            for faceIndex in 0..<(mesh.indices.count / 3) {
                guard geometry.triangleCount < maxTriangleCount else { break }

                let indexOffset = faceIndex * 3
                let localIndices = [
                    mesh.indices[indexOffset],
                    mesh.indices[indexOffset + 1],
                    mesh.indices[indexOffset + 2]
                ]
                guard localIndices.allSatisfy({ Int($0) < mesh.vertices.count }) else { continue }

                for localIndex in localIndices {
                    let globalIndex: UInt32
                    if let existing = remappedIndices[localIndex] {
                        globalIndex = existing
                    } else {
                        globalIndex = UInt32(geometry.vertexCount)
                        remappedIndices[localIndex] = globalIndex
                        appendWorldVertex(
                            mesh.vertices[Int(localIndex)],
                            transform: mesh.transform,
                            to: &geometry.vertexData
                        )
                        geometry.vertexCount += 1
                    }
                    appendLittleEndianUInt32(globalIndex, to: &geometry.indexData)
                }
                geometry.triangleCount += 1
            }
        }

        return geometry
    }

    private static func fittedTrianglePoints(
        _ points: [[String: Float]],
        header: [String: Any],
        snapshotID: String,
        source: String,
        frameID: String,
        anchorCount: Int,
        originalPointCount: Int,
        originalPayloadBytes: Int,
        maxPayloadBytes: Int?,
        compression: String,
        metadata: [String: Any],
        topic: String
    ) -> [[String: Float]] {
        guard let maxPayloadBytes, maxPayloadBytes > 0 else { return points }

        var fittedPoints = points
        while !fittedPoints.isEmpty {
            let candidate = baseMessage(
                header: header,
                snapshotID: snapshotID,
                source: source,
                frameID: frameID,
                anchorCount: anchorCount,
                trianglePoints: fittedPoints,
                originalPointCount: originalPointCount,
                isTruncated: fittedPoints.count < originalPointCount,
                originalPayloadBytes: originalPayloadBytes,
                publishedPayloadBytes: 0,
                compression: compression,
                metadata: metadata
            )
            guard let byteCount = encodedPublishPayloadByteCount(topic: topic, msg: candidate),
                  byteCount > maxPayloadBytes else {
                return fittedPoints
            }

            let estimatedCount = Int(Double(fittedPoints.count) * Double(maxPayloadBytes) / Double(byteCount))
            let alignedCount = max(0, min(fittedPoints.count - 3, estimatedCount) - (min(fittedPoints.count - 3, estimatedCount) % 3))

            if alignedCount >= 3 {
                fittedPoints = Array(fittedPoints.prefix(alignedCount))
            } else {
                fittedPoints.removeAll()
            }
        }

        return []
    }

    private static func baseMessage(
        header: [String: Any],
        snapshotID: String,
        source: String,
        frameID: String,
        anchorCount: Int,
        trianglePoints: [[String: Float]],
        originalPointCount: Int,
        isTruncated: Bool,
        originalPayloadBytes: Int,
        publishedPayloadBytes: Int,
        compression: String,
        metadata: [String: Any]
    ) -> [String: Any] {
        let vertexCount = trianglePoints.count
        return basePackedMessage(
            header: header,
            snapshotID: snapshotID,
            source: source,
            frameID: frameID,
            anchorCount: anchorCount,
            vertexCount: vertexCount,
            triangleCount: vertexCount / 3,
            originalVertexCount: originalPointCount,
            originalTriangleCount: originalPointCount / 3,
            isTruncated: isTruncated,
            originalPayloadBytes: originalPayloadBytes,
            publishedPayloadBytes: publishedPayloadBytes,
            compression: compression,
            vertexData: packedVertexData(trianglePoints),
            indexData: packedSequentialIndexData(count: vertexCount),
            metadata: metadata
        )
    }

    private static func basePackedMessage(
        header: [String: Any],
        snapshotID: String,
        source: String,
        frameID: String,
        anchorCount: Int,
        vertexCount: Int,
        triangleCount: Int,
        originalVertexCount: Int,
        originalTriangleCount: Int,
        isTruncated: Bool,
        originalPayloadBytes: Int,
        publishedPayloadBytes: Int,
        compression: String,
        vertexData: Data,
        indexData: Data,
        metadata: [String: Any]
    ) -> [String: Any] {
        [
            "header": header,
            "schema_version": MeshSnapshotMessageSchema.shared.schemaVersion,
            "snapshot_id": snapshotID,
            "source": source,
            "frame_id": frameID,
            "anchor_count": max(0, anchorCount),
            "vertex_count": vertexCount,
            "triangle_count": triangleCount,
            "original_vertex_count": originalVertexCount,
            "original_triangle_count": originalTriangleCount,
            "is_truncated": isTruncated,
            "original_payload_bytes": originalPayloadBytes,
            "published_payload_bytes": publishedPayloadBytes,
            "compression": compression,
            "vertex_encoding": "float32_xyz_le_base64",
            "vertex_stride_bytes": 12,
            "vertex_data": vertexData.base64EncodedString(),
            "index_encoding": "uint32_le_base64",
            "index_stride_bytes": 4,
            "index_data": indexData.base64EncodedString(),
            "metadata_json": metadataJSONString(metadata)
        ]
    }

    private static func packedVertexData(_ points: [[String: Float]]) -> Data {
        var data = Data()
        data.reserveCapacity(points.count * 3 * MemoryLayout<Float32>.size)

        for point in points {
            appendLittleEndianFloat32(sanitizedFloat(point["x"]), to: &data)
            appendLittleEndianFloat32(sanitizedFloat(point["y"]), to: &data)
            appendLittleEndianFloat32(sanitizedFloat(point["z"]), to: &data)
        }

        return data
    }

    private static func packedSequentialIndexData(count: Int) -> Data {
        var data = Data()
        data.reserveCapacity(count * MemoryLayout<UInt32>.size)

        for index in 0..<count {
            appendLittleEndianUInt32(UInt32(index), to: &data)
        }

        return data
    }

    private static func appendWorldVertex(
        _ vertex: SIMD3<Float>,
        transform: simd_float4x4,
        to data: inout Data
    ) {
        let world = simd_mul(transform, SIMD4<Float>(vertex.x, vertex.y, vertex.z, 1.0))
        appendLittleEndianFloat32(sanitizedFloat(world.x), to: &data)
        appendLittleEndianFloat32(sanitizedFloat(world.y), to: &data)
        appendLittleEndianFloat32(sanitizedFloat(world.z), to: &data)
    }

    private static func appendLittleEndianFloat32(_ value: Float32, to data: inout Data) {
        var littleEndian = value.bitPattern.littleEndian
        withUnsafeBytes(of: &littleEndian) { bytes in
            data.append(contentsOf: bytes)
        }
    }

    private static func appendLittleEndianUInt32(_ value: UInt32, to data: inout Data) {
        var littleEndian = value.littleEndian
        withUnsafeBytes(of: &littleEndian) { bytes in
            data.append(contentsOf: bytes)
        }
    }

    private static func sanitizedFloat(_ value: Float?) -> Float32 {
        guard let value, value.isFinite else { return 0 }
        return Float32(value)
    }

    private static func metadataJSONString(_ metadata: [String: Any]) -> String {
        guard JSONSerialization.isValidJSONObject(metadata),
              let data = try? JSONSerialization.data(withJSONObject: metadata, options: [.sortedKeys]),
              let string = String(data: data, encoding: .utf8) else {
            return "{}"
        }
        return string
    }
}
