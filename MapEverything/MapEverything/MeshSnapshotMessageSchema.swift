//
//  MeshSnapshotMessageSchema.swift
//  MapEverything
//

import Foundation

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

    let messageType = "reconstructor_msgs/msg/MeshSnapshot"
    let packageName = "reconstructor_msgs"
    let messageName = "MeshSnapshot"
    let topic = "/mapping/mesh_snapshot"
    let schemaVersion = 1
    let dependencies = [
        "std_msgs/msg/Header",
        "geometry_msgs/msg/Point"
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
        geometry_msgs/Point[] vertices
        uint32[] triangle_indices
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
            unsetValue: "1"
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
            name: "vertices",
            type: "geometry_msgs/Point[]",
            description: "Triangle-list vertices in frame_id coordinates.",
            unsetValue: "[]"
        ),
        MeshSnapshotMessageField(
            name: "triangle_indices",
            type: "uint32[]",
            description: "Triangle indices over vertices; every three indices form one triangle.",
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
    static func makeTriangleListMessage(
        header: [String: Any],
        snapshotID: String,
        source: String,
        frameID: String,
        anchorCount: Int,
        trianglePoints: [[String: Float]],
        originalTrianglePointCount: Int? = nil,
        maxPayloadBytes: Int? = nil,
        compression: String = "none",
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
        let vertices = trianglePoints.map(pointMessage)
        return [
            "header": header,
            "schema_version": MeshSnapshotMessageSchema.shared.schemaVersion,
            "snapshot_id": snapshotID,
            "source": source,
            "frame_id": frameID,
            "anchor_count": max(0, anchorCount),
            "vertex_count": vertices.count,
            "triangle_count": vertices.count / 3,
            "original_vertex_count": originalPointCount,
            "original_triangle_count": originalPointCount / 3,
            "is_truncated": isTruncated,
            "original_payload_bytes": originalPayloadBytes,
            "published_payload_bytes": publishedPayloadBytes,
            "compression": compression,
            "vertices": vertices,
            "triangle_indices": Array(0..<vertices.count),
            "metadata_json": metadataJSONString(metadata)
        ]
    }

    private static func pointMessage(_ point: [String: Float]) -> [String: Double] {
        [
            "x": finiteDouble(point["x"] ?? 0),
            "y": finiteDouble(point["y"] ?? 0),
            "z": finiteDouble(point["z"] ?? 0)
        ]
    }

    private static func finiteDouble(_ value: Float) -> Double {
        value.isFinite ? Double(value) : 0
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
