//
//  MappingPersistenceModels.swift
//  MapEverything
//

import Foundation
import SwiftData

enum MapEverythingModelSchema {
    static let models: [any PersistentModel.Type] = [
        MappingSessionModel.self,
        SensorStreamModel.self,
        GeoTileModel.self
    ]

    static var schema: Schema {
        Schema(models)
    }
}

@Model
final class MappingSessionModel {
    @Attribute(.unique) var id: UUID
    var sessionID: UUID?
    var name: String
    var state: String
    var recorderURL: String
    var bridgeTransport: String
    var enabledStreams: [String]
    var coordinateFrameConfigJSON: String
    var providerConfigJSON: String
    var metadataJSON: String
    var startedAt: Date?
    var endedAt: Date?
    var lastUpdatedAt: Date
    var lastError: String?
    var sessionDirectoryPath: String?

    init(
        id: UUID = UUID(),
        sessionID: UUID? = nil,
        name: String = "",
        state: String = "Idle",
        recorderURL: String = "",
        bridgeTransport: String = ROS2BridgeTransportProfile.current.kind.rawValue,
        enabledStreams: [String] = [],
        coordinateFrameConfigJSON: String = MappingSessionModel.defaultCoordinateFrameConfigJSON,
        providerConfigJSON: String = "[]",
        metadataJSON: String = "{}",
        startedAt: Date? = nil,
        endedAt: Date? = nil,
        lastUpdatedAt: Date = Date(),
        lastError: String? = nil,
        sessionDirectoryPath: String? = nil
    ) {
        self.id = id
        self.sessionID = sessionID
        self.name = name
        self.state = state
        self.recorderURL = recorderURL
        self.bridgeTransport = bridgeTransport
        self.enabledStreams = enabledStreams.sorted()
        self.coordinateFrameConfigJSON = coordinateFrameConfigJSON
        self.providerConfigJSON = providerConfigJSON
        self.metadataJSON = metadataJSON
        self.startedAt = startedAt
        self.endedAt = endedAt
        self.lastUpdatedAt = lastUpdatedAt
        self.lastError = lastError
        self.sessionDirectoryPath = sessionDirectoryPath
    }

    convenience init(snapshot: MappingSessionSnapshot, metadataJSON: String = "{}") {
        self.init(
            sessionID: snapshot.sessionID,
            name: snapshot.sessionID?.uuidString ?? "Unsaved mapping session",
            state: snapshot.state,
            recorderURL: snapshot.recorderURL,
            bridgeTransport: ROS2BridgeTransportProfile.current.kind.rawValue,
            enabledStreams: snapshot.enabledStreams,
            coordinateFrameConfigJSON: Self.defaultCoordinateFrameConfigJSON,
            providerConfigJSON: Self.providerConfigurationsJSON(),
            metadataJSON: metadataJSON,
            startedAt: snapshot.startedAt,
            endedAt: snapshot.endedAt,
            lastError: snapshot.lastError
        )
    }

    func update(from snapshot: MappingSessionSnapshot, metadataJSON: String? = nil, at date: Date = Date()) {
        sessionID = snapshot.sessionID
        if name.isEmpty {
            name = snapshot.sessionID?.uuidString ?? "Unsaved mapping session"
        }
        state = snapshot.state
        recorderURL = snapshot.recorderURL
        bridgeTransport = ROS2BridgeTransportProfile.current.kind.rawValue
        enabledStreams = snapshot.enabledStreams.sorted()
        providerConfigJSON = Self.providerConfigurationsJSON()
        if let metadataJSON {
            self.metadataJSON = metadataJSON
        }
        startedAt = snapshot.startedAt
        endedAt = snapshot.endedAt
        lastUpdatedAt = date
        lastError = snapshot.lastError
    }

    static let defaultCoordinateFrameConfigJSON = """
    {"earth_frame_id":"earth","map_frame_id":"map","odom_frame_id":"odom","base_frame_id":"base_link","camera_frame_id":"iphone_camera"}
    """

    static func providerConfigurationsJSON(
        configurations: [GeoTileOptionalProviderConfiguration] = GeoTileProviderConfigurationStore.load()
    ) -> String {
        jsonString(configurations.map(\.rosMessage), fallback: "[]")
    }
}

@Model
final class SensorStreamModel {
    @Attribute(.unique) var id: String
    var streamID: String
    var displayName: String
    var topic: String
    var messageType: String
    var isEnabled: Bool
    var isImplemented: Bool
    var targetRateHz: Double
    var lastPublishRateHz: Double
    var messageCount: Int
    var sentMessages: Int
    var droppedMessages: Int
    var retriedMessages: Int
    var failedMessages: Int
    var lastPublishedAt: Date?
    var lastError: String?
    var lastErrorAt: Date?

    init(
        id: String,
        streamID: String,
        displayName: String,
        topic: String,
        messageType: String,
        isEnabled: Bool,
        isImplemented: Bool,
        targetRateHz: Double,
        lastPublishRateHz: Double = 0,
        messageCount: Int = 0,
        sentMessages: Int = 0,
        droppedMessages: Int = 0,
        retriedMessages: Int = 0,
        failedMessages: Int = 0,
        lastPublishedAt: Date? = nil,
        lastError: String? = nil,
        lastErrorAt: Date? = nil
    ) {
        self.id = id
        self.streamID = streamID
        self.displayName = displayName
        self.topic = topic
        self.messageType = messageType
        self.isEnabled = isEnabled
        self.isImplemented = isImplemented
        self.targetRateHz = targetRateHz
        self.lastPublishRateHz = lastPublishRateHz
        self.messageCount = messageCount
        self.sentMessages = sentMessages
        self.droppedMessages = droppedMessages
        self.retriedMessages = retriedMessages
        self.failedMessages = failedMessages
        self.lastPublishedAt = lastPublishedAt
        self.lastError = lastError
        self.lastErrorAt = lastErrorAt
    }

    convenience init(topic: ROS2TopicDefinition, isEnabled: Bool) {
        self.init(
            id: topic.id.rawValue,
            streamID: topic.stream.rawValue,
            displayName: topic.stream.displayName,
            topic: topic.topic,
            messageType: topic.messageType,
            isEnabled: isEnabled,
            isImplemented: topic.isImplemented,
            targetRateHz: topic.defaultRateHz ?? 0
        )
    }

    func apply(stats: PublishQueueStats, lastPublishedAt: Date? = Date()) {
        sentMessages = stats.sentMessages
        droppedMessages = stats.droppedMessages
        retriedMessages = stats.retriedMessages
        failedMessages = stats.failedMessages
        messageCount = stats.sentMessages + stats.failedMessages
        self.lastPublishedAt = lastPublishedAt
        lastError = stats.lastError
        lastErrorAt = stats.lastErrorAt
    }

    func apply(payloadMetrics snapshot: StreamPayloadMetricSnapshot) {
        messageCount = snapshot.messageCount
        lastPublishedAt = snapshot.lastRecordedAt
        lastPublishRateHz = 0
    }
}

@Model
final class GeoTileModel {
    @Attribute(.unique) var id: String
    var providerName: String
    var layer: String
    var kind: String
    var crs: String
    var zoom: Int
    var tileX: Int
    var tileY: Int
    var time: String?
    var format: String
    var mimeType: String
    var encoding: String
    var cachePath: String
    var sourceURL: String
    var attribution: String
    var license: String
    var sourcePolicyJSON: String
    var west: Double
    var south: Double
    var east: Double
    var north: Double
    var byteCount: Int
    var fetchedAt: Date
    var lastAccessedAt: Date
    var isRecordableByDefault: Bool

    init(
        id: String,
        providerName: String,
        layer: String,
        kind: String,
        crs: String,
        zoom: Int,
        tileX: Int,
        tileY: Int,
        time: String?,
        format: String,
        mimeType: String,
        encoding: String,
        cachePath: String,
        sourceURL: String,
        attribution: String,
        license: String,
        sourcePolicyJSON: String,
        west: Double,
        south: Double,
        east: Double,
        north: Double,
        byteCount: Int,
        fetchedAt: Date,
        lastAccessedAt: Date = Date(),
        isRecordableByDefault: Bool
    ) {
        self.id = id
        self.providerName = providerName
        self.layer = layer
        self.kind = kind
        self.crs = crs
        self.zoom = zoom
        self.tileX = tileX
        self.tileY = tileY
        self.time = time
        self.format = format
        self.mimeType = mimeType
        self.encoding = encoding
        self.cachePath = cachePath
        self.sourceURL = sourceURL
        self.attribution = attribution
        self.license = license
        self.sourcePolicyJSON = sourcePolicyJSON
        self.west = west
        self.south = south
        self.east = east
        self.north = north
        self.byteCount = byteCount
        self.fetchedAt = fetchedAt
        self.lastAccessedAt = lastAccessedAt
        self.isRecordableByDefault = isRecordableByDefault
    }

    convenience init(payload: GeoTilePayload, cachePath: String) {
        self.init(
            id: Self.id(
                provider: payload.provider,
                coordinate: payload.coordinate,
                time: payload.time
            ),
            providerName: payload.provider.name,
            layer: payload.provider.layer,
            kind: payload.provider.kind.rawValue,
            crs: payload.provider.crs,
            zoom: payload.coordinate.z,
            tileX: payload.coordinate.x,
            tileY: payload.coordinate.y,
            time: payload.time,
            format: payload.provider.format,
            mimeType: payload.provider.mimeType,
            encoding: payload.provider.encoding,
            cachePath: cachePath,
            sourceURL: payload.sourceURL.absoluteString,
            attribution: payload.provider.attribution,
            license: payload.provider.license,
            sourcePolicyJSON: jsonString(payload.provider.sourcePolicy.rosMessage),
            west: payload.bounds.west,
            south: payload.bounds.south,
            east: payload.bounds.east,
            north: payload.bounds.north,
            byteCount: payload.data.count,
            fetchedAt: payload.fetchedAt,
            isRecordableByDefault: payload.provider.sourcePolicy.recordableByDefault
        )
    }

    static func id(provider: GeoTileProvider, coordinate: GeoTileCoordinate, time: String?) -> String {
        [
            provider.kind.rawValue,
            provider.name,
            provider.layer,
            time ?? "static",
            String(coordinate.z),
            String(coordinate.x),
            String(coordinate.y)
        ].joined(separator: "|")
    }
}

nonisolated func jsonString(_ value: Any, fallback: String = "{}") -> String {
    let normalized = jsonCompatible(value)
    guard JSONSerialization.isValidJSONObject(normalized),
          let data = try? JSONSerialization.data(withJSONObject: normalized, options: [.sortedKeys]),
          let string = String(data: data, encoding: .utf8) else {
        return fallback
    }
    return string
}

nonisolated private func jsonCompatible(_ value: Any) -> Any {
    switch value {
    case let dictionary as [String: Any]:
        return Dictionary(uniqueKeysWithValues: dictionary.map { key, value in
            (key, jsonCompatible(value))
        })
    case let array as [Any]:
        return array.map { jsonCompatible($0) }
    case let value as Date:
        return ISO8601DateFormatter().string(from: value)
    case let value as UUID:
        return value.uuidString
    case let value as Double:
        return value.isFinite ? value : 0.0
    case let value as Float:
        return value.isFinite ? Double(value) : 0.0
    case let value as String:
        return value
    case let value as Bool:
        return value
    case let value as NSNumber:
        return value
    default:
        return String(describing: value)
    }
}
