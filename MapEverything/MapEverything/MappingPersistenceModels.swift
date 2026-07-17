//
//  MappingPersistenceModels.swift
//  MapEverything
//

import Foundation
import SwiftData

enum MapEverythingModelSchema {
    static let models: [any PersistentModel.Type] = [
        MappingSessionModel.self
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
    /// User-authored free text; never touched by the snapshot upsert path.
    var notes: String

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
        sessionDirectoryPath: String? = nil,
        notes: String = ""
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
        self.notes = notes
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

/// Upserts one `MappingSessionModel` record per mapping session so the app
/// keeps a browsable history across launches.
@MainActor
final class MappingSessionHistoryStore {
    private let context: ModelContext

    init(context: ModelContext) {
        self.context = context
    }

    func record(
        _ snapshot: MappingSessionSnapshot,
        metadataJSON: String = "{}",
        sessionDirectoryPath: String? = nil
    ) {
        guard let sessionID = snapshot.sessionID else { return }

        do {
            let target: UUID? = sessionID
            var descriptor = FetchDescriptor<MappingSessionModel>(
                predicate: #Predicate { $0.sessionID == target }
            )
            descriptor.fetchLimit = 1

            if let existing = try context.fetch(descriptor).first {
                existing.update(from: snapshot, metadataJSON: metadataJSON)
                if let sessionDirectoryPath {
                    existing.sessionDirectoryPath = sessionDirectoryPath
                }
            } else {
                let record = MappingSessionModel(snapshot: snapshot, metadataJSON: metadataJSON)
                record.sessionDirectoryPath = sessionDirectoryPath
                context.insert(record)
            }

            try context.save()
        } catch {
            print("MappingSessionHistoryStore: failed to save session record: \(error)")
        }
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
