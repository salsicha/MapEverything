//
//  ROS2TopicRegistry.swift
//  MapEverything
//

import Foundation

enum ROS2TopicID: String, CaseIterable, Codable, Hashable {
    case pose
    case pointCloud
    case imu
    case meshMarkers
    case meshSnapshot
    case tf
    case cameraCompressed
    case cameraInfo
    case surfels
    case gpsFix
    case gpsMetadata
    case odom
    case status
    case session
    case radio
    case indoorLocalization
    case satelliteImage
    case satelliteTileInfo
    case demTile
}

struct ROS2TopicDefinition: Identifiable, Codable, Hashable {
    let id: ROS2TopicID
    let stream: MappingSensorStream
    let topic: String
    let messageType: String
    let defaultRateHz: Double?
    let isImplemented: Bool
}

final class ROS2TopicRegistry {
    static let shared = ROS2TopicRegistry()

    private let lock = NSLock()
    private let definitions: [ROS2TopicDefinition]
    private let definitionsByID: [ROS2TopicID: ROS2TopicDefinition]
    private var enabledStreams: Set<MappingSensorStream>

    init(
        definitions: [ROS2TopicDefinition] = ROS2TopicRegistry.defaultDefinitions,
        enabledStreams: Set<MappingSensorStream> = ROS2TopicRegistry.defaultEnabledStreams
    ) {
        self.definitions = definitions
        self.definitionsByID = Dictionary(uniqueKeysWithValues: definitions.map { ($0.id, $0) })
        self.enabledStreams = enabledStreams
    }

    func configure(enabledStreams: Set<MappingSensorStream>) {
        lock.lock()
        self.enabledStreams = enabledStreams
        lock.unlock()
    }

    func setStream(_ stream: MappingSensorStream, isEnabled: Bool) {
        lock.lock()
        if isEnabled {
            enabledStreams.insert(stream)
        } else {
            enabledStreams.remove(stream)
        }
        lock.unlock()
    }

    func isStreamEnabled(_ stream: MappingSensorStream) -> Bool {
        lock.lock()
        let isEnabled = enabledStreams.contains(stream)
        lock.unlock()
        return isEnabled
    }

    func definition(_ id: ROS2TopicID) -> ROS2TopicDefinition {
        guard let definition = definitionsByID[id] else {
            fatalError("Missing ROS2 topic definition for \(id.rawValue)")
        }
        return definition
    }

    func topic(_ id: ROS2TopicID) -> String {
        definition(id).topic
    }

    func definition(forTopic topic: String) -> ROS2TopicDefinition? {
        definitions.first { $0.topic == topic }
    }

    func advertisedTopics() -> [ROS2TopicDefinition] {
        lock.lock()
        let streams = enabledStreams
        lock.unlock()

        return definitions.filter { definition in
            definition.isImplemented && streams.contains(definition.stream)
        }
    }

    func allTopics() -> [ROS2TopicDefinition] {
        definitions
    }

    private static let defaultEnabledStreams: Set<MappingSensorStream> = [
        .pose,
        .surfels,
        .gps,
        .satelliteImagery,
        .dem
    ]

    private static let defaultDefinitions: [ROS2TopicDefinition] = [
        ROS2TopicDefinition(
            id: .pose,
            stream: .pose,
            topic: "/reconstructor/pose",
            messageType: "geometry_msgs/msg/PoseStamped",
            defaultRateHz: 30,
            isImplemented: true
        ),
        ROS2TopicDefinition(
            id: .pointCloud,
            stream: .pointCloud,
            topic: "/reconstructor/pointcloud",
            messageType: "sensor_msgs/msg/PointCloud2",
            defaultRateHz: 5,
            isImplemented: true
        ),
        ROS2TopicDefinition(
            id: .imu,
            stream: .imu,
            topic: "/reconstructor/imu",
            messageType: "sensor_msgs/msg/Imu",
            defaultRateHz: 100,
            isImplemented: true
        ),
        ROS2TopicDefinition(
            id: .meshMarkers,
            stream: .mesh,
            topic: "/reconstructor/map",
            messageType: "visualization_msgs/msg/MarkerArray",
            defaultRateHz: 0.5,
            isImplemented: true
        ),
        ROS2TopicDefinition(
            id: .meshSnapshot,
            stream: .mesh,
            topic: MeshSnapshotMessageSchema.shared.topic,
            messageType: MeshSnapshotMessageSchema.shared.messageType,
            defaultRateHz: 0.5,
            isImplemented: true
        ),
        ROS2TopicDefinition(
            id: .tf,
            stream: .tf,
            topic: "/tf",
            messageType: "tf2_msgs/msg/TFMessage",
            defaultRateHz: 30,
            isImplemented: true
        ),
        ROS2TopicDefinition(
            id: .cameraCompressed,
            stream: .camera,
            topic: "/reconstructor/camera/image/compressed",
            messageType: "sensor_msgs/msg/CompressedImage",
            defaultRateHz: 10,
            isImplemented: true
        ),
        ROS2TopicDefinition(
            id: .cameraInfo,
            stream: .camera,
            topic: "/reconstructor/camera/camera_info",
            messageType: "sensor_msgs/msg/CameraInfo",
            defaultRateHz: 10,
            isImplemented: true
        ),
        ROS2TopicDefinition(
            id: .surfels,
            stream: .surfels,
            topic: "/reconstructor/surfels",
            messageType: "sensor_msgs/msg/PointCloud2",
            defaultRateHz: 1,
            isImplemented: true
        ),
        ROS2TopicDefinition(
            id: .gpsFix,
            stream: .gps,
            topic: "/reconstructor/gps/fix",
            messageType: "sensor_msgs/msg/NavSatFix",
            defaultRateHz: 1,
            isImplemented: true
        ),
        ROS2TopicDefinition(
            id: .gpsMetadata,
            stream: .gps,
            topic: "/reconstructor/gps/metadata",
            messageType: "reconstructor_msgs/msg/GPSMetadata",
            defaultRateHz: 1,
            isImplemented: true
        ),
        ROS2TopicDefinition(
            id: .odom,
            stream: .odometry,
            topic: "/reconstructor/odom",
            messageType: "nav_msgs/msg/Odometry",
            defaultRateHz: 30,
            isImplemented: true
        ),
        ROS2TopicDefinition(
            id: .status,
            stream: .diagnostics,
            topic: "/reconstructor/status",
            messageType: "diagnostic_msgs/msg/DiagnosticArray",
            defaultRateHz: 1,
            isImplemented: true
        ),
        ROS2TopicDefinition(
            id: .session,
            stream: .session,
            topic: "/reconstructor/session",
            messageType: "reconstructor_msgs/msg/MappingSession",
            defaultRateHz: nil,
            isImplemented: true
        ),
        ROS2TopicDefinition(
            id: .radio,
            stream: .radio,
            topic: "/reconstructor/radio",
            messageType: "reconstructor_msgs/msg/RadioObservation",
            defaultRateHz: 2,
            isImplemented: true
        ),
        ROS2TopicDefinition(
            id: .indoorLocalization,
            stream: .indoorLocalization,
            topic: "/reconstructor/indoor_localization",
            messageType: "reconstructor_msgs/msg/IndoorLocalization",
            defaultRateHz: 1,
            isImplemented: true
        ),
        ROS2TopicDefinition(
            id: .satelliteImage,
            stream: .satelliteImagery,
            topic: "/reconstructor/satellite/image/compressed",
            messageType: "sensor_msgs/msg/CompressedImage",
            defaultRateHz: nil,
            isImplemented: true
        ),
        ROS2TopicDefinition(
            id: .satelliteTileInfo,
            stream: .satelliteImagery,
            topic: "/reconstructor/satellite/tile_info",
            messageType: "reconstructor_msgs/msg/GeoTileInfo",
            defaultRateHz: nil,
            isImplemented: true
        ),
        ROS2TopicDefinition(
            id: .demTile,
            stream: .dem,
            topic: "/reconstructor/dem/tile",
            messageType: "reconstructor_msgs/msg/GeoRasterTile",
            defaultRateHz: nil,
            isImplemented: true
        )
    ]
}
