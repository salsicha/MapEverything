//
//  MapEverythingTests.swift
//  MapEverythingTests
//
//  Created by Alex Moran on 5/2/26.
//

import Testing
import Foundation
import simd
import CoreGraphics
import CoreVideo
import CoreLocation
import SwiftData
import SQLite3
@testable import MapEverything

struct MapEverythingTests {

    @Test("Filters out points beyond 20 meters")
    func testPointCloudProcessorOutlierRemoval() {
        let processor = PointCloudProcessor()
        let validPoint = ColoredPoint(position: SIMD3<Float>(1.0, 2.0, 3.0))
        let outlierPoint = ColoredPoint(position: SIMD3<Float>(30.0, 0.0, 0.0))
        
        let points = [validPoint, outlierPoint]
        let filtered = processor.removeOutliers(points: points, maxDistance: 20.0)
        
        #expect(filtered.count == 1)
        #expect(filtered.first == validPoint)
    }
    
    @Test("Downsamples points within the same voxel")
    func testPointCloudProcessorVoxelGridFilter() {
        let processor = PointCloudProcessor()
        let p1 = ColoredPoint(position: SIMD3<Float>(0.01, 0.01, 0.01))
        let p2 = ColoredPoint(position: SIMD3<Float>(0.02, 0.01, 0.01))
        let p3 = ColoredPoint(position: SIMD3<Float>(0.01, 0.02, 0.01))
        let p4 = ColoredPoint(position: SIMD3<Float>(1.0, 1.0, 1.0))
        
        let points = [p1, p2, p3, p4]
        // A voxel size of 0.05 groups p1, p2, and p3 into the same grid index
        let downsampled = processor.voxelGridFilter(points: points, voxelSize: 0.05)
        
        #expect(downsampled.count == 2)
    }

    @Test("Colored surfel map fuses repeated RGB-D samples")
    func testColoredSurfelMapFusesSamples() async throws {
        let map = ColoredSurfelMap(voxelSize: 0.05, maxSurfels: 10)
        let first = ColoredPoint(
            position: SIMD3<Float>(0.01, 0.01, -1.0),
            color: SIMD3<UInt8>(200, 10, 20)
        )
        let second = ColoredPoint(
            position: SIMD3<Float>(0.02, 0.01, -1.0),
            color: SIMD3<UInt8>(100, 30, 40)
        )

        let count = await map.fuse(
            points: [first, second],
            observerPosition: SIMD3<Float>(0, 0, 0),
            timestamp: 1
        )
        let surfels = await map.snapshot()
        let surfel = try #require(surfels.first)

        #expect(count == 1)
        #expect(surfels.count == 1)
        #expect(surfel.observationCount == 2)
        #expect(surfel.confidence > 0.2)
        #expect(surfel.radius > 0)
        #expect(simd_length(surfel.normal) > 0.99)
        #expect(surfel.color.x < 200)
        #expect(surfel.color.x > 100)
    }
    
    @Test("Successfully saves and loads PLY files to disk")
    func testStorageManagerSaveAndLoadPLY() {
        let manager = PointCloudStorageManager.shared
        let points = [ColoredPoint(position: SIMD3<Float>(1.0, 2.0, 3.0)), ColoredPoint(position: SIMD3<Float>(4.0, 5.0, 6.0))]
        let testFilename = "test_pointcloud_\(UUID().uuidString)"
        
        let savedFile = manager.saveBinaryPLY(points: points, to: testFilename)
        #expect(savedFile != nil)
        #expect(savedFile == "\(testFilename).ply")
        
        let loadedPoints = manager.loadBinaryPLY(from: savedFile!)
        #expect(loadedPoints != nil)
        #expect(loadedPoints?.count == 2)
        #expect(loadedPoints?.first == points.first)
    }

    @Test("Successfully saves surfel PLY files with point-cloud compatible loading")
    func testStorageManagerSaveAndLoadSurfelPLY() throws {
        let manager = PointCloudStorageManager.shared
        let filename = "test_surfels_\(UUID().uuidString)"
        let surfels = [
            ColoredSurfel(
                position: SIMD3<Float>(1.0, 2.0, 3.0),
                normal: SIMD3<Float>(0, 1, 0),
                color: SIMD3<UInt8>(10, 20, 30),
                radius: 0.04,
                confidence: 0.8,
                observationCount: 3
            )
        ]

        let savedFile = try #require(manager.saveBinaryPLY(surfels: surfels, to: filename))
        let loadedPoints = try #require(manager.loadBinaryPLY(from: savedFile))

        #expect(savedFile == "\(filename).ply")
        #expect(loadedPoints.count == 1)
        #expect(loadedPoints.first?.position == surfels.first?.position)
        #expect(loadedPoints.first?.color == surfels.first?.color)
    }

    @Test("Environment model initializes correctly")
    func testEnvironmentModelInitialization() {
        let date = Date()
        let model = EnvironmentModel(name: "Test Scan", creationDate: date, filePathToPointCloudData: "path/to/file.ply")

        #expect(model.name == "Test Scan")
        #expect(model.creationDate == date)
        #expect(model.filePathToPointCloudData == "path/to/file.ply")
        #expect(model.id != nil)
    }

    @Test("Depth Anything V2 model loads and produces a depth map")
    func testDepthAnythingProcessorInference() throws {
        let processor = DepthAnythingProcessor()
        try #require(processor != nil, "DepthAnythingV2SmallF16 model failed to load. Ensure the .mlpackage is in the app bundle.")

        // Create a synthetic 640x480 BGRA image filled with a vertical gradient
        let width = 640
        let height = 480
        var pixelBuffer: CVPixelBuffer?
        let attrs: [String: Any] = [
            kCVPixelBufferCGImageCompatibilityKey as String: true,
            kCVPixelBufferCGBitmapContextCompatibilityKey as String: true
        ]
        let status = CVPixelBufferCreate(
            kCFAllocatorDefault, width, height,
            kCVPixelFormatType_32BGRA,
            attrs as CFDictionary, &pixelBuffer
        )
        try #require(status == kCVReturnSuccess && pixelBuffer != nil)
        let buffer = pixelBuffer!

        CVPixelBufferLockBaseAddress(buffer, [])
        let bytesPerRow = CVPixelBufferGetBytesPerRow(buffer)
        if let base = CVPixelBufferGetBaseAddress(buffer)?.assumingMemoryBound(to: UInt8.self) {
            for y in 0..<height {
                let v = UInt8(Float(y) / Float(height - 1) * 255.0)
                for x in 0..<width {
                    let i = y * bytesPerRow + x * 4
                    base[i]     = v        // B
                    base[i + 1] = v        // G
                    base[i + 2] = 255 - v  // R
                    base[i + 3] = 255      // A
                }
            }
        }
        CVPixelBufferUnlockBaseAddress(buffer, [])

        let depthMap = processor!.inferRelativeDepth(from: buffer)
        try #require(depthMap != nil, "inferRelativeDepth returned nil")

        let map = depthMap!
        #expect(map.width > 0)
        #expect(map.height > 0)
        #expect(map.data.count == map.width * map.height)

        // Verify the output has variation (not all zeros or a constant)
        let firstValue = map.data[0]
        let hasVariation = map.data.contains { abs($0 - firstValue) > 0.001 }
        #expect(hasVariation, "Depth map is uniform — model may not be inferring properly")

        // Verify all values are finite
        let allFinite = map.data.allSatisfy { $0.isFinite }
        #expect(allFinite, "Depth map contains non-finite values")
    }

    @Test("Affine transform correctly maps relative depth to metric")
    func testRelativeDepthAffineTransform() {
        var map = RelativeDepthMap(width: 2, height: 2, data: [0.0, 0.25, 0.5, 1.0])
        map.applyAffine(a: 4.0, b: 1.0)
        // metric = 4 * relative + 1
        #expect(map.data[0] == 1.0)
        #expect(map.data[1] == 2.0)
        #expect(map.data[2] == 3.0)
        #expect(map.data[3] == 5.0)
    }

    @Test("RadioObservation schema covers every radio telemetry channel")
    func testRadioObservationSchemaDefinition() {
        let schema = RadioObservationMessageSchema.shared
        let fieldNames = Set(schema.fields.map(\.name))
        let catalogChannelIDs = RadioTelemetryChannelID.allCases.map(\.rawValue).sorted()

        #expect(schema.messageType == "reconstructor_msgs/msg/RadioObservation")
        #expect(schema.topic == "/mapping/radio")
        #expect(schema.schemaVersion == 1)
        #expect(schema.unsetNumericValue == "0.0")
        #expect(schema.supportedChannelIDs == catalogChannelIDs)
        #expect(schema.messageDefinition.contains("std_msgs/Header header"))
        #expect(schema.messageDefinition.contains("geometry_msgs/Point map_position"))
        #expect(fieldNames.contains("channel_id"))
        #expect(fieldNames.contains("rssi_dbm"))
        #expect(fieldNames.contains("metadata_json"))
    }

    @Test("RadioObservation messages sanitize non-finite values for rosbridge JSON")
    func testRadioObservationMessageIsJSONEncodable() throws {
        let observation = RadioObservationMessage(
            timestamp: Date(timeIntervalSince1970: 1_700_000_000),
            sessionID: "session-1",
            channelID: .bleAdvertisement,
            observationKind: "ble_advertisement",
            sourceAPI: "CoreBluetooth.CBCentralManager",
            sourceID: "peripheral-1",
            radioType: "ble",
            metadata: [
                "bad_number": Double.nan
            ],
            values: [
                "rssi_dbm": Double.nan
            ]
        )

        #expect(observation.fields["rssi_dbm"] as? Double == 0.0)
        #expect((observation.fields["metadata_json"] as? String)?.contains("\"bad_number\":0") == true)

        let payload: [String: Any] = [
            "op": "publish",
            "topic": RadioObservationMessageSchema.shared.topic,
            "msg": observation.fields
        ]
        #expect(JSONSerialization.isValidJSONObject(payload))
        _ = try JSONSerialization.data(withJSONObject: payload, options: [])
    }

    @Test("Radio telemetry catalog documents iOS platform restrictions")
    func testRadioTelemetryPlatformRestrictions() {
        let restrictions = RadioTelemetryCatalog.shared.platformRestrictions
        let restrictionIDs = Set(restrictions.map(\.id))
        let message = RadioTelemetryCatalog.shared.platformRestrictionsMessage

        #expect(restrictionIDs.contains("ios_no_broad_wifi_scans"))
        #expect(restrictionIDs.contains("ios_no_reliable_public_cellular_rf_metrics"))
        #expect(restrictions.contains { restriction in
            restriction.affectedChannelIDs.contains(.currentWiFiNetwork)
                && restriction.summary.localizedCaseInsensitiveContains("scan")
        })
        #expect(restrictions.contains { restriction in
            restriction.affectedChannelIDs.contains(.externalAdapter)
                && restriction.operatorGuidance.localizedCaseInsensitiveContains("cellular")
        })
        #expect(JSONSerialization.isValidJSONObject(message))
    }

    @Test("Mesh publishing advertises MarkerArray fallback and structured MeshSnapshot")
    func testMeshPublishingTopicsAndSchema() throws {
        let registry = ROS2TopicRegistry(enabledStreams: [.mesh])
        let markerTopic = registry.definition(.meshMarkers)
        let snapshotTopic = registry.definition(.meshSnapshot)
        let advertisedIDs = Set(registry.advertisedTopics().map(\.id))
        let schema = MeshSnapshotMessageSchema.shared

        #expect(markerTopic.topic == "/mapping/map")
        #expect(markerTopic.messageType == "visualization_msgs/msg/MarkerArray")
        #expect(snapshotTopic.topic == schema.topic)
        #expect(snapshotTopic.messageType == schema.messageType)
        #expect(advertisedIDs.contains(.meshMarkers))
        #expect(advertisedIDs.contains(.meshSnapshot))
        #expect(schema.messageDefinition.contains("geometry_msgs/Point[] vertices"))
        #expect(schema.messageDefinition.contains("uint32[] triangle_indices"))
        #expect(schema.fields.contains { $0.name == "published_payload_bytes" })
        #expect(JSONSerialization.isValidJSONObject(schema.rosMessage))
        _ = try JSONSerialization.data(withJSONObject: schema.rosMessage, options: [])
    }

    @Test("Large MeshSnapshot messages are triangle-aligned and payload limited")
    func testMeshSnapshotBuilderFitsLargePayloads() throws {
        var trianglePoints: [[String: Float]] = []
        trianglePoints.reserveCapacity(9_000)
        for index in 0..<9_000 {
            trianglePoints.append([
                "x": Float(index) * 0.01,
                "y": Float(index % 97) * 0.02,
                "z": Float(index % 31) * 0.03
            ])
        }
        let header: [String: Any] = [
            "stamp": ["sec": 1, "nanosec": 2],
            "frame_id": "map"
        ]
        let maxPayloadBytes = 16_000
        let message = MeshSnapshotMessageBuilder.makeTriangleListMessage(
            header: header,
            snapshotID: "stress-snapshot",
            source: "unit_test",
            frameID: "map",
            anchorCount: 8,
            trianglePoints: trianglePoints,
            maxPayloadBytes: maxPayloadBytes,
            metadata: ["test": "large_mesh_snapshot"]
        )

        let vertices = try #require(message["vertices"] as? [[String: Double]])
        let indices = try #require(message["triangle_indices"] as? [Int])
        let encodedBytes = try #require(
            MeshSnapshotMessageBuilder.encodedPublishPayloadByteCount(
                topic: MeshSnapshotMessageSchema.shared.topic,
                msg: message
            )
        )

        #expect(message["is_truncated"] as? Bool == true)
        #expect(vertices.count % 3 == 0)
        #expect(indices.count == vertices.count)
        #expect(message["original_vertex_count"] as? Int == trianglePoints.count)
        #expect((message["published_payload_bytes"] as? Int ?? 0) <= maxPayloadBytes)
        #expect(encodedBytes <= maxPayloadBytes)
        #expect(JSONSerialization.isValidJSONObject(message))
        _ = try JSONSerialization.data(withJSONObject: message, options: [])
    }

    @Test("Payload metrics cover compressed camera and long-running point-cloud sessions")
    func testStreamPayloadMetricsAccumulateLongRunningSessions() {
        var camera = StreamPayloadMetricAccumulator(streamID: MappingSensorStream.camera.rawValue)
        camera.record(
            originalBytes: 1_228_800,
            encodedBytes: 92_000,
            compression: "jpeg_q0.4_base64",
            recordedAt: Date(timeIntervalSince1970: 10)
        )

        #expect(camera.snapshot.messageCount == 1)
        #expect(camera.snapshot.compressionRatio < 1.0)
        #expect(camera.snapshot.lastCompression == "jpeg_q0.4_base64")

        var pointCloud = StreamPayloadMetricAccumulator(streamID: MappingSensorStream.pointCloud.rawValue)
        for index in 0..<10_000 {
            pointCloud.record(
                originalBytes: 16_000,
                encodedBytes: 21_336,
                compression: "pointcloud2_binary_base64",
                recordedAt: Date(timeIntervalSince1970: TimeInterval(index))
            )
        }

        let snapshot = pointCloud.snapshot
        #expect(snapshot.messageCount == 10_000)
        #expect(snapshot.originalBytesTotal == 160_000_000)
        #expect(snapshot.encodedBytesTotal == 213_360_000)
        #expect(snapshot.maxEncodedBytes == 21_336)
        #expect(snapshot.compressionRatio > 1.0)
        #expect(JSONSerialization.isValidJSONObject(snapshot.rosMessage))
    }

    @Test("Adaptive mapping policy prefers RoomPlan for strong indoor semantics")
    func testAdaptiveMappingPolicyPrefersRoomPlanIndoors() {
        let policy = AdaptiveMappingModePolicy()
        let recommendation = policy.recommendation(
            for: AdaptiveMappingModeInput(
                roomPlanAvailable: true,
                roomPlanObjectCount: 8,
                indoorRegistrationQuality: 0.92,
                globalRegistrationQuality: 0.25,
                gpsHorizontalAccuracyMeters: 42,
                lidarDepthConfidence: 0.7,
                depthAnythingAvailable: true
            )
        )

        #expect(recommendation.mode == .roomPlanParametric)
        #expect(recommendation.roomPlanScore > recommendation.outdoorScore)
        #expect(recommendation.reasons.contains(.roomPlanSemanticsStrong))
        #expect(recommendation.reasons.contains(.indoorRegistrationStrong))
        #expect(recommendation.metadata["active_mapping_mode"] == AdaptiveMappingMode.roomPlanParametric.rawValue)
    }

    @Test("Adaptive mapping policy prefers LiDAR Depth Anything for outdoor context")
    func testAdaptiveMappingPolicyPrefersOutdoorDepthAnything() {
        let policy = AdaptiveMappingModePolicy()
        let recommendation = policy.recommendation(
            for: AdaptiveMappingModeInput(
                roomPlanAvailable: true,
                roomPlanObjectCount: 0,
                indoorRegistrationQuality: 0.1,
                globalRegistrationQuality: 0.94,
                gpsHorizontalAccuracyMeters: 4,
                lidarDepthConfidence: 0.88,
                depthAnythingAvailable: true
            )
        )

        #expect(recommendation.mode == .lidarDepthAnythingOutdoor)
        #expect(recommendation.outdoorScore > recommendation.roomPlanScore)
        #expect(recommendation.reasons.contains(.outdoorGPSStrong))
        #expect(recommendation.reasons.contains(.depthAnythingAvailable))
    }

    @Test("Adaptive mapping policy honors operator override")
    func testAdaptiveMappingPolicyHonorsOperatorOverride() {
        let policy = AdaptiveMappingModePolicy()
        let recommendation = policy.recommendation(
            for: AdaptiveMappingModeInput(
                roomPlanAvailable: true,
                roomPlanObjectCount: 10,
                indoorRegistrationQuality: 1.0,
                globalRegistrationQuality: 0.1,
                gpsHorizontalAccuracyMeters: 80,
                lidarDepthConfidence: 0.3,
                depthAnythingAvailable: false,
                operatorOverride: .forceLiDARDepthAnything
            )
        )

        #expect(recommendation.mode == .lidarDepthAnythingOutdoor)
        #expect(recommendation.confidence == 1.0)
        #expect(recommendation.reasons.first == .operatorForcedLiDARDepthAnything)
        #expect(recommendation.metadata["adaptive_mapping_operator_override"] == AdaptiveMappingOperatorOverride.forceLiDARDepthAnything.rawValue)
    }

    @Test("Adaptive mapping controller switches active capture mode from policy input")
    @MainActor
    func testAdaptiveMappingControllerSwitchesActiveCaptureMode() throws {
        let suiteName = "AdaptiveMappingControllerTests-\(UUID().uuidString)"
        let defaults = try #require(UserDefaults(suiteName: suiteName))
        defer { defaults.removePersistentDomain(forName: suiteName) }

        let controller = AdaptiveMappingModeController(
            userDefaults: defaults,
            publishesSessionUpdates: false,
            roomPlanCaptureSupported: true
        )

        controller.update(
            input: AdaptiveMappingModeInput(
                roomPlanAvailable: true,
                roomPlanObjectCount: 9,
                indoorRegistrationQuality: 0.95,
                globalRegistrationQuality: 0.2,
                gpsHorizontalAccuracyMeters: 55,
                lidarDepthConfidence: 0.65,
                depthAnythingAvailable: true
            )
        )
        #expect(controller.activeMode == .roomPlanParametric)
        #expect(controller.usesRoomPlanCapture)

        controller.update(
            input: AdaptiveMappingModeInput(
                roomPlanAvailable: true,
                roomPlanObjectCount: 0,
                indoorRegistrationQuality: 0.05,
                globalRegistrationQuality: 0.96,
                gpsHorizontalAccuracyMeters: 4,
                lidarDepthConfidence: 0.92,
                depthAnythingAvailable: true
            )
        )
        #expect(controller.activeMode == .lidarDepthAnythingOutdoor)
        #expect(!controller.usesRoomPlanCapture)
    }

    @Test("Adaptive mapping controller persists override and emits ROS metadata")
    @MainActor
    func testAdaptiveMappingControllerPersistsOverrideAndMetadata() throws {
        let suiteName = "AdaptiveMappingOverrideTests-\(UUID().uuidString)"
        let defaults = try #require(UserDefaults(suiteName: suiteName))
        defer { defaults.removePersistentDomain(forName: suiteName) }

        let controller = AdaptiveMappingModeController(
            userDefaults: defaults,
            publishesSessionUpdates: false,
            roomPlanCaptureSupported: true
        )
        controller.setOperatorOverride(.forceLiDARDepthAnything)
        controller.update(
            input: AdaptiveMappingModeInput(
                roomPlanAvailable: true,
                roomPlanObjectCount: 12,
                indoorRegistrationQuality: 1.0,
                globalRegistrationQuality: 0.1,
                gpsHorizontalAccuracyMeters: 80,
                lidarDepthConfidence: 0.3,
                depthAnythingAvailable: false
            )
        )

        #expect(defaults.string(forKey: AdaptiveMappingModeController.overrideStorageKey) == AdaptiveMappingOperatorOverride.forceLiDARDepthAnything.rawValue)
        #expect(controller.activeMode == .lidarDepthAnythingOutdoor)
        #expect(controller.diagnosticValues["adaptive_mapping_operator_override"] == AdaptiveMappingOperatorOverride.forceLiDARDepthAnything.rawValue)

        let metadata = controller.sessionMetadata
        #expect(metadata["active_mapping_mode"] as? String == AdaptiveMappingMode.lidarDepthAnythingOutdoor.rawValue)
        #expect(metadata["operator_override"] as? String == AdaptiveMappingOperatorOverride.forceLiDARDepthAnything.rawValue)
        #expect((metadata["reason_codes"] as? [String])?.contains(AdaptiveMappingModeReason.operatorForcedLiDARDepthAnything.rawValue) == true)
        #expect(JSONSerialization.isValidJSONObject(metadata))
        _ = try JSONSerialization.data(withJSONObject: metadata, options: [])
    }

    @Test("Default geo tile providers expose source policy metadata")
    func testGeoTileProviderSourcePolicyMetadata() throws {
        let satellite = GeoTileProvider.defaultSatellite
        let usgs = GeoTileProvider.usgs3DEPDEM
        let mapzen = GeoTileProvider.mapzenTerrainTiles

        #expect(satellite.sourcePolicy.recordableByDefault)
        #expect(!satellite.sourcePolicy.transientCacheOnly)
        #expect(satellite.sourcePolicy.credentialRequirement == .none)
        #expect(!satellite.sourcePolicy.requiresCredentials)
        #expect(satellite.sourcePolicy.attributionURL.contains("earthdata"))

        #expect(usgs.sourcePolicy.recordableByDefault)
        #expect(!usgs.sourcePolicy.transientCacheOnly)
        #expect(usgs.sourcePolicy.credentialRequirement == .none)
        #expect(!usgs.sourcePolicy.requiresCredentials)
        #expect(usgs.sourcePolicy.attributionURL.contains("usgs"))

        #expect(mapzen.sourcePolicy.recordableByDefault)
        #expect(!mapzen.sourcePolicy.transientCacheOnly)
        #expect(mapzen.sourcePolicy.credentialRequirement == .none)
        #expect(!mapzen.sourcePolicy.requiresCredentials)
        #expect(mapzen.sourcePolicy.attributionURL.contains("joerd"))

        let payload: [String: Any] = [
            "satellite": satellite.sourcePolicy.rosMessage,
            "usgs": usgs.sourcePolicy.rosMessage,
            "mapzen": mapzen.sourcePolicy.rosMessage
        ]
        #expect(JSONSerialization.isValidJSONObject(payload))
        _ = try JSONSerialization.data(withJSONObject: payload, options: [])
    }

    @Test("USGS 3DEP is the preferred US DEM provider with Mapzen fallback")
    func testDEMProviderSelection() throws {
        let denver = CLLocation(latitude: 39.7392, longitude: -104.9903)
        let london = CLLocation(latitude: 51.5074, longitude: -0.1278)
        let providers = [
            GeoTileProvider.defaultSatellite,
            GeoTileProvider.usgs3DEPDEM,
            GeoTileProvider.mapzenTerrainTiles
        ]

        let usCandidates = GeoTileProviderSelection.demCandidates(for: denver, providers: providers)
        #expect(usCandidates.map { $0.name } == ["USGS 3DEP", "Mapzen Terrain Tiles"])

        let globalCandidates = GeoTileProviderSelection.demCandidates(for: london, providers: providers)
        #expect(globalCandidates.map { $0.name } == ["Mapzen Terrain Tiles"])

        let coordinate = GeoTileCoordinate.webMercator(
            latitude: denver.coordinate.latitude,
            longitude: denver.coordinate.longitude,
            zoom: GeoTileProvider.usgs3DEPDEM.zoom
        )
        let url = try #require(GeoTileProvider.usgs3DEPDEM.makeURL(coordinate, nil))
        let urlString = url.absoluteString

        #expect(urlString.contains("3DEPElevation/ImageServer/exportImage"))
        #expect(urlString.contains("bboxSR=3857"))
        #expect(urlString.contains("imageSR=3857"))
        #expect(urlString.contains("format=tiff"))
        #expect(urlString.contains("pixelType=F32"))
        #expect(urlString.contains("f=image"))
    }

    @Test("Optional geo provider configuration slots do not expose secret values")
    func testOptionalGeoProviderConfigurationSlots() throws {
        let suiteName = "GeoTileProviderConfigurationTests-\(UUID().uuidString)"
        let defaults = try #require(UserDefaults(suiteName: suiteName))
        defer { defaults.removePersistentDomain(forName: suiteName) }

        defaults.set(true, forKey: GeoTileProviderConfigurationStore.key("enabled", .copernicusDataSpace))
        defaults.set("keychain://copernicus-token", forKey: GeoTileProviderConfigurationStore.key("credentialReference", .copernicusDataSpace))
        defaults.set(true, forKey: GeoTileProviderConfigurationStore.key("hasCredentialMaterial", .copernicusDataSpace))
        defaults.set(false, forKey: GeoTileProviderConfigurationStore.key("recordingAllowed", .copernicusDataSpace))
        defaults.set("https://example.invalid/copernicus", forKey: GeoTileProviderConfigurationStore.key("endpointURL", .copernicusDataSpace))
        defaults.set("Copernicus attribution", forKey: GeoTileProviderConfigurationStore.key("attributionOverride", .copernicusDataSpace))

        defaults.set(true, forKey: GeoTileProviderConfigurationStore.key("enabled", .openTopography))
        defaults.set(true, forKey: GeoTileProviderConfigurationStore.key("hasCredentialMaterial", .openTopography))
        defaults.set(true, forKey: GeoTileProviderConfigurationStore.key("recordingAllowed", .openTopography))

        let configurations = GeoTileProviderConfigurationStore.load(from: defaults)
        let copernicus = try #require(configurations.first { $0.id == .copernicusDataSpace })
        let openTopography = try #require(configurations.first { $0.id == .openTopography })
        let commercialImagery = try #require(configurations.first { $0.id == .commercialImagery })

        #expect(configurations.count == GeoTileOptionalProviderID.allCases.count)
        #expect(copernicus.id.credentialRequirement == .userLogin)
        #expect(copernicus.statusLabel == "recording_not_allowed")
        #expect(!copernicus.isConfigured)
        #expect((copernicus.rosMessage["credential_reference"] as? String) == "keychain://copernicus-token")
        #expect(copernicus.rosMessage["has_credential_material"] as? Bool == true)
        #expect(copernicus.rosMessage["credential_value"] == nil)

        #expect(openTopography.id.credentialRequirement == .userAPIKey)
        #expect(openTopography.isConfigured)
        #expect(openTopography.endpointURL == GeoTileOptionalProviderID.openTopography.defaultEndpointURL)

        #expect(commercialImagery.id.credentialRequirement == .commercialAccount)
        #expect(commercialImagery.statusLabel == "disabled")

        let payload: [String: Any] = [
            "optional_geo_provider_configurations": configurations.map(\.rosMessage)
        ]
        #expect(JSONSerialization.isValidJSONObject(payload))
        _ = try JSONSerialization.data(withJSONObject: payload, options: [])
    }

    @Test("Implemented ROS2 topics serialize advertised schema metadata")
    func testROS2TopicRegistryAdvertisedTopicsSerialize() throws {
        let registry = ROS2TopicRegistry()
        let allTopics = registry.allTopics()
        let advertisedTopics = registry.advertisedTopics()
        let advertisedIDs = Set(advertisedTopics.map(\.id))

        #expect(allTopics.count == ROS2TopicID.allCases.count)
        #expect(Set(allTopics.map(\.id)) == Set(ROS2TopicID.allCases))
        #expect(advertisedIDs.isSuperset(of: [.pose, .cameraCompressed, .cameraInfo, .pointCloud, .gpsFix, .gpsMetadata, .satelliteImage, .satelliteTileInfo, .demTile]))
        #expect(allTopics.allSatisfy { $0.topic == "/tf" || $0.topic.hasPrefix("/mapping/") })
        #expect(!advertisedIDs.contains(.odom))
        #expect(!advertisedIDs.contains(.surfels))
        #expect(!advertisedIDs.contains(.imu))
        #expect(!advertisedIDs.contains(.meshMarkers))
        #expect(!advertisedIDs.contains(.meshSnapshot))

        let advertisedPayload = advertisedTopics.map { definition in
            [
                "id": definition.id.rawValue,
                "stream": definition.stream.rawValue,
                "topic": definition.topic,
                "message_type": definition.messageType,
                "default_rate_hz": definition.defaultRateHz.map { $0 as Any } ?? NSNull(),
                "is_implemented": definition.isImplemented
            ] as [String: Any]
        }
        let payload: [String: Any] = ["advertised_topics": advertisedPayload]

        for definition in advertisedTopics {
            #expect(definition.topic.hasPrefix("/"))
            #expect(definition.messageType.contains("/msg/"))
            if let defaultRateHz = definition.defaultRateHz {
                #expect(defaultRateHz >= 0)
            }
        }

        #expect(JSONSerialization.isValidJSONObject(payload))
        _ = try JSONSerialization.data(withJSONObject: payload, options: [])
    }

    @Test("CameraInfo payload exposes loop-closure intrinsics")
    func testCameraInfoPayloadExposesLoopClosureIntrinsics() throws {
        let registry = ROS2TopicRegistry()
        let cameraInfoTopic = registry.definition(.cameraInfo)
        let header: [String: Any] = [
            "stamp": ["sec": 10, "nanosec": 20],
            "frame_id": "iphone_camera"
        ]
        let intrinsics = simd_float3x3(
            SIMD3<Float>(600, 0, 0),
            SIMD3<Float>(0, 590, 0),
            SIMD3<Float>(320, 240, 1)
        )

        let msg = ROS2BridgeClient.makeCameraInfoMessage(
            header: header,
            intrinsics: intrinsics,
            imageResolution: CGSize(width: 640, height: 480)
        )

        #expect(cameraInfoTopic.topic == "/mapping/camera/camera_info")
        #expect(cameraInfoTopic.messageType == "sensor_msgs/msg/CameraInfo")
        #expect(msg["width"] as? Int == 640)
        #expect(msg["height"] as? Int == 480)
        #expect(msg["distortion_model"] as? String == "plumb_bob")
        #expect(msg["d"] as? [Double] == [0.0, 0.0, 0.0, 0.0, 0.0])
        #expect(msg["k"] as? [Double] == [
            600.0, 0.0, 320.0,
            0.0, 590.0, 240.0,
            0.0, 0.0, 1.0
        ])
        #expect(msg["p"] as? [Double] == [
            600.0, 0.0, 320.0, 0.0,
            0.0, 590.0, 240.0, 0.0,
            0.0, 0.0, 1.0, 0.0
        ])
        #expect(JSONSerialization.isValidJSONObject(msg))
        _ = try JSONSerialization.data(withJSONObject: msg, options: [])
    }

    @Test("Surfel cloud payload uses PointCloud2 fields for colored surface reconstruction")
    func testSurfelCloudPayloadUsesPointCloud2Fields() throws {
        let registry = ROS2TopicRegistry()
        let surfelTopic = registry.definition(.surfels)
        let header: [String: Any] = [
            "stamp": ["sec": 10, "nanosec": 20],
            "frame_id": "map"
        ]
        let surfels = [
            ColoredSurfel(
                position: SIMD3<Float>(1, 2, 3),
                normal: SIMD3<Float>(0, 1, 0),
                color: SIMD3<UInt8>(10, 20, 30),
                radius: 0.04,
                confidence: 0.9,
                observationCount: 4
            )
        ]

        let msg = ROS2BridgeClient.makeSurfelPointCloudMessage(
            surfels: surfels,
            header: header
        )
        let fields = try #require(msg["fields"] as? [[String: Any]])
        let fieldNames = Set(fields.compactMap { $0["name"] as? String })
        let encodedData = try #require(msg["data"] as? String)
        let decodedData = try #require(Data(base64Encoded: encodedData))

        #expect(surfelTopic.topic == "/mapping/surfels")
        #expect(surfelTopic.stream == .surfels)
        #expect(surfelTopic.messageType == "sensor_msgs/msg/PointCloud2")
        #expect(!surfelTopic.isImplemented)
        #expect(msg["point_step"] as? Int == 40)
        #expect(msg["row_step"] as? Int == 40)
        #expect(msg["width"] as? Int == 1)
        #expect(decodedData.count == 40)
        #expect(fieldNames.isSuperset(of: [
            "x", "y", "z",
            "normal_x", "normal_y", "normal_z",
            "radius", "confidence", "rgb", "observation_count"
        ]))
        #expect(JSONSerialization.isValidJSONObject(msg))
        _ = try JSONSerialization.data(withJSONObject: msg, options: [])
    }

    @Test("GPS fixes convert to ENU and map-frame coordinates")
    func testGPSLocationConvertsToENUMeters() throws {
        let georeferencer = MapGeoreferencer(maximumOriginHorizontalAccuracy: 10)
        var transform = matrix_identity_float4x4
        transform.columns.3 = SIMD4<Float>(10, 2, -3, 1)
        georeferencer.updateMapPose(transform, timestamp: 100)

        let originLatitude = 37.3349
        let originLongitude = -122.0090
        let originLocation = CLLocation(
            coordinate: CLLocationCoordinate2D(latitude: originLatitude, longitude: originLongitude),
            altitude: 20,
            horizontalAccuracy: 3,
            verticalAccuracy: 4,
            timestamp: Date(timeIntervalSince1970: 1_700_000_000)
        )

        let originSnapshot = try #require(georeferencer.snapshot(for: originLocation))
        #expect(abs(originSnapshot.enuMeters.x) < 0.01)
        #expect(abs(originSnapshot.enuMeters.y) < 0.01)
        #expect(abs(originSnapshot.enuMeters.z) < 0.01)

        let eastMeters = 5.0
        let northMeters = 12.0
        let upMeters = 3.0
        let targetCoordinate = offsetCoordinate(
            latitude: originLatitude,
            longitude: originLongitude,
            eastMeters: eastMeters,
            northMeters: northMeters
        )
        let targetLocation = CLLocation(
            coordinate: targetCoordinate,
            altitude: originLocation.altitude + upMeters,
            horizontalAccuracy: 3,
            verticalAccuracy: 4,
            timestamp: Date(timeIntervalSince1970: 1_700_000_010)
        )

        let snapshot = try #require(georeferencer.snapshot(for: targetLocation))
        #expect(abs(snapshot.enuMeters.x - eastMeters) < 0.5)
        #expect(abs(snapshot.enuMeters.y - northMeters) < 0.5)
        #expect(abs(snapshot.enuMeters.z - upMeters) < 0.1)
        #expect(abs(snapshot.mapPositionMeters.x - (10 + eastMeters)) < 0.5)
        #expect(abs(snapshot.mapPositionMeters.y - (2 + upMeters)) < 0.1)
        #expect(abs(snapshot.mapPositionMeters.z - (-3 - northMeters)) < 0.5)
        #expect(JSONSerialization.isValidJSONObject(snapshot.rosMessage))
        _ = try JSONSerialization.data(withJSONObject: snapshot.rosMessage, options: [])
    }

    @Test("GPS origin is gated by horizontal accuracy")
    func testGPSOriginRequiresAccurateFix() {
        let georeferencer = MapGeoreferencer(maximumOriginHorizontalAccuracy: 5)
        var transform = matrix_identity_float4x4
        transform.columns.3 = SIMD4<Float>(0, 0, 0, 1)
        georeferencer.updateMapPose(transform, timestamp: 1)

        let poorFix = CLLocation(
            coordinate: CLLocationCoordinate2D(latitude: 37.3349, longitude: -122.0090),
            altitude: 20,
            horizontalAccuracy: 50,
            verticalAccuracy: 4,
            timestamp: Date(timeIntervalSince1970: 1_700_000_000)
        )

        #expect(georeferencer.snapshot(for: poorFix) == nil)
        #expect(georeferencer.unavailableMessage["reason"] as? String == "waiting_for_accurate_gps_origin")
    }

    @Test("Geo tile metadata and cache indexing stay stable")
    func testGeoTileMetadataAndCacheIndexing() throws {
        let provider = GeoTileProvider.usgs3DEPDEM
        let location = CLLocation(
            coordinate: CLLocationCoordinate2D(latitude: 39.7392, longitude: -104.9903),
            altitude: 1609,
            horizontalAccuracy: 5,
            verticalAccuracy: 8,
            timestamp: Date(timeIntervalSince1970: 1_700_000_030)
        )
        let coordinate = GeoTileCoordinate.webMercator(
            latitude: location.coordinate.latitude,
            longitude: location.coordinate.longitude,
            zoom: provider.zoom
        )
        let bounds = GeoTileBounds.webMercatorBounds(for: coordinate)
        let pixel = GeoTilePixelCoordinate.webMercator(
            latitude: location.coordinate.latitude,
            longitude: location.coordinate.longitude,
            coordinate: coordinate,
            tileSizePixels: provider.tileSizePixels
        )
        let deviceLocation = GeoTileDeviceLocation(
            latitude: location.coordinate.latitude,
            longitude: location.coordinate.longitude,
            altitude: location.altitude,
            horizontalAccuracy: location.horizontalAccuracy,
            verticalAccuracy: location.verticalAccuracy,
            timestamp: location.timestamp,
            pixel: pixel
        )
        let cache = GeoTileCache()
        let time = "2026-06-22"
        let cachePath = cache.relativePath(provider: provider, coordinate: coordinate, time: time)
        let payload = GeoTilePayload(
            provider: provider,
            coordinate: coordinate,
            bounds: bounds,
            deviceLocation: deviceLocation,
            time: time,
            data: Data([1, 2, 3, 4]),
            sourceURL: try #require(provider.makeURL(coordinate, time)),
            fetchedAt: Date(timeIntervalSince1970: 1_700_000_040),
            isCached: true
        )
        let model = GeoTileModel(payload: payload, cachePath: cachePath)
        let tileInfoMessage = ROS2BridgeClient().makeGeoTileInfoMessage(
            tile: payload,
            header: ["stamp": ["sec": 10, "nanosec": 20], "frame_id": "earth"]
        )

        #expect(cachePath.hasPrefix("USGS_3DEP/3DEPElevation/\(time)/\(coordinate.z)/"))
        #expect(cachePath.hasSuffix("/\(coordinate.y).tif"))
        #expect(!cachePath.contains(" "))
        #expect(model.id == GeoTileModel.id(provider: provider, coordinate: coordinate, time: time))
        #expect(model.cachePath == cachePath)
        #expect(model.providerName == "USGS 3DEP")
        #expect(model.kind == GeoTileLayerKind.dem.rawValue)
        #expect(model.sourcePolicyJSON.contains("\"recordable_by_default\":true"))
        #expect(pixel.x >= 0 && pixel.x <= Double(provider.tileSizePixels))
        #expect(pixel.y >= 0 && pixel.y <= Double(provider.tileSizePixels))
        #expect(tileInfoMessage["device_pixel_x"] as? Double == pixel.x)
        #expect(tileInfoMessage["device_pixel_y"] as? Double == pixel.y)
        #expect(tileInfoMessage["tile_width"] as? Int == pixel.width)
        #expect(tileInfoMessage["tile_height"] as? Int == pixel.height)
        #expect(tileInfoMessage["pixel_origin"] as? String == "upper_left")
        #expect(tileInfoMessage["pixel_units"] as? String == "pixels")
        #expect(bounds.west < location.coordinate.longitude)
        #expect(bounds.east > location.coordinate.longitude)
        #expect(bounds.south < location.coordinate.latitude)
        #expect(bounds.north > location.coordinate.latitude)
        #expect(JSONSerialization.isValidJSONObject(tileInfoMessage))
        _ = try JSONSerialization.data(withJSONObject: tileInfoMessage, options: [])
    }

    @Test("Publish queue drops oldest publish when capacity is exceeded")
    func testPublishQueueBackpressureDropsOldestPublish() async throws {
        let stats = PublishQueueStatsRecorder()
        let sends = PublishQueueSendRecorder()
        let queue = PublishQueue(
            configuration: PublishQueue.Configuration(
                capacity: 2,
                maxRetries: 0,
                retryDelayMilliseconds: 1,
                dropPolicy: .dropOldestPublish
            )
        ) { data, completion in
            sends.record(data: data, completion: completion)
        }
        queue.onStatsChange = stats.record

        queue.enqueueEncodedPayload(Data("first".utf8), op: "publish", topic: "/test/first")
        queue.enqueueEncodedPayload(Data("second".utf8), op: "publish", topic: "/test/second")
        queue.enqueueEncodedPayload(Data("third".utf8), op: "publish", topic: "/test/third")
        queue.enqueueEncodedPayload(Data("fourth".utf8), op: "publish", topic: "/test/fourth")

        #expect(await waitUntil { stats.latest?.droppedMessages == 1 && stats.latest?.depth == 2 })
        #expect(stats.latest?.lastError?.contains("/test/second") == true)

        sends.completeNext(with: nil)
        #expect(await waitUntil { sends.sentPayloads.count == 2 })
        sends.completeNext(with: nil)
        #expect(await waitUntil { sends.sentPayloads.count == 3 })
        sends.completeNext(with: nil)
        #expect(await waitUntil { stats.latest?.sentMessages == 3 })

        #expect(sends.sentPayloads == ["first", "third", "fourth"])
        #expect(stats.latest?.failedMessages == 0)
    }

    @Test("Publish queue retries transient publish failures")
    func testPublishQueueRetriesTransientFailures() async throws {
        let stats = PublishQueueStatsRecorder()
        let sends = PublishQueueSendRecorder()
        let queue = PublishQueue(
            configuration: PublishQueue.Configuration(
                capacity: 4,
                maxRetries: 2,
                retryDelayMilliseconds: 1,
                dropPolicy: .dropOldestPublish
            )
        ) { data, completion in
            sends.record(data: data, completion: completion)
        }
        queue.onStatsChange = stats.record

        queue.enqueueEncodedPayload(Data("retry-me".utf8), op: "publish", topic: "/test/retry")
        #expect(await waitUntil { sends.sentPayloads.count == 1 })
        sends.completeNext(with: TestPublishError.temporary)

        #expect(await waitUntil { stats.latest?.retriedMessages == 1 && sends.sentPayloads.count == 2 })
        sends.completeNext(with: nil)

        #expect(await waitUntil { stats.latest?.sentMessages == 1 })
        #expect(sends.sentPayloads == ["retry-me", "retry-me"])
        #expect(stats.latest?.failedMessages == 0)
        #expect(stats.latest?.lastError == nil)
    }

    @Test("Local ROS2 bag storage is disabled by default")
    func testLocalROS2BagConfigurationDefaultsOff() throws {
        let suiteName = "LocalROS2BagConfigurationTests-\(UUID().uuidString)"
        let defaults = try #require(UserDefaults(suiteName: suiteName))
        defer { defaults.removePersistentDomain(forName: suiteName) }

        let configuration = LocalROS2BagRecorderConfiguration.load(from: defaults)

        #expect(!configuration.isEnabled)
        #expect(configuration.chunkSizeMB == LocalROS2BagRecorderConfiguration.defaultChunkSizeMB)
    }

    @Test("Local ROS2 bag recorder writes chunked SQLite rosbridge JSON bags")
    func testLocalROS2BagRecorderWritesChunkedSQLiteBag() throws {
        let fileManager = FileManager.default
        let rootURL = fileManager.temporaryDirectory
            .appendingPathComponent("MapEverythingLocalBag-\(UUID().uuidString)", isDirectory: true)
        try fileManager.createDirectory(at: rootURL, withIntermediateDirectories: true)
        defer { try? fileManager.removeItem(at: rootURL) }

        let recorder = LocalROS2BagRecorder(fileManager: fileManager, baseDirectoryURL: rootURL)
        let sessionID = try #require(UUID(uuidString: "11111111-2222-3333-4444-555555555555"))
        recorder.start(
            sessionID: sessionID,
            configuration: LocalROS2BagRecorderConfiguration(isEnabled: true, maxChunkBytes: 900)
        )

        let firstMessage = makeLocalBagMessage(sequence: 1, payloadSize: 1_100)
        let secondMessage = makeLocalBagMessage(sequence: 2, payloadSize: 1_100)
        recorder.recordPublishedTopic(
            topic: "/mapping/status",
            messageType: "diagnostic_msgs/msg/DiagnosticArray",
            msg: firstMessage
        )
        recorder.recordPublishedTopic(
            topic: "/mapping/status",
            messageType: "diagnostic_msgs/msg/DiagnosticArray",
            msg: secondMessage
        )
        recorder.stopAndWait()

        let bagDirectories = try fileManager.contentsOfDirectory(
            at: rootURL,
            includingPropertiesForKeys: [.isDirectoryKey]
        ).filter { url in
            (try? url.resourceValues(forKeys: [.isDirectoryKey]).isDirectory) == true
        }
        let bagDirectory = try #require(bagDirectories.first)
        let metadata = try String(
            contentsOf: bagDirectory.appendingPathComponent("metadata.yaml"),
            encoding: .utf8
        )
        let dbFiles = try fileManager.contentsOfDirectory(at: bagDirectory, includingPropertiesForKeys: nil)
            .filter { $0.pathExtension == "db3" }
            .sorted { $0.lastPathComponent < $1.lastPathComponent }
        let sessions = try recorder.listBagSessions()
        let listedSession = try #require(sessions.first)

        #expect(dbFiles.count == 2)
        #expect(metadata.contains("storage_identifier: sqlite3"))
        #expect(metadata.contains("serialization_format: 'rosbridge_json'"))
        #expect(metadata.contains("mapeverything_0.db3"))
        #expect(metadata.contains("mapeverything_1.db3"))
        #expect(metadata.contains("Messages are stored as rosbridge publish JSON payloads"))

        let totalMessages = try dbFiles.reduce(0) { count, url in
            count + (try sqliteInteger(url: url, sql: "SELECT COUNT(*) FROM messages"))
        }
        let topicCount = try dbFiles.reduce(0) { count, url in
            count + (try sqliteInteger(url: url, sql: "SELECT COUNT(*) FROM topics WHERE name = '/mapping/status' AND type = 'diagnostic_msgs/msg/DiagnosticArray' AND serialization_format = 'rosbridge_json'"))
        }

        #expect(totalMessages == 2)
        #expect(topicCount == 2)
        #expect(sessions.count == 1)
        #expect(listedSession.chunkCount == 2)
        #expect(listedSession.files.contains { $0.name == "metadata.yaml" && $0.kind == .metadata })
        #expect(listedSession.files.contains { $0.name == "mapeverything_0.db3" && $0.kind == .sqliteChunk })

        try recorder.deleteBagSession(listedSession)
        #expect(try recorder.listBagSessions().isEmpty)
    }

    @Test("Expanded SwiftData schema preserves EnvironmentModel and stores mapping records")
    @MainActor
    func testMappingPersistenceModelsAndEnvironmentMigration() throws {
        let schema = MapEverythingModelSchema.schema
        let configuration = ModelConfiguration(schema: schema, isStoredInMemoryOnly: true)
        let container = try ModelContainer(for: schema, configurations: [configuration])
        let context = ModelContext(container)

        let environment = EnvironmentModel(
            id: UUID(),
            name: "Existing Environment",
            creationDate: Date(timeIntervalSince1970: 1_700_000_000),
            filePathToPointCloudData: "existing_scan.ply",
            meshPath: "existing_mesh.usdz"
        )
        context.insert(environment)

        let sessionID = UUID()
        let snapshot = MappingSessionSnapshot(
            event: "started",
            sessionID: sessionID,
            state: "Active",
            recorderURL: "ws://127.0.0.1:9090",
            enabledStreams: ["pose", "radio"],
            startedAt: Date(timeIntervalSince1970: 1_700_000_010),
            endedAt: nil,
            lastError: nil
        )
        let session = MappingSessionModel(snapshot: snapshot, metadataJSON: "{\"test\":true}")
        session.sessionDirectoryPath = "Sessions/\(sessionID.uuidString)"
        context.insert(session)

        let topic = ROS2TopicRegistry().definition(.radio)
        let stream = SensorStreamModel(topic: topic, isEnabled: true)
        stream.apply(
            stats: PublishQueueStats(
                capacity: 10,
                sentMessages: 7,
                droppedMessages: 1,
                retriedMessages: 2,
                failedMessages: 1,
                lastError: "last radio error",
                lastErrorAt: Date(timeIntervalSince1970: 1_700_000_020)
            )
        )
        context.insert(stream)

        let provider = GeoTileProvider.usgs3DEPDEM
        let coordinate = GeoTileCoordinate(z: 12, x: 818, y: 1583)
        let bounds = GeoTileBounds.webMercatorBounds(for: coordinate)
        let deviceLocation = GeoTileDeviceLocation(
            latitude: 39.7392,
            longitude: -104.9903,
            altitude: 1609,
            horizontalAccuracy: 5,
            verticalAccuracy: 8,
            timestamp: Date(timeIntervalSince1970: 1_700_000_030),
            pixel: GeoTilePixelCoordinate(x: 100, y: 120, width: 256, height: 256)
        )
        let payload = GeoTilePayload(
            provider: provider,
            coordinate: coordinate,
            bounds: bounds,
            deviceLocation: deviceLocation,
            time: nil,
            data: Data([1, 2, 3, 4]),
            sourceURL: try #require(provider.makeURL(coordinate, nil)),
            fetchedAt: Date(timeIntervalSince1970: 1_700_000_040),
            isCached: false
        )
        let cache = GeoTileCache()
        let tile = GeoTileModel(
            payload: payload,
            cachePath: cache.relativePath(provider: provider, coordinate: coordinate, time: nil)
        )
        context.insert(tile)

        try context.save()

        let environments = try context.fetch(FetchDescriptor<EnvironmentModel>())
        let sessions = try context.fetch(FetchDescriptor<MappingSessionModel>())
        let streams = try context.fetch(FetchDescriptor<SensorStreamModel>())
        let tiles = try context.fetch(FetchDescriptor<GeoTileModel>())

        #expect(environments.count == 1)
        #expect(environments.first?.name == "Existing Environment")
        #expect(environments.first?.filePathToPointCloudData == "existing_scan.ply")

        #expect(sessions.count == 1)
        #expect(sessions.first?.sessionID == sessionID)
        #expect(sessions.first?.recorderURL == "ws://127.0.0.1:9090")
        #expect(sessions.first?.enabledStreams == ["pose", "radio"])
        #expect(sessions.first?.providerConfigJSON.hasPrefix("[") == true)

        #expect(streams.count == 1)
        #expect(streams.first?.topic == "/mapping/radio")
        #expect(streams.first?.sentMessages == 7)
        #expect(streams.first?.droppedMessages == 1)
        #expect(streams.first?.lastError == "last radio error")

        #expect(tiles.count == 1)
        #expect(tiles.first?.providerName == "USGS 3DEP")
        #expect(tiles.first?.kind == GeoTileLayerKind.dem.rawValue)
        #expect(tiles.first?.byteCount == 4)
        #expect(tiles.first?.cachePath.contains("USGS_3DEP") == true)
    }

    @Test("Transient radio observation buffer deduplicates and drops oldest observations")
    func testRadioObservationTransientBuffer() {
        var buffer = RadioObservationTransientBuffer(capacity: 2)
        let first = makeRadioObservation(timestamp: 1, sourceID: "first")
        let second = makeRadioObservation(timestamp: 2, sourceID: "second")
        let third = makeRadioObservation(timestamp: 3, sourceID: "third")

        buffer.enqueue(first)
        buffer.enqueue(second)
        buffer.enqueue(first)
        buffer.enqueue(third)

        #expect(buffer.count == 2)

        let flushed = buffer.flush()
        #expect(flushed.map(\.deduplicationKey) == [second.deduplicationKey, third.deduplicationKey])
        #expect(buffer.count == 0)
    }

    @Test("Cleanup removes orphaned point-cloud mesh imagery DEM and session files")
    func testMappingFileCleanupRemovesOrphans() throws {
        let fileManager = FileManager.default
        let root = fileManager.temporaryDirectory
            .appendingPathComponent("MapEverythingCleanup-\(UUID().uuidString)", isDirectory: true)
        try fileManager.createDirectory(at: root, withIntermediateDirectories: true)
        defer { try? fileManager.removeItem(at: root) }

        let referencedPaths: Set<String> = [
            "kept_scan.ply",
            "kept_mesh.usdz",
            "kept_satellite.jpg",
            "kept_dem.tif",
            "kept_session.mcap"
        ]
        let orphanPaths = [
            "orphan_scan.ply",
            "orphan_mesh.obj",
            "orphan_satellite.png",
            "orphan_dem.tiff",
            "orphan_session.db3"
        ]
        let ignoredPath = "operator_notes.txt"

        for path in referencedPaths.union(orphanPaths).union([ignoredPath]) {
            let url = root.appendingPathComponent(path)
            try Data(path.utf8).write(to: url)
        }

        let result = MappingFileCleanupManager.removeOrphanedDocumentFiles(
            in: root,
            referencedPaths: referencedPaths,
            fileManager: fileManager
        )

        #expect(Set(result.removedFiles) == Set(orphanPaths))
        for path in referencedPaths {
            #expect(fileManager.fileExists(atPath: root.appendingPathComponent(path).path))
        }
        for path in orphanPaths {
            #expect(!fileManager.fileExists(atPath: root.appendingPathComponent(path).path))
        }
        #expect(fileManager.fileExists(atPath: root.appendingPathComponent(ignoredPath).path))

        let cacheRoot = root.appendingPathComponent("Cache", isDirectory: true)
        let keptTilePath = "USGS_3DEP/3DEPElevation/static/12/818/1583.tif"
        let orphanTilePath = "GeoTiles/USGS_3DEP/3DEPElevation/static/12/818/1584.tif"
        let keptTileURL = cacheRoot
            .appendingPathComponent("GeoTiles", isDirectory: true)
            .appendingPathComponent(keptTilePath)
        let orphanTileURL = cacheRoot.appendingPathComponent(orphanTilePath)
        try fileManager.createDirectory(at: keptTileURL.deletingLastPathComponent(), withIntermediateDirectories: true)
        try fileManager.createDirectory(at: orphanTileURL.deletingLastPathComponent(), withIntermediateDirectories: true)
        try Data([1]).write(to: keptTileURL)
        try Data([2]).write(to: orphanTileURL)

        let cacheResult = MappingFileCleanupManager.removeOrphanedCacheFiles(
            in: cacheRoot,
            referencedPaths: [keptTilePath],
            fileManager: fileManager
        )

        #expect(cacheResult.removedFiles == [orphanTilePath])
        #expect(fileManager.fileExists(atPath: keptTileURL.path))
        #expect(!fileManager.fileExists(atPath: orphanTileURL.path))
    }

    private func makeRadioObservation(timestamp: TimeInterval, sourceID: String) -> RadioObservationMessage {
        RadioObservationMessage(
            timestamp: Date(timeIntervalSince1970: timestamp),
            sessionID: "session",
            channelID: .networkPath,
            observationKind: "network_path_state",
            sourceAPI: "unit_test",
            sourceID: sourceID,
            radioType: "network_path",
            values: [
                "success": true
            ]
        )
    }

    private func makeLocalBagMessage(sequence: Int, payloadSize: Int) -> [String: Any] {
        [
            "header": [
                "stamp": [
                    "sec": 1_700_000_000 + sequence,
                    "nanosec": sequence
                ],
                "frame_id": "map"
            ],
            "status": [
                [
                    "level": 0,
                    "name": "unit_test/local_bag",
                    "message": String(repeating: "x", count: payloadSize),
                    "hardware_id": "simulator",
                    "values": []
                ]
            ]
        ]
    }

    private func offsetCoordinate(
        latitude: Double,
        longitude: Double,
        eastMeters: Double,
        northMeters: Double
    ) -> CLLocationCoordinate2D {
        let earthRadiusMeters = 6_378_137.0
        let latitudeRadians = latitude * .pi / 180.0
        let deltaLatitude = northMeters / earthRadiusMeters
        let deltaLongitude = eastMeters / (earthRadiusMeters * cos(latitudeRadians))

        return CLLocationCoordinate2D(
            latitude: latitude + deltaLatitude * 180.0 / .pi,
            longitude: longitude + deltaLongitude * 180.0 / .pi
        )
    }

    private func waitUntil(
        timeoutSeconds: TimeInterval = 1,
        condition: @escaping () -> Bool
    ) async -> Bool {
        let deadline = Date().addingTimeInterval(timeoutSeconds)
        while Date() < deadline {
            if condition() {
                return true
            }
            try? await Task.sleep(nanoseconds: 10_000_000)
        }
        return condition()
    }

    private enum TestPublishError: LocalizedError {
        case temporary

        var errorDescription: String? {
            "Temporary publish failure"
        }
    }

    private final class PublishQueueStatsRecorder {
        private let lock = NSLock()
        private var snapshots: [PublishQueueStats] = []

        var latest: PublishQueueStats? {
            lock.lock()
            defer { lock.unlock() }
            return snapshots.last
        }

        func record(_ snapshot: PublishQueueStats) {
            lock.lock()
            snapshots.append(snapshot)
            lock.unlock()
        }
    }

    private final class PublishQueueSendRecorder {
        private let lock = NSLock()
        private var completions: [(Error?) -> Void] = []
        private var payloads: [String] = []

        var sentPayloads: [String] {
            lock.lock()
            defer { lock.unlock() }
            return payloads
        }

        func record(data: Data, completion: @escaping (Error?) -> Void) {
            lock.lock()
            payloads.append(String(data: data, encoding: .utf8) ?? "")
            completions.append(completion)
            lock.unlock()
        }

        func completeNext(with error: Error?) {
            let completion: ((Error?) -> Void)?
            lock.lock()
            completion = completions.isEmpty ? nil : completions.removeFirst()
            lock.unlock()
            completion?(error)
        }
    }

    private func sqliteInteger(url: URL, sql: String) throws -> Int {
        var database: OpaquePointer?
        guard sqlite3_open_v2(url.path, &database, SQLITE_OPEN_READONLY, nil) == SQLITE_OK else {
            throw TestSQLiteError(database: database, fallback: "Unable to open SQLite database")
        }
        defer { sqlite3_close(database) }

        var statement: OpaquePointer?
        guard sqlite3_prepare_v2(database, sql, -1, &statement, nil) == SQLITE_OK else {
            throw TestSQLiteError(database: database, fallback: "Unable to prepare SQLite statement")
        }
        defer { sqlite3_finalize(statement) }

        guard sqlite3_step(statement) == SQLITE_ROW else {
            throw TestSQLiteError(database: database, fallback: "SQLite query returned no rows")
        }

        return Int(sqlite3_column_int64(statement, 0))
    }

    private struct TestSQLiteError: LocalizedError {
        let message: String

        init(database: OpaquePointer?, fallback: String) {
            if let database, let sqliteMessage = sqlite3_errmsg(database) {
                message = String(cString: sqliteMessage)
            } else {
                message = fallback
            }
        }

        var errorDescription: String? {
            message
        }
    }
}
