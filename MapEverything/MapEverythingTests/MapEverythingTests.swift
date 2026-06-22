//
//  MapEverythingTests.swift
//  MapEverythingTests
//
//  Created by Alex Moran on 5/2/26.
//

import Testing
import Foundation
import simd
import CoreVideo
import CoreLocation
import SwiftData
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
        #expect(schema.topic == "/reconstructor/radio")
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
        let registry = ROS2TopicRegistry()
        let markerTopic = registry.definition(.meshMarkers)
        let snapshotTopic = registry.definition(.meshSnapshot)
        let advertisedIDs = Set(registry.advertisedTopics().map(\.id))
        let schema = MeshSnapshotMessageSchema.shared

        #expect(markerTopic.topic == "/reconstructor/map")
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
        #expect(streams.first?.topic == "/reconstructor/radio")
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
}
