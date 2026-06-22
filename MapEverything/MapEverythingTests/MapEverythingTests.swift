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
}
