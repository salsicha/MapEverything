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
import CoreML
import CoreVideo
import CoreLocation
import SwiftData
import SQLite3
import UIKit
@testable import MapEverything

struct MapEverythingTests {

    @Test("Filters out points beyond 20 meters")
    func testPointCloudProcessorOutlierRemoval() {
        let processor = PointCloudProcessor()
        let validPoints = [
            ColoredPoint(position: SIMD3<Float>(1.0, 2.0, 3.0)),
            ColoredPoint(position: SIMD3<Float>(1.04, 2.0, 3.0)),
            ColoredPoint(position: SIMD3<Float>(1.0, 2.04, 3.0)),
            ColoredPoint(position: SIMD3<Float>(1.0, 2.0, 3.04))
        ]
        let outlierPoint = ColoredPoint(position: SIMD3<Float>(30.0, 0.0, 0.0))
        
        let points = validPoints + [outlierPoint]
        let filtered = processor.removeOutliers(points: points, maxDistance: 20.0)
        
        #expect(filtered.count == validPoints.count)
        #expect(!filtered.contains(outlierPoint))
        #expect(validPoints.allSatisfy { filtered.contains($0) })
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

    @Test("Point cloud processor fuses Depth Anything and LiDAR directly into points")
    func testPointCloudProcessorDirectFusedDepthPoints() throws {
        let width = 32
        let height = 32
        var relativeData: [Float] = []
        relativeData.reserveCapacity(width * height)

        for y in 0..<height {
            for x in 0..<width {
                relativeData.append(0.5 + Float(x + y) / Float(width + height))
            }
        }

        let relativeDepthMap = RelativeDepthMap(width: width, height: height, data: relativeData)
        let lidarDepthMap = try makeDepthFloat32PixelBuffer(
            width: width,
            height: height,
            values: relativeData.map { 2.0 * $0 + 0.5 }
        )
        let cameraImage = try makeYpCbCrPixelBuffer(width: width, height: height, luma: 128, cb: 128, cr: 128)

        var intrinsics = matrix_identity_float3x3
        intrinsics[0][0] = 24
        intrinsics[1][1] = 24
        intrinsics[2][0] = Float(width) / 2
        intrinsics[2][1] = Float(height) / 2

        let points = PointCloudProcessor().processFusedPointCloud(
            cameraImage: cameraImage,
            intrinsics: intrinsics,
            imageResolution: CGSize(width: width, height: height),
            transform: matrix_identity_float4x4,
            relativeDepthMap: relativeDepthMap,
            lidarDepthMap: lidarDepthMap
        )

        #expect(points.count == 36)
        #expect(points.allSatisfy { $0.position.z < -1.4 && $0.position.z > -3.5 })
        #expect(points.allSatisfy { $0.color == SIMD3<UInt8>(128, 128, 128) })
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

    @Test("Colored surfel capped snapshots keep highest-priority surfels")
    func testColoredSurfelMapCappedSnapshotKeepsHighestPrioritySurfels() async throws {
        let map = ColoredSurfelMap(voxelSize: 0.05, maxSurfels: 20)

        for index in 0..<5 {
            let point = ColoredPoint(
                position: SIMD3<Float>(Float(index) * 0.1 + 0.01, 0.01, -1.0),
                color: SIMD3<UInt8>(UInt8(20 * index), 30, 40)
            )
            _ = await map.fuse(
                points: [point],
                observerPosition: SIMD3<Float>(0, 0, 0),
                timestamp: TimeInterval(index)
            )
        }

        let surfels = await map.snapshot(maxCount: 2)

        #expect(surfels.count == 2)
        #expect(surfels.allSatisfy { $0.position.x > 0.25 })
    }

    @Test("Colored surfel map trims oldest overflow without losing recent surfels")
    func testColoredSurfelMapTrimsOldestOverflow() async throws {
        let map = ColoredSurfelMap(voxelSize: 0.05, maxSurfels: 3)

        for index in 0..<5 {
            let point = ColoredPoint(
                position: SIMD3<Float>(Float(index) * 0.1 + 0.01, 0.01, -1.0),
                color: SIMD3<UInt8>(UInt8(20 * index), 30, 40)
            )
            _ = await map.fuse(
                points: [point],
                observerPosition: SIMD3<Float>(0, 0, 0),
                timestamp: TimeInterval(index)
            )
        }

        let surfels = await map.snapshot()

        #expect(surfels.count == 3)
        #expect(surfels.allSatisfy { $0.position.x > 0.15 })
    }

    @Test("Mesh rebuild throttle gates work per anchor")
    func testMeshRebuildThrottleGatesPerAnchor() throws {
        var throttle = MeshRebuildThrottle(minimumInterval: 0.5)
        let firstAnchor = UUID()
        let secondAnchor = UUID()

        let initialToken = throttle.begin(anchorID: firstAnchor, now: 1.0)
        let firstToken = try #require(initialToken)
        let skippedInFlightToken = throttle.begin(anchorID: firstAnchor, now: 1.1)
        let otherAnchorToken = throttle.begin(anchorID: secondAnchor, now: 1.1)
        #expect(skippedInFlightToken == nil)
        #expect(otherAnchorToken != nil)

        let finishedInitial = throttle.finish(anchorID: firstAnchor, token: firstToken)
        #expect(finishedInitial)
        let skippedIntervalToken = throttle.begin(anchorID: firstAnchor, now: 1.2)
        #expect(skippedIntervalToken == nil)

        let intervalToken = throttle.begin(anchorID: firstAnchor, now: 1.6)
        let secondToken = try #require(intervalToken)
        let replacementToken = throttle.begin(anchorID: firstAnchor, now: 1.7, force: true)
        let forcedToken = try #require(replacementToken)

        let finishedStale = throttle.finish(anchorID: firstAnchor, token: secondToken)
        let finishedForced = throttle.finish(anchorID: firstAnchor, token: forcedToken)
        #expect(!finishedStale)
        #expect(finishedForced)

        throttle.removeAnchor(secondAnchor)
        let resetAnchorToken = throttle.begin(anchorID: secondAnchor, now: 1.2)
        #expect(resetAnchorToken != nil)
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

        let secondDepthMap = processor!.inferRelativeDepth(from: buffer)
        let secondMap = try #require(secondDepthMap, "reused Vision request returned nil")
        #expect(secondMap.width == map.width)
        #expect(secondMap.height == map.height)
        #expect(secondMap.data.count == map.data.count)
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

    @Test("RelativeDepthMap reads native model outputs without eager copies")
    func testRelativeDepthMapViewsNativeBackings() throws {
        let array = try MLMultiArray(shape: [2, 2], dataType: .float32)
        let arrayValues = array.dataPointer.assumingMemoryBound(to: Float32.self)
        arrayValues[0] = 1
        arrayValues[1] = 2
        arrayValues[2] = 3
        arrayValues[3] = 4

        let arrayMap = try #require(RelativeDepthMap(fromMultiArray: array, size: 2))
        #expect(arrayMap.value(atX: 1, y: 0) == 2)
        arrayValues[1] = 22
        #expect(arrayMap.value(atX: 1, y: 0) == 22)

        let buffer = try makeDepthFloat32PixelBuffer(
            width: 2,
            height: 2,
            values: [5, 6, 7, 8]
        )
        let bufferMap = try #require(RelativeDepthMap(fromPixelBuffer: buffer))
        #expect(bufferMap.value(atX: 0, y: 1) == 7)

        CVPixelBufferLockBaseAddress(buffer, [])
        if let base = CVPixelBufferGetBaseAddress(buffer)?.assumingMemoryBound(to: Float32.self) {
            let floatsPerRow = CVPixelBufferGetBytesPerRow(buffer) / MemoryLayout<Float32>.stride
            base[floatsPerRow] = 77
        }
        CVPixelBufferUnlockBaseAddress(buffer, [])

        #expect(bufferMap.value(atX: 0, y: 1) == 77)
    }

    @Test("Depth Anything calibration fits metric scale from LiDAR")
    func testDepthAnythingCalibrationUsesLiDARDepthAndConfidence() throws {
        let width = 32
        let height = 32
        var relativeValues: [Float] = []
        relativeValues.reserveCapacity(width * height)
        for y in 0..<height {
            for x in 0..<width {
                relativeValues.append(0.25 + Float(x + y) / Float(width + height))
            }
        }

        let relative = RelativeDepthMap(width: width, height: height, data: relativeValues)
        let lidar = try makeDepthFloat32PixelBuffer(
            width: width,
            height: height,
            values: relativeValues.map { 2.0 * $0 + 0.5 }
        )
        let confidence = try makeLiDARConfidencePixelBuffer(
            width: width,
            height: height,
            value: 2
        )

        let calibration = try #require(DepthAnythingProcessor.maximumLikelihoodCalibration(
            relative: relative,
            lidarDepthMap: lidar,
            lidarConfidenceMap: confidence
        ))

        #expect(abs(calibration.scale - 2.0) < 0.001)
        #expect(abs(calibration.offset - 0.5) < 0.001)
    }

    @Test("Depth Anything calibration cache reuses nearby frames")
    func testDepthAnythingCalibrationCacheReusesNearbyFrames() throws {
        let width = 32
        let height = 32
        var relativeValues: [Float] = []
        relativeValues.reserveCapacity(width * height)
        for y in 0..<height {
            for x in 0..<width {
                relativeValues.append(0.5 + Float(x + y) / Float(width + height))
            }
        }

        let relative = RelativeDepthMap(width: width, height: height, data: relativeValues)
        let lidar = try makeDepthFloat32PixelBuffer(
            width: width,
            height: height,
            values: relativeValues.map { 2.0 * $0 + 0.5 }
        )
        let confidence = try makeLiDARConfidencePixelBuffer(
            width: width,
            height: height,
            value: 2
        )
        let cache = DepthAnythingCalibrationCache(
            maxAge: 1.0,
            maxTranslationMeters: 0.25,
            maxRotationRadians: 0.25
        )
        let pose = matrix_identity_float4x4

        let first = try #require(cache.calibration(
            relative: relative,
            lidarDepthMap: lidar,
            lidarConfidenceMap: confidence,
            timestamp: 10.0,
            cameraTransform: pose
        ))

        let changedRelative = RelativeDepthMap(
            width: width,
            height: height,
            data: relativeValues.map { $0 * 3.0 }
        )
        let changedLidar = try makeDepthFloat32PixelBuffer(
            width: width,
            height: height,
            values: relativeValues.map { 1.2 * $0 + 0.4 }
        )

        let reused = try #require(cache.calibration(
            relative: changedRelative,
            lidarDepthMap: changedLidar,
            lidarConfidenceMap: confidence,
            timestamp: 10.5,
            cameraTransform: pose
        ))
        #expect(reused.scale == first.scale)
        #expect(reused.offset == first.offset)

        var movedPose = pose
        movedPose.columns.3.x = 1.0
        let recomputed = try #require(cache.calibration(
            relative: changedRelative,
            lidarDepthMap: changedLidar,
            lidarConfidenceMap: confidence,
            timestamp: 10.6,
            cameraTransform: movedPose
        ))
        #expect(abs(recomputed.scale - first.scale) > 0.1)
    }

    @Test("RadioObservation schema covers every radio telemetry channel")
    func testRadioObservationSchemaDefinition() {
        let schema = RadioObservationMessageSchema.shared
        let fieldNames = Set(schema.fields.map(\.name))
        let catalogChannelIDs = RadioTelemetryChannelID.allCases.map(\.rawValue).sorted()

        #expect(schema.messageType == "mapeverything_msgs/msg/RadioObservation")
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
        #expect(schema.schemaVersion == 2)
        #expect(!schema.dependencies.contains("geometry_msgs/msg/Point"))
        #expect(schema.messageDefinition.contains("uint8[] vertex_data"))
        #expect(schema.messageDefinition.contains("uint8[] index_data"))
        #expect(schema.fields.contains { $0.name == "vertex_encoding" && $0.type == "string" })
        #expect(schema.fields.contains { $0.name == "vertex_data" && $0.type == "uint8[]" })
        #expect(schema.fields.contains { $0.name == "index_data" && $0.type == "uint8[]" })
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

        let vertexCount = try #require(message["vertex_count"] as? Int)
        let vertexDataText = try #require(message["vertex_data"] as? String)
        let indexDataText = try #require(message["index_data"] as? String)
        let vertexData = try #require(Data(base64Encoded: vertexDataText))
        let indexData = try #require(Data(base64Encoded: indexDataText))
        let encodedBytes = try #require(
            MeshSnapshotMessageBuilder.encodedPublishPayloadByteCount(
                topic: MeshSnapshotMessageSchema.shared.topic,
                msg: message
            )
        )

        #expect(message["is_truncated"] as? Bool == true)
        #expect(message["schema_version"] as? Int == 2)
        #expect(message["compression"] as? String == "mesh_snapshot_binary_base64")
        #expect(message["vertex_encoding"] as? String == "float32_xyz_le_base64")
        #expect(message["index_encoding"] as? String == "uint32_le_base64")
        #expect(message["vertex_stride_bytes"] as? Int == 12)
        #expect(message["index_stride_bytes"] as? Int == 4)
        #expect(vertexCount % 3 == 0)
        #expect(vertexData.count == vertexCount * 12)
        #expect(indexData.count == vertexCount * 4)
        #expect(message["vertices"] == nil)
        #expect(message["triangle_indices"] == nil)
        #expect(message["original_vertex_count"] as? Int == trianglePoints.count)
        #expect((message["published_payload_bytes"] as? Int ?? 0) <= maxPayloadBytes)
        #expect(encodedBytes <= maxPayloadBytes)
        #expect(JSONSerialization.isValidJSONObject(message))
        _ = try JSONSerialization.data(withJSONObject: message, options: [])
    }

    @Test("SafeARMesh snapshots pack indexed binary geometry directly")
    func testMeshSnapshotBuilderPacksSafeARMeshBytesDirectly() throws {
        var transform = matrix_identity_float4x4
        transform.columns.3 = SIMD4<Float>(10, 20, 30, 1)
        let mesh = SafeARMesh(
            identifier: UUID(),
            vertices: [
                SIMD3<Float>(0, 0, 0),
                SIMD3<Float>(1, 0, 0),
                SIMD3<Float>(0, 1, 0),
                SIMD3<Float>(0, 0, 1)
            ],
            indices: [0, 1, 2, 0, 2, 3],
            transform: transform
        )
        let header: [String: Any] = [
            "stamp": ["sec": 1, "nanosec": 2],
            "frame_id": "map"
        ]

        let message = MeshSnapshotMessageBuilder.makeSafeMeshMessage(
            header: header,
            snapshotID: "safe-mesh-direct",
            source: "unit_test",
            frameID: "map",
            safeMeshes: [mesh],
            maxTrianglePoints: 6,
            maxPayloadBytes: 32_000,
            metadata: ["test": "safe_mesh_direct"]
        )

        let vertexDataText = try #require(message["vertex_data"] as? String)
        let indexDataText = try #require(message["index_data"] as? String)
        let vertexData = try #require(Data(base64Encoded: vertexDataText))
        let indexData = try #require(Data(base64Encoded: indexDataText))

        #expect(message["vertex_count"] as? Int == 4)
        #expect(message["triangle_count"] as? Int == 2)
        #expect(message["original_vertex_count"] as? Int == 4)
        #expect(message["original_triangle_count"] as? Int == 2)
        #expect(message["is_truncated"] as? Bool == false)
        #expect(message["vertices"] == nil)
        #expect(message["triangle_indices"] == nil)
        #expect(vertexData.count == 4 * 12)
        #expect(indexData.count == 6 * 4)
        #expect(float32(from: vertexData, at: 0) == 10)
        #expect(float32(from: vertexData, at: 4) == 20)
        #expect(float32(from: vertexData, at: 8) == 30)
        #expect(float32(from: vertexData, at: 12) == 11)
        let decodedIndices: [UInt32] = (0..<6).map { uint32(from: indexData, at: $0 * 4) }
        #expect(decodedIndices == [0, 1, 2, 0, 2, 3])
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

    @Test("Implemented ROS2 topics serialize advertised schema metadata")
    func testROS2TopicRegistryAdvertisedTopicsSerialize() throws {
        let registry = ROS2TopicRegistry()
        let allTopics = registry.allTopics()
        let advertisedTopics = registry.advertisedTopics()
        let advertisedIDs = Set(advertisedTopics.map(\.id))

        #expect(allTopics.count == ROS2TopicID.allCases.count)
        #expect(Set(allTopics.map(\.id)) == Set(ROS2TopicID.allCases))
        #expect(advertisedIDs.isSuperset(of: [.pose, .cameraCompressed, .cameraInfo, .lidarPointCloud, .depthAnythingPointCloud, .depthAnythingCalibration, .gpsFix, .gpsMetadata, .satelliteImage, .satelliteTileInfo, .demTile]))
        #expect(registry.definition(.lidarPointCloud).topic == "/mapping/pointcloud/lidar")
        #expect(registry.definition(.depthAnythingPointCloud).topic == "/mapping/pointcloud/depth_anything")
        #expect(registry.definition(.depthAnythingCalibration).topic == "/mapping/depth_anything/calibration")
        #expect(registry.definition(.depthAnythingCalibration).messageType == "mapeverything_msgs/msg/DepthAnythingCalibration")
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

    @Test("Colored point-cloud payload uses standard PointCloud2 fields")
    func testColoredPointCloudPayloadUsesPointCloud2Fields() throws {
        let header: [String: Any] = [
            "stamp": ["sec": 10, "nanosec": 20],
            "frame_id": "map"
        ]
        let points = [
            ColoredPoint(
                position: SIMD3<Float>(1, 2, 3),
                color: SIMD3<UInt8>(10, 20, 30)
            )
        ]

        let msg = ROS2BridgeClient.makeColoredPointCloudMessage(points: points, header: header)
        let fields = try #require(msg["fields"] as? [[String: Any]])
        let fieldNames = Set(fields.compactMap { $0["name"] as? String })
        let encodedData = try #require(msg["data"] as? String)
        let decodedData = try #require(Data(base64Encoded: encodedData))

        #expect(msg["point_step"] as? Int == 16)
        #expect(msg["row_step"] as? Int == 16)
        #expect(msg["width"] as? Int == 1)
        #expect(decodedData.count == 16)
        #expect(fieldNames == Set(["x", "y", "z", "rgb"]))
        #expect(JSONSerialization.isValidJSONObject(msg))
        _ = try JSONSerialization.data(withJSONObject: msg, options: [])
    }

    @Test("Depth Anything calibration payload serializes scale and relative cloud metadata")
    func testDepthAnythingCalibrationPayloadSerializes() throws {
        let header: [String: Any] = [
            "stamp": ["sec": 10, "nanosec": 20],
            "frame_id": "iphone_camera"
        ]
        let calibration = DepthAnythingProcessor.MaximumLikelihoodCalibration(scale: 2.5, offset: 0.25)

        let msg = ROS2BridgeClient.makeDepthAnythingCalibrationMessage(
            calibration: calibration,
            header: header,
            relativeDepthSize: CGSize(width: 518, height: 518),
            imageResolution: CGSize(width: 1920, height: 1440),
            relativePointCloudTopic: "/mapping/pointcloud/depth_anything",
            frameID: "iphone_camera"
        )

        #expect(msg["schema_version"] as? Int == 1)
        #expect(msg["relative_pointcloud_topic"] as? String == "/mapping/pointcloud/depth_anything")
        #expect(msg["frame_id"] as? String == "iphone_camera")
        #expect(msg["relative_depth_width"] as? Int == 518)
        #expect(msg["relative_depth_height"] as? Int == 518)
        #expect(msg["scale"] as? Double == 2.5)
        #expect(msg["offset"] as? Double == 0.25)
        #expect(msg["equation"] as? String == "metric_depth_m = scale * relative_depth + offset")
        #expect((msg["metadata_json"] as? String)?.contains("overlay_mesh_uses_calibrated_depth") == true)
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
        #expect(GeoTilePublisher.Configuration.default.publishInterval == 60)

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

    @Test("Satellite and DEM map messages include GPS pixel coordinates")
    func testGeoTileMessagesIncludeGPSPixelCoordinatesForBothLayers() throws {
        let location = CLLocation(
            coordinate: CLLocationCoordinate2D(latitude: 39.7392, longitude: -104.9903),
            altitude: 1609,
            horizontalAccuracy: 5,
            verticalAccuracy: 8,
            timestamp: Date(timeIntervalSince1970: 1_700_000_030)
        )
        let timestamp = Date(timeIntervalSince1970: 1_700_000_040)
        let bridge = ROS2BridgeClient()
        let header: [String: Any] = ["stamp": ["sec": 10, "nanosec": 20], "frame_id": "earth"]
        let satellitePayload = try makeTestGeoTilePayload(
            provider: .defaultSatellite,
            location: location,
            date: timestamp,
            data: Data([0xFF, 0xD8, 0xFF])
        )
        let demPayload = try makeTestGeoTilePayload(
            provider: .usgs3DEPDEM,
            location: location,
            date: timestamp,
            data: Data([1, 2, 3, 4])
        )

        let satelliteMessage = bridge.makeGeoTileInfoMessage(tile: satellitePayload, header: header)
        let demMessage = bridge.makeGeoRasterTileMessage(tile: demPayload, header: header)

        try assertGeoTileMessage(
            satelliteMessage,
            matches: satellitePayload,
            expectedKind: GeoTileLayerKind.satelliteImagery,
            includesRasterPayload: false
        )
        try assertGeoTileMessage(
            demMessage,
            matches: demPayload,
            expectedKind: GeoTileLayerKind.dem,
            includesRasterPayload: true
        )
    }

    @Test("Geo tile topics advertise one-minute publish cadence")
    func testGeoTileTopicsAdvertiseOneMinuteCadence() {
        let registry = ROS2TopicRegistry()
        let expectedRate = 1.0 / 60.0

        #expect(registry.definition(.satelliteImage).defaultRateHz == expectedRate)
        #expect(registry.definition(.satelliteTileInfo).defaultRateHz == expectedRate)
        #expect(registry.definition(.demTile).defaultRateHz == expectedRate)
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

        #expect(await waitUntil { stats.latest?.droppedMessages == 2 && stats.latest?.depth == 2 })
        #expect(stats.latest?.lastError?.contains("/test/third") == true)

        sends.completeNext(with: nil)
        #expect(await waitUntil { sends.sentPayloads.count == 2 })
        sends.completeNext(with: nil)
        #expect(await waitUntil { stats.latest?.sentMessages == 2 })

        #expect(sends.sentPayloads == ["first", "fourth"])
        #expect(stats.latest?.depth == 0)
        #expect(stats.latest?.failedMessages == 0)
    }

    @Test("Publish queue counts the in-flight send against capacity")
    func testPublishQueueBackpressureCountsInFlightSend() async throws {
        let stats = PublishQueueStatsRecorder()
        let sends = PublishQueueSendRecorder()
        let queue = PublishQueue(
            configuration: PublishQueue.Configuration(
                capacity: 1,
                maxRetries: 0,
                retryDelayMilliseconds: 1,
                dropPolicy: .dropOldestPublish
            )
        ) { data, completion in
            sends.record(data: data, completion: completion)
        }
        queue.onStatsChange = stats.record

        queue.enqueueEncodedPayload(Data("first".utf8), op: "publish", topic: "/test/first")
        #expect(await waitUntil { sends.sentPayloads == ["first"] && stats.latest?.depth == 1 })

        queue.enqueueEncodedPayload(Data("second".utf8), op: "publish", topic: "/test/second")
        queue.enqueueEncodedPayload(Data("third".utf8), op: "advertise", topic: "/test/third")

        #expect(await waitUntil { stats.latest?.droppedMessages == 2 })
        #expect(stats.latest?.depth == 1)
        #expect(stats.latest?.lastError?.contains("/test/third") == true)

        sends.completeNext(with: nil)
        #expect(await waitUntil { stats.latest?.sentMessages == 1 && stats.latest?.depth == 0 })
        #expect(sends.sentPayloads == ["first"])
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

    @Test("Publish queue reports payload encoding failures")
    func testPublishQueueReportsPayloadEncodingFailures() async throws {
        let stats = PublishQueueStatsRecorder()
        let sends = PublishQueueSendRecorder()
        let queue = PublishQueue(
            configuration: PublishQueue.Configuration(
                capacity: 4,
                maxRetries: 0,
                retryDelayMilliseconds: 1,
                dropPolicy: .dropOldestPublish
            )
        ) { data, completion in
            sends.record(data: data, completion: completion)
        }
        queue.onStatsChange = stats.record

        queue.enqueue(
            payload: [
                "op": "publish",
                "topic": "/test/bad",
                "msg": ["invalid": Date()]
            ],
            op: "publish",
            topic: "/test/bad"
        )

        #expect(await waitUntil { stats.latest?.failedMessages == 1 })
        #expect(stats.latest?.lastError?.contains("/test/bad") == true)
        #expect(sends.sentPayloads.isEmpty)
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

    @Test("Enabled ROS topics record to local SQLite bag while bridge is disconnected")
    func testPublishersRecordLocalBagWhileDisconnected() throws {
        let fileManager = FileManager.default
        let rootURL = fileManager.temporaryDirectory
            .appendingPathComponent("MapEverythingLocalBagAllTopics-\(UUID().uuidString)", isDirectory: true)
        try fileManager.createDirectory(at: rootURL, withIntermediateDirectories: true)
        defer { try? fileManager.removeItem(at: rootURL) }

        let recorder = LocalROS2BagRecorder(fileManager: fileManager, baseDirectoryURL: rootURL)
        recorder.start(
            sessionID: UUID(),
            configuration: LocalROS2BagRecorderConfiguration(isEnabled: true, maxChunkBytes: 8 * 1_048_576)
        )

        let registry = ROS2TopicRegistry(
            enabledStreams: [
                .pose,
                .odometry,
                .tf,
                .camera,
                .pointCloud,
                .mesh,
                .gps,
                .radio,
                .indoorLocalization,
                .satelliteImagery,
                .dem,
                .diagnostics,
                .session
            ]
        )
        let bridge = ROS2BridgeClient(topicRegistry: registry, localBagRecorder: recorder)
        var transform = matrix_identity_float4x4
        transform.columns.3 = SIMD4<Float>(1.25, -0.5, 2.0, 1.0)
        let timestamp: TimeInterval = 12.5
        let location = CLLocation(
            coordinate: CLLocationCoordinate2D(latitude: 39.7392, longitude: -104.9903),
            altitude: 1609,
            horizontalAccuracy: 5,
            verticalAccuracy: 8,
            timestamp: Date(timeIntervalSince1970: 1_700_000_030)
        )
        let satellitePayload = try makeTestGeoTilePayload(
            provider: .defaultSatellite,
            location: location,
            date: Date(timeIntervalSince1970: 1_700_000_040),
            data: Data([0xFF, 0xD8, 0xFF])
        )
        let demPayload = try makeTestGeoTilePayload(
            provider: .usgs3DEPDEM,
            location: location,
            date: Date(timeIntervalSince1970: 1_700_000_040),
            data: Data([1, 2, 3, 4])
        )

        bridge.publishPose(transform, timestamp: timestamp)
        bridge.publishOdometry(transform, timestamp: timestamp)
        bridge.publishTF(transform, timestamp: timestamp)
        bridge.publishImage(
            pixelBuffer: try makeBGRA8PixelBuffer(width: 4, height: 4),
            intrinsics: matrix_identity_float3x3,
            imageResolution: CGSize(width: 4, height: 4),
            timestamp: timestamp
        )
        bridge.publishLiDARPointCloud(
            [ColoredPoint(position: SIMD3<Float>(0, 0, 1), color: SIMD3<UInt8>(255, 0, 0))],
            timestamp: timestamp
        )
        bridge.publishDepthAnythingPointCloud(
            [ColoredPoint(position: SIMD3<Float>(0.1, 0, 1), color: SIMD3<UInt8>(0, 255, 0))],
            timestamp: timestamp
        )
        bridge.publishDepthAnythingCalibration(
            DepthAnythingProcessor.MaximumLikelihoodCalibration(scale: 1.0, offset: 0.0),
            relativeDepthSize: CGSize(width: 4, height: 4),
            imageResolution: CGSize(width: 4, height: 4),
            timestamp: timestamp
        )
        bridge.publishMap(
            safeMeshes: [
                SafeARMesh(
                    identifier: UUID(),
                    vertices: [
                        SIMD3<Float>(0, 0, 0),
                        SIMD3<Float>(1, 0, 0),
                        SIMD3<Float>(0, 1, 0)
                    ],
                    indices: [0, 1, 2],
                    transform: matrix_identity_float4x4
                )
            ],
            timestamp: timestamp
        )
        bridge.publishSatelliteTile(satellitePayload, timestamp: timestamp)
        bridge.publishGeoTileInfo(satellitePayload, timestamp: timestamp)
        bridge.publishDEMTile(demPayload, timestamp: timestamp)
        bridge.publishNavSatFix(location, timestamp: timestamp)
        bridge.publishGPSMetadata(location)
        bridge.publishIndoorLocalization(
            IndoorLocalizationSample(
                location: location,
                heading: nil,
                indoorRegistrationQuality: 1.0,
                globalRegistrationQuality: 1.0,
                indoorQualityLabel: "excellent",
                globalQualityLabel: "excellent",
                timestamp: Date(timeIntervalSince1970: 1_700_000_041)
            ),
            timestamp: timestamp
        )
        bridge.publishRadioObservation(makeRadioObservation(timestamp: 1_700_000_042, sourceID: "local-bag-radio"))
        bridge.publishSessionMetadata(
            MappingSessionSnapshot(
                event: "started",
                sessionID: UUID(),
                state: "Active",
                recorderURL: "ws://127.0.0.1:9090",
                enabledStreams: registry.advertisedTopics().map(\.stream.rawValue).sorted(),
                startedAt: Date(timeIntervalSince1970: 1_700_000_043),
                endedAt: nil,
                lastError: nil
            ),
            timestamp: timestamp
        )
        bridge.publishDiagnostics()
        recorder.flushAndWait()

        let bagDirectory = try #require(
            try fileManager.contentsOfDirectory(
                at: rootURL,
                includingPropertiesForKeys: [.isDirectoryKey]
            ).first { url in
                (try? url.resourceValues(forKeys: [.isDirectoryKey]).isDirectory) == true
            }
        )
        let chunkURL = bagDirectory.appendingPathComponent("mapeverything_0.db3")
        let expectedTopics: [(name: String, type: String)] = [
            ("/mapping/pose", "geometry_msgs/msg/PoseStamped"),
            ("/mapping/odom", "nav_msgs/msg/Odometry"),
            ("/tf", "tf2_msgs/msg/TFMessage"),
            ("/mapping/camera/image/compressed", "sensor_msgs/msg/CompressedImage"),
            ("/mapping/camera/camera_info", "sensor_msgs/msg/CameraInfo"),
            ("/mapping/pointcloud/lidar", "sensor_msgs/msg/PointCloud2"),
            ("/mapping/pointcloud/depth_anything", "sensor_msgs/msg/PointCloud2"),
            ("/mapping/depth_anything/calibration", "mapeverything_msgs/msg/DepthAnythingCalibration"),
            ("/mapping/map", "visualization_msgs/msg/MarkerArray"),
            ("/mapping/mesh_snapshot", "mapeverything_msgs/msg/MeshSnapshot"),
            ("/mapping/satellite/image/compressed", "sensor_msgs/msg/CompressedImage"),
            ("/mapping/satellite/tile_info", "mapeverything_msgs/msg/GeoTileInfo"),
            ("/mapping/dem/tile", "mapeverything_msgs/msg/GeoRasterTile"),
            ("/mapping/gps/fix", "sensor_msgs/msg/NavSatFix"),
            ("/mapping/gps/metadata", "mapeverything_msgs/msg/GPSMetadata"),
            ("/mapping/indoor_localization", "mapeverything_msgs/msg/IndoorLocalization"),
            ("/mapping/radio", "mapeverything_msgs/msg/RadioObservation"),
            ("/mapping/session", "mapeverything_msgs/msg/MappingSession"),
            ("/mapping/status", "diagnostic_msgs/msg/DiagnosticArray")
        ]

        for expectedTopic in expectedTopics {
            #expect(try sqliteInteger(
                url: chunkURL,
                sql: "SELECT COUNT(*) FROM topics WHERE name = '\(expectedTopic.name)' AND type = '\(expectedTopic.type)'"
            ) == 1)
            #expect(try sqliteInteger(
                url: chunkURL,
                sql: "SELECT COUNT(*) FROM messages JOIN topics ON topics.id = messages.topic_id WHERE topics.name = '\(expectedTopic.name)'"
            ) >= 1)
        }

        recorder.stopAndWait()
    }

    @Test("Local ROS2 bag recorder writes chunked SQLite rosbridge JSON bags")
    func testLocalROS2BagRecorderWritesChunkedSQLiteBag() async throws {
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
        let cameraMessage = makeLocalBagCameraMessage(sequence: 3)
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
        recorder.recordPublishedTopic(
            topic: "/mapping/camera/image/compressed",
            messageType: "sensor_msgs/msg/CompressedImage",
            msg: cameraMessage
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
        let cachedOnlySessions = try recorder.listBagSessions()
        let cachedOnlySession = try #require(cachedOnlySessions.first)

        #expect(cachedOnlySession.preview == nil)
        #expect(!fileManager.fileExists(atPath: bagDirectory.appendingPathComponent(LocalROS2BagSessionPreview.cacheFileName).path))
        #expect(!fileManager.fileExists(atPath: bagDirectory.appendingPathComponent(LocalROS2BagSessionPreview.thumbnailFileName).path))

        let sessions = try await recorder.listBagSessionsAsync(previewLoadingMode: .scanIfNeeded)
        let listedSession = try #require(sessions.first)

        #expect(dbFiles.count == 3)
        #expect(metadata.contains("storage_identifier: sqlite3"))
        #expect(metadata.contains("serialization_format: 'rosbridge_json'"))
        #expect(metadata.contains("mapeverything_0.db3"))
        #expect(metadata.contains("mapeverything_1.db3"))
        #expect(metadata.contains("mapeverything_2.db3"))
        #expect(metadata.contains("Messages are stored as rosbridge publish JSON payloads"))

        let totalMessages = try dbFiles.reduce(0) { count, url in
            count + (try sqliteInteger(url: url, sql: "SELECT COUNT(*) FROM messages"))
        }
        let topicCount = try dbFiles.reduce(0) { count, url in
            count + (try sqliteInteger(url: url, sql: "SELECT COUNT(*) FROM topics WHERE name = '/mapping/status' AND type = 'diagnostic_msgs/msg/DiagnosticArray' AND serialization_format = 'rosbridge_json'"))
        }

        #expect(totalMessages == 3)
        #expect(topicCount == 2)
        #expect(cachedOnlySessions.count == 1)
        #expect(sessions.count == 1)
        #expect(listedSession.chunkCount == 3)
        #expect(listedSession.files.contains { $0.name == "metadata.yaml" && $0.kind == .metadata })
        #expect(listedSession.files.contains { $0.name == "mapeverything_0.db3" && $0.kind == .sqliteChunk })
        let preview = try #require(listedSession.preview)
        #expect(preview.messageCount == 3)
        #expect(preview.topicNames.contains("/mapping/status"))
        #expect(preview.topicNames.contains("/mapping/camera/image/compressed"))
        #expect(preview.thumbnailRelativePath == LocalROS2BagSessionPreview.thumbnailFileName)
        #expect(fileManager.fileExists(atPath: bagDirectory.appendingPathComponent(LocalROS2BagSessionPreview.cacheFileName).path))
        #expect(fileManager.fileExists(atPath: bagDirectory.appendingPathComponent(LocalROS2BagSessionPreview.thumbnailFileName).path))

        try recorder.deleteBagSession(listedSession)
        #expect(try recorder.listBagSessions().isEmpty)
    }

    @Test("Local ROS2 bag recorder writes final overlay mesh artifacts")
    func testLocalROS2BagRecorderWritesFinalOverlayMeshArtifact() throws {
        let fileManager = FileManager.default
        let rootURL = fileManager.temporaryDirectory
            .appendingPathComponent("MapEverythingLocalBagMesh-\(UUID().uuidString)", isDirectory: true)
        try fileManager.createDirectory(at: rootURL, withIntermediateDirectories: true)
        defer { try? fileManager.removeItem(at: rootURL) }

        let recorder = LocalROS2BagRecorder(fileManager: fileManager, baseDirectoryURL: rootURL)
        recorder.start(
            sessionID: UUID(),
            configuration: LocalROS2BagRecorderConfiguration(isEnabled: true, maxChunkBytes: 8 * 1_048_576)
        )
        let targetDirectoryURL = try #require(recorder.currentArtifactDirectoryURL)
        recorder.stopAndWait()

        recorder.recordFinalOverlayMesh(
            LocalOverlayMeshArtifact(
                source: "unit_test",
                coordinateFrame: "map",
                capturedAt: Date(timeIntervalSince1970: 1_700_000_050),
                vertices: [
                    SIMD3<Float>(0, 0, 0),
                    SIMD3<Float>(1, 0, 0),
                    SIMD3<Float>(0, 1, 0)
                ],
                indices: [0, 1, 2],
                metadata: ["visualization_mode": "Solid Mesh"]
            ),
            in: targetDirectoryURL
        )
        recorder.recordFinalPointCloud(
            LocalPointCloudArtifact(
                source: "unit_test_points",
                coordinateFrame: "map",
                capturedAt: Date(timeIntervalSince1970: 1_700_000_051),
                points: [
                    LocalPointCloudArtifact.Point(
                        position: SIMD3<Float>(0.25, 0.5, 0.75),
                        color: SIMD3<UInt8>(10, 20, 30)
                    )
                ],
                metadata: ["point_count_source": "unit_test"]
            ),
            in: targetDirectoryURL
        )
        recorder.flushAndWait()

        let session = try #require(try recorder.listBagSessions().first)
        #expect(session.files.contains { $0.name == LocalOverlayMeshArtifact.objFileName && $0.kind == .overlayMesh })
        #expect(session.files.contains { $0.name == LocalOverlayMeshArtifact.metadataFileName && $0.kind == .overlayMeshMetadata })
        #expect(session.files.contains { $0.name == LocalPointCloudArtifact.plyFileName && $0.kind == .pointCloud })
        #expect(session.files.contains { $0.name == LocalPointCloudArtifact.metadataFileName && $0.kind == .pointCloudMetadata })

        let objURL = session.directoryURL.appendingPathComponent(LocalOverlayMeshArtifact.objFileName)
        let obj = try String(contentsOf: objURL, encoding: .utf8)
        #expect(obj.contains("# source: unit_test"))
        #expect(obj.contains("v 1.000000 0.000000 0.000000"))
        #expect(obj.contains("f 1 2 3"))

        let metadataURL = session.directoryURL.appendingPathComponent(LocalOverlayMeshArtifact.metadataFileName)
        let metadataData = try Data(contentsOf: metadataURL)
        let metadata = try #require(JSONSerialization.jsonObject(with: metadataData) as? [String: Any])
        #expect(metadata["source"] as? String == "unit_test")
        #expect(metadata["coordinate_frame"] as? String == "map")
        #expect(metadata["vertex_count"] as? Int == 3)
        #expect(metadata["triangle_count"] as? Int == 1)

        let plyURL = session.directoryURL.appendingPathComponent(LocalPointCloudArtifact.plyFileName)
        let ply = try String(contentsOf: plyURL, encoding: .utf8)
        #expect(ply.contains("element vertex 1"))
        #expect(ply.contains("0.250000 0.500000 0.750000 10 20 30"))

        let pointMetadataURL = session.directoryURL.appendingPathComponent(LocalPointCloudArtifact.metadataFileName)
        let pointMetadataData = try Data(contentsOf: pointMetadataURL)
        let pointMetadata = try #require(JSONSerialization.jsonObject(with: pointMetadataData) as? [String: Any])
        #expect(pointMetadata["source"] as? String == "unit_test_points")
        #expect(pointMetadata["point_count"] as? Int == 1)
    }

    @Test("Local ROS2 bag recorder flushes pending SQLite batches")
    func testLocalROS2BagRecorderFlushesPendingSQLiteBatches() throws {
        let fileManager = FileManager.default
        let rootURL = fileManager.temporaryDirectory
            .appendingPathComponent("MapEverythingLocalBagBatch-\(UUID().uuidString)", isDirectory: true)
        try fileManager.createDirectory(at: rootURL, withIntermediateDirectories: true)
        defer { try? fileManager.removeItem(at: rootURL) }

        let recorder = LocalROS2BagRecorder(fileManager: fileManager, baseDirectoryURL: rootURL)
        recorder.start(
            sessionID: UUID(),
            configuration: LocalROS2BagRecorderConfiguration(isEnabled: true, maxChunkBytes: 8 * 1_048_576)
        )

        recorder.recordPublishedTopic(
            topic: "/mapping/status",
            messageType: "diagnostic_msgs/msg/DiagnosticArray",
            msg: makeLocalBagMessage(sequence: 1, payloadSize: 128)
        )
        recorder.recordPublishedTopic(
            topic: "/mapping/status",
            messageType: "diagnostic_msgs/msg/DiagnosticArray",
            msg: makeLocalBagMessage(sequence: 2, payloadSize: 128)
        )
        recorder.flushAndWait()

        let bagDirectory = try #require(
            try fileManager.contentsOfDirectory(
                at: rootURL,
                includingPropertiesForKeys: [.isDirectoryKey]
            ).first { url in
                (try? url.resourceValues(forKeys: [.isDirectoryKey]).isDirectory) == true
            }
        )
        let chunkURL = bagDirectory.appendingPathComponent("mapeverything_0.db3")
        #expect(try sqliteInteger(url: chunkURL, sql: "SELECT COUNT(*) FROM messages") == 2)
        #expect(try sqliteInteger(url: chunkURL, sql: "SELECT COUNT(*) FROM topics WHERE name = '/mapping/status'") == 1)

        recorder.stopAndWait()
    }

    @Test("Expanded SwiftData schema stores mapping session records")
    @MainActor
    func testMappingPersistenceModels() throws {
        let schema = MapEverythingModelSchema.schema
        let configuration = ModelConfiguration(schema: schema, isStoredInMemoryOnly: true)
        let container = try ModelContainer(for: schema, configurations: [configuration])
        let context = ModelContext(container)

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

        let sessions = try context.fetch(FetchDescriptor<MappingSessionModel>())
        let streams = try context.fetch(FetchDescriptor<SensorStreamModel>())
        let tiles = try context.fetch(FetchDescriptor<GeoTileModel>())

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

    private func makeLocalBagCameraMessage(sequence: Int) -> [String: Any] {
        let renderer = UIGraphicsImageRenderer(size: CGSize(width: 12, height: 8))
        let jpegData = renderer.jpegData(withCompressionQuality: 0.8) { context in
            UIColor.systemRed.setFill()
            context.fill(CGRect(x: 0, y: 0, width: 6, height: 8))
            UIColor.systemBlue.setFill()
            context.fill(CGRect(x: 6, y: 0, width: 6, height: 8))
        }

        return [
            "header": [
                "stamp": [
                    "sec": 1_700_000_000 + sequence,
                    "nanosec": sequence
                ],
                "frame_id": "iphone_camera"
            ],
            "format": "jpeg",
            "data": jpegData.base64EncodedString()
        ]
    }

    private func makeBGRA8PixelBuffer(width: Int, height: Int) throws -> CVPixelBuffer {
        var pixelBuffer: CVPixelBuffer?
        let attributes: [String: Any] = [
            kCVPixelBufferIOSurfacePropertiesKey as String: [:]
        ]
        let status = CVPixelBufferCreate(
            kCFAllocatorDefault,
            width,
            height,
            kCVPixelFormatType_32BGRA,
            attributes as CFDictionary,
            &pixelBuffer
        )
        guard status == kCVReturnSuccess, let pixelBuffer else {
            throw TestPixelBufferError.creationFailed(status)
        }

        CVPixelBufferLockBaseAddress(pixelBuffer, [])
        defer { CVPixelBufferUnlockBaseAddress(pixelBuffer, []) }

        guard let baseAddress = CVPixelBufferGetBaseAddress(pixelBuffer) else {
            throw TestPixelBufferError.missingBaseAddress
        }

        let bytesPerRow = CVPixelBufferGetBytesPerRow(pixelBuffer)
        let buffer = baseAddress.assumingMemoryBound(to: UInt8.self)
        for y in 0..<height {
            for x in 0..<width {
                let offset = y * bytesPerRow + x * 4
                buffer[offset] = UInt8((x * 63) % 255)
                buffer[offset + 1] = UInt8((y * 63) % 255)
                buffer[offset + 2] = 128
                buffer[offset + 3] = 255
            }
        }

        return pixelBuffer
    }

    private func uint32(from data: Data, at offset: Int) -> UInt32 {
        guard offset + 3 < data.count else { return 0 }
        let value = UInt32(data[offset])
            | (UInt32(data[offset + 1]) << 8)
            | (UInt32(data[offset + 2]) << 16)
            | (UInt32(data[offset + 3]) << 24)
        return UInt32(littleEndian: value)
    }

    private func float32(from data: Data, at offset: Int) -> Float32 {
        Float32(bitPattern: uint32(from: data, at: offset))
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

    private func makeTestGeoTilePayload(
        provider: GeoTileProvider,
        location: CLLocation,
        date: Date,
        data: Data
    ) throws -> GeoTilePayload {
        let coordinate = GeoTileCoordinate.webMercator(
            latitude: location.coordinate.latitude,
            longitude: location.coordinate.longitude,
            zoom: provider.zoom
        )
        let time = provider.tileTime(for: date)
        let sourceURL = try #require(provider.makeURL(coordinate, time))
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

        return GeoTilePayload(
            provider: provider,
            coordinate: coordinate,
            bounds: GeoTileBounds.webMercatorBounds(for: coordinate),
            deviceLocation: deviceLocation,
            time: time,
            data: data,
            sourceURL: sourceURL,
            fetchedAt: date,
            isCached: false
        )
    }

    private func assertGeoTileMessage(
        _ message: [String: Any],
        matches payload: GeoTilePayload,
        expectedKind: GeoTileLayerKind,
        includesRasterPayload: Bool
    ) throws {
        let pixel = payload.deviceLocation.pixel
        let deviceLocationJSON = try #require(message["device_location"] as? String)
        let deviceLocationData = try #require(deviceLocationJSON.data(using: .utf8))
        let decodedDeviceLocation = try JSONSerialization.jsonObject(with: deviceLocationData) as? [String: Any]
        let deviceLocation = try #require(decodedDeviceLocation)
        let decodedPixel = try #require(deviceLocation["pixel"] as? [String: Any])

        #expect(message["kind"] as? String == expectedKind.rawValue)
        #expect(message["zoom"] as? Int == payload.coordinate.z)
        #expect(message["tile_x"] as? Int == payload.coordinate.x)
        #expect(message["tile_y"] as? Int == payload.coordinate.y)
        #expect(message["device_pixel_x"] as? Double == pixel.x)
        #expect(message["device_pixel_y"] as? Double == pixel.y)
        #expect(message["tile_width"] as? Int == pixel.width)
        #expect(message["tile_height"] as? Int == pixel.height)
        #expect(message["pixel_origin"] as? String == "upper_left")
        #expect(message["pixel_units"] as? String == "pixels")
        #expect(decodedPixel["x"] as? Double == pixel.x)
        #expect(decodedPixel["y"] as? Double == pixel.y)
        #expect(decodedPixel["width"] as? Int == pixel.width)
        #expect(decodedPixel["height"] as? Int == pixel.height)

        if includesRasterPayload {
            #expect(message["encoding"] as? String == payload.provider.encoding)
            #expect(message["data"] as? String == payload.data.base64EncodedString())
        } else {
            #expect(message["encoding"] == nil)
            #expect(message["data"] == nil)
        }

        #expect(JSONSerialization.isValidJSONObject(message))
        _ = try JSONSerialization.data(withJSONObject: message, options: [])
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

    private func makeDepthFloat32PixelBuffer(width: Int, height: Int, values: [Float]) throws -> CVPixelBuffer {
        guard values.count == width * height else {
            throw TestPixelBufferError.invalidDataCount
        }

        var pixelBuffer: CVPixelBuffer?
        let attributes: [String: Any] = [
            kCVPixelBufferIOSurfacePropertiesKey as String: [:]
        ]
        let status = CVPixelBufferCreate(
            kCFAllocatorDefault,
            width,
            height,
            kCVPixelFormatType_DepthFloat32,
            attributes as CFDictionary,
            &pixelBuffer
        )

        guard status == kCVReturnSuccess, let pixelBuffer else {
            throw TestPixelBufferError.creationFailed(status)
        }

        CVPixelBufferLockBaseAddress(pixelBuffer, [])
        defer { CVPixelBufferUnlockBaseAddress(pixelBuffer, []) }

        guard let base = CVPixelBufferGetBaseAddress(pixelBuffer)?.assumingMemoryBound(to: Float32.self) else {
            throw TestPixelBufferError.missingBaseAddress
        }

        let floatsPerRow = CVPixelBufferGetBytesPerRow(pixelBuffer) / MemoryLayout<Float32>.stride
        for y in 0..<height {
            for x in 0..<width {
                base[y * floatsPerRow + x] = values[y * width + x]
            }
        }

        return pixelBuffer
    }

    private func makeLiDARConfidencePixelBuffer(width: Int, height: Int, value: UInt8) throws -> CVPixelBuffer {
        var pixelBuffer: CVPixelBuffer?
        let attributes: [String: Any] = [
            kCVPixelBufferIOSurfacePropertiesKey as String: [:]
        ]
        let status = CVPixelBufferCreate(
            kCFAllocatorDefault,
            width,
            height,
            kCVPixelFormatType_OneComponent8,
            attributes as CFDictionary,
            &pixelBuffer
        )

        guard status == kCVReturnSuccess, let pixelBuffer else {
            throw TestPixelBufferError.creationFailed(status)
        }

        CVPixelBufferLockBaseAddress(pixelBuffer, [])
        defer { CVPixelBufferUnlockBaseAddress(pixelBuffer, []) }

        guard let base = CVPixelBufferGetBaseAddress(pixelBuffer)?.assumingMemoryBound(to: UInt8.self) else {
            throw TestPixelBufferError.missingBaseAddress
        }

        let bytesPerRow = CVPixelBufferGetBytesPerRow(pixelBuffer)
        for y in 0..<height {
            memset(base + y * bytesPerRow, Int32(value), width)
        }

        return pixelBuffer
    }

    private func makeYpCbCrPixelBuffer(
        width: Int,
        height: Int,
        luma: UInt8,
        cb: UInt8,
        cr: UInt8
    ) throws -> CVPixelBuffer {
        var pixelBuffer: CVPixelBuffer?
        let attributes: [String: Any] = [
            kCVPixelBufferIOSurfacePropertiesKey as String: [:]
        ]
        let status = CVPixelBufferCreate(
            kCFAllocatorDefault,
            width,
            height,
            kCVPixelFormatType_420YpCbCr8BiPlanarFullRange,
            attributes as CFDictionary,
            &pixelBuffer
        )

        guard status == kCVReturnSuccess, let pixelBuffer else {
            throw TestPixelBufferError.creationFailed(status)
        }

        CVPixelBufferLockBaseAddress(pixelBuffer, [])
        defer { CVPixelBufferUnlockBaseAddress(pixelBuffer, []) }

        guard let yPlane = CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 0)?.assumingMemoryBound(to: UInt8.self),
              let cbcrPlane = CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 1)?.assumingMemoryBound(to: UInt8.self)
        else {
            throw TestPixelBufferError.missingBaseAddress
        }

        let yBytesPerRow = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 0)
        let yHeight = CVPixelBufferGetHeightOfPlane(pixelBuffer, 0)
        let yWidth = CVPixelBufferGetWidthOfPlane(pixelBuffer, 0)
        for y in 0..<yHeight {
            memset(yPlane + y * yBytesPerRow, Int32(luma), yWidth)
        }

        let cbcrBytesPerRow = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 1)
        let cbcrHeight = CVPixelBufferGetHeightOfPlane(pixelBuffer, 1)
        let cbcrWidth = CVPixelBufferGetWidthOfPlane(pixelBuffer, 1)
        for y in 0..<cbcrHeight {
            for x in 0..<cbcrWidth {
                let offset = y * cbcrBytesPerRow + x * 2
                cbcrPlane[offset] = cb
                cbcrPlane[offset + 1] = cr
            }
        }

        return pixelBuffer
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

    private enum TestPixelBufferError: Error {
        case invalidDataCount
        case creationFailed(CVReturn)
        case missingBaseAddress
    }
}
