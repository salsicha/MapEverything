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
}
