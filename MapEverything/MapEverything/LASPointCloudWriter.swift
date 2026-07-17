//
//  LASPointCloudWriter.swift
//  MapEverything
//

import Foundation
import simd

/// Serializes colored points as little-endian LAS 1.2 using Point Data Record
/// Format 2 (XYZ + intensity + flags + classification + scan angle + user
/// data + point source ID + RGB) so the final cloud opens directly in GIS and
/// photogrammetry tools alongside the PLY artifact.
enum LASPointCloudWriter {
    static let versionMajor: UInt8 = 1
    static let versionMinor: UInt8 = 2
    static let pointDataFormat: UInt8 = 2
    static let pointRecordLength: UInt16 = 26
    static let headerSize: UInt16 = 227

    private static let systemIdentifier = "OTHER"
    private static let generatingSoftware = "MapEverything"
    // LAS quantizes coordinates to int32. The extent-based scale keeps large
    // clouds inside +/-2e9 counts; the floor keeps 0.1mm precision for the
    // room-scale clouds this app produces.
    private static let minimumScale = 0.0001
    private static let quantizationRange = 2e9

    static func lasData(points: [ColoredPoint], sourceID: UInt16 = 0) -> Data? {
        let finitePoints = points.filter { point in
            point.position.x.isFinite && point.position.y.isFinite && point.position.z.isFinite
        }
        guard !finitePoints.isEmpty else { return nil }

        var minBounds = doublePosition(of: finitePoints[0])
        var maxBounds = minBounds
        for point in finitePoints.dropFirst() {
            let position = doublePosition(of: point)
            minBounds = simd_min(minBounds, position)
            maxBounds = simd_max(maxBounds, position)
        }

        let extent = maxBounds - minBounds
        let scale = SIMD3<Double>(
            max(extent.x / quantizationRange, minimumScale),
            max(extent.y / quantizationRange, minimumScale),
            max(extent.z / quantizationRange, minimumScale)
        )
        let offset = minBounds
        let pointCount = UInt32(clamping: finitePoints.count)

        var data = Data(capacity: Int(headerSize) + finitePoints.count * Int(pointRecordLength))

        // Public header block (227 bytes).
        data.append(contentsOf: Array("LASF".utf8))
        data.appendUInt16LE(sourceID)                       // file source ID
        data.appendUInt16LE(0)                              // global encoding
        data.appendUInt32LE(0)                              // project GUID 1
        data.appendUInt16LE(0)                              // project GUID 2
        data.appendUInt16LE(0)                              // project GUID 3
        data.append(contentsOf: [UInt8](repeating: 0, count: 8)) // project GUID 4
        data.append(versionMajor)
        data.append(versionMinor)
        data.appendPaddedASCII(systemIdentifier, length: 32)
        data.appendPaddedASCII(generatingSoftware, length: 32)
        let creation = creationDayAndYear()
        data.appendUInt16LE(creation.dayOfYear)
        data.appendUInt16LE(creation.year)
        data.appendUInt16LE(headerSize)
        data.appendUInt32LE(UInt32(headerSize))             // offset to point data
        data.appendUInt32LE(0)                              // number of VLRs
        data.append(pointDataFormat)
        data.appendUInt16LE(pointRecordLength)
        data.appendUInt32LE(pointCount)
        data.appendUInt32LE(pointCount)                     // points by return, first entry
        for _ in 0..<4 {
            data.appendUInt32LE(0)
        }
        data.appendFloat64LE(scale.x)
        data.appendFloat64LE(scale.y)
        data.appendFloat64LE(scale.z)
        data.appendFloat64LE(offset.x)
        data.appendFloat64LE(offset.y)
        data.appendFloat64LE(offset.z)
        data.appendFloat64LE(maxBounds.x)
        data.appendFloat64LE(minBounds.x)
        data.appendFloat64LE(maxBounds.y)
        data.appendFloat64LE(minBounds.y)
        data.appendFloat64LE(maxBounds.z)
        data.appendFloat64LE(minBounds.z)

        for point in finitePoints {
            let position = doublePosition(of: point)
            data.appendInt32LE(Int32(((position.x - offset.x) / scale.x).rounded()))
            data.appendInt32LE(Int32(((position.y - offset.y) / scale.y).rounded()))
            data.appendInt32LE(Int32(((position.z - offset.z) / scale.z).rounded()))
            data.appendUInt16LE(0)                          // intensity (not captured)
            data.append(0b0000_1001)                        // return 1 of 1
            data.append(0)                                  // classification: never classified
            data.append(0)                                  // scan angle rank
            data.append(0)                                  // user data
            data.appendUInt16LE(sourceID)                   // point source ID
            data.appendUInt16LE(UInt16(point.color.x) * 257)
            data.appendUInt16LE(UInt16(point.color.y) * 257)
            data.appendUInt16LE(UInt16(point.color.z) * 257)
        }

        return data
    }

    private static func doublePosition(of point: ColoredPoint) -> SIMD3<Double> {
        SIMD3<Double>(
            Double(point.position.x),
            Double(point.position.y),
            Double(point.position.z)
        )
    }

    private static func creationDayAndYear(date: Date = Date()) -> (dayOfYear: UInt16, year: UInt16) {
        var calendar = Calendar(identifier: .gregorian)
        calendar.timeZone = TimeZone(secondsFromGMT: 0) ?? calendar.timeZone
        let dayOfYear = calendar.ordinality(of: .day, in: .year, for: date) ?? 0
        let year = calendar.component(.year, from: date)
        return (UInt16(clamping: dayOfYear), UInt16(clamping: year))
    }
}

private extension Data {
    mutating func appendUInt16LE(_ value: UInt16) {
        Swift.withUnsafeBytes(of: value.littleEndian) { append(contentsOf: $0) }
    }

    mutating func appendUInt32LE(_ value: UInt32) {
        Swift.withUnsafeBytes(of: value.littleEndian) { append(contentsOf: $0) }
    }

    mutating func appendInt32LE(_ value: Int32) {
        Swift.withUnsafeBytes(of: value.littleEndian) { append(contentsOf: $0) }
    }

    mutating func appendFloat64LE(_ value: Double) {
        Swift.withUnsafeBytes(of: value.bitPattern.littleEndian) { append(contentsOf: $0) }
    }

    mutating func appendPaddedASCII(_ string: String, length: Int) {
        var bytes = Array(string.utf8.prefix(length))
        bytes.append(contentsOf: [UInt8](repeating: 0, count: length - bytes.count))
        append(contentsOf: bytes)
    }
}
