//
//  PointCloudStorageManager.swift
//  MapEverything
//
//

import Foundation
import RealityKit

extension FileManager {
    var cloudDocumentsURL: URL? {
        // Automatically sync user exports via CloudKit / iCloud Drive if available
        if let cloudURL = self.url(forUbiquityContainerIdentifier: nil)?.appendingPathComponent("Documents") {
            if !self.fileExists(atPath: cloudURL.path) {
                try? self.createDirectory(at: cloudURL, withIntermediateDirectories: true, attributes: nil)
            }
            return cloudURL
        }
        return self.urls(for: .documentDirectory, in: .userDomainMask).first
    }
}

class PointCloudStorageManager {
    static let shared = PointCloudStorageManager()
    
    private init() {}
    
    /// Saves a point cloud to a highly-efficient binary PLY file in the app's document directory.
    /// - Parameters:
    ///   - points: The point cloud data to save.
    ///   - filename: The name of the file (without extension).
    /// - Returns: The relative path (filename with extension) if successful, nil otherwise.
    func saveBinaryPLY(points: [ColoredPoint], to filename: String) -> String? {
        let header = """
        ply
        format binary_little_endian 1.0
        element vertex \(points.count)
        property float x
        property float y
        property float z
        property uchar red
        property uchar green
        property uchar blue
        end_header\n
        """
        
        guard let documentDirectory = FileManager.default.cloudDocumentsURL else { return nil }
        let fileNameWithExtension = "\(filename).ply"
        let fileURL = documentDirectory.appendingPathComponent(fileNameWithExtension)
        
        FileManager.default.createFile(atPath: fileURL.path, contents: nil, attributes: nil)
        guard let fileHandle = try? FileHandle(forWritingTo: fileURL) else { return nil }
        defer { try? fileHandle.close() }
        
        if let headerData = header.data(using: .utf8) {
            try? fileHandle.write(contentsOf: headerData)
        }
        
        let vertexSize = 15
        let chunkSize = 10_000 // Process in chunks of 10,000 points
        var offset = 0
        
        while offset < points.count {
            let end = min(offset + chunkSize, points.count)
            let chunk = points[offset..<end]
            var binaryData = Data(count: chunk.count * vertexSize)
            
            binaryData.withUnsafeMutableBytes { rawBuffer in
                guard let pointer = rawBuffer.bindMemory(to: UInt8.self).baseAddress else { return }
                var localOffset = 0
                for point in chunk {
                    var x = point.position.x; var y = point.position.y; var z = point.position.z
                    memcpy(pointer + localOffset, &x, 4); localOffset += 4
                    memcpy(pointer + localOffset, &y, 4); localOffset += 4
                    memcpy(pointer + localOffset, &z, 4); localOffset += 4
                    pointer[localOffset] = point.color.x; localOffset += 1
                    pointer[localOffset] = point.color.y; localOffset += 1
                    pointer[localOffset] = point.color.z; localOffset += 1
                }
            }
            try? fileHandle.write(contentsOf: binaryData)
            offset += chunkSize
        }
        
        return fileNameWithExtension
    }
    
    /// Loads a point cloud from a binary PLY file located in the app's document directory.
    /// - Parameter filename: The filename (with extension) to load.
    /// - Returns: An array of ColoredPoint representing the point cloud.
    func loadBinaryPLY(from filename: String) -> [ColoredPoint]? {
        guard let documentDirectory = FileManager.default.cloudDocumentsURL else { return nil }
        let fileURL = documentDirectory.appendingPathComponent(filename)
        
        do {
            let data = try Data(contentsOf: fileURL)
            
            guard let headerString = "end_header\n".data(using: .utf8),
                  let headerEndRange = data.range(of: headerString) else { return nil }
            
            let headerStr = String(data: data.subdata(in: 0..<headerEndRange.upperBound), encoding: .utf8) ?? ""
            let hasColor = headerStr.contains("property uchar red")
            
            let binaryData = data.subdata(in: headerEndRange.upperBound..<data.count)
            let vertexSize = hasColor ? 15 : 12
            let pointCount = binaryData.count / vertexSize
            var points: [ColoredPoint] = []
            
            points.reserveCapacity(pointCount)
            
            binaryData.withUnsafeBytes { rawBuffer in
                guard let pointer = rawBuffer.bindMemory(to: UInt8.self).baseAddress else { return }
                for i in 0..<pointCount {
                    let offset = i * vertexSize
                    var x: Float = 0, y: Float = 0, z: Float = 0
                    memcpy(&x, pointer + offset, 4)
                    memcpy(&y, pointer + offset + 4, 4)
                    memcpy(&z, pointer + offset + 8, 4)
                    
                    var r: UInt8 = 255, g: UInt8 = 255, b: UInt8 = 255
                    if hasColor {
                        r = pointer[offset + 12]
                        g = pointer[offset + 13]
                        b = pointer[offset + 14]
                    }
                    points.append(ColoredPoint(position: SIMD3<Float>(x, y, z), color: SIMD3<UInt8>(r, g, b)))
                }
            }
            return points
            
        } catch {
            print("Error loading PLY file: \(error)")
            return nil
        }
    }
}