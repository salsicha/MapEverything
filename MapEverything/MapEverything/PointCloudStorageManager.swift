//
//  PointCloudStorageManager.swift
//  MapEverything
//
//

import Foundation

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

    /// Saves a surfel cloud as binary PLY with normals, radius, confidence, color, and observation counts.
    func saveBinaryPLY(surfels: [ColoredSurfel], to filename: String) -> String? {
        let header = """
        ply
        format binary_little_endian 1.0
        element vertex \(surfels.count)
        property float x
        property float y
        property float z
        property float normal_x
        property float normal_y
        property float normal_z
        property float radius
        property float confidence
        property uchar red
        property uchar green
        property uchar blue
        property uint observation_count
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

        let vertexSize = 39
        let chunkSize = 10_000
        var offset = 0

        while offset < surfels.count {
            let end = min(offset + chunkSize, surfels.count)
            let chunk = surfels[offset..<end]
            var binaryData = Data(count: chunk.count * vertexSize)

            binaryData.withUnsafeMutableBytes { rawBuffer in
                guard let pointer = rawBuffer.bindMemory(to: UInt8.self).baseAddress else { return }
                var localOffset = 0
                for surfel in chunk {
                    var x = surfel.position.x; var y = surfel.position.y; var z = surfel.position.z
                    var nx = surfel.normal.x; var ny = surfel.normal.y; var nz = surfel.normal.z
                    var radius = surfel.radius
                    var confidence = surfel.confidence
                    var observationCount = surfel.observationCount
                    memcpy(pointer + localOffset, &x, 4); localOffset += 4
                    memcpy(pointer + localOffset, &y, 4); localOffset += 4
                    memcpy(pointer + localOffset, &z, 4); localOffset += 4
                    memcpy(pointer + localOffset, &nx, 4); localOffset += 4
                    memcpy(pointer + localOffset, &ny, 4); localOffset += 4
                    memcpy(pointer + localOffset, &nz, 4); localOffset += 4
                    memcpy(pointer + localOffset, &radius, 4); localOffset += 4
                    memcpy(pointer + localOffset, &confidence, 4); localOffset += 4
                    pointer[localOffset] = surfel.color.x; localOffset += 1
                    pointer[localOffset] = surfel.color.y; localOffset += 1
                    pointer[localOffset] = surfel.color.z; localOffset += 1
                    memcpy(pointer + localOffset, &observationCount, 4); localOffset += 4
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
            let layout = Self.vertexLayout(from: headerStr)
            
            let binaryData = data.subdata(in: headerEndRange.upperBound..<data.count)
            let pointCount = min(layout.vertexCount, binaryData.count / layout.vertexSize)
            var points: [ColoredPoint] = []
            
            points.reserveCapacity(pointCount)
            
            binaryData.withUnsafeBytes { rawBuffer in
                guard let pointer = rawBuffer.bindMemory(to: UInt8.self).baseAddress else { return }
                for i in 0..<pointCount {
                    let offset = i * layout.vertexSize
                    var x: Float = 0, y: Float = 0, z: Float = 0
                    memcpy(&x, pointer + offset + layout.xOffset, 4)
                    memcpy(&y, pointer + offset + layout.yOffset, 4)
                    memcpy(&z, pointer + offset + layout.zOffset, 4)
                    
                    var r: UInt8 = 255, g: UInt8 = 255, b: UInt8 = 255
                    if let redOffset = layout.redOffset,
                       let greenOffset = layout.greenOffset,
                       let blueOffset = layout.blueOffset {
                        r = pointer[offset + redOffset]
                        g = pointer[offset + greenOffset]
                        b = pointer[offset + blueOffset]
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

    private struct PLYVertexLayout {
        let vertexCount: Int
        let vertexSize: Int
        let xOffset: Int
        let yOffset: Int
        let zOffset: Int
        let redOffset: Int?
        let greenOffset: Int?
        let blueOffset: Int?
    }

    private static func vertexLayout(from header: String) -> PLYVertexLayout {
        var vertexCount = 0
        var inVertexElement = false
        var offset = 0
        var xOffset = 0
        var yOffset = 4
        var zOffset = 8
        var redOffset: Int?
        var greenOffset: Int?
        var blueOffset: Int?

        for line in header.split(separator: "\n").map(String.init) {
            let parts = line.split(separator: " ").map(String.init)
            guard !parts.isEmpty else { continue }

            if parts.count >= 3, parts[0] == "element" {
                inVertexElement = parts[1] == "vertex"
                if inVertexElement {
                    vertexCount = Int(parts[2]) ?? 0
                    offset = 0
                }
                continue
            }

            guard inVertexElement, parts.count >= 3, parts[0] == "property" else { continue }

            let type = parts[1]
            let name = parts[2]
            switch name {
            case "x": xOffset = offset
            case "y": yOffset = offset
            case "z": zOffset = offset
            case "red": redOffset = offset
            case "green": greenOffset = offset
            case "blue": blueOffset = offset
            default: break
            }
            offset += byteCount(forPLYType: type)
        }

        return PLYVertexLayout(
            vertexCount: vertexCount,
            vertexSize: max(offset, 12),
            xOffset: xOffset,
            yOffset: yOffset,
            zOffset: zOffset,
            redOffset: redOffset,
            greenOffset: greenOffset,
            blueOffset: blueOffset
        )
    }

    private static func byteCount(forPLYType type: String) -> Int {
        switch type {
        case "char", "uchar", "int8", "uint8":
            return 1
        case "short", "ushort", "int16", "uint16":
            return 2
        case "float", "int", "uint", "float32", "int32", "uint32":
            return 4
        case "double", "float64", "int64", "uint64":
            return 8
        default:
            return 4
        }
    }
}
