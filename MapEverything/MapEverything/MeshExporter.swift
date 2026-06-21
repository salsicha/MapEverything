//
//  MeshExporter.swift
//  MapEverything
//

import Foundation
import ARKit
import ModelIO

class MeshExporter {
    /// Converts a collection of safe mesh data into an .obj file and a native iOS .usdz file.
    static func exportToOBJAndUSDZ(safeMeshes: [SafeARMesh], filename: String) -> (objURL: URL, usdzURL: URL?)? {
        guard let docDir = FileManager.default.cloudDocumentsURL else { return nil }
        let fileURL = docDir.appendingPathComponent("\(filename).obj")
        
        FileManager.default.createFile(atPath: fileURL.path, contents: nil, attributes: nil)
        guard let fileHandle = try? FileHandle(forWritingTo: fileURL) else { return nil }
        
        if let headerData = "# MapEverything LiDAR Mesh Export\n".data(using: .utf8) {
            try? fileHandle.write(contentsOf: headerData)
        }
        
        var vertexOffset: UInt32 = 1 // OBJ indices are 1-based and global across shapes
        for mesh in safeMeshes {
            
            var vertexData = ""
            for (i, vertex) in mesh.vertices.enumerated() {
                
                // Transform local vertex to world space coordinates
                let vertex4 = simd_float4(vertex.x, vertex.y, vertex.z, 1.0)
                let worldVertex = simd_mul(mesh.transform, vertex4)
                
                vertexData.append("v \(worldVertex.x) \(worldVertex.y) \(worldVertex.z)\n")
                
                // Flush to disk to avoid high memory usage
                if i > 0 && i % 2000 == 0 {
                    if let data = vertexData.data(using: .utf8) { try? fileHandle.write(contentsOf: data) }
                    vertexData = ""
                }
            }
            if !vertexData.isEmpty {
                if let data = vertexData.data(using: .utf8) { try? fileHandle.write(contentsOf: data) }
            }
            
            var faceData = ""
            for i in stride(from: 0, to: mesh.indices.count, by: 3) {
                let v1 = mesh.indices[i]
                let v2 = mesh.indices[i+1]
                let v3 = mesh.indices[i+2]
                
                faceData.append("f \(v1 + vertexOffset) \(v2 + vertexOffset) \(v3 + vertexOffset)\n")
                
                if i > 0 && (i / 3) % 2000 == 0 {
                    if let data = faceData.data(using: .utf8) { try? fileHandle.write(contentsOf: data) }
                    faceData = ""
                }
            }
            if !faceData.isEmpty {
                if let data = faceData.data(using: .utf8) { try? fileHandle.write(contentsOf: data) }
            }
            
            vertexOffset += UInt32(mesh.vertices.count)
        }
        
        // Explicitly close the file handle so the contents are flushed to disk before MDLAsset reads it
        try? fileHandle.close()
        
        var usdzURL: URL? = nil
        if let asset = MDLAsset(url: fileURL) as MDLAsset? {
            
            // Inject a PBR material so the LiDAR mesh catches real-world lighting and shadows in QuickLook
            let scatteringFunction = MDLPhysicallyPlausibleScatteringFunction()
            let material = MDLMaterial(name: "LiDARMaterial", scatteringFunction: scatteringFunction)
            let color = CGColor(srgbRed: 0.8, green: 0.8, blue: 0.8, alpha: 1.0)
            let colorProperty = MDLMaterialProperty(name: "baseColor", semantic: .baseColor, color: color)
            material.setProperty(colorProperty)
            
            for i in 0..<asset.count {
                if let mesh = asset.object(at: i) as? MDLMesh {
                    for submesh in mesh.submeshes ?? [] {
                        if let sub = submesh as? MDLSubmesh { sub.material = material }
                    }
                }
            }
            
            let usdzFile = docDir.appendingPathComponent("\(filename).usdz")
            if (try? asset.export(to: usdzFile)) != nil {
                usdzURL = usdzFile
            }
        }
        
        return (fileURL, usdzURL)
    }
}
