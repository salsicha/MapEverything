//
//  MeshGenerator.swift
//  MapEverything
//
//

import Foundation
import ARKit
import RealityKit

struct SafeARMesh {
    let identifier: UUID
    let vertices: [SIMD3<Float>]
    let indices: [UInt32]
    let transform: simd_float4x4
}

enum MeshGenerator {
    static func createDescriptor(from geometry: ARMeshGeometry) -> MeshDescriptor {
        var desc = MeshDescriptor()
        
        // Extract vertices (positions)
        let verticesPointer = geometry.vertices.buffer.contents()
        let verticesByteOffset = geometry.vertices.offset
        let verticesByteStride = geometry.vertices.stride
        var positions: [SIMD3<Float>] = []
        positions.reserveCapacity(geometry.vertices.count)
        
        for i in 0..<geometry.vertices.count {
            let pointer = verticesPointer.advanced(by: verticesByteOffset + (i * verticesByteStride))
            let vertex = pointer.assumingMemoryBound(to: SIMD3<Float>.self).pointee
            positions.append(vertex)
        }
        desc.positions = MeshBuffers.Positions(positions)
        
        // Extract faces (indices)
        let facesPointer = geometry.faces.buffer.contents()
        let facesByteStride = geometry.faces.indexCountPerPrimitive * geometry.faces.bytesPerIndex
        var indices: [UInt32] = []
        indices.reserveCapacity(geometry.faces.count * 3)
        
        for i in 0..<geometry.faces.count {
            let pointer = facesPointer.advanced(by: i * facesByteStride)
            
            if geometry.faces.bytesPerIndex == 2 {
                let typedPointer = pointer.assumingMemoryBound(to: UInt16.self)
                indices.append(UInt32(typedPointer[0]))
                indices.append(UInt32(typedPointer[1]))
                indices.append(UInt32(typedPointer[2]))
            } else {
                let typedPointer = pointer.assumingMemoryBound(to: UInt32.self)
                indices.append(typedPointer[0])
                indices.append(typedPointer[1])
                indices.append(typedPointer[2])
            }
        }
        desc.primitives = .triangles(indices)
        
        return desc
    }
    
    /// Generates a RealityKit MeshResource from an ARKit ARMeshGeometry
    static func generateMeshResource(from geometry: ARMeshGeometry) throws -> MeshResource {
        let desc = createDescriptor(from: geometry)
        return try MeshResource.generate(from: [desc])
    }
    
    static func extractSafeMeshes(from anchors: [ARMeshAnchor]) -> [SafeARMesh] {
        return anchors.map { anchor in
            let geometry = anchor.geometry
            
            let vPointer = geometry.vertices.buffer.contents()
            let vOffset = geometry.vertices.offset
            let vStride = geometry.vertices.stride
            var vertices: [SIMD3<Float>] = []
            vertices.reserveCapacity(geometry.vertices.count)
            for i in 0..<geometry.vertices.count {
                let ptr = vPointer.advanced(by: vOffset + (i * vStride))
                vertices.append(ptr.assumingMemoryBound(to: SIMD3<Float>.self).pointee)
            }
            
            let fPointer = geometry.faces.buffer.contents()
            let fStride = geometry.faces.indexCountPerPrimitive * geometry.faces.bytesPerIndex

            var indices: [UInt32] = []
            indices.reserveCapacity(geometry.faces.count * 3)
            for i in 0..<geometry.faces.count {
                let ptr = fPointer.advanced(by: i * fStride)

                if geometry.faces.bytesPerIndex == 2 {
                    let tPtr = ptr.assumingMemoryBound(to: UInt16.self)
                    indices.append(contentsOf: [UInt32(tPtr[0]), UInt32(tPtr[1]), UInt32(tPtr[2])])
                } else {
                    let tPtr = ptr.assumingMemoryBound(to: UInt32.self)
                    indices.append(contentsOf: [tPtr[0], tPtr[1], tPtr[2]])
                }
            }
            return SafeARMesh(
                identifier: anchor.identifier,
                vertices: vertices,
                indices: indices,
                transform: anchor.transform
            )
        }
    }
}
