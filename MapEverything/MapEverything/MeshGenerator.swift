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
    struct DepthAnythingMeshConfiguration {
        let step: Int
        let minimumDepth: Float
        let maximumDepth: Float
        let maximumDepthDiscontinuity: Float
        let maximumTriangleCount: Int

        static let overlay = DepthAnythingMeshConfiguration(
            step: 12,
            minimumDepth: 0.1,
            maximumDepth: 8.0,
            maximumDepthDiscontinuity: 0.45,
            maximumTriangleCount: 8_000
        )
    }

    struct DepthAnythingMeshSnapshot {
        let descriptor: MeshDescriptor
        let vertices: [SIMD3<Float>]
        let indices: [UInt32]
    }

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

    static func createDepthAnythingDescriptor(
        from relativeDepthMap: RelativeDepthMap,
        calibration: DepthAnythingProcessor.MaximumLikelihoodCalibration,
        intrinsics: simd_float3x3,
        imageResolution resolution: CGSize,
        transform: simd_float4x4,
        configuration: DepthAnythingMeshConfiguration = .overlay
    ) -> MeshDescriptor? {
        createDepthAnythingMeshSnapshot(
            from: relativeDepthMap,
            calibration: calibration,
            intrinsics: intrinsics,
            imageResolution: resolution,
            transform: transform,
            configuration: configuration
        )?.descriptor
    }

    static func createDepthAnythingMeshSnapshot(
        from relativeDepthMap: RelativeDepthMap,
        calibration: DepthAnythingProcessor.MaximumLikelihoodCalibration,
        intrinsics: simd_float3x3,
        imageResolution resolution: CGSize,
        transform: simd_float4x4,
        configuration: DepthAnythingMeshConfiguration = .overlay
    ) -> DepthAnythingMeshSnapshot? {
        let depthWidth = relativeDepthMap.width
        let depthHeight = relativeDepthMap.height
        let step = max(1, configuration.step)

        guard depthWidth > 1,
              depthHeight > 1,
              resolution.width > 0,
              resolution.height > 0 else {
            return nil
        }

        let scaleX = Float(depthWidth) / Float(resolution.width)
        let scaleY = Float(depthHeight) / Float(resolution.height)
        let fx = intrinsics[0][0] * scaleX
        let fy = intrinsics[1][1] * scaleY
        let cx = intrinsics[2][0] * scaleX
        let cy = intrinsics[2][1] * scaleY
        guard fx.isFinite,
              fy.isFinite,
              cx.isFinite,
              cy.isFinite,
              abs(fx) > 1e-5,
              abs(fy) > 1e-5 else {
            return nil
        }

        return relativeDepthMap.withReadAccess { reader in
            let columns = sampledIndices(length: depthWidth, step: step)
            let rows = sampledIndices(length: depthHeight, step: step)
            guard columns.count > 1, rows.count > 1 else { return nil }

            var positions: [SIMD3<Float>] = []
            var gridIndices = [Int32](repeating: -1, count: rows.count * columns.count)
            var gridDepths = [Float](repeating: .nan, count: rows.count * columns.count)
            positions.reserveCapacity(rows.count * columns.count)

            for (rowIndex, y) in rows.enumerated() {
                for (columnIndex, x) in columns.enumerated() {
                    let relativeDepth = reader.value(atX: x, y: y)
                    let depth = calibration.scale * relativeDepth + calibration.offset
                    guard depth.isFinite,
                          depth >= configuration.minimumDepth,
                          depth <= configuration.maximumDepth else {
                        continue
                    }

                    let pointCamera = simd_float4(
                        ((Float(x) - cx) / fx) * depth,
                        ((cy - Float(y)) / fy) * depth,
                        -depth,
                        1.0
                    )
                    let pointWorld = simd_mul(transform, pointCamera)
                    let gridIndex = rowIndex * columns.count + columnIndex
                    gridIndices[gridIndex] = Int32(positions.count)
                    gridDepths[gridIndex] = depth
                    positions.append(SIMD3<Float>(pointWorld.x, pointWorld.y, pointWorld.z))
                }
            }

            guard positions.count >= 3 else { return nil }

            var indices: [UInt32] = []
            indices.reserveCapacity(min(configuration.maximumTriangleCount, rows.count * columns.count * 2) * 3)

            for rowIndex in 0..<(rows.count - 1) {
                guard indices.count / 3 < configuration.maximumTriangleCount else { break }

                for columnIndex in 0..<(columns.count - 1) {
                    guard indices.count / 3 < configuration.maximumTriangleCount else { break }

                    let upperLeft = rowIndex * columns.count + columnIndex
                    let upperRight = upperLeft + 1
                    let lowerLeft = upperLeft + columns.count
                    let lowerRight = lowerLeft + 1

                    appendDepthTriangleIfContinuous(
                        gridIndices[upperLeft],
                        gridIndices[lowerLeft],
                        gridIndices[upperRight],
                        depths: [
                            gridDepths[upperLeft],
                            gridDepths[lowerLeft],
                            gridDepths[upperRight]
                        ],
                        maximumDepthDiscontinuity: configuration.maximumDepthDiscontinuity,
                        indices: &indices
                    )
                    guard indices.count / 3 < configuration.maximumTriangleCount else { break }

                    appendDepthTriangleIfContinuous(
                        gridIndices[upperRight],
                        gridIndices[lowerLeft],
                        gridIndices[lowerRight],
                        depths: [
                            gridDepths[upperRight],
                            gridDepths[lowerLeft],
                            gridDepths[lowerRight]
                        ],
                        maximumDepthDiscontinuity: configuration.maximumDepthDiscontinuity,
                        indices: &indices
                    )
                }
            }

            guard indices.count >= 3 else { return nil }

            var descriptor = MeshDescriptor()
            descriptor.positions = MeshBuffers.Positions(positions)
            descriptor.primitives = .triangles(indices)
            return DepthAnythingMeshSnapshot(
                descriptor: descriptor,
                vertices: positions,
                indices: indices
            )
        }
    }

    private static func sampledIndices(length: Int, step: Int) -> [Int] {
        guard length > 0 else { return [] }

        var indices = Array(stride(from: 0, to: length, by: step))
        let lastIndex = length - 1
        if indices.last != lastIndex {
            indices.append(lastIndex)
        }
        return indices
    }

    private static func appendDepthTriangleIfContinuous(
        _ a: Int32,
        _ b: Int32,
        _ c: Int32,
        depths: [Float],
        maximumDepthDiscontinuity: Float,
        indices: inout [UInt32]
    ) {
        guard a >= 0, b >= 0, c >= 0 else { return }
        guard let minimumDepth = depths.min(),
              let maximumDepth = depths.max(),
              minimumDepth.isFinite,
              maximumDepth.isFinite else {
            return
        }

        let allowedJump = max(maximumDepthDiscontinuity, minimumDepth * 0.18)
        guard maximumDepth - minimumDepth <= allowedJump else { return }

        indices.append(UInt32(a))
        indices.append(UInt32(b))
        indices.append(UInt32(c))
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
