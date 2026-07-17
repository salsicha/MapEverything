//
//  ColoredMeshArtifactTests.swift
//  MapEverythingTests
//

import Testing
import Foundation
import simd
@testable import MapEverything

struct ColoredMeshArtifactTests {
    private let vertices = [
        SIMD3<Float>(0, 0, 0),
        SIMD3<Float>(1, 0, 0),
        SIMD3<Float>(0, 1, 0)
    ]
    private let indices: [UInt32] = [0, 1, 2]
    private let colors = [
        SIMD3<UInt8>(255, 0, 0),
        SIMD3<UInt8>(0, 128, 0),
        SIMD3<UInt8>(0, 0, 64)
    ]

    @Test("Colored overlay artifact emits v x y z r g b lines and a metadata flag")
    func testColoredArtifactOBJ() throws {
        let artifact = LocalOverlayMeshArtifact(
            source: "test",
            coordinateFrame: "map",
            capturedAt: Date(timeIntervalSince1970: 0),
            vertices: vertices,
            indices: indices,
            colors: colors,
            metadata: [:]
        )

        #expect(artifact.hasVertexColors)
        let obj = artifact.objString()
        let vertexLines = obj.split(separator: "\n").filter { $0.hasPrefix("v ") }
        #expect(vertexLines.count == 3)

        for (index, line) in vertexLines.enumerated() {
            let parts = line.split(separator: " ")
            // "v" + x y z + r g b == 7 tokens.
            #expect(parts.count == 7)
            let r = try #require(Double(parts[4]))
            let g = try #require(Double(parts[5]))
            let b = try #require(Double(parts[6]))
            #expect(abs(r - Double(colors[index].x) / 255.0) < 1e-4)
            #expect(abs(g - Double(colors[index].y) / 255.0) < 1e-4)
            #expect(abs(b - Double(colors[index].z) / 255.0) < 1e-4)
        }

        let metadata = try JSONSerialization.jsonObject(with: artifact.metadataData()) as? [String: Any]
        #expect(metadata?["has_vertex_colors"] as? Bool == true)
    }

    @Test("Uncolored overlay artifact keeps the legacy 3-component vertex lines")
    func testUncoloredArtifactOBJ() throws {
        let artifact = LocalOverlayMeshArtifact(
            source: "test",
            coordinateFrame: "map",
            capturedAt: Date(timeIntervalSince1970: 0),
            vertices: vertices,
            indices: indices,
            metadata: [:]
        )

        #expect(!artifact.hasVertexColors)
        let obj = artifact.objString()
        let vertexLines = obj.split(separator: "\n").filter { $0.hasPrefix("v ") }
        #expect(vertexLines.count == 3)
        for line in vertexLines {
            #expect(line.split(separator: " ").count == 4) // "v" + x y z
        }

        let metadata = try JSONSerialization.jsonObject(with: artifact.metadataData()) as? [String: Any]
        #expect(metadata?["has_vertex_colors"] as? Bool == false)
    }

    @Test("Mismatched color count is treated as uncolored")
    func testMismatchedColorCountFallsBack() {
        let artifact = LocalOverlayMeshArtifact(
            source: "test",
            coordinateFrame: "map",
            capturedAt: Date(timeIntervalSince1970: 0),
            vertices: vertices,
            indices: indices,
            colors: [SIMD3<UInt8>(1, 2, 3)], // only one color for three vertices
            metadata: [:]
        )
        #expect(!artifact.hasVertexColors)
        let vertexLines = artifact.objString().split(separator: "\n").filter { $0.hasPrefix("v ") }
        for line in vertexLines {
            #expect(line.split(separator: " ").count == 4)
        }
    }
}
