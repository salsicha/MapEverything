import Foundation
import Testing
@testable import MapEverything

/// Regression coverage for the frozen Start button: Stop queued the whole
/// scan's OBJ/PLY/LAS export on the recording queue, and start() is a
/// queue.sync from the main thread - measured at 38.5 s of blocked main
/// thread on the simulator for a street-scale (400k-vertex) mesh.
struct FinalArtifactExportTests {
    private func streetScaleMesh(vertexCount: Int) -> (LocalOverlayMeshArtifact, LocalPointCloudArtifact) {
        var vertices: [SIMD3<Float>] = []
        var colors: [SIMD3<UInt8>] = []
        vertices.reserveCapacity(vertexCount)
        colors.reserveCapacity(vertexCount)
        for i in 0..<vertexCount {
            vertices.append(SIMD3(Float(i % 700) * 0.013, Float(i / 700) * 0.017, -Float(i % 97) * 0.21))
            colors.append(SIMD3(UInt8(i % 255), UInt8((i / 3) % 255), UInt8((i / 7) % 255)))
        }
        var indices: [UInt32] = []
        indices.reserveCapacity(vertexCount * 3)
        for i in 0..<(vertexCount * 3 / 2) {
            let base = UInt32(i % (vertexCount - 2))
            indices += [base, base + 1, base + 2]
        }
        let mesh = LocalOverlayMeshArtifact(
            source: "test", coordinateFrame: "map", capturedAt: Date(),
            vertices: vertices, indices: indices, colors: colors, metadata: [:]
        )
        let cloud = LocalPointCloudArtifact(
            source: "test", coordinateFrame: "map", capturedAt: Date(),
            points: zip(vertices, colors).map { LocalPointCloudArtifact.Point(position: $0.0, color: $0.1) },
            metadata: [:]
        )
        return (mesh, cloud)
    }

    @Test("Starting a new recording never waits for the previous scan's export")
    func startDoesNotWaitForPendingExports() throws {
        let root = FileManager.default.temporaryDirectory
            .appendingPathComponent("FinalArtifactExport-\(UUID().uuidString)")
        defer { try? FileManager.default.removeItem(at: root) }
        let recorder = LocalROS2BagRecorder(baseDirectoryURL: root)
        let configuration = LocalROS2BagRecorderConfiguration(isEnabled: true, maxChunkBytes: 8 * 1_048_576)
        recorder.start(sessionID: UUID(), configuration: configuration)
        let finishedScan = try #require(recorder.currentArtifactDirectoryURL)
        recorder.stop()

        let (mesh, cloud) = streetScaleMesh(vertexCount: 400_000)
        recorder.recordFinalOverlayMesh(mesh, in: finishedScan)
        recorder.recordFinalPointCloud(cloud, in: finishedScan)

        let clock = ContinuousClock()
        let started = clock.now
        recorder.start(sessionID: UUID(), configuration: configuration)
        let startLatency = clock.now - started
        #expect(startLatency < .seconds(1), "start() blocked \(startLatency) behind the previous scan's export")
        #expect(recorder.currentArtifactDirectoryURL != finishedScan)

        // The export still completes, into the FINISHED scan's directory.
        recorder.waitForFinalArtifacts()
        recorder.stopAndWait()
        for name in [LocalOverlayMeshArtifact.objFileName, LocalPointCloudArtifact.plyFileName,
                     LocalPointCloudArtifact.lasFileName] {
            #expect(FileManager.default.fileExists(atPath: finishedScan.appendingPathComponent(name).path), "\(name)")
        }
    }

    @Test("The fast six-decimal formatter matches printf %.6f exactly")
    func sixDecimalMatchesPrintf() {
        let posix = Locale(identifier: "en_US_POSIX")
        func reference(_ value: Float) -> String {
            String(format: "%.6f", locale: posix, Double(value.isFinite ? value : 0))
        }
        var edgeCases: [Float] = [
            0, -0.0, 1, -1, 0.5, 0.0078125, -0.0078125,      // exact half-unit ties
            1e-7, -1e-7, 4.9999995e-7, 5.0000005e-7,          // round to +/-0 keeps the sign
            0.9999995, 9.9999995, 999_999.94, -123.4567891,
            .infinity, -.infinity, .nan,                      // non-finite map to 0
            .leastNonzeroMagnitude, -.leastNonzeroMagnitude,
            .greatestFiniteMagnitude, 8.9e9, 9.1e9            // beyond the fast path
        ]
        // Every dyadic tie k/128 across a few units, plus a dense random sweep.
        for k in -512...512 { edgeCases.append(Float(k) / 128) }
        var generator = SystemRandomNumberGenerator()
        for _ in 0..<200_000 {
            edgeCases.append(Float.random(in: -50...50, using: &generator))
        }
        for _ in 0..<20_000 {
            edgeCases.append(Float(bitPattern: UInt32.random(in: 0...UInt32.max, using: &generator)))
        }
        var mismatches: [String] = []
        for value in edgeCases where SixDecimalFormat.string(value) != reference(value) {
            mismatches.append("\(value): \(SixDecimalFormat.string(value)) vs \(reference(value))")
            if mismatches.count > 5 { break }
        }
        #expect(mismatches.isEmpty, "\(mismatches)")
        for channel in 0...255 {
            #expect(SixDecimalFormat.colorChannel[channel]
                    == String(format: "%.6f", locale: posix, Double(channel) / 255.0))
        }
    }
}
