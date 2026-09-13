import ARKit
import Foundation
import Testing
@testable import MapEverything

struct ThermalObserverIsolationTests {
    /// iOS posts thermalStateDidChangeNotification on a global queue and
    /// selector observers run on the posting thread. Fails to compile if the
    /// handler regains MainActor isolation.
    @Test("Thermal state changes accept background-queue delivery")
    func thermalHandlerIsQueueAgnostic() async {
        let controller = await MainActor.run { ARViewController() }
        await withCheckedContinuation { continuation in
            DispatchQueue.global(qos: .utility).async {
                controller.thermalStateChanged()
                continuation.resume()
            }
        }
    }
}

struct WorldMapArchiveIsolationTests {
    /// ARKit delivers getCurrentWorldMap's completion on its own queue, so
    /// the archive handler must stay nonisolated. This call fails to compile
    /// if it regains MainActor isolation, and at runtime it walks the entry
    /// prologue whose executor assertion crashed the app at Stop whenever the
    /// resume-world-map setting was enabled.
    @Test("World map archiving accepts background-queue delivery")
    func worldMapHandlerIsQueueAgnostic() async {
        let controller = await MainActor.run { ARViewController() }
        let url = FileManager.default.temporaryDirectory.appendingPathComponent(UUID().uuidString)
        await withCheckedContinuation { continuation in
            DispatchQueue.global(qos: .utility).async {
                controller.archiveCapturedWorldMap(nil, error: nil, to: url)
                continuation.resume()
            }
        }
    }
}
