import Foundation
import MetricKit
import Testing
@testable import MapEverything

struct MetricKitDiagnosticsTests {
    /// MetricKit delivers payloads on its own queue through an ObjC thunk, so
    /// the subscriber must accept invocation from any thread. This call fails
    /// to compile if MetricKitDiagnostics regains MainActor isolation, and at
    /// runtime it walks the same entry prologue whose executor assertion
    /// crash-looped the app on device (MapEverything-2026-09-12-18*.ips).
    @Test("MetricKit delivery callbacks accept background-queue invocation")
    func metricKitCallbacksAreQueueAgnostic() async {
        let diagnostics = MetricKitDiagnostics.shared
        await withCheckedContinuation { continuation in
            DispatchQueue.global(qos: .utility).async {
                diagnostics.didReceive([] as [MXDiagnosticPayload])
                diagnostics.didReceive([] as [MXMetricPayload])
                continuation.resume()
            }
        }
    }

    /// The unit-test host must not boot the live AR interface: on a physical
    /// device the camera, RealityKit rendering, and CoreML warm-up saturate
    /// the main queue and starve main-queue work that tests schedule.
    @Test("The app detects that it is hosting unit tests")
    @MainActor
    func unitTestHostIsDetected() {
        #expect(MapEverythingApp.isHostingUnitTests)
    }
}
