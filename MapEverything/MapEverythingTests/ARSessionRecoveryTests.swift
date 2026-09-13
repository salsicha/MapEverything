import ARKit
import Foundation
import Testing
@testable import MapEverything

struct ARSessionRecoveryTests {
    @Test("Transient failures restart tracking a bounded number of times")
    func transientFailuresRestartWithBound() {
        let first = ARViewController.sessionFailureResponse(
            code: .worldTrackingFailed, restartsSoFar: 0, isScanning: true
        )
        #expect(first.action == .restartTracking)
        #expect(!first.deletesWorldMap)

        let exhausted = ARViewController.sessionFailureResponse(
            code: .worldTrackingFailed, restartsSoFar: 3, isScanning: true
        )
        #expect(exhausted.action == .surface)
    }

    @Test("An invalid resume map is deleted so it cannot brick every scan")
    func invalidWorldMapIsDeleted() {
        let response = ARViewController.sessionFailureResponse(
            code: .invalidWorldMap, restartsSoFar: 0, isScanning: true
        )
        #expect(response.deletesWorldMap)
        #expect(response.action == .restartTracking)
    }

    @Test("Camera denial surfaces actionable guidance instead of retry loops")
    func cameraDenialSurfacesGuidance() {
        let response = ARViewController.sessionFailureResponse(
            code: .cameraUnauthorized, restartsSoFar: 0, isScanning: true
        )
        #expect(response.action == .surfaceCameraGuidance)
    }

    @Test("Failures outside a scan surface without restart churn")
    func idleFailuresSurface() {
        let response = ARViewController.sessionFailureResponse(
            code: .sensorFailed, restartsSoFar: 0, isScanning: false
        )
        #expect(response.action == .surface)
    }
}
