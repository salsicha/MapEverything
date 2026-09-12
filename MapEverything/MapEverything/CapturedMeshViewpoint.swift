import Foundation
import simd

/// The camera that observed an accepted depth frame. Retains no image buffers.
nonisolated struct CapturedMeshViewpoint: Sendable {
    let transform: simd_float4x4
    let projection: simd_float4x4
    let imageAspect: Float

    /// Fit the captured field of view into the inspector without stretching it
    /// or letting distant geometry change the camera's position or zoom.
    func fittedProjection(viewportAspect: Float) -> simd_float4x4 {
        guard viewportAspect.isFinite, viewportAspect > 0,
              imageAspect.isFinite, imageAspect > 0 else { return projection }
        let x = min(1, imageAspect / viewportAspect)
        let y = min(1, viewportAspect / imageAspect)
        let fit = simd_float4x4(diagonal: SIMD4<Float>(x, y, 1, 1))
        return fit * projection
    }
}
