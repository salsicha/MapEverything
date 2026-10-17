import Foundation
import simd

/// Fixed three-view graph contract. Kept separate from Core ML for replay tests.
nonisolated enum DA3WindowGeometry {
    static let enabledStorageKey = "da3MultiViewEnabled"
    static let windowSize = 3
    static let width = 518
    static let height = 392
    static let maximumAge: TimeInterval = 5
    static let minimumTranslation: Float = 0.04
    static let minimumRotation: Float = 3 * .pi / 180
    static let maximumTranslation: Float = 1.5
    static let maximumRotation: Float = 45 * .pi / 180

    struct Camera: Sendable {
        let timestamp: TimeInterval
        let transform: simd_float4x4 // ARKit camera to world, native sensor axes
        let intrinsics: simd_float3x3
        let resolution: CGSize
    }

    static func translation(_ a: Camera, _ b: Camera) -> Float {
        simd_length(a.transform.columns.3.xyz - b.transform.columns.3.xyz)
    }

    static func rotation(_ a: Camera, _ b: Camera) -> Float {
        let qa = simd_quatf(a.transform), qb = simd_quatf(b.transform)
        return 2 * acos(min(1, abs(simd_dot(qa.vector, qb.vector))))
    }

    static func isCompatible(_ older: Camera, with current: Camera) -> Bool {
        let age = current.timestamp - older.timestamp
        return age > 0 && age <= maximumAge
            && older.resolution.width == current.resolution.width
            && older.resolution.height == current.resolution.height
            && translation(older, current) <= maximumTranslation
            && rotation(older, current) <= maximumRotation
    }

    static func isKeyframe(_ current: Camera, after previous: Camera) -> Bool {
        translation(previous, current) >= minimumTranslation
            || rotation(previous, current) >= minimumRotation
    }

    /// Mirrors DA3 API normalization + extri_intri_to_pose_encoding. Normalizing
    /// poses does NOT set depth scale: same-frame LiDAR does that after inference.
    static func poseEncoding(_ cameras: [Camera]) -> [Float]? {
        guard cameras.count == windowSize,
              cameras.allSatisfy({ $0.resolution.width > 0 && $0.resolution.height > 0
                  && $0.intrinsics[0, 0] > 0 && $0.intrinsics[1, 1] > 0 }) else { return nil }
        // Upright CV axes = clockwise image rotation * ARKit-to-CV axes.
        let uprightFromAR = simd_float4x4(columns: (
            SIMD4(0, 1, 0, 0), SIMD4(1, 0, 0, 0),
            SIMD4(0, 0, -1, 0), SIMD4(0, 0, 0, 1)))
        let firstWorldToUpright = uprightFromAR * cameras[0].transform.inverse
        let relative = cameras.map { firstWorldToUpright * $0.transform * uprightFromAR.inverse }
        let distances = relative.map { simd_length($0.columns.3.xyz) }.sorted()
        let scale = max(0.1, distances[1])
        var result: [Float] = []
        for (camera, pose) in zip(cameras, relative) {
            let t = pose.columns.3.xyz / scale
            var q = simd_quatf(pose).vector // XYZW, standardized just like DA3
            if q.w < 0 { q = -q }
            // Rotating swaps sensor x/y focal lengths; resize cancels in FOV.
            let fovH = 2 * atan(Float(camera.resolution.width) / (2 * camera.intrinsics[0, 0]))
            let fovW = 2 * atan(Float(camera.resolution.height) / (2 * camera.intrinsics[1, 1]))
            result += [t.x, t.y, t.z, q.x, q.y, q.z, q.w, fovH, fovW]
        }
        return result.allSatisfy(\.isFinite) ? result : nil
    }
}

private nonisolated extension SIMD4 where Scalar == Float {
    var xyz: SIMD3<Float> { SIMD3(x, y, z) }
}
