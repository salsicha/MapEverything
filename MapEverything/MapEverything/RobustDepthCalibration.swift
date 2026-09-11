import Foundation

nonisolated struct DepthCalibrationSample {
    let relative: Double
    let depth: Double
    let weight: Double
}

/// Fits only the current prediction image: relative inverse-depth coefficients
/// are not transferable between independently normalized network outputs.
/// Constrain a>0, b>=0 in inverse_depth=a*relative+b. This regularizes far
/// extrapolation without masking pixels outside the LiDAR support. It is a
/// modeling choice: if this family cannot explain the LiDAR overlap within
/// the residual tolerance, reject the calibration rather than invent scale.
nonisolated enum RobustDepthCalibration {
    private typealias Calibration = DepthAnythingProcessor.MaximumLikelihoodCalibration

    static func fit(_ input: [DepthCalibrationSample]) -> DepthAnythingProcessor.MaximumLikelihoodCalibration? {
        let samples = input.filter {
            $0.relative.isFinite && $0.relative > 0 && $0.depth.isFinite && $0.depth > 0 &&
            $0.weight.isFinite && $0.weight > 0
        }
        guard samples.count > 50 else { return nil }
        var best: [DepthCalibrationSample] = []
        var bestError = Double.infinity

        func consider(_ fit: Calibration) {
            var inliers: [DepthCalibrationSample] = []
            var error = 0.0
            for sample in samples {
                let inverse = Double(fit.scale) * sample.relative + Double(fit.offset)
                guard inverse > 0 else { continue }
                let residual = abs(1 / inverse - sample.depth)
                // Metric residuals prevent long-range, strongly weighted bad
                // LiDAR returns from dominating the inverse-depth regression.
                if residual <= max(0.06, sample.depth * 0.08) {
                    inliers.append(sample)
                    error += residual / sample.depth
                }
            }
            if inliers.count > best.count || (inliers.count == best.count && error < bestError) {
                best = inliers
                bestError = error
            }
        }

        if let initial = leastSquares(samples) { consider(initial) }
        // Fixed-size deterministic RANSAC keeps calibration work bounded.
        var random: UInt64 = 0x5eed
        func nextIndex() -> Int {
            random = random &* 6364136223846793005 &+ 1
            return Int((random >> 32) % UInt64(samples.count))
        }
        for _ in 0..<64 {
            let a = samples[nextIndex()]
            let b = samples[nextIndex()]
            guard abs(a.relative - b.relative) > 1e-8 else { continue }
            let scale = (1 / a.depth - 1 / b.depth) / (a.relative - b.relative)
            let offset = 1 / a.depth - scale * a.relative
            guard scale.isFinite, scale > 0, offset.isFinite else { continue }
            if offset >= 0 {
                consider(Calibration(scale: Float(scale), offset: Float(offset)))
            }
            // Include the boundary of the constrained fit. Merely discarding
            // negative-offset candidates would miss a valid scale-only fit.
            consider(Calibration(scale: Float(1 / (a.depth * a.relative)), offset: 0))
        }

        guard best.count > 50, Double(best.count) >= Double(samples.count) * 0.65 else { return nil }
        // Nearly constant measured depth cannot constrain both scale and offset.
        let depths = best.map(\.depth).sorted()
        let span = depths[depths.count * 9 / 10] - depths[depths.count / 10]
        guard span >= max(0.1, depths[depths.count / 2] * 0.08),
              let result = leastSquares(best) else { return nil }
        let goodCount = best.reduce(0) { count, sample in
            let inverse = Double(result.scale) * sample.relative + Double(result.offset)
            return count + (inverse > 0 && abs(1 / inverse - sample.depth) <= max(0.06, sample.depth * 0.08) ? 1 : 0)
        }
        guard Double(goodCount) >= Double(best.count) * 0.9 else { return nil }
        return result
    }

    private static func leastSquares(_ samples: [DepthCalibrationSample]) -> Calibration? {
        let weight = samples.reduce(0) { $0 + $1.weight }
        guard weight > 0 else { return nil }
        let meanX = samples.reduce(0) { $0 + $1.weight * $1.relative } / weight
        let meanY = samples.reduce(0) { $0 + $1.weight / $1.depth } / weight
        var variance = 0.0
        var covariance = 0.0
        for sample in samples {
            let delta = sample.relative - meanX
            variance += sample.weight * delta * delta
            covariance += sample.weight * delta * (1 / sample.depth - meanY)
        }
        guard variance > weight * max(meanX * meanX, 1e-12) * 1e-7 else { return nil }
        var scale = covariance / variance
        var offset = meanY - scale * meanX
        if offset < 0 {
            // A negative offset introduces a pole at a positive network value.
            // Solve the weighted least-squares boundary b=0, rather than clamp
            // b after fitting or crop the distant part of the prediction.
            let xx = samples.reduce(0) { $0 + $1.weight * $1.relative * $1.relative }
            let xy = samples.reduce(0) { $0 + $1.weight * $1.relative / $1.depth }
            guard xx > 0 else { return nil }
            scale = xy / xx
            offset = 0
        }
        guard scale.isFinite, scale > 0, offset.isFinite,
              Float(scale).isFinite, Float(offset).isFinite else { return nil }
        return Calibration(scale: Float(scale), offset: Float(offset))
    }
}
