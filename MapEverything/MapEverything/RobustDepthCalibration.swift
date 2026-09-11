import Foundation

nonisolated struct DepthCalibrationSample {
    let relative: Double
    let depth: Double
    let weight: Double
}

/// Fits only the current prediction image: relative inverse-depth coefficients
/// are not transferable between independently normalized network outputs.
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
            consider(Calibration(scale: Float(scale), offset: Float(offset)))
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
        let supportedSamples = best.filter { sample in
            let inverse = Double(result.scale) * sample.relative + Double(result.offset)
            return inverse > 0 && abs(1 / inverse - sample.depth) <= max(0.06, sample.depth * 0.08)
        }
        guard let support = DepthCalibrationSupport(samples: supportedSamples, scale: result.scale, offset: result.offset) else {
            return nil
        }
        return Calibration(scale: result.scale, offset: result.offset, support: support)
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
        let scale = covariance / variance
        let offset = meanY - scale * meanX
        guard scale.isFinite, scale > 0, offset.isFinite,
              Float(scale).isFinite, Float(offset).isFinite else { return nil }
        return Calibration(scale: Float(scale), offset: Float(offset))
    }
}

/// Conservative extrapolation gate, not a claim of statistical confidence.
/// Pixel errors are correlated, so the error envelope does not shrink with N.
/// Its growth outside the observed relative-depth spread prevents a good local
/// fit from turning a near-zero inverse-depth denominator into distant sheets.
nonisolated struct DepthCalibrationSupport: Sendable {
    let relativeMean: Float
    let relativeVariance: Float
    let inverseErrorVariance: Float

    static let maximumRelativeDepthError: Float = 0.25
    // Require the upper-depth endpoint 1/(inverse - 2*sigma) to differ by <=25%.
    // Squaring avoids a square root per pixel, including in the Metal kernel.
    static let inverseErrorBudgetSquared: Float = 0.01

    init(relativeMean: Float, relativeVariance: Float, inverseErrorVariance: Float) {
        self.relativeMean = relativeMean
        self.relativeVariance = relativeVariance
        self.inverseErrorVariance = inverseErrorVariance
    }

    init?(samples: [DepthCalibrationSample], scale: Float, offset: Float) {
        guard !samples.isEmpty else { return nil }
        let count = Double(samples.count)
        let mean = samples.reduce(0) { $0 + $1.relative } / count
        let variance = samples.reduce(0) { $0 + pow($1.relative - mean, 2) } / count
        var residualVariance = 0.0
        var sensorVariance = 0.0
        for sample in samples {
            let residual = Double(scale) * sample.relative + Double(offset) - 1 / sample.depth
            residualVariance += residual * residual
            let sigma = Double(DepthAnythingProcessor.lidarStandardDeviation(depth: Float(sample.depth)))
                / (sample.depth * sample.depth)
            sensorVariance += sigma * sigma
        }
        let errorVariance = max(residualVariance, sensorVariance) / count
        guard variance > 0, Float(variance).isFinite, Float(variance) > 0,
              Float(mean).isFinite, Float(errorVariance).isFinite, Float(errorVariance) > 0 else { return nil }
        self.init(relativeMean: Float(mean), relativeVariance: Float(variance), inverseErrorVariance: Float(errorVariance))
    }

    func accepts(relative: Float, inverseDepth: Float) -> Bool {
        guard relativeVariance > 0, inverseErrorVariance > 0,
              inverseDepth.isFinite, inverseDepth > 0 else { return false }
        let delta = relative - relativeMean
        let errorVariance = inverseErrorVariance * (1 + delta * delta / relativeVariance)
        return errorVariance.isFinite && errorVariance <= inverseDepth * inverseDepth * Self.inverseErrorBudgetSquared
    }
}
