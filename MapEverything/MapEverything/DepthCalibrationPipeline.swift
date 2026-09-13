import CoreVideo
import Foundation
import simd

/// Per-frame calibration with the accumulated map as a standing far-field
/// anchor (docs/consistent-scene-plan.md, stages 1-2). LiDAR owns absolute
/// scale where it sees (0.2-5 m); z-buffered map anchors constrain the far
/// field that LiDAR cannot, which removes the offset=0 / interior fit-family
/// split measured on scan6. Every accepted calibration also reports how deep
/// its supporting evidence actually reaches, so unconstrained extrapolation
/// never enters the persistent map.
nonisolated enum DepthCalibrationPipeline {

    struct Configuration: Sendable {
        /// Map pseudo-sample weights are normalized so their total equals
        /// this fraction of the LiDAR total — the balance validated by the
        /// scan6 simulation (-58% far-field spread at 1.0).
        var pseudoWeightBalance = 1.0
        /// Joint fitting engages once this many pseudo-samples are visible;
        /// below it the map cannot say anything trustworthy about the frame.
        var minimumJointPseudoSamples = 60
        /// Frames integrate to at most this multiple of their supporting
        /// samples' 95th-percentile depth.
        var supportDepthFactor: Float = 1.75
        /// Absolute ceiling on accumulated-scene integration depth; beyond
        /// it the monocular sigma makes fused geometry mush regardless.
        var maximumIntegrationDepth: Float = 12
        var propagation = MeshDepthPropagation.Configuration.default

        static let `default` = Configuration()
    }

    struct Result {
        let calibration: DepthAnythingProcessor.MaximumLikelihoodCalibration
        let source: DepthCalibrationSource
        /// Depth cap for integrating this frame into the persistent map.
        let integrationDepthCap: Float
    }

    static func calibrate(
        relative: RelativeDepthMap,
        lidarDepthMap: CVPixelBuffer,
        lidarConfidenceMap: CVPixelBuffer?,
        anchors: [SIMD3<Float>],
        intrinsics: simd_float3x3,
        imageResolution: CGSize,
        transform: simd_float4x4,
        configuration: Configuration = .default
    ) -> Result? {
        let lidarSamples = DepthAnythingProcessor.lidarCalibrationSamples(
            relative: relative, lidarDepthMap: lidarDepthMap, lidarConfidenceMap: lidarConfidenceMap
        )
        // Pseudo-samples skip the count/coverage gates here: for a JOINT fit
        // even a partial far-field constraint helps, and acceptance is judged
        // by the fit gates plus the agreement checks below. The gates still
        // apply in full when the map must carry the fit alone.
        let pseudoSamples = rawPseudoSamples(
            anchors: anchors, relative: relative, intrinsics: intrinsics,
            imageResolution: imageResolution, transform: transform,
            configuration: configuration.propagation
        )

        if lidarSamples.count > 50 {
            if pseudoSamples.count >= configuration.minimumJointPseudoSamples,
               let joint = jointFit(lidarSamples: lidarSamples, pseudoSamples: pseudoSamples,
                                    configuration: configuration),
               agrees(joint, with: lidarSamples, toleranceFloor: 0.12, toleranceFraction: 0.15) != false,
               agrees(joint, with: pseudoSamples, toleranceFloor: 0.3, toleranceFraction: 0.2) != false {
                return Result(
                    calibration: joint, source: .lidar,
                    integrationDepthCap: supportCap(lidarSamples + pseudoSamples, configuration: configuration)
                )
            }

            // No usable map constraint (scan start, or new territory): the
            // LiDAR-only fit stands, but its far field is unsupported, so
            // integration is capped at the LiDAR evidence horizon.
            if let lidarOnly = RobustDepthCalibration.fit(lidarSamples),
               agrees(lidarOnly, with: pseudoSamples, toleranceFloor: 0.3, toleranceFraction: 0.2) != false {
                return Result(
                    calibration: lidarOnly, source: .lidar,
                    integrationDepthCap: supportCap(lidarSamples, configuration: configuration)
                )
            }
        }

        // LiDAR cannot scale this frame (or contradicts the map it should
        // agree with): fall back to the map alone, full gates applied.
        if let propagated = MeshDepthPropagation.fitCalibration(
            anchors: anchors, relative: relative, intrinsics: intrinsics,
            imageResolution: imageResolution, transform: transform,
            lidarDepthMap: lidarDepthMap, lidarConfidenceMap: lidarConfidenceMap,
            configuration: configuration.propagation
        ) {
            let support = rawPseudoSamples(
                anchors: anchors, relative: relative, intrinsics: intrinsics,
                imageResolution: imageResolution, transform: transform,
                configuration: configuration.propagation
            )
            return Result(
                calibration: propagated, source: .meshPropagated,
                integrationDepthCap: supportCap(support, configuration: configuration)
            )
        }
        return nil
    }

    private static func rawPseudoSamples(
        anchors: [SIMD3<Float>],
        relative: RelativeDepthMap,
        intrinsics: simd_float3x3,
        imageResolution: CGSize,
        transform: simd_float4x4,
        configuration: MeshDepthPropagation.Configuration
    ) -> [DepthCalibrationSample] {
        var ungated = configuration
        ungated.minimumSamples = 1
        ungated.minimumCoverage = 0
        return MeshDepthPropagation.pseudoDepthSamples(
            anchors: anchors, relative: relative, intrinsics: intrinsics,
            imageResolution: imageResolution, transform: transform, configuration: ungated
        ) ?? []
    }

    private static func jointFit(
        lidarSamples: [DepthCalibrationSample],
        pseudoSamples: [DepthCalibrationSample],
        configuration: Configuration
    ) -> DepthAnythingProcessor.MaximumLikelihoodCalibration? {
        let lidarTotal = lidarSamples.reduce(0) { $0 + $1.weight }
        let pseudoTotal = pseudoSamples.reduce(0) { $0 + $1.weight }
        guard lidarTotal > 0, pseudoTotal > 0 else { return nil }
        let gain = configuration.pseudoWeightBalance * lidarTotal / pseudoTotal
        let balanced = pseudoSamples.map {
            DepthCalibrationSample(relative: $0.relative, depth: $0.depth, weight: $0.weight * gain)
        }
        return RobustDepthCalibration.fit(lidarSamples + balanced)
    }

    /// Nil = too few samples to judge; the caller treats nil as "cannot
    /// veto". False vetoes the fit: it contradicts independent references.
    static func agrees(
        _ calibration: DepthAnythingProcessor.MaximumLikelihoodCalibration,
        with samples: [DepthCalibrationSample],
        toleranceFloor: Double,
        toleranceFraction: Double
    ) -> Bool? {
        var agreeing = 0
        var total = 0
        for sample in samples {
            guard let predicted = DepthAnythingProcessor.calibratedMetricDepth(
                relativeDepth: Float(sample.relative), calibration: calibration
            ) else {
                total += 1
                continue
            }
            total += 1
            if abs(Double(predicted) - sample.depth) <= max(toleranceFloor, toleranceFraction * sample.depth) {
                agreeing += 1
            }
        }
        guard total >= 10 else { return nil }
        return agreeing * 2 >= total
    }

    /// 95th-percentile supporting depth times the support factor, clamped to
    /// the global ceiling: the deepest geometry this frame may write.
    static func supportCap(
        _ samples: [DepthCalibrationSample],
        configuration: Configuration = .default
    ) -> Float {
        guard !samples.isEmpty else { return DepthAnythingProcessor.minimumCalibratedDepth }
        let depths = samples.map(\.depth).sorted()
        let p95 = Float(depths[min(depths.count - 1, depths.count * 19 / 20)])
        return min(configuration.maximumIntegrationDepth,
                   max(DepthAnythingProcessor.minimumCalibratedDepth + 0.1,
                       configuration.supportDepthFactor * p95))
    }
}
