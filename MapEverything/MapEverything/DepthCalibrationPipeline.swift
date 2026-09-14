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
        /// Joint fitting engages once this many pseudo-samples are visible —
        /// deliberately low: engaging is safe (the agreement gates judge the
        /// result), and frames facing new territory often see few anchors.
        var minimumJointPseudoSamples = 30
        /// Vetoing a LiDAR fit takes a substantially larger population than
        /// merely helping one.
        var minimumVetoPseudoSamples = 60
        /// Frames integrate to at most this multiple of their supporting
        /// samples' 95th-percentile depth.
        var supportDepthFactor: Float = 1.75
        /// Absolute ceiling on accumulated-scene integration depth. Chosen
        /// for street-scale scenes. Measured on the street/scan6 replays:
        /// inter-frame same-ray spread is ~0.47 m median in the 6-11 m band
        /// but grows to ~1.9 m median beyond 11 m — intrinsic monocular
        /// noise that the TSDF absorbs through its range-scaled truncation
        /// band, 1/z^2 observation weights, and free-space erosion rather
        /// than through this cap. The ceiling bounds how far that machinery
        /// must stretch, not where the geometry is trustworthy.
        var maximumIntegrationDepth: Float = 18
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
        // The horizon is judged on EVERY visibility-passing anchor: the
        // per-cell nearest-anchor selection that keeps fit samples spread is
        // systematically near-biased and would pin the cap.
        let anchorDepths = MeshDepthPropagation.visibleAnchorDepths(
            anchors: anchors, depthWidth: relative.width, depthHeight: relative.height,
            intrinsics: intrinsics, imageResolution: imageResolution, transform: transform,
            configuration: configuration.propagation
        )

        if lidarSamples.count > 50 {
            if pseudoSamples.count >= configuration.minimumJointPseudoSamples,
               let joint = jointFit(lidarSamples: lidarSamples, pseudoSamples: pseudoSamples,
                                    configuration: configuration),
               agrees(joint, with: lidarSamples, toleranceFloor: 0.12, toleranceFraction: 0.15) != false,
               agrees(joint, with: pseudoSamples, toleranceFloor: 0.3, toleranceFraction: 0.2) != false {
                // An ACCEPTED joint fit carries its own far evidence: the
                // pseudo-samples it predicts within the fitter's inlier
                // tolerance are direct proof the calibration holds at those
                // depths, so they extend the integration horizon under the
                // same 1.75x contract LiDAR evidence gets. LiDAR-only and
                // mesh-propagated frames never receive this population —
                // without a joint acceptance the far field is extrapolation,
                // not evidence.
                return Result(
                    calibration: joint, source: .lidar,
                    integrationDepthCap: supportCap(
                        lidarSamples: lidarSamples,
                        anchorDepths: anchorDepths,
                        inlyingPseudoDepths: fitInlyingPseudoDepths(calibration: joint,
                                                                    pseudoSamples: pseudoSamples),
                        configuration: configuration
                    )
                )
            }

            // No usable map constraint (scan start, or new territory): the
            // LiDAR-only fit stands, but its far field is unsupported, so
            // integration is capped at the LiDAR evidence horizon. The map
            // may only VETO a LiDAR fit with the same substantial sample
            // population joint fitting requires — a stale handful of anchors
            // must never stall mapping (carving self-heals them instead).
            let mapMayVeto = pseudoSamples.count >= configuration.minimumVetoPseudoSamples
            if let lidarOnly = RobustDepthCalibration.fit(lidarSamples) {
                // The map's testimony is judged once and gates two things:
                // a substantial population may veto the fit outright, and
                // the anchor-shell horizon is inherited only from anchors
                // this fit does not contradict. A frame the map disputes
                // (joint fit attempted and REJECTED, or the sub-veto
                // population disagreeing) may still map — LiDAR is trusted
                // where LiDAR sees — but only to the LiDAR evidence
                // horizon: inheriting a deep cap from the very anchors
                // that testify against the fit would write a far skin
                // displaced by more than the agreement tolerance (2 m at
                // 10 m) in a single frame, outside the TSDF truncation
                // band and largely uncarvable.
                let mapAgreement = agrees(lidarOnly, with: pseudoSamples,
                                          toleranceFloor: 0.3, toleranceFraction: 0.2)
                if !mapMayVeto || mapAgreement != false {
                    // Visible anchors still testify to the scene's supported
                    // depth even when the fit itself is LiDAR-only — but
                    // only when they and the fit tell one story.
                    return Result(
                        calibration: lidarOnly, source: .lidar,
                        integrationDepthCap: supportCap(lidarSamples: lidarSamples,
                                                        anchorDepths: mapAgreement != false ? anchorDepths : [],
                                                        configuration: configuration)
                    )
                }
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
            return Result(
                calibration: propagated, source: .meshPropagated,
                integrationDepthCap: supportCap(lidarSamples: [],
                                                anchorDepths: anchorDepths,
                                                configuration: configuration)
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

    /// Depths of the pseudo-samples `calibration` predicts within
    /// RobustDepthCalibration's inlier tolerance (max(0.06, 8% of depth)).
    /// For an ACCEPTED joint fit these are direct evidence the calibration
    /// holds at those depths — each is a z-buffer-visible, mass-qualified
    /// map anchor whose predicted metric depth matches its mapped position —
    /// which is exactly the support contract LiDAR samples get. Samples the
    /// fit cannot predict within tolerance testify to nothing and are
    /// excluded.
    static func fitInlyingPseudoDepths(
        calibration: DepthAnythingProcessor.MaximumLikelihoodCalibration,
        pseudoSamples: [DepthCalibrationSample]
    ) -> [Float] {
        pseudoSamples.compactMap { sample in
            guard let predicted = DepthAnythingProcessor.calibratedMetricDepth(
                relativeDepth: Float(sample.relative), calibration: calibration
            ) else { return nil }
            return abs(Double(predicted) - sample.depth) <= max(0.06, 0.08 * sample.depth)
                ? Float(sample.depth) : nil
        }
    }

    /// The deepest depth in `depths` whose reference shell [0.7*D, D] holds
    /// at least 10 of the population's own members, or 0 when no candidate's
    /// frontier is corroborated. Percentiles over a whole population
    /// re-dilute against the dense near field at every level (the exact
    /// statistic that pinned on-device caps inside LiDAR range), and a bare
    /// maximum lets a LONE deep member certify a horizon by itself — for
    /// small populations the p95 index IS the maximum. A shell count asks
    /// the only question that matters: is the frontier itself validated by
    /// enough independent members?
    static func shellSupportedHorizon(_ depths: [Float]) -> Float {
        let sorted = depths.sorted(by: >)
        var index = 0
        while index < sorted.count {
            let candidate = sorted[index]
            var shell = 0
            var probe = index
            while probe < sorted.count, sorted[probe] >= candidate * 0.7 {
                shell += 1
                probe += 1
            }
            if shell >= 10 { return candidate }
            index += 1
        }
        return 0
    }

    /// The deepest geometry a frame may write: the support factor times the
    /// evidence horizon, clamped to the global ceiling. The horizon is the
    /// MAXIMUM of each reference population's own statistic — never a
    /// percentile of the pooled set. Pooling is a contraction on real
    /// devices: thousands of confidence-gated near-field LiDAR samples
    /// numerically drown the few far map anchors, pinning the cap inside
    /// LiDAR range forever (reproduced from scan6 with device-realistic
    /// gating). Each population is a trusted reference in its own right, so
    /// each extends the horizon independently — but the two map-derived
    /// populations must corroborate their own frontier via the shell bar.
    static func supportCap(
        lidarSamples: [DepthCalibrationSample],
        anchorDepths: [Float] = [],
        inlyingPseudoDepths: [Float] = [],
        configuration: Configuration = .default
    ) -> Float {
        var horizon: Float = 0
        if !lidarSamples.isEmpty {
            let depths = lidarSamples.map(\.depth).sorted()
            horizon = Float(depths[min(depths.count - 1, depths.count * 19 / 20)])
        }
        // Fit-inlying pseudo-samples of an accepted joint calibration (the
        // only caller that passes them) are a trusted reference population
        // in their own right and extend the horizon like the LiDAR
        // population does — but through the same shell bar the anchor
        // branch enforces, never a bare percentile: a single deep inlier is
        // circular evidence (it was a gain-balanced joint-fit constraint,
        // so the fit conforming to it is expected, not independent
        // validation) and must not push the cap to the ceiling alone.
        if !inlyingPseudoDepths.isEmpty {
            horizon = max(horizon, shellSupportedHorizon(inlyingPseudoDepths))
        }
        // The anchor horizon takes the same shell statistic, behind the
        // stricter >= 30 population gate a fit-independent reference set
        // must clear.
        if anchorDepths.count >= 30 {
            horizon = max(horizon, shellSupportedHorizon(anchorDepths))
        }
        guard horizon > 0 else { return DepthAnythingProcessor.minimumCalibratedDepth }
        return min(configuration.maximumIntegrationDepth,
                   max(DepthAnythingProcessor.minimumCalibratedDepth + 0.1,
                       configuration.supportDepthFactor * horizon))
    }
}
