import CoreVideo
import Foundation
import simd

/// Which reference data calibrated a Depth Anything frame.
nonisolated enum DepthCalibrationSource: Sendable {
    case lidar
    case meshPropagated
}

/// Fallback calibration source for frames whose per-frame LiDAR calibration
/// failed: projects LiDAR-anchored accumulated-mesh vertices into the current
/// camera and fits the same constrained inverse-depth family against those
/// pseudo-depth samples (design: docs/mesh-propagated-calibration.md).
/// Free-standing and nonisolated, like RobustDepthCalibration; scale is never
/// invented — every gate failure returns nil and the frame is dropped exactly
/// as it was before this feature existed.
nonisolated enum MeshDepthPropagation {

    struct Configuration: Sendable {
        /// Z-buffer cell edge in relative-depth-grid pixels.
        var cellDownsample = 8
        /// Physical surface patch each anchor represents (the anchor voxel
        /// edge); its projected footprint drives the splat radius.
        var anchorFootprint: Float = 0.2
        /// Stricter than RobustDepthCalibration's >50: pseudo-samples are
        /// individually weaker than LiDAR returns.
        var minimumSamples = 200
        /// Minimum 10th–90th percentile spread of occupied cells, as a
        /// fraction of the cell grid, in each axis. Prevents calibrating a
        /// whole frame from a mesh sliver in one corner.
        var minimumCoverage: Float = 0.4
        /// Pseudo-depth range serving as calibration reference. Below 0.3 m
        /// mesh positions carry weld-voxel-scale relative error; beyond 30 m
        /// anchor angular error and pose drift dominate the sigma model.
        var depthRange: ClosedRange<Float> = 0.3...30

        static let `default` = Configuration()
    }

    /// One z-buffer-visible anchor chosen per occupied cell.
    struct ProjectedAnchor {
        let gridX: Int
        let gridY: Int
        let depth: Float
    }

    /// One splatted anchor footprint, kept so the occlusion pass can replay
    /// the z-buffer population in depth order.
    private struct SplatRecord {
        var cellX: Int32
        var cellY: Int32
        var radius: Int32
        var depth: Float
    }

    /// Grazing-aware occlusion horizon: per z-buffer cell, the deepest splat
    /// CONTINUOUSLY connected to the cell's nearest splat. Testing anchors
    /// against the nearest splat alone kills far anchors of the same receding
    /// surface — a street at grazing incidence — because a near cell's
    /// footprint covers the rays of deeper cells of that surface with a depth
    /// gap far beyond the visibility tolerance (at 8 m on a ground plane one
    /// 8-px cell spans ~1 m of surface depth). A continuous surface, however,
    /// populates every intermediate depth of the cell at the anchor map's own
    /// quantization (one anchor per `anchorFootprint`), while a genuinely
    /// detached occluder pair — a wall hiding separate geometry — leaves a
    /// gap. Walking each cell's splat depths in ascending order and stopping
    /// at the first gap wider than the continuity step therefore follows
    /// grazing surfaces to their true depth yet still halts at the near side
    /// of any detached surface.
    private static func chainedSurfaceReach(
        splats: [SplatRecord],
        nearestDepth: [Float],
        cellsW: Int,
        cellsH: Int,
        anchorFootprint: Float
    ) -> [Float] {
        var reach = nearestDepth
        let ordered = splats.sorted { $0.depth < $1.depth }
        // Adjacent same-surface anchors differ by up to ~√2 footprints of
        // depth (diagonal grid steps); the 5% term mirrors the visibility
        // tolerance so deep links tolerate the same fractional noise.
        let stepFloor = 1.5 * anchorFootprint
        for splat in ordered {
            let step = max(stepFloor, 0.05 * splat.depth)
            let radius = Int(splat.radius)
            let cellX = Int(splat.cellX)
            let cellY = Int(splat.cellY)
            for y in max(0, cellY - radius)...min(cellsH - 1, cellY + radius) {
                let rowBase = y * cellsW
                for x in max(0, cellX - radius)...min(cellsW - 1, cellX + radius) {
                    let index = rowBase + x
                    if splat.depth > reach[index], splat.depth <= reach[index] + step {
                        reach[index] = splat.depth
                    }
                }
            }
        }
        return reach
    }

    /// Projects anchors into the relative-depth grid with the same pinhole
    /// convention as MeshGenerator's unprojection, resolves occlusion with a
    /// footprint-splatted z-buffer (grazing-aware: same-surface continuity
    /// extends each cell's occlusion horizon, see chainedSurfaceReach), and
    /// returns at most one visible anchor per coarse cell. Returns nil when
    /// the count or coverage gates fail.
    static func visibleAnchors(
        anchors: [SIMD3<Float>],
        depthWidth: Int,
        depthHeight: Int,
        intrinsics: simd_float3x3,
        imageResolution: CGSize,
        transform: simd_float4x4,
        configuration: Configuration = .default
    ) -> [ProjectedAnchor]? {
        guard depthWidth > 1, depthHeight > 1,
              imageResolution.width > 0, imageResolution.height > 0 else { return nil }
        let scaleX = Float(depthWidth) / Float(imageResolution.width)
        let scaleY = Float(depthHeight) / Float(imageResolution.height)
        let fx = intrinsics[0][0] * scaleX
        let fy = intrinsics[1][1] * scaleY
        let cx = intrinsics[2][0] * scaleX
        let cy = intrinsics[2][1] * scaleY
        guard fx.isFinite, fy.isFinite, cx.isFinite, cy.isFinite,
              abs(fx) > 1e-5, abs(fy) > 1e-5 else { return nil }

        let worldToCamera = transform.inverse
        guard worldToCamera.columns.0.x.isFinite else { return nil }

        let down = max(1, configuration.cellDownsample)
        let cellsW = (depthWidth + down - 1) / down
        let cellsH = (depthHeight + down - 1) / down
        var cellDepth = [Float](repeating: .infinity, count: cellsW * cellsH)

        var projected: [(x: Int32, y: Int32, depth: Float)] = []
        projected.reserveCapacity(min(anchors.count, 65_536))
        var splats: [SplatRecord] = []
        splats.reserveCapacity(min(anchors.count, 65_536))

        // Pass 1: project and splat each anchor's physical footprint into the
        // z-buffer, so near surfaces suppress back-surface anchors even in
        // the gaps between sparse projected points. Occlusion is decoupled
        // from the sampling range: an anchored surface nearer than the
        // minimum sampling depth still hides everything behind it, it just
        // cannot serve as a calibration reference itself.
        let minimumOccluderDepth: Float = 0.05
        let footprintScale = 0.5 * configuration.anchorFootprint * max(abs(fx), abs(fy))
        for anchor in anchors {
            let camera = worldToCamera * SIMD4<Float>(anchor.x, anchor.y, anchor.z, 1)
            let depth = -camera.z
            guard depth >= minimumOccluderDepth,
                  depth <= configuration.depthRange.upperBound else { continue }
            let gridX = cx + fx * camera.x / depth
            let gridY = cy - fy * camera.y / depth
            guard gridX.isFinite, gridY.isFinite else { continue }
            let px = Int(gridX.rounded())
            let py = Int(gridY.rounded())
            guard px >= 0, px < depthWidth, py >= 0, py < depthHeight else { continue }
            if depth >= configuration.depthRange.lowerBound {
                projected.append((Int32(px), Int32(py), depth))
            }

            let radius = min(6, max(1, Int((footprintScale / (depth * Float(down))).rounded())))
            let cellX = px / down
            let cellY = py / down
            splats.append(SplatRecord(cellX: Int32(cellX), cellY: Int32(cellY),
                                      radius: Int32(radius), depth: depth))
            for y in max(0, cellY - radius)...min(cellsH - 1, cellY + radius) {
                for x in max(0, cellX - radius)...min(cellsW - 1, cellX + radius) {
                    let index = y * cellsW + x
                    if depth < cellDepth[index] { cellDepth[index] = depth }
                }
            }
        }

        // Pass 2: an anchor is visible when nothing splatted meaningfully in
        // front of it in its own cell — where "in front" honors surface
        // continuity: the occlusion horizon is the deepest splat chained to
        // the cell's nearest one, so grazing surfaces keep their own far
        // anchors while detached occluders still suppress. Keep the nearest
        // visible anchor per cell so samples stay spread instead of
        // clustering on dense geometry.
        let reach = chainedSurfaceReach(splats: splats, nearestDepth: cellDepth,
                                        cellsW: cellsW, cellsH: cellsH,
                                        anchorFootprint: configuration.anchorFootprint)
        var winnerDepth = [Float](repeating: .infinity, count: cellsW * cellsH)
        var winnerPixel = [Int32](repeating: -1, count: cellsW * cellsH)
        for point in projected {
            let cell = (Int(point.y) / down) * cellsW + Int(point.x) / down
            let tolerance = max(0.10, 0.05 * point.depth)
            guard point.depth <= reach[cell] + tolerance,
                  point.depth < winnerDepth[cell] else { continue }
            winnerDepth[cell] = point.depth
            winnerPixel[cell] = point.y * Int32(depthWidth) + point.x
        }

        var result: [ProjectedAnchor] = []
        var cellXs: [Int] = []
        var cellYs: [Int] = []
        result.reserveCapacity(cellsW * cellsH / 4)
        for cell in 0..<(cellsW * cellsH) where winnerPixel[cell] >= 0 {
            let pixel = Int(winnerPixel[cell])
            result.append(ProjectedAnchor(gridX: pixel % depthWidth,
                                          gridY: pixel / depthWidth,
                                          depth: winnerDepth[cell]))
            cellXs.append(cell % cellsW)
            cellYs.append(cell / cellsW)
        }

        guard result.count >= configuration.minimumSamples else { return nil }
        guard percentileSpread(cellXs) >= configuration.minimumCoverage * Float(cellsW),
              percentileSpread(cellYs) >= configuration.minimumCoverage * Float(cellsH) else { return nil }
        return result
    }

    /// Depths of EVERY visibility-passing anchor in this frame - no
    /// per-cell winner selection. The per-cell nearest-anchor choice that
    /// keeps fit samples spread is systematically near-biased, so the
    /// integration horizon must be judged on this unbiased population.
    static func visibleAnchorDepths(
        anchors: [SIMD3<Float>],
        depthWidth: Int,
        depthHeight: Int,
        intrinsics: simd_float3x3,
        imageResolution: CGSize,
        transform: simd_float4x4,
        configuration: Configuration = .default
    ) -> [Float] {
        guard depthWidth > 1, depthHeight > 1,
              imageResolution.width > 0, imageResolution.height > 0 else { return [] }
        let scaleX = Float(depthWidth) / Float(imageResolution.width)
        let scaleY = Float(depthHeight) / Float(imageResolution.height)
        let fx = intrinsics[0][0] * scaleX
        let fy = intrinsics[1][1] * scaleY
        let cx = intrinsics[2][0] * scaleX
        let cy = intrinsics[2][1] * scaleY
        guard fx.isFinite, fy.isFinite, abs(fx) > 1e-5, abs(fy) > 1e-5 else { return [] }
        let worldToCamera = transform.inverse
        guard worldToCamera.columns.0.x.isFinite else { return [] }

        let down = max(1, configuration.cellDownsample)
        let cellsW = (depthWidth + down - 1) / down
        let cellsH = (depthHeight + down - 1) / down
        var cellDepth = [Float](repeating: .infinity, count: cellsW * cellsH)
        var projected: [(cell: Int, depth: Float)] = []
        projected.reserveCapacity(min(anchors.count, 65_536))
        var splats: [SplatRecord] = []
        splats.reserveCapacity(min(anchors.count, 65_536))
        let footprintScale = 0.5 * configuration.anchorFootprint * max(abs(fx), abs(fy))
        for anchor in anchors {
            let camera = worldToCamera * SIMD4<Float>(anchor.x, anchor.y, anchor.z, 1)
            let depth = -camera.z
            guard depth >= configuration.depthRange.lowerBound,
                  depth <= configuration.depthRange.upperBound else { continue }
            let gridX = cx + fx * camera.x / depth
            let gridY = cy - fy * camera.y / depth
            guard gridX.isFinite, gridY.isFinite else { continue }
            let px = Int(gridX.rounded())
            let py = Int(gridY.rounded())
            guard px >= 0, px < depthWidth, py >= 0, py < depthHeight else { continue }
            let cellX = px / down
            let cellY = py / down
            projected.append((cellY * cellsW + cellX, depth))
            let radius = min(6, max(1, Int((footprintScale / (depth * Float(down))).rounded())))
            splats.append(SplatRecord(cellX: Int32(cellX), cellY: Int32(cellY),
                                      radius: Int32(radius), depth: depth))
            for y in max(0, cellY - radius)...min(cellsH - 1, cellY + radius) {
                for x in max(0, cellX - radius)...min(cellsW - 1, cellX + radius) {
                    let index = y * cellsW + x
                    if depth < cellDepth[index] { cellDepth[index] = depth }
                }
            }
        }
        // The same grazing-aware horizon as visibleAnchors: the integration
        // horizon must not be near-biased by same-surface footprint overlap.
        let reach = chainedSurfaceReach(splats: splats, nearestDepth: cellDepth,
                                        cellsW: cellsW, cellsH: cellsH,
                                        anchorFootprint: configuration.anchorFootprint)
        return projected.compactMap { point in
            point.depth <= reach[point.cell] + max(0.10, 0.05 * point.depth) ? point.depth : nil
        }
    }

    /// Full fallback fit: visible-anchor sampling, the verbatim
    /// RobustDepthCalibration gates, and — when enough individually valid
    /// LiDAR pixels exist despite the failed LiDAR fit — an absolute-scale
    /// cross-check against them. Nil means: drop the frame, exactly like a
    /// failed LiDAR calibration always has.
    static func fitCalibration(
        anchors: [SIMD3<Float>],
        relative: RelativeDepthMap,
        intrinsics: simd_float3x3,
        imageResolution: CGSize,
        transform: simd_float4x4,
        lidarDepthMap: CVPixelBuffer? = nil,
        lidarConfidenceMap: CVPixelBuffer? = nil,
        configuration: Configuration = .default
    ) -> DepthAnythingProcessor.MaximumLikelihoodCalibration? {
        guard let samples = pseudoDepthSamples(
            anchors: anchors,
            relative: relative,
            intrinsics: intrinsics,
            imageResolution: imageResolution,
            transform: transform,
            configuration: configuration
        ) else { return nil }
        guard let fit = RobustDepthCalibration.fit(samples) else { return nil }

        if let lidarDepthMap,
           DepthAnythingProcessor.sparseLiDARAgreement(
               calibration: fit,
               relative: relative,
               lidarDepthMap: lidarDepthMap,
               lidarConfidenceMap: lidarConfidenceMap
           ) == false {
            return nil
        }
        return fit
    }

    /// Visible-anchor pseudo-samples ready for the fitter: z-buffered
    /// projection, per-cell selection, and the count/coverage gates. Nil when
    /// the anchors cannot legitimately constrain this frame.
    static func pseudoDepthSamples(
        anchors: [SIMD3<Float>],
        relative: RelativeDepthMap,
        intrinsics: simd_float3x3,
        imageResolution: CGSize,
        transform: simd_float4x4,
        configuration: Configuration = .default
    ) -> [DepthCalibrationSample]? {
        guard let visible = visibleAnchors(
            anchors: anchors,
            depthWidth: relative.width,
            depthHeight: relative.height,
            intrinsics: intrinsics,
            imageResolution: imageResolution,
            transform: transform,
            configuration: configuration
        ) else { return nil }

        let samples: [DepthCalibrationSample] = relative.withReadAccess { reader in
            visible.compactMap { anchor in
                let value = reader.value(atX: anchor.gridX, y: anchor.gridY)
                guard value.isFinite, value > 0 else { return nil }
                // Inverse-variance weight in the inverse-depth domain, in the
                // LiDAR path's exact convention but with the honest monocular
                // sigma: mesh pseudo-depth inherits network shape error, not
                // LiDAR sensor noise.
                let sigma = DepthAnythingProcessor.monocularStandardDeviation(depth: anchor.depth)
                    / (anchor.depth * anchor.depth)
                return DepthCalibrationSample(relative: Double(value),
                                              depth: Double(anchor.depth),
                                              weight: 1 / Double(sigma * sigma))
            }
        }
        guard samples.count >= configuration.minimumSamples else { return nil }
        return samples
    }

    /// 10th-to-90th percentile spread of the given coordinates.
    private static func percentileSpread(_ values: [Int]) -> Float {
        guard values.count > 1 else { return 0 }
        let sorted = values.sorted()
        return Float(sorted[sorted.count * 9 / 10] - sorted[sorted.count / 10])
    }
}
