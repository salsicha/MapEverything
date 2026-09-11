import Foundation
import Testing
@testable import MapEverything

struct DepthCalibrationSupportTests {
    // Four frames from the 2026-09-11 22:07 recording. Support summaries were
    // reconstructed from corresponding saved LiDAR/DA points, without the raw
    // confidence image; they test the extrapolation gate, not exact fit replay.
    private let outdoorFrames: [(Float, Float, Float, Float, Float)] = [
        (0.78529912, -0.02570801, 0.49635313, 0.02426196, 0.00007011335),
        (0.72444069, -0.04655098, 0.54187137, 0.02231646, 0.00005292668),
        (0.62490857, -0.03916411, 0.58907699, 0.02186911, 0.00004994576),
        (0.58345944, -0.04287531, 0.65998204, 0.02388046, 0.00010846595)
    ]

    @Test("Recorded outdoor fits keep supported surfaces and reject the distant fan")
    func recordedPoleFrames() throws {
        for (scale, offset, mean, variance, error) in outdoorFrames {
            let fit = DepthAnythingProcessor.MaximumLikelihoodCalibration(
                scale: scale, offset: offset,
                support: .init(relativeMean: mean, relativeVariance: variance, inverseErrorVariance: error)
            )
            for depth: Float in [2.5, 3, 4] {
                let relative = (1 / depth - offset) / scale
                let accepted = try #require(DepthAnythingProcessor.calibratedMetricDepth(relativeDepth: relative, calibration: fit))
                #expect(abs(accepted - depth) < 0.001)
            }
            for depth: Float in [30, 50, 99] {
                let relative = (1 / depth - offset) / scale
                #expect(DepthAnythingProcessor.calibratedMetricDepth(relativeDepth: relative, calibration: fit) == nil)
            }
        }
    }

    @Test("Live fits carry support and can retain depth beyond LiDAR range")
    func fittedSupportPreservesBeyondSensorRange() throws {
        let samples = (0..<400).map { index in
            let depth = 1 + Double(index) / 399 * 3.8
            return DepthCalibrationSample(relative: 1 / depth, depth: depth, weight: 1)
        }
        let fit = try #require(RobustDepthCalibration.fit(samples))
        _ = try #require(fit.support)
        let accepted = try #require(DepthAnythingProcessor.calibratedMetricDepth(relativeDepth: 1 / 6, calibration: fit))
        #expect(abs(accepted - 6) < 0.001)
        #expect(DepthAnythingProcessor.calibratedMetricDepth(relativeDepth: 1 / 99, calibration: fit) == nil)
    }

    @Test("Repeating correlated calibration pixels does not create false certainty")
    func repeatedPixelsDoNotReduceError() throws {
        let samples = (0..<100).map { index in
            let relative = 0.3 + Double(index) / 100
            return DepthCalibrationSample(relative: relative, depth: 1 / (0.5 * relative + 0.1), weight: 1)
        }
        let first = try #require(DepthCalibrationSupport(samples: samples, scale: 0.5, offset: 0.1))
        let repeated = try #require(DepthCalibrationSupport(samples: Array(repeating: samples, count: 10).flatMap { $0 }, scale: 0.5, offset: 0.1))
        #expect(abs(first.inverseErrorVariance - repeated.inverseErrorVariance) < 1e-8)
        #expect(abs(first.relativeVariance - repeated.relativeVariance) < 1e-6)
    }

    @Test("The reciprocal error bound rejects the uncertain side of its boundary")
    func errorBoundary() {
        let support = DepthCalibrationSupport(relativeMean: 0.5, relativeVariance: 0.1, inverseErrorVariance: 0.0001)
        #expect(support.accepts(relative: 0.5, inverseDepth: 0.101))
        #expect(!support.accepts(relative: 0.5, inverseDepth: 0.099))
        #expect(!support.accepts(relative: .nan, inverseDepth: 1))
        #expect(!support.accepts(relative: 0.5, inverseDepth: .infinity))
    }

    @Test("Saved calibration messages describe the applied extrapolation filter")
    func recordsSupportMetadata() throws {
        let message = ROS2BridgeClient.makeDepthAnythingCalibrationMessage(
            calibration: .init(scale: 0.7, offset: -0.03, support: .init(relativeMean: 0.5, relativeVariance: 0.02, inverseErrorVariance: 0.0001)),
            header: [:], relativeDepthSize: .init(width: 518, height: 392),
            imageResolution: .init(width: 1920, height: 1440),
            relativePointCloudTopic: "/test", frameID: "camera"
        )
        let json = try #require(message["metadata_json"] as? String)
        let metadata = try #require(JSONSerialization.jsonObject(with: Data(json.utf8)) as? [String: Any])
        #expect(metadata["depth_validity_filter"] as? String == "calibration_extrapolation_error_envelope")
        #expect(metadata["relative_variance"] != nil)
        #expect(metadata["inverse_error_variance"] != nil)
    }
}
