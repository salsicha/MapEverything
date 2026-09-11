import Testing
@testable import MapEverything

struct RobustDepthCalibrationTests {
    private func samples(count: Int = 400) -> [DepthCalibrationSample] {
        (0..<count).map {
            let r = 0.3 + Double($0) / Double(count)
            return DepthCalibrationSample(relative: r, depth: 1 / (0.5 * r + 0.15), weight: 1)
        }
    }

    @Test("A minority of bad outdoor returns cannot dominate the scale fit")
    func rejectsOutliers() throws {
        var values = samples()
        for i in stride(from: 0, to: values.count, by: 5) {
            values[i] = DepthCalibrationSample(relative: values[i].relative, depth: 4.8, weight: 100)
        }
        let fit = try #require(RobustDepthCalibration.fit(values))
        #expect(abs(fit.scale - 0.5) < 0.001)
        #expect(abs(fit.offset - 0.15) < 0.001)
    }

    @Test("Uncorrelated depth estimates are rejected rather than projected into the scene")
    func rejectsUncorrelatedDepth() {
        let values = samples().enumerated().map { index, sample in
            DepthCalibrationSample(relative: sample.relative,
                depth: 0.5 + Double((index * 137) % 389) / 100, weight: 1)
        }
        #expect(RobustDepthCalibration.fit(values) == nil)
    }

    @Test("A reversed inverse-depth relationship is rejected")
    func rejectsNegativeScale() {
        let values = samples().map {
            DepthCalibrationSample(relative: $0.relative, depth: 1 / (1 - 0.5 * $0.relative), weight: 1)
        }
        #expect(RobustDepthCalibration.fit(values) == nil)
    }

    @Test("A single flat depth cannot determine both scale and offset")
    func rejectsUnconstrainedScale() {
        let values = samples().map {
            DepthCalibrationSample(relative: $0.relative, depth: 2 + $0.relative * 0.001, weight: 1)
        }
        #expect(RobustDepthCalibration.fit(values) == nil)
        #expect(RobustDepthCalibration.fit(samples(count: 50)) == nil)
    }

    @Test("Moderate measurement noise preserves the indoor fit")
    func acceptsNoisyValidDepth() throws {
        let values = samples().enumerated().map { index, sample in
            DepthCalibrationSample(relative: sample.relative,
                depth: sample.depth * (1 + Double(index % 7 - 3) * 0.005), weight: 1)
        }
        let fit = try #require(RobustDepthCalibration.fit(values))
        #expect(abs(fit.scale - 0.5) < 0.02)
        #expect(abs(fit.offset - 0.15) < 0.02)
    }
}
