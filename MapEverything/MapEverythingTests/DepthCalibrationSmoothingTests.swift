//
//  DepthCalibrationSmoothingTests.swift
//  MapEverythingTests
//
//  Created by Alex Moran on 7/17/26.
//

import Testing
@testable import MapEverything

struct DepthCalibrationSmoothingTests {

    private typealias Calibration = DepthAnythingProcessor.MaximumLikelihoodCalibration

    @Test("Smoothing blends close successive fits with the EMA factor")
    func testSmoothedCalibrationBlendsCloseFits() {
        let previous = Calibration(scale: 1.0, offset: 0.2)
        let new = Calibration(scale: 1.2, offset: 0.25)

        let smoothed = DepthAnythingCalibrationCache.smoothedCalibration(new: new, previous: previous)

        let alpha = DepthAnythingCalibrationCache.smoothingFactor
        #expect(alpha == 0.4)
        let expectedScale = alpha * new.scale + (1 - alpha) * previous.scale
        let expectedOffset = alpha * new.offset + (1 - alpha) * previous.offset
        #expect(abs(smoothed.scale - expectedScale) < 1e-6)
        #expect(abs(smoothed.offset - expectedOffset) < 1e-6)
        #expect(abs(smoothed.scale - 1.08) < 1e-6)
        #expect(abs(smoothed.offset - 0.22) < 1e-6)
    }

    @Test("Smoothing passes the new fit through when the scale jump exceeds the threshold")
    func testSmoothedCalibrationPassesThroughOnScaleJump() {
        let previous = Calibration(scale: 1.0, offset: 0.2)
        // Scale jump 0.5 > 0.3 * |previous.scale|; offset stays close.
        let new = Calibration(scale: 1.5, offset: 0.21)

        let smoothed = DepthAnythingCalibrationCache.smoothedCalibration(new: new, previous: previous)

        #expect(smoothed.scale == new.scale)
        #expect(smoothed.offset == new.offset)
    }

    @Test("Smoothing passes the new fit through when the offset jump exceeds the threshold")
    func testSmoothedCalibrationPassesThroughOnOffsetJump() {
        let previous = Calibration(scale: 1.0, offset: 0.2)
        // Offset jump 0.3 > 0.3 * |previous.offset|; scale stays close.
        let new = Calibration(scale: 1.1, offset: 0.5)

        let smoothed = DepthAnythingCalibrationCache.smoothedCalibration(new: new, previous: previous)

        #expect(smoothed.scale == new.scale)
        #expect(smoothed.offset == new.offset)
    }

    @Test("Smoothing blends sign-consistent negative offsets")
    func testSmoothedCalibrationBlendsNegativeOffsets() {
        let previous = Calibration(scale: 2.0, offset: -0.2)
        let new = Calibration(scale: 1.9, offset: -0.25)

        let smoothed = DepthAnythingCalibrationCache.smoothedCalibration(new: new, previous: previous)

        #expect(abs(smoothed.scale - 1.96) < 1e-6)
        #expect(abs(smoothed.offset - (-0.22)) < 1e-6)
    }

    @Test("First-ever calibration is returned unmodified")
    func testFirstCalibrationIsUnmodified() {
        let new = Calibration(scale: 0.5, offset: 0.15)

        let smoothed = DepthAnythingCalibrationCache.smoothedCalibration(new: new, previous: nil)

        #expect(smoothed.scale == new.scale)
        #expect(smoothed.offset == new.offset)
    }
}
