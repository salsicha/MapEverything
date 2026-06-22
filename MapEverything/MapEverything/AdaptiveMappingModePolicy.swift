//
//  AdaptiveMappingModePolicy.swift
//  MapEverything
//

import Foundation

enum AdaptiveMappingMode: String, Codable, CaseIterable, Hashable {
    case roomPlanParametric = "roomplan_parametric"
    case lidarDepthAnythingOutdoor = "lidar_depthanything_outdoor"
}

enum AdaptiveMappingOperatorOverride: String, Codable, CaseIterable, Hashable {
    case automatic
    case forceRoomPlanParametric = "force_roomplan_parametric"
    case forceLiDARDepthAnything = "force_lidar_depthanything"
}

enum AdaptiveMappingModeReason: String, Codable, CaseIterable, Hashable {
    case operatorForcedRoomPlan = "operator_forced_roomplan"
    case operatorForcedLiDARDepthAnything = "operator_forced_lidar_depthanything"
    case roomPlanUnavailable = "roomplan_unavailable"
    case roomPlanSemanticsStrong = "roomplan_semantics_strong"
    case roomPlanSemanticsWeak = "roomplan_semantics_weak"
    case indoorRegistrationStrong = "indoor_registration_strong"
    case indoorRegistrationWeak = "indoor_registration_weak"
    case outdoorGPSStrong = "outdoor_gps_strong"
    case outdoorGPSWeak = "outdoor_gps_weak"
    case lidarDepthStrong = "lidar_depth_strong"
    case lidarDepthWeak = "lidar_depth_weak"
    case depthAnythingAvailable = "depthanything_available"
    case depthAnythingUnavailable = "depthanything_unavailable"
    case thermalPressure = "thermal_pressure"
    case closeScores = "close_scores"
}

struct AdaptiveMappingModeInput: Equatable {
    var roomPlanAvailable: Bool
    var roomPlanObjectCount: Int
    var indoorRegistrationQuality: Double
    var globalRegistrationQuality: Double
    var gpsHorizontalAccuracyMeters: Double?
    var lidarDepthConfidence: Double
    var depthAnythingAvailable: Bool
    var thermalState: ProcessInfo.ThermalState
    var operatorOverride: AdaptiveMappingOperatorOverride

    init(
        roomPlanAvailable: Bool,
        roomPlanObjectCount: Int = 0,
        indoorRegistrationQuality: Double = 0,
        globalRegistrationQuality: Double = 0,
        gpsHorizontalAccuracyMeters: Double? = nil,
        lidarDepthConfidence: Double = 0,
        depthAnythingAvailable: Bool,
        thermalState: ProcessInfo.ThermalState = .nominal,
        operatorOverride: AdaptiveMappingOperatorOverride = .automatic
    ) {
        self.roomPlanAvailable = roomPlanAvailable
        self.roomPlanObjectCount = max(0, roomPlanObjectCount)
        self.indoorRegistrationQuality = indoorRegistrationQuality.clamped01
        self.globalRegistrationQuality = globalRegistrationQuality.clamped01
        self.gpsHorizontalAccuracyMeters = gpsHorizontalAccuracyMeters
        self.lidarDepthConfidence = lidarDepthConfidence.clamped01
        self.depthAnythingAvailable = depthAnythingAvailable
        self.thermalState = thermalState
        self.operatorOverride = operatorOverride
    }
}

struct AdaptiveMappingModeRecommendation: Equatable {
    let mode: AdaptiveMappingMode
    let confidence: Double
    let roomPlanScore: Double
    let outdoorScore: Double
    let operatorOverride: AdaptiveMappingOperatorOverride
    let reasons: [AdaptiveMappingModeReason]

    var metadata: [String: String] {
        [
            "active_mapping_mode": mode.rawValue,
            "adaptive_mapping_confidence": Self.format(confidence),
            "roomplan_score": Self.format(roomPlanScore),
            "outdoor_score": Self.format(outdoorScore),
            "adaptive_mapping_operator_override": operatorOverride.rawValue,
            "adaptive_mapping_reasons": reasons.map(\.rawValue).joined(separator: ",")
        ]
    }

    private static func format(_ value: Double) -> String {
        String(format: "%.3f", value)
    }
}

struct AdaptiveMappingModePolicy {
    private let closeScoreMargin: Double

    init(closeScoreMargin: Double = 0.08) {
        self.closeScoreMargin = closeScoreMargin
    }

    func recommendation(for input: AdaptiveMappingModeInput) -> AdaptiveMappingModeRecommendation {
        let automaticScores = scores(for: input)

        switch input.operatorOverride {
        case .forceRoomPlanParametric:
            return AdaptiveMappingModeRecommendation(
                mode: .roomPlanParametric,
                confidence: 1.0,
                roomPlanScore: automaticScores.roomPlan,
                outdoorScore: automaticScores.outdoor,
                operatorOverride: input.operatorOverride,
                reasons: [.operatorForcedRoomPlan] + automaticScores.reasons
            )
        case .forceLiDARDepthAnything:
            return AdaptiveMappingModeRecommendation(
                mode: .lidarDepthAnythingOutdoor,
                confidence: 1.0,
                roomPlanScore: automaticScores.roomPlan,
                outdoorScore: automaticScores.outdoor,
                operatorOverride: input.operatorOverride,
                reasons: [.operatorForcedLiDARDepthAnything] + automaticScores.reasons
            )
        case .automatic:
            break
        }

        let delta = automaticScores.outdoor - automaticScores.roomPlan
        let selectedMode: AdaptiveMappingMode = delta > closeScoreMargin ? .lidarDepthAnythingOutdoor : .roomPlanParametric
        var reasons = automaticScores.reasons
        if abs(delta) <= closeScoreMargin {
            reasons.append(.closeScores)
        }

        return AdaptiveMappingModeRecommendation(
            mode: selectedMode,
            confidence: confidence(selected: max(automaticScores.roomPlan, automaticScores.outdoor), other: min(automaticScores.roomPlan, automaticScores.outdoor)),
            roomPlanScore: automaticScores.roomPlan,
            outdoorScore: automaticScores.outdoor,
            operatorOverride: input.operatorOverride,
            reasons: reasons
        )
    }

    private func scores(for input: AdaptiveMappingModeInput) -> (roomPlan: Double, outdoor: Double, reasons: [AdaptiveMappingModeReason]) {
        let roomSemanticsScore = min(Double(input.roomPlanObjectCount) / 6.0, 1.0)
        let gpsAccuracyScore = gpsScore(input.gpsHorizontalAccuracyMeters)
        let thermalPenalty = outdoorThermalPenalty(input.thermalState)

        var reasons: [AdaptiveMappingModeReason] = []
        reasons.append(input.roomPlanAvailable ? .roomPlanSemanticsWeak : .roomPlanUnavailable)
        if roomSemanticsScore >= 0.55 {
            reasons.removeAll { $0 == .roomPlanSemanticsWeak }
            reasons.append(.roomPlanSemanticsStrong)
        }
        reasons.append(input.indoorRegistrationQuality >= 0.65 ? .indoorRegistrationStrong : .indoorRegistrationWeak)
        reasons.append((input.globalRegistrationQuality >= 0.65 || gpsAccuracyScore >= 0.7) ? .outdoorGPSStrong : .outdoorGPSWeak)
        reasons.append(input.lidarDepthConfidence >= 0.65 ? .lidarDepthStrong : .lidarDepthWeak)
        reasons.append(input.depthAnythingAvailable ? .depthAnythingAvailable : .depthAnythingUnavailable)
        if thermalPenalty > 0 {
            reasons.append(.thermalPressure)
        }

        let roomPlanScore = input.roomPlanAvailable ? (
            0.22
            + input.indoorRegistrationQuality * 0.36
            + roomSemanticsScore * 0.28
            + (1.0 - input.globalRegistrationQuality) * 0.08
            + (thermalPenalty > 0 ? 0.06 : 0.0)
        ).clamped01 : 0.0

        let outdoorScore = (
            input.globalRegistrationQuality * 0.30
            + gpsAccuracyScore * 0.22
            + input.lidarDepthConfidence * 0.25
            + (input.depthAnythingAvailable ? 0.20 : 0.04)
            + (input.roomPlanAvailable ? 0.03 : 0.08)
            - thermalPenalty
        ).clamped01

        return (roomPlanScore, outdoorScore, unique(reasons))
    }

    private func gpsScore(_ accuracy: Double?) -> Double {
        guard let accuracy, accuracy.isFinite, accuracy >= 0 else { return 0 }
        if accuracy <= 5 { return 1 }
        if accuracy >= 75 { return 0 }
        return 1.0 - ((accuracy - 5.0) / 70.0)
    }

    private func outdoorThermalPenalty(_ thermalState: ProcessInfo.ThermalState) -> Double {
        switch thermalState {
        case .nominal, .fair:
            return 0
        case .serious:
            return 0.16
        case .critical:
            return 0.32
        @unknown default:
            return 0.16
        }
    }

    private func confidence(selected: Double, other: Double) -> Double {
        (0.55 + min(max(selected - other, 0), 0.45)).clamped01
    }

    private func unique(_ reasons: [AdaptiveMappingModeReason]) -> [AdaptiveMappingModeReason] {
        var seen = Set<AdaptiveMappingModeReason>()
        return reasons.filter { seen.insert($0).inserted }
    }
}

private extension Double {
    var clamped01: Double {
        guard isFinite else { return 0 }
        return min(max(self, 0), 1)
    }
}
