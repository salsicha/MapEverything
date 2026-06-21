//
//  MeshSnapshotPublishConfiguration.swift
//  Reconstructor
//

import Foundation

struct MeshSnapshotPublishConfiguration {
    let publishInterval: TimeInterval
    let maxPayloadBytes: Int
    let maxTrianglePoints: Int

    static let `default` = MeshSnapshotPublishConfiguration(
        publishInterval: 2.0,
        maxPayloadBytes: 1_500_000,
        maxTrianglePoints: 12_000
    )
}
