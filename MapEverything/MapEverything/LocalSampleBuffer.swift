//
//  LocalSampleBuffer.swift
//  MapEverything
//

import Foundation

nonisolated struct LocalSampleBufferStats: Equatable, Sendable {
    let maxTotalBytes: Int
    let maxPointCloudSamples: Int
    let maxMeshSamples: Int
    let totalBytes: Int
    let pointCloudSamples: Int
    let meshSamples: Int
    let droppedSamples: Int
    let replayedSamples: Int
    let lastBufferedAt: Date?

    init(
        maxTotalBytes: Int = 0,
        maxPointCloudSamples: Int = 0,
        maxMeshSamples: Int = 0,
        totalBytes: Int = 0,
        pointCloudSamples: Int = 0,
        meshSamples: Int = 0,
        droppedSamples: Int = 0,
        replayedSamples: Int = 0,
        lastBufferedAt: Date? = nil
    ) {
        self.maxTotalBytes = maxTotalBytes
        self.maxPointCloudSamples = maxPointCloudSamples
        self.maxMeshSamples = maxMeshSamples
        self.totalBytes = totalBytes
        self.pointCloudSamples = pointCloudSamples
        self.meshSamples = meshSamples
        self.droppedSamples = droppedSamples
        self.replayedSamples = replayedSamples
        self.lastBufferedAt = lastBufferedAt
    }
}

nonisolated enum LocalBufferedSampleKind: Sendable {
    case pointCloud
    case mesh
}

nonisolated struct LocalBufferedSample: Sendable {
    let sequence: UInt64
    let kind: LocalBufferedSampleKind
    let topic: String
    let data: Data
}

nonisolated struct LocalSampleBufferConfiguration: Sendable {
    let maxPointCloudSamples: Int
    let maxMeshSamples: Int
    let maxTotalBytes: Int

    static let `default` = LocalSampleBufferConfiguration(
        maxPointCloudSamples: 30,
        maxMeshSamples: 5,
        maxTotalBytes: 20_000_000
    )
}

/// Buffers point-cloud/mesh samples while the bridge is disconnected so they
/// can be replayed on reconnect. All mutable state is confined to the serial
/// `queue`.
nonisolated final class LocalSampleBuffer: @unchecked Sendable {
    private let queue = DispatchQueue(label: "com.mapeverything.localSampleBuffer", qos: .utility)
    private let configuration: LocalSampleBufferConfiguration
    private var samples: [LocalBufferedSample] = []
    private var totalBytes = 0
    private var sequence: UInt64 = 0
    private var droppedSamples = 0
    private var replayedSamples = 0
    private var lastBufferedAt: Date?

    // Set once by the owner during setup, before any traffic.
    var onStatsChange: (@Sendable (LocalSampleBufferStats) -> Void)?

    init(configuration: LocalSampleBufferConfiguration = .default) {
        self.configuration = configuration
    }

    func buffer(kind: LocalBufferedSampleKind, topic: String, data: Data) {
        queue.async {
            self.sequence += 1
            self.samples.append(
                LocalBufferedSample(
                    sequence: self.sequence,
                    kind: kind,
                    topic: topic,
                    data: data
                )
            )
            self.totalBytes += data.count
            self.lastBufferedAt = Date()
            self.trim()
            self.publishStats()
        }
    }

    /// Drains the buffer and hands the samples to `deliver` in sequence order.
    func flush(_ deliver: @escaping @Sendable ([LocalBufferedSample]) -> Void) {
        queue.async {
            let drained = self.samples.sorted { $0.sequence < $1.sequence }
            guard !drained.isEmpty else {
                self.publishStats()
                return
            }

            self.samples.removeAll()
            self.totalBytes = 0
            self.replayedSamples += drained.count
            self.publishStats()
            deliver(drained)
        }
    }

    func clear() {
        queue.async {
            self.samples.removeAll()
            self.totalBytes = 0
            self.lastBufferedAt = nil
            self.publishStats()
        }
    }

    private func trim() {
        while count(of: .pointCloud) > configuration.maxPointCloudSamples {
            dropOldest(kind: .pointCloud)
        }

        while count(of: .mesh) > configuration.maxMeshSamples {
            dropOldest(kind: .mesh)
        }

        while totalBytes > configuration.maxTotalBytes, !samples.isEmpty {
            drop(at: 0)
        }
    }

    private func count(of kind: LocalBufferedSampleKind) -> Int {
        samples.reduce(0) { count, sample in
            count + (sample.kind == kind ? 1 : 0)
        }
    }

    private func dropOldest(kind: LocalBufferedSampleKind) {
        guard let index = samples.firstIndex(where: { $0.kind == kind }) else { return }
        drop(at: index)
    }

    private func drop(at index: Int) {
        let sample = samples.remove(at: index)
        totalBytes -= sample.data.count
        droppedSamples += 1
    }

    private func publishStats() {
        onStatsChange?(
            LocalSampleBufferStats(
                maxTotalBytes: configuration.maxTotalBytes,
                maxPointCloudSamples: configuration.maxPointCloudSamples,
                maxMeshSamples: configuration.maxMeshSamples,
                totalBytes: totalBytes,
                pointCloudSamples: count(of: .pointCloud),
                meshSamples: count(of: .mesh),
                droppedSamples: droppedSamples,
                replayedSamples: replayedSamples,
                lastBufferedAt: lastBufferedAt
            )
        )
    }
}
