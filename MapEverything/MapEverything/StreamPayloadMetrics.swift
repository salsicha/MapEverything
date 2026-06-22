//
//  StreamPayloadMetrics.swift
//  MapEverything
//

import Foundation

struct StreamPayloadMetricSnapshot: Equatable {
    let streamID: String
    let messageCount: Int
    let originalBytesTotal: Int
    let encodedBytesTotal: Int
    let maxEncodedBytes: Int
    let lastOriginalBytes: Int
    let lastEncodedBytes: Int
    let lastCompression: String
    let lastRecordedAt: Date?

    var compressionRatio: Double {
        guard originalBytesTotal > 0 else { return 0 }
        return Double(encodedBytesTotal) / Double(originalBytesTotal)
    }

    var rosMessage: [String: Any] {
        [
            "stream_id": streamID,
            "message_count": messageCount,
            "original_bytes_total": originalBytesTotal,
            "encoded_bytes_total": encodedBytesTotal,
            "max_encoded_bytes": maxEncodedBytes,
            "last_original_bytes": lastOriginalBytes,
            "last_encoded_bytes": lastEncodedBytes,
            "last_compression": lastCompression,
            "compression_ratio": compressionRatio,
            "last_recorded_at": lastRecordedAt.map { ISO8601DateFormatter().string(from: $0) } ?? ""
        ]
    }
}

struct StreamPayloadMetricAccumulator: Equatable {
    let streamID: String
    private(set) var messageCount = 0
    private(set) var originalBytesTotal = 0
    private(set) var encodedBytesTotal = 0
    private(set) var maxEncodedBytes = 0
    private(set) var lastOriginalBytes = 0
    private(set) var lastEncodedBytes = 0
    private(set) var lastCompression = ""
    private(set) var lastRecordedAt: Date?

    init(streamID: String) {
        self.streamID = streamID
    }

    mutating func record(
        originalBytes: Int,
        encodedBytes: Int,
        compression: String,
        recordedAt: Date = Date()
    ) {
        let boundedOriginalBytes = max(0, originalBytes)
        let boundedEncodedBytes = max(0, encodedBytes)
        messageCount += 1
        originalBytesTotal += boundedOriginalBytes
        encodedBytesTotal += boundedEncodedBytes
        maxEncodedBytes = max(maxEncodedBytes, boundedEncodedBytes)
        lastOriginalBytes = boundedOriginalBytes
        lastEncodedBytes = boundedEncodedBytes
        lastCompression = compression
        lastRecordedAt = recordedAt
    }

    var snapshot: StreamPayloadMetricSnapshot {
        StreamPayloadMetricSnapshot(
            streamID: streamID,
            messageCount: messageCount,
            originalBytesTotal: originalBytesTotal,
            encodedBytesTotal: encodedBytesTotal,
            maxEncodedBytes: maxEncodedBytes,
            lastOriginalBytes: lastOriginalBytes,
            lastEncodedBytes: lastEncodedBytes,
            lastCompression: lastCompression,
            lastRecordedAt: lastRecordedAt
        )
    }
}

final class StreamPayloadMetricsStore {
    static let shared = StreamPayloadMetricsStore()

    private let lock = NSLock()
    private var accumulators: [MappingSensorStream: StreamPayloadMetricAccumulator] = [:]

    func record(
        stream: MappingSensorStream,
        originalBytes: Int,
        encodedBytes: Int,
        compression: String,
        recordedAt: Date = Date()
    ) {
        lock.lock()
        var accumulator = accumulators[stream] ?? StreamPayloadMetricAccumulator(streamID: stream.rawValue)
        accumulator.record(
            originalBytes: originalBytes,
            encodedBytes: encodedBytes,
            compression: compression,
            recordedAt: recordedAt
        )
        accumulators[stream] = accumulator
        lock.unlock()
    }

    func snapshot(for stream: MappingSensorStream) -> StreamPayloadMetricSnapshot? {
        lock.lock()
        let snapshot = accumulators[stream]?.snapshot
        lock.unlock()
        return snapshot
    }

    func allSnapshots() -> [StreamPayloadMetricSnapshot] {
        lock.lock()
        let snapshots = accumulators.values.map(\.snapshot).sorted { $0.streamID < $1.streamID }
        lock.unlock()
        return snapshots
    }

    var rosMessage: [[String: Any]] {
        allSnapshots().map(\.rosMessage)
    }

    func reset() {
        lock.lock()
        accumulators.removeAll()
        lock.unlock()
    }
}
