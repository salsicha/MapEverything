//
//  PublishQueue.swift
//  MapEverything
//

import Foundation

struct PublishQueueStats: Equatable {
    let capacity: Int
    let depth: Int
    let inFlight: Int
    let sentMessages: Int
    let droppedMessages: Int
    let retriedMessages: Int
    let failedMessages: Int
    let lastError: String?
    let lastErrorAt: Date?

    init(
        capacity: Int,
        depth: Int = 0,
        inFlight: Int = 0,
        sentMessages: Int = 0,
        droppedMessages: Int = 0,
        retriedMessages: Int = 0,
        failedMessages: Int = 0,
        lastError: String? = nil,
        lastErrorAt: Date? = nil
    ) {
        self.capacity = capacity
        self.depth = depth
        self.inFlight = inFlight
        self.sentMessages = sentMessages
        self.droppedMessages = droppedMessages
        self.retriedMessages = retriedMessages
        self.failedMessages = failedMessages
        self.lastError = lastError
        self.lastErrorAt = lastErrorAt
    }
}

enum PublishQueueTransportError: LocalizedError {
    case disconnected

    var errorDescription: String? {
        switch self {
        case .disconnected:
            return "ROS2 WebSocket is not connected."
        }
    }
}

final class PublishQueue {
    enum DropPolicy {
        case dropOldestPublish
    }

    struct Configuration {
        let capacity: Int
        let maxRetries: Int
        let retryDelayMilliseconds: Int
        let dropPolicy: DropPolicy

        static let `default` = Configuration(
            capacity: 120,
            maxRetries: 2,
            retryDelayMilliseconds: 250,
            dropPolicy: .dropOldestPublish
        )
    }

    typealias SendHandler = (Data, @escaping (Error?) -> Void) -> Void

    var onStatsChange: ((PublishQueueStats) -> Void)?

    var capacity: Int {
        configuration.capacity
    }

    private struct Entry {
        let op: String
        let topic: String
        let data: Data
        let generation: Int
        var attempts: Int
    }

    private let queue = DispatchQueue(label: "com.reconstructor.publishQueue", qos: .userInitiated)
    private let configuration: Configuration
    private let sendHandler: SendHandler

    private var pending: [Entry] = []
    private var isSending = false
    private var generation = 0
    private var sentMessages = 0
    private var droppedMessages = 0
    private var retriedMessages = 0
    private var failedMessages = 0
    private var lastError: String?
    private var lastErrorAt: Date?

    init(configuration: Configuration = .default, sendHandler: @escaping SendHandler) {
        self.configuration = configuration
        self.sendHandler = sendHandler
    }

    func reset() {
        queue.async {
            self.generation += 1
            self.pending.removeAll()
            self.isSending = false
            self.sentMessages = 0
            self.droppedMessages = 0
            self.retriedMessages = 0
            self.failedMessages = 0
            self.lastError = nil
            self.lastErrorAt = nil
            self.publishStats()
        }
    }

    func discardPending() {
        queue.async {
            self.generation += 1
            self.pending.removeAll()
            self.isSending = false
            self.publishStats()
        }
    }

    func enqueue(payload: [String: Any], op: String, topic: String) {
        do {
            let data = try JSONSerialization.data(withJSONObject: payload, options: [])
            enqueueEncodedPayload(data, op: op, topic: topic)
        } catch {
            queue.async {
                self.failedMessages += 1
                self.recordError("Failed to encode \(topic): \(error.localizedDescription)")
                self.publishStats()
            }
        }
    }

    func enqueueEncodedPayload(_ data: Data, op: String, topic: String) {
        queue.async {
            let entry = Entry(op: op, topic: topic, data: data, generation: self.generation, attempts: 0)
            guard self.makeRoom(for: entry) else {
                self.publishStats()
                return
            }
            self.pending.append(entry)
            self.publishStats()
            self.drainIfNeeded()
        }
    }

    private func drainIfNeeded() {
        guard !isSending, !pending.isEmpty else { return }

        var entry = pending.removeFirst()
        isSending = true
        publishStats()

        sendHandler(entry.data) { [weak self] error in
            self?.queue.async {
                self?.handleCompletion(entry: &entry, error: error)
            }
        }
    }

    private func handleCompletion(entry: inout Entry, error: Error?) {
        guard entry.generation == generation else { return }

        isSending = false

        if let error {
            recordError("Failed to publish \(entry.topic): \(error.localizedDescription)")

            if shouldRetry(error: error), entry.attempts < configuration.maxRetries {
                entry.attempts += 1
                retriedMessages += 1
                let retryGeneration = entry.generation
                let retryEntry = entry

                queue.asyncAfter(deadline: .now() + .milliseconds(configuration.retryDelayMilliseconds)) {
                    guard retryGeneration == self.generation else { return }
                    let queuedEntry = retryEntry
                    guard self.makeRoom(for: queuedEntry) else {
                        self.publishStats()
                        self.drainIfNeeded()
                        return
                    }
                    self.pending.insert(queuedEntry, at: 0)
                    self.publishStats()
                    self.drainIfNeeded()
                }
            } else {
                failedMessages += 1
            }
        } else {
            sentMessages += 1
        }

        publishStats()
        drainIfNeeded()
    }

    private func shouldRetry(error: Error) -> Bool {
        !(error is PublishQueueTransportError)
    }

    private func makeRoom(for entry: Entry) -> Bool {
        guard pending.count >= configuration.capacity else { return true }

        switch configuration.dropPolicy {
        case .dropOldestPublish:
            if let publishIndex = pending.firstIndex(where: { $0.op == "publish" }) {
                let dropped = pending.remove(at: publishIndex)
                droppedMessages += 1
                recordError("Dropped queued message for \(dropped.topic): publish queue full.")
                return true
            }

            if entry.op == "publish" {
                droppedMessages += 1
                recordError("Dropped incoming message for \(entry.topic): publish queue full.")
                return false
            }

            let dropped = pending.removeFirst()
            droppedMessages += 1
            recordError("Dropped queued message for \(dropped.topic): publish queue full.")
            return true
        }
    }

    private func recordError(_ message: String) {
        lastError = message
        lastErrorAt = Date()
    }

    private func publishStats() {
        onStatsChange?(
            PublishQueueStats(
                capacity: configuration.capacity,
                depth: pending.count,
                inFlight: isSending ? 1 : 0,
                sentMessages: sentMessages,
                droppedMessages: droppedMessages,
                retriedMessages: retriedMessages,
                failedMessages: failedMessages,
                lastError: lastError,
                lastErrorAt: lastErrorAt
            )
        )
    }
}
