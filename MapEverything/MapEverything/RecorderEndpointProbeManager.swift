//
//  RecorderEndpointProbeManager.swift
//  MapEverything
//

import Combine
import Foundation

struct RecorderEndpointProbeSample: Equatable {
    let recorderURL: String
    let roundTripTimeMilliseconds: Double
    let throughputBytesPerSecond: Double
    let throughputPayloadBytes: Int
    let throughputMessageCount: Int
    let throughputElapsedMilliseconds: Double
    let success: Bool
    let error: String
    let timestamp: Date

    var rosMessage: [String: Any] {
        [
            "recorder_url": recorderURL,
            "round_trip_time_ms": roundTripTimeMilliseconds,
            "throughput_bytes_per_second": throughputBytesPerSecond,
            "throughput_payload_bytes": throughputPayloadBytes,
            "throughput_message_count": throughputMessageCount,
            "throughput_elapsed_ms": throughputElapsedMilliseconds,
            "success": success,
            "error": error,
            "timestamp": ISO8601DateFormatter().string(from: timestamp)
        ]
    }
}

final class RecorderEndpointProbeManager: ObservableObject {
    struct Configuration {
        let probeInterval: TimeInterval
        let timeout: TimeInterval
        let throughputPayloadBytes: Int
        let throughputMessageCount: Int
        let probeTopic: String

        static let `default` = Configuration(
            probeInterval: 15,
            timeout: 5,
            throughputPayloadBytes: 16_384,
            throughputMessageCount: 3,
            probeTopic: "/reconstructor/probe/throughput"
        )
    }

    static let shared = RecorderEndpointProbeManager()

    @Published private(set) var isRunning = false
    @Published private(set) var recorderURL = ""
    @Published private(set) var lastSample: RecorderEndpointProbeSample?
    @Published private(set) var lastProbeAt: Date?
    @Published private(set) var lastError: String?

    private let configuration: Configuration
    private var probeTimer: DispatchSourceTimer?
    private var activeProbeID: UUID?
    private var activeTask: URLSessionWebSocketTask?
    private var isProbeInFlight = false

    init(configuration: Configuration = .default) {
        self.configuration = configuration
    }

    var sessionMetadata: [String: Any] {
        var metadata: [String: Any] = [
            "source_api": "URLSessionWebSocketTask.sendPing and rosbridge publish",
            "recorder_url": recorderURL,
            "running": isRunning,
            "probe_interval_seconds": configuration.probeInterval,
            "timeout_seconds": configuration.timeout,
            "throughput_payload_bytes": configuration.throughputPayloadBytes,
            "throughput_message_count": configuration.throughputMessageCount,
            "probe_topic": configuration.probeTopic,
            "last_probe_at": lastProbeAt.map { ISO8601DateFormatter().string(from: $0) } ?? "",
            "last_error": lastError ?? "",
            "limitations": [
                "latency_uses_websocket_ping_pong",
                "throughput_measures_bounded_rosbridge_upload_write_rate",
                "probe_topic_may_be_visible_to_recorder_subscriptions"
            ]
        ]

        if let lastSample {
            metadata["last_sample"] = lastSample.rosMessage
        }

        return metadata
    }

    var diagnosticLevel: Int {
        guard isRunning else { return 1 }
        guard let lastSample else { return 1 }
        return lastSample.success ? 0 : 1
    }

    var diagnosticMessage: String {
        guard isRunning else {
            return "Recorder endpoint probes are not running"
        }
        guard let lastSample else {
            return lastError ?? "Waiting for recorder endpoint probe"
        }
        guard lastSample.success else {
            return "Recorder endpoint probe failed: \(lastSample.error)"
        }

        return String(
            format: "Recorder RTT %.1f ms, upload %.1f KB/s",
            lastSample.roundTripTimeMilliseconds,
            lastSample.throughputBytesPerSecond / 1_024
        )
    }

    var diagnosticValues: [String: String] {
        var values: [String: String] = [
            "running": String(isRunning),
            "source_api": "URLSessionWebSocketTask.sendPing and rosbridge publish",
            "recorder_url": recorderURL,
            "probe_interval_seconds": String(format: "%.1f", configuration.probeInterval),
            "timeout_seconds": String(format: "%.1f", configuration.timeout),
            "throughput_payload_bytes": String(configuration.throughputPayloadBytes),
            "throughput_message_count": String(configuration.throughputMessageCount),
            "probe_topic": configuration.probeTopic,
            "last_probe_at": lastProbeAt.map { ISO8601DateFormatter().string(from: $0) } ?? "",
            "last_error": lastError ?? ""
        ]

        if let lastSample {
            values["round_trip_time_ms"] = String(format: "%.1f", lastSample.roundTripTimeMilliseconds)
            values["throughput_bytes_per_second"] = String(format: "%.1f", lastSample.throughputBytesPerSecond)
            values["throughput_kilobytes_per_second"] = String(format: "%.1f", lastSample.throughputBytesPerSecond / 1_024)
            values["throughput_elapsed_ms"] = String(format: "%.1f", lastSample.throughputElapsedMilliseconds)
            values["success"] = String(lastSample.success)
            values["sample_timestamp"] = ISO8601DateFormatter().string(from: lastSample.timestamp)
            values["error"] = lastSample.error
        }

        return values
    }

    func start(recorderURL: String) {
        self.recorderURL = recorderURL.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !isRunning else { return }

        isRunning = true
        lastError = nil
        scheduleProbeTimer()
        runProbe()
    }

    func stop() {
        isRunning = false
        probeTimer?.cancel()
        probeTimer = nil
        activeProbeID = nil
        isProbeInFlight = false
        activeTask?.cancel(with: .normalClosure, reason: nil)
        activeTask = nil
    }

    private func scheduleProbeTimer() {
        probeTimer?.cancel()

        let timer = DispatchSource.makeTimerSource(queue: .main)
        timer.schedule(deadline: .now() + configuration.probeInterval, repeating: configuration.probeInterval)
        timer.setEventHandler { [weak self] in
            self?.runProbe()
        }
        probeTimer = timer
        timer.resume()
    }

    private func runProbe() {
        guard isRunning, !isProbeInFlight else { return }

        guard let url = URL(string: recorderURL),
              let scheme = url.scheme?.lowercased(),
              ["ws", "wss"].contains(scheme),
              url.host != nil else {
            recordFailure("Invalid ROS2 recorder WebSocket URL.")
            return
        }

        let probeID = UUID()
        activeProbeID = probeID
        isProbeInFlight = true

        let task = URLSession.shared.webSocketTask(with: URLRequest(url: url))
        activeTask = task

        let pingStartedAt = ProcessInfo.processInfo.systemUptime
        task.resume()

        DispatchQueue.main.asyncAfter(deadline: .now() + configuration.timeout) { [weak self] in
            self?.failProbeIfActive(probeID, message: "Recorder endpoint probe timed out.")
        }

        task.sendPing { [weak self] error in
            DispatchQueue.main.async {
                guard let self, self.activeProbeID == probeID else { return }
                if let error {
                    self.completeProbe(
                        probeID,
                        sample: self.failureSample("Recorder WebSocket ping failed: \(error.localizedDescription)")
                    )
                    return
                }

                let roundTripTimeMilliseconds = (ProcessInfo.processInfo.systemUptime - pingStartedAt) * 1_000
                self.runThroughputProbe(
                    task: task,
                    probeID: probeID,
                    roundTripTimeMilliseconds: roundTripTimeMilliseconds
                )
            }
        }
    }

    private func runThroughputProbe(
        task: URLSessionWebSocketTask,
        probeID: UUID,
        roundTripTimeMilliseconds: Double
    ) {
        guard let advertisePayload = encodedRosbridgePayload([
            "op": "advertise",
            "topic": configuration.probeTopic,
            "type": "std_msgs/msg/String"
        ]) else {
            completeProbe(probeID, sample: failureSample("Failed to encode throughput advertise payload."))
            return
        }

        send(payload: advertisePayload, task: task, probeID: probeID) { [weak self] advertiseError in
            guard let self, self.activeProbeID == probeID else { return }
            if let advertiseError {
                self.completeProbe(
                    probeID,
                    sample: self.failureSample("Throughput probe advertise failed: \(advertiseError.localizedDescription)")
                )
                return
            }

            self.publishThroughputPayloads(
                task: task,
                probeID: probeID,
                roundTripTimeMilliseconds: roundTripTimeMilliseconds
            )
        }
    }

    private func publishThroughputPayloads(
        task: URLSessionWebSocketTask,
        probeID: UUID,
        roundTripTimeMilliseconds: Double
    ) {
        let payloadText = String(repeating: "x", count: configuration.throughputPayloadBytes)
        guard let publishPayload = encodedRosbridgePayload([
            "op": "publish",
            "topic": configuration.probeTopic,
            "msg": [
                "data": payloadText
            ]
        ]) else {
            completeProbe(probeID, sample: failureSample("Failed to encode throughput publish payload."))
            return
        }

        let throughputStartedAt = ProcessInfo.processInfo.systemUptime
        let publishBytes = publishPayload.count * configuration.throughputMessageCount

        sendRepeatedly(
            payload: publishPayload,
            remainingCount: configuration.throughputMessageCount,
            task: task,
            probeID: probeID
        ) { [weak self] publishError in
            guard let self, self.activeProbeID == probeID else { return }
            if let publishError {
                self.completeProbe(
                    probeID,
                    sample: self.failureSample("Throughput probe publish failed: \(publishError.localizedDescription)")
                )
                return
            }

            let elapsedSeconds = max(ProcessInfo.processInfo.systemUptime - throughputStartedAt, 0.001)
            let elapsedMilliseconds = elapsedSeconds * 1_000
            let throughputBytesPerSecond = Double(publishBytes) / elapsedSeconds

            self.unadvertiseProbeTopic(task: task, probeID: probeID) {
                self.completeProbe(
                    probeID,
                    sample: RecorderEndpointProbeSample(
                        recorderURL: self.recorderURL,
                        roundTripTimeMilliseconds: roundTripTimeMilliseconds,
                        throughputBytesPerSecond: throughputBytesPerSecond,
                        throughputPayloadBytes: self.configuration.throughputPayloadBytes,
                        throughputMessageCount: self.configuration.throughputMessageCount,
                        throughputElapsedMilliseconds: elapsedMilliseconds,
                        success: true,
                        error: "",
                        timestamp: Date()
                    )
                )
            }
        }
    }

    private func sendRepeatedly(
        payload: Data,
        remainingCount: Int,
        task: URLSessionWebSocketTask,
        probeID: UUID,
        completion: @escaping (Error?) -> Void
    ) {
        guard activeProbeID == probeID else { return }
        guard remainingCount > 0 else {
            completion(nil)
            return
        }

        send(payload: payload, task: task, probeID: probeID) { [weak self] error in
            guard let self, self.activeProbeID == probeID else { return }
            if let error {
                completion(error)
                return
            }
            self.sendRepeatedly(
                payload: payload,
                remainingCount: remainingCount - 1,
                task: task,
                probeID: probeID,
                completion: completion
            )
        }
    }

    private func unadvertiseProbeTopic(
        task: URLSessionWebSocketTask,
        probeID: UUID,
        completion: @escaping () -> Void
    ) {
        guard let unadvertisePayload = encodedRosbridgePayload([
            "op": "unadvertise",
            "topic": configuration.probeTopic
        ]) else {
            completion()
            return
        }

        send(payload: unadvertisePayload, task: task, probeID: probeID) { _ in
            completion()
        }
    }

    private func send(
        payload: Data,
        task: URLSessionWebSocketTask,
        probeID: UUID,
        completion: @escaping (Error?) -> Void
    ) {
        guard activeProbeID == probeID else { return }

        task.send(.data(payload)) { error in
            DispatchQueue.main.async {
                completion(error)
            }
        }
    }

    private func encodedRosbridgePayload(_ payload: [String: Any]) -> Data? {
        try? JSONSerialization.data(withJSONObject: payload, options: [])
    }

    private func failProbeIfActive(_ probeID: UUID, message: String) {
        guard activeProbeID == probeID else { return }
        completeProbe(probeID, sample: failureSample(message))
    }

    private func recordFailure(_ message: String) {
        completeProbe(UUID(), sample: failureSample(message), force: true)
    }

    private func failureSample(_ message: String) -> RecorderEndpointProbeSample {
        RecorderEndpointProbeSample(
            recorderURL: recorderURL,
            roundTripTimeMilliseconds: -1,
            throughputBytesPerSecond: 0,
            throughputPayloadBytes: configuration.throughputPayloadBytes,
            throughputMessageCount: configuration.throughputMessageCount,
            throughputElapsedMilliseconds: 0,
            success: false,
            error: message,
            timestamp: Date()
        )
    }

    private func completeProbe(
        _ probeID: UUID,
        sample: RecorderEndpointProbeSample,
        force: Bool = false
    ) {
        guard force || activeProbeID == probeID else { return }

        lastSample = sample
        lastProbeAt = sample.timestamp
        lastError = sample.success ? nil : sample.error
        isProbeInFlight = false
        activeProbeID = nil
        activeTask?.cancel(with: .normalClosure, reason: nil)
        activeTask = nil
    }
}
