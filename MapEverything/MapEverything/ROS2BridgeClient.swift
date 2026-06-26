//
//  ROS2BridgeClient.swift
//  MapEverything
//

import Foundation
import ARKit
import CoreMotion
import CoreImage
import CoreLocation
import Combine
import UIKit

struct LocalSampleBufferStats: Equatable {
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

class ROS2BridgeClient: ObservableObject {
    private struct OdometrySample {
        let timestamp: TimeInterval
        let position: SIMD3<Float>
        let orientation: simd_quatf
    }

    private enum FrameID {
        static let earth = "earth"
        static let map = "map"
        static let odom = "odom"
        static let baseLink = "base_link"
        static let iphoneCamera = "iphone_camera"
    }

    private enum LocalBufferedSampleKind {
        case pointCloud
        case mesh
    }

    private struct LocalBufferedSample {
        let sequence: UInt64
        let kind: LocalBufferedSampleKind
        let topic: String
        let data: Data
    }

    private struct LocalSampleBufferConfiguration {
        let maxPointCloudSamples: Int
        let maxMeshSamples: Int
        let maxTotalBytes: Int

        static let `default` = LocalSampleBufferConfiguration(
            maxPointCloudSamples: 30,
            maxMeshSamples: 5,
            maxTotalBytes: 20_000_000
        )
    }

    static let shared = ROS2BridgeClient()
    private var webSocket: URLSessionWebSocketTask?
    private let motionManager = CMMotionManager()
    private let ciContext = CIContext() // Reuse context to avoid massive CPU/GPU overhead
    private let motionQueue = OperationQueue()
    private let colorSpace = CGColorSpace(name: CGColorSpace.sRGB) // Reuse to prevent allocation per frame
    private let topicRegistry = ROS2TopicRegistry.shared
    private let meshSnapshotConfiguration = MeshSnapshotPublishConfiguration.default
    private let streamPayloadMetrics = StreamPayloadMetricsStore.shared
    private let localBagRecorder = LocalROS2BagRecorder.shared
    private let localSampleBufferConfiguration = LocalSampleBufferConfiguration.default
    private let localSampleBufferQueue = DispatchQueue(label: "com.mapeverything.localSampleBuffer", qos: .utility)
    private var diagnosticsTimer: DispatchSourceTimer?
    private var reconnectWorkItem: DispatchWorkItem?
    private lazy var publishQueue: PublishQueue = {
        let queue = PublishQueue { [weak self] data, completion in
            self?.sendQueuedPayload(data, completion: completion)
                ?? completion(PublishQueueTransportError.disconnected)
        }
        queue.onStatsChange = { [weak self] stats in
            DispatchQueue.main.async {
                self?.publishQueueStats = stats
            }
        }
        return queue
    }()
    
    private var currentURL: String?
    private var lastOdometrySample: OdometrySample?
    private var bufferedSamples: [LocalBufferedSample] = []
    private var bufferedSampleBytes = 0
    private var bufferedSampleSequence: UInt64 = 0
    private var droppedBufferedSamples = 0
    private var replayedBufferedSamples = 0
    private var lastBufferedSampleAt: Date?
    @Published var isConnected = false
    @Published private(set) var publishQueueStats = PublishQueueStats(
        capacity: PublishQueue.Configuration.default.capacity
    )
    @Published private(set) var localSampleBufferStats = LocalSampleBufferStats(
        maxTotalBytes: LocalSampleBufferConfiguration.default.maxTotalBytes,
        maxPointCloudSamples: LocalSampleBufferConfiguration.default.maxPointCloudSamples,
        maxMeshSamples: LocalSampleBufferConfiguration.default.maxMeshSamples
    )
    
    func connect(to url: String) {
        currentURL = url
        guard let wsURL = URL(string: url) else { return }
        
        // Explicitly close any lingering connection to prevent socket exhaustion on reconnects
        reconnectWorkItem?.cancel()
        reconnectWorkItem = nil
        webSocket?.cancel(with: .normalClosure, reason: nil)
        publishQueue.reset()
        lastOdometrySample = nil
        
        let request = URLRequest(url: wsURL)
        webSocket = URLSession.shared.webSocketTask(with: request)
        webSocket?.resume()
        
        isConnected = true
        
        advertiseTopics()
        flushBufferedLocalSamples()
        startDiagnostics()
        startIMU()
        listenForDisconnection()
    }
    
    func disconnect(after delay: TimeInterval = 0) {
        stopDiagnostics()
        motionManager.stopDeviceMotionUpdates()

        let closeConnection = { [weak self] in
            guard let self else { return }
            self.reconnectWorkItem?.cancel()
            self.reconnectWorkItem = nil
            self.publishQueue.discardPending()
            self.lastOdometrySample = nil
            self.webSocket?.cancel(with: .normalClosure, reason: nil)
            self.clearBufferedLocalSamples()
            DispatchQueue.main.async { self.isConnected = false }
        }

        if delay > 0 {
            DispatchQueue.main.asyncAfter(deadline: .now() + delay) {
                closeConnection()
            }
        } else {
            closeConnection()
        }
    }
    
    private func listenForDisconnection() {
        webSocket?.receive { [weak self] result in
            switch result {
            case .success(_):
                self?.listenForDisconnection()
            case .failure(let error):
                self?.handleConnectionFailure(error)
            }
        }
    }
    
    private func attemptReconnect() {
        guard let url = currentURL else { return }
        guard reconnectWorkItem == nil else { return }

        let workItem = DispatchWorkItem { [weak self] in
            DispatchQueue.main.async {
                self?.reconnectWorkItem = nil
                if UserDefaults.standard.bool(forKey: "ros2Enabled") && !(self?.isConnected ?? false) {
                    self?.connect(to: url)
                }
            }
        }
        reconnectWorkItem = workItem
        DispatchQueue.global().asyncAfter(deadline: .now() + 3.0, execute: workItem)
    }

    private func handleConnectionFailure(_ error: Error) {
        DispatchQueue.main.async {
            guard self.isConnected || self.webSocket != nil else { return }
            print("ROS2 Bridge connection unavailable: \(error.localizedDescription)")
            self.stopDiagnostics()
            self.motionManager.stopDeviceMotionUpdates()
            self.publishQueue.discardPending()
            self.lastOdometrySample = nil
            self.webSocket?.cancel(with: .goingAway, reason: nil)
            self.webSocket = nil
            self.isConnected = false
            self.attemptReconnect()
        }
    }
    
    private func advertiseTopics() {
        for definition in topicRegistry.advertisedTopics() {
            send(op: "advertise", topic: definition.topic, type: definition.messageType)
        }
    }

    private func startDiagnostics() {
        stopDiagnostics()
        guard topicRegistry.isStreamEnabled(.diagnostics) else { return }

        let timer = DispatchSource.makeTimerSource(queue: .main)
        timer.schedule(deadline: .now() + 1.0, repeating: 1.0)
        timer.setEventHandler { [weak self] in
            self?.publishDiagnostics()
        }
        diagnosticsTimer = timer
        timer.resume()
    }

    private func stopDiagnostics() {
        diagnosticsTimer?.cancel()
        diagnosticsTimer = nil
    }
    
    private func send(op: String, topic: String, type: String? = nil, msg: [String: Any]? = nil) {
        var payload: [String: Any] = ["op": op, "topic": topic]
        if let type = type { payload["type"] = type }
        if let msg = msg { payload["msg"] = msg }

        guard let data = encodeRosbridgePayload(payload, topic: topic) else { return }

        if op == "publish", let msg {
            recordLocalBagPublish(topic: topic, msg: msg, encodedData: data)
        }

        publishQueue.enqueueEncodedPayload(data, op: op, topic: topic)
    }

    private func publishOrBufferLocalSample(
        kind: LocalBufferedSampleKind,
        topic: String,
        msg: [String: Any]
    ) {
        let payload: [String: Any] = [
            "op": "publish",
            "topic": topic,
            "msg": msg
        ]

        guard let data = encodeRosbridgePayload(payload, topic: topic) else { return }
        recordLocalBagPublish(topic: topic, msg: msg, encodedData: data)

        if isConnected {
            publishQueue.enqueueEncodedPayload(data, op: "publish", topic: topic)
        } else {
            bufferLocalSample(kind: kind, topic: topic, data: data)
        }
    }

    private func encodeRosbridgePayload(_ payload: [String: Any], topic: String) -> Data? {
        guard JSONSerialization.isValidJSONObject(payload) else {
            publishQueue.recordEncodingFailure(topic: topic, error: PublishQueueEncodingError.invalidJSONObject)
            return nil
        }

        do {
            return try JSONSerialization.data(withJSONObject: payload, options: [])
        } catch {
            publishQueue.recordEncodingFailure(topic: topic, error: error)
            return nil
        }
    }

    private func recordLocalBagPublish(topic: String, msg: [String: Any], encodedData data: Data) {
        let messageType = topicRegistry.definition(forTopic: topic)?.messageType ?? "unknown_msgs/msg/Unknown"
        let timestamp = LocalROS2BagRecorder.timestampNanoseconds(from: msg) ?? LocalROS2BagRecorder.nowNanoseconds()
        localBagRecorder.recordEncodedPublishPayload(
            data,
            topic: topic,
            messageType: messageType,
            timestampNanoseconds: timestamp
        )
    }

    private func bufferLocalSample(kind: LocalBufferedSampleKind, topic: String, data: Data) {
        localSampleBufferQueue.async {
            self.bufferedSampleSequence += 1
            self.bufferedSamples.append(
                LocalBufferedSample(
                    sequence: self.bufferedSampleSequence,
                    kind: kind,
                    topic: topic,
                    data: data
                )
            )
            self.bufferedSampleBytes += data.count
            self.lastBufferedSampleAt = Date()
            self.trimBufferedLocalSamples()
            self.publishLocalSampleBufferStats()
        }
    }

    private func trimBufferedLocalSamples() {
        while countBufferedSamples(kind: .pointCloud) > localSampleBufferConfiguration.maxPointCloudSamples {
            dropOldestBufferedSample(kind: .pointCloud)
        }

        while countBufferedSamples(kind: .mesh) > localSampleBufferConfiguration.maxMeshSamples {
            dropOldestBufferedSample(kind: .mesh)
        }

        while bufferedSampleBytes > localSampleBufferConfiguration.maxTotalBytes,
              !bufferedSamples.isEmpty {
            dropBufferedSample(at: 0)
        }
    }

    private func countBufferedSamples(kind: LocalBufferedSampleKind) -> Int {
        bufferedSamples.reduce(0) { count, sample in
            count + (sample.kind == kind ? 1 : 0)
        }
    }

    private func dropOldestBufferedSample(kind: LocalBufferedSampleKind) {
        guard let index = bufferedSamples.firstIndex(where: { $0.kind == kind }) else { return }
        dropBufferedSample(at: index)
    }

    private func dropBufferedSample(at index: Int) {
        let sample = bufferedSamples.remove(at: index)
        bufferedSampleBytes -= sample.data.count
        droppedBufferedSamples += 1
    }

    private func flushBufferedLocalSamples() {
        localSampleBufferQueue.async {
            let samples = self.bufferedSamples.sorted { $0.sequence < $1.sequence }
            guard !samples.isEmpty else {
                self.publishLocalSampleBufferStats()
                return
            }

            self.bufferedSamples.removeAll()
            self.bufferedSampleBytes = 0
            self.replayedBufferedSamples += samples.count
            self.publishLocalSampleBufferStats()

            for sample in samples {
                self.publishQueue.enqueueEncodedPayload(sample.data, op: "publish", topic: sample.topic)
            }
        }
    }

    private func clearBufferedLocalSamples() {
        localSampleBufferQueue.async {
            self.bufferedSamples.removeAll()
            self.bufferedSampleBytes = 0
            self.lastBufferedSampleAt = nil
            self.publishLocalSampleBufferStats()
        }
    }

    private func publishLocalSampleBufferStats() {
        let pointCloudSamples = countBufferedSamples(kind: .pointCloud)
        let meshSamples = countBufferedSamples(kind: .mesh)
        let stats = LocalSampleBufferStats(
            maxTotalBytes: localSampleBufferConfiguration.maxTotalBytes,
            maxPointCloudSamples: localSampleBufferConfiguration.maxPointCloudSamples,
            maxMeshSamples: localSampleBufferConfiguration.maxMeshSamples,
            totalBytes: bufferedSampleBytes,
            pointCloudSamples: pointCloudSamples,
            meshSamples: meshSamples,
            droppedSamples: droppedBufferedSamples,
            replayedSamples: replayedBufferedSamples,
            lastBufferedAt: lastBufferedSampleAt
        )

        DispatchQueue.main.async {
            self.localSampleBufferStats = stats
        }
    }

    private func sendQueuedPayload(_ data: Data, completion: @escaping (Error?) -> Void) {
        guard isConnected, let webSocket else {
            completion(PublishQueueTransportError.disconnected)
            return
        }

        let message = URLSessionWebSocketTask.Message.data(data)
        webSocket.send(message) { [weak self] error in
            if let error = error {
                self?.handleConnectionFailure(error)
            }
            completion(error)
        }
    }
    
    private func createHeader(frameId: String, timestamp: TimeInterval) -> [String: Any] {
        // Convert Apple's system uptime (ARKit/IMU hardware clock) accurately to the UNIX Epoch for ROS2
        let systemUptime = ProcessInfo.processInfo.systemUptime
        let nowUnix = Date().timeIntervalSince1970
        let hardwareUnix = nowUnix - systemUptime + timestamp
        
        let sec = Int(hardwareUnix)
        let nanosec = Int((hardwareUnix - Double(sec)) * 1_000_000_000)
        return [
            "stamp": ["sec": sec, "nanosec": nanosec],
            "frame_id": frameId
        ]
    }

    private func createHeader(frameId: String, date: Date) -> [String: Any] {
        let timestamp = date.timeIntervalSince1970
        let sec = Int(timestamp)
        let nanosec = Int((timestamp - Double(sec)) * 1_000_000_000)
        return [
            "stamp": ["sec": sec, "nanosec": nanosec],
            "frame_id": frameId
        ]
    }

    private func finiteROSNumber(_ value: Double, fallback: Double = -1) -> Double {
        value.isFinite ? value : fallback
    }

    private func finiteROSNumber(_ value: Float, fallback: Double = -1) -> Double {
        value.isFinite ? Double(value) : fallback
    }

    private func rosJSONString(_ value: Any, fallback: String = "{}") -> String {
        let normalized = rosJSONCompatible(value)
        guard JSONSerialization.isValidJSONObject(normalized),
              let data = try? JSONSerialization.data(withJSONObject: normalized, options: [.sortedKeys]),
              let string = String(data: data, encoding: .utf8) else {
            return fallback
        }
        return string
    }

    private func rosJSONCompatible(_ value: Any) -> Any {
        switch value {
        case let dictionary as [String: Any]:
            return Dictionary(uniqueKeysWithValues: dictionary.map { key, value in
                (key, rosJSONCompatible(value))
            })
        case let array as [Any]:
            return array.map { rosJSONCompatible($0) }
        case let value as Double:
            return finiteROSNumber(value, fallback: 0)
        case let value as Float:
            return finiteROSNumber(value, fallback: 0)
        case let value as String:
            return value
        case let value as Bool:
            return value
        case let value as NSNumber:
            return value
        default:
            return String(describing: value)
        }
    }
    
    // MARK: - Publishers
    
    func publishPose(_ transform: simd_float4x4, timestamp: TimeInterval) {
        guard isConnected, topicRegistry.isStreamEnabled(.pose) else { return }
        let pos = transform.columns.3
        let quat = simd_quatf(transform)
        
        let msg: [String: Any] = [
            "header": createHeader(frameId: FrameID.map, timestamp: timestamp),
            "pose": [
                "position": ["x": pos.x, "y": pos.y, "z": pos.z],
                "orientation": ["x": quat.vector.x, "y": quat.vector.y, "z": quat.vector.z, "w": quat.vector.w]
            ]
        ]
        send(op: "publish", topic: topicRegistry.topic(.pose), msg: msg)
    }

    func publishOdometry(_ transform: simd_float4x4, timestamp: TimeInterval) {
        guard isConnected, topicRegistry.isStreamEnabled(.odometry) else { return }

        let position = SIMD3<Float>(
            transform.columns.3.x,
            transform.columns.3.y,
            transform.columns.3.z
        )
        let orientation = simd_quatf(transform)
        let twist = odometryTwist(position: position, orientation: orientation, timestamp: timestamp)
        lastOdometrySample = OdometrySample(
            timestamp: timestamp,
            position: position,
            orientation: orientation
        )

        let msg: [String: Any] = [
            "header": createHeader(frameId: FrameID.odom, timestamp: timestamp),
            "child_frame_id": FrameID.baseLink,
            "pose": [
                "pose": [
                    "position": [
                        "x": finiteROSNumber(position.x, fallback: 0),
                        "y": finiteROSNumber(position.y, fallback: 0),
                        "z": finiteROSNumber(position.z, fallback: 0)
                    ],
                    "orientation": [
                        "x": finiteROSNumber(orientation.vector.x, fallback: 0),
                        "y": finiteROSNumber(orientation.vector.y, fallback: 0),
                        "z": finiteROSNumber(orientation.vector.z, fallback: 0),
                        "w": finiteROSNumber(orientation.vector.w, fallback: 1)
                    ]
                ],
                "covariance": odometryPoseCovariance()
            ],
            "twist": [
                "twist": [
                    "linear": [
                        "x": finiteROSNumber(twist.linear.x, fallback: 0),
                        "y": finiteROSNumber(twist.linear.y, fallback: 0),
                        "z": finiteROSNumber(twist.linear.z, fallback: 0)
                    ],
                    "angular": [
                        "x": finiteROSNumber(twist.angular.x, fallback: 0),
                        "y": finiteROSNumber(twist.angular.y, fallback: 0),
                        "z": finiteROSNumber(twist.angular.z, fallback: 0)
                    ]
                ],
                "covariance": odometryTwistCovariance()
            ]
        ]

        send(op: "publish", topic: topicRegistry.topic(.odom), msg: msg)
    }
    
    func publishTF(_ transform: simd_float4x4, timestamp: TimeInterval) {
        guard isConnected, topicRegistry.isStreamEnabled(.tf) else { return }
        let pos = transform.columns.3
        let quat = simd_quatf(transform)

        var transforms: [[String: Any]] = [
            identityTransformMessage(
                parentFrameID: FrameID.map,
                childFrameID: FrameID.odom,
                timestamp: timestamp
            ),
            transformMessage(
                parentFrameID: FrameID.odom,
                childFrameID: FrameID.baseLink,
                timestamp: timestamp,
                translation: SIMD3<Double>(Double(pos.x), Double(pos.y), Double(pos.z)),
                rotation: SIMD4<Double>(
                    Double(quat.vector.x),
                    Double(quat.vector.y),
                    Double(quat.vector.z),
                    Double(quat.vector.w)
                )
            ),
            identityTransformMessage(
                parentFrameID: FrameID.baseLink,
                childFrameID: FrameID.iphoneCamera,
                timestamp: timestamp
            )
        ]

        if let earthToMap = MapGeoreferencer.shared.earthToMapTransform() {
            transforms.insert(
                transformMessage(
                    parentFrameID: FrameID.earth,
                    childFrameID: FrameID.map,
                    timestamp: timestamp,
                    translation: earthToMap.translationMeters,
                    rotation: earthToMap.rotation.vector
                ),
                at: 0
            )
        }

        let msg: [String: Any] = ["transforms": transforms]
        send(op: "publish", topic: topicRegistry.topic(.tf), msg: msg)
    }

    private func identityTransformMessage(
        parentFrameID: String,
        childFrameID: String,
        timestamp: TimeInterval
    ) -> [String: Any] {
        transformMessage(
            parentFrameID: parentFrameID,
            childFrameID: childFrameID,
            timestamp: timestamp,
            translation: .zero,
            rotation: SIMD4<Double>(0, 0, 0, 1)
        )
    }

    private func transformMessage(
        parentFrameID: String,
        childFrameID: String,
        timestamp: TimeInterval,
        translation: SIMD3<Double>,
        rotation: SIMD4<Double>
    ) -> [String: Any] {
        [
            "header": createHeader(frameId: parentFrameID, timestamp: timestamp),
            "child_frame_id": childFrameID,
            "transform": [
                "translation": [
                    "x": finiteROSNumber(translation.x, fallback: 0),
                    "y": finiteROSNumber(translation.y, fallback: 0),
                    "z": finiteROSNumber(translation.z, fallback: 0)
                ],
                "rotation": [
                    "x": finiteROSNumber(rotation.x, fallback: 0),
                    "y": finiteROSNumber(rotation.y, fallback: 0),
                    "z": finiteROSNumber(rotation.z, fallback: 0),
                    "w": finiteROSNumber(rotation.w, fallback: 1)
                ]
            ]
        ]
    }
    
    static func makeCameraInfoMessage(
        header: [String: Any],
        intrinsics: simd_float3x3,
        imageResolution: CGSize
    ) -> [String: Any] {
        let width = Int(imageResolution.width.rounded())
        let height = Int(imageResolution.height.rounded())
        let fx = Double(intrinsics[0][0])
        let fy = Double(intrinsics[1][1])
        let cx = Double(intrinsics[2][0])
        let cy = Double(intrinsics[2][1])

        return [
            "header": header,
            "height": height,
            "width": width,
            "distortion_model": "plumb_bob",
            "d": [0.0, 0.0, 0.0, 0.0, 0.0],
            "k": [
                fx, 0.0, cx,
                0.0, fy, cy,
                0.0, 0.0, 1.0
            ],
            "r": [
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0
            ],
            "p": [
                fx, 0.0, cx, 0.0,
                0.0, fy, cy, 0.0,
                0.0, 0.0, 1.0, 0.0
            ],
            "binning_x": 0,
            "binning_y": 0,
            "roi": [
                "x_offset": 0,
                "y_offset": 0,
                "height": 0,
                "width": 0,
                "do_rectify": false
            ]
        ]
    }

    func publishImage(frame: ARFrame, timestamp: TimeInterval) {
        publishImage(
            pixelBuffer: frame.capturedImage,
            intrinsics: frame.camera.intrinsics,
            imageResolution: frame.camera.imageResolution,
            timestamp: timestamp
        )
    }

    func publishImage(
        pixelBuffer: CVPixelBuffer,
        intrinsics: simd_float3x3,
        imageResolution: CGSize,
        timestamp: TimeInterval
    ) {
        guard isConnected, topicRegistry.isStreamEnabled(.camera) else { return }

        let header = createHeader(frameId: FrameID.iphoneCamera, timestamp: timestamp)
        let ciImage = CIImage(cvPixelBuffer: pixelBuffer)

        guard let safeColorSpace = colorSpace,
              let jpegData = ciContext.jpegRepresentation(of: ciImage, colorSpace: safeColorSpace, options: [kCGImageDestinationLossyCompressionQuality as CIImageRepresentationOption: 0.4]) else { return }
        let encodedImage = jpegData.base64EncodedString()
        recordStreamPayloadMetric(
            stream: .camera,
            originalBytes: CVPixelBufferGetDataSize(pixelBuffer),
            encodedBytes: encodedImage.utf8.count,
            compression: "jpeg_q0.4_base64"
        )
        
        let msg: [String: Any] = [
            "header": header,
            "format": "jpeg",
            "data": encodedImage
        ]
        send(op: "publish", topic: topicRegistry.topic(.cameraCompressed), msg: msg)

        let cameraInfo = Self.makeCameraInfoMessage(
            header: header,
            intrinsics: intrinsics,
            imageResolution: imageResolution
        )
        send(op: "publish", topic: topicRegistry.topic(.cameraInfo), msg: cameraInfo)
    }
    
    func publishLiDARPointCloud(_ points: [ColoredPoint], timestamp: TimeInterval) {
        publishPointCloud(points, topicID: .lidarPointCloud, frameID: FrameID.map, timestamp: timestamp)
    }

    func publishDepthAnythingPointCloud(_ points: [ColoredPoint], timestamp: TimeInterval) {
        publishPointCloud(points, topicID: .depthAnythingPointCloud, frameID: FrameID.iphoneCamera, timestamp: timestamp)
    }

    func publishDepthAnythingCalibration(
        _ calibration: DepthAnythingProcessor.MaximumLikelihoodCalibration,
        relativeDepthSize: CGSize,
        imageResolution: CGSize,
        timestamp: TimeInterval
    ) {
        guard topicRegistry.isStreamEnabled(.pointCloud) else { return }

        let topic = topicRegistry.topic(.depthAnythingCalibration)
        let msg = Self.makeDepthAnythingCalibrationMessage(
            calibration: calibration,
            header: createHeader(frameId: FrameID.iphoneCamera, timestamp: timestamp),
            relativeDepthSize: relativeDepthSize,
            imageResolution: imageResolution,
            relativePointCloudTopic: topicRegistry.topic(.depthAnythingPointCloud),
            frameID: FrameID.iphoneCamera
        )
        let encodedBytes = encodedPublishPayloadByteCount(topic: topic, msg: msg) ?? 0
        recordStreamPayloadMetric(
            stream: .pointCloud,
            originalBytes: MemoryLayout<Float>.stride * 2,
            encodedBytes: encodedBytes,
            compression: "depthanything_calibration_json"
        )

        publishOrBufferLocalSample(
            kind: .pointCloud,
            topic: topic,
            msg: msg
        )
    }

    func publishPointCloud(_ points: [ColoredPoint], timestamp: TimeInterval) {
        publishDepthAnythingPointCloud(points, timestamp: timestamp)
    }

    private func publishPointCloud(
        _ points: [ColoredPoint],
        topicID: ROS2TopicID,
        frameID: String,
        timestamp: TimeInterval
    ) {
        guard topicRegistry.isStreamEnabled(.pointCloud), !points.isEmpty else { return }

        let topic = topicRegistry.topic(topicID)
        let msg = Self.makeColoredPointCloudMessage(
            points: points,
            header: createHeader(frameId: frameID, timestamp: timestamp)
        )
        let dataCount = points.count * 16
        let encodedPointData = msg["data"] as? String ?? ""
        recordStreamPayloadMetric(
            stream: .pointCloud,
            originalBytes: dataCount,
            encodedBytes: encodedPointData.utf8.count,
            compression: "pointcloud2_binary_base64"
        )

        publishOrBufferLocalSample(
            kind: .pointCloud,
            topic: topic,
            msg: msg
        )
    }

    static func makeDepthAnythingCalibrationMessage(
        calibration: DepthAnythingProcessor.MaximumLikelihoodCalibration,
        header: [String: Any],
        relativeDepthSize: CGSize,
        imageResolution: CGSize,
        relativePointCloudTopic: String,
        frameID: String
    ) -> [String: Any] {
        let metadata: [String: Any] = [
            "relative_pointcloud_coordinate_frame": "camera",
            "relative_pointcloud_semantics": "x_y_z_are_camera_ray_coordinates_scaled_by_raw_depthanything_relative_depth",
            "metric_reconstruction": "metric_depth_m = scale * relative_depth + offset; recompute camera ray coordinates with the same intrinsics before transforming to map",
            "uses_lidar_for_scale_calibration": true,
            "overlay_mesh_uses_calibrated_depth": true
        ]

        let metadataJSON: String
        if JSONSerialization.isValidJSONObject(metadata),
           let data = try? JSONSerialization.data(withJSONObject: metadata, options: [.sortedKeys]),
           let string = String(data: data, encoding: .utf8) {
            metadataJSON = string
        } else {
            metadataJSON = "{}"
        }

        return [
            "header": header,
            "schema_version": 1,
            "source": "depth_anything_v2_lidar_calibrated",
            "relative_pointcloud_topic": relativePointCloudTopic,
            "overlay_mesh_source": "calibrated_depthanything_grid",
            "frame_id": frameID,
            "relative_depth_width": max(0, Int(relativeDepthSize.width.rounded())),
            "relative_depth_height": max(0, Int(relativeDepthSize.height.rounded())),
            "image_width": max(0, Int(imageResolution.width.rounded())),
            "image_height": max(0, Int(imageResolution.height.rounded())),
            "scale": Double(calibration.scale),
            "offset": Double(calibration.offset),
            "equation": "metric_depth_m = scale * relative_depth + offset",
            "relative_depth_units": "depthanything_relative",
            "metric_depth_units": "m",
            "calibration_source": "arkit_lidar_maximum_likelihood",
            "metadata_json": metadataJSON
        ]
    }

    static func makeColoredPointCloudMessage(
        points: [ColoredPoint],
        header: [String: Any]
    ) -> [String: Any] {
        // Pack points and colors into a tight Base64 byte array for PointCloud2
        let pointStep = 16
        let dataCount = points.count * pointStep
        var data = Data(count: dataCount)
        
        data.withUnsafeMutableBytes { rawBuffer in
            guard let pointer = rawBuffer.bindMemory(to: UInt8.self).baseAddress else { return }
            var offset = 0
            for p in points {
                var x = p.position.x; var y = p.position.y; var z = p.position.z
                memcpy(pointer + offset, &x, 4); offset += 4
                memcpy(pointer + offset, &y, 4); offset += 4
                memcpy(pointer + offset, &z, 4); offset += 4
                var rgb: UInt32 = (UInt32(p.color.x) << 16) | (UInt32(p.color.y) << 8) | UInt32(p.color.z)
                memcpy(pointer + offset, &rgb, 4); offset += 4
            }
        }
        
        return [
            "header": header,
            "height": 1,
            "width": points.count,
            "fields": [
                ["name": "x", "offset": 0, "datatype": 7, "count": 1],
                ["name": "y", "offset": 4, "datatype": 7, "count": 1],
                ["name": "z", "offset": 8, "datatype": 7, "count": 1],
                ["name": "rgb", "offset": 12, "datatype": 6, "count": 1]
            ],
            "is_bigendian": false,
            "point_step": pointStep,
            "row_step": points.count * pointStep,
            "data": data.base64EncodedString(),
            "is_dense": true
        ]
    }

    static func makeSurfelPointCloudMessage(
        surfels: [ColoredSurfel],
        header: [String: Any]
    ) -> [String: Any] {
        let pointStep = 40
        let dataCount = surfels.count * pointStep
        var data = Data(count: dataCount)

        data.withUnsafeMutableBytes { rawBuffer in
            guard let pointer = rawBuffer.bindMemory(to: UInt8.self).baseAddress else { return }
            var offset = 0
            for surfel in surfels {
                var x = surfel.position.x; var y = surfel.position.y; var z = surfel.position.z
                var nx = surfel.normal.x; var ny = surfel.normal.y; var nz = surfel.normal.z
                var radius = surfel.radius
                var confidence = surfel.confidence
                var rgb: UInt32 = (UInt32(surfel.color.x) << 16) | (UInt32(surfel.color.y) << 8) | UInt32(surfel.color.z)
                var observationCount = surfel.observationCount
                memcpy(pointer + offset, &x, 4); offset += 4
                memcpy(pointer + offset, &y, 4); offset += 4
                memcpy(pointer + offset, &z, 4); offset += 4
                memcpy(pointer + offset, &nx, 4); offset += 4
                memcpy(pointer + offset, &ny, 4); offset += 4
                memcpy(pointer + offset, &nz, 4); offset += 4
                memcpy(pointer + offset, &radius, 4); offset += 4
                memcpy(pointer + offset, &confidence, 4); offset += 4
                memcpy(pointer + offset, &rgb, 4); offset += 4
                memcpy(pointer + offset, &observationCount, 4); offset += 4
            }
        }

        return [
            "header": header,
            "height": 1,
            "width": surfels.count,
            "fields": [
                ["name": "x", "offset": 0, "datatype": 7, "count": 1],
                ["name": "y", "offset": 4, "datatype": 7, "count": 1],
                ["name": "z", "offset": 8, "datatype": 7, "count": 1],
                ["name": "normal_x", "offset": 12, "datatype": 7, "count": 1],
                ["name": "normal_y", "offset": 16, "datatype": 7, "count": 1],
                ["name": "normal_z", "offset": 20, "datatype": 7, "count": 1],
                ["name": "radius", "offset": 24, "datatype": 7, "count": 1],
                ["name": "confidence", "offset": 28, "datatype": 7, "count": 1],
                ["name": "rgb", "offset": 32, "datatype": 6, "count": 1],
                ["name": "observation_count", "offset": 36, "datatype": 6, "count": 1]
            ],
            "is_bigendian": false,
            "point_step": pointStep,
            "row_step": surfels.count * pointStep,
            "data": data.base64EncodedString(),
            "is_dense": true
        ]
    }

    func publishSurfels(_ surfels: [ColoredSurfel], timestamp: TimeInterval) {
        // Surfels remain an internal reconstruction/export format. ROS output
        // uses source-specific /mapping/pointcloud/... PointCloud2 topics.
    }
    
    func publishMap(meshAnchors: [ARMeshAnchor], timestamp: TimeInterval) {
        publishMap(safeMeshes: MeshGenerator.extractSafeMeshes(from: meshAnchors), timestamp: timestamp)
    }

    func publishMap(safeMeshes: [SafeARMesh], timestamp: TimeInterval) {
        guard topicRegistry.isStreamEnabled(.mesh), !safeMeshes.isEmpty else { return }

        var markers: [[String: Any]] = []
        var trianglePointsIncluded = 0
        for mesh in safeMeshes {
            let remainingPointBudget = meshSnapshotConfiguration.maxTrianglePoints - trianglePointsIncluded
            guard remainingPointBudget >= 3 else { break }

            let points = meshTrianglePoints(for: mesh, maxPointCount: remainingPointBudget)
            guard !points.isEmpty else { continue }
            trianglePointsIncluded += points.count

            let markerId = abs(mesh.identifier.hashValue) % Int(Int32.max)

            let marker: [String: Any] = [
                "header": createHeader(frameId: "map", timestamp: timestamp),
                "ns": "mesh",
                "id": markerId,
                "type": 11, // TRIANGLE_LIST
                "action": 0, // ADD
                "pose": [
                    "position": ["x": 0, "y": 0, "z": 0],
                    "orientation": ["x": 0, "y": 0, "z": 0, "w": 1]
                ],
                "scale": ["x": 1.0, "y": 1.0, "z": 1.0],
                "color": ["r": 0.5, "g": 0.5, "b": 0.5, "a": 0.5],
                "points": points
            ]
            markers.append(marker)
        }

        let topic = topicRegistry.topic(.meshMarkers)
        if !markers.isEmpty {
            let fittedMarkers = fitMeshMarkersToPayloadLimit(
                markers,
                topic: topic,
                maxPayloadBytes: meshSnapshotConfiguration.maxPayloadBytes
            )
            if !fittedMarkers.isEmpty {
                let markerMessage: [String: Any] = ["markers": fittedMarkers]
                if let encodedBytes = encodedPublishPayloadByteCount(topic: topic, msg: markerMessage) {
                    recordStreamPayloadMetric(
                        stream: .mesh,
                        originalBytes: trianglePointsIncluded * 3 * MemoryLayout<Float>.size,
                        encodedBytes: encodedBytes,
                        compression: "visualization_marker_array_json"
                    )
                }
                publishOrBufferLocalSample(kind: .mesh, topic: topic, msg: markerMessage)
            }
        }

        let snapshotTopic = topicRegistry.topic(.meshSnapshot)
        let snapshotMessage = MeshSnapshotMessageBuilder.makeSafeMeshMessage(
            header: createHeader(frameId: FrameID.map, timestamp: timestamp),
            snapshotID: UUID().uuidString,
            source: "arkit_mesh",
            frameID: FrameID.map,
            safeMeshes: safeMeshes,
            maxTrianglePoints: meshSnapshotConfiguration.maxTrianglePoints,
            maxPayloadBytes: meshSnapshotConfiguration.maxPayloadBytes,
            compression: "mesh_snapshot_binary_base64",
            metadata: [
                "fallback_marker_topic": topic,
                "max_payload_bytes": meshSnapshotConfiguration.maxPayloadBytes,
                "max_triangle_points": meshSnapshotConfiguration.maxTrianglePoints
            ],
            topic: snapshotTopic
        )
        guard (snapshotMessage["triangle_count"] as? Int ?? 0) > 0 else { return }
        let originalSnapshotBytes = snapshotMessage["original_payload_bytes"] as? Int ?? 0
        let publishedSnapshotBytes = snapshotMessage["published_payload_bytes"] as? Int
            ?? encodedPublishPayloadByteCount(topic: snapshotTopic, msg: snapshotMessage)
            ?? 0
        recordStreamPayloadMetric(
            stream: .mesh,
            originalBytes: originalSnapshotBytes,
            encodedBytes: publishedSnapshotBytes,
            compression: "mesh_snapshot_binary_base64"
        )
        publishOrBufferLocalSample(kind: .mesh, topic: snapshotTopic, msg: snapshotMessage)
    }

    private func meshTrianglePoints(for mesh: SafeARMesh, maxPointCount: Int) -> [[String: Float]] {
        let maxFaceCount = min(mesh.indices.count / 3, maxPointCount / 3)
        guard maxFaceCount > 0 else { return [] }

        var worldVertices: [simd_float3] = []
        worldVertices.reserveCapacity(mesh.vertices.count)
        for vertex in mesh.vertices {
            let worldVertex = simd_mul(mesh.transform, simd_float4(vertex.x, vertex.y, vertex.z, 1.0))
            worldVertices.append(simd_float3(worldVertex.x, worldVertex.y, worldVertex.z))
        }

        var points: [[String: Float]] = []
        points.reserveCapacity(maxFaceCount * 3)
        for faceIndex in 0..<maxFaceCount {
            let baseIndex = faceIndex * 3
            let v1 = Int(mesh.indices[baseIndex])
            let v2 = Int(mesh.indices[baseIndex + 1])
            let v3 = Int(mesh.indices[baseIndex + 2])

            guard worldVertices.indices.contains(v1),
                  worldVertices.indices.contains(v2),
                  worldVertices.indices.contains(v3) else { continue }

            let p1 = worldVertices[v1]
            let p2 = worldVertices[v2]
            let p3 = worldVertices[v3]
            points.append(["x": p1.x, "y": p1.y, "z": p1.z])
            points.append(["x": p2.x, "y": p2.y, "z": p2.z])
            points.append(["x": p3.x, "y": p3.y, "z": p3.z])
        }

        return points
    }

    private func meshTrianglePoints(for anchor: ARMeshAnchor, maxPointCount: Int) -> [[String: Float]] {
        let geometry = anchor.geometry
        let transform = anchor.transform
        let maxFaceCount = min(geometry.faces.count, maxPointCount / 3)
        guard maxFaceCount > 0 else { return [] }

        let verticesPointer = geometry.vertices.buffer.contents()
        let verticesStride = geometry.vertices.stride
        let verticesOffset = geometry.vertices.offset

        var worldVertices: [simd_float3] = []
        worldVertices.reserveCapacity(geometry.vertices.count)
        for vIdx in 0..<geometry.vertices.count {
            let pointer = verticesPointer.advanced(by: verticesOffset + (vIdx * verticesStride))
            let vertex = pointer.assumingMemoryBound(to: SIMD3<Float>.self).pointee
            let worldVertex = simd_mul(transform, simd_float4(vertex.x, vertex.y, vertex.z, 1.0))
            worldVertices.append(simd_float3(worldVertex.x, worldVertex.y, worldVertex.z))
        }

        let facesPointer = geometry.faces.buffer.contents()
        let bytesPerIndex = geometry.faces.bytesPerIndex
        let facesStride = geometry.faces.indexCountPerPrimitive * bytesPerIndex

        var points: [[String: Float]] = []
        points.reserveCapacity(maxFaceCount * 3)
        for fIdx in 0..<maxFaceCount {
            let pointer = facesPointer.advanced(by: fIdx * facesStride)

            let v1: Int
            let v2: Int
            let v3: Int
            if bytesPerIndex == 2 {
                let typedPointer = pointer.assumingMemoryBound(to: UInt16.self)
                v1 = Int(typedPointer[0])
                v2 = Int(typedPointer[1])
                v3 = Int(typedPointer[2])
            } else {
                let typedPointer = pointer.assumingMemoryBound(to: UInt32.self)
                v1 = Int(typedPointer[0])
                v2 = Int(typedPointer[1])
                v3 = Int(typedPointer[2])
            }

            guard worldVertices.indices.contains(v1),
                  worldVertices.indices.contains(v2),
                  worldVertices.indices.contains(v3) else { continue }

            let p1 = worldVertices[v1]
            let p2 = worldVertices[v2]
            let p3 = worldVertices[v3]
            points.append(["x": p1.x, "y": p1.y, "z": p1.z])
            points.append(["x": p2.x, "y": p2.y, "z": p2.z])
            points.append(["x": p3.x, "y": p3.y, "z": p3.z])
        }

        return points
    }

    private func fitMeshMarkersToPayloadLimit(
        _ markers: [[String: Any]],
        topic: String,
        maxPayloadBytes: Int
    ) -> [[String: Any]] {
        var fittedMarkers = markers

        while let byteCount = encodedPublishPayloadByteCount(topic: topic, msg: ["markers": fittedMarkers]),
              byteCount > maxPayloadBytes,
              !fittedMarkers.isEmpty {
            let lastIndex = fittedMarkers.count - 1
            guard let points = fittedMarkers[lastIndex]["points"] as? [[String: Float]],
                  points.count > 3 else {
                fittedMarkers.removeLast()
                continue
            }

            let estimatedCount = Int(Double(points.count) * Double(maxPayloadBytes) / Double(byteCount))
            let reducedCount = min(points.count - 3, estimatedCount)
            let alignedCount = max(0, reducedCount - (reducedCount % 3))

            if alignedCount >= 3 {
                fittedMarkers[lastIndex]["points"] = Array(points.prefix(alignedCount))
            } else {
                fittedMarkers.removeLast()
            }
        }

        return fittedMarkers
    }

    private func encodedPublishPayloadByteCount(topic: String, msg: [String: Any]) -> Int? {
        let payload: [String: Any] = [
            "op": "publish",
            "topic": topic,
            "msg": msg
        ]
        return try? JSONSerialization.data(withJSONObject: payload, options: []).count
    }

    private func recordStreamPayloadMetric(
        stream: MappingSensorStream,
        originalBytes: Int,
        encodedBytes: Int,
        compression: String
    ) {
        streamPayloadMetrics.record(
            stream: stream,
            originalBytes: originalBytes,
            encodedBytes: encodedBytes,
            compression: compression
        )
    }

    func publishSatelliteTile(_ tile: GeoTilePayload, timestamp: TimeInterval) {
        guard isConnected, topicRegistry.isStreamEnabled(.satelliteImagery) else { return }

        let msg: [String: Any] = [
            "header": createHeader(frameId: "earth", timestamp: timestamp),
            "format": tile.provider.format,
            "data": tile.data.base64EncodedString()
        ]

        send(op: "publish", topic: topicRegistry.topic(.satelliteImage), msg: msg)
    }

    func publishGeoTileInfo(_ tile: GeoTilePayload, timestamp: TimeInterval) {
        guard isConnected, topicRegistry.isStreamEnabled(.satelliteImagery) else { return }

        let msg = createGeoTileInfoMessage(tile, timestamp: timestamp)
        send(op: "publish", topic: topicRegistry.topic(.satelliteTileInfo), msg: msg)
    }

    func publishDEMTile(_ tile: GeoTilePayload, timestamp: TimeInterval) {
        guard isConnected, topicRegistry.isStreamEnabled(.dem) else { return }

        let msg = createGeoRasterTileMessage(tile, timestamp: timestamp)
        send(op: "publish", topic: topicRegistry.topic(.demTile), msg: msg)
    }

    func publishNavSatFix(_ location: CLLocation, timestamp: TimeInterval) {
        guard isConnected, topicRegistry.isStreamEnabled(.gps) else { return }

        let latitude = finiteROSNumber(location.coordinate.latitude, fallback: 0)
        let longitude = finiteROSNumber(location.coordinate.longitude, fallback: 0)
        let altitude = finiteROSNumber(location.altitude, fallback: 0)
        let horizontalAccuracy = finiteROSNumber(location.horizontalAccuracy)
        let verticalAccuracy = finiteROSNumber(location.verticalAccuracy)
        let hasCoordinate = location.coordinate.latitude.isFinite && location.coordinate.longitude.isFinite
        let hasHorizontalAccuracy = horizontalAccuracy >= 0
        let hasVerticalAccuracy = verticalAccuracy >= 0
        let hasFix = hasCoordinate && hasHorizontalAccuracy
        let horizontalVariance = hasHorizontalAccuracy ? horizontalAccuracy * horizontalAccuracy : 0
        let verticalVariance = hasVerticalAccuracy ? verticalAccuracy * verticalAccuracy : horizontalVariance
        let covarianceType: Int

        if hasHorizontalAccuracy && hasVerticalAccuracy {
            covarianceType = 2
        } else if hasHorizontalAccuracy {
            covarianceType = 1
        } else {
            covarianceType = 0
        }

        let msg: [String: Any] = [
            "header": createHeader(frameId: "earth", date: location.timestamp),
            "status": [
                "status": hasFix ? 0 : -1,
                "service": hasFix ? 1 : 0
            ],
            "latitude": latitude,
            "longitude": longitude,
            "altitude": altitude,
            "position_covariance": [
                horizontalVariance, 0, 0,
                0, horizontalVariance, 0,
                0, 0, verticalVariance
            ],
            "position_covariance_type": covarianceType
        ]

        send(op: "publish", topic: topicRegistry.topic(.gpsFix), msg: msg)
    }

    func publishGPSMetadata(_ location: CLLocation) {
        guard isConnected, topicRegistry.isStreamEnabled(.gps) else { return }

        let sourceInformation = location.sourceInformation
        let hasHorizontalAccuracy = location.horizontalAccuracy.isFinite && location.horizontalAccuracy >= 0
        let hasVerticalAccuracy = location.verticalAccuracy.isFinite && location.verticalAccuracy >= 0
        let hasSpeed = location.speed.isFinite && location.speed >= 0
        let hasSpeedAccuracy = location.speedAccuracy.isFinite && location.speedAccuracy >= 0
        let hasCourse = location.course.isFinite && location.course >= 0
        let hasCourseAccuracy = location.courseAccuracy.isFinite && location.courseAccuracy >= 0
        let validity: [String: Any] = [
            "coordinate": location.coordinate.latitude.isFinite && location.coordinate.longitude.isFinite,
            "horizontal_accuracy": hasHorizontalAccuracy,
            "vertical_accuracy": hasVerticalAccuracy,
            "altitude": location.altitude.isFinite,
            "speed": hasSpeed,
            "speed_accuracy": hasSpeedAccuracy,
            "course": hasCourse,
            "course_accuracy": hasCourseAccuracy
        ]
        let source: [String: Any] = [
            "simulated_by_software": sourceInformation?.isSimulatedBySoftware ?? false,
            "produced_by_accessory": sourceInformation?.isProducedByAccessory ?? false
        ]

        var msg: [String: Any] = [
            "header": createHeader(frameId: "earth", date: location.timestamp),
            "service": "core_location",
            "latitude": finiteROSNumber(location.coordinate.latitude, fallback: 0),
            "longitude": finiteROSNumber(location.coordinate.longitude, fallback: 0),
            "altitude": finiteROSNumber(location.altitude),
            "ellipsoidal_altitude": finiteROSNumber(location.ellipsoidalAltitude),
            "horizontal_accuracy": finiteROSNumber(location.horizontalAccuracy),
            "vertical_accuracy": finiteROSNumber(location.verticalAccuracy),
            "speed": finiteROSNumber(location.speed),
            "speed_accuracy": finiteROSNumber(location.speedAccuracy),
            "course": finiteROSNumber(location.course),
            "course_accuracy": finiteROSNumber(location.courseAccuracy),
            "timestamp": ISO8601DateFormatter().string(from: location.timestamp),
            "unix_time": location.timestamp.timeIntervalSince1970,
            "age_seconds": abs(location.timestamp.timeIntervalSinceNow),
            "validity": rosJSONString(validity),
            "source": rosJSONString(source)
        ]

        if let georeference = MapGeoreferencer.shared.snapshot(for: location) {
            msg["georeference"] = rosJSONString(georeference.rosMessage)
        } else {
            msg["georeference"] = rosJSONString(MapGeoreferencer.shared.unavailableMessage)
        }

        send(op: "publish", topic: topicRegistry.topic(.gpsMetadata), msg: msg)
    }

    func publishIndoorLocalization(_ sample: IndoorLocalizationSample, timestamp: TimeInterval) {
        guard isConnected, topicRegistry.isStreamEnabled(.indoorLocalization) else { return }

        let location = sample.location
        let heading = sample.heading
        let sourceInformation = location.sourceInformation
        let source: [String: Any] = [
            "simulated_by_software": sourceInformation?.isSimulatedBySoftware ?? false,
            "produced_by_accessory": sourceInformation?.isProducedByAccessory ?? false,
            "beacon_ranging_configured": false
        ]
        let registrationQuality: [String: Any] = [
            "indoor": finiteROSNumber(sample.indoorRegistrationQuality, fallback: 0),
            "indoor_label": sample.indoorQualityLabel,
            "global": finiteROSNumber(sample.globalRegistrationQuality, fallback: 0),
            "global_label": sample.globalQualityLabel
        ]

        let msg: [String: Any] = [
            "header": createHeader(frameId: "earth", timestamp: timestamp),
            "service": "core_location",
            "latitude": finiteROSNumber(location.coordinate.latitude, fallback: 0),
            "longitude": finiteROSNumber(location.coordinate.longitude, fallback: 0),
            "altitude": finiteROSNumber(location.altitude),
            "ellipsoidal_altitude": finiteROSNumber(location.ellipsoidalAltitude),
            "horizontal_accuracy": finiteROSNumber(location.horizontalAccuracy),
            "vertical_accuracy": finiteROSNumber(location.verticalAccuracy),
            "speed": finiteROSNumber(location.speed),
            "speed_accuracy": finiteROSNumber(location.speedAccuracy),
            "course": finiteROSNumber(location.course),
            "course_accuracy": finiteROSNumber(location.courseAccuracy),
            "floor_available": location.floor != nil,
            "floor_level": location.floor?.level ?? 0,
            "heading_available": heading != nil,
            "true_heading": finiteROSNumber(heading?.trueHeading ?? -1),
            "magnetic_heading": finiteROSNumber(heading?.magneticHeading ?? -1),
            "heading_accuracy": finiteROSNumber(heading?.headingAccuracy ?? -1),
            "source": rosJSONString(source),
            "registration_quality": rosJSONString(registrationQuality),
            "timestamp": ISO8601DateFormatter().string(from: sample.timestamp)
        ]

        send(op: "publish", topic: topicRegistry.topic(.indoorLocalization), msg: msg)
    }

    func publishRadioObservation(_ observation: RadioObservationMessage) {
        guard isConnected, topicRegistry.isStreamEnabled(.radio) else { return }

        var msg = observation.fields
        msg["header"] = createHeader(frameId: observation.frameID, date: observation.timestamp)

        send(op: "publish", topic: topicRegistry.topic(.radio), msg: msg)
    }

    func makeGeoTileInfoMessage(tile: GeoTilePayload, header: [String: Any]) -> [String: Any] {
        let pixel = tile.deviceLocation.pixel
        var msg: [String: Any] = [
            "header": header,
            "provider": tile.provider.name,
            "layer": tile.provider.layer,
            "kind": tile.provider.kind.rawValue,
            "crs": tile.provider.crs,
            "zoom": tile.coordinate.z,
            "tile_x": tile.coordinate.x,
            "tile_y": tile.coordinate.y,
            "bounds": rosJSONString(tile.bounds.rosMessage),
            "device_location": rosJSONString(tile.deviceLocation.rosMessage),
            "device_pixel_x": pixel.x,
            "device_pixel_y": pixel.y,
            "tile_width": pixel.width,
            "tile_height": pixel.height,
            "pixel_origin": pixel.origin,
            "pixel_units": pixel.units,
            "format": tile.provider.format,
            "mime_type": tile.provider.mimeType,
            "source_url": tile.sourceURL.absoluteString,
            "attribution": tile.provider.attribution,
            "license": tile.provider.license,
            "source_policy": rosJSONString(tile.provider.sourcePolicy.rosMessage),
            "is_cached": tile.isCached
        ]

        if let time = tile.time {
            msg["time"] = time
        }

        return msg
    }

    func makeGeoRasterTileMessage(tile: GeoTilePayload, header: [String: Any]) -> [String: Any] {
        var msg = makeGeoTileInfoMessage(tile: tile, header: header)
        msg["encoding"] = tile.provider.encoding
        msg["data"] = tile.data.base64EncodedString()
        return msg
    }

    private func createGeoTileInfoMessage(_ tile: GeoTilePayload, timestamp: TimeInterval) -> [String: Any] {
        makeGeoTileInfoMessage(
            tile: tile,
            header: createHeader(frameId: "earth", timestamp: timestamp)
        )
    }

    private func createGeoRasterTileMessage(_ tile: GeoTilePayload, timestamp: TimeInterval) -> [String: Any] {
        makeGeoRasterTileMessage(
            tile: tile,
            header: createHeader(frameId: "earth", timestamp: timestamp)
        )
    }

    func publishSessionMetadata(_ snapshot: MappingSessionSnapshot, timestamp: TimeInterval) {
        guard isConnected, topicRegistry.isStreamEnabled(.session) else { return }

        let queueStats = publishQueueStats
        let transportProfile = ROS2BridgeTransportProfile.current
        let currentWiFiTelemetryManager = CurrentWiFiTelemetryManager.shared
        let bleBeaconTelemetryManager = BLEBeaconTelemetryManager.shared
        let networkPathDiagnosticsManager = NetworkPathDiagnosticsManager.shared
        let recorderEndpointProbeManager = RecorderEndpointProbeManager.shared
        let localBagRecorder = LocalROS2BagRecorder.shared
        let optionalGeoProviderConfigurations = GeoTileProviderConfigurationStore.load()
        let advertisedTopics = topicRegistry.advertisedTopics().map { definition in
            [
                "id": definition.id.rawValue,
                "stream": definition.stream.rawValue,
                "topic": definition.topic,
                "message_type": definition.messageType,
                "default_rate_hz": definition.defaultRateHz.map { String($0) } ?? ""
            ]
        }
        let app: [String: Any] = [
            "version": Bundle.main.infoDictionary?["CFBundleShortVersionString"] as? String ?? "",
            "build": Bundle.main.infoDictionary?["CFBundleVersion"] as? String ?? ""
        ]
        let device: [String: Any] = [
            "name": UIDevice.current.name,
            "model": UIDevice.current.model,
            "system_name": UIDevice.current.systemName,
            "system_version": UIDevice.current.systemVersion,
            "vendor_id": UIDevice.current.identifierForVendor?.uuidString ?? ""
        ]

        let msg: [String: Any] = [
            "header": createHeader(frameId: "iphone_camera", timestamp: timestamp),
            "event": snapshot.event,
            "session_id": snapshot.sessionID?.uuidString ?? "",
            "state": snapshot.state,
            "recorder_url": snapshot.recorderURL,
            "bridge_transport": rosJSONString(transportProfile.rosMessage),
            "enabled_streams": snapshot.enabledStreams,
            "advertised_topics": rosJSONString(advertisedTopics, fallback: "[]"),
            "radio_channels": rosJSONString(RadioTelemetryCatalog.shared.rosMessage),
            "radio_platform_restrictions": rosJSONString(RadioTelemetryCatalog.shared.platformRestrictionsMessage),
            "radio_observation_schema": rosJSONString(RadioObservationMessageSchema.shared.rosMessage),
            "mesh_snapshot_schema": rosJSONString(MeshSnapshotMessageSchema.shared.rosMessage),
            "stream_payload_metrics": rosJSONString(streamPayloadMetrics.rosMessage),
            "optional_geo_provider_configurations": rosJSONString(optionalGeoProviderConfigurations.map(\.rosMessage), fallback: "[]"),
            "current_wifi_telemetry": rosJSONString(currentWiFiTelemetryManager.sessionMetadata),
            "ble_beacon_telemetry": rosJSONString(bleBeaconTelemetryManager.sessionMetadata),
            "network_path_diagnostics": rosJSONString(networkPathDiagnosticsManager.sessionMetadata),
            "recorder_endpoint_probe": rosJSONString(recorderEndpointProbeManager.sessionMetadata),
            "local_bag_storage": rosJSONString(localBagRecorder.sessionMetadata),
            "started_at": iso8601String(snapshot.startedAt),
            "ended_at": iso8601String(snapshot.endedAt),
            "app": rosJSONString(app),
            "device": rosJSONString(device),
            "publish_queue": [
                "capacity": queueStats.capacity,
                "depth": queueStats.depth,
                "in_flight": queueStats.inFlight,
                "sent_messages": queueStats.sentMessages,
                "dropped_messages": queueStats.droppedMessages,
                "retried_messages": queueStats.retriedMessages,
                "failed_messages": queueStats.failedMessages,
                "last_error": queueStats.lastError ?? ""
            ],
            "last_error": snapshot.lastError ?? ""
        ]

        send(op: "publish", topic: topicRegistry.topic(.session), msg: msg)
    }

    private func iso8601String(_ date: Date?) -> String {
        guard let date else { return "" }
        return ISO8601DateFormatter().string(from: date)
    }

    private func publishDiagnostics() {
        guard isConnected, topicRegistry.isStreamEnabled(.diagnostics) else { return }

        let queueStats = publishQueueStats
        let localBufferStats = localSampleBufferStats
        let geoTilePublisher = GeoTilePublisher.shared
        let indoorLocalizationManager = IndoorLocalizationManager.shared
        let transportProfile = ROS2BridgeTransportProfile.current
        let currentWiFiTelemetryManager = CurrentWiFiTelemetryManager.shared
        let bleBeaconTelemetryManager = BLEBeaconTelemetryManager.shared
        let networkPathDiagnosticsManager = NetworkPathDiagnosticsManager.shared
        let recorderEndpointProbeManager = RecorderEndpointProbeManager.shared
        let localBagRecorder = LocalROS2BagRecorder.shared
        let optionalGeoProviderDiagnosticValues = GeoTileProviderConfigurationStore.diagnosticValues
        let payloadMetricSnapshots = streamPayloadMetrics.allSnapshots()
        let enabledStreams = MappingSensorStream.allCases
            .filter { topicRegistry.isStreamEnabled($0) }
            .map(\.rawValue)
            .sorted()
            .joined(separator: ",")
        var bridgeDiagnosticValues: [String: String] = [
            "connected": String(isConnected),
            "recorder_url": currentURL ?? "",
            "enabled_streams": enabledStreams
        ]
        transportProfile.diagnosticValues.forEach { key, value in
            bridgeDiagnosticValues[key] = value
        }

        let msg: [String: Any] = [
            "header": createHeader(frameId: "iphone_camera", timestamp: ProcessInfo.processInfo.systemUptime),
            "status": [
                diagnosticStatus(
                    name: "mapping/ros2_bridge",
                    level: isConnected ? 0 : 2,
                    message: isConnected ? "Connected to ROS2 recorder" : "Disconnected from ROS2 recorder",
                    values: bridgeDiagnosticValues
                ),
                diagnosticStatus(
                    name: "mapping/publish_queue",
                    level: diagnosticLevel(for: queueStats),
                    message: diagnosticMessage(for: queueStats),
                    values: [
                        "capacity": String(queueStats.capacity),
                        "depth": String(queueStats.depth),
                        "in_flight": String(queueStats.inFlight),
                        "sent_messages": String(queueStats.sentMessages),
                        "dropped_messages": String(queueStats.droppedMessages),
                        "retried_messages": String(queueStats.retriedMessages),
                        "failed_messages": String(queueStats.failedMessages),
                        "last_error": queueStats.lastError ?? ""
                    ]
                ),
                diagnosticStatus(
                    name: "mapping/local_sample_buffer",
                    level: localSampleBufferDiagnosticLevel(localBufferStats),
                    message: localSampleBufferDiagnosticMessage(localBufferStats),
                    values: [
                        "point_cloud_samples": String(localBufferStats.pointCloudSamples),
                        "mesh_samples": String(localBufferStats.meshSamples),
                        "total_bytes": String(localBufferStats.totalBytes),
                        "max_total_bytes": String(localBufferStats.maxTotalBytes),
                        "max_point_cloud_samples": String(localBufferStats.maxPointCloudSamples),
                        "max_mesh_samples": String(localBufferStats.maxMeshSamples),
                        "dropped_samples": String(localBufferStats.droppedSamples),
                        "replayed_samples": String(localBufferStats.replayedSamples),
                        "last_buffered_at": localBufferStats.lastBufferedAt.map { ISO8601DateFormatter().string(from: $0) } ?? ""
                    ]
                ),
                diagnosticStatus(
                    name: "mapping/local_bag_storage",
                    level: localBagRecorder.diagnosticLevel,
                    message: localBagRecorder.diagnosticMessage,
                    values: localBagRecorder.stats.diagnosticValues
                ),
                diagnosticStatus(
                    name: "mapping/stream_payload_metrics",
                    level: 0,
                    message: payloadMetricSnapshots.isEmpty ? "No payload metrics published yet" : "Stream payload metrics nominal",
                    values: streamPayloadDiagnosticValues(payloadMetricSnapshots)
                ),
                diagnosticStatus(
                    name: "mapping/geotiles",
                    level: geoTilePublisher.lastError == nil ? 0 : 1,
                    message: geoTilePublisher.lastError ?? "Satellite imagery and DEM tile publisher nominal",
                    values: [
                        "running": String(geoTilePublisher.isRunning),
                        "last_published_at": geoTilePublisher.lastPublishedAt.map { ISO8601DateFormatter().string(from: $0) } ?? "",
                        "publish_interval_seconds": String(Int(GeoTilePublisher.Configuration.default.publishInterval))
                    ]
                ),
                diagnosticStatus(
                    name: "mapping/geotile_optional_providers",
                    level: 0,
                    message: "Optional geospatial provider configuration slots available",
                    values: optionalGeoProviderDiagnosticValues
                ),
                diagnosticStatus(
                    name: "mapping/gps_quality",
                    level: gpsQualityDiagnosticLevel(indoorLocalizationManager),
                    message: gpsQualityDiagnosticMessage(indoorLocalizationManager),
                    values: gpsQualityDiagnosticValues(indoorLocalizationManager)
                ),
                diagnosticStatus(
                    name: "mapping/indoor_localization",
                    level: indoorLocalizationDiagnosticLevel(indoorLocalizationManager),
                    message: indoorLocalizationDiagnosticMessage(indoorLocalizationManager),
                    values: [
                        "running": String(indoorLocalizationManager.isRunning),
                        "indoor_registration_quality": String(indoorLocalizationManager.lastIndoorRegistrationQuality),
                        "global_registration_quality": String(indoorLocalizationManager.lastGlobalRegistrationQuality),
                        "last_published_at": indoorLocalizationManager.lastPublishedAt.map { ISO8601DateFormatter().string(from: $0) } ?? "",
                        "last_gps_fix_published_at": indoorLocalizationManager.lastGPSFixPublishedAt.map { ISO8601DateFormatter().string(from: $0) } ?? "",
                        "location_authorization": indoorLocalizationManager.authorizationStatusLabel,
                        "accuracy_authorization": indoorLocalizationManager.accuracyAuthorizationLabel,
                        "precise_location_authorized": String(indoorLocalizationManager.isPreciseLocationAuthorized),
                        "heading_available": String(indoorLocalizationManager.isHeadingAvailable),
                        "last_heading_accuracy": String(indoorLocalizationManager.lastHeadingAccuracy),
                        "last_location_age_seconds": indoorLocalizationManager.lastLocationAgeSeconds.map { String(format: "%.1f", $0) } ?? "",
                        "horizontal_accuracy": String(indoorLocalizationManager.lastHorizontalAccuracy),
                        "vertical_accuracy": String(indoorLocalizationManager.lastVerticalAccuracy),
                        "last_error": indoorLocalizationManager.lastError ?? ""
                    ]
                ),
                diagnosticStatus(
                    name: "mapping/current_wifi",
                    level: currentWiFiTelemetryManager.diagnosticLevel,
                    message: currentWiFiTelemetryManager.diagnosticMessage,
                    values: currentWiFiTelemetryManager.diagnosticValues
                ),
                diagnosticStatus(
                    name: "mapping/ble_beacons",
                    level: bleBeaconTelemetryManager.diagnosticLevel,
                    message: bleBeaconTelemetryManager.diagnosticMessage,
                    values: bleBeaconTelemetryManager.diagnosticValues
                ),
                diagnosticStatus(
                    name: "mapping/network_path",
                    level: networkPathDiagnosticsManager.diagnosticLevel,
                    message: networkPathDiagnosticsManager.diagnosticMessage,
                    values: networkPathDiagnosticsManager.diagnosticValues
                ),
                diagnosticStatus(
                    name: "mapping/recorder_probe",
                    level: recorderEndpointProbeManager.diagnosticLevel,
                    message: recorderEndpointProbeManager.diagnosticMessage,
                    values: recorderEndpointProbeManager.diagnosticValues
                )
            ]
        ]

        send(op: "publish", topic: topicRegistry.topic(.status), msg: msg)
    }

    private func diagnosticStatus(
        name: String,
        level: Int,
        message: String,
        values: [String: String]
    ) -> [String: Any] {
        [
            "level": level,
            "name": name,
            "message": message,
            "hardware_id": UIDevice.current.identifierForVendor?.uuidString ?? UIDevice.current.model,
            "values": values
                .sorted { $0.key < $1.key }
                .map { ["key": $0.key, "value": $0.value] }
        ]
    }

    private func diagnosticLevel(for stats: PublishQueueStats) -> Int {
        if stats.failedMessages > 0 { return 2 }
        if stats.droppedMessages > 0 || stats.depth > Int(Double(stats.capacity) * 0.8) { return 1 }
        return 0
    }

    private func diagnosticMessage(for stats: PublishQueueStats) -> String {
        if stats.failedMessages > 0 {
            return "Publish queue has failed messages"
        }
        if stats.droppedMessages > 0 {
            return "Publish queue has dropped messages"
        }
        if stats.depth > Int(Double(stats.capacity) * 0.8) {
            return "Publish queue depth is high"
        }
        return "Publish queue nominal"
    }

    private func localSampleBufferDiagnosticLevel(_ stats: LocalSampleBufferStats) -> Int {
        if stats.droppedSamples > 0 { return 1 }
        if stats.totalBytes > Int(Double(stats.maxTotalBytes) * 0.8) { return 1 }
        return 0
    }

    private func localSampleBufferDiagnosticMessage(_ stats: LocalSampleBufferStats) -> String {
        let sampleCount = stats.pointCloudSamples + stats.meshSamples
        if stats.droppedSamples > 0 {
            return "Local sample buffer has dropped offline samples"
        }
        if stats.totalBytes > Int(Double(stats.maxTotalBytes) * 0.8) {
            return "Local sample buffer is near capacity"
        }
        if sampleCount > 0 {
            return "Local sample buffer has pending offline samples"
        }
        if stats.replayedSamples > 0 {
            return "Local sample buffer has replayed offline samples"
        }
        return "Local sample buffer empty"
    }

    private func streamPayloadDiagnosticValues(_ snapshots: [StreamPayloadMetricSnapshot]) -> [String: String] {
        var values: [String: String] = [:]
        for snapshot in snapshots {
            let prefix = snapshot.streamID
            values["\(prefix)_message_count"] = String(snapshot.messageCount)
            values["\(prefix)_original_bytes_total"] = String(snapshot.originalBytesTotal)
            values["\(prefix)_encoded_bytes_total"] = String(snapshot.encodedBytesTotal)
            values["\(prefix)_max_encoded_bytes"] = String(snapshot.maxEncodedBytes)
            values["\(prefix)_last_encoded_bytes"] = String(snapshot.lastEncodedBytes)
            values["\(prefix)_last_compression"] = snapshot.lastCompression
            values["\(prefix)_compression_ratio"] = String(format: "%.4f", snapshot.compressionRatio)
            values["\(prefix)_last_recorded_at"] = snapshot.lastRecordedAt.map { ISO8601DateFormatter().string(from: $0) } ?? ""
        }
        return values
    }

    private func gpsQualityDiagnosticLevel(_ manager: IndoorLocalizationManager) -> Int {
        if manager.authorizationStatusLabel == "denied" || manager.authorizationStatusLabel == "restricted" {
            return 2
        }
        if !manager.isRunning { return 1 }
        if manager.authorizationStatusLabel == "not_determined" { return 1 }
        if !manager.isPreciseLocationAuthorized { return 1 }
        if manager.lastLocationAgeSeconds.map({ $0 > 10 }) ?? true { return 1 }
        if manager.lastHorizontalAccuracy < 0 || manager.lastHorizontalAccuracy > 25 { return 1 }
        if manager.isHeadingAvailable {
            if manager.lastHeadingAccuracy < 0 || manager.lastHeadingAccuracy > 45 { return 1 }
            if manager.lastHeadingAgeSeconds.map({ $0 > 10 }) ?? true { return 1 }
        } else {
            return 1
        }
        return 0
    }

    private func gpsQualityDiagnosticMessage(_ manager: IndoorLocalizationManager) -> String {
        if manager.authorizationStatusLabel == "denied" || manager.authorizationStatusLabel == "restricted" {
            return "Location permission is denied or restricted"
        }
        if !manager.isRunning {
            return "GPS quality monitor is not running"
        }
        if manager.authorizationStatusLabel == "not_determined" {
            return "Location permission has not been granted yet"
        }
        if !manager.isPreciseLocationAuthorized {
            return "Precise location is not authorized"
        }
        if manager.lastLocationAgeSeconds.map({ $0 > 10 }) ?? true {
            return "GPS fix is stale or unavailable"
        }
        if manager.lastHorizontalAccuracy < 0 {
            return "GPS horizontal accuracy is unavailable"
        }
        if manager.lastHorizontalAccuracy > 25 {
            return "GPS horizontal accuracy is poor"
        }
        if !manager.isHeadingAvailable {
            return "Heading updates are unavailable"
        }
        if manager.lastHeadingAccuracy < 0 {
            return "Heading accuracy is unavailable"
        }
        if manager.lastHeadingAccuracy > 45 {
            return "Heading confidence is poor"
        }
        if manager.lastHeadingAgeSeconds.map({ $0 > 10 }) ?? true {
            return "Heading fix is stale or unavailable"
        }
        return "GPS quality nominal"
    }

    private func gpsQualityDiagnosticValues(_ manager: IndoorLocalizationManager) -> [String: String] {
        [
            "running": String(manager.isRunning),
            "location_authorization": manager.authorizationStatusLabel,
            "accuracy_authorization": manager.accuracyAuthorizationLabel,
            "precise_location_authorized": String(manager.isPreciseLocationAuthorized),
            "last_gps_fix_published_at": manager.lastGPSFixPublishedAt.map { ISO8601DateFormatter().string(from: $0) } ?? "",
            "last_location_age_seconds": manager.lastLocationAgeSeconds.map { String(format: "%.1f", $0) } ?? "",
            "horizontal_accuracy": String(manager.lastHorizontalAccuracy),
            "vertical_accuracy": String(manager.lastVerticalAccuracy),
            "heading_available": String(manager.isHeadingAvailable),
            "last_heading_accuracy": String(manager.lastHeadingAccuracy),
            "last_heading_age_seconds": manager.lastHeadingAgeSeconds.map { String(format: "%.1f", $0) } ?? "",
            "last_error": manager.lastError ?? "",
            "stale_fix_threshold_seconds": "10",
            "poor_horizontal_accuracy_threshold_meters": "25",
            "poor_heading_accuracy_threshold_degrees": "45"
        ]
    }

    private func odometryTwist(
        position: SIMD3<Float>,
        orientation: simd_quatf,
        timestamp: TimeInterval
    ) -> (linear: SIMD3<Float>, angular: SIMD3<Float>) {
        guard let lastOdometrySample else {
            return (.zero, .zero)
        }

        let deltaTime = timestamp - lastOdometrySample.timestamp
        guard deltaTime.isFinite, deltaTime > 0.000_1 else {
            return (.zero, .zero)
        }

        let deltaTimeFloat = Float(deltaTime)
        let linearVelocityWorld = (position - lastOdometrySample.position) / deltaTimeFloat
        let deltaRotation = orientation * simd_inverse(lastOdometrySample.orientation)
        let clampedW = min(max(Double(deltaRotation.vector.w), -1), 1)
        var angle = 2 * acos(clampedW)
        if angle > Double.pi {
            angle -= 2 * Double.pi
        }

        let sinHalfAngle = sqrt(max(0, 1 - clampedW * clampedW))
        let axis: SIMD3<Float>
        if sinHalfAngle > 0.000_001 {
            axis = SIMD3<Float>(
                deltaRotation.vector.x,
                deltaRotation.vector.y,
                deltaRotation.vector.z
            ) / Float(sinHalfAngle)
        } else {
            axis = .zero
        }

        let angularVelocityWorld = axis * Float(angle / deltaTime)
        let worldToCamera = simd_inverse(orientation)

        return (
            linear: simd_act(worldToCamera, linearVelocityWorld),
            angular: simd_act(worldToCamera, angularVelocityWorld)
        )
    }

    private func odometryPoseCovariance() -> [Double] {
        covariance(diagonal: [0.0025, 0.0025, 0.0025, 0.0004, 0.0004, 0.0004])
    }

    private func odometryTwistCovariance() -> [Double] {
        covariance(diagonal: [0.04, 0.04, 0.04, 0.01, 0.01, 0.01])
    }

    private func covariance(diagonal: [Double]) -> [Double] {
        var values = Array(repeating: 0.0, count: 36)
        for index in 0..<min(diagonal.count, 6) {
            values[index * 6 + index] = diagonal[index]
        }
        return values
    }

    private func indoorLocalizationDiagnosticLevel(_ manager: IndoorLocalizationManager) -> Int {
        if manager.lastError != nil { return 1 }
        if !manager.isRunning { return 1 }
        if !manager.isPreciseLocationAuthorized { return 1 }
        if manager.lastLocationAgeSeconds.map({ $0 > 10 }) ?? false { return 1 }
        if manager.lastIndoorRegistrationQuality < 0.25 || manager.lastGlobalRegistrationQuality < 0.25 { return 1 }
        return 0
    }

    private func indoorLocalizationDiagnosticMessage(_ manager: IndoorLocalizationManager) -> String {
        if let lastError = manager.lastError {
            return lastError
        }
        if !manager.isRunning {
            return "Indoor localization is not running"
        }
        if !manager.isPreciseLocationAuthorized {
            return "Precise location is not authorized"
        }
        if manager.lastLocationAgeSeconds.map({ $0 > 10 }) ?? false {
            return "Location fix is stale"
        }
        if manager.lastIndoorRegistrationQuality < 0.25 || manager.lastGlobalRegistrationQuality < 0.25 {
            return "Indoor or global registration quality is low"
        }
        return "Indoor localization nominal"
    }
    
    private func startIMU() {
        if motionManager.isDeviceMotionAvailable {
            let imuTopic = topicRegistry.topic(.imu)
            motionManager.deviceMotionUpdateInterval = 0.01 // 100Hz for high-fidelity ROS2 sensor fusion
            motionQueue.name = "com.mapeverything.imuQueue"
            motionQueue.qualityOfService = .userInitiated
            motionManager.startDeviceMotionUpdates(to: motionQueue) { [weak self] data, error in
                guard let self = self, let data = data, self.isConnected else { return }
                
                let msg: [String: Any] = [
                    "header": self.createHeader(frameId: "iphone_camera", timestamp: data.timestamp),
                    "orientation": [
                        "x": data.attitude.quaternion.x,
                        "y": data.attitude.quaternion.y,
                        "z": data.attitude.quaternion.z,
                        "w": data.attitude.quaternion.w
                    ],
                    "angular_velocity": [
                        "x": data.rotationRate.x,
                        "y": data.rotationRate.y,
                        "z": data.rotationRate.z
                    ],
                    "linear_acceleration": [
                        "x": (data.userAcceleration.x + data.gravity.x) * 9.81, // ROS expects m/s^2 including gravity
                        "y": (data.userAcceleration.y + data.gravity.y) * 9.81,
                        "z": (data.userAcceleration.z + data.gravity.z) * 9.81
                    ]
                ]
                self.send(op: "publish", topic: imuTopic, msg: msg)
            }
        }
    }
}
