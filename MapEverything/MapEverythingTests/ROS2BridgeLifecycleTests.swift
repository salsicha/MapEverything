//
//  ROS2BridgeLifecycleTests.swift
//  MapEverythingTests
//

import Testing
import Foundation
@testable import MapEverything

final class MockROSBridgeSocket: ROSBridgeSocket, @unchecked Sendable {
    private let lock = NSLock()
    private var _resumed = false
    private var _cancelled = false
    private var _sentMessages: [URLSessionWebSocketTask.Message] = []
    private var _receiveHandler: ((Result<URLSessionWebSocketTask.Message, Error>) -> Void)?
    private var _pongHandler: ((Error?) -> Void)?

    var resumed: Bool { lock.withLock { _resumed } }
    var cancelled: Bool { lock.withLock { _cancelled } }
    var sentCount: Int { lock.withLock { _sentMessages.count } }
    var pingRequested: Bool { lock.withLock { _pongHandler != nil } }

    func resume() {
        lock.withLock { _resumed = true }
    }

    func cancel(with closeCode: URLSessionWebSocketTask.CloseCode, reason: Data?) {
        lock.withLock { _cancelled = true }
    }

    func send(_ message: URLSessionWebSocketTask.Message, completionHandler: @escaping @Sendable (Error?) -> Void) {
        lock.withLock { _sentMessages.append(message) }
        completionHandler(nil)
    }

    func receive(completionHandler: @escaping @Sendable (Result<URLSessionWebSocketTask.Message, Error>) -> Void) {
        lock.withLock { _receiveHandler = completionHandler }
    }

    func sendPing(pongReceiveHandler: @escaping @Sendable (Error?) -> Void) {
        lock.withLock { _pongHandler = pongReceiveHandler }
    }

    func failReceive(_ error: Error) {
        let handler = lock.withLock { () -> ((Result<URLSessionWebSocketTask.Message, Error>) -> Void)? in
            let handler = _receiveHandler
            _receiveHandler = nil
            return handler
        }
        handler?(.failure(error))
    }

    func succeedPong() {
        let handler = lock.withLock { _pongHandler }
        handler?(nil)
    }
}

private final class MockSocketFactory {
    private let lock = NSLock()
    private var _sockets: [MockROSBridgeSocket] = []

    var sockets: [MockROSBridgeSocket] { lock.withLock { _sockets } }
    var count: Int { lock.withLock { _sockets.count } }

    func make(_ request: URLRequest) -> ROSBridgeSocket {
        let socket = MockROSBridgeSocket()
        lock.withLock { _sockets.append(socket) }
        return socket
    }
}

@Suite(.serialized)
@MainActor
struct ROS2BridgeLifecycleTests {
    private static let testError = NSError(domain: "test", code: 57)

    private func makeClient(
        factory: MockSocketFactory,
        reconnectDelay: TimeInterval = 0.05
    ) -> ROS2BridgeClient {
        let temporary = FileManager.default.temporaryDirectory
            .appendingPathComponent("BridgeLifecycle-\(UUID().uuidString)", isDirectory: true)
        return ROS2BridgeClient(
            topicRegistry: ROS2TopicRegistry(),
            localBagRecorder: LocalROS2BagRecorder(fileManager: .default, baseDirectoryURL: temporary),
            socketFactory: { factory.make($0) },
            reconnectDelay: reconnectDelay
        )
    }

    // Suspends the main actor so queued main-queue blocks (the client's async
    // hops) can run; a nested RunLoop pump would let other tests interleave.
    private func settle(_ seconds: TimeInterval = 0.05) async {
        try? await Task.sleep(nanoseconds: UInt64(seconds * 1_000_000_000))
    }

    @Test("connect resumes the socket, requests a ping, and sends advertise ops")
    func testConnectResumesSocket() async throws {
        let factory = MockSocketFactory()
        let client = makeClient(factory: factory)

        client.connect(to: "ws://127.0.0.1:9090")
        await settle(0.15)

        try #require(factory.count == 1)
        #expect(factory.sockets[0].resumed)
        #expect(factory.sockets[0].pingRequested)
        #expect(factory.sockets[0].sentCount > 0)
        #expect(client.isConnected)
        client.disconnect()
        await settle()
    }

    @Test("A stale socket's failure does not tear down the new connection")
    func testStaleSocketFailureIgnored() async throws {
        let factory = MockSocketFactory()
        let client = makeClient(factory: factory)

        client.connect(to: "ws://127.0.0.1:9090")
        client.connect(to: "ws://127.0.0.1:9090")
        await settle()
        try #require(factory.count == 2)

        factory.sockets[0].failReceive(Self.testError)
        await settle(0.15)

        #expect(!factory.sockets[1].cancelled)
        #expect(client.isConnected)
        client.disconnect()
        await settle()
    }

    @Test("A deliberate disconnect does not auto-reconnect")
    func testDeliberateDisconnectDoesNotReconnect() async throws {
        let previous = UserDefaults.standard.object(forKey: "ros2Enabled")
        UserDefaults.standard.set(true, forKey: "ros2Enabled")
        defer { UserDefaults.standard.set(previous, forKey: "ros2Enabled") }

        let factory = MockSocketFactory()
        let client = makeClient(factory: factory)

        client.connect(to: "ws://127.0.0.1:9090")
        await settle()
        client.disconnect()
        await settle()
        try #require(factory.count == 1)
        factory.sockets[0].failReceive(Self.testError)
        await settle(0.3)

        #expect(factory.count == 1)
        #expect(!client.isConnected)
    }

    @Test("A transient receive failure schedules a reconnect")
    func testTransientFailureReconnects() async throws {
        let previous = UserDefaults.standard.object(forKey: "ros2Enabled")
        UserDefaults.standard.set(true, forKey: "ros2Enabled")
        defer { UserDefaults.standard.set(previous, forKey: "ros2Enabled") }

        let factory = MockSocketFactory()
        let client = makeClient(factory: factory)

        client.connect(to: "ws://127.0.0.1:9090")
        await settle()
        try #require(factory.count == 1)
        factory.sockets[0].failReceive(Self.testError)
        await settle(0.4)

        try #require(factory.count == 2)
        #expect(factory.sockets[1].resumed)
        client.disconnect()
        await settle()
    }

    @Test("A delayed disconnect does not kill a connection made inside its window")
    func testDelayedDisconnectSkipsNewConnection() async throws {
        let factory = MockSocketFactory()
        let client = makeClient(factory: factory)

        client.connect(to: "ws://127.0.0.1:9090")
        await settle()
        client.disconnect(after: 0.05)
        client.connect(to: "ws://127.0.0.1:9090")
        await settle(0.2)

        try #require(factory.count == 2)
        #expect(!factory.sockets[1].cancelled)
        #expect(client.isConnected)
        client.disconnect()
        await settle()
    }

    @Test("Tracking quality maps to covariance multipliers")
    func testCovarianceMultiplier() {
        #expect(ROS2BridgeClient.covarianceMultiplier(for: .normal) == 1)
        #expect(ROS2BridgeClient.covarianceMultiplier(for: .limited) == 25)
        #expect(ROS2BridgeClient.covarianceMultiplier(for: .notAvailable) == 10_000)
    }
}

struct CBOREncoderTests {
    private func hex(_ data: Data?) -> String {
        (data ?? Data()).map { String(format: "%02x", $0) }.joined()
    }

    @Test("Encodes RFC 8949 reference values")
    func testReferenceVectors() {
        #expect(hex(CBOREncoder.encode(["v": 10])) == "a161760a")
        #expect(hex(CBOREncoder.encode(["v": -10])) == "a1617629")
        #expect(hex(CBOREncoder.encode(["v": 1000])) == "a161761903e8")
        #expect(hex(CBOREncoder.encode(["v": "IETF"])) == "a161766449455446")
        #expect(hex(CBOREncoder.encode(["v": [1, 2, 3]])) == "a1617683010203")
        #expect(hex(CBOREncoder.encode(["v": 1.1])) == "a16176fb3ff199999999999a")
        #expect(hex(CBOREncoder.encode(["v": true])) == "a16176f5")
        #expect(hex(CBOREncoder.encode(["v": false])) == "a16176f4")
        #expect(hex(CBOREncoder.encode(["v": Data([1, 2, 3])])) == "a1617643010203")
    }

    @Test("Encodes nested maps with deterministic key order")
    func testNestedMap() {
        #expect(hex(CBOREncoder.encode(["a": 1, "b": [2, 3]])) == "a26161016162820203")
    }

    @Test("Rejects unsupported values instead of mis-encoding them")
    func testUnsupportedValue() {
        #expect(CBOREncoder.encode(["v": Date()]) == nil)
    }
}

struct RosbridgeAuthTests {
    @Test("MAC matches the rosauth SHA-512 concatenation")
    func testMACGoldenValue() {
        let mac = RosbridgeAuth.mac(
            secret: "s3cret",
            client: "MapEverything-iOS",
            destination: "192.168.1.50",
            rand: "abc123",
            t: 1_700_000_000,
            level: "user",
            end: 1_700_000_120
        )
        #expect(mac == "fea1924051f013d6d5822fb0442dd69e6cb00ac1987c2bfe5f02b61119641bef4ee7c05f4c95fe85f5dc86f1a2b3f374e3963bc18e42e4ea1a97f16cbc27f229")
    }

    @Test("Auth message carries the rosauth op fields")
    func testAuthMessageShape() {
        let message = RosbridgeAuth.authMessage(
            secret: "s",
            client: "c",
            destination: "d",
            rand: "r",
            t: 1,
            level: "user",
            end: 2
        )
        #expect(message["op"] as? String == "auth")
        #expect(message["client"] as? String == "c")
        #expect(message["dest"] as? String == "d")
        #expect((message["mac"] as? String)?.count == 128)
        #expect(JSONSerialization.isValidJSONObject(message))
    }
}

struct CertificatePinningTests {
    @Test("Fingerprints normalize to bare lowercase hex")
    func testNormalization() {
        #expect(RecorderCertificatePinningDelegate.normalizedFingerprint("AB:CD ef") == "abcdef")
        #expect(RecorderCertificatePinningDelegate.normalizedFingerprint("") == "")
    }

    @Test("Fingerprint of known data matches SHA-256")
    func testFingerprint() {
        let fingerprint = RecorderCertificatePinningDelegate.fingerprint(of: Data("hello".utf8))
        #expect(fingerprint == "2cf24dba5fb0a30e26e83b2ac5b9e29e1b161e5c1fa7425e73043362938b9824")
    }
}
