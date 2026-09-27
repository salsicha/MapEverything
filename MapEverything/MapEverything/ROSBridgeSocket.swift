//
//  ROSBridgeSocket.swift
//  MapEverything
//

import Foundation
import CryptoKit

/// Seam over URLSessionWebSocketTask so the bridge connection lifecycle can be
/// unit tested with a mock socket.
nonisolated protocol ROSBridgeSocket: AnyObject, Sendable {
    func resume()
    func cancel(with closeCode: URLSessionWebSocketTask.CloseCode, reason: Data?)
    func send(_ message: URLSessionWebSocketTask.Message, completionHandler: @escaping @Sendable (Error?) -> Void)
    func receive(completionHandler: @escaping @Sendable (Result<URLSessionWebSocketTask.Message, Error>) -> Void)
    func sendPing(pongReceiveHandler: @escaping @Sendable (Error?) -> Void)
}

extension URLSessionWebSocketTask: ROSBridgeSocket {}

typealias ROSBridgeSocketFactory = @Sendable (URLRequest) -> ROSBridgeSocket

extension RecorderCertificatePinningDelegate {
    /// Shared session for every recorder-bound WebSocket - bridge and
    /// endpoint probes alike - so the certificate pin is enforced
    /// uniformly. A probe built from URLSession.shared would bypass it.
    nonisolated static let pinnedSession = URLSession(
        configuration: .default,
        delegate: RecorderCertificatePinningDelegate(),
        delegateQueue: nil
    )
}

/// Optional trust override for `wss://` recorders with self-signed
/// certificates: the connection is accepted only when the leaf certificate's
/// SHA-256 fingerprint matches the user-configured value. With no fingerprint
/// configured, default system trust evaluation applies unchanged.
nonisolated final class RecorderCertificatePinningDelegate: NSObject, URLSessionDelegate {
    static let fingerprintDefaultsKey = "recorderCertificateSHA256Fingerprint"

    func urlSession(
        _ session: URLSession,
        didReceive challenge: URLAuthenticationChallenge,
        completionHandler: @escaping (URLSession.AuthChallengeDisposition, URLCredential?) -> Void
    ) {
        guard challenge.protectionSpace.authenticationMethod == NSURLAuthenticationMethodServerTrust,
              let trust = challenge.protectionSpace.serverTrust else {
            completionHandler(.performDefaultHandling, nil)
            return
        }

        let expected = Self.normalizedFingerprint(
            UserDefaults.standard.string(forKey: Self.fingerprintDefaultsKey) ?? ""
        )
        guard !expected.isEmpty else {
            completionHandler(.performDefaultHandling, nil)
            return
        }

        var leafFingerprint: String?
        if let chain = SecTrustCopyCertificateChain(trust) as? [SecCertificate],
           let leaf = chain.first {
            leafFingerprint = Self.fingerprint(of: SecCertificateCopyData(leaf) as Data)
        }
        switch Self.disposition(expected: expected, leafFingerprint: leafFingerprint) {
        case .useCredential:
            completionHandler(.useCredential, URLCredential(trust: trust))
        case .performDefaultHandling:
            completionHandler(.performDefaultHandling, nil)
        default:
            completionHandler(.cancelAuthenticationChallenge, nil)
        }
    }

    /// A configured pin is a commitment, not a hint: a presented chain that
    /// does not match (or cannot be read) must be REJECTED, never handed
    /// back to system trust - otherwise any publicly-trusted certificate
    /// could stand in for the pinned recorder on a DNS-named wss:// host.
    static func disposition(
        expected: String, leafFingerprint: String?
    ) -> URLSession.AuthChallengeDisposition {
        guard !expected.isEmpty else { return .performDefaultHandling }
        return leafFingerprint == expected ? .useCredential : .cancelAuthenticationChallenge
    }

    static func fingerprint(of derData: Data) -> String {
        SHA256.hash(data: derData).map { String(format: "%02x", $0) }.joined()
    }

    static func normalizedFingerprint(_ value: String) -> String {
        value.lowercased().filter { $0.isHexDigit }
    }
}
