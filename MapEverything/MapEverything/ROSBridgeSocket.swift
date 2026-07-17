//
//  ROSBridgeSocket.swift
//  MapEverything
//

import Foundation
import CryptoKit

/// Seam over URLSessionWebSocketTask so the bridge connection lifecycle can be
/// unit tested with a mock socket.
protocol ROSBridgeSocket: AnyObject {
    func resume()
    func cancel(with closeCode: URLSessionWebSocketTask.CloseCode, reason: Data?)
    func send(_ message: URLSessionWebSocketTask.Message, completionHandler: @escaping @Sendable (Error?) -> Void)
    func receive(completionHandler: @escaping @Sendable (Result<URLSessionWebSocketTask.Message, Error>) -> Void)
    func sendPing(pongReceiveHandler: @escaping @Sendable (Error?) -> Void)
}

extension URLSessionWebSocketTask: ROSBridgeSocket {}

typealias ROSBridgeSocketFactory = (URLRequest) -> ROSBridgeSocket

/// Optional trust override for `wss://` recorders with self-signed
/// certificates: the connection is accepted only when the leaf certificate's
/// SHA-256 fingerprint matches the user-configured value. With no fingerprint
/// configured, default system trust evaluation applies unchanged.
final class RecorderCertificatePinningDelegate: NSObject, URLSessionDelegate {
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

        if let chain = SecTrustCopyCertificateChain(trust) as? [SecCertificate],
           let leaf = chain.first,
           Self.fingerprint(of: SecCertificateCopyData(leaf) as Data) == expected {
            completionHandler(.useCredential, URLCredential(trust: trust))
            return
        }

        completionHandler(.performDefaultHandling, nil)
    }

    static func fingerprint(of derData: Data) -> String {
        SHA256.hash(data: derData).map { String(format: "%02x", $0) }.joined()
    }

    static func normalizedFingerprint(_ value: String) -> String {
        value.lowercased().filter { $0.isHexDigit }
    }
}
