//
//  RosbridgeAuth.swift
//  MapEverything
//

import Foundation
import CryptoKit
import Security

/// Builds the rosbridge `auth` op per the rosauth protocol: the MAC is the
/// SHA-512 of secret + client + dest + rand + t + level + end.
enum RosbridgeAuth {
    static func authMessage(
        secret: String,
        client: String,
        destination: String,
        rand: String,
        t: Int,
        level: String,
        end: Int
    ) -> [String: Any] {
        [
            "op": "auth",
            "mac": mac(secret: secret, client: client, destination: destination, rand: rand, t: t, level: level, end: end),
            "client": client,
            "dest": destination,
            "rand": rand,
            "t": t,
            "level": level,
            "end": end
        ]
    }

    static func mac(
        secret: String,
        client: String,
        destination: String,
        rand: String,
        t: Int,
        level: String,
        end: Int
    ) -> String {
        let concatenated = secret + client + destination + rand + String(t) + level + String(end)
        return SHA512.hash(data: Data(concatenated.utf8))
            .map { String(format: "%02x", $0) }
            .joined()
    }
}

/// Keychain-backed storage for the optional rosauth shared secret; a secret
/// must never sit in UserDefaults where it would be captured by backups and
/// the session-metadata dictionaries.
enum RosbridgeAuthSecretStore {
    private static let service = "com.salsicha.MapEverything.rosauth"
    private static let account = "rosauth-secret"

    static func save(_ secret: String?) {
        let query: [String: Any] = [
            kSecClass as String: kSecClassGenericPassword,
            kSecAttrService as String: service,
            kSecAttrAccount as String: account
        ]
        SecItemDelete(query as CFDictionary)

        guard let secret, !secret.isEmpty else { return }

        var attributes = query
        attributes[kSecValueData as String] = Data(secret.utf8)
        attributes[kSecAttrAccessible as String] = kSecAttrAccessibleAfterFirstUnlockThisDeviceOnly
        SecItemAdd(attributes as CFDictionary, nil)
    }

    static func load() -> String? {
        let query: [String: Any] = [
            kSecClass as String: kSecClassGenericPassword,
            kSecAttrService as String: service,
            kSecAttrAccount as String: account,
            kSecReturnData as String: true,
            kSecMatchLimit as String: kSecMatchLimitOne
        ]
        var result: AnyObject?
        guard SecItemCopyMatching(query as CFDictionary, &result) == errSecSuccess,
              let data = result as? Data else {
            return nil
        }
        return String(data: data, encoding: .utf8)
    }
}
