//
//  MetricKitDiagnostics.swift
//  MapEverything
//

import Foundation
import MetricKit

/// Subscribes to MetricKit and archives crash/hang diagnostic payloads and
/// daily metric payloads as JSON under Documents/Diagnostics so they can be
/// shared from the Advanced settings panel.
final class MetricKitDiagnostics: NSObject, MXMetricManagerSubscriber {
    static let shared = MetricKitDiagnostics()

    /// Number of files retained per payload kind; older files are deleted.
    nonisolated static let maxRetainedFilesPerKind = 20

    enum PayloadKind: String, CaseIterable {
        case diagnostic
        case metric
    }

    private let ioQueue = DispatchQueue(label: "MapEverything.MetricKitDiagnostics", qos: .utility)
    func start() {
        MXMetricManager.shared.add(self)
    }

    func didReceive(_ payloads: [MXDiagnosticPayload]) {
        persist(payloads.map { $0.jsonRepresentation() }, kind: .diagnostic)
    }

    func didReceive(_ payloads: [MXMetricPayload]) {
        persist(payloads.map { $0.jsonRepresentation() }, kind: .metric)
    }

    /// All archived payload files, newest first.
    func savedDiagnosticFileURLs() -> [URL] {
        let directoryURL = Self.diagnosticsDirectoryURL()
        return ioQueue.sync {
            Self.listPayloadFiles(in: directoryURL)
                .sorted { $0.lastPathComponent > $1.lastPathComponent }
        }
    }

    /// Returns the files that exceed the per-kind retention limit. Filenames
    /// are `<ISO8601 timestamp>-<kind>.json`, so a descending lexicographic
    /// sort orders each kind newest first. Files that do not match a known
    /// payload kind are never selected for deletion.
    nonisolated static func staleFileURLs(
        in urls: [URL],
        keepingMostRecent limit: Int = maxRetainedFilesPerKind
    ) -> [URL] {
        guard limit >= 0 else { return [] }

        var stale: [URL] = []
        for kind in PayloadKind.allCases {
            let suffix = "-\(kind.rawValue).json"
            let matching = urls
                .filter { $0.lastPathComponent.hasSuffix(suffix) }
                .sorted { $0.lastPathComponent > $1.lastPathComponent }
            stale.append(contentsOf: matching.dropFirst(limit))
        }
        return stale
    }

    // MARK: - Private

    nonisolated private static func diagnosticsDirectoryURL() -> URL? {
        FileManager.default
            .urls(for: .documentDirectory, in: .userDomainMask)
            .first?
            .appendingPathComponent("Diagnostics", isDirectory: true)
    }

    private func persist(_ jsonPayloads: [Data], kind: PayloadKind) {
        guard !jsonPayloads.isEmpty else { return }

        guard let directoryURL = Self.diagnosticsDirectoryURL() else { return }
        let kindRawValue = kind.rawValue

        ioQueue.async {
            let timestampFormatter = ISO8601DateFormatter()
            timestampFormatter.formatOptions = [.withInternetDateTime, .withFractionalSeconds]

            do {
                try FileManager.default.createDirectory(
                    at: directoryURL,
                    withIntermediateDirectories: true
                )
                for (index, json) in jsonPayloads.enumerated() {
                    let timestamp = timestampFormatter.string(from: Date())
                    let prefix = index == 0 ? timestamp : "\(timestamp)-\(index)"
                    let fileURL = directoryURL.appendingPathComponent(
                        "\(prefix)-\(kindRawValue).json"
                    )
                    try json.write(to: fileURL, options: .atomic)
                }
            } catch {
                print("MetricKitDiagnostics: failed to persist \(kindRawValue) payloads: \(error)")
            }

            Self.pruneStaleFiles(in: directoryURL)
        }
    }

    /// Must run on `ioQueue`.
    nonisolated private static func pruneStaleFiles(in directoryURL: URL) {
        for staleURL in staleFileURLs(in: listPayloadFiles(in: directoryURL)) {
            do {
                try FileManager.default.removeItem(at: staleURL)
            } catch {
                print("MetricKitDiagnostics: failed to prune \(staleURL.lastPathComponent): \(error)")
            }
        }
    }

    /// Must run on `ioQueue`.
    nonisolated private static func listPayloadFiles(in directoryURL: URL?) -> [URL] {
        guard let directoryURL,
              let contents = try? FileManager.default.contentsOfDirectory(
                  at: directoryURL,
                  includingPropertiesForKeys: nil,
                  options: [.skipsHiddenFiles]
              ) else {
            return []
        }
        return contents.filter { $0.pathExtension == "json" }
    }
}
