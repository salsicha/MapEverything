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
    static let maxRetainedFilesPerKind = 20

    enum PayloadKind: String, CaseIterable {
        case diagnostic
        case metric
    }

    private let ioQueue = DispatchQueue(label: "MapEverything.MetricKitDiagnostics", qos: .utility)
    private let timestampFormatter: ISO8601DateFormatter = {
        let formatter = ISO8601DateFormatter()
        formatter.formatOptions = [.withInternetDateTime, .withFractionalSeconds]
        return formatter
    }()

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
        ioQueue.sync {
            listPayloadFiles().sorted { $0.lastPathComponent > $1.lastPathComponent }
        }
    }

    /// Returns the files that exceed the per-kind retention limit. Filenames
    /// are `<ISO8601 timestamp>-<kind>.json`, so a descending lexicographic
    /// sort orders each kind newest first. Files that do not match a known
    /// payload kind are never selected for deletion.
    static func staleFileURLs(
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

    private var diagnosticsDirectoryURL: URL? {
        FileManager.default
            .urls(for: .documentDirectory, in: .userDomainMask)
            .first?
            .appendingPathComponent("Diagnostics", isDirectory: true)
    }

    private func persist(_ jsonPayloads: [Data], kind: PayloadKind) {
        guard !jsonPayloads.isEmpty else { return }

        ioQueue.async { [self] in
            guard let directoryURL = diagnosticsDirectoryURL else { return }

            do {
                try FileManager.default.createDirectory(
                    at: directoryURL,
                    withIntermediateDirectories: true
                )
                for (index, json) in jsonPayloads.enumerated() {
                    let timestamp = timestampFormatter.string(from: Date())
                    let prefix = index == 0 ? timestamp : "\(timestamp)-\(index)"
                    let fileURL = directoryURL.appendingPathComponent(
                        "\(prefix)-\(kind.rawValue).json"
                    )
                    try json.write(to: fileURL, options: .atomic)
                }
            } catch {
                print("MetricKitDiagnostics: failed to persist \(kind.rawValue) payloads: \(error)")
            }

            pruneStaleFiles()
        }
    }

    /// Must run on `ioQueue`.
    private func pruneStaleFiles() {
        for staleURL in Self.staleFileURLs(in: listPayloadFiles()) {
            do {
                try FileManager.default.removeItem(at: staleURL)
            } catch {
                print("MetricKitDiagnostics: failed to prune \(staleURL.lastPathComponent): \(error)")
            }
        }
    }

    /// Must run on `ioQueue`.
    private func listPayloadFiles() -> [URL] {
        guard let directoryURL = diagnosticsDirectoryURL,
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
