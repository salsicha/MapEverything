import Foundation

/// Captured by asynchronous frame work. Cancellation and the final enqueue of
/// its publications share a lock, so stop cannot race a write into the next bag.
/// Expensive image/point-cloud encoding runs outside this lock.
nonisolated final class ScanWorkSession: @unchecked Sendable {
    @TaskLocal static var current: ScanWorkSession?

    private let lock = NSLock()
    private var active = true

    var isActive: Bool { lock.withLock { active } }

    func cancel() {
        lock.withLock { active = false }
    }

    func withPublishing(_ operation: () -> Void) {
        Self.$current.withValue(self, operation: operation)
    }

    static func commitPublication(_ operation: () -> Void) {
        guard let session = current else {
            operation()
            return
        }
        session.lock.withLock {
            guard session.active, !Task.isCancelled else { return }
            operation()
        }
    }
}
