//
//  UncheckedSendable.swift
//  MapEverything
//

/// Transfers a non-Sendable value across an isolation boundary when the
/// producer provably stops touching it (single-owner handoff), e.g. ARKit
/// pixel buffers handed to a processing task.
nonisolated struct UncheckedSendable<Value>: @unchecked Sendable {
    let value: Value

    init(_ value: Value) {
        self.value = value
    }
}
