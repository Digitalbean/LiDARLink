import Foundation

public enum SequenceDisposition: Equatable, Sendable {
    case expected
    case duplicate
    case gap(skipped: UInt32)
    case reordered
}

/// Tracks incoming sequence numbers, classifying duplicates, gaps, and reordering.
/// Thread-safe.
public final class SequenceTracker: @unchecked Sendable {
    private let lock = NSLock()
    public private(set) var lastSequence: UInt32?
    public private(set) var receivedCount = 0
    public private(set) var duplicateCount = 0
    public private(set) var gapCount = 0
    public private(set) var reorderedCount = 0

    public init() {}

    @discardableResult
    public func record(_ sequence: UInt32) -> SequenceDisposition {
        lock.lock()
        defer { lock.unlock() }
        receivedCount += 1
        guard let last = lastSequence else {
            lastSequence = sequence
            return .expected
        }
        let delta = Int32(bitPattern: sequence &- last) // wraparound-aware
        if delta == 0 {
            duplicateCount += 1
            return .duplicate
        }
        if delta > 0 {
            if delta == 1 {
                lastSequence = sequence
                return .expected
            }
            gapCount += 1
            lastSequence = sequence
            return .gap(skipped: UInt32(bitPattern: delta - 1))
        }
        reorderedCount += 1
        return .reordered
    }
}
