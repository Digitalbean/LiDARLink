import Foundation

/// A single-slot "latest wins" queue used to apply backpressure: when the sender is
/// busy, new items replace the pending one and the obsolete item is counted as
/// dropped. Never grows unbounded. Thread-safe.
public final class LatestWinsQueue<Element>: @unchecked Sendable {
    private let lock = NSLock()
    private var pending: Element?
    public private(set) var droppedCount = 0
    public private(set) var acceptedCount = 0

    public init() {}

    /// Replaces the pending element. Returns true when a previous pending element
    /// was discarded (a dropped frame).
    @discardableResult
    public func submit(_ element: Element) -> Bool {
        lock.lock()
        defer { lock.unlock() }
        let dropped = pending != nil
        if dropped { droppedCount += 1 }
        pending = element
        acceptedCount += 1
        return dropped
    }

    public func take() -> Element? {
        lock.lock()
        defer { lock.unlock() }
        let element = pending
        pending = nil
        return element
    }

    public func clear() {
        lock.lock()
        defer { lock.unlock() }
        pending = nil
    }
}
