import Foundation

public enum Time {
    /// Monotonic milliseconds since boot (safe against wall-clock changes).
    public static func monotonicMilliseconds() -> UInt64 {
        UInt64(DispatchTime.now().uptimeNanoseconds / 1_000_000)
    }
}

/// Small stateful gate for work that must be both throttled and single-flight.
public struct RefreshThrottle: Sendable {
    public let intervalMs: UInt64
    public private(set) var lastStartMs: UInt64 = 0

    public init(intervalMs: UInt64) {
        self.intervalMs = intervalMs
    }

    public mutating func shouldStart(nowMs: UInt64, enabled: Bool, isInFlight: Bool) -> Bool {
        guard enabled, !isInFlight else { return false }
        guard lastStartMs == 0 || nowMs &- lastStartMs >= intervalMs else { return false }
        lastStartMs = nowMs
        return true
    }

    public mutating func reset() {
        lastStartMs = 0
    }
}
