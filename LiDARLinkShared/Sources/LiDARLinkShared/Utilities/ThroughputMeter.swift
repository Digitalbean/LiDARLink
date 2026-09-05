import Foundation

/// Measures throughput (bytes/second) over a sliding 1-second window. Thread-safe.
public final class ThroughputMeter: @unchecked Sendable {
    private let lock = NSLock()
    private var windowStartMs: UInt64 = 0
    private var windowBytes = 0
    public private(set) var bytesPerSecond: Double = 0
    public private(set) var totalBytes = 0

    public init() {}

    public func record(bytes: Int, timestampMs: UInt64) {
        lock.lock()
        defer { lock.unlock() }
        totalBytes += bytes
        windowBytes += bytes
        guard windowStartMs != 0 else {
            windowStartMs = timestampMs
            return
        }
        let elapsed = Double(timestampMs &- windowStartMs) / 1000.0
        if elapsed >= 1.0 {
            bytesPerSecond = Double(windowBytes) / elapsed
            windowBytes = 0
            windowStartMs = timestampMs
        }
    }

    public func reset() {
        lock.lock()
        defer { lock.unlock() }
        windowStartMs = 0
        windowBytes = 0
        bytesPerSecond = 0
        totalBytes = 0
    }
}
