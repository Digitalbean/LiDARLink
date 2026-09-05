import Foundation

/// Rolling-window frames-per-second estimator. Thread-safe.
public final class FPSMeter: @unchecked Sendable {
    private let lock = NSLock()
    private var samples: [UInt64] = []
    private let windowSize: Int
    public private(set) var fps: Double = 0

    public init(windowSize: Int = 40) {
        self.windowSize = max(windowSize, 2)
    }

    public func record(timestampMs: UInt64) {
        lock.lock()
        defer { lock.unlock() }
        samples.append(timestampMs)
        if samples.count > windowSize {
            samples.removeFirst(samples.count - windowSize)
        }
        guard samples.count >= 2 else { return }
        let span = Double(samples[samples.count - 1] - samples[0]) / 1000.0
        guard span > 0 else { return }
        fps = Double(samples.count - 1) / span
    }

    public func reset() {
        lock.lock()
        defer { lock.unlock() }
        samples.removeAll()
        fps = 0
    }
}
