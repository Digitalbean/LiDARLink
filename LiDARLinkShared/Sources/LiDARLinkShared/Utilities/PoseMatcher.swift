import Foundation
import simd

/// Matches a timestamp to the nearest camera pose sample.
/// ARKit's smoothed scene depth lags the current frame by a few frames; using the
/// pose that corresponds to the depth map's own timestamp removes motion ghosting.
public enum PoseMatcher {
    public static func nearestTransform(to timestamp: TimeInterval,
                                        in samples: [(timestamp: TimeInterval, transform: simd_float4x4)]) -> simd_float4x4? {
        guard let first = samples.first else { return nil }
        var best = first
        var bestDelta = abs(first.timestamp - timestamp)
        for sample in samples.dropFirst() {
            let delta = abs(sample.timestamp - timestamp)
            if delta < bestDelta {
                best = sample
                bestDelta = delta
            }
        }
        return best.transform
    }
}

/// Drops frames captured during the initial world-tracking calibration window,
/// when camera poses are still unstable (points anchored to empty space).
public struct WarmupGate {
    public let startMs: UInt64
    public let durationMs: UInt64

    public init(startMs: UInt64, durationMs: UInt64) {
        self.startMs = startMs
        self.durationMs = durationMs
    }

    public func shouldPass(nowMs: UInt64) -> Bool {
        nowMs >= startMs &+ durationMs
    }
}
