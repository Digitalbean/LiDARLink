import Foundation

/// Optional extra temporal depth smoothing (motion-gated, per-pixel EMA).
/// Applied on top of ARKit's own smoothed scene depth; best used when the camera
/// is fairly still.
public enum DepthTemporal {
    /// Blends `current` into `previous` per pixel: out = alpha*current + (1-alpha)*previous,
    /// only where both are valid and the change is within `maxDelta` (avoids
    /// smearing across occlusions and edges). Invalid pixels pass through.
    /// Returns true if any pixel was blended.
    public static func blend(current: inout [Float], previous: [Float], alpha: Float, maxDelta: Float) -> Bool {
        guard current.count == previous.count, alpha > 0, alpha < 1 else { return false }
        var blended = false
        for i in current.indices {
            let value = current[i]
            let prev = previous[i]
            if value.isFinite, value > 0, prev.isFinite, prev > 0, abs(value - prev) <= maxDelta {
                current[i] = value * alpha + prev * (1 - alpha)
                blended = true
            }
        }
        return blended
    }
}
