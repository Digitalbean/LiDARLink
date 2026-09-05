import Foundation

/// A pose-independent "what does this keyframe see" signature, computed
/// straight from its own depth image — no unprojection, no world pose.
///
/// Loop-closure candidates today are found by pose proximity: two keyframes
/// are tried only if the pose graph's *current* estimate puts them within a
/// few metres of each other. That's backwards for the case that matters most —
/// the more a scan has drifted, the less likely a genuine revisit is to still
/// look "nearby," so the correction that would fix the drift never gets
/// attempted. A descriptor comparison doesn't have that problem: it only asks
/// whether two views look like the same place, independent of where the graph
/// currently thinks they are.
///
/// The camera has a narrow, fixed FOV (not a spinning 360° LiDAR), so unlike
/// Scan Context there's no need to project into a rotation-invariant polar
/// grid — a plain downsampled depth grid in the camera's own image space
/// already requires two keyframes to be looking in roughly the same direction
/// to match, which is exactly the constraint we want (a revisit means facing
/// the same wall again, not just standing in the same spot).
public struct KeyframeDescriptor: Sendable, Equatable {
    public let cols: Int
    public let rows: Int
    /// Row-major mean metric depth per bin, in metres. `nil` = too few
    /// confident, finite depth samples fell in that bin to trust it.
    public let bins: [Float?]

    public static let defaultCols = 12
    public static let defaultRows = 9
    /// A bin needs at least this fraction of its pixels to be valid,
    /// confident depth before it's trusted.
    public static let minValidFraction: Float = 0.3

    public static func compute(depth: DepthPayload,
                               depthScale: Float,
                               confidence: [UInt8]?,
                               minConfidence: UInt8 = 1,
                               cols: Int = defaultCols,
                               rows: Int = defaultRows) -> KeyframeDescriptor {
        let width = depth.width, height = depth.height
        var bins = [Float?](repeating: nil, count: cols * rows)
        guard width > 0, height > 0, cols > 0, rows > 0 else {
            return KeyframeDescriptor(cols: cols, rows: rows, bins: bins)
        }
        let raw = depth.float16Array()
        let scale = depthScale == 0 ? 1 : depthScale
        var sums = [Float](repeating: 0, count: cols * rows)
        var counts = [Int](repeating: 0, count: cols * rows)
        var totals = [Int](repeating: 0, count: cols * rows)

        for y in 0..<height {
            let by = min(rows - 1, y * rows / height)
            for x in 0..<width {
                let bx = min(cols - 1, x * cols / width)
                let bin = by * cols + bx
                totals[bin] += 1
                let index = y * width + x
                if let confidence, index < confidence.count, confidence[index] < minConfidence { continue }
                let d = Float(raw[index]) * scale
                guard d.isFinite, d > 0 else { continue }
                sums[bin] += d
                counts[bin] += 1
            }
        }

        for bin in 0..<bins.count {
            guard totals[bin] > 0, Float(counts[bin]) >= Float(totals[bin]) * minValidFraction else { continue }
            bins[bin] = sums[bin] / Float(counts[bin])
        }
        return KeyframeDescriptor(cols: cols, rows: rows, bins: bins)
    }

    /// Mean absolute depth difference over bins valid in both descriptors, in
    /// metres — lower is more similar. `nil` when too few bins are shared to
    /// judge (e.g. one keyframe stares at a blank wall, the other at a corner).
    public static func distance(_ a: KeyframeDescriptor, _ b: KeyframeDescriptor) -> Float? {
        guard a.cols == b.cols, a.rows == b.rows else { return nil }
        var sum: Float = 0
        var shared = 0
        for i in 0..<a.bins.count {
            guard let av = a.bins[i], let bv = b.bins[i] else { continue }
            sum += abs(av - bv)
            shared += 1
        }
        guard shared >= Int(Float(a.bins.count) * minValidFraction) else { return nil }
        return sum / Float(shared)
    }
}
