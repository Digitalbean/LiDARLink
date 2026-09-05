import Foundation

/// Edge-preserving bilateral smoothing for depth maps: each pixel is replaced by
/// a weighted average of its neighborhood, where weights combine a spatial
/// Gaussian with a depth-similarity (range) Gaussian. Noise is reduced while
/// sharp edges (object silhouettes) are preserved.
public enum DepthSmoother {
    public static func bilateralSmooth(depth: inout [Float],
                                       width: Int,
                                       height: Int,
                                       spatialSigma: Float = 1.0,
                                       rangeSigma: Float = 0.05) {
        guard width > 0, height > 0, depth.count >= width * height else { return }
        let radius = max(Int(ceil(spatialSigma * 2)), 1)
        let original = depth

        // Precompute spatial weights for each offset in the kernel.
        let offsets: [(dx: Int, dy: Int, weight: Float)] = {
            var result: [(Int, Int, Float)] = []
            result.reserveCapacity((2 * radius + 1) * (2 * radius + 1))
            for dy in -radius...radius {
                for dx in -radius...radius {
                    let w = exp(-Float(dx * dx + dy * dy) / (2 * spatialSigma * spatialSigma))
                    result.append((dx, dy, w))
                }
            }
            return result
        }()
        for y in 0..<height {
            for x in 0..<width {
                let index = y * width + x
                let center = original[index]
                guard center.isFinite, center > 0 else { continue }
                // LiDAR range noise grows with distance, so widen the range
                // sigma past ~2 m (and never below the baseline up close).
                let effectiveRangeSigma = rangeSigma * max(center / 2, 1)
                let rangeDenom = 2 * effectiveRangeSigma * effectiveRangeSigma
                var sum: Float = 0
                var weightSum: Float = 0
                for (dx, dy, spatial) in offsets {
                    let nx = x + dx
                    let ny = y + dy
                    guard nx >= 0, nx < width, ny >= 0, ny < height else { continue }
                    let neighbor = original[ny * width + nx]
                    guard neighbor.isFinite, neighbor > 0 else { continue }
                    let diff = neighbor - center
                    let range = exp(-(diff * diff) / rangeDenom)
                    let weight = spatial * range
                    sum += neighbor * weight
                    weightSum += weight
                }
                if weightSum > 0 {
                    depth[index] = sum / weightSum
                }
            }
        }
    }
}
