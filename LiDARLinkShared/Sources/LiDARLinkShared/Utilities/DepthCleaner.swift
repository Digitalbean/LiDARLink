import Foundation

/// Pure depth-map cleanup operations applied on the phone before downsampling.
public enum DepthCleaner {
    /// Zeroes depth values beyond `maxDistanceMeters` (LiDAR depth degrades with range).
    public static func applyMaxDistance(depth: inout [Float], width: Int, height: Int, maxDistanceMeters: Float) {
        guard maxDistanceMeters > 0, depth.count >= width * height else { return }
        for i in 0..<(width * height) {
            let value = depth[i]
            if value.isFinite, value > maxDistanceMeters {
                depth[i] = 0
            }
        }
    }

    /// Removes "flying pixels": isolated depth spikes at surface boundaries where
    /// the sensor blended foreground/background. A pixel survives only if at least
    /// `minAgreeingNeighbors` of its orthogonal neighbors are within
    /// `maxNeighborDelta` meters of it.
    public static func removeFlyingPixels(depth: inout [Float],
                                          width: Int,
                                          height: Int,
                                          maxNeighborDelta: Float,
                                          minAgreeingNeighbors: Int = 2) {
        guard width > 0, height > 0, depth.count >= width * height else { return }
        let original = depth
        let neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        for y in 0..<height {
            for x in 0..<width {
                let index = y * width + x
                let value = original[index]
                guard value.isFinite, value > 0 else { continue }
                var agreeing = 0
                for (dx, dy) in neighbors {
                    let nx = x + dx
                    let ny = y + dy
                    guard nx >= 0, nx < width, ny >= 0, ny < height else { continue }
                    let neighbor = original[ny * width + nx]
                    if neighbor.isFinite, neighbor > 0, abs(neighbor - value) <= maxNeighborDelta {
                        agreeing += 1
                    }
                }
                if agreeing < minAgreeingNeighbors {
                    depth[index] = 0
                }
            }
        }
    }
}
