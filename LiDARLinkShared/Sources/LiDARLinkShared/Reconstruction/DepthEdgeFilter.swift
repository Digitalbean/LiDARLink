import Foundation

/// Depth-edge / flying-pixel rejection for the unprojection loop.
///
/// At a depth discontinuity (a near object silhouetted against a far
/// background) the LiDAR / ARKit depth map returns interpolated "mixed"
/// pixels strung through mid-air between the two surfaces. Unprojecting them
/// produces a spray of points floating between the near and far geometry —
/// one of the most visible noise sources in the cloud.
///
/// This helper flags those pixels so `PoseMath.buildPointCloud` can skip
/// them. It is deliberately cheap (a single 8-neighbour pass plus an optional
/// one-pixel dilation) so it can run on every frame.
///
/// Detection rule for a valid pixel with depth `d`:
///  1. First difference — the largest absolute depth jump to a valid
///     neighbour must exceed `threshold(d) = baseMeters + slope * d`.
///     A flat surface (or a gentle gradient) never trips this.
///  2. Second difference — the *curvature* across at least one opposing
///     neighbour pair must also exceed `threshold(d)`. A true edge is a step
///     (one side near, the other far → large curvature); a wall seen at a
///     grazing angle is a ramp (consistent gradient → curvature ≈ 0) and is
///     therefore preserved.
/// A pixel is rejected only when *both* conditions hold. With `erode` on, the
/// one-pixel border around every detected edge is dropped too, since the
/// flying pixels sit right on the boundary.
public enum DepthEdgeFilter {

    public struct Parameters: Sendable, Equatable {
        /// Constant part of the allowed neighbour deviation, in meters.
        public var baseMeters: Float
        /// Depth-proportional part: the allowed deviation grows by this
        /// fraction of the pixel's depth (accounts for range-dependent noise
        /// and the widening metric footprint of a pixel).
        public var slope: Float
        /// Also drop the one-pixel border around every detected edge.
        public var erode: Bool

        public init(baseMeters: Float = 0.03, slope: Float = 0.05, erode: Bool = true) {
            self.baseMeters = baseMeters
            self.slope = slope
            self.erode = erode
        }

        public static let `default` = Parameters()
    }

    /// Returns a mask (row-major, one entry per pixel) where `true` marks a
    /// pixel to **reject**. Invalid pixels (non-finite or `<= 0`) are also
    /// marked `true`; they were never going to be unprojected anyway.
    ///
    /// - Parameter depth: metric depth in meters (already scaled).
    public static func rejectionMask(depth: [Float],
                                     width: Int,
                                     height: Int,
                                     parameters: Parameters = .default) -> [Bool] {
        let count = width * height
        guard width > 1, height > 1, depth.count >= count else {
            return [Bool](repeating: false, count: max(depth.count, 0))
        }

        func depthAt(_ x: Int, _ y: Int) -> Float? {
            guard x >= 0, x < width, y >= 0, y < height else { return nil }
            let v = depth[y * width + x]
            return (v.isFinite && v > 0) ? v : nil
        }

        let allNeighbours = [(-1, -1), (0, -1), (1, -1), (-1, 0),
                             (1, 0), (-1, 1), (0, 1), (1, 1)]
        // Opposing neighbour pairs: horizontal, vertical, both diagonals.
        let opposingPairs: [((Int, Int), (Int, Int))] = [
            ((-1, 0), (1, 0)),
            ((0, -1), (0, 1)),
            ((-1, -1), (1, 1)),
            ((-1, 1), (1, -1)),
        ]

        var reject = [Bool](repeating: false, count: count)
        var isEdge = [Bool](repeating: false, count: count)

        for y in 0..<height {
            for x in 0..<width {
                let index = y * width + x
                let v = depth[index]
                guard v.isFinite, v > 0 else { reject[index] = true; continue }

                let threshold = parameters.baseMeters + parameters.slope * v

                // 1. First difference: largest jump to any valid neighbour.
                var maxJump: Float = 0
                var validNeighbours = 0
                for (dx, dy) in allNeighbours {
                    guard let n = depthAt(x + dx, y + dy) else { continue }
                    validNeighbours += 1
                    maxJump = max(maxJump, abs(n - v))
                }
                guard validNeighbours >= 3 else { continue } // too sparse to judge
                guard maxJump > threshold else { continue }   // flat / gentle gradient

                // 2. Second difference: curvature across opposing pairs. A
                //    linear ramp (grazing plane) has curvature ~ 0; a step
                //    edge has curvature ~ the depth jump.
                var maxCurvature: Float = 0
                var pairs = 0
                for (a, b) in opposingPairs {
                    guard let da = depthAt(x + a.0, y + a.1),
                          let db = depthAt(x + b.0, y + b.1) else { continue }
                    pairs += 1
                    maxCurvature = max(maxCurvature, abs(2 * v - da - db) * 0.5)
                }
                guard pairs > 0 else { continue } // no opposing pair → keep (conservative)

                if maxCurvature > threshold {
                    reject[index] = true
                    isEdge[index] = true
                }
            }
        }

        guard parameters.erode else { return reject }

        // Dilate the detected-edge set by one pixel so the flying pixels that
        // sit right on the boundary (just inside either surface) go too. Only
        // real edges seed the dilation — not the invalid pixels above.
        var out = reject
        for y in 0..<height {
            for x in 0..<width {
                let index = y * width + x
                if reject[index] { continue }
                let v = depth[index]
                guard v.isFinite, v > 0 else { continue }
                for (dx, dy) in allNeighbours {
                    let nx = x + dx, ny = y + dy
                    guard nx >= 0, nx < width, ny >= 0, ny < height else { continue }
                    if isEdge[ny * width + nx] {
                        out[index] = true
                        break
                    }
                }
            }
        }
        return out
    }
}
