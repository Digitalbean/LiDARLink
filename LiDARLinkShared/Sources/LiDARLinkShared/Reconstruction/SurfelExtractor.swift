import Foundation
import simd

/// Turns one posed depth frame into a small set of oriented surface patches.
///
/// Instead of a surfel per depth sample (noisy, tens of thousands per frame),
/// the depth image is cut into a regular grid of tiles; each tile that is
/// roughly planar becomes a single surfel fitted to its samples, and a tile
/// straddling a depth discontinuity is split once before being kept or dropped.
/// This is the cheap "superpixel" that dense-surfel mapping systems use — far
/// fewer surfels, each averaged over a region so sensor noise cancels.
public enum SurfelExtractor {

    public struct Parameters: Sendable {
        /// Base tile edge in depth pixels. Finer tiles give the progressive
        /// surfel refinement real sub-patch structure to lock onto.
        public var tilePixels: Int = 5
        /// A tile is planar if every sample is within this fraction of its mean
        /// depth of the fitted plane.
        public var planarThresholdFraction: Float = 0.01
        /// Split a non-planar tile down to this edge before giving up on it.
        public var minTilePixels: Int = 2
        /// Hard cap on a patch's radius (metres) regardless of range.
        public var maxRadiusMeters: Float = 0.05
        /// Need at least this fraction of a tile's pixels to be valid depth.
        public var minValidFraction: Float = 0.5
        public var minConfidence: UInt8 = 1
        public var minDepthMeters: Float = 0.15
        public var maxDepthMeters: Float = 5
        public init() {}
    }

    public static func extract(depth: [Float16],
                               width: Int, height: Int,
                               depthScale: Float,
                               intrinsics: CameraIntrinsics,
                               pose: simd_float4x4,
                               confidence: [UInt8]?,
                               keyframeID: Int32,
                               parameters: Parameters = Parameters(),
                               colorFor: ((Int, Int) -> SIMD3<UInt8>)? = nil) -> [Surfel] {
        guard width > 0, height > 0, depth.count >= width * height else { return [] }
        let k = intrinsics.scaled(to: width, height: height)
        let scale = depthScale == 0 ? 1 : depthScale
        let rot = Lie.rotation(pose)

        func cam(_ x: Int, _ y: Int) -> SIMD3<Float>? {
            let raw = Float(depth[y * width + x]) * scale
            guard raw.isFinite, raw >= parameters.minDepthMeters, raw <= parameters.maxDepthMeters else { return nil }
            if parameters.minConfidence > 0, (confidence?[y * width + x] ?? 2) < parameters.minConfidence { return nil }
            return PoseMath.cameraPoint(pixelX: Float(x), pixelY: Float(y), depth: raw, intrinsics: k)
        }

        var out: [Surfel] = []
        out.reserveCapacity((width / parameters.tilePixels + 1) * (height / parameters.tilePixels + 1))

        func processTile(_ x0: Int, _ y0: Int, _ size: Int) {
            let x1 = min(x0 + size, width), y1 = min(y0 + size, height)
            var pts: [SIMD3<Float>] = []
            pts.reserveCapacity(size * size)
            var y = y0
            while y < y1 { var x = x0
                while x < x1 { if let c = cam(x, y) { pts.append(c) }; x += 1 }
                y += 1
            }
            let total = (x1 - x0) * (y1 - y0)
            guard total > 0, Float(pts.count) >= Float(total) * parameters.minValidFraction, pts.count >= 6 else { return }

            var centroid = SIMD3<Float>.zero
            for p in pts { centroid += p }
            centroid /= Float(pts.count)

            // Plane normal from the covariance's smallest eigenvector (power
            // iteration on the inverse is overkill — use the cross-product of
            // the two dominant spread directions via a small covariance).
            var cxx: Float = 0, cxy: Float = 0, cxz: Float = 0, cyy: Float = 0, cyz: Float = 0, czz: Float = 0
            for p in pts {
                let d = p - centroid
                cxx += d.x * d.x; cxy += d.x * d.y; cxz += d.x * d.z
                cyy += d.y * d.y; cyz += d.y * d.z; czz += d.z * d.z
            }
            let normal = PlaneFit.smallestEigenvector(cxx: cxx, cxy: cxy, cxz: cxz, cyy: cyy, cyz: cyz, czz: czz)
            guard simd_length(normal) > 1e-5 else { return }
            var n = simd_normalize(normal)
            let toCam = -simd_normalize(centroid)
            if simd_dot(n, toCam) < 0 { n = -n }   // face the camera
            // Reject near-edge-on patches — their depth and normal are unreliable.
            guard simd_dot(n, toCam) > 0.2 else { return }

            let meanDepth = -centroid.z
            var maxResidual: Float = 0
            for p in pts { maxResidual = max(maxResidual, abs(simd_dot(p - centroid, n))) }

            if maxResidual > parameters.planarThresholdFraction * meanDepth {
                let half = size / 2
                if half >= parameters.minTilePixels {
                    processTile(x0, y0, half); processTile(x0 + half, y0, half)
                    processTile(x0, y0 + half, half); processTile(x0 + half, y0 + half, half)
                }
                return
            }

            // Radius: cover the tile's world extent so neighbours overlap.
            var extent: Float = 0
            for p in pts { extent = max(extent, simd_length((p - centroid) - simd_dot(p - centroid, n) * n)) }
            let radius = simd_clamp(extent * 0.62, 0.008, parameters.maxRadiusMeters)

            let world = pose * SIMD4<Float>(centroid, 1)
            let cxp = (x0 + x1) / 2, cyp = (y0 + y1) / 2
            let colObs: SIMD3<Float>
            if let colorFor {
                let cc = colorFor(cxp, cyp)
                colObs = SIMD3<Float>(Float(cc.x), Float(cc.y), Float(cc.z))
            } else {
                colObs = SIMD3<Float>(repeating: 170)
            }
            let conf = confidence?[min(cyp, height - 1) * width + min(cxp, width - 1)] ?? 2
            let w = (conf >= 2 ? 1.0 : 0.6) * Double(simd_clamp(4 / max(meanDepth * meanDepth, 1), 0.3, 1))
                  * Double(min(1, Float(pts.count) / 24))

            out.append(Surfel(position: SIMD3<Float>(world.x, world.y, world.z),
                              normal: simd_normalize(rot * n),
                              color: colObs,
                              radius: radius,
                              weight: Float(w),
                              owner: keyframeID,
                              lastSeen: keyframeID))
        }

        var ty = 0
        while ty < height { var tx = 0
            while tx < width { processTile(tx, ty, parameters.tilePixels); tx += parameters.tilePixels }
            ty += parameters.tilePixels
        }
        return out
    }

}
