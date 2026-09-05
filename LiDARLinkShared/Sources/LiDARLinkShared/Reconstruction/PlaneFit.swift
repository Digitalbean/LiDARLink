import Foundation
import simd

/// Robust plane fitting: iteratively-reweighted least squares. Points far off the
/// current plane estimate (a corner join, a bowed patch of drywall, sensor
/// noise) get down-weighted, so the fit converges to the *flat* part of the
/// surface and can be extrapolated cleanly toward a corner.
public enum PlaneFit {

    public struct Plane: Sendable {
        public var normal: SIMD3<Float>
        public var offset: Float           // dot(normal, p) == offset on the plane
        public var centroid: SIMD3<Float>  // robust centroid (a point on the plane)
        public var rms: Float              // inlier RMS residual, metres
        public var inlierWeight: Float     // summed weight of points within ~2σ
        public func signedDistance(_ p: SIMD3<Float>) -> Float { simd_dot(normal, p) - offset }
        /// Re-derive the offset for a substituted normal (e.g. after gravity snap).
        public func offset(for n: SIMD3<Float>) -> Float { simd_dot(n, centroid) }
    }

    /// - Parameters:
    ///   - points: `(position, weight)` samples.
    ///   - iterations: IRLS passes.
    ///   - scaleMeters: residual scale σ for the robust weight `1/(1+(r/σ)²)`.
    public static func fit(_ points: [(position: SIMD3<Float>, weight: Float)],
                           iterations: Int = 3,
                           scaleMeters: Float = 0.02) -> Plane? {
        guard points.count >= 3 else { return nil }
        let sigma = max(scaleMeters, 1e-3)

        var w = points.map { max($0.weight, 1e-4) }
        var plane = leastSquares(points, weights: w)
            ?? Plane(normal: SIMD3<Float>(0, 1, 0), offset: 0, centroid: .zero, rms: 0, inlierWeight: 0)

        for _ in 0..<iterations {
            for i in points.indices {
                let r = plane.signedDistance(points[i].position)
                w[i] = max(points[i].weight, 1e-4) / (1 + (r / sigma) * (r / sigma))
            }
            guard let refit = leastSquares(points, weights: w) else { break }
            plane = refit
        }

        // Final stats over inliers (|r| < 2σ).
        var sq: Float = 0, wsum: Float = 0, inlier: Float = 0
        for pnt in points {
            let r = plane.signedDistance(pnt.position)
            if abs(r) < 2 * sigma { inlier += pnt.weight }
            sq += r * r * pnt.weight
            wsum += pnt.weight
        }
        plane.rms = wsum > 0 ? sqrt(sq / wsum) : 0
        plane.inlierWeight = inlier
        return plane
    }

    private static func leastSquares(_ points: [(position: SIMD3<Float>, weight: Float)],
                                     weights w: [Float]) -> Plane? {
        var wsum: Float = 0
        var centroid = SIMD3<Float>.zero
        for i in points.indices { centroid += points[i].position * w[i]; wsum += w[i] }
        guard wsum > 1e-6 else { return nil }
        centroid /= wsum

        var cxx: Float = 0, cxy: Float = 0, cxz: Float = 0, cyy: Float = 0, cyz: Float = 0, czz: Float = 0
        for i in points.indices {
            let d = points[i].position - centroid
            let wi = w[i]
            cxx += wi * d.x * d.x; cxy += wi * d.x * d.y; cxz += wi * d.x * d.z
            cyy += wi * d.y * d.y; cyz += wi * d.y * d.z; czz += wi * d.z * d.z
        }
        let n = smallestEigenvector(cxx: cxx, cxy: cxy, cxz: cxz, cyy: cyy, cyz: cyz, czz: czz)
        guard simd_length(n) > 1e-5 else { return nil }
        let unit = simd_normalize(n)
        return Plane(normal: unit, offset: simd_dot(unit, centroid), centroid: centroid, rms: 0, inlierWeight: 0)
    }

    /// Smallest-eigenvalue eigenvector of a symmetric 3×3 (the plane normal),
    /// via inverse power iteration with a small ridge for stability.
    static func smallestEigenvector(cxx: Float, cxy: Float, cxz: Float,
                                    cyy: Float, cyz: Float, czz: Float) -> SIMD3<Float> {
        let trace = cxx + cyy + czz
        let ridge = max(trace, 1) * 1e-5
        let m = simd_float3x3(SIMD3<Float>(cxx + ridge, cxy, cxz),
                              SIMD3<Float>(cxy, cyy + ridge, cyz),
                              SIMD3<Float>(cxz, cyz, czz + ridge))
        let inv = m.inverse
        var v = SIMD3<Float>(0.577, 0.577, 0.577)
        for _ in 0..<16 {
            v = inv * v
            let len = simd_length(v)
            if !len.isFinite || len < 1e-20 { return SIMD3<Float>(0, 0, 1) }
            v /= len
        }
        return v
    }
}
