import Foundation
import simd

/// Frame-to-model alignment: nudges a freshly unprojected depth frame onto the
/// accumulated voxel map to soak up ARKit pose drift before fusion.
///
/// Point-to-point ICP (KISS-ICP style — robust weighting, shrinking
/// correspondence radius, stop on small correction) solved with a small-angle
/// Gauss-Newton step. ARKit already gives a good pose prior every frame, so the
/// correction is expected to be sub-centimetre; anything large is treated as a
/// tracking failure and rejected in favour of ARKit's pose.
public enum PoseRefiner {

    public struct Result: Sendable, Equatable {
        /// World-space correction: `correctedWorld = correction * arkitWorld`.
        public var correction: simd_float4x4
        /// False when the correction was rejected (no map, too little overlap,
        /// diverged, or implausibly large) — caller should keep the ARKit pose.
        public var applied: Bool
        public var iterations: Int
        public var correspondences: Int
        public var rmsErrorMeters: Float
        /// How well the correspondence set constrains *translation*, in `[0, 1]`.
        ///
        /// It is `λ₁ / λ₀`, the ratio of the second-largest to the largest
        /// eigenvalue of the scatter matrix `Σ nᵢnᵢᵀ` of the unit correspondence
        /// normals. A single flat plane has normals all pointing one way, so the
        /// scatter matrix is rank-1 (`λ₁ ≈ 0`) and translation slides freely in
        /// the plane — the aperture problem — even when the RMS looks great.
        /// Two independent surface orientations (a crease, a corner, varied
        /// geometry) push `λ₁` up toward `λ₀`. `0` when the fit was not applied.
        public var translationConditioning: Float = 0

        public static let identity = Result(correction: matrix_identity_float4x4,
                                            applied: false, iterations: 0,
                                            correspondences: 0, rmsErrorMeters: 0,
                                            translationConditioning: 0)
    }

    public static func refine(worldPoints: [PointCloudPoint],
                              against map: ProgressiveMap,
                              maxIterations: Int = 6,      // point-to-plane converges fast
                              sampleLimit: Int = 1200,
                              startRadius: Float = 0.12,
                              minRadius: Float = 0.03,
                              maxTranslationMeters: Float = 0.08,
                              maxRotationDegrees: Float = 4,
                              minCorrespondences: Int = 40,
                              minLifecycle: ProgressiveMap.GeometryLifecycle = .candidate) -> Result {
        guard map.occupiedCellCount > 0, worldPoints.count >= minCorrespondences else {
            return .identity
        }

        // Even subsample of the frame.
        var source: [SIMD3<Float>] = []
        let step = max(1, worldPoints.count / sampleLimit)
        source.reserveCapacity(worldPoints.count / step + 1)
        var i = 0
        while i < worldPoints.count { source.append(worldPoints[i].position); i += step }

        var total = matrix_identity_float4x4
        var radius = startRadius
        var lastCorrespondences = 0
        var lastRms: Float = 0
        // Unweighted scatter of the correspondence normals from the last
        // iteration: [xx, xy, xz, yy, yz, zz]. Drives `translationConditioning`.
        var lastNormalScatter = [Float](repeating: 0, count: 6)

        var iteration = 0
        while iteration < maxIterations {
            iteration += 1

            // Normal equations for [ω | τ] (6-vector).
            var ata = [Float](repeating: 0, count: 36)
            var atb = [Float](repeating: 0, count: 6)
            var correspondences = 0
            var squaredError: Float = 0
            var normalScatter = [Float](repeating: 0, count: 6)
            let huberDelta = max(radius * 0.5, 0.01)

            let surfels = map.nearestSurfels(to: source, maxRadius: radius, minLifecycle: minLifecycle)
            for (index, s) in source.enumerated() {
                guard let surfel = surfels[index] else { continue }
                // Point-to-plane: residual is the signed distance to the local
                // surface, so tangential ambiguity on planar regions can't drag
                // the estimate toward zero.
                let n = surfel.normal
                let planeResidual = simd_dot(surfel.point - s, n)
                let residual = abs(planeResidual)
                let weight: Float = residual <= huberDelta ? 1 : huberDelta / max(residual, 1e-6)
                correspondences += 1
                squaredError += planeResidual * planeResidual
                normalScatter[0] += n.x * n.x
                normalScatter[1] += n.x * n.y
                normalScatter[2] += n.x * n.z
                normalScatter[3] += n.y * n.y
                normalScatter[4] += n.y * n.z
                normalScatter[5] += n.z * n.z

                // One equation: n · ( [-[s]x | I] [ω;τ] ) = planeResidual
                let nCrossS = simd_cross(n, s)     // n · (ω × s) = (s × n) · ω = -(n × s) · ω
                let row: [Float] = [-nCrossS.x, -nCrossS.y, -nCrossS.z, n.x, n.y, n.z]
                let wr = weight * planeResidual
                for a in 0..<6 {
                    atb[a] += row[a] * wr
                    let wa = weight * row[a]
                    for b in a..<6 { ata[a * 6 + b] += wa * row[b] }
                }
            }

            guard correspondences >= minCorrespondences else { return .identity }
            lastCorrespondences = correspondences
            lastRms = sqrt(squaredError / Float(correspondences))
            lastNormalScatter = normalScatter

            // Mirror + Levenberg damping, then solve.
            for a in 0..<6 {
                ata[a * 6 + a] += 1e-6 * ata[a * 6 + a] + 1e-9
                for b in (a + 1)..<6 { ata[b * 6 + a] = ata[a * 6 + b] }
            }
            guard let delta = solve6(ata, atb) else { return .identity }

            let omega = SIMD3<Float>(delta[0], delta[1], delta[2])
            let tau = SIMD3<Float>(delta[3], delta[4], delta[5])
            let increment = rigidTransform(rotationVector: omega, translation: tau)
            total = increment * total
            for j in source.indices {
                let p = increment * SIMD4<Float>(source[j], 1)
                source[j] = SIMD3<Float>(p.x, p.y, p.z)
            }

            radius = max(radius * 0.7, minRadius)
            if simd_length(omega) < 1e-4 && simd_length(tau) < 1e-4 { break }
        }

        // Plausibility gate — large corrections are tracking failures.
        let translation = simd_length(SIMD3<Float>(total.columns.3.x, total.columns.3.y, total.columns.3.z))
        let rotationDegrees = rotationAngleDegrees(total)
        guard translation <= maxTranslationMeters, rotationDegrees <= maxRotationDegrees else {
            return .identity
        }

        return Result(correction: total, applied: true, iterations: iteration,
                      correspondences: lastCorrespondences, rmsErrorMeters: lastRms,
                      translationConditioning: translationConditioning(scatter: lastNormalScatter,
                                                                       count: lastCorrespondences))
    }

    /// `λ₁ / λ₀` of the correspondence-normal scatter matrix (see
    /// `Result.translationConditioning`). `scatter` is `[xx, xy, xz, yy, yz, zz]`
    /// summed over `count` correspondences.
    static func translationConditioning(scatter: [Float], count: Int) -> Float {
        guard count > 0 else { return 0 }
        let inv = 1 / Float(count)
        let e = symmetricEigenvalues3x3(xx: scatter[0] * inv, xy: scatter[1] * inv,
                                        xz: scatter[2] * inv, yy: scatter[3] * inv,
                                        yz: scatter[4] * inv, zz: scatter[5] * inv)
        // e is sorted descending; e[0] is the trace-dominant direction.
        guard e[0] > 1e-12 else { return 0 }
        return simd_clamp(e[1] / e[0], 0, 1)
    }

    /// Eigenvalues of a real symmetric 3x3 matrix, sorted descending. Closed-form
    /// (Smith 1961) — no iteration, safe for the tiny matrices here.
    static func symmetricEigenvalues3x3(xx: Float, xy: Float, xz: Float,
                                        yy: Float, yz: Float, zz: Float) -> SIMD3<Float> {
        let p1 = xy * xy + xz * xz + yz * yz
        let q = (xx + yy + zz) / 3
        if p1 < 1e-18 {
            // Already diagonal.
            let vals = [xx, yy, zz].sorted(by: >)
            return SIMD3<Float>(vals[0], vals[1], vals[2])
        }
        let p2 = (xx - q) * (xx - q) + (yy - q) * (yy - q) + (zz - q) * (zz - q) + 2 * p1
        let p = sqrt(p2 / 6)
        // B = (A - qI) / p
        let bxx = (xx - q) / p, byy = (yy - q) / p, bzz = (zz - q) / p
        let bxy = xy / p, bxz = xz / p, byz = yz / p
        let detB = bxx * (byy * bzz - byz * byz)
                 - bxy * (bxy * bzz - byz * bxz)
                 + bxz * (bxy * byz - byy * bxz)
        let r = simd_clamp(detB / 2, -1, 1)
        let phi = acos(r) / 3
        let eig1 = q + 2 * p * cos(phi)                       // largest
        let eig3 = q + 2 * p * cos(phi + 2 * Float.pi / 3)    // smallest
        let eig2 = 3 * q - eig1 - eig3                        // middle (trace)
        return SIMD3<Float>(eig1, eig2, eig3)
    }

    // MARK: Math

    /// Rodrigues rotation from a small rotation vector, plus translation.
    static func rigidTransform(rotationVector omega: SIMD3<Float>, translation tau: SIMD3<Float>) -> simd_float4x4 {
        let theta = simd_length(omega)
        let rotation: simd_float3x3
        if theta < 1e-8 {
            rotation = matrix_identity_float3x3
        } else {
            let axis = omega / theta
            let k = simd_float3x3(SIMD3<Float>(0, axis.z, -axis.y),
                                  SIMD3<Float>(-axis.z, 0, axis.x),
                                  SIMD3<Float>(axis.y, -axis.x, 0))
            rotation = matrix_identity_float3x3 + sin(theta) * k + (1 - cos(theta)) * (k * k)
        }
        return simd_float4x4(SIMD4<Float>(rotation.columns.0, 0),
                             SIMD4<Float>(rotation.columns.1, 0),
                             SIMD4<Float>(rotation.columns.2, 0),
                             SIMD4<Float>(tau, 1))
    }

    static func rotationAngleDegrees(_ transform: simd_float4x4) -> Float {
        let trace = transform.columns.0.x + transform.columns.1.y + transform.columns.2.z
        return acos(simd_clamp((trace - 1) / 2, -1, 1)) * 180 / .pi
    }

    /// Cholesky solve of a symmetric positive-definite 6x6 (row-major).
    static func solve6(_ a: [Float], _ b: [Float]) -> [Float]? {
        var l = [Float](repeating: 0, count: 36)
        for j in 0..<6 {
            var diag = a[j * 6 + j]
            for k in 0..<j { diag -= l[j * 6 + k] * l[j * 6 + k] }
            guard diag > 1e-12 else { return nil }
            l[j * 6 + j] = sqrt(diag)
            for i in (j + 1)..<6 {
                var sum = a[i * 6 + j]
                for k in 0..<j { sum -= l[i * 6 + k] * l[j * 6 + k] }
                l[i * 6 + j] = sum / l[j * 6 + j]
            }
        }
        var y = [Float](repeating: 0, count: 6)
        for i in 0..<6 {
            var sum = b[i]
            for k in 0..<i { sum -= l[i * 6 + k] * y[k] }
            y[i] = sum / l[i * 6 + i]
        }
        var x = [Float](repeating: 0, count: 6)
        for i in stride(from: 5, through: 0, by: -1) {
            var sum = y[i]
            for k in (i + 1)..<6 { sum -= l[k * 6 + i] * x[k] }
            x[i] = sum / l[i * 6 + i]
        }
        return x
    }
}
