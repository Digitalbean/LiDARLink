import Foundation
import simd

/// SO(3) / SE(3) helpers for the pose graph. Poses are `simd_float4x4`
/// camera-to-world transforms; tangent vectors are `SIMD3` (rotation) + `SIMD3`
/// (translation), applied as a right perturbation `T · [exp(ω) | ρ]`.
public enum Lie {

    // MARK: SO(3)

    /// Rotation matrix from a rotation vector (axis * angle), via Rodrigues.
    public static func so3Exp(_ omega: SIMD3<Float>) -> simd_float3x3 {
        let theta = simd_length(omega)
        if theta < 1e-8 { return matrix_identity_float3x3 }
        let axis = omega / theta
        let k = skew(axis)
        return matrix_identity_float3x3 + sin(theta) * k + (1 - cos(theta)) * (k * k)
    }

    /// Rotation vector from a rotation matrix.
    public static func so3Log(_ r: simd_float3x3) -> SIMD3<Float> {
        let trace = r.columns.0.x + r.columns.1.y + r.columns.2.z
        let cosTheta = simd_clamp((trace - 1) / 2, -1, 1)
        let theta = acos(cosTheta)
        if theta < 1e-7 {
            // First-order: vee of the antisymmetric part.
            return SIMD3<Float>(r.columns.1.z - r.columns.2.y,
                                r.columns.2.x - r.columns.0.z,
                                r.columns.0.y - r.columns.1.x) * 0.5
        }
        let s = 2 * sin(theta)
        return SIMD3<Float>(r.columns.1.z - r.columns.2.y,
                            r.columns.2.x - r.columns.0.z,
                            r.columns.0.y - r.columns.1.x) * (theta / s)
    }

    public static func skew(_ v: SIMD3<Float>) -> simd_float3x3 {
        simd_float3x3(SIMD3<Float>(0, v.z, -v.y),
                      SIMD3<Float>(-v.z, 0, v.x),
                      SIMD3<Float>(v.y, -v.x, 0))
    }

    // MARK: Pose (SE(3) as 4x4)

    public static func pose(rotation r: simd_float3x3, translation t: SIMD3<Float>) -> simd_float4x4 {
        simd_float4x4(SIMD4<Float>(r.columns.0, 0),
                      SIMD4<Float>(r.columns.1, 0),
                      SIMD4<Float>(r.columns.2, 0),
                      SIMD4<Float>(t, 1))
    }

    public static func rotation(_ T: simd_float4x4) -> simd_float3x3 {
        simd_float3x3(SIMD3<Float>(T.columns.0.x, T.columns.0.y, T.columns.0.z),
                      SIMD3<Float>(T.columns.1.x, T.columns.1.y, T.columns.1.z),
                      SIMD3<Float>(T.columns.2.x, T.columns.2.y, T.columns.2.z))
    }

    public static func translation(_ T: simd_float4x4) -> SIMD3<Float> {
        SIMD3<Float>(T.columns.3.x, T.columns.3.y, T.columns.3.z)
    }

    /// Right perturbation: `T · [ exp(ω) | ρ ]`.
    public static func retract(_ T: simd_float4x4, rotation omega: SIMD3<Float>, translation rho: SIMD3<Float>) -> simd_float4x4 {
        T * pose(rotation: so3Exp(omega), translation: rho)
    }

    /// The 6-vector error `log( Z⁻¹ · (Tᵢ⁻¹ · Tⱼ) )` — how far the current
    /// estimate of the relative transform is from the measured one `Z`.
    /// Returns (rotation error, translation error).
    public static func betweenError(from Ti: simd_float4x4,
                                    to Tj: simd_float4x4,
                                    measured Z: simd_float4x4) -> (rotation: SIMD3<Float>, translation: SIMD3<Float>) {
        let predicted = Ti.inverse * Tj
        let err = Z.inverse * predicted
        return (so3Log(rotation(err)), translation(err))
    }
}
