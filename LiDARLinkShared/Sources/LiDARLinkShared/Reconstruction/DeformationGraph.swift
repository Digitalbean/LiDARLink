import Foundation
import simd

/// Embedded-deformation-style warping: a set of nodes, each an anchor position
/// plus a rigid correction, blended by a smooth inverse-square weight so a loop
/// closure bends a whole point set or surfel map without seams.
///
/// The nodes only sample the camera trajectory (one per keyframe), so a naïve
/// blend has two failure modes that this implementation guards against:
///
///  1. **Rotational lever arm.** A keyframe's rotational correction, applied to a
///     point metres away, displaces it in proportion to that distance. Points on
///     one wall dominated by different keyframes then shear apart — the wall
///     smears into a thick slab. Each node's rotational contribution is clamped
///     (`Limits.maxLeverDisplacement`), and rotations are blended in SO(3) log
///     space, not by averaging matrices.
///  2. **No support far from the path.** A point far from every node has no real
///     information about its correction. The blend fades back toward identity as
///     the summed weight drops (`Limits.supportFalloff`), so far geometry stays
///     at its self-consistent raw pose instead of being dragged by a guess.
///
/// A final `Limits.maxDisplacement` cap keeps a single bad loop closure from
/// spraying the cloud across the room.
public enum DeformationGraph {

    public struct Limits: Sendable {
        /// Max nodes actually blended per point (the list is strided down past
        /// this). A fixed subsample, so it adds no per-point discontinuity;
        /// keeps the whole-cloud re-warp well under a frame.
        public var maxNodes = 32
        /// Inverse-square softening, in m². Larger = smoother, less local.
        public var epsilon: Float = 0.04
        /// Cap on one node's rotational displacement of a point (metres).
        public var maxLeverDisplacement: Float = 0.15
        /// Cap on the magnitude of the blended rotation (radians, ~5.7°).
        public var maxRotationRadians: Float = 0.10
        /// Weight at which the blend is at half strength toward identity; below
        /// this the correction fades out (point is far from every node). Tuned so
        /// anything within ~2 m of a node gets the full correction and only
        /// truly isolated geometry (>3 m from the whole path) fades.
        public var supportFalloff: Float = 0.05
        /// Final safety cap on total displacement of any point (metres).
        public var maxDisplacement: Float = 0.6
        public init() {}
    }

    public static var limits = Limits()

    public struct Node: Sendable {
        /// Position used for distance weighting — the raw-space camera position
        /// of the keyframe (the cloud being warped is in raw space).
        public var anchor: SIMD3<Float>
        /// Rotation of the correction (about `pivot`).
        public var rotation: simd_float3x3
        /// Centre of rotation. Raw camera position for a keyframe node; origin
        /// for a legacy explicitly-constructed node (global `R·p + offset`).
        public var pivot: SIMD3<Float>
        /// Translation added after the pivoted rotation.
        public var offset: SIMD3<Float>

        /// Legacy: a global affine `f(p) = rotation·p + translation`.
        public init(anchor: SIMD3<Float>, rotation: simd_float3x3, translation: SIMD3<Float>) {
            self.anchor = anchor
            self.rotation = rotation
            self.pivot = .zero
            self.offset = translation
        }

        /// A node built from a keyframe's raw → corrected poses. The correction
        /// rotates about the raw camera position and maps it to the corrected
        /// one: `f(p) = Rδ·(p − a_raw) + a_corrected`.
        public init(rawPose: simd_float4x4, correctedPose: simd_float4x4) {
            let aRaw = Lie.translation(rawPose)
            let aCor = Lie.translation(correctedPose)
            self.rotation = Lie.rotation(correctedPose) * Lie.rotation(rawPose).transpose
            self.anchor = aRaw
            self.pivot = aRaw
            self.offset = aCor - aRaw
        }

        /// `translation` in the legacy sense — kept for call sites / tests that
        /// read it back.
        public var translation: SIMD3<Float> { offset - rotation * pivot + pivot }

        /// This node's correction applied to `p`, with the rotational lever arm
        /// clamped.
        func apply(to p: SIMD3<Float>, limits: Limits) -> SIMD3<Float> {
            let lever = p - pivot
            var rotated = rotation * lever
            let disp = rotated - lever
            let d = simd_length(disp)
            if d > limits.maxLeverDisplacement {
                rotated = lever + disp * (limits.maxLeverDisplacement / d)
            }
            return rotated + pivot + offset
        }

        var isNegligible: Bool {
            simd_length(offset) < 1e-4 &&
            simd_length(Lie.so3Log(rotation)) < 1e-4
        }
    }

    /// Warp one position by a smooth blend of the nodes.
    public static func warp(_ p: SIMD3<Float>, nodes: [Node], influence: Int = 0) -> SIMD3<Float> {
        warpWithRotation(p, nodes: nodes, influence: influence).position
    }

    public static func warpWithRotation(_ p: SIMD3<Float>, nodes: [Node], influence: Int = 0)
        -> (position: SIMD3<Float>, rotation: simd_float3x3) {
        guard !nodes.isEmpty else { return (p, matrix_identity_float3x3) }
        if nodes.count == 1 {
            return (nodes[0].apply(to: p, limits: limits), nodes[0].rotation)
        }
        return PreparedField(nodes: nodes, limits: limits).warp(p)
    }

    /// Bulk warp a point cloud (returns a new array). No-ops when the field
    /// carries no meaningful correction. Per-node terms (`so3Log`, strided
    /// subsample) are computed once, not per point.
    public static func warp(points: [PointCloudPoint], nodes: [Node], influence: Int = 0) -> [PointCloudPoint] {
        guard !nodes.isEmpty, nodes.contains(where: { !$0.isNegligible }) else { return points }
        if nodes.count == 1 {
            let n = nodes[0]
            return points.map { var o = $0; o.position = n.apply(to: $0.position, limits: limits); return o }
        }
        let field = PreparedField(nodes: nodes, limits: limits)
        return points.map { var o = $0; o.position = field.warp($0.position).position; return o }
    }

    /// A node list with its per-node blend terms precomputed.
    struct PreparedField {
        let nodes: [Node]
        let omega: [SIMD3<Float>]          // so3Log(rotation) per node
        let limits: Limits

        init(nodes allNodes: [Node], limits: Limits) {
            self.nodes = DeformationGraph.strided(allNodes, max: limits.maxNodes)
            self.omega = self.nodes.map { Lie.so3Log($0.rotation) }
            self.limits = limits
        }

        func warp(_ p: SIMD3<Float>) -> (position: SIMD3<Float>, rotation: simd_float3x3) {
            let lim = limits
            var wsum: Float = 0
            var pos = SIMD3<Float>.zero
            var om = SIMD3<Float>.zero
            for i in nodes.indices {
                let d2 = simd_length_squared(nodes[i].anchor - p)
                let w = 1 / (d2 + lim.epsilon)
                wsum += w
                pos += w * nodes[i].apply(to: p, limits: lim)
                om += w * omega[i]
            }
            guard wsum > 1e-9 else { return (p, matrix_identity_float3x3) }

            var warped = pos / wsum
            om /= wsum
            let ang = simd_length(om)
            if ang > lim.maxRotationRadians { om *= lim.maxRotationRadians / ang }

            // Fade toward identity where node support is weak (far from the path).
            let support = wsum / (wsum + lim.supportFalloff)
            warped = p + (warped - p) * support

            // Safety cap — a single bad closure must not spray the cloud.
            let total = warped - p
            let tl = simd_length(total)
            if tl > lim.maxDisplacement { warped = p + total * (lim.maxDisplacement / tl) }

            return (warped, Lie.so3Exp(om * support))
        }
    }

    private static func strided(_ nodes: [Node], max: Int) -> [Node] {
        guard nodes.count > max, max > 1 else { return nodes }
        let step = Float(nodes.count - 1) / Float(max - 1)
        return (0..<max).map { nodes[Int((Float($0) * step).rounded())] }
    }

    static func orthonormalized(_ m: simd_float3x3) -> simd_float3x3 {
        var x = m.columns.0
        if simd_length(x) < 1e-6 { return matrix_identity_float3x3 }
        x = simd_normalize(x)
        var y = m.columns.1 - simd_dot(m.columns.1, x) * x
        if simd_length(y) < 1e-6 { return matrix_identity_float3x3 }
        y = simd_normalize(y)
        return simd_float3x3(x, y, simd_cross(x, y))
    }
}
