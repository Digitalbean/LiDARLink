import Foundation
import simd

/// Finds loop closures: pairs of non-adjacent keyframes that observe the same
/// geometry. A candidate pair earns an (expensive) ICP attempt either because
/// the pose graph's current estimate already puts them close together, or
/// because their depth "shapes" look alike regardless of where the graph
/// currently thinks they are — see `KeyframeDescriptor`. Each candidate is
/// verified by point-to-plane ICP between the two keyframes' clouds, and only
/// tight, small-correction, well-conditioned alignments become constraints.
public enum LoopClosureDetector {

    public struct Closure: Sendable, Equatable {
        public var from: Int32
        public var to: Int32
        /// Relative transform `from → to` implied by the ICP alignment.
        public var measured: simd_float4x4
        public var rmsErrorMeters: Float
        public var correspondences: Int
        /// `PoseRefiner.Result.translationConditioning` for the accepted
        /// alignment — how well the matched geometry pinned down translation.
        public var translationConditioning: Float = 0
        /// True when this pair was found only by looking alike (descriptor),
        /// not by the graph's current pose estimate already being nearby —
        /// i.e. this is the closure that recovered from real drift.
        public var recoveredFromDrift: Bool = false
    }

    public struct Parameters: Sendable {
        /// Skip pairs closer than this in capture order (they're already
        /// constrained by odometry).
        public var sequentialWindow: Int = 8
        public var maxCameraDistanceMeters: Float = 1.5
        public var maxViewAngleDegrees: Float = 45
        /// Mean per-bin depth difference (metres) below which two keyframes'
        /// `KeyframeDescriptor`s count as "look alike", independent of pose.
        /// This is what lets a revisit be found after the graph has drifted
        /// past `maxCameraDistanceMeters` — see `KeyframeDescriptor`.
        public var maxDescriptorDistanceMeters: Float = 0.25
        /// Reject an alignment that moves the frame more than this — a large
        /// correction means a wrong match, not a closed loop. Applies equally
        /// to both candidate paths: B's cloud is always seeded at A's pose
        /// before ICP (never B's own, possibly badly-drifted, estimate), so
        /// ICP only ever has to find the small residual between two similar
        /// viewpoints — never the accumulated drift itself.
        public var maxCorrectionMeters: Float = 0.25
        public var maxRMSMeters: Float = 0.02
        public var minCorrespondences: Int = 400
        /// Keep at most this many closures per keyframe (lowest RMS first). Stops
        /// a long revisit from flooding the graph with near-duplicate, weakly
        /// constrained edges that fight each other.
        public var maxClosuresPerKeyframe: Int = 4
        /// Require this fraction of the sampled points to find a correspondence
        /// after alignment — a low match rate means the two clouds barely
        /// overlap and the fit locked onto a small patch.
        public var minMatchedFraction: Float = 0.45
        public var samplePointLimit = 4000
        /// Degeneracy / aperture guard. Reject a closure whose ICP alignment was
        /// not geometrically well-constrained in translation — see
        /// `PoseRefiner.Result.translationConditioning`. A blank wall (or any
        /// single dominant plane) fits with a great RMS and a high match
        /// fraction while sliding freely along the plane; that injects a bogus
        /// 10–20 cm relative-pose constraint and corrupts the graph. Require the
        /// correspondence normals to span at least two independent directions:
        /// `λ₁ / λ₀ ≥ this`. A crease, a corner, or varied room geometry clears
        /// it comfortably (~0.1+); a flat wall sits near 0.
        public var minTranslationConditioning: Float = 0.04
        public init() {}
    }

    /// - Parameter onlyInvolving: when set, only test pairs that include this
    ///   keyframe id. Live mapping uses it to check just the newest keyframe
    ///   against the map instead of re-scanning every pair each cycle.
    public static func detect(keyframes: [Keyframe],
                              poses: [Int32: simd_float4x4],
                              parameters: Parameters = Parameters(),
                              onlyInvolving: Int32? = nil) -> [Closure] {
        let ordered = keyframes.sorted { $0.id < $1.id }
        guard ordered.count > parameters.sequentialWindow + 1 else { return [] }

        var closures: [Closure] = []
        let cosLimit = cos(parameters.maxViewAngleDegrees * .pi / 180)

        for a in 0..<ordered.count {
            let firstB = a + parameters.sequentialWindow + 1
            guard firstB < ordered.count else { continue }
            for b in firstB..<ordered.count {
                let kfA = ordered[a], kfB = ordered[b]
                let idA = kfA.id, idB = kfB.id
                if let focus = onlyInvolving, idA != focus, idB != focus { continue }
                guard let poseA = poses[idA], let poseB = poses[idB] else { continue }

                // Two independent, cheap ways a pair earns an ICP attempt.
                let posA = Lie.translation(poseA), posB = Lie.translation(poseB)
                let rA = Lie.rotation(poseA), rB = Lie.rotation(poseB)
                let dirA = -SIMD3<Float>(rA.columns.2.x, rA.columns.2.y, rA.columns.2.z)
                let dirB = -SIMD3<Float>(rB.columns.2.x, rB.columns.2.y, rB.columns.2.z)
                let poseNearby = simd_length(posA - posB) <= parameters.maxCameraDistanceMeters
                               && simd_dot(dirA, dirB) >= cosLimit
                let descriptorDistance = KeyframeDescriptor.distance(kfA.descriptor, kfB.descriptor)
                let looksAlike = (descriptorDistance ?? .infinity) <= parameters.maxDescriptorDistanceMeters
                guard poseNearby || looksAlike else { continue }

                // Seed B's cloud at A's pose, not B's own (possibly badly
                // drifted) estimate. ICP then only has to find the small
                // residual between two similar viewpoints, never the
                // accumulated drift itself — the same small search radius
                // works whether the pair was found by proximity or by
                // looking alike from far apart in the graph.
                let cloudA = PoseMath.buildPointCloud(depthPayload: kfA.depth,
                                                      depthScale: kfA.depthScale == 0 ? 1 : kfA.depthScale,
                                                      intrinsics: kfA.depthIntrinsics,
                                                      pose: poseA, step: 2)
                let cloudB = PoseMath.buildPointCloud(depthPayload: kfB.depth,
                                                      depthScale: kfB.depthScale == 0 ? 1 : kfB.depthScale,
                                                      intrinsics: kfB.depthIntrinsics,
                                                      pose: poseA, step: 2)
                guard !cloudA.isEmpty, !cloudB.isEmpty else { continue }

                let target = ProgressiveMap(cellSize: 0.03)
                _ = target.deduplicate(cloudA)
                let sample = PointCloudSampler.evenlySample(cloudB, limit: parameters.samplePointLimit)
                // Loop-closure misalignment can be large (accumulated drift), so
                // widen the correspondence search well past the per-frame default.
                let result = PoseRefiner.refine(worldPoints: sample, against: target,
                                                maxIterations: 14,
                                                sampleLimit: parameters.samplePointLimit,
                                                startRadius: parameters.maxCorrectionMeters * 1.5,
                                                maxTranslationMeters: parameters.maxCorrectionMeters,
                                                maxRotationDegrees: 15,
                                                minCorrespondences: parameters.minCorrespondences)
                guard result.applied,
                      result.rmsErrorMeters <= parameters.maxRMSMeters,
                      result.correspondences >= parameters.minCorrespondences,
                      Float(result.correspondences) >= Float(sample.count) * parameters.minMatchedFraction
                else { continue }

                // Degeneracy / aperture guard: a near-planar overlap fits with a
                // tight RMS and a high match fraction yet leaves translation
                // free to slide along the plane. Trust it only if the matched
                // geometry constrained translation in ≥ 2 independent directions.
                guard result.translationConditioning >= parameters.minTranslationConditioning
                else { continue }

                let correctedWorldB = result.correction * poseA   // seeded at poseA, not poseB
                let measured = poseA.inverse * correctedWorldB
                closures.append(Closure(from: idA, to: idB, measured: measured,
                                        rmsErrorMeters: result.rmsErrorMeters,
                                        correspondences: result.correspondences,
                                        translationConditioning: result.translationConditioning,
                                        recoveredFromDrift: !poseNearby))
            }
        }

        // Cap per-keyframe: keep the lowest-RMS closures touching each node.
        guard parameters.maxClosuresPerKeyframe > 0 else { return closures }
        var perNode: [Int32: Int] = [:]
        var kept: [Closure] = []
        for c in closures.sorted(by: { $0.rmsErrorMeters < $1.rmsErrorMeters }) {
            let a = perNode[c.from, default: 0], b = perNode[c.to, default: 0]
            if a >= parameters.maxClosuresPerKeyframe || b >= parameters.maxClosuresPerKeyframe { continue }
            perNode[c.from] = a + 1
            perNode[c.to] = b + 1
            kept.append(c)
        }
        return kept
    }
}
