import Foundation
import simd

/// Runs a keyframe pose graph + loop closure alongside the v1 dense point cloud.
/// When the graph shifts, it emits a deformation (one node per keyframe) that the
/// caller applies to the accumulated cloud — v1's density and colour, globally
/// corrected.
public actor LiveDriftCorrector {

    public struct Tuning: Sendable {
        public var minTranslationMeters: Float = 0.06
        public var minRotationRadians: Float = 0.09
        public var optimiseEveryNKeyframes = 8
        public var loopClosureEveryNKeyframes = 20
        public var optimiseIterations = 8
        public var loopParameters = LoopClosureDetector.Parameters()
        /// A keyframe counts as "moved" past this (m or rad). Also the gate for
        /// publishing an Update at all — below this the whole-cloud re-warp isn't
        /// worth the churn (a loop closure always publishes regardless).
        public var moveThreshold: Float = 0.03
        public init() {}
    }

    public struct Update: Sendable {
        /// One node per keyframe, delta = corrected · rawARKit⁻¹. Applied to the
        /// *raw* accumulated cloud (and to freshly unprojected frames) to place
        /// everything in the corrected frame.
        public var nodes: [DeformationGraph.Node]
        public var keyframes: Int
        public var loopClosures: Int
        public var initialError: Float
        public var finalError: Float
        public var movedKeyframes: Int
    }

    /// Current correction field — for warping freshly unprojected points before
    /// they're accumulated, so new sweeps register with the corrected cloud.
    public func currentNodes() -> [DeformationGraph.Node] {
        let poses = graph.allPoses()
        return order.compactMap { id in
            guard let corrected = poses[id], let raw = keyframes[id]?.arkitPose else { return nil }
            return DeformationGraph.Node(rawPose: raw, correctedPose: corrected)
        }
    }

    private var config: Tuning
    private let graph = PoseGraph()
    private var keyframes: [Int32: Keyframe] = [:]
    private var order: [Int32] = []
    private var integratedPose: [Int32: simd_float4x4] = [:]
    private var lastKeyframePose: simd_float4x4?
    private var nextID: Int32 = 0
    private var kfCount = 0
    private var loopCount = 0

    private let continuation: AsyncStream<Update>.Continuation
    public nonisolated let updates: AsyncStream<Update>

    public init(tuning: Tuning = Tuning()) {
        self.config = tuning
        (self.updates, self.continuation) = AsyncStream.makeStream(bufferingPolicy: .bufferingNewest(4))
    }

    public func clear() {
        keyframes.removeAll(); order.removeAll(); integratedPose.removeAll()
        lastKeyframePose = nil; nextID = 0; kfCount = 0; loopCount = 0
    }

    /// The corrected pose for the most recent keyframe as an ARKit→corrected
    /// rigid transform (for registering RoomPlan etc).
    public func driftCorrection() -> simd_float4x4 {
        guard let newest = order.last, let corrected = graph.pose(id: newest),
              let raw = keyframes[newest]?.arkitPose else { return matrix_identity_float4x4 }
        return corrected * raw.inverse
    }

    /// Feed a streamed frame. Motion-gated into keyframes internally.
    public func consider(_ frame: ScanFrame) {
        guard let depth = frame.depth else { return }
        if let last = lastKeyframePose {
            let dt = simd_length(Lie.translation(frame.pose) - Lie.translation(last))
            let dr = simd_length(Lie.so3Log(Lie.rotation(last).transpose * Lie.rotation(frame.pose)))
            if dt < config.minTranslationMeters && dr < config.minRotationRadians { return }
        }
        let relative = lastKeyframePose.map { $0.inverse * frame.pose } ?? matrix_identity_float4x4
        lastKeyframePose = frame.pose

        let id = nextID; nextID += 1
        let kf = Keyframe(id: id, capturedAtMs: frame.captureTimestampMs,
                          depth: depth, depthScale: frame.depthScale == 0 ? 1 : frame.depthScale,
                          depthIntrinsics: frame.intrinsics,
                          colorJPEG: nil, colorIntrinsics: nil,
                          arkitPose: frame.pose, relativePose: relative)
        keyframes[id] = kf
        graph.addNode(id: id, pose: frame.pose)
        if let prev = order.last {
            graph.addConstraint(.init(from: prev, to: id, measured: relative, weight: 1))
        }
        order.append(id)
        integratedPose[id] = frame.pose
        kfCount += 1

        var loops = 0
        if config.loopClosureEveryNKeyframes > 0, kfCount % config.loopClosureEveryNKeyframes == 0 {
            loops = detectLoopClosureForNewest()
        }
        if kfCount % config.optimiseEveryNKeyframes == 0 || loops > 0 {
            optimise(loopClosures: loops)
        }
    }

    // MARK: Internals

    @discardableResult
    private func detectLoopClosureForNewest() -> Int {
        guard let newest = order.last else { return 0 }
        let closures = LoopClosureDetector.detect(keyframes: Array(keyframes.values),
                                                  poses: graph.allPoses(),
                                                  parameters: config.loopParameters,
                                                  onlyInvolving: newest)
        for c in closures {
            let w = simd_clamp(0.03 / max(c.rmsErrorMeters, 0.005), 1, 8)
            graph.addConstraint(.init(from: c.from, to: c.to, measured: c.measured, weight: w))
        }
        loopCount += closures.count
        return closures.count
    }

    private func optimise(loopClosures: Int) {
        let (initial, final) = graph.optimize(iterations: config.optimiseIterations)
        let poses = graph.allPoses()
        var nodes: [DeformationGraph.Node] = []
        nodes.reserveCapacity(order.count)
        var moved = 0
        for id in order {
            guard let newPose = poses[id], let raw = keyframes[id]?.arkitPose else { continue }
            let prev = integratedPose[id] ?? raw
            let dt = simd_length(Lie.translation(prev) - Lie.translation(newPose))
            let dr = simd_length(Lie.so3Log(Lie.rotation(prev).transpose * Lie.rotation(newPose)))
            if max(dt, dr) >= config.moveThreshold { moved += 1 }
            // Node delta = corrected · raw⁻¹ — always relative to raw ARKit, so
            // the caller re-warps a pristine cloud each time (no accumulation).
            nodes.append(.init(rawPose: raw, correctedPose: newPose))
            integratedPose[id] = newPose
        }
        guard moved > 0 || loopClosures > 0 else { return }
        continuation.yield(Update(nodes: nodes, keyframes: order.count, loopClosures: loopClosures,
                                  initialError: initial, finalError: final, movedKeyframes: moved))
    }
}
