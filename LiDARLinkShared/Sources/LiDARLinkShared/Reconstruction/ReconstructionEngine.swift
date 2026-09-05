import Foundation
import simd

/// v2 reconstruction: keyframes in, a pose-graph-consistent textured surface
/// out. The pose graph is the source of truth for where every keyframe sits;
/// the TSDF is always integrated *at the graph's current poses*, and a graph
/// update deintegrates each moved keyframe at its old pose and reintegrates it
/// at the new one.
///
/// Headless. Serialised through actor isolation.
public actor ReconstructionEngine {

    public struct Configuration: Sendable {
        public var voxelSizeMeters: Float = 0.02
        public var truncationVoxels: Float = 4
        public var minConfidence: UInt8 = 1
        /// A keyframe is re-integrated when the graph moves its pose by more
        /// than this (metres of translation or radians of rotation).
        public var reintegrationThreshold: Float = 0.01
        /// Emit a `.surface` event after every `add`. Live display wants this;
        /// batch/CLI processing turns it off and meshes once at the end.
        public var emitSurfaceOnAdd: Bool = true
        public init() {}
    }

    /// Pacing for incremental live mapping (`ingestLive`).
    public struct LiveTuning: Sendable {
        /// Re-mesh the surface at most this often, in ms of capture time. A full
        /// re-mesh grows with the scanned volume, so keep this loose.
        public var surfaceIntervalMs: UInt64 = 1_500
        /// Tighten the pose graph every N keyframes.
        public var optimiseEveryNKeyframes = 10
        public var optimiseIterations = 4
        /// Look for a loop closure on the newest keyframe every N keyframes
        /// (0 disables live loop closure).
        public var loopClosureEveryNKeyframes = 24
        public var loopParameters = LoopClosureDetector.Parameters()
        /// Cap keyframe re-integrations per optimise so a large correction never
        /// stalls capture; the rest catch up on following cycles.
        public var maxReintegrationsPerCycle = 8
        public init() {}
    }

    public enum Event: Sendable {
        case surface(MarchingCubes.Mesh)
        case cleared
        case optimised(keyframes: Int, constraints: Int, initialError: Float, finalError: Float, reintegrated: Int)
    }

    private var config: Configuration
    private let graph = PoseGraph()
    private var keyframes: [Int32: Keyframe] = [:]
    private var order: [Int32] = []
    private var integratedPose: [Int32: simd_float4x4] = [:]
    private var tsdf: TSDFVolume
    private var liveCount = 0
    private var lastSurfaceMs: UInt64 = 0

    private let continuation: AsyncStream<Event>.Continuation
    public nonisolated let events: AsyncStream<Event>

    public init(configuration: Configuration = Configuration()) {
        self.config = configuration
        self.tsdf = TSDFVolume(voxelSize: configuration.voxelSizeMeters, truncationVoxels: configuration.truncationVoxels)
        (self.events, self.continuation) = AsyncStream.makeStream(bufferingPolicy: .bufferingNewest(16))
    }

    // MARK: Ingest

    public func add(_ keyframe: Keyframe) {
        keyframes[keyframe.id] = keyframe
        graph.addNode(id: keyframe.id, pose: keyframe.arkitPose)
        if let previous = order.last {
            graph.addConstraint(.init(from: previous, to: keyframe.id, measured: keyframe.relativePose, weight: 1))
        }
        order.append(keyframe.id)
        integrate(keyframe, at: keyframe.arkitPose, sign: 1)
        integratedPose[keyframe.id] = keyframe.arkitPose
        if config.emitSurfaceOnAdd { continuation.yield(.surface(tsdf.mesh())) }
    }

    /// Incremental live mapping: fuse the keyframe now, and periodically tighten
    /// the pose graph, look for a loop closure on the newest keyframe, and
    /// re-mesh. Surface updates and optimise reports arrive on `events`.
    public func ingestLive(_ keyframe: Keyframe, tuning: LiveTuning = LiveTuning()) {
        keyframes[keyframe.id] = keyframe
        graph.addNode(id: keyframe.id, pose: keyframe.arkitPose)
        if let previous = order.last {
            graph.addConstraint(.init(from: previous, to: keyframe.id, measured: keyframe.relativePose, weight: 1))
        }
        order.append(keyframe.id)
        integrate(keyframe, at: keyframe.arkitPose, sign: 1)
        integratedPose[keyframe.id] = keyframe.arkitPose
        liveCount += 1

        if tuning.loopClosureEveryNKeyframes > 0, liveCount % tuning.loopClosureEveryNKeyframes == 0 {
            detectLoopClosuresForNewest(parameters: tuning.loopParameters)
        }

        let dueForSurface = lastSurfaceMs == 0 || keyframe.capturedAtMs &- lastSurfaceMs >= tuning.surfaceIntervalMs
        if liveCount % tuning.optimiseEveryNKeyframes == 0 {
            let reintegrated = optimiseBounded(iterations: tuning.optimiseIterations,
                                               maxReintegrations: tuning.maxReintegrationsPerCycle,
                                               remesh: dueForSurface)
            if dueForSurface || reintegrated > 0 { lastSurfaceMs = keyframe.capturedAtMs }
        } else if dueForSurface {
            lastSurfaceMs = keyframe.capturedAtMs
            continuation.yield(.surface(tsdf.mesh()))
        }
    }

    private func detectLoopClosuresForNewest(parameters: LoopClosureDetector.Parameters) {
        guard let newest = order.last else { return }
        let closures = LoopClosureDetector.detect(keyframes: Array(keyframes.values),
                                                  poses: graph.allPoses(),
                                                  parameters: parameters,
                                                  onlyInvolving: newest)
        for c in closures {
            let weight = simd_clamp(0.03 / max(c.rmsErrorMeters, 0.005), 1, 8)
            graph.addConstraint(.init(from: c.from, to: c.to, measured: c.measured, weight: weight))
        }
    }

    /// Like `optimise` but only re-integrates the `maxReintegrations` most-moved
    /// keyframes this cycle, so a big loop-closure correction is absorbed over a
    /// few frames instead of one long stall. Returns how many were re-integrated.
    @discardableResult
    private func optimiseBounded(iterations: Int, maxReintegrations: Int, remesh: Bool) -> Int {
        let (initial, final) = graph.optimize(iterations: iterations)
        let poses = graph.allPoses()

        var moved: [(id: Int32, delta: Float, newPose: simd_float4x4, oldPose: simd_float4x4)] = []
        for id in order {
            guard let newPose = poses[id], let oldPose = integratedPose[id] else { continue }
            let d = poseDelta(oldPose, newPose)
            if d >= config.reintegrationThreshold {
                moved.append((id, d, newPose, oldPose))
            }
        }
        moved.sort { $0.delta > $1.delta }
        let batch = maxReintegrations > 0 ? Array(moved.prefix(maxReintegrations)) : moved
        for m in batch {
            guard let kf = keyframes[m.id] else { continue }
            integrate(kf, at: m.oldPose, sign: -1)
            integrate(kf, at: m.newPose, sign: 1)
            integratedPose[m.id] = m.newPose
        }

        continuation.yield(.optimised(keyframes: order.count, constraints: graph.constraintCount,
                                      initialError: initial, finalError: final, reintegrated: batch.count))
        if remesh || !batch.isEmpty {
            continuation.yield(.surface(tsdf.mesh()))
        }
        return batch.count
    }

    /// A geometric loop closure: `to`'s keyframe is the same place as `from`'s,
    /// related by `measured` (from → to). Typically produced by ICP between two
    /// overlapping keyframes.
    public func addLoopClosure(from: Int32, to: Int32, measured: simd_float4x4, weight: Float = 4) {
        graph.addConstraint(.init(from: from, to: to, measured: measured, weight: weight))
    }

    /// Runs loop-closure detection over the current keyframes/poses and adds any
    /// verified closures as constraints. Returns how many were added.
    @discardableResult
    public func detectLoopClosures(parameters: LoopClosureDetector.Parameters = .init()) -> Int {
        let closures = LoopClosureDetector.detect(keyframes: Array(keyframes.values),
                                                  poses: graph.allPoses(),
                                                  parameters: parameters)
        for c in closures {
            let weight = simd_clamp(0.03 / max(c.rmsErrorMeters, 0.005), 1, 8)
            graph.addConstraint(.init(from: c.from, to: c.to, measured: c.measured, weight: weight))
        }
        return closures.count
    }

    // MARK: Optimise

    @discardableResult
    public func optimise(iterations: Int = 15) -> Event {
        let (initial, final) = graph.optimize(iterations: iterations)
        let optimisedPoses = graph.allPoses()

        var reintegrated = 0
        for id in order {
            guard let keyframe = keyframes[id],
                  let newPose = optimisedPoses[id],
                  let oldPose = integratedPose[id] else { continue }
            if poseDelta(oldPose, newPose) < config.reintegrationThreshold { continue }
            integrate(keyframe, at: oldPose, sign: -1)
            integrate(keyframe, at: newPose, sign: 1)
            integratedPose[id] = newPose
            reintegrated += 1
        }

        let event = Event.optimised(keyframes: order.count, constraints: graph.constraintCount,
                                    initialError: initial, finalError: final, reintegrated: reintegrated)
        continuation.yield(event)
        if reintegrated > 0 { continuation.yield(.surface(tsdf.mesh())) }
        return event
    }

    // MARK: Access

    public func currentMesh() -> MarchingCubes.Mesh { tsdf.mesh() }
    public func pose(of id: Int32) -> simd_float4x4? { graph.pose(id: id) }
    public var keyframeCount: Int { order.count }
    public func poseGraphError() -> (initial: Float, final: Float) { graph.optimize(iterations: 0) }

    /// Finalize: the current mesh, textured by projecting the colour keyframes at
    /// their graph-optimised poses.
    public func bakeTexturedMesh(atlasSize: Int = 2048, maxTriangles: Int = 0) -> TextureBaker.BakedMesh? {
        let raw = tsdf.mesh()
        guard !raw.isEmpty else { return nil }
        // Per-triangle atlas charting only holds up when the triangle count fits
        // the atlas — otherwise every chart collapses to a few texels. Decimate
        // to a budget derived from the atlas area (~one 26px cell per triangle).
        let budget = maxTriangles > 0 ? maxTriangles : max(20_000, (atlasSize * atlasSize) / 700)
        let mesh = MeshDecimator.decimated(raw, targetTriangles: budget)
        let optimised = graph.allPoses()
        var views: [TextureBaker.View] = []
        for id in order {
            guard let keyframe = keyframes[id], let jpeg = keyframe.colorJPEG, let pose = optimised[id],
                  let sampler = ColorImageSampler(color: ColorPayload(width: 0, height: 0, jpegData: jpeg)) else { continue }
            let source = keyframe.colorIntrinsics ?? keyframe.depthIntrinsics
            let k = source.scaled(to: sampler.width, height: sampler.height)
            views.append(.init(sampler: sampler, pose: pose, intrinsics: k))
        }
        guard !views.isEmpty else { return nil }
        return TextureBaker.bake(mesh: mesh, views: views, atlasSize: atlasSize)
    }

    public func clear() {
        keyframes.removeAll()
        order.removeAll()
        integratedPose.removeAll()
        tsdf.clear()
        liveCount = 0
        lastSurfaceMs = 0
        continuation.yield(.cleared)
    }

    // MARK: Internals

    private func integrate(_ keyframe: Keyframe, at pose: simd_float4x4, sign: Float) {
        let depth = keyframe.depth
        let sampler = keyframe.colorJPEG.flatMap { ColorImageSampler(color: ColorPayload(width: keyframe.colorIntrinsics.map { $0.imageWidth } ?? 0,
                                                                                        height: keyframe.colorIntrinsics.map { $0.imageHeight } ?? 0,
                                                                                        jpegData: $0)) }
        tsdf.integrate(depth: depth.float16Array(),
                       width: depth.width, height: depth.height,
                       depthScale: keyframe.depthScale == 0 ? 1 : keyframe.depthScale,
                       intrinsics: keyframe.depthIntrinsics,
                       pose: pose,
                       confidence: depth.confidenceArray(),
                       minConfidence: config.minConfidence,
                       sign: sign,
                       colorFor: sampler.map { s in { x, y in
                           s.color(atX: x * s.width / max(depth.width, 1),
                                   y: y * s.height / max(depth.height, 1))
                       } })
    }

    private func poseDelta(_ a: simd_float4x4, _ b: simd_float4x4) -> Float {
        let dt = simd_length(Lie.translation(a) - Lie.translation(b))
        let dr = simd_length(Lie.so3Log(Lie.rotation(a).transpose * Lie.rotation(b)))
        return max(dt, dr)
    }
}
