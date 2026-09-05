import Foundation
import simd

/// v2 live mapping: a pose graph over keyframes with a surfel map riding on top.
///
/// Each keyframe is fused into the surfel map at its current graph pose. Every
/// few keyframes the graph is tightened (and the newest keyframe checked for a
/// loop closure); any keyframe the graph moved has its surfels rigidly
/// transformed to match — no volume, no re-integration.
///
/// Headless. Serialised through actor isolation.
public actor SurfelEngine {

    public struct Configuration: Sendable {
        public var surfel = SurfelMap.Parameters()
        /// A keyframe's surfels are moved when the graph shifts its pose by more
        /// than this (metres or radians).
        public var correctionThreshold: Float = 0.008
        /// Snap coplanar surfels onto detected walls/floor/ceiling (and
        /// gravity-align them) before display and before the mesh bake.
        public var quantizePlanes = true
        public var planeParameters = PlaneQuantizer.Parameters()
        /// After plane quantization, snap + clip surfels at plane intersections
        /// so wall/floor/ceiling corners come out crisp, not fuzzy.
        public var extractCorners = true
        public var cornerParameters = CornerExtractor.Parameters()
        /// Compute per-surfel ambient occlusion (+ bent normal) each
        /// consolidation — cheap, and it makes the surface read as solid.
        public var ambientOcclusion = true
        public var aoParameters = SurfelMap.AOParameters()
        /// Estimate de-lit `albedo` (capture lighting removed) so the surface
        /// can be relit. Opt-in.
        public var delighting = false
        public var delightParameters = SurfelMap.DelightParameters()
        /// Relax surfels toward an even, non-overlapping tiling each
        /// consolidation — drops redundant surfels, spreads the rest.
        public var areaOptimization = true
        public var compactParameters = SurfelMap.CompactParameters()
        /// Compute per-surfel reconstruction debt (undersampled / grazing /
        /// unresolved) and a scan-completeness figure.
        public var detailDebt = true
        public var debtParameters = SurfelMap.DebtParameters()
        public init() {}
    }

    public struct LiveTuning: Sendable {
        /// Emit a cloud snapshot at most this often (ms of capture time).
        public var snapshotIntervalMs: UInt64 = 400
        public var optimiseEveryNKeyframes = 6
        public var optimiseIterations = 5
        /// Loop-closure check on the newest keyframe every N keyframes (0 = off).
        public var loopClosureEveryNKeyframes = 16
        public var loopParameters = LoopClosureDetector.Parameters()
        public init() {}
    }

    public enum Event: Sendable {
        /// The current surfels (capped for display), for rendering as oriented disks.
        case surfels([Surfel], meanDebt: Float)
        case optimised(keyframes: Int, constraints: Int, initialError: Float, finalError: Float, moved: Int)
        case cleared
    }

    /// Cap on surfels sent in a display event (evenly subsampled past this).
    public var displayLimit = 90_000

    private var config: Configuration
    private let graph = PoseGraph()
    private let map: SurfelMap
    private var keyframes: [Int32: Keyframe] = [:]
    private var order: [Int32] = []
    private var integratedPose: [Int32: simd_float4x4] = [:]
    private var liveCount = 0
    private var lastSnapshotMs: UInt64 = 0
    private var aoTick = 0
    /// Optional structural priors (RoomPlan walls/floor/ceiling). Empty = no change.
    private var planePriors: [PlaneQuantizer.Plane] = []

    private let continuation: AsyncStream<Event>.Continuation
    public nonisolated let events: AsyncStream<Event>

    public init(configuration: Configuration = Configuration()) {
        self.config = configuration
        self.map = SurfelMap(parameters: configuration.surfel)
        (self.events, self.continuation) = AsyncStream.makeStream(bufferingPolicy: .bufferingNewest(8))
    }

    public func ingestLive(_ keyframe: Keyframe, tuning: LiveTuning = LiveTuning()) {
        keyframes[keyframe.id] = keyframe
        graph.addNode(id: keyframe.id, pose: keyframe.arkitPose)
        if let previous = order.last {
            graph.addConstraint(.init(from: previous, to: keyframe.id, measured: keyframe.relativePose, weight: 1))
        }
        order.append(keyframe.id)
        fuse(keyframe, at: keyframe.arkitPose)
        integratedPose[keyframe.id] = keyframe.arkitPose
        liveCount += 1

        var moved = 0
        if tuning.loopClosureEveryNKeyframes > 0, liveCount % tuning.loopClosureEveryNKeyframes == 0 {
            detectLoopClosureForNewest(parameters: tuning.loopParameters)
        }
        if liveCount % tuning.optimiseEveryNKeyframes == 0 {
            moved = tighten(iterations: tuning.optimiseIterations)
        }

        let now = keyframe.capturedAtMs
        if moved > 0 || lastSnapshotMs == 0 || now &- lastSnapshotMs >= tuning.snapshotIntervalMs {
            lastSnapshotMs = now
            aoTick += 1
            // Anti-overlap relaxation — cheap, run most snapshots.
            if config.areaOptimization, aoTick % 2 == 0 {
                map.compact(parameters: config.compactParameters)
            }
            if config.detailDebt { map.updateDebt(parameters: config.debtParameters) }
            if config.quantizePlanes, aoTick % 4 == 0 {
                map.updateClassification(planes: PlaneQuantizer.detect(map.orientedSnapshot(),
                                                                       parameters: config.planeParameters,
                                                                       priors: planePriors))
            }
            // AO is grid-local but O(surfels) — refresh every ~4th snapshot.
            if config.ambientOcclusion, aoTick % 4 == 0 {
                map.updateAmbientOcclusion(parameters: config.aoParameters)
                if config.delighting {
                    map.updateDelighting(parameters: config.delightParameters)
                }
            }
            emitCloud()
        }
    }

    public func snapshot() -> [Surfel] { presentable(map.orientedSnapshot()) }
    public func cloud() -> [PointCloudPoint] { map.snapshot() }
    public var keyframeCount: Int { order.count }

    /// Supply structural priors (e.g. RoomPlan walls). Empty clears them and
    /// restores pure surfel-driven detection.
    public func setPlanePriors(_ planes: [PlaneQuantizer.Plane]) {
        planePriors = planes
    }

    public func setDelighting(_ on: Bool) { config.delighting = on }

    /// The rigid transform from raw ARKit world space to the pose-graph-corrected
    /// world the surfels live in, evaluated at the most recent keyframe. Apply it
    /// to anything expressed in ARKit's frame (e.g. a RoomPlan overlay) to keep
    /// it registered with the map after loop closures.
    public func driftCorrection() -> simd_float4x4 {
        guard let newest = order.last,
              let corrected = graph.pose(id: newest),
              let raw = keyframes[newest]?.arkitPose else { return matrix_identity_float4x4 }
        return corrected * raw.inverse
    }

    /// Detected planes over the current map (walls / floor / ceiling).
    public func planes() -> [PlaneQuantizer.Plane] {
        config.quantizePlanes ? PlaneQuantizer.detect(map.orientedSnapshot(), parameters: config.planeParameters, priors: planePriors) : []
    }

    /// Detected corner edges and vertices over the current map.
    public func corners() -> (edges: [CornerExtractor.Edge], vertices: [CornerExtractor.Vertex]) {
        guard config.quantizePlanes, config.extractCorners else { return ([], []) }
        let snapped = PlaneQuantizer.quantize(map.orientedSnapshot(), parameters: config.planeParameters, priors: planePriors)
        let r = CornerExtractor.extract(surfels: snapped,
                                        planes: PlaneQuantizer.detect(snapped, parameters: config.planeParameters, priors: planePriors),
                                        parameters: config.cornerParameters)
        return (r.edges, r.vertices)
    }

    /// Finalize: bake the surfel map into a watertight vertex-coloured mesh.
    public func bakeMesh(parameters: SurfelMesher.Parameters = SurfelMesher.Parameters()) -> MarchingCubes.Mesh {
        SurfelMesher.mesh(from: presentable(map.orientedSnapshot(minWeight: 1)), parameters: parameters)
    }

    private func presentable(_ surfels: [Surfel]) -> [Surfel] {
        guard config.quantizePlanes else { return surfels }
        let quantized = PlaneQuantizer.quantize(surfels, parameters: config.planeParameters, priors: planePriors)
        guard config.extractCorners else { return quantized }
        let planes = PlaneQuantizer.detect(quantized, parameters: config.planeParameters, priors: planePriors)
        return CornerExtractor.extract(surfels: quantized, planes: planes, parameters: config.cornerParameters).surfels
    }

    public func clear() {
        map.clear()
        keyframes.removeAll()
        order.removeAll()
        integratedPose.removeAll()
        liveCount = 0
        lastSnapshotMs = 0
        continuation.yield(.cleared)
    }

    // MARK: Internals

    private func fuse(_ keyframe: Keyframe, at pose: simd_float4x4) {
        let depth = keyframe.depth
        let sampler = keyframe.colorJPEG.flatMap {
            ColorImageSampler(color: ColorPayload(width: 0, height: 0, jpegData: $0))
        }
        map.integrate(depth: depth.float16Array(),
                      width: depth.width, height: depth.height,
                      depthScale: keyframe.depthScale == 0 ? 1 : keyframe.depthScale,
                      intrinsics: keyframe.depthIntrinsics,
                      pose: pose,
                      confidence: depth.confidenceArray(),
                      keyframeID: keyframe.id,
                      colorFor: sampler.map { s in { x, y in
                          s.color(atX: x * s.width / max(depth.width, 1),
                                  y: y * s.height / max(depth.height, 1))
                      } })
    }

    private func detectLoopClosureForNewest(parameters: LoopClosureDetector.Parameters) {
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

    /// Optimise the graph. If any keyframe shifted, warp the whole surfel map
    /// through a deformation graph — a distance-weighted blend of every
    /// keyframe's correction — so the surface bends smoothly instead of leaving
    /// a seam between keyframes. Returns how many keyframes moved.
    @discardableResult
    private func tighten(iterations: Int) -> Int {
        let (initial, final) = graph.optimize(iterations: iterations)
        let poses = graph.allPoses()

        var nodes: [SurfelMap.DeformNode] = []
        nodes.reserveCapacity(order.count)
        var moved = 0
        for id in order {
            guard let newPose = poses[id], let oldPose = integratedPose[id] else { continue }
            let delta = newPose * oldPose.inverse
            let dt = simd_length(Lie.translation(oldPose) - Lie.translation(newPose))
            let dr = simd_length(Lie.so3Log(Lie.rotation(oldPose).transpose * Lie.rotation(newPose)))
            if max(dt, dr) >= config.correctionThreshold { moved += 1 }
            nodes.append(.init(anchor: Lie.translation(newPose),
                               rotation: Lie.rotation(delta),
                               translation: Lie.translation(delta)))
            integratedPose[id] = newPose
        }
        if moved > 0 { map.applyDeformation(nodes) }

        continuation.yield(.optimised(keyframes: order.count, constraints: graph.constraintCount,
                                      initialError: initial, finalError: final, moved: moved))
        return moved
    }

    private func emitCloud() {
        var all = presentable(map.orientedSnapshot())
        if all.count > displayLimit {
            let stride = Double(all.count) / Double(displayLimit)
            var sampled: [Surfel] = []
            sampled.reserveCapacity(displayLimit)
            var i = 0.0
            while Int(i) < all.count { sampled.append(all[Int(i)]); i += stride }
            all = sampled
        }
        continuation.yield(.surfels(all, meanDebt: map.meanDebt))
    }
}
