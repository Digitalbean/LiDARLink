import Foundation
import simd

/// Headless point-cloud fusion engine. Owns the accumulator, the voxel map, and
/// all of the ingest → unproject → dedup → decimate → refresh pipeline. Runs on
/// its own executor (never the main thread) and serialises everything through
/// actor isolation, so there are no locks, generation counters, or
/// cancel-and-replace tasks in the callers.
///
/// Consumers drive it with `ingest`/config calls and react to `outputs`, an
/// `AsyncStream` of display-ready point batches and stats. SceneKit / rendering
/// lives entirely in the client.
public actor ScanEngine {

    // MARK: Configuration

    /// How the live scan is turned into a map.
    public enum MappingMode: String, Sendable, Equatable, CaseIterable {
        /// v1: unproject every frame and pile the points into a voxel-dedup
        /// grid. Fast, but ARKit drift bakes in and there is no real surface.
        case pointCloud
        /// v2: motion-spaced keyframes drive a pose graph + re-integrable TSDF;
        /// drift is corrected live and the output is a fused surface.
        case surface
    }

    public struct Configuration: Sendable, Equatable {
        public var progressiveMapEnabled = true
        public var voxelSizeMeters: Float = 0.01
        public var minConfidence: UInt8 = 1
        public var autoClean = true
        /// Frame-to-model ICP: align each frame onto the accumulated map to soak
        /// up ARKit pose drift before fusion. Needs the progressive map.
        public var poseRefinementEnabled = false
        /// v1 TSDF toggle (open-loop, inflated voxels). Kept for comparison;
        /// `mappingMode == .surface` is the real v2 path.
        public var tsdfEnabled = false
        /// The live mapping pipeline. The Mac app defaults this to `.surface`
        /// (v2); the bare default stays `.pointCloud` for headless/test callers.
        public var mappingMode: MappingMode = .pointCloud
        /// Feed RoomPlan walls/floor/ceiling into the v2 plane quantiser as
        /// priors. Additive — off, the surfel path is unchanged.
        public var useRoomPlanPriors = false
        /// Emit a live shaded mesh (`Output.surfaceMesh`) from the surfel map,
        /// alongside the splat cloud. Display-only; off by default.
        public var liveMeshEnabled = false
        /// Estimate de-lit albedo so the surface can be relit. Opt-in.
        public var delighting = false
        /// v1 dense cloud: run a keyframe pose graph + loop closure alongside it
        /// and warp the accumulated points when the graph corrects drift.
        public var poseGraphEnabled = true
        /// v1 dense cloud: drop flying / mixed pixels at depth discontinuities
        /// before unprojection (`DepthEdgeFilter`). Conservative — only step
        /// edges, never gradients.
        public var edgeFilter = true
        /// v1 dense cloud: carve free space along each frame's depth rays, so
        /// stale floaters, drift ghosts and things that moved away fade out
        /// (`ProgressiveMap.carveFreeSpace`). Needs the progressive map.
        public var freeSpaceCarving = true
        /// Hard cap on accumulated points.
        public var maxPoints = 10_000_000
        /// Points beyond this in the display are added more coarsely.
        public var displayPointLimit = 3_000_000

        public init() {}
    }

    // MARK: Output

    public struct Stats: Sendable, Equatable {
        public var pointCount = 0
        public var occupiedCellCount = 0
        public var lastFrameNewPoints = 0
        public var cloudFull = false
        public var mapSaturated = false
        /// Magnitude of the last frame-to-model pose correction, in mm (0 when
        /// refinement is off or the frame was not corrected).
        public var poseCorrectionMillimetres: Float = 0
        /// v2 surfel map: 1 − mean detail debt. A rough "how done is this scan".
        public var scanCompleteness: Float = 0
        /// v1 pose graph: keyframe count and accepted loop closures.
        public var poseGraphKeyframes = 0
        public var poseGraphLoopClosures = 0
    }

    public enum Output: Sendable {
        /// Append these points to the current display (already decimated).
        case appended([PointCloudPoint])
        /// Replace the entire display with these points. `floorY` is the
        /// estimated floor height in the (possibly re-levelled) frame;
        /// `recenter` asks the client to reframe the camera.
        case replaced([PointCloudPoint], floorY: Float?, recenter: Bool)
        /// The scan was cleared; drop all display geometry.
        case cleared
        case stats(Stats)
        /// The first points of a fresh scan — a hint to recentre the camera.
        case firstPoints([PointCloudPoint])
        /// A reconstructed surface mesh (v1 TSDF mode). `nil` clears it.
        case surfaceMesh(MarchingCubes.Mesh?, recenter: Bool)
        /// The v2 surfel map — oriented disks to render as a surface.
        case surfelCloud([Surfel], recenter: Bool)
    }

    // MARK: State

    private var config: Configuration
    private let accumulator: PointCloudAccumulator
    private var progressiveMap: ProgressiveMap
    private var tsdf: TSDFVolume
    private var pendingFrame: ScanFrame?
    private var processing = false

    private var hasProcessedFrame = false
    private var hasEmittedSurface = false
    private var lastBuildMs: UInt64 = 0
    private var lastFusedRefreshMs: UInt64 = 0
    private var lastCleanupMs: UInt64 = 0
    private var lastCleanupCount = 0
    private var emittedDisplayCount = 0
    private var stats = Stats()

    // v2 surfel mapping
    private var surfelEngine: SurfelEngine?
    private var surfelTask: Task<Void, Never>?
    private var lastKeyframePose: simd_float4x4?
    private var nextKeyframeID: Int32 = 0
    private var lastSurfelCloud: [PointCloudPoint] = []

    // v1 + pose graph: keyframe drift correction over the dense cloud. The
    // accumulator/progressiveMap stay in RAW ARKit space; the correction is a
    // deformation applied at display and export time only.
    private var driftCorrector: LiveDriftCorrector?
    private var driftTask: Task<Void, Never>?
    private var latestDriftNodes: [DeformationGraph.Node] = []

    private var usingTSDF: Bool { config.tsdfEnabled }
    private var usingSurface: Bool { config.mappingMode == .surface }

    private static let decimationThreshold = 8_000_000
    private let continuation: AsyncStream<Output>.Continuation
    public nonisolated let outputs: AsyncStream<Output>

    public init(configuration: Configuration = Configuration()) {
        self.config = configuration
        self.accumulator = PointCloudAccumulator(maxPoints: configuration.maxPoints)
        self.progressiveMap = ProgressiveMap(cellSize: configuration.voxelSizeMeters)
        self.tsdf = TSDFVolume(voxelSize: max(configuration.voxelSizeMeters * 3.5, 0.03))
        (self.outputs, self.continuation) = AsyncStream.makeStream(bufferingPolicy: .bufferingNewest(64))
    }

    // MARK: Ingest

    /// Submits a frame. Latest-wins: if a frame is already being processed, this
    /// one replaces any other frame waiting behind it.
    public func ingest(_ frame: ScanFrame) async {
        pendingFrame = frame
        guard !processing else { return }
        processing = true
        while let next = pendingFrame {
            pendingFrame = nil
            if usingSurface {
                await processFrameSurface(next)
            } else {
                processFrame(next)
            }
            await Task.yield() // let queued ingest() calls refresh pendingFrame
        }
        processing = false
    }

    // MARK: v2 surfel path

    private func processFrameSurface(_ frame: ScanFrame) async {
        guard let depth = frame.depth else { return }
        let now = frame.captureTimestampMs
        // Mac-side keyframe gate — coarser than the phone's motion gate so the
        // pose graph stays small: ~5 cm of travel or ~5° of rotation.
        if let last = lastKeyframePose {
            let dt = simd_length(Lie.translation(frame.pose) - Lie.translation(last))
            let dr = simd_length(Lie.so3Log(Lie.rotation(last).transpose * Lie.rotation(frame.pose)))
            if dt < 0.05 && dr < 0.09 { return }
        }
        let relative = lastKeyframePose.map { $0.inverse * frame.pose } ?? matrix_identity_float4x4
        lastKeyframePose = frame.pose

        let keyframe = Keyframe(id: nextKeyframeID,
                                capturedAtMs: now,
                                depth: depth,
                                depthScale: frame.depthScale == 0 ? 1 : frame.depthScale,
                                depthIntrinsics: frame.intrinsics,
                                colorJPEG: frame.color?.jpegData,
                                colorIntrinsics: frame.color != nil ? frame.intrinsics : nil,
                                arkitPose: frame.pose,
                                relativePose: relative)
        nextKeyframeID += 1
        if nextKeyframeID % 10 == 1 {
            FileLog.append("surfel: keyframe \(nextKeyframeID - 1)", category: "v2map")
        }
        await ensureSurfelEngine().ingestLive(keyframe)
    }

    private func ensureSurfelEngine() -> SurfelEngine {
        if let surfelEngine { return surfelEngine }
        var sc = SurfelEngine.Configuration()
        sc.surfel.cellSize = max(config.voxelSizeMeters * 3, 0.035)
        sc.surfel.minConfidence = config.minConfidence
        sc.delighting = config.delighting
        let engine = SurfelEngine(configuration: sc)
        surfelEngine = engine
        if config.useRoomPlanPriors, let room = pendingRoomStructure {
            let priors = Self.planePriors(from: room)
            Task { await engine.setPlanePriors(priors) }
        }
        surfelTask = Task { [weak self] in
            for await event in engine.events {
                guard let self else { return }
                switch event {
                case .surfels(let surfels, let meanDebt):
                    await self.forwardSurfels(surfels, meanDebt: meanDebt)
                case .optimised(_, _, let initial, let final, let moved):
                    if moved > 0 {
                        FileLog.append("surfel: graph \(String(format: "%.2f", initial))→\(String(format: "%.2f", final)), moved \(moved) kf",
                                       category: "v2map")
                    }
                    await self.maybeEmitLiveMesh()
                case .cleared:
                    break
                }
            }
        }
        return engine
    }

    private var surfelConsolidations = 0
    private var pendingRoomStructure: RoomStructure?

    /// Convert RoomPlan surfaces to plane priors and apply them (when enabled).
    /// RoomPlan geometry is in raw ARKit space, so it is registered to the
    /// pose-graph-corrected surfels before use.
    public func setRoomStructure(_ room: RoomStructure) {
        pendingRoomStructure = room
        guard config.useRoomPlanPriors, let engine = surfelEngine else { return }
        Task {
            let correction = await engine.driftCorrection()
            await engine.setPlanePriors(Self.planePriors(from: room, correction: correction))
        }
    }

    private static func planePriors(from room: RoomStructure,
                                    correction: simd_float4x4 = matrix_identity_float4x4) -> [PlaneQuantizer.Plane] {
        let rot = Lie.rotation(correction)
        let t = Lie.translation(correction)
        return room.surfaces.compactMap { surface in
            switch surface.category {
            case .wall, .floor, .ceiling:
                let p = surface.plane
                let n = simd_normalize(rot * p.normal)
                let offset = p.offset + simd_dot(n, t)          // rigid-transformed plane
                let poly = surface.outline.map { pt -> SIMD3<Float> in
                    let w = correction * SIMD4<Float>(pt, 1)
                    return SIMD3<Float>(w.x, w.y, w.z)
                }
                return PlaneQuantizer.Plane(normal: n, offset: offset,
                                            weight: max(surface.confidence, 0.2) * 120,
                                            polygon: poly)
            default:
                return nil
            }
        }
    }

    private func maybeEmitLiveMesh() async {
        guard config.liveMeshEnabled, let engine = surfelEngine else { return }
        surfelConsolidations += 1
        guard surfelConsolidations % 4 == 0 else { return }
        let mesh = await engine.bakeMesh()
        continuation.yield(.surfaceMesh(mesh.isEmpty ? nil : mesh, recenter: false))
    }

    private func forwardSurfels(_ surfels: [Surfel], meanDebt: Float = 0) {
        stats.pointCount = surfels.count
        stats.occupiedCellCount = surfels.count
        stats.scanCompleteness = 1 - meanDebt
        if !hasEmittedSurface || surfels.count % 5000 < 400 {
            FileLog.append("surfel: \(surfels.count) surfels, \(Int((1 - meanDebt) * 100))% complete", category: "v2map")
        }
        continuation.yield(.stats(stats))
        lastSurfelCloud = surfels.map { PointCloudPoint(position: $0.position, color: $0.colorBytes, confidence: 2) }
        let recenter = !hasEmittedSurface
        hasEmittedSurface = true
        continuation.yield(.surfelCloud(surfels, recenter: recenter))
    }

    private func resetRecon() {
        surfelTask?.cancel()
        surfelTask = nil
        surfelEngine = nil
        driftTask?.cancel()
        driftTask = nil
        driftCorrector = nil
        latestDriftNodes = []
        lastKeyframePose = nil
        nextKeyframeID = 0
        lastSurfelCloud = []
    }

    // MARK: v1 + pose graph

    private func ensureDriftCorrector() -> LiveDriftCorrector {
        if let driftCorrector { return driftCorrector }
        let corrector = LiveDriftCorrector()
        driftCorrector = corrector
        driftTask = Task { [weak self] in
            for await update in corrector.updates {
                guard let self else { return }
                await self.applyDrift(update)
            }
        }
        return corrector
    }

    private func applyDrift(_ update: LiveDriftCorrector.Update) {
        guard !update.nodes.isEmpty else { return }
        FileLog.append("posegraph: \(update.keyframes) kf, \(update.loopClosures) loops, "
                       + "error \(String(format: "%.2f", update.initialError))→\(String(format: "%.2f", update.finalError)), "
                       + "moved \(update.movedKeyframes) kf", category: "v2map")
        // The raw cloud is untouched — just swap in the new correction field and
        // re-emit. New frames keep accumulating raw and are warped on emit.
        latestDriftNodes = update.nodes
        stats.poseGraphKeyframes = update.keyframes
        stats.poseGraphLoopClosures += update.loopClosures
        continuation.yield(.stats(stats))
        // Unfiltered: a lifecycle-filtered replace here made freshly-scanned
        // geometry pop in via .appended and then vanish at the next refresh
        // if it hadn't yet earned .confirmed status — visibly unstable during
        // active scanning. Lifecycle still gates ICP registration targets
        // (ScanEngine's frame-to-model alignment), just not what's on screen.
        emitReplaced(from: config.progressiveMapEnabled ? progressiveMap.mergedPoints() : accumulator.snapshot())
    }

    // MARK: v1 point / TSDF path

    private func processFrame(_ frame: ScanFrame) {
        // Paced by capture time so the rebuild rate is bounded regardless of how
        // fast frames arrive, and so behaviour is deterministic under test.
        let now = frame.captureTimestampMs
        if !hasProcessedFrame {
            hasProcessedFrame = true
            lastCleanupMs = now      // measure delays from scan start
            lastFusedRefreshMs = now
        } else if now &- lastBuildMs < 150 {
            return
        }
        lastBuildMs = now

        if usingTSDF {
            processFrameTSDF(frame, now: now)
            return
        }

        if config.poseGraphEnabled {
            let corrector = ensureDriftCorrector()
            Task { await corrector.consider(frame) }
        }

        let wasEmpty = accumulator.count == 0
        var points = ScanExporter.pointCloud(from: frame, step: 1,
                                             minConfidence: config.minConfidence,
                                             edgeFilter: config.edgeFilter)

        // Carve free space along this frame's rays before merging it in, so
        // stale floaters / drift ghosts / departed objects decay out.
        if config.freeSpaceCarving, config.progressiveMapEnabled, !points.isEmpty {
            progressiveMap.carveFreeSpace(from: Lie.translation(frame.pose), towards: points)
        }

        // Frame-to-model alignment: correct ARKit drift against the fused map
        // before this frame is merged in.
        stats.poseCorrectionMillimetres = 0
        if config.poseRefinementEnabled, config.progressiveMapEnabled,
           progressiveMap.occupiedCellCount > 20_000 {
            // Align against well-attested geometry only — a wall scanned from
            // several angles, not a speck that happens to be nearby.
            let result = PoseRefiner.refine(worldPoints: points, against: progressiveMap, minLifecycle: .confirmed)
            if result.applied {
                let correction = result.correction
                points = points.map { point in
                    let w = correction * SIMD4<Float>(point.position, 1)
                    var moved = point
                    moved.position = SIMD3<Float>(w.x, w.y, w.z)
                    return moved
                }
                let t = correction.columns.3
                stats.poseCorrectionMillimetres = simd_length(SIMD3<Float>(t.x, t.y, t.z)) * 1000
            }
        }

        var toAdd = config.progressiveMapEnabled
            ? progressiveMap.deduplicate(points, viewpoint: Lie.translation(frame.pose))
            : points

        let count = accumulator.count
        if count > Self.decimationThreshold {
            stats.mapSaturated = true
            let step = min(max((count - Self.decimationThreshold) / 500_000 + 2, 2), 16)
            toAdd = Self.strideSample(toAdd, step: step)
        } else {
            stats.mapSaturated = false
        }

        accumulator.add(toAdd)
        stats.lastFrameNewPoints = toAdd.count
        stats.pointCount = accumulator.count
        stats.cloudFull = accumulator.isFull
        stats.occupiedCellCount = progressiveMap.occupiedCellCount
        continuation.yield(.stats(stats))

        if !toAdd.isEmpty {
            let stride = max(1, emittedDisplayCount / config.displayPointLimit + 1)
            var display = stride > 1 ? Self.strideSample(toAdd, step: stride) : toAdd
            // Register the new sweep with the corrected cloud.
            if !latestDriftNodes.isEmpty {
                display = DeformationGraph.warp(points: display, nodes: latestDriftNodes)
            }
            emittedDisplayCount += display.count
            continuation.yield(wasEmpty ? .firstPoints(display) : .appended(display))
        }

        maybeFusedRefresh(now: now)
        maybeCleanup(now: now)
    }

    // MARK: TSDF path

    private func processFrameTSDF(_ frame: ScanFrame, now: UInt64) {
        guard let depth = frame.depth else { return }
        let sampler = frame.color.flatMap { ColorImageSampler(color: $0) }
        let scale = frame.depthScale == 0 ? 1 : frame.depthScale
        tsdf.integrate(depth: depth.float16Array(),
                       width: depth.width,
                       height: depth.height,
                       depthScale: scale,
                       intrinsics: frame.intrinsics,
                       pose: frame.pose,
                       confidence: depth.confidenceArray(),
                       minConfidence: config.minConfidence,
                       colorFor: sampler.map { s in { x, y in
                           s.color(atX: x * s.width / max(depth.width, 1),
                                   y: y * s.height / max(depth.height, 1))
                       } })

        stats.occupiedCellCount = tsdf.occupiedBlockCount
        continuation.yield(.stats(stats))
        maybeTSDFRefresh(now: now, force: false)
    }

    private func maybeTSDFRefresh(now: UInt64, force: Bool) {
        guard force || now &- lastFusedRefreshMs >= 1_500 else { return }
        lastFusedRefreshMs = now
        let mesh = tsdf.mesh()
        guard !mesh.isEmpty || hasEmittedSurface else { return }
        stats.pointCount = mesh.vertices.count
        continuation.yield(.stats(stats))
        let recenter = !hasEmittedSurface
        hasEmittedSurface = true
        continuation.yield(.surfaceMesh(mesh.isEmpty ? nil : mesh, recenter: recenter))
    }

    // MARK: Periodic work

    private func maybeFusedRefresh(now: UInt64) {
        guard config.progressiveMapEnabled, now &- lastFusedRefreshMs >= 1_000 else { return }
        lastFusedRefreshMs = now
        emitReplaced(from: progressiveMap.mergedPoints())
    }

    private func maybeCleanup(now: UInt64) {
        let count = accumulator.count
        guard config.autoClean,
              now &- lastCleanupMs > 30_000,
              count > 100_000,
              count - lastCleanupCount > max(count / 10, 50_000) else { return }
        lastCleanupMs = now
        lastCleanupCount = count
        let source = config.progressiveMapEnabled ? progressiveMap.mergedPoints() : accumulator.snapshot()
        let cleaned = PointCloudCleaner.radiusOutlierRemoval(source)
        guard cleaned.count < source.count else { return }
        accumulator.replace(with: cleaned)
        progressiveMap.rebuild(from: cleaned)
        emitReplaced(from: cleaned)
    }

    // MARK: Configuration changes

    public func updateConfiguration(_ new: Configuration) {
        let voxelChanged = new.voxelSizeMeters != config.voxelSizeMeters
        let mapToggled = new.progressiveMapEnabled != config.progressiveMapEnabled
        let tsdfToggled = new.tsdfEnabled != config.tsdfEnabled
        let modeChanged = new.mappingMode != config.mappingMode
        let delightToggled = new.delighting != config.delighting
        config = new
        if delightToggled, let engine = surfelEngine {
            Task { await engine.setDelighting(new.delighting) }
        }

        if modeChanged {
            // Switching mapping pipeline: tear both down and let the scan rebuild.
            resetRecon()
            tsdf.clear()
            accumulator.clear()
            progressiveMap.clear()
            hasEmittedSurface = false
            emittedDisplayCount = 0
            lastFusedRefreshMs = 0
            stats = Stats()
            continuation.yield(.surfaceMesh(nil, recenter: false))
            emitReplaced(from: [], recenter: false)
            continuation.yield(.stats(stats))
            return
        }

        if tsdfToggled || (new.tsdfEnabled && voxelChanged) {
            // TSDF can't be re-seeded from points; it needs depth frames. Reset
            // and let the scan rebuild it. Switching away just shows the cloud.
            tsdf = TSDFVolume(voxelSize: max(new.voxelSizeMeters * 3.5, 0.03))
            hasEmittedSurface = false
            lastFusedRefreshMs = 0
            if new.tsdfEnabled {
                stats.pointCount = 0
                emitReplaced(from: [], recenter: false)   // drop the point cloud
            } else {
                continuation.yield(.surfaceMesh(nil, recenter: false))   // drop the TSDF mesh
                emitReplaced(from: config.progressiveMapEnabled ? progressiveMap.mergedPoints() : accumulator.snapshot(), recenter: true)
            }
            return
        }

        guard voxelChanged || mapToggled else { return }
        if new.progressiveMapEnabled {
            let seed = accumulator.snapshot()
            progressiveMap = ProgressiveMap(cellSize: new.voxelSizeMeters)
            progressiveMap.rebuild(from: seed)
            emitReplaced(from: progressiveMap.mergedPoints())
        } else {
            emitReplaced(from: accumulator.snapshot())
        }
    }

    // MARK: Lifecycle

    public func clear() {
        accumulator.clear()
        progressiveMap.clear()
        tsdf.clear()
        resetRecon()
        emittedDisplayCount = 0
        hasProcessedFrame = false
        hasEmittedSurface = false
        lastBuildMs = 0
        lastFusedRefreshMs = 0
        lastCleanupMs = 0
        lastCleanupCount = 0
        stats = Stats()
        continuation.yield(.cleared)
        continuation.yield(.surfaceMesh(nil, recenter: false))
        continuation.yield(.stats(stats))
    }

    /// Replaces the cloud (recording playback). Rebuilds the voxel map from it.
    public func loadPoints(_ points: [PointCloudPoint]) {
        accumulator.replace(with: points)
        progressiveMap.rebuild(from: points)
        emittedDisplayCount = 0
        stats.pointCount = points.count
        stats.occupiedCellCount = progressiveMap.occupiedCellCount
        emitReplaced(from: config.progressiveMapEnabled ? progressiveMap.mergedPoints() : points, recenter: true)
        continuation.yield(.stats(stats))
    }

    /// Re-integrates a recording through TSDF fusion (playback in TSDF mode).
    public func loadFrames(_ frames: [ScanFrame]) {
        tsdf.clear()
        hasEmittedSurface = false
        for frame in frames {
            guard let depth = frame.depth else { continue }
            let sampler = frame.color.flatMap { ColorImageSampler(color: $0) }
            let scale = frame.depthScale == 0 ? 1 : frame.depthScale
            tsdf.integrate(depth: depth.float16Array(), width: depth.width, height: depth.height,
                           depthScale: scale, intrinsics: frame.intrinsics, pose: frame.pose,
                           confidence: depth.confidenceArray(), minConfidence: config.minConfidence,
                           colorFor: sampler.map { s in { x, y in
                               s.color(atX: x * s.width / max(depth.width, 1),
                                       y: y * s.height / max(depth.height, 1))
                           } })
        }
        let mesh = tsdf.mesh()
        stats.pointCount = mesh.vertices.count
        stats.occupiedCellCount = tsdf.occupiedBlockCount
        hasEmittedSurface = true
        continuation.yield(.surfaceMesh(mesh.isEmpty ? nil : mesh, recenter: true))
        continuation.yield(.stats(stats))
    }

    /// Levels the cloud to its floor plane. Returns the transform applied (so the
    /// caller can apply it to meshes too), or nil if no floor was found.
    public func levelToFloor() -> simd_float4x4? {
        // TSDF re-levelling means re-integrating from corrected poses (phase D).
        // For now level the extracted surface for display/export only.
        if usingTSDF {
            let surface = tsdf.surfacePoints()
            guard let transform = GroundPlane.levelingTransform(for: surface) else { return nil }
            emitReplaced(from: GroundPlane.apply(transform, to: surface))
            return transform
        }
        let source = config.progressiveMapEnabled ? progressiveMap.mergedPoints() : accumulator.snapshot()
        guard let transform = GroundPlane.levelingTransform(for: source) else { return nil }
        let leveled = GroundPlane.apply(transform, to: accumulator.snapshot())
        accumulator.replace(with: leveled)
        progressiveMap.rebuild(from: config.progressiveMapEnabled
                               ? GroundPlane.apply(transform, to: progressiveMap.mergedPoints())
                               : leveled)
        emitReplaced(from: config.progressiveMapEnabled ? progressiveMap.mergedPoints() : leveled)
        return transform
    }

    /// Forces a full `.replaced` emission (e.g. the client changed how it colours
    /// points and needs to re-render everything).
    public func refresh() {
        if usingSurface { return }   // the surface path re-emits its mesh on its own cadence
        emitReplaced(from: pointSnapshot())
    }

    /// Snapshot of the fused cloud (for export / recentre) — drift-corrected.
    public func pointSnapshot() -> [PointCloudPoint] {
        if usingSurface { return lastSurfelCloud }
        if usingTSDF { return tsdf.surfacePoints() }
        let raw = config.progressiveMapEnabled ? progressiveMap.mergedPoints() : accumulator.snapshot()
        return latestDriftNodes.isEmpty ? raw : DeformationGraph.warp(points: raw, nodes: latestDriftNodes)
    }

    /// The current v2 surfel map (oriented, for meshing / export), or empty in
    /// point-cloud mode.
    public func surfelSnapshot() async -> [Surfel] {
        guard usingSurface, let surfelEngine else { return [] }
        return await surfelEngine.snapshot()
    }

    /// ARKit-frame → corrected map-frame rigid transform (for registering a
    /// RoomPlan overlay). Works in both v1 (pose graph) and v2 modes.
    public func driftCorrection() async -> simd_float4x4 {
        if usingSurface, let surfelEngine { return await surfelEngine.driftCorrection() }
        if let driftCorrector { return await driftCorrector.driftCorrection() }
        return matrix_identity_float4x4
    }

    /// A classified, oriented point cloud (for LAS / enhanced-PLY export).
    /// v2: the surfel map. v1: the dense cloud with normals estimated on the fly.
    public func classifiedPointCloud() async -> RichPointCloud? {
        if !usingSurface { return classifiedDenseCloud() }
        guard let surfelEngine else { return nil }
        let surfels = await surfelEngine.snapshot()
        guard !surfels.isEmpty else { return nil }
        let planes = await surfelEngine.planes()
        var room: RoomStructure? = pendingRoomStructure
        if room != nil {
            let correction = await surfelEngine.driftCorrection()
            room = pendingRoomStructure.map { r in
                var out = r
                out.surfaces = r.surfaces.map {
                    var s = $0
                    s.transform = correction * $0.transform
                    return s
                }
                out.objects = r.objects.map {
                    var o = $0
                    o.transform = correction * $0.transform
                    return o
                }
                return out
            }
        }
        return PointCloudClassifier.classify(surfels: surfels, planes: planes, room: room)
    }

    /// v1: voxel-downsample the dense cloud, estimate normals, classify from
    /// detected planes. Runs off the caller's task — a few seconds for a room.
    private func classifiedDenseCloud() -> RichPointCloud? {
        let dense = pointSnapshot()   // already drift-corrected
        guard dense.count > 200 else { return nil }
        // Downsample to ~2 cm so normal estimation is tractable.
        let grid = ProgressiveMap(cellSize: 0.02)
        let sampled = grid.deduplicate(dense)
        let index = PointIndex(cellSize: 0.08)
        index.rebuild(sampled)
        let normals = PointNormalEstimator.normals(indices: Array(sampled.indices), in: index,
                                                   points: sampled, radius: 0.08, minNeighbors: 6)
        var pseudo: [Surfel] = []
        pseudo.reserveCapacity(sampled.count)
        for (i, pt) in sampled.enumerated() {
            guard let n = normals[i] else { continue }
            pseudo.append(Surfel(position: pt.position, normal: n,
                                 color: SIMD3<Float>(Float(pt.color.x), Float(pt.color.y), Float(pt.color.z)),
                                 radius: 0.02, weight: 10, owner: 0, lastSeen: 0, bestViewCos: 0.8))
        }
        guard !pseudo.isEmpty else { return nil }
        let planes = PlaneQuantizer.detect(pseudo)
        var room = pendingRoomStructure
        // (drift correction for the room is folded in by applyDrift already
        //  moving the cloud; the RoomPlan boxes stay in ARKit frame here — a
        //  small residual, acceptable for classification.)
        _ = room
        room = nil
        return PointCloudClassifier.classify(surfels: pseudo, planes: planes, room: room)
    }

    /// Finalize the v2 map into a watertight vertex-coloured mesh, or nil in
    /// point-cloud mode / when the map is empty.
    public func bakeSurfaceMesh() async -> MarchingCubes.Mesh? {
        guard usingSurface, let surfelEngine else { return nil }
        let mesh = await surfelEngine.bakeMesh()
        return mesh.isEmpty ? nil : mesh
    }

    // MARK: Helpers

    private func emitReplaced(from source: [PointCloudPoint], recenter: Bool = false) {
        // When the pose graph is correcting, the display cloud is warped (all
        // nodes, per point) on every emit — cap it so that stays well under a
        // frame.
        let limit = latestDriftNodes.isEmpty ? config.displayPointLimit : min(config.displayPointLimit, 1_500_000)
        var display = PointCloudSampler.evenlySample(source, limit: limit)
        if !latestDriftNodes.isEmpty {
            display = DeformationGraph.warp(points: display, nodes: latestDriftNodes)
        }
        emittedDisplayCount = display.count
        let floorY = ViewNavigationMath.estimatedFloorY(of: display)
        continuation.yield(.replaced(display, floorY: floorY, recenter: recenter))
    }

    private static func strideSample(_ points: [PointCloudPoint], step: Int) -> [PointCloudPoint] {
        guard step > 1 else { return points }
        var result: [PointCloudPoint] = []
        result.reserveCapacity(points.count / step + 1)
        var i = 0
        while i < points.count {
            result.append(points[i])
            i += step
        }
        return result
    }
}
