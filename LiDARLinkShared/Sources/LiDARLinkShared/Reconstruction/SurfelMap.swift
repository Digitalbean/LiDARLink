import Foundation
import simd

/// An incremental surfel map built from posed depth frames.
///
/// Integrating a frame merges each depth sample into a nearby compatible surfel
/// or spawns a new one — O(samples), no global volume. A pose-graph correction
/// for a keyframe rigidly transforms exactly that keyframe's surfels, so drift
/// gets fixed without re-melting anything.
///
/// Not thread-safe on its own; drive it from a single actor.
public final class SurfelMap {

    public struct Parameters: Sendable {
        /// Spatial-hash cell size (roughly a patch width).
        public var cellSize: Float = 0.05
        /// Merge a patch into a surfel within this many radii.
        public var mergeRadiusFactor: Float = 1.2
        /// Reject a merge when the normals disagree by more than this.
        public var maxNormalAngleDegrees: Float = 30
        /// Weight saturates here so late observations can still nudge a surfel.
        public var maxWeight: Float = 40
        /// Progressive refinement: a well-observed surfel whose incoming samples
        /// keep landing off its plane has real relief at a finer scale, so it
        /// subdivides into four half-size children.
        public var splitMinWeight: Float = 6
        /// Split when the sample-to-plane EWMA exceeds this fraction of the
        /// surfel radius.
        public var splitRoughnessFraction: Float = 0.35
        public var minSurfelRadius: Float = 0.006
        public var maxSplitLevel: UInt8 = 3
        /// Hard cap; the lowest-weight surfels are evicted past this.
        public var maxSurfels: Int = 140_000
        /// Depth → surface-patch extraction (the "superpixel" front end).
        public var extractor = SurfelExtractor.Parameters()
        /// Convenience passthrough so callers can set one confidence gate.
        public var minConfidence: UInt8 {
            get { extractor.minConfidence }
            set { extractor.minConfidence = newValue }
        }
        public init() {}
    }

    private var params: Parameters
    private var surfels: [Surfel] = []
    private var freeList: [Int] = []
    private var grid: [Int64: [Int]] = [:]
    private var byOwner: [Int32: [Int]] = [:]
    private let invCell: Float

    public init(parameters: Parameters = Parameters()) {
        self.params = parameters
        self.invCell = 1 / max(parameters.cellSize, 0.005)
    }

    public var count: Int { surfels.count - freeList.count }

    public func clear() {
        surfels.removeAll(keepingCapacity: true)
        freeList.removeAll(keepingCapacity: true)
        grid.removeAll(keepingCapacity: true)
        byOwner.removeAll(keepingCapacity: true)
    }

    // MARK: Integration

    public func integrate(depth: [Float16],
                          width: Int, height: Int,
                          depthScale: Float,
                          intrinsics: CameraIntrinsics,
                          pose: simd_float4x4,
                          confidence: [UInt8]?,
                          keyframeID: Int32,
                          colorFor: ((Int, Int) -> SIMD3<UInt8>)? = nil) {
        let patches = SurfelExtractor.extract(depth: depth, width: width, height: height,
                                              depthScale: depthScale, intrinsics: intrinsics,
                                              pose: pose, confidence: confidence, keyframeID: keyframeID,
                                              parameters: params.extractor, colorFor: colorFor)
        let cameraPos = SIMD3<Float>(pose.columns.3.x, pose.columns.3.y, pose.columns.3.z)
        for patch in patches {
            mergeOrAdd(position: patch.position, normal: patch.normal, color: patch.color,
                       radius: patch.radius, weight: patch.weight, keyframeID: keyframeID,
                       cameraPos: cameraPos)
        }
    }

    private func mergeOrAdd(position p: SIMD3<Float>, normal n: SIMD3<Float>,
                            color: SIMD3<Float>, radius: Float, weight w: Float, keyframeID: Int32,
                            cameraPos: SIMD3<Float> = .zero) {
        let viewCos: Float = {
            let toCam = cameraPos - p
            let len = simd_length(toCam)
            return len > 1e-4 ? max(0, simd_dot(n, toCam / len)) : 0
        }()
        let cosLimit = cos(params.maxNormalAngleDegrees * .pi / 180)
        let reach = radius * params.mergeRadiusFactor
        let (cx, cy, cz) = cell(p)
        let span = Int64(max(1, Int(ceil(reach * invCell))))   // cover `reach`, not just one cell

        var bestIdx = -1
        var bestDist = Float.greatestFiniteMagnitude
        var bz = cz - span
        while bz <= cz + span {
            var by = cy - span
            while by <= cy + span {
                var bx = cx - span
                while bx <= cx + span {
                    if let bucket = grid[key(bx, by, bz)] {
                        for idx in bucket {
                            let s = surfels[idx]
                            if s.weight < 0 { continue }              // freed slot
                            let dist = simd_length(s.position - p)
                            if dist > max(reach, s.radius * params.mergeRadiusFactor) { continue }
                            if simd_dot(s.normal, n) < cosLimit { continue }
                            // Same scale only — a coarse parent must not swallow a
                            // sample meant to refine a fine child.
                            if max(s.radius, radius) > 2.5 * min(s.radius, radius) { continue }
                            // Prefer the finest matching surfel (detail wins), then nearest.
                            let score = dist + s.radius
                            if score < bestDist { bestDist = score; bestIdx = idx }
                        }
                    }
                    bx += 1
                }
                by += 1
            }
            bz += 1
        }

        if bestIdx >= 0 {
            var s = surfels[bestIdx]
            let oldCell = cell(s.position)
            // Perpendicular offset of this sample from the surfel's current
            // plane — the signal that there's finer structure here.
            let perp = abs(simd_dot(p - s.position, s.normal))
            s.roughness = s.roughness * 0.75 + perp * 0.25
            let nw = s.weight + w
            s.position = (s.position * s.weight + p * w) / nw
            s.normal = simd_normalize(s.normal * s.weight + n * w)
            s.color = (s.color * s.weight + color * w) / nw
            s.radius = min(s.radius, radius)
            s.weight = min(nw, params.maxWeight)
            s.lastSeen = keyframeID
            s.bestViewCos = max(s.bestViewCos, viewCos)
            surfels[bestIdx] = s
            let newCell = cell(s.position)
            if newCell != oldCell {
                removeFromGrid(bestIdx, at: oldCell)
                grid[key(newCell.0, newCell.1, newCell.2), default: []].append(bestIdx)
            }
            considerSplit(bestIdx)
            return
        }

        guard count < params.maxSurfels || evictWeakest() else { return }
        let surfel = Surfel(position: p, normal: n, color: color, radius: radius,
                            weight: w, owner: keyframeID, lastSeen: keyframeID,
                            bestViewCos: viewCos)
        add(surfel, atCell: (cx, cy, cz))
    }

    private func add(_ surfel: Surfel, atCell c: (Int64, Int64, Int64)) {
        let idx: Int
        if let reused = freeList.popLast() {
            idx = reused
            surfels[idx] = surfel
        } else {
            idx = surfels.count
            surfels.append(surfel)
        }
        grid[key(c.0, c.1, c.2), default: []].append(idx)
        byOwner[surfel.owner, default: []].append(idx)
    }

    // MARK: Progressive refinement

    private func considerSplit(_ idx: Int) {
        let s = surfels[idx]
        guard s.weight >= params.splitMinWeight,
              s.splitLevel < params.maxSplitLevel,
              s.radius * 0.5 >= params.minSurfelRadius,
              s.roughness >= params.splitRoughnessFraction * s.radius,
              count + 3 <= params.maxSurfels else { return }

        // Tangent frame on the surfel plane.
        var u = simd_cross(s.normal, SIMD3<Float>(0, 1, 0))
        if simd_length(u) < 1e-4 { u = simd_cross(s.normal, SIMD3<Float>(1, 0, 0)) }
        u = simd_normalize(u)
        let v = simd_normalize(simd_cross(s.normal, u))
        let half = s.radius * 0.5
        let offset = half   // children tile the parent's [-r, r] span

        removeFromGrid(idx, at: cell(s.position))
        removeFromOwner(idx, owner: s.owner)
        surfels[idx].weight = -1
        freeList.append(idx)

        for (du, dv) in [(-1, -1), (1, -1), (-1, 1), (1, 1)] as [(Float, Float)] {
            let pos = s.position + du * offset * u + dv * offset * v
            let child = Surfel(position: pos, normal: s.normal, color: s.color,
                               radius: half, weight: s.weight * 0.4,
                               owner: s.owner, lastSeen: s.lastSeen,
                               roughness: s.roughness * 0.5, splitLevel: s.splitLevel + 1)
            add(child, atCell: cell(pos))
        }
    }

    @discardableResult
    private func evictWeakest() -> Bool {
        var worst = -1
        var worstScore = Float.greatestFiniteMagnitude
        for (i, s) in surfels.enumerated() where s.weight >= 0 {
            let score = s.weight * 1000 + Float(s.lastSeen)   // low weight, long unseen
            if score < worstScore { worstScore = score; worst = i }
        }
        guard worst >= 0 else { return false }
        removeFromGrid(worst, at: cell(surfels[worst].position))
        removeFromOwner(worst, owner: surfels[worst].owner)
        surfels[worst].weight = -1
        freeList.append(worst)
        return true
    }

    private func removeFromOwner(_ idx: Int, owner: Int32) {
        if var list = byOwner[owner] {
            list.removeAll { $0 == idx }
            byOwner[owner] = list.isEmpty ? nil : list
        }
    }

    // MARK: Detail debt

    public struct DebtParameters: Sendable {
        /// Observation weight above which a surfel is "well sampled".
        public var targetWeight: Float = 12
        /// The facing angle a surfel wants at least one observation from
        /// (cos 0.7 ≈ within 45° of head-on).
        public var wantViewCos: Float = 0.7
        public var maxSplitLevel: UInt8 = 3
        /// Blend of (undersampling, grazing-only, unresolved relief).
        public var mix = SIMD3<Float>(0.4, 0.4, 0.2)
        public init() {}
    }

    /// Per-surfel reconstruction debt in 0…1 — where the scan still needs work.
    public func updateDebt(parameters p: DebtParameters = DebtParameters()) {
        for i in 0..<surfels.count {
            var s = surfels[i]
            if s.weight < 0 { continue }
            let under = simd_clamp(1 - s.weight / max(p.targetWeight, 1), 0, 1)
            let graze = simd_clamp((p.wantViewCos - s.bestViewCos) / p.wantViewCos, 0, 1)
            let canRefine: Float = s.splitLevel < p.maxSplitLevel ? 1 : 0.25
            let relief = simd_clamp(s.roughness / max(0.4 * s.radius, 1e-4), 0, 1) * canRefine
            s.debt = simd_clamp(p.mix.x * under + p.mix.y * graze + p.mix.z * relief, 0, 1)
            surfels[i] = s
        }
    }

    /// Tag every surfel with a floor/wall/ceiling/clutter class from the given
    /// planes — for the live "Class" colour mode.
    public func updateClassification(planes: [PlaneQuantizer.Plane]) {
        for i in 0..<surfels.count {
            if surfels[i].weight < 0 { continue }
            surfels[i].classification = PointCloudClassifier.classOf(
                normal: surfels[i].normal, position: surfels[i].position, planes: planes).rawValue
        }
    }

    /// Mean debt across the map — 1 − this is a rough "scan completeness".
    public var meanDebt: Float {
        var sum: Float = 0, n: Float = 0
        for s in surfels where s.weight >= 0 { sum += s.debt; n += 1 }
        return n > 0 ? sum / n : 0
    }

    // MARK: Area optimization (anti-overlap)

    public struct CompactParameters: Sendable {
        /// Two surfels compete for area only if their normals agree within this…
        public var maxNormalAngleDegrees: Float = 22
        /// …and they sit within this of each other's plane (coplanar).
        public var coplanarThickness: Float = 0.02
        /// A surfel whose rim is this covered by neighbours is redundant → removed.
        public var redundancyCoverage: Float = 0.86
        /// Lloyd relaxation step size (0 = off, 1 = full).
        public var relaxation: Float = 0.35
        /// Target centre-to-centre spacing as a multiple of the local radius.
        public var spacingFactor: Float = 1.9
        public init() {}
    }

    /// One relaxation sweep: drop surfels the surface no longer needs, spread the
    /// survivors toward an even tiling, and shrink each radius to just touch its
    /// nearest neighbour. Converges a pile of overlapping disks into a clean
    /// single-layer packing over a few sweeps.
    @discardableResult
    public func compact(parameters p: CompactParameters = CompactParameters()) -> Int {
        let cosLimit = cos(p.maxNormalAngleDegrees * .pi / 180)
        let searchInv = invCell
        var removed = 0

        // Snapshot of live indices — we mutate positions but not the set until the end.
        let live = surfels.indices.filter { surfels[$0].weight >= 0 }

        struct Move { var idx: Int; var newPos: SIMD3<Float>; var newRadius: Float; var drop: Bool }
        var moves: [Move] = []
        moves.reserveCapacity(live.count)

        for i in live {
            let s = surfels[i]
            let ni = s.normal
            // Local same-surface neighbours.
            var neigh: [(pos2: SIMD2<Float>, dist: Float, r: Float, w: Float)] = []
            var u = simd_cross(ni, SIMD3<Float>(0, 1, 0))
            if simd_length(u) < 1e-4 { u = simd_cross(ni, SIMD3<Float>(1, 0, 0)) }
            u = simd_normalize(u)
            let v = simd_normalize(simd_cross(ni, u))

            let reach = max(s.radius * (p.spacingFactor + 0.6), 0.03)
            let span = Int64(max(1, Int(ceil(reach * searchInv))))
            let (cx, cy, cz) = cell(s.position)
            var bz = cz - span
            while bz <= cz + span {
                var by = cy - span
                while by <= cy + span {
                    var bx = cx - span
                    while bx <= cx + span {
                        if let bucket = grid[key(bx, by, bz)] {
                            for j in bucket where j != i {
                                let o = surfels[j]
                                if o.weight < 0 { continue }
                                if simd_dot(o.normal, ni) < cosLimit { continue }
                                let d = o.position - s.position
                                if abs(simd_dot(d, ni)) > p.coplanarThickness { continue }
                                let dist = simd_length(d)
                                if dist > reach || dist < 1e-5 { continue }
                                neigh.append((SIMD2<Float>(simd_dot(d, u), simd_dot(d, v)), dist, o.radius, o.weight))
                            }
                        }
                        bx += 1
                    }
                    by += 1
                }
                bz += 1
            }

            if neigh.isEmpty {
                moves.append(Move(idx: i, newPos: s.position, newRadius: s.radius, drop: false))
                continue
            }

            // Redundancy: is this surfel's rim covered by *stronger* neighbours?
            var covered = 0
            let samples = 8
            for k in 0..<samples {
                let a = Float(k) / Float(samples) * 2 * .pi
                let rim = SIMD2<Float>(cos(a), sin(a)) * s.radius
                for nb in neigh where nb.w >= s.weight * 0.8 {
                    if simd_length(rim - nb.pos2) < nb.r { covered += 1; break }
                }
            }
            if Float(covered) / Float(samples) >= p.redundancyCoverage, neigh.count >= 3 {
                moves.append(Move(idx: i, newPos: s.position, newRadius: s.radius, drop: true))
                continue
            }

            // Lloyd / repulsion: push away from close neighbours, toward gaps.
            let nearest = neigh.map(\.dist).min() ?? s.radius
            var push = SIMD2<Float>.zero
            let spacing = max(s.radius * p.spacingFactor, 0.01)
            for nb in neigh where nb.dist < spacing {
                let dir = simd_length(nb.pos2) > 1e-5 ? -simd_normalize(nb.pos2) : SIMD2<Float>(1, 0)
                push += dir * (1 - nb.dist / spacing)
            }
            let step = push * (p.relaxation * s.radius)
            let newPos = s.position + step.x * u + step.y * v
            let newRadius = simd_clamp(0.5 * nearest, 0.004, s.radius)
            moves.append(Move(idx: i, newPos: newPos, newRadius: newRadius, drop: false))
        }

        // Apply.
        for m in moves {
            if m.drop {
                let s = surfels[m.idx]
                removeFromGrid(m.idx, at: cell(s.position))
                removeFromOwner(m.idx, owner: s.owner)
                surfels[m.idx].weight = -1
                freeList.append(m.idx)
                removed += 1
            } else {
                var s = surfels[m.idx]
                let oldCell = cell(s.position)
                // Keep it on its own plane.
                let drift = simd_dot(m.newPos - s.position, s.normal)
                s.position = m.newPos - drift * s.normal
                s.radius = m.newRadius
                surfels[m.idx] = s
                let newCell = cell(s.position)
                if newCell != oldCell {
                    removeFromGrid(m.idx, at: oldCell)
                    grid[key(newCell.0, newCell.1, newCell.2), default: []].append(m.idx)
                }
            }
        }
        return removed
    }

    // MARK: Pose-graph corrections

    /// Rigidly move every surfel anchored to `keyframe` by `delta` (world-space
    /// `newPose * oldPose⁻¹`).
    public func applyCorrection(keyframe: Int32, delta: simd_float4x4) {
        guard let indices = byOwner[keyframe], !indices.isEmpty else { return }
        let rot = Lie.rotation(delta)
        for idx in indices {
            var s = surfels[idx]
            if s.weight < 0 { continue }
            let oldCell = cell(s.position)
            let moved = delta * SIMD4<Float>(s.position, 1)
            s.position = SIMD3<Float>(moved.x, moved.y, moved.z)
            s.normal = simd_normalize(rot * s.normal)
            surfels[idx] = s
            let newCell = cell(s.position)
            if newCell != oldCell {
                removeFromGrid(idx, at: oldCell)
                grid[key(newCell.0, newCell.1, newCell.2), default: []].append(idx)
            }
        }
    }

    /// One node of the deformation graph: a keyframe's world-space correction
    /// `newPose · oldPose⁻¹`, anchored at its camera position.
    public struct DeformNode: Sendable {
        public var anchor: SIMD3<Float>
        public var rotation: simd_float3x3
        public var translation: SIMD3<Float>
        public init(anchor: SIMD3<Float>, rotation: simd_float3x3, translation: SIMD3<Float>) {
            self.anchor = anchor; self.rotation = rotation; self.translation = translation
        }
    }

    /// Warp the whole map: each surfel is moved by a distance-weighted blend of
    /// its `influence` nearest nodes' corrections, so a loop closure bends the
    /// surface smoothly instead of leaving a seam between keyframes.
    public func applyDeformation(_ nodes: [DeformNode], influence: Int = 4) {
        guard nodes.count >= 2 else {
            if let n = nodes.first {
                let d = pose(rotation: n.rotation, translation: n.translation)
                for idx in 0..<surfels.count where surfels[idx].weight >= 0 {
                    var s = surfels[idx]
                    let p = d * SIMD4<Float>(s.position, 1)
                    s.position = SIMD3<Float>(p.x, p.y, p.z)
                    s.normal = simd_normalize(n.rotation * s.normal)
                    surfels[idx] = s
                }
                rebuildGrid()
            }
            return
        }
        let k = min(influence, nodes.count)
        for idx in 0..<surfels.count {
            var s = surfels[idx]
            if s.weight < 0 { continue }
            var picks: [(d2: Float, j: Int)] = []
            picks.reserveCapacity(k + 1)
            for (j, node) in nodes.enumerated() {
                let d2 = simd_length_squared(node.anchor - s.position)
                if picks.count < k {
                    picks.append((d2, j)); picks.sort { $0.d2 < $1.d2 }
                } else if d2 < picks[k - 1].d2 {
                    picks[k - 1] = (d2, j); picks.sort { $0.d2 < $1.d2 }
                }
            }
            var wsum: Float = 0
            var newPos = SIMD3<Float>.zero
            var rotAccum = simd_float3x3(SIMD3<Float>(0, 0, 0), SIMD3<Float>(0, 0, 0), SIMD3<Float>(0, 0, 0))
            for p in picks {
                let w = 1 / (sqrt(p.d2) + 0.05)
                let node = nodes[p.j]
                newPos += w * (node.rotation * s.position + node.translation)
                rotAccum += w * node.rotation
                wsum += w
            }
            guard wsum > 0 else { continue }
            s.position = newPos / wsum
            let blended = orthonormalized(rotAccum * (1 / wsum))
            s.normal = simd_normalize(blended * s.normal)
            surfels[idx] = s
        }
        rebuildGrid()
    }

    private func pose(rotation: simd_float3x3, translation: SIMD3<Float>) -> simd_float4x4 {
        simd_float4x4(SIMD4<Float>(rotation.columns.0, 0),
                      SIMD4<Float>(rotation.columns.1, 0),
                      SIMD4<Float>(rotation.columns.2, 0),
                      SIMD4<Float>(translation, 1))
    }

    /// Nearest orthonormal matrix (Gram–Schmidt) — a weighted sum of rotations
    /// is not itself a rotation.
    private func orthonormalized(_ m: simd_float3x3) -> simd_float3x3 {
        var x = m.columns.0
        if simd_length(x) < 1e-6 { return matrix_identity_float3x3 }
        x = simd_normalize(x)
        var y = m.columns.1 - simd_dot(m.columns.1, x) * x
        if simd_length(y) < 1e-6 { return matrix_identity_float3x3 }
        y = simd_normalize(y)
        let z = simd_cross(x, y)
        return simd_float3x3(x, y, z)
    }

    private func rebuildGrid() {
        grid.removeAll(keepingCapacity: true)
        for (idx, s) in surfels.enumerated() where s.weight >= 0 {
            let c = cell(s.position)
            grid[key(c.0, c.1, c.2), default: []].append(idx)
        }
    }

    // MARK: Ambient occlusion

    public struct AOParameters: Sendable {
        /// Neighbours within this range occlude.
        public var radius: Float = 0.55
        public var strength: Float = 3.0
        /// Ignore occluders closer than this (self / coplanar).
        public var bias: Float = 0.015
        /// A neighbour must rise at least this far above the surfel's horizon to
        /// occlude — screens out the near-coplanar noise of a flat surface.
        public var minHorizonCos: Float = 0.2
        public init() {}
    }

    /// Per-surfel ambient occlusion + bent normal from the neighbourhood: each
    /// nearby above-horizon surfel subtracts its subtended solid angle from the
    /// open hemisphere. A dedicated coarse grid (cell = AO radius) keeps the
    /// search to 27 cells. No ray marching.
    public func updateAmbientOcclusion(parameters p: AOParameters = AOParameters()) {
        let r2 = p.radius * p.radius
        let inv = 1 / max(p.radius, 0.05)
        func aoKey(_ pos: SIMD3<Float>) -> Int64 {
            key(Int64(floor(pos.x * inv)), Int64(floor(pos.y * inv)), Int64(floor(pos.z * inv)))
        }
        var aoGrid: [Int64: [Int]] = [:]
        aoGrid.reserveCapacity(surfels.count / 8 + 1)
        for (i, s) in surfels.enumerated() where s.weight >= 0 {
            aoGrid[aoKey(s.position), default: []].append(i)
        }

        for i in 0..<surfels.count {
            var s = surfels[i]
            if s.weight < 0 { continue }
            let ni = s.normal
            var occlusion: Float = 0
            var openDir = ni
            let base = SIMD3<Int64>(Int64(floor(s.position.x * inv)),
                                    Int64(floor(s.position.y * inv)),
                                    Int64(floor(s.position.z * inv)))
            var bz = base.z - 1
            cells: while bz <= base.z + 1 {
                var by = base.y - 1
                while by <= base.y + 1 {
                    var bx = base.x - 1
                    while bx <= base.x + 1 {
                        if let bucket = aoGrid[key(bx, by, bz)] {
                            for j in bucket where j != i {
                                let o = surfels[j]
                                let d = o.position - s.position
                                let dist2 = simd_length_squared(d)
                                if dist2 > r2 || dist2 < p.bias * p.bias { continue }
                                let dist = sqrt(dist2)
                                let dir = d / dist
                                let cosT = simd_dot(ni, dir)
                                if cosT < p.minHorizonCos { continue }
                                if simd_dot(o.normal, ni) < -0.5 && dist < 0.05 { continue }
                                let solid = min(.pi * o.radius * o.radius / dist2, .pi)
                                let contrib = (solid / .pi) * cosT * (1 - dist / p.radius)
                                occlusion += contrib
                                openDir -= contrib * dir
                            }
                        }
                        if occlusion >= 1 { break cells }
                        bx += 1
                    }
                    by += 1
                }
                bz += 1
            }
            s.ao = simd_clamp(1 - p.strength * occlusion, 0, 1)
            s.bentNormal = simd_length(openDir) > 1e-4 ? simd_normalize(openDir) : ni
            surfels[i] = s
        }
    }

    // MARK: De-lighting

    public struct DelightParameters: Sendable {
        /// Neighbourhood radius the low-frequency shading is estimated over.
        public var radius: Float = 0.35
        /// How much of the local vs. global luminance ratio to remove (1 = all
        /// low-frequency shading, 0 = none).
        public var strength: Float = 0.8
        /// Clamp the per-surfel shading correction to this range.
        public var minShading: Float = 0.45
        public var maxShading: Float = 2.4
        public init() {}
    }

    /// Estimate `albedo` = colour with the capture lighting removed. The shading
    /// is taken as the low-frequency luminance field (smoothed over `radius`)
    /// plus the AO term; dividing it out flattens hotspots, vignetting and
    /// corner shadow while keeping texture and overall colour level.
    public func updateDelighting(parameters p: DelightParameters = DelightParameters()) {
        func luma(_ c: SIMD3<Float>) -> Float { 0.299 * c.x + 0.587 * c.y + 0.114 * c.z }

        var globalLuma: Float = 0
        var n: Float = 0
        for s in surfels where s.weight >= 0 { globalLuma += luma(s.color); n += 1 }
        guard n > 0, globalLuma > 1 else { return }
        globalLuma /= n

        let r2 = p.radius * p.radius
        let inv = 1 / max(p.radius, 0.05)
        var g: [Int64: [Int]] = [:]
        g.reserveCapacity(surfels.count / 8 + 1)
        for (i, s) in surfels.enumerated() where s.weight >= 0 {
            g[key(Int64(floor(s.position.x * inv)), Int64(floor(s.position.y * inv)), Int64(floor(s.position.z * inv))), default: []].append(i)
        }

        for i in 0..<surfels.count {
            var s = surfels[i]
            if s.weight < 0 { continue }
            var wsum: Float = 0, lsum: Float = 0
            let bx = Int64(floor(s.position.x * inv)), by = Int64(floor(s.position.y * inv)), bz = Int64(floor(s.position.z * inv))
            var z = bz - 1
            while z <= bz + 1 {
                var y = by - 1
                while y <= by + 1 {
                    var x = bx - 1
                    while x <= bx + 1 {
                        if let bucket = g[key(x, y, z)] {
                            for j in bucket {
                                let o = surfels[j]
                                let dist2 = simd_length_squared(o.position - s.position)
                                if dist2 > r2 { continue }
                                let w = 1 - dist2 / r2
                                wsum += w
                                lsum += w * luma(o.color)
                            }
                        }
                        x += 1
                    }
                    y += 1
                }
                z += 1
            }
            let localLuma = wsum > 0 ? lsum / wsum : globalLuma
            // Relative shading: local brightness vs. the whole map, blended
            // toward 1 by (1 - strength), plus a mild AO term.
            var shading = (localLuma / globalLuma) * p.strength + (1 - p.strength)
            shading *= 0.6 + 0.4 * simd_clamp(s.ao, 0, 1)
            shading = simd_clamp(shading, p.minShading, p.maxShading)
            s.albedo = simd_clamp(s.color / shading, SIMD3<Float>(repeating: 0), SIMD3<Float>(repeating: 255))
            surfels[i] = s
        }
    }

    // MARK: Snapshots

    /// Every live surfel centre, coloured — for point/splat display. Colour is
    /// modulated by a soft overhead light against the surfel normal so the cloud
    /// reads as a lit surface (and stays visible in a dim room) instead of flat
    /// dots.
    public func snapshot(minWeight: Float = 1) -> [PointCloudPoint] {
        let light = simd_normalize(SIMD3<Float>(0.25, 1, 0.35))
        var out: [PointCloudPoint] = []
        out.reserveCapacity(count)
        for s in surfels where s.weight >= minWeight {
            let lambert = 0.45 + 0.55 * max(0, simd_dot(s.normal, light))
            let c = simd_clamp(s.color * lambert, SIMD3<Float>(repeating: 0), SIMD3<Float>(repeating: 255))
            out.append(PointCloudPoint(position: s.position,
                                       color: SIMD3<UInt8>(UInt8(c.x), UInt8(c.y), UInt8(c.z)),
                                       confidence: 2))
        }
        return out
    }

    /// Live surfels with orientation and radius — for meshing / oriented export.
    public func orientedSnapshot(minWeight: Float = 1) -> [Surfel] {
        surfels.filter { $0.weight >= minWeight }
    }

    public var averageRadius: Float {
        var sum: Float = 0, n: Float = 0
        for s in surfels where s.weight >= 0 { sum += s.radius; n += 1 }
        return n > 0 ? sum / n : params.cellSize
    }

    // MARK: Grid helpers

    private func cell(_ p: SIMD3<Float>) -> (Int64, Int64, Int64) {
        (Int64(floor(p.x * invCell)), Int64(floor(p.y * invCell)), Int64(floor(p.z * invCell)))
    }

    private func key(_ x: Int64, _ y: Int64, _ z: Int64) -> Int64 {
        let mask: Int64 = 0x1F_FFFF
        return (x & mask) | ((y & mask) << 21) | ((z & mask) << 42)
    }

    private func removeFromGrid(_ idx: Int, at c: (Int64, Int64, Int64)) {
        let k = key(c.0, c.1, c.2)
        if var bucket = grid[k] {
            bucket.removeAll { $0 == idx }
            if bucket.isEmpty { grid[k] = nil } else { grid[k] = bucket }
        }
    }
}
