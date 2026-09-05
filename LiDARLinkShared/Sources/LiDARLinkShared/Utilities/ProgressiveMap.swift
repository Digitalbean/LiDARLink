import Foundation
import simd

/// Persistent world-space voxel map. New observations are merged into occupied
/// cells with a weighted average (position and color) instead of being dropped,
/// so pose drift collapses into single averaged surfaces rather than ghosting.
/// Thread-safe.
public final class ProgressiveMap: @unchecked Sendable {
    public let cellSize: Float
    private let lock = NSLock()

    private struct Cell {
        var positionSum: SIMD3<Float>
        var colorSum: SIMD3<Float>
        var weight: Float
        var confidenceSum: Float
        /// One bit per 45° azimuthal sector the cell has been observed from
        /// (bit 0 = camera due +X of the point, going counter-clockwise around
        /// +Y). Ten observations from a tripod set the same bit ten times;
        /// four observations while walking around set four different ones —
        /// that distinction is what `GeometryLifecycle` promotion cares about.
        var viewDirMask: UInt16 = 0
    }

    /// How well-attested a piece of geometry is — repeat count alone isn't
    /// enough (see `viewDirMask`): a surface only ever glimpsed from one
    /// direction shouldn't outrank one seen briefly from several.
    public enum GeometryLifecycle: Int, Sendable, Comparable, CaseIterable {
        case candidate = 0
        case confirmed = 1
        case established = 2
        public static func < (lhs: GeometryLifecycle, rhs: GeometryLifecycle) -> Bool { lhs.rawValue < rhs.rawValue }
    }

    public struct LifecycleParameters: Sendable {
        /// Minimum weight *and* minimum distinct view directions to leave
        /// `.candidate` (one or two hits, or many hits from a single vantage).
        public var confirmedWeight: Float = 6
        public var confirmedMinDirections: Int = 2
        /// Matches `CarveParameters.establishedWeight` — the same bar carving
        /// already treats as "too well-attested to erode".
        public var establishedWeight: Float = 24
        public init() {}
    }

    private static func lifecycle(weight: Float, directionCount: Int, parameters: LifecycleParameters) -> GeometryLifecycle {
        if weight > parameters.establishedWeight { return .established }
        if weight >= parameters.confirmedWeight, directionCount >= parameters.confirmedMinDirections { return .confirmed }
        return .candidate
    }

    /// Buckets the direction from `point` to `viewpoint`, projected onto the
    /// horizontal plane, into one of 8 sectors. Looking straight up/down
    /// (no horizontal component) falls back to sector 0 rather than being lost.
    private static func directionBit(from point: SIMD3<Float>, to viewpoint: SIMD3<Float>) -> UInt16 {
        let d = viewpoint - point
        let horizontal = SIMD2<Float>(d.x, d.z)
        guard simd_length(horizontal) > 1e-4 else { return 1 }
        let angle = atan2(horizontal.y, horizontal.x)                 // -pi...pi
        let normalized = (angle + .pi) / (2 * .pi)                    // 0..<1
        let bin = min(7, Int(normalized * 8))
        return 1 << UInt16(bin)
    }

    private var cells: [UInt64: Cell] = [:]
    public private(set) var addedPointCount = 0
    public private(set) var skippedPointCount = 0
    public private(set) var carvedCellCount = 0

    public init(cellSize: Float = 0.01) {
        self.cellSize = max(cellSize, 0.0001)
    }

    /// Returns one point per newly-occupied cell. Observations for occupied
    /// cells are merged into the stored cell (weighted average). `viewpoint`,
    /// when given, is the camera position this batch was observed from — it
    /// feeds `viewDirMask` for lifecycle promotion; omit it (e.g. re-seeding
    /// from an already-fused cloud) to leave diversity tracking untouched.
    public func deduplicate(_ points: [PointCloudPoint], viewpoint: SIMD3<Float>? = nil) -> [PointCloudPoint] {
        lock.lock()
        defer { lock.unlock() }
        guard !points.isEmpty else { return [] }
        let inv = 1 / cellSize
        var result: [PointCloudPoint] = []
        result.reserveCapacity(points.count)
        for point in points {
            let key = Self.packCell(position: point.position, invCellSize: inv)
            let observationWeight = Self.weight(for: point.confidence)
            let dirBit = viewpoint.map { Self.directionBit(from: point.position, to: $0) } ?? 0
            if var cell = cells[key] {
                cell.positionSum += point.position * observationWeight
                cell.colorSum += SIMD3<Float>(Float(point.color.x), Float(point.color.y), Float(point.color.z)) * observationWeight
                cell.weight += observationWeight
                cell.confidenceSum += Float(point.confidence) * observationWeight
                cell.viewDirMask |= dirBit
                cells[key] = cell
                skippedPointCount += 1
            } else {
                cells[key] = Cell(positionSum: point.position * observationWeight,
                                  colorSum: SIMD3<Float>(Float(point.color.x), Float(point.color.y), Float(point.color.z)) * observationWeight,
                                  weight: observationWeight,
                                  confidenceSum: Float(point.confidence) * observationWeight,
                                  viewDirMask: dirBit)
                addedPointCount += 1
                result.append(point)
            }
        }
        return result
    }

    /// The current map as merged voxel centroids (one point per occupied cell).
    /// `minLifecycle` (default `.candidate`, i.e. everything) drops cells that
    /// haven't earned enough weight *and* viewpoint diversity yet — useful for
    /// a live display that shouldn't show every single-hit speck as if it were
    /// as real as a wall scanned from all sides. Never used to drop data
    /// permanently: cells stay in the map regardless and can still surface once
    /// they're promoted.
    public func mergedPoints(minLifecycle: GeometryLifecycle = .candidate,
                             lifecycleParameters: LifecycleParameters = LifecycleParameters()) -> [PointCloudPoint] {
        lock.lock()
        defer { lock.unlock() }
        var result: [PointCloudPoint] = []
        result.reserveCapacity(cells.count)
        for cell in cells.values where cell.weight > 0 {
            if minLifecycle > .candidate {
                let stage = Self.lifecycle(weight: cell.weight, directionCount: cell.viewDirMask.nonzeroBitCount,
                                           parameters: lifecycleParameters)
                guard stage >= minLifecycle else { continue }
            }
            let position = cell.positionSum / cell.weight
            let color = cell.colorSum / cell.weight
            let confidence = UInt8(min(max((cell.confidenceSum / cell.weight).rounded(), 0), 2))
            result.append(PointCloudPoint(position: position,
                                          color: SIMD3<UInt8>(UInt8(min(max(color.x, 0), 255)),
                                                              UInt8(min(max(color.y, 0), 255)),
                                                              UInt8(min(max(color.z, 0), 255))),
                                          confidence: confidence))
        }
        return result
    }

    /// The `GeometryLifecycle` of the cell at `position`, or `nil` if unoccupied.
    public func lifecycle(at position: SIMD3<Float>, parameters: LifecycleParameters = LifecycleParameters()) -> GeometryLifecycle? {
        lock.lock()
        defer { lock.unlock() }
        let key = Self.packCell(position: position, invCellSize: 1 / cellSize)
        guard let cell = cells[key], cell.weight > 0 else { return nil }
        return Self.lifecycle(weight: cell.weight, directionCount: cell.viewDirMask.nonzeroBitCount, parameters: parameters)
    }

    public struct CarveParameters: Sendable {
        /// Stop the ray this far short of the measured surface — never carve the
        /// surface itself or the cell just in front of it.
        public var margin: Float = 0.10
        /// Multiplier applied to a carved cell's weight per pass. A real cell
        /// wrongly carved recovers when it is re-observed next frame; a genuine
        /// floater keeps decaying.
        public var decay: Float = 0.55
        /// A cell at or below this weight after carving is removed.
        public var evictionWeight: Float = 0.4
        /// Rays longer than this (camera to surface) are ignored — far depth is
        /// too noisy to carve confidently.
        public var maxRange: Float = 4.5
        /// Only carve along rays whose observation is at least this confident.
        public var minConfidence: UInt8 = 1
        /// Use at most this many rays per call (observations are strided down).
        public var maxRays: Int = 5000
        /// A cell with more accumulated weight than this is established geometry
        /// — a stray see-through ray (e.g. a real thin wall glimpsed edge-on
        /// through a doorway) must not erode it. Floaters never reach this.
        public var establishedWeight: Float = 24
        public init() {}
    }

    /// Free-space carving. Any occupied cell lying between `origin` and an
    /// observed surface point — and clearly in front of it — is being seen
    /// through, so it is not really there: its weight decays and it is evicted
    /// once too weak. Removes flying pixels behind depth edges, ghost points
    /// from pose drift, and geometry from something that has since moved away
    /// (a person, a shifted blanket). Conservative: only cells in front of a
    /// confident measurement, by more than `margin`, are ever touched.
    @discardableResult
    public func carveFreeSpace(from origin: SIMD3<Float>,
                               towards observations: [PointCloudPoint],
                               parameters: CarveParameters = CarveParameters()) -> Int {
        lock.lock()
        defer { lock.unlock() }
        guard cells.count > 8, !observations.isEmpty else { return 0 }
        let inv = 1 / cellSize
        let step = max(1, observations.count / parameters.maxRays)
        var carved = 0
        // A cell is decayed at most once per call — one frame is one piece of
        // "seen through" evidence no matter how many rays graze the voxel.
        var visited = Set<UInt64>()

        var oi = 0
        while oi < observations.count {
            let obs = observations[oi]
            oi += step
            guard obs.confidence >= parameters.minConfidence else { continue }
            let toSurface = obs.position - origin
            let range = simd_length(toSurface)
            guard range > cellSize, range <= parameters.maxRange else { continue }
            let dir = toSurface / range
            let stopAt = range - parameters.margin
            guard stopAt > cellSize else { continue }

            // Amanatides–Woo voxel traversal from just outside the camera cell.
            var t = cellSize                       // skip the camera's own cell
            var voxel = SIMD3<Int32>(Int32(floor((origin.x + dir.x * t) * inv)),
                                     Int32(floor((origin.y + dir.y * t) * inv)),
                                     Int32(floor((origin.z + dir.z * t) * inv)))
            let stepDir = SIMD3<Int32>(dir.x >= 0 ? 1 : -1, dir.y >= 0 ? 1 : -1, dir.z >= 0 ? 1 : -1)
            func tMax(_ axis: Int) -> Float {
                let o = origin[axis], d = dir[axis]
                if abs(d) < 1e-9 { return .greatestFiniteMagnitude }
                let next = (Float(voxel[axis]) + (d >= 0 ? 1 : 0)) * cellSize
                return (next - o) / d
            }
            var next = SIMD3<Float>(tMax(0), tMax(1), tMax(2))
            let delta = SIMD3<Float>(abs(dir.x) < 1e-9 ? .greatestFiniteMagnitude : cellSize / abs(dir.x),
                                     abs(dir.y) < 1e-9 ? .greatestFiniteMagnitude : cellSize / abs(dir.y),
                                     abs(dir.z) < 1e-9 ? .greatestFiniteMagnitude : cellSize / abs(dir.z))

            var guardSteps = 0
            while t < stopAt && guardSteps < 512 {
                guardSteps += 1
                let key = Self.packCell(ix: voxel.x, iy: voxel.y, iz: voxel.z)
                if var cell = cells[key], cell.weight > 0, cell.weight <= parameters.establishedWeight {
                    // Don't carve a cell whose own centroid sits at/behind the
                    // surface point (it belongs to that surface, seen obliquely).
                    let centroidRange = simd_length(cell.positionSum / cell.weight - origin)
                    if centroidRange < stopAt, !visited.contains(key) {
                        visited.insert(key)
                        // Scale every accumulator by the same factor so the
                        // centroid (positionSum / weight) is preserved — only
                        // the confidence in the cell drops.
                        cell.weight *= parameters.decay
                        cell.positionSum *= parameters.decay
                        cell.colorSum *= parameters.decay
                        cell.confidenceSum *= parameters.decay
                        if cell.weight <= parameters.evictionWeight {
                            cells[key] = nil
                            carved += 1
                        } else {
                            cells[key] = cell
                        }
                    }
                }
                if next.x < next.y && next.x < next.z {
                    voxel.x += stepDir.x; t = next.x; next.x += delta.x
                } else if next.y < next.z {
                    voxel.y += stepDir.y; t = next.y; next.y += delta.y
                } else {
                    voxel.z += stepDir.z; t = next.z; next.z += delta.z
                }
            }
        }
        carvedCellCount += carved
        return carved
    }

    /// Re-seeds occupancy from an existing cloud (e.g., after cleanup).
    public func rebuild(from points: [PointCloudPoint]) {
        lock.lock()
        defer { lock.unlock() }
        cells.removeAll(keepingCapacity: true)
        addedPointCount = 0
        skippedPointCount = 0
        carvedCellCount = 0
        let inv = 1 / cellSize
        for point in points {
            let key = Self.packCell(position: point.position, invCellSize: inv)
            if cells[key] == nil {
                let observationWeight = Self.weight(for: point.confidence)
                cells[key] = Cell(positionSum: point.position * observationWeight,
                                  colorSum: SIMD3<Float>(Float(point.color.x), Float(point.color.y), Float(point.color.z)) * observationWeight,
                                  weight: observationWeight,
                                  confidenceSum: Float(point.confidence) * observationWeight)
                addedPointCount += 1
            }
        }
    }

    public var occupiedCellCount: Int {
        lock.lock()
        defer { lock.unlock() }
        return cells.count
    }

    /// A local planar patch of the map: a point on the surface and its unit
    /// normal, from PCA over the occupied centroids around a query.
    public struct Surfel: Sendable, Equatable {
        public var point: SIMD3<Float>
        public var normal: SIMD3<Float>
    }

    /// For each query position, the local surface patch of the map near it —
    /// searching the containing cell and its 26 neighbours, one lock for the
    /// whole batch. `nil` where too few centroids are in range to fit a plane.
    /// For point-to-plane frame-to-model alignment.
    ///
    /// `minLifecycle` (default `.candidate`, i.e. unfiltered — the historical
    /// behaviour) restricts correspondence targets to well-attested geometry:
    /// registration should align against a wall scanned from all sides, not a
    /// speck that happened to land nearby. Pass `.candidate` for a map that
    /// was never seeded with viewpoints (nothing on it can ever be `.confirmed`
    /// — see `deduplicate(_:viewpoint:)`), such as `LoopClosureDetector`'s
    /// throwaway per-pair target maps.
    public func nearestSurfels(to positions: [SIMD3<Float>], maxRadius: Float,
                               minLifecycle: GeometryLifecycle = .candidate,
                               lifecycleParameters: LifecycleParameters = LifecycleParameters()) -> [Surfel?] {
        lock.lock()
        defer { lock.unlock() }
        guard cells.count >= 4 else { return Array(repeating: nil, count: positions.count) }
        let inv = 1 / cellSize
        let maxDistanceSquared = maxRadius * maxRadius
        return positions.map { position in
            let ix = Int32(floor(position.x * inv))
            let iy = Int32(floor(position.y * inv))
            let iz = Int32(floor(position.z * inv))
            var neighbours: [SIMD3<Float>] = []
            neighbours.reserveCapacity(27)
            var nearest: SIMD3<Float>?
            var nearestDistanceSquared = maxDistanceSquared
            for dz in Int32(-1)...1 {
                for dy in Int32(-1)...1 {
                    for dx in Int32(-1)...1 {
                        guard let cell = cells[Self.packCell(ix: ix + dx, iy: iy + dy, iz: iz + dz)],
                              cell.weight > 0 else { continue }
                        if minLifecycle > .candidate {
                            let stage = Self.lifecycle(weight: cell.weight, directionCount: cell.viewDirMask.nonzeroBitCount,
                                                       parameters: lifecycleParameters)
                            guard stage >= minLifecycle else { continue }
                        }
                        let centroid = cell.positionSum / cell.weight
                        neighbours.append(centroid)
                        let d = simd_distance_squared(centroid, position)
                        if d < nearestDistanceSquared {
                            nearestDistanceSquared = d
                            nearest = centroid
                        }
                    }
                }
            }
            guard let anchor = nearest, neighbours.count >= 4 else { return nil }

            var mean = SIMD3<Float>(0, 0, 0)
            for n in neighbours { mean += n }
            mean /= Float(neighbours.count)
            var cxx: Float = 0, cxy: Float = 0, cxz: Float = 0, cyy: Float = 0, cyz: Float = 0, czz: Float = 0
            for n in neighbours {
                let d = n - mean
                cxx += d.x * d.x; cxy += d.x * d.y; cxz += d.x * d.z
                cyy += d.y * d.y; cyz += d.y * d.z; czz += d.z * d.z
            }
            let covariance = simd_float3x3(SIMD3<Float>(cxx, cxy, cxz),
                                          SIMD3<Float>(cxy, cyy, cyz),
                                          SIMD3<Float>(cxz, cyz, czz))
            guard let axis = PointNormalEstimator.smallestEigenvector(of: covariance) else { return nil }
            let normal = simd_normalize(axis)
            guard normal.x.isFinite else { return nil }
            return Surfel(point: anchor, normal: normal)
        }
    }

    public func clear() {
        lock.lock()
        defer { lock.unlock() }
        cells.removeAll(keepingCapacity: true)
        addedPointCount = 0
        skippedPointCount = 0
        carvedCellCount = 0
    }

    public static func packCell(position: SIMD3<Float>, invCellSize: Float) -> UInt64 {
        packCell(ix: Int32(floor(position.x * invCellSize)),
                 iy: Int32(floor(position.y * invCellSize)),
                 iz: Int32(floor(position.z * invCellSize)))
    }

    /// Low confidence is retained when callers explicitly allow it, but it has
    /// less influence than medium/high-confidence observations.
    private static func weight(for confidence: UInt8) -> Float {
        switch confidence {
        case 0: return 0.25
        case 1: return 1
        default: return 2
        }
    }

    public static func packCell(ix: Int32, iy: Int32, iz: Int32) -> UInt64 {
        let mask: UInt32 = 0x1F_FFFF
        return UInt64(UInt32(bitPattern: ix) & mask)
            | (UInt64(UInt32(bitPattern: iy) & mask) << 21)
            | (UInt64(UInt32(bitPattern: iz) & mask) << 42)
    }
}
