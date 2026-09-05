import Foundation
import simd

/// Turns intersecting planes into explicit corners: where two detected planes
/// actually have surface up to their intersection line, that line becomes a
/// crisp edge, the surfels in the join zone are snapped onto one plane and
/// clipped at the line (no more fuzzy overlap), and three edges meeting yields
/// a corner vertex.
public enum CornerExtractor {

    public struct Edge: Sendable, Equatable {
        /// A point on the intersection line, and its unit direction.
        public var point: SIMD3<Float>
        public var direction: SIMD3<Float>
        /// The line parameter range over which both planes have coverage.
        public var tMin: Float
        public var tMax: Float
        public var planeA: Int
        public var planeB: Int
        public var dihedralDegrees: Float
        public var start: SIMD3<Float> { point + direction * tMin }
        public var end: SIMD3<Float> { point + direction * tMax }
        public var length: Float { tMax - tMin }
    }

    public struct Vertex: Sendable, Equatable {
        public var position: SIMD3<Float>
        public var planes: SIMD3<Int32>
    }

    public struct Result: Sendable {
        public var surfels: [Surfel]
        public var edges: [Edge]
        public var vertices: [Vertex]
    }

    public struct Parameters: Sendable {
        public var minDihedralDegrees: Float = 25
        public var maxDihedralDegrees: Float = 155
        /// Perp distance for a surfel to count toward an edge's occupancy.
        public var nearLineDistance: Float = 0.07
        public var binSize: Float = 0.05
        public var maxBridgeBins: Int = 2
        public var minEdgeLength: Float = 0.15
        /// Surfels within this perp distance of an edge get blended.
        public var cornerZone: Float = 0.10
        /// Half-width of the crease blend — the corner rounds over roughly this
        /// much. Small = sharper corner.
        public var filletWidth: Float = 0.02
        public var vertexMergeDistance: Float = 0.15
        /// A surfel is "on" a plane within this signed distance.
        public var planeAssignTolerance: Float = 0.06
        public init() {}
    }

    public static func extract(surfels: [Surfel],
                               planes: [PlaneQuantizer.Plane],
                               parameters p: Parameters = Parameters()) -> Result {
        guard planes.count >= 2, !surfels.isEmpty else { return Result(surfels: surfels, edges: [], vertices: []) }

        // Assign each surfel to its plane (or none).
        var assign = [Int](repeating: -1, count: surfels.count)
        var members: [[Int]] = Array(repeating: [], count: planes.count)
        for (i, s) in surfels.enumerated() {
            var best = -1
            var bestD = p.planeAssignTolerance
            for (pi, pl) in planes.enumerated() {
                let d = abs(pl.signedDistance(s.position))
                if d < bestD, simd_dot(simd_normalize(s.normal), pl.normal) > 0.5 { bestD = d; best = pi }
            }
            assign[i] = best
            if best >= 0 { members[best].append(i) }
        }

        var edges: [Edge] = []
        for a in 0..<planes.count {
            for b in (a + 1)..<planes.count {
                guard let e = edge(a, b, planes: planes, members: members, surfels: surfels, p: p) else { continue }
                edges.append(e)
            }
        }

        // Soft crease: blend each corner-zone surfel between the two planes with
        // a smooth ramp across the bisector. No hard boundary → no kink; the
        // bisector gets a small fillet (~filletWidth), matching real corners.
        var out = surfels
        for e in edges {
            let pA = planes[e.planeA], pB = planes[e.planeB]
            let na = pA.normal, nb = pB.normal
            // Every surfel in the crease zone — including the un-assigned messy
            // ones straddling the join, which are exactly what needs fixing.
            for idx in surfels.indices {
                let pos = surfels[idx].position
                let rel = pos - e.point
                let t = simd_dot(rel, e.direction)
                guard t >= e.tMin - p.cornerZone, t <= e.tMax + p.cornerZone else { continue }
                let perp = simd_length(rel - t * e.direction)
                guard perp <= p.cornerZone else { continue }

                let dA = pA.signedDistance(pos)
                let dB = pB.signedDistance(pos)
                // Only touch surfels that plausibly belong to this crease —
                // close to at least one of the two planes.
                guard min(abs(dA), abs(dB)) <= p.cornerZone else { continue }
                // blend = 1 → all plane A, 0 → all plane B. Transition over
                // 2·filletWidth centred on the bisector (|dA| == |dB|).
                let w = max(p.filletWidth, 1e-3)
                let blend = simd_clamp((abs(dB) - abs(dA)) / (2 * w) + 0.5, 0, 1)

                let projA = pos - dA * na       // onto plane A
                let projB = pos - dB * nb       // onto plane B
                out[idx].position = projA * blend + projB * (1 - blend)
                out[idx].normal = simd_normalize(na * blend + nb * (1 - blend))
            }
        }

        let vertices = solveVertices(edges: edges, planes: planes, p: p)
        return Result(surfels: out, edges: edges, vertices: vertices)
    }

    // MARK: One plane pair

    private static func edge(_ a: Int, _ b: Int,
                             planes: [PlaneQuantizer.Plane],
                             members: [[Int]],
                             surfels: [Surfel],
                             p: Parameters) -> Edge? {
        let na = planes[a].normal, nb = planes[b].normal
        let g = simd_dot(na, nb)
        let theta = acos(simd_clamp(g, -1, 1)) * 180 / .pi
        guard theta >= p.minDihedralDegrees, theta <= p.maxDihedralDegrees else { return nil }
        guard !members[a].isEmpty, !members[b].isEmpty else { return nil }

        let dir = simd_normalize(simd_cross(na, nb))
        guard simd_length(simd_cross(na, nb)) > 1e-4 else { return nil }

        // Point on the line nearest the two planes' combined centroid.
        var c = SIMD3<Float>.zero
        for i in members[a] + members[b] { c += surfels[i].position }
        c /= Float(members[a].count + members[b].count)
        let ra = planes[a].offset - simd_dot(na, c)
        let rb = planes[b].offset - simd_dot(nb, c)
        let denom = 1 - g * g
        guard abs(denom) > 1e-5 else { return nil }
        let la = (ra - g * rb) / denom
        let lb = (rb - g * ra) / denom
        let p0 = c + la * na + lb * nb

        // Occupancy of each plane along the line.
        func occupancy(_ mem: [Int]) -> [Int: Float] {
            var bins: [Int: Float] = [:]
            for i in mem {
                let rel = surfels[i].position - p0
                let t = simd_dot(rel, dir)
                let perp = simd_length(rel - t * dir)
                guard perp <= p.nearLineDistance else { continue }
                bins[Int(floor(t / p.binSize)), default: 0] += surfels[i].weight
            }
            return bins
        }
        let oa = occupancy(members[a]), ob = occupancy(members[b])
        guard !oa.isEmpty, !ob.isEmpty else { return nil }

        let lo = min(oa.keys.min()!, ob.keys.min()!)
        let hi = max(oa.keys.max()!, ob.keys.max()!)
        var bestRun: (start: Int, end: Int)?
        var runStart: Int?
        var gap = 0
        for bin in lo...hi {
            let both = (oa[bin] ?? 0) > 0 && (ob[bin] ?? 0) > 0
            if both {
                if runStart == nil { runStart = bin }
                gap = 0
            } else if runStart != nil {
                gap += 1
                if gap > p.maxBridgeBins {
                    let run = (runStart!, bin - gap)
                    if bestRun == nil || (run.1 - run.0) > (bestRun!.end - bestRun!.start) { bestRun = run }
                    runStart = nil
                    gap = 0
                }
            }
        }
        if let rs = runStart {
            let run = (rs, hi)
            if bestRun == nil || (run.1 - run.0) > (bestRun!.end - bestRun!.start) { bestRun = run }
        }
        guard let run = bestRun else { return nil }
        let tMin = Float(run.start) * p.binSize
        let tMax = Float(run.end + 1) * p.binSize
        guard tMax - tMin >= p.minEdgeLength else { return nil }

        return Edge(point: p0, direction: dir, tMin: tMin, tMax: tMax,
                    planeA: a, planeB: b, dihedralDegrees: theta)
    }

    // MARK: Vertices

    private static func solveVertices(edges: [Edge], planes: [PlaneQuantizer.Plane], p: Parameters) -> [Vertex] {
        // Planes that share edges pairwise → candidate triple.
        var adjacency: [Int: Set<Int>] = [:]
        for e in edges {
            adjacency[e.planeA, default: []].insert(e.planeB)
            adjacency[e.planeB, default: []].insert(e.planeA)
        }
        var found: [Vertex] = []
        let ids = Array(adjacency.keys).sorted()
        for x in 0..<ids.count {
            for y in (x + 1)..<ids.count {
                for z in (y + 1)..<ids.count {
                    let a = ids[x], b = ids[y], c = ids[z]
                    guard adjacency[a]!.contains(b), adjacency[a]!.contains(c), adjacency[b]!.contains(c) else { continue }
                    let m = simd_float3x3(planes[a].normal, planes[b].normal, planes[c].normal).transpose
                    let det = simd_determinant(m)
                    guard abs(det) > 0.05 else { continue }
                    let v = m.inverse * SIMD3<Float>(planes[a].offset, planes[b].offset, planes[c].offset)
                    // Must sit on all three edges' segments.
                    guard edges.contains(where: { onSegment(v, $0, tol: 0.2) && ($0.planeA == a || $0.planeB == a) }),
                          edges.contains(where: { onSegment(v, $0, tol: 0.2) && ($0.planeA == b || $0.planeB == b) }),
                          edges.contains(where: { onSegment(v, $0, tol: 0.2) && ($0.planeA == c || $0.planeB == c) }) else { continue }
                    if found.contains(where: { simd_length($0.position - v) < p.vertexMergeDistance }) { continue }
                    found.append(Vertex(position: v, planes: SIMD3<Int32>(Int32(a), Int32(b), Int32(c))))
                }
            }
        }
        return found
    }

    private static func onSegment(_ v: SIMD3<Float>, _ e: Edge, tol: Float) -> Bool {
        let rel = v - e.point
        let t = simd_dot(rel, e.direction)
        return simd_length(rel - t * e.direction) < tol && t >= e.tMin - tol && t <= e.tMax + tol
    }
}
