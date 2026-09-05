import Foundation
import simd

/// Snaps a surfel set toward the handful of large planes a built environment is
/// made of: walls, floor, ceiling. Two steps —
///
/// 1. **Gravity alignment.** ARKit's world is gravity-aligned (+y up), so a
///    surfel normal within a threshold of vertical is snapped to exactly
///    horizontal-facing, and one near horizontal is snapped to exactly ±y.
/// 2. **Plane snapping.** Aligned normals + plane offsets are voted into bins;
///    a bin with enough surfel weight and spatial spread is a plane, refined by
///    a weighted least-squares fit. Every surfel close to a detected plane (in
///    angle and distance) is projected onto it and takes its normal.
///
/// Non-planar geometry — furniture, clutter — matches no plane and is left as
/// its own surfels.
public enum PlaneQuantizer {

    public struct Plane: Sendable, Equatable {
        /// Unit normal. A point `p` lies on the plane when `dot(normal, p) == offset`.
        public var normal: SIMD3<Float>
        public var offset: Float
        public var weight: Float
        /// World-space outline of the finite surface (RoomPlan wall polygon).
        /// Empty = the plane is unbounded, snap anywhere on it.
        public var polygon: [SIMD3<Float>] = []

        public func signedDistance(_ p: SIMD3<Float>) -> Float { simd_dot(normal, p) - offset }

        /// Whether `p` (projected onto the plane) falls inside the polygon, with
        /// a slack `margin` in metres. Always true for an unbounded plane.
        public func contains(_ p: SIMD3<Float>, margin: Float = 0) -> Bool {
            guard polygon.count >= 3 else { return true }
            var u = simd_cross(normal, SIMD3<Float>(0, 1, 0))
            if simd_length(u) < 1e-4 { u = simd_cross(normal, SIMD3<Float>(1, 0, 0)) }
            u = simd_normalize(u)
            let v = simd_normalize(simd_cross(normal, u))
            let origin = polygon[0]
            func to2D(_ w: SIMD3<Float>) -> SIMD2<Float> {
                let r = w - origin
                return SIMD2<Float>(simd_dot(r, u), simd_dot(r, v))
            }
            let projected = p - signedDistance(p) * normal
            let pt = to2D(projected)
            let poly = polygon.map(to2D)

            var inside = false
            var j = poly.count - 1
            for i in poly.indices {
                let a = poly[i], b = poly[j]
                if (a.y > pt.y) != (b.y > pt.y),
                   pt.x < (b.x - a.x) * (pt.y - a.y) / (b.y - a.y) + a.x {
                    inside.toggle()
                }
                j = i
            }
            if inside { return true }
            guard margin > 0 else { return false }
            for i in poly.indices {
                let a = poly[i], b = poly[(i + 1) % poly.count]
                let ab = b - a, t = simd_clamp(simd_dot(pt - a, ab) / max(simd_dot(ab, ab), 1e-6), 0, 1)
                if simd_length(pt - (a + t * ab)) <= margin { return true }
            }
            return false
        }
    }

    public struct Parameters: Sendable {
        /// Snap a normal to vertical/horizontal when within this angle of it.
        public var gravitySnapDegrees: Float = 12
        /// Normal-bin width for voting (degrees) and the snap angle tolerance.
        public var normalToleranceDegrees: Float = 14
        /// Offset-bin width (metres) and the snap distance tolerance.
        public var offsetTolerance: Float = 0.04
        /// A plane needs at least this much summed surfel weight…
        public var minPlaneWeight: Float = 30
        /// …and its members must span at least this far.
        public var minPlaneExtent: Float = 0.4
        public var maxPlanes: Int = 24
        public init() {}
    }

    // MARK: Alignment

    public static func gravityAlign(_ n: SIMD3<Float>, degrees: Float) -> SIMD3<Float> {
        let up = SIMD3<Float>(0, 1, 0)
        let unit = simd_normalize(n)
        let cosSnap = cos(degrees * .pi / 180)
        let vy = simd_dot(unit, up)
        if abs(vy) >= cosSnap {                       // near horizontal surface
            return vy >= 0 ? up : -up
        }
        if abs(vy) <= sin(degrees * .pi / 180) {      // near vertical surface
            let horizontal = unit - vy * up
            let len = simd_length(horizontal)
            if len > 1e-4 { return horizontal / len }
        }
        return unit
    }

    // MARK: Detection

    /// `priors` (e.g. RoomPlan walls) are optional. A detected plane close to a
    /// prior takes the prior's exact orientation and offset; a prior with enough
    /// surfel support but no matching detection is added. Empty priors → behaves
    /// exactly as before.
    public static func detect(_ surfels: [Surfel],
                              parameters: Parameters = Parameters(),
                              priors: [Plane] = []) -> [Plane] {
        guard !surfels.isEmpty else { return [] }
        let normDot = cos(parameters.normalToleranceDegrees * .pi / 180)

        // Pass 1 — cluster by gravity-aligned normal direction only.
        struct Dir { var wsum: Float = 0; var nsum = SIMD3<Float>.zero; var members: [Int] = [] }
        var dirs: [Dir] = []
        for (idx, s) in surfels.enumerated() where s.weight > 0 {
            let n = gravityAlign(s.normal, degrees: parameters.gravitySnapDegrees)
            if let di = dirs.firstIndex(where: { simd_dot(simd_normalize($0.nsum), n) >= normDot }) {
                dirs[di].wsum += s.weight
                dirs[di].nsum += n * s.weight
                dirs[di].members.append(idx)
            } else {
                dirs.append(Dir(wsum: s.weight, nsum: n * s.weight, members: [idx]))
            }
        }

        // Pass 2 — within each direction cluster, 1-D cluster the offsets under
        // the *refined* normal.
        var planes: [Plane] = []
        for d in dirs where d.wsum >= parameters.minPlaneWeight {
            let n = simd_normalize(d.nsum)
            guard simd_length(d.nsum) > 1e-4 else { continue }
            let sorted = d.members
                .map { (offset: simd_dot(n, surfels[$0].position), s: surfels[$0]) }
                .sorted { $0.offset < $1.offset }

            var groupStart = 0
            func flush(_ end: Int) {
                guard end > groupStart else { return }
                let slice = sorted[groupStart..<end]
                let w = slice.reduce(Float(0)) { $0 + $1.s.weight }
                guard w >= parameters.minPlaneWeight else { return }
                var lo = slice.first!.s.position, hi = lo
                for m in slice { lo = simd_min(lo, m.s.position); hi = simd_max(hi, m.s.position) }
                guard simd_length(hi - lo) >= parameters.minPlaneExtent else { return }
                // Robust fit over the group — a corner join or a bowed patch
                // gets down-weighted, so the plane tracks the flat surface.
                if let fit = PlaneFit.fit(slice.map { ($0.s.position, $0.s.weight) }, scaleMeters: 0.02) {
                    var normal = simd_dot(fit.normal, n) < 0 ? -fit.normal : fit.normal
                    normal = gravityAlign(normal, degrees: parameters.gravitySnapDegrees)
                    planes.append(Plane(normal: normal, offset: fit.offset(for: normal), weight: w))
                } else {
                    var osum: Float = 0
                    for m in slice { osum += m.offset * m.s.weight }
                    planes.append(Plane(normal: n, offset: osum / w, weight: w))
                }
            }
            for i in 1..<max(sorted.count, 1) {
                if sorted[i].offset - sorted[i - 1].offset > parameters.offsetTolerance {
                    flush(i); groupStart = i
                }
            }
            flush(sorted.count)
        }
        // Fold in priors.
        if !priors.isEmpty {
            let offTol = parameters.offsetTolerance * 2
            for prior in priors {
                if let i = planes.firstIndex(where: {
                    abs(simd_dot($0.normal, prior.normal)) >= normDot && abs($0.offset - prior.offset * sign(simd_dot($0.normal, prior.normal))) <= offTol
                }) {
                    // Adopt the prior's exact geometry — normal, offset, and the
                    // wall outline — keeping the surfel weight.
                    let flip: Float = simd_dot(planes[i].normal, prior.normal) < 0 ? -1 : 1
                    planes[i] = Plane(normal: prior.normal * flip,
                                      offset: prior.offset * flip,
                                      weight: planes[i].weight + prior.weight,
                                      polygon: prior.polygon)
                } else {
                    // No detection near it — add it if the surfels *inside the
                    // wall outline* back it up.
                    var support: Float = 0
                    for s in surfels where s.weight > 0 && abs(prior.signedDistance(s.position)) <= offTol
                        && abs(simd_dot(simd_normalize(s.normal), prior.normal)) >= normDot
                        && prior.contains(s.position, margin: offTol) {
                        support += s.weight
                    }
                    if support >= parameters.minPlaneWeight {
                        planes.append(prior)
                    }
                }
            }
        }

        planes.sort { $0.weight > $1.weight }
        return Array(planes.prefix(parameters.maxPlanes))
    }

    // MARK: Snapping

    /// Surfels outside a bounded plane's outline (plus this much slack) are not
    /// snapped to it — a coplanar object or the far wall stays put.
    public static var polygonSnapMargin: Float = 0.15

    public static func snap(_ surfels: [Surfel], to planes: [Plane],
                            parameters: Parameters = Parameters()) -> [Surfel] {
        guard !planes.isEmpty else { return surfels }
        let normTol = cos(parameters.normalToleranceDegrees * .pi / 180)
        return surfels.map { s in
            var best = -1
            var bestDist = parameters.offsetTolerance
            for (i, p) in planes.enumerated() {
                guard simd_dot(simd_normalize(s.normal), p.normal) >= normTol else { continue }
                let d = abs(p.signedDistance(s.position))
                guard d < bestDist else { continue }
                guard p.contains(s.position, margin: polygonSnapMargin) else { continue }
                bestDist = d; best = i
            }
            guard best >= 0 else { return s }
            let p = planes[best]
            var out = s
            out.position = s.position - p.signedDistance(s.position) * p.normal
            out.normal = p.normal
            return out
        }
    }

    /// Detect + snap in one call.
    public static func quantize(_ surfels: [Surfel], parameters: Parameters = Parameters(),
                                priors: [Plane] = []) -> [Surfel] {
        snap(surfels, to: detect(surfels, parameters: parameters, priors: priors), parameters: parameters)
    }
}
