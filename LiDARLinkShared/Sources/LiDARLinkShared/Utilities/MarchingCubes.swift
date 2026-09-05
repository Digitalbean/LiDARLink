import Foundation
import simd

/// Extracts a triangle mesh from a scalar field sampled on a regular grid, using
/// the standard Marching Cubes case tables. Used to turn a `TSDFVolume` into a
/// shaded surface.
public enum MarchingCubes {

    /// One grid cell's eight corner values and world position, plus a lookup for
    /// neighbouring cells (nil where the field is unknown there).
    public struct Sampler {
        /// Field value at an integer grid coordinate, or nil if unobserved.
        public var value: (SIMD3<Int32>) -> Float?
        /// Colour at an integer grid coordinate.
        public var colour: (SIMD3<Int32>) -> SIMD3<Float>
        /// Grid coordinate → world position.
        public var worldPosition: (SIMD3<Float>) -> SIMD3<Float>
        public init(value: @escaping (SIMD3<Int32>) -> Float?,
                    colour: @escaping (SIMD3<Int32>) -> SIMD3<Float>,
                    worldPosition: @escaping (SIMD3<Float>) -> SIMD3<Float>) {
            self.value = value
            self.colour = colour
            self.worldPosition = worldPosition
        }
    }

    public struct Mesh: Sendable {
        public var vertices: [SIMD3<Float>] = []
        public var normals: [SIMD3<Float>] = []
        public var colours: [SIMD3<Float>] = []
        public var faces: [UInt32] = []
        public var isEmpty: Bool { faces.isEmpty }
    }

    /// The eight corner offsets of a unit cell, in the canonical MC order.
    static let corners: [SIMD3<Int32>] = [
        SIMD3(0, 0, 0), SIMD3(1, 0, 0), SIMD3(1, 1, 0), SIMD3(0, 1, 0),
        SIMD3(0, 0, 1), SIMD3(1, 0, 1), SIMD3(1, 1, 1), SIMD3(0, 1, 1),
    ]
    /// The two corners each of the 12 edges.
    static let edgeCorners: [(Int, Int)] = [
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    ]

    /// Triangulates every cell whose lower corner is in `cells`. `isoLevel` is
    /// the surface value (0 for a TSDF). A cell is skipped unless all eight
    /// corners have a known value.
    public static func surface(cells: [SIMD3<Int32>],
                               isoLevel: Float,
                               sampler: Sampler) -> Mesh {
        var mesh = Mesh()
        var vertexCache: [Int64: UInt32] = [:]

        for cell in cells {
            var values = [Float](repeating: 0, count: 8)
            var caseIndex = 0
            var ok = true
            for i in 0..<8 {
                guard let v = sampler.value(cell &+ corners[i]) else { ok = false; break }
                values[i] = v
                if v < isoLevel { caseIndex |= (1 << i) }
            }
            guard ok, caseIndex != 0, caseIndex != 255 else { continue }

            let tri = Self.triTable[caseIndex]
            var e = 0
            while tri[e] != -1 {
                var idx = [UInt32](repeating: 0, count: 3)
                for k in 0..<3 {
                    let edge = Int(tri[e + k])
                    let (a, b) = edgeCorners[edge]
                    let ca = cell &+ corners[a]
                    let cb = cell &+ corners[b]
                    let key = Self.edgeKey(ca, cb)
                    if let cached = vertexCache[key] {
                        idx[k] = cached
                    } else {
                        let va = values[a], vb = values[b]
                        let t = abs(vb - va) < 1e-6 ? 0.5 : (isoLevel - va) / (vb - va)
                        let gridPos = SIMD3<Float>(ca) + (SIMD3<Float>(cb) - SIMD3<Float>(ca)) * t
                        let world = sampler.worldPosition(gridPos)
                        let colour = simd_mix(sampler.colour(ca), sampler.colour(cb), SIMD3<Float>(repeating: t))
                        let normal = Self.gradientNormal(at: ca, and: cb, t: t, sampler: sampler)
                        let vi = UInt32(mesh.vertices.count)
                        mesh.vertices.append(world)
                        mesh.normals.append(normal)
                        mesh.colours.append(colour)
                        vertexCache[key] = vi
                        idx[k] = vi
                    }
                }
                if idx[0] != idx[1], idx[1] != idx[2], idx[0] != idx[2] {
                    // Orient the triangle so its winding agrees with the
                    // gradient normals (the table's winding can't be trusted
                    // across sign conventions).
                    let v0 = mesh.vertices[Int(idx[0])], v1 = mesh.vertices[Int(idx[1])], v2 = mesh.vertices[Int(idx[2])]
                    let faceNormal = simd_cross(v1 - v0, v2 - v0)
                    let vertexNormal = mesh.normals[Int(idx[0])] + mesh.normals[Int(idx[1])] + mesh.normals[Int(idx[2])]
                    if simd_dot(faceNormal, vertexNormal) < 0 {
                        mesh.faces.append(contentsOf: [idx[0], idx[2], idx[1]])
                    } else {
                        mesh.faces.append(contentsOf: idx)
                    }
                }
                e += 3
            }
        }
        return mesh
    }

    // MARK: Helpers

    private static func gradientNormal(at a: SIMD3<Int32>, and b: SIMD3<Int32>, t: Float, sampler: Sampler) -> SIMD3<Float> {
        func grad(_ p: SIMD3<Int32>) -> SIMD3<Float> {
            func s(_ o: SIMD3<Int32>) -> Float { sampler.value(p &+ o) ?? 0 }
            return SIMD3<Float>(s(SIMD3(1, 0, 0)) - s(SIMD3(-1, 0, 0)),
                                s(SIMD3(0, 1, 0)) - s(SIMD3(0, -1, 0)),
                                s(SIMD3(0, 0, 1)) - s(SIMD3(0, 0, -1)))
        }
        let g = grad(a) * (1 - t) + grad(b) * t
        let len = simd_length(g)
        // The field's gradient points from inside (negative) toward free space
        // (positive) — i.e. along the outward-facing surface normal.
        return len > 1e-6 ? g / len : SIMD3<Float>(0, 1, 0)
    }

    private static func edgeKey(_ a: SIMD3<Int32>, _ b: SIMD3<Int32>) -> Int64 {
        let (lo, hi) = (a.x, a.y, a.z) <= (b.x, b.y, b.z) ? (a, b) : (b, a)
        func pack(_ v: SIMD3<Int32>) -> Int64 {
            (Int64(v.x) & 0xFFFFF) | ((Int64(v.y) & 0xFFFFF) << 20) | ((Int64(v.z) & 0xFFFFF) << 40)
        }
        return pack(lo) &* 1_000_003 &+ pack(hi)
    }
}

private func <= (l: (Int32, Int32, Int32), r: (Int32, Int32, Int32)) -> Bool {
    if l.0 != r.0 { return l.0 < r.0 }
    if l.1 != r.1 { return l.1 < r.1 }
    return l.2 <= r.2
}
