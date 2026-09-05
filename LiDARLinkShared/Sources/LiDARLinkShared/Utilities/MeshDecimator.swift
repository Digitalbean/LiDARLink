import Foundation
import simd

/// Vertex-clustering decimation for a `MarchingCubes.Mesh`. Buckets vertices onto
/// a regular grid, collapses each bucket to its centroid (averaging normal and
/// colour), remaps the faces, and drops the faces that go degenerate.
///
/// Cheap and topology-agnostic — good enough to bring a dense TSDF surface down
/// to a triangle budget the texture baker can actually chart.
public enum MeshDecimator {

    /// Decimate to roughly `targetTriangles` by searching for a clustering cell
    /// size. Returns the input unchanged if it is already under budget.
    public static func decimated(_ mesh: MarchingCubes.Mesh,
                                 targetTriangles: Int,
                                 boundsHint: Float? = nil) -> MarchingCubes.Mesh {
        let triangleCount = mesh.faces.count / 3
        guard triangleCount > targetTriangles, targetTriangles > 0, !mesh.vertices.isEmpty else { return mesh }

        var lo = mesh.vertices[0], hi = mesh.vertices[0]
        for v in mesh.vertices { lo = simd_min(lo, v); hi = simd_max(hi, v) }
        let diag = simd_length(hi - lo)
        guard diag > 0 else { return mesh }

        // Start from an even areal split of the bounding box across the budget,
        // then grow the cell until we land under budget (max a few tries).
        var cell = boundsHint ?? max(diag / sqrt(Float(max(targetTriangles, 1))), 1e-4)
        var result = cluster(mesh, cell: cell)
        var guardCount = 0
        while result.faces.count / 3 > targetTriangles && guardCount < 12 {
            cell *= 1.35
            result = cluster(mesh, cell: cell)
            guardCount += 1
        }
        return result
    }

    /// One clustering pass at a fixed cell size.
    public static func cluster(_ mesh: MarchingCubes.Mesh, cell: Float) -> MarchingCubes.Mesh {
        guard cell > 0 else { return mesh }
        let inv = 1 / cell
        let hasColour = mesh.colours.count == mesh.vertices.count
        let hasNormal = mesh.normals.count == mesh.vertices.count

        var keyToCluster: [Int64: Int] = [:]
        var remap = [Int](repeating: -1, count: mesh.vertices.count)
        var posSum: [SIMD3<Float>] = []
        var nrmSum: [SIMD3<Float>] = []
        var colSum: [SIMD3<Float>] = []
        var count: [Float] = []

        for (i, v) in mesh.vertices.enumerated() {
            let key = pack(Int32(floor(v.x * inv)), Int32(floor(v.y * inv)), Int32(floor(v.z * inv)))
            let cluster: Int
            if let existing = keyToCluster[key] {
                cluster = existing
            } else {
                cluster = posSum.count
                keyToCluster[key] = cluster
                posSum.append(.zero); nrmSum.append(.zero); colSum.append(.zero); count.append(0)
            }
            remap[i] = cluster
            posSum[cluster] += v
            count[cluster] += 1
            if hasNormal { nrmSum[cluster] += mesh.normals[i] }
            if hasColour { colSum[cluster] += mesh.colours[i] }
        }

        var out = MarchingCubes.Mesh()
        out.vertices.reserveCapacity(posSum.count)
        for c in 0..<posSum.count {
            let n = max(count[c], 1)
            out.vertices.append(posSum[c] / n)
            if hasNormal {
                let s = nrmSum[c]
                out.normals.append(simd_length(s) > 1e-6 ? simd_normalize(s) : SIMD3<Float>(0, 1, 0))
            }
            if hasColour { out.colours.append(colSum[c] / n) }
        }

        out.faces.reserveCapacity(mesh.faces.count)
        var f = 0
        while f + 2 < mesh.faces.count {
            let a = remap[Int(mesh.faces[f])], b = remap[Int(mesh.faces[f + 1])], c = remap[Int(mesh.faces[f + 2])]
            f += 3
            if a != b && b != c && a != c {
                out.faces.append(UInt32(a)); out.faces.append(UInt32(b)); out.faces.append(UInt32(c))
            }
        }
        return out
    }

    private static func pack(_ x: Int32, _ y: Int32, _ z: Int32) -> Int64 {
        // 21-bit signed per axis.
        let mask: Int64 = 0x1F_FFFF
        return (Int64(x) & mask) | ((Int64(y) & mask) << 21) | ((Int64(z) & mask) << 42)
    }
}
