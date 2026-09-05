import Foundation
import simd

/// Vertex-clustering mesh decimation: clusters vertices by voxel and remaps faces,
/// dropping degenerate faces. Used to trim large reconstructed meshes for export.
public enum MeshSimplifier {
    public static func simplified(_ mesh: MeshData, cellSizeMeters: Float) -> MeshData {
        guard cellSizeMeters > 0, !mesh.vertices.isEmpty else { return mesh }
        let inv = 1 / cellSizeMeters
        var clusterKey: [Int64: Int] = [:]
        var newVertices: [SIMD3<Float>] = []
        var newNormals: [SIMD3<Float>] = []
        var remap = [Int](repeating: -1, count: mesh.vertices.count)

        for (index, vertex) in mesh.vertices.enumerated() {
            let key = Self.packKey(ix: Int32(floor(vertex.x * inv)),
                                   iy: Int32(floor(vertex.y * inv)),
                                   iz: Int32(floor(vertex.z * inv)))
            if let existing = clusterKey[key] {
                remap[index] = existing
            } else {
                let newIndex = newVertices.count
                clusterKey[key] = newIndex
                newVertices.append(vertex)
                newNormals.append(mesh.normals.indices.contains(index) ? mesh.normals[index] : SIMD3<Float>(0, 0, 1))
                remap[index] = newIndex
            }
        }

        var newFaces: [UInt32] = []
        newFaces.reserveCapacity(mesh.faces.count)
        var face = 0
        while face + 2 < mesh.faces.count {
            let a = remap[Int(mesh.faces[face])]
            let b = remap[Int(mesh.faces[face + 1])]
            let c = remap[Int(mesh.faces[face + 2])]
            if a != b, b != c, a != c {
                newFaces.append(UInt32(a))
                newFaces.append(UInt32(b))
                newFaces.append(UInt32(c))
            }
            face += 3
        }

        return MeshData(anchorID: mesh.anchorID,
                        vertices: newVertices,
                        normals: newNormals,
                        faces: newFaces,
                        transform: mesh.transform,
                        updatedAtMs: mesh.updatedAtMs)
    }

    static func packKey(ix: Int32, iy: Int32, iz: Int32) -> Int64 {
        let mask: Int64 = 0x3FFFF
        return Int64(ix & Int32(mask))
            | (Int64(iy & Int32(mask)) << 18)
            | (Int64(iz & Int32(mask)) << 36)
    }
}
