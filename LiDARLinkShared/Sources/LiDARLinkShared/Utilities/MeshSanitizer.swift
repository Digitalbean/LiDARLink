import Foundation
import simd

/// Defensive mesh cleanup: drops triangles with out-of-range or degenerate
/// indices and pads normals to match the vertex count. Applied by the assembler
/// and the renderer so corrupt/partial streams cannot crash SceneKit.
public enum MeshSanitizer {
    public static func sanitize(vertices: [SIMD3<Float>],
                                normals: [SIMD3<Float>],
                                faces: [UInt32]) -> (vertices: [SIMD3<Float>], normals: [SIMD3<Float>], faces: [UInt32]) {
        var safeFaces: [UInt32] = []
        safeFaces.reserveCapacity(faces.count)
        let vertexCount = vertices.count
        var face = 0
        while face + 2 < faces.count {
            let a = Int(faces[face])
            let b = Int(faces[face + 1])
            let c = Int(faces[face + 2])
            if a < vertexCount, b < vertexCount, c < vertexCount, a != b, b != c, a != c {
                safeFaces.append(UInt32(a))
                safeFaces.append(UInt32(b))
                safeFaces.append(UInt32(c))
            }
            face += 3
        }
        var safeNormals = normals
        if safeNormals.count < vertexCount {
            let pad = SIMD3<Float>(0, 0, 1)
            while safeNormals.count < vertexCount {
                safeNormals.append(pad)
            }
        }
        return (vertices, safeNormals, safeFaces)
    }
}
