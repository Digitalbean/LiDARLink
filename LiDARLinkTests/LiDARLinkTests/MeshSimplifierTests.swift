import XCTest
import simd
@testable import LiDARLinkShared

final class MeshSimplifierTests: XCTestCase {
    func testSimplifiesPlane() {
        // A 4x4 grid of quads (two triangles each) at 1cm spacing on z=0.
        var vertices: [SIMD3<Float>] = []
        var faces: [UInt32] = []
        let grid = 5
        for y in 0..<grid {
            for x in 0..<grid {
                vertices.append(SIMD3<Float>(Float(x) * 0.01, Float(y) * 0.01, 0))
            }
        }
        for y in 0..<(grid - 1) {
            for x in 0..<(grid - 1) {
                let a = UInt32(y * grid + x)
                let b = UInt32(y * grid + x + 1)
                let c = UInt32((y + 1) * grid + x)
                let d = UInt32((y + 1) * grid + x + 1)
                faces.append(contentsOf: [a, b, c, b, d, c])
            }
        }
        let normals = [SIMD3<Float>](repeating: SIMD3<Float>(0, 0, 1), count: vertices.count)
        let mesh = MeshData(anchorID: UUID(), vertices: vertices, normals: normals, faces: faces,
                            transform: matrix_identity_float4x4, updatedAtMs: 1)
        // 3cm clustering collapses the 5x5 grid (spread over 4cm) to a 2x2.
        let simplified = MeshSimplifier.simplified(mesh, cellSizeMeters: 0.03)
        XCTAssertLessThan(simplified.vertices.count, vertices.count)
        XCTAssertGreaterThan(simplified.vertices.count, 1)
        XCTAssertEqual(simplified.faces.count % 3, 0, "faces stay triangles")
        // No degenerate faces (no repeated cluster ids within a triangle).
        var face = 0
        while face + 2 < simplified.faces.count {
            let a = simplified.faces[face], b = simplified.faces[face + 1], c = simplified.faces[face + 2]
            XCTAssertNotEqual(a, b)
            XCTAssertNotEqual(b, c)
            XCTAssertNotEqual(a, c)
            face += 3
        }
    }

    func testTinyCellKeepsMesh() {
        let mesh = TestFixtures.mesh()
        let simplified = MeshSimplifier.simplified(mesh, cellSizeMeters: 0.0001)
        XCTAssertEqual(simplified.vertices.count, mesh.vertices.count)
        XCTAssertEqual(simplified.faces.count, mesh.faces.count)
    }
}
