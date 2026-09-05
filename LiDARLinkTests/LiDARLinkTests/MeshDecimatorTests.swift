import XCTest
import simd
@testable import LiDARLinkShared

final class MeshDecimatorTests: XCTestCase {

    /// A dense grid of triangles over a 2 m plane.
    private func grid(_ n: Int) -> MarchingCubes.Mesh {
        var mesh = MarchingCubes.Mesh()
        for j in 0...n { for i in 0...n {
            let x = Float(i) / Float(n) * 2, z = Float(j) / Float(n) * 2
            mesh.vertices.append(SIMD3<Float>(x, 0, z))
            mesh.normals.append(SIMD3<Float>(0, 1, 0))
            mesh.colours.append(SIMD3<Float>(120, 120, 120))
        }}
        let row = n + 1
        for j in 0..<n { for i in 0..<n {
            let a = UInt32(j * row + i), b = a + 1, c = a + UInt32(row), d = c + 1
            mesh.faces.append(contentsOf: [a, c, b, b, c, d])
        }}
        return mesh
    }

    func testDecimatesToRoughlyBudget() {
        let dense = grid(60)                       // ~7200 triangles
        XCTAssertGreaterThan(dense.faces.count / 3, 5000)

        let out = MeshDecimator.decimated(dense, targetTriangles: 800)
        XCTAssertLessThanOrEqual(out.faces.count / 3, 1600, "should be near the budget, not wildly over")
        XCTAssertGreaterThan(out.faces.count / 3, 100, "should not collapse to nothing")
        XCTAssertEqual(out.vertices.count, out.normals.count)
        XCTAssertEqual(out.vertices.count, out.colours.count)
    }

    func testUnderBudgetPassesThrough() {
        let small = grid(4)
        let out = MeshDecimator.decimated(small, targetTriangles: 10_000)
        XCTAssertEqual(out.faces.count, small.faces.count)
    }

    func testNoDegenerateFaces() {
        let out = MeshDecimator.decimated(grid(50), targetTriangles: 500)
        var f = 0
        while f + 2 < out.faces.count {
            let a = out.faces[f], b = out.faces[f + 1], c = out.faces[f + 2]
            XCTAssertFalse(a == b || b == c || a == c)
            f += 3
        }
    }

    func testStaysWithinOriginalBounds() {
        let dense = grid(40)
        let out = MeshDecimator.decimated(dense, targetTriangles: 300)
        for v in out.vertices {
            XCTAssertGreaterThanOrEqual(v.x, -0.2)
            XCTAssertLessThanOrEqual(v.x, 2.2)
            XCTAssertGreaterThanOrEqual(v.z, -0.2)
            XCTAssertLessThanOrEqual(v.z, 2.2)
        }
    }
}
