import XCTest
import simd
@testable import LiDARLinkShared

final class SurfelMesherTests: XCTestCase {

    /// A flat wall of surfels at z = -1, normals toward +z.
    private func wall(_ n: Int, weight: Float = 5) -> [Surfel] {
        var s: [Surfel] = []
        let span: Float = 0.6
        for j in 0..<n { for i in 0..<n {
            let x = (Float(i) / Float(n - 1) - 0.5) * span
            let y = (Float(j) / Float(n - 1) - 0.5) * span
            s.append(Surfel(position: SIMD3<Float>(x, y, -1), normal: SIMD3<Float>(0, 0, 1),
                            color: SIMD3<Float>(200, 120, 60), radius: 0.05, weight: weight,
                            owner: 0, lastSeen: 0))
        }}
        return s
    }

    func testBakesAContinuousSurfaceFromAWall() {
        let mesh = SurfelMesher.mesh(from: wall(12))
        XCTAssertFalse(mesh.isEmpty)
        XCTAssertEqual(mesh.faces.count % 3, 0)
        XCTAssertEqual(mesh.vertices.count, mesh.normals.count)
        // Surface sits at the wall depth, within a couple of voxels.
        for v in mesh.vertices { XCTAssertEqual(v.z, -1, accuracy: 0.06) }
        // Carries the surfel colour.
        let mean = mesh.colours.reduce(SIMD3<Float>.zero, +) / Float(max(mesh.colours.count, 1))
        XCTAssertGreaterThan(mean.x, mean.z, "warm colour survives the bake")
    }

    func testEmptyInputYieldsEmptyMesh() {
        XCTAssertTrue(SurfelMesher.mesh(from: []).isEmpty)
    }

    func testLowWeightSurfelsAreIgnored() {
        let mesh = SurfelMesher.mesh(from: wall(12, weight: 0.5))
        XCTAssertTrue(mesh.isEmpty, "nothing above the min-weight floor")
    }
}
