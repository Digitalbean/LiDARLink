import XCTest
import simd
@testable import LiDARLinkShared

final class PlanePolygonBoundTests: XCTestCase {

    /// A 2×2 m wall in the x = 0 plane, y,z ∈ [0, 2].
    private func rectWall() -> PlaneQuantizer.Plane {
        PlaneQuantizer.Plane(
            normal: SIMD3<Float>(1, 0, 0), offset: 0, weight: 100,
            polygon: [SIMD3<Float>(0, 0, 0), SIMD3<Float>(0, 0, 2),
                      SIMD3<Float>(0, 2, 2), SIMD3<Float>(0, 2, 0)])
    }

    /// Same wall with the top-left corner (y > 1.4, z < 0.6) cut away.
    private func lWall() -> PlaneQuantizer.Plane {
        PlaneQuantizer.Plane(
            normal: SIMD3<Float>(1, 0, 0), offset: 0, weight: 100,
            polygon: [SIMD3<Float>(0, 0, 0), SIMD3<Float>(0, 0, 2), SIMD3<Float>(0, 2, 2),
                      SIMD3<Float>(0, 2, 0.6), SIMD3<Float>(0, 1.4, 0.6), SIMD3<Float>(0, 1.4, 0)])
    }

    func testContainsRespectsAnUnboundedPlane() {
        let p = PlaneQuantizer.Plane(normal: SIMD3<Float>(0, 1, 0), offset: 0, weight: 1)
        XCTAssertTrue(p.contains(SIMD3<Float>(100, 0, -50)))
    }

    func testRectangleContains() {
        let w = rectWall()
        XCTAssertTrue(w.contains(SIMD3<Float>(0.01, 1.0, 1.0)))       // inside
        XCTAssertTrue(w.contains(SIMD3<Float>(0.4, 0.5, 0.5)))        // off-plane, projects inside
        XCTAssertFalse(w.contains(SIMD3<Float>(0, 3.0, 1.0)))         // above the wall
        XCTAssertFalse(w.contains(SIMD3<Float>(0, 1.0, -0.4)))        // beside the wall
        XCTAssertTrue(w.contains(SIMD3<Float>(0, 1.0, -0.3), margin: 0.5))  // within slack
    }

    func testCutCornerIsExcluded() {
        let w = lWall()
        XCTAssertTrue(w.contains(SIMD3<Float>(0, 0.5, 1.0)))          // solid part
        XCTAssertFalse(w.contains(SIMD3<Float>(0, 1.8, 0.2)))         // the cut-away corner
    }

    func testSnapSkipsSurfelsOutsideTheWallOutline() {
        func surfel(_ p: SIMD3<Float>) -> Surfel {
            Surfel(position: p, normal: SIMD3<Float>(1, 0, 0), color: .zero,
                   radius: 0.03, weight: 4, owner: 0, lastSeen: 0)
        }
        let inside = surfel(SIMD3<Float>(0.02, 1.0, 1.0))
        let beyond = surfel(SIMD3<Float>(0.02, 1.0, 3.5))   // coplanar but well past the wall
        let snapped = PlaneQuantizer.snap([inside, beyond], to: [rectWall()])
        XCTAssertEqual(snapped[0].position.x, 0, accuracy: 1e-4, "inside → snapped onto x = 0")
        XCTAssertEqual(snapped[1].position.x, 0.02, accuracy: 1e-4, "past the outline → untouched")
    }
}
