import XCTest
import simd
@testable import LiDARLinkShared

final class CornerExtractorTests: XCTestCase {

    private func surfel(_ p: SIMD3<Float>, _ n: SIMD3<Float>) -> Surfel {
        Surfel(position: p, normal: simd_normalize(n), color: SIMD3<Float>(repeating: 150),
               radius: 0.04, weight: 4, owner: 0, lastSeen: 0)
    }

    /// Two walls meeting at x = 0, z = 0: one in the x = 0 plane (normal +x),
    /// one in the z = 0 plane (normal +z), both spanning y = 0…2, and each
    /// running 0…1.5 m out from the corner.
    private func lShape() -> ([Surfel], [PlaneQuantizer.Plane]) {
        var s: [Surfel] = []
        for yi in 0..<20 { for k in 0..<15 {
            let y = Float(yi) * 0.1
            s.append(surfel(SIMD3<Float>(0, y, Float(k) * 0.1), SIMD3<Float>(1, 0, 0)))   // x=0 wall
            s.append(surfel(SIMD3<Float>(Float(k) * 0.1, y, 0), SIMD3<Float>(0, 0, 1)))   // z=0 wall
        }}
        let planes = [
            PlaneQuantizer.Plane(normal: SIMD3<Float>(1, 0, 0), offset: 0, weight: 100),
            PlaneQuantizer.Plane(normal: SIMD3<Float>(0, 0, 1), offset: 0, weight: 100),
        ]
        return (s, planes)
    }

    func testFindsTheEdgeWhereTwoWallsMeet() {
        let (s, planes) = lShape()
        let r = CornerExtractor.extract(surfels: s, planes: planes)
        XCTAssertEqual(r.edges.count, 1)
        let e = r.edges[0]
        XCTAssertEqual(e.dihedralDegrees, 90, accuracy: 2)
        // Edge line is the y axis through the origin.
        XCTAssertEqual(simd_length(e.point - simd_dot(e.point, SIMD3<Float>(0,1,0)) * SIMD3<Float>(0,1,0)), 0, accuracy: 0.03)
        XCTAssertGreaterThan(abs(simd_dot(e.direction, SIMD3<Float>(0, 1, 0))), 0.99)
        XCTAssertGreaterThan(e.length, 1.5)
    }

    func testNoEdgeForPlanesThatNeverMeet() {
        var s: [Surfel] = []
        for yi in 0..<20 { for k in 0..<10 {
            s.append(surfel(SIMD3<Float>(0, Float(yi) * 0.1, Float(k) * 0.1), SIMD3<Float>(1, 0, 0)))
            s.append(surfel(SIMD3<Float>(3, Float(yi) * 0.1, Float(k) * 0.1), SIMD3<Float>(0, 0, 1)))  // 3 m away
        }}
        let planes = [
            PlaneQuantizer.Plane(normal: SIMD3<Float>(1, 0, 0), offset: 0, weight: 100),
            PlaneQuantizer.Plane(normal: SIMD3<Float>(0, 0, 1), offset: 0, weight: 100),
        ]
        XCTAssertTrue(CornerExtractor.extract(surfels: s, planes: planes).edges.isEmpty)
    }

    func testCreaseBlendPullsMessyJoinSurfelsToTheEdge() {
        var (s, planes) = lShape()
        // Messy surfels straddling the join, up to 8 cm off both planes.
        let messyStart = s.count
        for yi in 0..<10 {
            s.append(surfel(SIMD3<Float>(-0.06, Float(yi) * 0.2, -0.06), SIMD3<Float>(0.7, 0, 0.7)))
        }
        let r = CornerExtractor.extract(surfels: s, planes: planes)

        // The wall surfels away from the join stay flat on their plane.
        for (i, out) in r.surfels.enumerated() where i < messyStart {
            XCTAssertTrue(abs(out.position.x) < 0.02 || abs(out.position.z) < 0.02)
        }
        // The messy join surfels are pulled to within ~fillet distance of the
        // crease line (the y axis) — no longer 8 cm out.
        for i in messyStart..<r.surfels.count {
            let p = r.surfels[i].position
            let perp = simd_length(SIMD3<Float>(p.x, 0, p.z))
            XCTAssertLessThan(perp, 0.05, "join surfel \(perp) m from the crease, was 0.085")
        }
    }

    func testThreeWallsGiveACornerVertex() {
        var s: [Surfel] = []
        // x=0, z=0, y=0 all meeting at the origin, each a 1.2 m patch.
        for i in 0..<15 { for j in 0..<15 {
            let u = Float(i) * 0.08, v = Float(j) * 0.08
            s.append(surfel(SIMD3<Float>(0, u, v), SIMD3<Float>(1, 0, 0)))
            s.append(surfel(SIMD3<Float>(u, 0, v), SIMD3<Float>(0, 1, 0)))
            s.append(surfel(SIMD3<Float>(u, v, 0), SIMD3<Float>(0, 0, 1)))
        }}
        let planes = [
            PlaneQuantizer.Plane(normal: SIMD3<Float>(1, 0, 0), offset: 0, weight: 100),
            PlaneQuantizer.Plane(normal: SIMD3<Float>(0, 1, 0), offset: 0, weight: 100),
            PlaneQuantizer.Plane(normal: SIMD3<Float>(0, 0, 1), offset: 0, weight: 100),
        ]
        let r = CornerExtractor.extract(surfels: s, planes: planes)
        XCTAssertEqual(r.edges.count, 3)
        XCTAssertEqual(r.vertices.count, 1)
        XCTAssertEqual(simd_length(r.vertices[0].position), 0, accuracy: 0.05)
    }
}
