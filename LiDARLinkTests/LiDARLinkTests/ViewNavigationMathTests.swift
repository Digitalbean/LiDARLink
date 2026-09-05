import XCTest
import simd
@testable import LiDARLinkShared

final class ViewNavigationMathTests: XCTestCase {
    private func point(at position: SIMD3<Float>) -> PointCloudPoint {
        PointCloudPoint(position: position, color: SIMD3<UInt8>(255, 0, 0))
    }

    func testCenterPixelRayPointsStraightAhead() {
        let ray = ViewNavigationMath.viewRay(cameraPosition: SIMD3<Float>(0, 0, 0),
                                             cameraForward: SIMD3<Float>(0, 0, -1),
                                             cameraUp: SIMD3<Float>(0, 1, 0),
                                             viewportPoint: CGPoint(x: 400, y: 300),
                                             viewportSize: CGSize(width: 800, height: 600),
                                             fieldOfViewY: 60)
        XCTAssertEqual(ray.origin, SIMD3<Float>(0, 0, 0))
        XCTAssertEqual(ray.direction.x, 0, accuracy: 0.001)
        XCTAssertEqual(ray.direction.y, 0, accuracy: 0.001)
        XCTAssertEqual(ray.direction.z, -1, accuracy: 0.001)
    }

    func testRightEdgePixelRayPointsRight() {
        let ray = ViewNavigationMath.viewRay(cameraPosition: SIMD3<Float>(0, 0, 0),
                                             cameraForward: SIMD3<Float>(0, 0, -1),
                                             cameraUp: SIMD3<Float>(0, 1, 0),
                                             viewportPoint: CGPoint(x: 800, y: 300),
                                             viewportSize: CGSize(width: 800, height: 600),
                                             fieldOfViewY: 60)
        XCTAssertGreaterThan(ray.direction.x, 0)
        XCTAssertEqual(ray.direction.y, 0, accuracy: 0.001)
        XCTAssertLessThan(ray.direction.z, 0)
    }

    func testTopEdgePixelRayPointsUp() {
        let ray = ViewNavigationMath.viewRay(cameraPosition: SIMD3<Float>(0, 0, 0),
                                             cameraForward: SIMD3<Float>(0, 0, -1),
                                             cameraUp: SIMD3<Float>(0, 1, 0),
                                             viewportPoint: CGPoint(x: 400, y: 0),
                                             viewportSize: CGSize(width: 800, height: 600),
                                             fieldOfViewY: 60)
        XCTAssertGreaterThan(ray.direction.y, 0)
        XCTAssertEqual(ray.direction.x, 0, accuracy: 0.001)
    }

    func testNearestPointPicksClosestToRay() {
        let points = [point(at: SIMD3<Float>(0.1, 0, -5)),
                      point(at: SIMD3<Float>(10, 0, -5)),
                      point(at: SIMD3<Float>(-0.05, 0, -5))]
        let hit = ViewNavigationMath.nearestPoint(to: SIMD3<Float>(0, 0, 0),
                                                  direction: SIMD3<Float>(0, 0, -1),
                                                  in: points,
                                                  maxDistance: 1)
        XCTAssertEqual(Double(hit?.position.x ?? 0), -0.05, accuracy: 0.001)
    }

    func testNearestPointRejectsFarPoints() {
        let points = [point(at: SIMD3<Float>(5, 0, -5))]
        let hit = ViewNavigationMath.nearestPoint(to: SIMD3<Float>(0, 0, 0),
                                                  direction: SIMD3<Float>(0, 0, -1),
                                                  in: points,
                                                  maxDistance: 1)
        XCTAssertNil(hit)
    }

    func testNearestPointEmptyCloud() {
        let hit = ViewNavigationMath.nearestPoint(to: SIMD3<Float>(0, 0, 0),
                                                  direction: SIMD3<Float>(0, 0, -1),
                                                  in: [],
                                                  maxDistance: 1)
        XCTAssertNil(hit)
    }

    func testOrbitPositionRoundTrip() {
        let target = SIMD3<Float>(1, 2, 3)
        let position = ViewNavigationMath.orbitCameraPosition(target: target, yaw: 0.7, pitch: 0.4, distance: 5)
        let components = ViewNavigationMath.orbitComponents(cameraPosition: position, target: target)
        XCTAssertEqual(components.yaw, 0.7, accuracy: 0.001)
        XCTAssertEqual(components.pitch, 0.4, accuracy: 0.001)
        XCTAssertEqual(components.distance, 5, accuracy: 0.001)
    }

    func testOrbitPositionZeroPitch() {
        let target = SIMD3<Float>(1, 2, 3)
        let position = ViewNavigationMath.orbitCameraPosition(target: target, yaw: 0, pitch: 0, distance: 2)
        XCTAssertEqual(position.x, 1, accuracy: 0.001)
        XCTAssertEqual(position.y, 2, accuracy: 0.001)
        XCTAssertEqual(position.z, 5, accuracy: 0.001)
    }

    // MARK: Walk navigation

    func testWalkLookDirectionYawZeroFacesPositiveZ() {
        let dir = ViewNavigationMath.walkLookDirection(yaw: 0, pitch: 0)
        XCTAssertEqual(dir.x, 0, accuracy: 0.001)
        XCTAssertEqual(dir.y, 0, accuracy: 0.001)
        XCTAssertEqual(dir.z, 1, accuracy: 0.001)
    }

    func testWalkForwardMovesAlongFacingRegardlessOfPitch() {
        // Looking steeply down must not shorten a forward step.
        let start = SIMD3<Float>(0, 1.6, 0)
        let moved = ViewNavigationMath.walkStep(position: start, yaw: 0,
                                                forward: 1, strafe: 0, lift: 0,
                                                distance: 2, floorY: nil, eyeHeight: 1.6)
        XCTAssertEqual(moved.x, 0, accuracy: 0.001)
        XCTAssertEqual(moved.y, 1.6, accuracy: 0.001)
        XCTAssertEqual(moved.z, 2, accuracy: 0.001)
    }

    func testWalkFloorLockPinsHeight() {
        let moved = ViewNavigationMath.walkStep(position: SIMD3<Float>(0, 5, 0), yaw: 0,
                                                forward: 1, strafe: 0, lift: 1,
                                                distance: 1, floorY: -0.4, eyeHeight: 1.6)
        XCTAssertEqual(moved.y, 1.2, accuracy: 0.001) // floorY + eyeHeight, lift ignored
    }

    func testWalkStrafeRightIsScreenRight() {
        // Facing +Z (yaw 0) with +Y up, screen-right is world -X.
        let moved = ViewNavigationMath.walkStep(position: .zero, yaw: 0,
                                                forward: 0, strafe: 1, lift: 0,
                                                distance: 1, floorY: nil, eyeHeight: 0)
        XCTAssertEqual(moved.x, -1, accuracy: 0.001)
        XCTAssertEqual(moved.z, 0, accuracy: 0.001)
    }

    func testEstimatedFloorIgnoresStrayLowOutlier() {
        var pts = (0..<200).map { _ in point(at: SIMD3<Float>(0, 0, 0)) }
        pts[0] = point(at: SIMD3<Float>(0, -9, 0)) // single deep outlier
        let floor = ViewNavigationMath.estimatedFloorY(of: pts, percentile: 0.02)
        // The lone deep point must not drag the floor down with it.
        XCTAssertEqual(floor ?? .nan, 0, accuracy: 0.2)
    }

    func testEstimatedFloorTracksTheBulkOfALayeredCloud() {
        // Floor slab at y = -0.5, wall points spread from -0.4 up to 2.
        var pts = (0..<300).map { _ in point(at: SIMD3<Float>(0, -0.5, 0)) }
        pts += (0..<700).map { i in point(at: SIMD3<Float>(0, -0.4 + Float(i) / 300, 0)) }
        let floor = ViewNavigationMath.estimatedFloorY(of: pts)
        XCTAssertEqual(floor ?? .nan, -0.5, accuracy: 0.1)
    }

    func testEstimatedFloorNilForEmptyCloud() {
        XCTAssertNil(ViewNavigationMath.estimatedFloorY(of: []))
    }

    func testWalkStandpointLooksIntoTheFullerHalfOfALongScan() {
        // Hallway long in Z: dense cluster at z in [-8,-6], a thin tail to +1.
        var pts = (0..<300).map { i in point(at: SIMD3<Float>(0, 1, -8 + Float(i) / 150)) }
        pts += (0..<20).map { i in point(at: SIMD3<Float>(0, 1, Float(i) / 20)) }
        let stand = ViewNavigationMath.walkStandpoint(for: pts, eyeHeight: 1.6, floorY: 0)
        XCTAssertEqual(stand.yaw, .pi, accuracy: 0.001)          // faces -Z, toward the crowd
        XCTAssertGreaterThan(stand.position.z, -3.5)             // stood back toward the +Z end
        XCTAssertEqual(stand.position.y, 1.6, accuracy: 0.001)   // eye height above the floor
    }

    func testWalkStandpointFallsBackForEmptyCloud() {
        let stand = ViewNavigationMath.walkStandpoint(for: [], eyeHeight: 1.6, floorY: nil)
        XCTAssertEqual(stand.yaw, .pi, accuracy: 0.001)
    }

    // MARK: Ground plane

    func testGroundPlaneLevelsATiltedFloor() {
        // Floor tilted ~10 degrees about X, at height y0, plus a wall of points above.
        let tilt = simd_quatf(angle: 0.17, axis: SIMD3<Float>(1, 0, 0))
        var pts: [PointCloudPoint] = []
        for xi in 0..<20 {
            for zi in 0..<20 {
                let flat = SIMD3<Float>(Float(xi) / 10 - 1, 0.3, Float(zi) / 10 - 1)
                pts.append(point(at: tilt.act(flat)))
            }
        }
        for i in 0..<50 { pts.append(point(at: SIMD3<Float>(0, 1 + Float(i) / 25, 0))) }

        guard let transform = GroundPlane.levelingTransform(for: pts) else {
            return XCTFail("no floor found")
        }
        let leveled = GroundPlane.apply(transform, to: pts)
        let floorYs = leveled.prefix(400).map { $0.position.y }
        let spread = (floorYs.max() ?? 0) - (floorYs.min() ?? 0)
        XCTAssertLessThan(spread, 0.05)                       // floor is flat now
        XCTAssertEqual(floorYs.reduce(0, +) / Float(floorYs.count), 0, accuracy: 0.05) // at y = 0
    }

    func testGroundPlaneNilWithoutAFloor() {
        let wall = (0..<200).map { i in point(at: SIMD3<Float>(0, Float(i) / 20, 0)) }
        XCTAssertNil(GroundPlane.levelingTransform(for: wall))
    }
}
