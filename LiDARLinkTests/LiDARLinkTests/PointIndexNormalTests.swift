import XCTest
import simd
@testable import LiDARLinkShared

final class PointIndexNormalTests: XCTestCase {
    private func point(_ x: Float, _ y: Float, _ z: Float) -> PointCloudPoint {
        PointCloudPoint(position: SIMD3<Float>(x, y, z), color: SIMD3<UInt8>(200, 200, 200))
    }

    private func planePoints(count: Int, cell: Float, z: Float) -> [PointCloudPoint] {
        var points: [PointCloudPoint] = []
        for i in 0..<count {
            points.append(point(Float(i % 10) * cell, Float(i / 10) * cell, z))
        }
        return points
    }

    func testNeighborQuery() {
        let index = PointIndex(cellSize: 0.05)
        let points = planePoints(count: 100, cell: 0.01, z: 0)
        index.add(points)
        let neighbors = index.neighbors(of: 55, radius: 0.04, points: points)
        XCTAssertGreaterThan(neighbors.count, 8)
        XCTAssertTrue(neighbors.allSatisfy { $0.distance <= 0.04 })
    }

    func testPlaneNormalPointsUp() {
        let index = PointIndex(cellSize: 0.05)
        let points = planePoints(count: 100, cell: 0.01, z: 0)
        index.add(points)
        guard let normal = PointNormalEstimator.estimateNormal(at: 55, in: index, points: points, radius: 0.04, minNeighbors: 8) else {
            XCTFail("expected a normal")
            return
        }
        // Plane normal is (0,0,±1) — magnitude of z dominates.
        XCTAssertGreaterThan(abs(normal.z), 0.9)
    }

    func testLitColorBrighterWhenFacingLight() {
        let bright = PointColorizer.litColor(color: SIMD3<UInt8>(200, 200, 200),
                                             normal: SIMD3<Float>(0, 1, 0),
                                             lightDirection: SIMD3<Float>(0, 1, 0))
        let dim = PointColorizer.litColor(color: SIMD3<UInt8>(200, 200, 200),
                                          normal: SIMD3<Float>(0, -1, 0),
                                          lightDirection: SIMD3<Float>(0, 1, 0))
        XCTAssertGreaterThan(bright.x, dim.x)
        XCTAssertGreaterThan(bright.x, 190) // faces light → near full brightness
        XCTAssertLessThan(dim.x, 110)       // faces away → ambient only
    }

    func testTemporalBlendConvergesAndSkipsJumps() {
        // delta 0.15 is within maxDelta 0.2 → blended: 0.5*1.0 + 0.5*0.85 = 0.925.
        var current = [Float](repeating: 1.0, count: 8)
        let previous = [Float](repeating: 0.85, count: 8)
        XCTAssertTrue(DepthTemporal.blend(current: &current, previous: previous, alpha: 0.5, maxDelta: 0.2))
        XCTAssertEqual(current[0], 0.925, accuracy: 0.001)

        var withJump = [Float](repeating: 1.0, count: 8)
        withJump[0] = 5.0 // occlusion jump — exceeds maxDelta
        _ = DepthTemporal.blend(current: &withJump, previous: previous, alpha: 0.5, maxDelta: 0.2)
        XCTAssertEqual(withJump[0], 5.0, accuracy: 0.001, "large jump must pass through")
        XCTAssertEqual(withJump[1], 0.925, accuracy: 0.001)
    }
}
