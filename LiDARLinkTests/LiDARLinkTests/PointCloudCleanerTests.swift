import XCTest
import simd
@testable import LiDARLinkShared

final class PointCloudCleanerTests: XCTestCase {
    private func point(_ x: Float, _ y: Float, _ z: Float) -> PointCloudPoint {
        PointCloudPoint(position: SIMD3<Float>(x, y, z), color: SIMD3<UInt8>(255, 255, 255))
    }

    /// A 10x10 grid of points at 1cm spacing (a flat wall), plus a few far outliers.
    private func wallWithOutliers(outlierCount: Int) -> [PointCloudPoint] {
        var points: [PointCloudPoint] = []
        for y in 0..<10 {
            for x in 0..<10 {
                points.append(point(Float(x) * 0.01, Float(y) * 0.01, 0))
            }
        }
        for i in 0..<outlierCount {
            points.append(point(Float(i) * 0.5, 5, 5))
        }
        return points
    }

    func testRemovesIsolatedOutliersKeepsWall() {
        let points = wallWithOutliers(outlierCount: 5)
        let cleaned = PointCloudCleaner.radiusOutlierRemoval(points, radius: 0.04, minNeighbors: 8)
        XCTAssertEqual(cleaned.count, 100, "all 5 outliers removed, wall fully kept (incl. borders)")
    }

    func testKeepsDenseSurfaceIncludingBorders() {
        let points = wallWithOutliers(outlierCount: 0)
        let cleaned = PointCloudCleaner.radiusOutlierRemoval(points, radius: 0.04, minNeighbors: 8)
        XCTAssertEqual(cleaned.count, 100)
    }

    func testSmallInputsReturnedUnchanged() {
        let points = [point(0, 0, 0), point(1, 1, 1), point(2, 2, 2)]
        XCTAssertEqual(PointCloudCleaner.radiusOutlierRemoval(points).count, 3)
        XCTAssertTrue(PointCloudCleaner.radiusOutlierRemoval([]).isEmpty)
    }

    func testAutoRadiusAdaptsToSpacing() {
        // 2cm spacing wall: auto radius (4x spacing) should keep it.
        var points: [PointCloudPoint] = []
        for y in 0..<10 {
            for x in 0..<10 {
                points.append(point(Float(x) * 0.02, Float(y) * 0.02, 0))
            }
        }
        points.append(point(5, 5, 5)) // one far outlier
        let cleaned = PointCloudCleaner.radiusOutlierRemoval(points, minNeighbors: 8)
        XCTAssertEqual(cleaned.count, 100)
    }

    func testHighMinNeighborsRemovesEverything() {
        // A 1cm grid has ~50 neighbors within 4cm, so requiring 60 removes all.
        let points = wallWithOutliers(outlierCount: 0)
        let cleaned = PointCloudCleaner.radiusOutlierRemoval(points, radius: 0.04, minNeighbors: 60)
        XCTAssertTrue(cleaned.isEmpty)
    }
}
