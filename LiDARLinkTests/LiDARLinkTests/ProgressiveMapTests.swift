import XCTest
import simd
@testable import LiDARLinkShared

final class ProgressiveMapTests: XCTestCase {
    private func point(_ x: Float, _ y: Float, _ z: Float, _ c: SIMD3<UInt8> = SIMD3<UInt8>(255, 0, 0)) -> PointCloudPoint {
        PointCloudPoint(position: SIMD3<Float>(x, y, z), color: c)
    }

    func testSameCellSkipped() {
        let map = ProgressiveMap(cellSize: 0.01)
        let first = map.deduplicate([point(0.0, 0, 0), point(0.005, 0, 0)])
        XCTAssertEqual(first.count, 1)
        XCTAssertEqual(map.deduplicate([point(0.0, 0, 0)]).count, 0)
        XCTAssertEqual(map.addedPointCount, 1)
        XCTAssertEqual(map.skippedPointCount, 2)
    }

    func testDifferentCellsAdded() {
        let map = ProgressiveMap(cellSize: 0.01)
        let added = map.deduplicate([point(0, 0, 0), point(0.05, 0, 0), point(0, 0.05, 0), point(0, 0, 0.05)])
        XCTAssertEqual(added.count, 4)
    }

    func testNegativeCoordinates() {
        let map = ProgressiveMap(cellSize: 0.01)
        let added = map.deduplicate([point(-0.005, 0, 0), point(-0.015, 0, 0)])
        XCTAssertEqual(added.count, 2)
        XCTAssertEqual(map.deduplicate([point(-0.005, 0, 0)]).count, 0)
    }

    func testCellSizeChangesDedupGranularity() {
        let coarse = ProgressiveMap(cellSize: 0.02)
        XCTAssertEqual(coarse.deduplicate([point(0, 0, 0), point(0.01, 0, 0)]).count, 1)
        let fine = ProgressiveMap(cellSize: 0.005)
        XCTAssertEqual(fine.deduplicate([point(0, 0, 0), point(0.01, 0, 0)]).count, 2)
    }

    func testMergedPointAveragesObservations() {
        let map = ProgressiveMap(cellSize: 0.05)
        _ = map.deduplicate([point(0.0, 0, 0, SIMD3<UInt8>(100, 0, 0))])
        _ = map.deduplicate([point(0.02, 0, 0, SIMD3<UInt8>(200, 0, 0))])
        let merged = map.mergedPoints()
        XCTAssertEqual(merged.count, 1)
        XCTAssertEqual(merged[0].position.x, 0.01, accuracy: 0.001)
        XCTAssertEqual(merged[0].color.x, 150, accuracy: 1)
    }

    func testMergedPointConvergesWithMoreObservations() {
        let map = ProgressiveMap(cellSize: 0.05)
        _ = map.deduplicate([point(0, 0, 0)])
        _ = map.deduplicate([point(0.02, 0, 0)])
        _ = map.deduplicate([point(0.04, 0, 0)])
        let merged = map.mergedPoints()
        XCTAssertEqual(merged[0].position.x, 0.02, accuracy: 0.001)
    }

    func testHighConfidenceObservationHasMoreInfluence() {
        let map = ProgressiveMap(cellSize: 0.05)
        _ = map.deduplicate([
            PointCloudPoint(position: SIMD3<Float>(0, 0, 0),
                            color: SIMD3<UInt8>(100, 0, 0),
                            confidence: 1),
            PointCloudPoint(position: SIMD3<Float>(0.02, 0, 0),
                            color: SIMD3<UInt8>(200, 0, 0),
                            confidence: 2)
        ])

        let merged = map.mergedPoints()
        XCTAssertEqual(merged.count, 1)
        XCTAssertEqual(merged[0].position.x, 0.013_333, accuracy: 0.001)
        XCTAssertEqual(merged[0].color.x, 166, accuracy: 1)
        XCTAssertEqual(merged[0].confidence, 2)
    }

    func testRebuildSeedsOccupancy() {
        let map = ProgressiveMap(cellSize: 0.01)
        _ = map.deduplicate([point(0, 0, 0)])
        map.rebuild(from: [point(0, 0, 0), point(0, 0, 0), point(1, 1, 1)])
        XCTAssertEqual(map.occupiedCellCount, 2)
        XCTAssertEqual(map.deduplicate([point(0, 0, 0), point(1, 1, 1)]).count, 0)
    }

    func testClearResets() {
        let map = ProgressiveMap(cellSize: 0.01)
        _ = map.deduplicate([point(0, 0, 0)])
        map.clear()
        XCTAssertEqual(map.occupiedCellCount, 0)
        XCTAssertEqual(map.addedPointCount, 0)
        XCTAssertEqual(map.deduplicate([point(0, 0, 0)]).count, 1)
    }

    func testEmptyInput() {
        let map = ProgressiveMap(cellSize: 0.01)
        XCTAssertEqual(map.deduplicate([]).count, 0)
        XCTAssertTrue(map.mergedPoints().isEmpty)
    }
}
