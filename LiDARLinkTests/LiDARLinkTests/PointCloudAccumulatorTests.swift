import XCTest
@testable import LiDARLinkShared

final class PointCloudAccumulatorTests: XCTestCase {
    private func point(_ i: Int) -> PointCloudPoint {
        PointCloudPoint(position: SIMD3<Float>(Float(i), 0, 0), color: SIMD3<UInt8>(255, 0, 0))
    }

    func testAccumulatesUntilCapThenSkips() {
        let accumulator = PointCloudAccumulator(maxPoints: 50)
        accumulator.add((0..<100).map { point($0) })
        XCTAssertEqual(accumulator.count, 50)
        XCTAssertEqual(accumulator.skippedPointCount, 50)
        XCTAssertTrue(accumulator.isFull)
        // Oldest points are preserved (object permanence).
        XCTAssertEqual(accumulator.snapshot().first?.position.x, 0)
        // Further additions are skipped entirely.
        accumulator.add([point(999)])
        XCTAssertEqual(accumulator.count, 50)
        XCTAssertEqual(accumulator.skippedPointCount, 51)
    }

    func testPartialRoom() {
        let accumulator = PointCloudAccumulator(maxPoints: 10)
        accumulator.add((0..<7).map { point($0) })
        accumulator.add((0..<10).map { point($0 + 100) })
        XCTAssertEqual(accumulator.count, 10)
        XCTAssertEqual(accumulator.skippedPointCount, 7)
        XCTAssertEqual(accumulator.snapshot().first?.position.x, 0)
    }

    func testSnapshotIsCopy() {
        let accumulator = PointCloudAccumulator(maxPoints: 100)
        accumulator.add([point(1), point(2)])
        var snapshot = accumulator.snapshot()
        snapshot.removeAll()
        XCTAssertEqual(accumulator.count, 2)
    }

    func testClear() {
        let accumulator = PointCloudAccumulator(maxPoints: 100)
        accumulator.add([point(1)])
        accumulator.clear()
        XCTAssertEqual(accumulator.count, 0)
        XCTAssertEqual(accumulator.addedPointCount, 0)
        XCTAssertEqual(accumulator.skippedPointCount, 0)
        XCTAssertFalse(accumulator.isFull)
    }

    func testAddEmptyIsNoOp() {
        let accumulator = PointCloudAccumulator(maxPoints: 10)
        accumulator.add([])
        XCTAssertEqual(accumulator.count, 0)
    }

    func testEvenSamplingHonorsCapAndCoversCloud() {
        let sampled = PointCloudSampler.evenlySample((0..<100).map { point($0) }, limit: 10)
        XCTAssertEqual(sampled.count, 10)
        XCTAssertEqual(sampled.first?.position.x, 0)
        XCTAssertEqual(sampled.last?.position.x, 90)
    }

    func testRefreshThrottleIsEnabledTimedAndSingleFlight() {
        var throttle = RefreshThrottle(intervalMs: 1_000)
        XCTAssertTrue(throttle.shouldStart(nowMs: 100, enabled: true, isInFlight: false))
        XCTAssertFalse(throttle.shouldStart(nowMs: 1_000, enabled: true, isInFlight: false))
        XCTAssertFalse(throttle.shouldStart(nowMs: 1_100, enabled: true, isInFlight: true))
        XCTAssertTrue(throttle.shouldStart(nowMs: 1_100, enabled: true, isInFlight: false))
        XCTAssertFalse(throttle.shouldStart(nowMs: 2_100, enabled: false, isInFlight: false))
        throttle.reset()
        XCTAssertTrue(throttle.shouldStart(nowMs: 2_100, enabled: true, isInFlight: false))
    }
}
