import XCTest
import simd
@testable import LiDARLinkShared

final class ProgressiveMapLifecycleTests: XCTestCase {

    private let target = SIMD3<Float>(0.5, 0.5, 2.0)

    private func observe(_ map: ProgressiveMap, from viewpoint: SIMD3<Float>?, confidence: UInt8 = 2) {
        _ = map.deduplicate([PointCloudPoint(position: target, color: .init(repeating: 120), confidence: confidence)],
                            viewpoint: viewpoint)
    }

    // MARK: Direction tracking

    func testTripodRepeatsStayCandidateDespiteHighWeight() {
        // Ten observations, all from the same direction (a static phone on a
        // tripod) — raw repeat count must not be enough on its own.
        let map = ProgressiveMap(cellSize: 0.02)
        let sameSpot = SIMD3<Float>(0.5, 0.5, 0)   // due -Z of the target every time
        for _ in 0..<10 { observe(map, from: sameSpot) }
        XCTAssertEqual(map.lifecycle(at: target), .candidate,
                      "ten hits from one vantage point is not the same as evidence from several")
    }

    func testFewObservationsFromDistinctViewpointsBecomeConfirmed() {
        // Four observations, each from a meaningfully different direction
        // (walking around, not standing still) — less raw repeat count than
        // the tripod case, but it should promote past candidate.
        let map = ProgressiveMap(cellSize: 0.02)
        let viewpoints = [
            SIMD3<Float>(target.x, target.y, target.z - 1),   // -Z side
            SIMD3<Float>(target.x + 1, target.y, target.z),   // +X side
            SIMD3<Float>(target.x, target.y, target.z + 1),   // +Z side
            SIMD3<Float>(target.x - 1, target.y, target.z),   // -X side
        ]
        for vp in viewpoints { observe(map, from: vp) }
        XCTAssertEqual(map.lifecycle(at: target), .confirmed)
    }

    func testHighWeightBecomesEstablishedRegardlessOfDirectionCount() {
        // Matches carveFreeSpace's existing establishedWeight immunity bar —
        // established status is about accumulated support, direction
        // diversity is only the gate between candidate and confirmed.
        let map = ProgressiveMap(cellSize: 0.02)
        let sameSpot = SIMD3<Float>(0.5, 0.5, 0)
        for _ in 0..<20 { observe(map, from: sameSpot) }   // weight ~40, one direction
        XCTAssertEqual(map.lifecycle(at: target), .established)
    }

    func testWithoutAViewpointWeightAloneDoesNotReachConfirmed() {
        // No viewpoint => no direction evidence ever recorded => the
        // confirmed diversity requirement can never be met by weight alone.
        // This is the intentional fallback for callers that don't track a
        // camera position (e.g. re-seeding from an already-fused cloud): they
        // stay candidate until they cross all the way to established, never
        // pausing at confirmed on weight alone.
        let map = ProgressiveMap(cellSize: 0.02)
        for _ in 0..<8 { observe(map, from: nil) }   // weight 16: above confirmedWeight (6), below establishedWeight (24)
        XCTAssertEqual(map.lifecycle(at: target), .candidate)
    }

    func testUnoccupiedCellHasNoLifecycle() {
        let map = ProgressiveMap(cellSize: 0.02)
        XCTAssertNil(map.lifecycle(at: target))
    }

    // MARK: mergedPoints filtering

    func testMergedPointsDefaultsToEverythingUnchanged() {
        let map = ProgressiveMap(cellSize: 0.02)
        observe(map, from: SIMD3<Float>(0.5, 0.5, 0))   // one hit -> candidate
        XCTAssertEqual(map.mergedPoints().count, 1, "default behaviour is unfiltered, as before this feature existed")
    }

    func testMergedPointsWithMinLifecycleHidesUnconfirmedSpeckle() {
        let map = ProgressiveMap(cellSize: 0.02)
        // A confirmed surface...
        let confirmed = SIMD3<Float>(1, 1, 1)
        for vp in [SIMD3<Float>(1, 1, 0), SIMD3<Float>(2, 1, 1), SIMD3<Float>(1, 1, 2), SIMD3<Float>(0, 1, 1)] {
            _ = map.deduplicate([PointCloudPoint(position: confirmed, color: .zero, confidence: 2)], viewpoint: vp)
        }
        // ...and one single-hit speck elsewhere.
        _ = map.deduplicate([PointCloudPoint(position: SIMD3<Float>(5, 5, 5), color: .zero, confidence: 2)],
                            viewpoint: SIMD3<Float>(5, 5, 4))

        XCTAssertEqual(map.mergedPoints().count, 2, "unfiltered still returns both")
        let filtered = map.mergedPoints(minLifecycle: .confirmed)
        XCTAssertEqual(filtered.count, 1, "the speck is hidden, not the confirmed surface")
        XCTAssertEqual(filtered[0].position, confirmed)
    }

    func testHiddenSpeckIsStillInTheMapNotDeleted() {
        // Filtering is a display concern only — the cell survives and can
        // still be promoted (or carved, or re-observed) like any other.
        let map = ProgressiveMap(cellSize: 0.02)
        observe(map, from: SIMD3<Float>(0.5, 0.5, 0))
        XCTAssertTrue(map.mergedPoints(minLifecycle: .confirmed).isEmpty)
        XCTAssertEqual(map.occupiedCellCount, 1, "the cell itself was never removed")
    }
}
