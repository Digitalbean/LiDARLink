import XCTest
@testable import LiDARLinkShared

final class DepthCleanerTests: XCTestCase {
    func testMaxDistanceZeroesFarValues() {
        var depth: [Float] = [0.5, 2.0, 5.0, 10.0]
        DepthCleaner.applyMaxDistance(depth: &depth, width: 2, height: 2, maxDistanceMeters: 4)
        XCTAssertEqual(depth[0], 0.5)
        XCTAssertEqual(depth[1], 2.0)
        XCTAssertEqual(depth[2], 0)
        XCTAssertEqual(depth[3], 0)
    }

    func testMaxDistanceKeepsNaN() {
        var depth: [Float] = [.nan, 3.0]
        DepthCleaner.applyMaxDistance(depth: &depth, width: 2, height: 1, maxDistanceMeters: 4)
        XCTAssertTrue(depth[0].isNaN)
        XCTAssertEqual(depth[1], 3.0)
    }

    func testFlyingPixelRemoved() {
        // 3x3: center is a 2.0m spike, everything else 0.5m.
        var depth: [Float] = Array(repeating: 0.5, count: 9)
        depth[4] = 2.0
        DepthCleaner.removeFlyingPixels(depth: &depth, width: 3, height: 3, maxNeighborDelta: 0.15, minAgreeingNeighbors: 2)
        XCTAssertEqual(depth[4], 0, "isolated spike should be removed")
        XCTAssertEqual(depth[0], 0.5, "valid surface should be kept")
    }

    func testSmoothSurfaceUntouched() {
        var depth: [Float] = Array(repeating: 1.0, count: 25)
        DepthCleaner.removeFlyingPixels(depth: &depth, width: 5, height: 5, maxNeighborDelta: 0.15, minAgreeingNeighbors: 2)
        XCTAssertEqual(depth, Array(repeating: 1.0, count: 25))
    }

    func testEdgeSpikeRemovedBorderKept() {
        var depth: [Float] = Array(repeating: 1.0, count: 9)
        depth[0] = 3.0 // corner spike
        depth[1] = 1.0
        DepthCleaner.removeFlyingPixels(depth: &depth, width: 3, height: 3, maxNeighborDelta: 0.2, minAgreeingNeighbors: 2)
        XCTAssertEqual(depth[0], 0, "corner spike removed (both available neighbors disagree)")
        XCTAssertEqual(depth[1], 1.0)
    }

    func testSmallGradientKept() {
        // A 0.1m gradient (slanted wall) should survive a 0.15m delta filter.
        var depth: [Float] = [1.0, 1.1, 1.2]
        DepthCleaner.removeFlyingPixels(depth: &depth, width: 3, height: 1, maxNeighborDelta: 0.15, minAgreeingNeighbors: 1)
        XCTAssertEqual(depth, [1.0, 1.1, 1.2])
    }
}
