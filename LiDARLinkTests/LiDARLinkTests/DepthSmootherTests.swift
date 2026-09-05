import XCTest
@testable import LiDARLinkShared

final class DepthSmootherTests: XCTestCase {
    func testReducesNoiseOnFlatSurface() {
        // Flat 9x9 surface at 1.0 with random ±0.05 noise.
        var depth: [Float] = []
        for _ in 0..<81 {
            let noise = Float.random(in: -0.05...0.05)
            depth.append(1.0 + noise)
        }
        let original = depth
        DepthSmoother.bilateralSmooth(depth: &depth, width: 9, height: 9, spatialSigma: 1.2, rangeSigma: 0.05)
        var beforeDeviation: Float = 0
        var afterDeviation: Float = 0
        for i in depth.indices {
            beforeDeviation += abs(original[i] - 1.0)
            afterDeviation += abs(depth[i] - 1.0)
        }
        XCTAssertLessThan(afterDeviation, beforeDeviation, "smoothing should reduce deviation from the flat value")
    }

    func testPreservesSharpEdge() {
        // 9x9: left half 1.0, right half 2.0 — a sharp step edge.
        var depth: [Float] = []
        for y in 0..<9 {
            for x in 0..<9 {
                depth.append(x < 4 ? 1.0 : 2.0)
            }
        }
        let original = depth
        DepthSmoother.bilateralSmooth(depth: &depth, width: 9, height: 9, spatialSigma: 1.0, rangeSigma: 0.05)
        // Edge pixels stay within 0.2 of their side's value (edge preserved).
        for y in 0..<9 {
            let leftIndex = y * 9 + 3
            let rightIndex = y * 9 + 4
            XCTAssertEqual(depth[leftIndex], original[leftIndex], accuracy: 0.2)
            XCTAssertEqual(depth[rightIndex], original[rightIndex], accuracy: 0.2)
        }
    }

    func testInvalidPixelsUntouched() {
        var depth: [Float] = [.nan, 1.0, 0, 2.0]
        DepthSmoother.bilateralSmooth(depth: &depth, width: 2, height: 2, spatialSigma: 1.0, rangeSigma: 0.05)
        XCTAssertTrue(depth[0].isNaN)
        XCTAssertEqual(depth[2], 0)
    }
}
