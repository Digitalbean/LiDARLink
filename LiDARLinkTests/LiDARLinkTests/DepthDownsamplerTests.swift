import XCTest
@testable import LiDARLinkShared

final class DepthDownsamplerTests: XCTestCase {
    func testDownsampleDimensions() {
        let depth = [Float](repeating: 1, count: 16 * 16)
        let result = DepthDownsampler.downsample(depth: depth, width: 16, height: 16, targetWidth: 4, targetHeight: 4)
        XCTAssertEqual(result.count, 16)
    }

    func testDownsampleAveragesWithinASurface() {
        // 2x2 → 1x1, all within 10 cm: average of [2.00, 2.02, 2.04, 2.06] = 2.03
        let depth: [Float] = [2.00, 2.02, 2.04, 2.06]
        let result = DepthDownsampler.downsample(depth: depth, width: 2, height: 2, targetWidth: 1, targetHeight: 1)
        XCTAssertEqual(Float(result[0]), 2.03, accuracy: 0.01)
    }

    func testDownsampleTakesNearestAcrossADepthEdge() {
        // Box straddles an edge (2 m vs 5 m): use the near surface, not a
        // phantom average floating between them.
        let depth: [Float] = [2.0, 2.01, 5.0, 5.01]
        let result = DepthDownsampler.downsample(depth: depth, width: 2, height: 2, targetWidth: 1, targetHeight: 1)
        XCTAssertEqual(Float(result[0]), 2.0, accuracy: 0.01)
    }

    func testDownsampleIgnoresInvalidSamples() {
        // 2x2 → 1x1 with one NaN and one zero: average of [4.00, 4.02] = 4.01
        let depth: [Float] = [.nan, 4.00, 0, 4.02]
        let result = DepthDownsampler.downsample(depth: depth, width: 2, height: 2, targetWidth: 1, targetHeight: 1)
        XCTAssertEqual(Float(result[0]), 4.01, accuracy: 0.01)
    }

    func testDownsampleAllInvalidYieldsZero() {
        let depth: [Float] = [.nan, .nan, 0, 0]
        let result = DepthDownsampler.downsample(depth: depth, width: 2, height: 2, targetWidth: 1, targetHeight: 1)
        XCTAssertEqual(Float(result[0]), 0, accuracy: 0.001)
    }

    func testConfidenceNearestSampling() {
        let confidence: [UInt8] = [0, 1, 2, 2]
        let result = DepthDownsampler.downsampleConfidence(confidence: confidence, width: 2, height: 2, targetWidth: 1, targetHeight: 1)
        XCTAssertEqual(result, [0])
    }

    func testDownsamplePreservesFloat16PrecisionRoughly() {
        let depth = [Float](repeating: 1.5, count: 4 * 4)
        let result = DepthDownsampler.downsample(depth: depth, width: 4, height: 4, targetWidth: 2, targetHeight: 2)
        XCTAssertEqual(Float(result[0]), 1.5, accuracy: 0.001)
    }
}
