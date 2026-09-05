import XCTest
@testable import LiDARLinkShared

final class KeyframeDescriptorTests: XCTestCase {

    private func payload(_ depth: [Float], width: Int, height: Int, confidence: [UInt8]? = nil) -> DepthPayload {
        let f16 = depth.map { Float16($0) }
        let confData = confidence.map { Binary.data(from: $0) }
        return DepthPayload(width: width, height: height, depthData: Binary.data(from: f16), confidenceData: confData)
    }

    func testUniformPlaneProducesUniformDescriptor() {
        let w = 96, h = 72
        let depth = payload([Float](repeating: 2.0, count: w * h), width: w, height: h)
        let descriptor = KeyframeDescriptor.compute(depth: depth, depthScale: 1, confidence: nil)
        XCTAssertTrue(descriptor.bins.allSatisfy { $0 != nil })
        for bin in descriptor.bins {
            XCTAssertEqual(bin!, 2.0, accuracy: 1e-3)
        }
    }

    func testIdenticalScenesHaveZeroDistance() {
        let w = 96, h = 72
        var d = [Float](repeating: 0, count: w * h)
        for y in 0..<h { for x in 0..<w { d[y * w + x] = 1.5 + 0.5 * Float(x) / Float(w) } }
        let a = KeyframeDescriptor.compute(depth: payload(d, width: w, height: h), depthScale: 1, confidence: nil)
        let b = KeyframeDescriptor.compute(depth: payload(d, width: w, height: h), depthScale: 1, confidence: nil)
        let distance = try! XCTUnwrap(KeyframeDescriptor.distance(a, b))
        XCTAssertEqual(distance, 0, accuracy: 1e-3)
    }

    func testSlightlyDifferentViewsOfTheSamePlaceAreClose() {
        // Same wall, second view is 5 cm closer overall — a plausible revisit
        // from a slightly different standpoint, not a pixel-identical replay.
        let w = 96, h = 72
        var d1 = [Float](repeating: 0, count: w * h)
        var d2 = [Float](repeating: 0, count: w * h)
        for y in 0..<h { for x in 0..<w {
            let base = 2.0 + 0.3 * Float(x) / Float(w) + 0.2 * Float(y) / Float(h)
            d1[y * w + x] = base
            d2[y * w + x] = base - 0.05
        }}
        let a = KeyframeDescriptor.compute(depth: payload(d1, width: w, height: h), depthScale: 1, confidence: nil)
        let b = KeyframeDescriptor.compute(depth: payload(d2, width: w, height: h), depthScale: 1, confidence: nil)
        let distance = try! XCTUnwrap(KeyframeDescriptor.distance(a, b))
        XCTAssertLessThan(distance, 0.1)
    }

    func testDifferentPlacesHaveLargeDistance() {
        let w = 96, h = 72
        let near = payload([Float](repeating: 0.8, count: w * h), width: w, height: h)
        let far = payload([Float](repeating: 4.5, count: w * h), width: w, height: h)
        let a = KeyframeDescriptor.compute(depth: near, depthScale: 1, confidence: nil)
        let b = KeyframeDescriptor.compute(depth: far, depthScale: 1, confidence: nil)
        let distance = try! XCTUnwrap(KeyframeDescriptor.distance(a, b))
        XCTAssertGreaterThan(distance, 1.0)
    }

    func testLowConfidencePixelsAreExcludedFromTheirBin() {
        let w = 96, h = 72
        var d = [Float](repeating: 2.0, count: w * h)
        var conf = [UInt8](repeating: 2, count: w * h)
        // Corrupt a quarter of bin 0 (an 8x8 block, top-left) with garbage
        // depth marked low-confidence — it must not pollute the bin's mean.
        for y in 0..<4 { for x in 0..<4 {
            d[y * w + x] = 50.0
            conf[y * w + x] = 0
        }}
        let descriptor = KeyframeDescriptor.compute(depth: payload(d, width: w, height: h, confidence: conf),
                                                     depthScale: 1, confidence: conf, minConfidence: 1)
        // Bin 0 still has plenty of untouched high-confidence pixels (only a
        // quarter of it was corrupted), so it should read close to 2.0, not
        // be dragged toward 50.
        XCTAssertEqual(descriptor.bins[0]!, 2.0, accuracy: 0.5)
    }

    func testTooFewSharedValidBinsReturnsNilDistance() {
        let w = 96, h = 72
        // All-invalid (zero) depth on one side => every bin is nil.
        let empty = payload([Float](repeating: 0, count: w * h), width: w, height: h)
        let real = payload([Float](repeating: 2.0, count: w * h), width: w, height: h)
        let a = KeyframeDescriptor.compute(depth: empty, depthScale: 1, confidence: nil)
        let b = KeyframeDescriptor.compute(depth: real, depthScale: 1, confidence: nil)
        XCTAssertTrue(a.bins.allSatisfy { $0 == nil })
        XCTAssertNil(KeyframeDescriptor.distance(a, b))
    }

    func testMismatchedGridSizeReturnsNilDistance() {
        let w = 96, h = 72
        let depth = payload([Float](repeating: 2.0, count: w * h), width: w, height: h)
        let a = KeyframeDescriptor.compute(depth: depth, depthScale: 1, confidence: nil, cols: 12, rows: 9)
        let b = KeyframeDescriptor.compute(depth: depth, depthScale: 1, confidence: nil, cols: 8, rows: 6)
        XCTAssertNil(KeyframeDescriptor.distance(a, b))
    }
}
