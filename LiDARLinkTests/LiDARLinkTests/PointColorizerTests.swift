import XCTest
import simd
@testable import LiDARLinkShared

final class PointColorizerTests: XCTestCase {
    private func point(y: Float, color: SIMD3<UInt8> = SIMD3<UInt8>(1, 2, 3)) -> PointCloudPoint {
        PointCloudPoint(position: SIMD3<Float>(0, y, 0), color: color)
    }

    func testHeightRampEndpoints() {
        XCTAssertEqual(PointColorizer.heightRamp(0), SIMD3<UInt8>(30, 60, 220))
        XCTAssertEqual(PointColorizer.heightRamp(1), SIMD3<UInt8>(230, 60, 50))
        XCTAssertEqual(PointColorizer.heightRamp(-5), SIMD3<UInt8>(30, 60, 220)) // clamped
        XCTAssertEqual(PointColorizer.heightRamp(2), SIMD3<UInt8>(230, 60, 50))  // clamped
    }

    func testHeightRampMidpoint() {
        let mid = PointColorizer.heightRamp(0.5)
        // Between green (0.5) — the stop at 0.5 is exactly (90, 220, 90).
        XCTAssertEqual(mid, SIMD3<UInt8>(90, 220, 90))
    }

    func testSolidMode() {
        let points = [point(y: 1), point(y: 2)]
        let result = PointColorizer.apply(.solid(SIMD3<UInt8>(255, 0, 0)), to: points)
        XCTAssertEqual(result.count, 2)
        XCTAssertEqual(result[0].color, SIMD3<UInt8>(255, 0, 0))
        XCTAssertEqual(result[1].color, SIMD3<UInt8>(255, 0, 0))
        // Positions untouched.
        XCTAssertEqual(result[1].position.y, 2)
    }

    func testCameraColorModeIsPassthrough() {
        let points = [point(y: 1, color: SIMD3<UInt8>(10, 20, 30))]
        let result = PointColorizer.apply(.cameraColor, to: points)
        XCTAssertEqual(result[0].color, SIMD3<UInt8>(10, 20, 30))
    }

    func testConfidenceModeBlendsTrafficLightTintOverCameraColor() {
        let base = SIMD3<UInt8>(120, 120, 120)
        let low = PointColorizer.apply(.confidence, to: [PointCloudPoint(position: .zero, color: base, confidence: 0)])[0].color
        let mid = PointColorizer.apply(.confidence, to: [PointCloudPoint(position: .zero, color: base, confidence: 1)])[0].color
        let high = PointColorizer.apply(.confidence, to: [PointCloudPoint(position: .zero, color: base, confidence: 2)])[0].color

        XCTAssertGreaterThan(low.x, low.z)    // low confidence reads red
        XCTAssertGreaterThan(high.y, high.x)  // high confidence reads green
        XCTAssertGreaterThan(mid.x, mid.z)    // medium reads amber
        XCTAssertGreaterThan(mid.y, mid.z)
        // The camera color still contributes — a black point is not pure tint.
        let darkLow = PointColorizer.apply(.confidence, to: [PointCloudPoint(position: .zero, color: .zero, confidence: 0)])[0].color
        XCTAssertLessThan(darkLow.x, 235)
        XCTAssertEqual(PointColorizer.apply(.confidence, to: [PointCloudPoint(position: .zero, color: base, confidence: 0)])[0].confidence, 0)
    }

    func testHeightGradientMapsBounds() {
        let points = [point(y: 0), point(y: 10), point(y: 5)]
        let result = PointColorizer.apply(.heightGradient, to: points)
        XCTAssertEqual(result[0].color, PointColorizer.heightRamp(0))
        XCTAssertEqual(result[1].color, PointColorizer.heightRamp(1))
        XCTAssertEqual(result[2].color, PointColorizer.heightRamp(0.5))
    }

    func testHeightGradientFlatRangeUsesMidpoint() {
        let points = [point(y: 3), point(y: 3)]
        let result = PointColorizer.apply(.heightGradient, to: points)
        XCTAssertEqual(result[0].color, PointColorizer.heightRamp(0.5))
    }

    func testEmptyInput() {
        XCTAssertTrue(PointColorizer.apply(.heightGradient, to: []).isEmpty)
        XCTAssertTrue(PointColorizer.apply(.solid(SIMD3<UInt8>(1, 1, 1)), to: []).isEmpty)
        XCTAssertNil(PointColorizer.heightRange(of: []))
    }

    func testHeightRange() {
        let points = [point(y: 3), point(y: -2), point(y: 9)]
        let range = PointColorizer.heightRange(of: points)
        XCTAssertEqual(range?.minY, -2)
        XCTAssertEqual(range?.maxY, 9)
    }

    func testColorTransformCameraPassthrough() {
        let transform = PointColorizer.colorTransform(mode: .cameraColor)
        let input = point(y: 1, color: SIMD3<UInt8>(10, 20, 30))
        XCTAssertEqual(transform(input), SIMD3<UInt8>(10, 20, 30))
    }

    func testColorTransformSolid() {
        let transform = PointColorizer.colorTransform(mode: .solid(SIMD3<UInt8>(5, 6, 7)))
        XCTAssertEqual(transform(point(y: 0)), SIMD3<UInt8>(5, 6, 7))
    }

    func testColorTransformHeightGradientWithGlobalRange() {
        // Global range 0...10: y=0 → ramp(0), y=10 → ramp(1).
        let transform = PointColorizer.colorTransform(mode: .heightGradient, heightRange: (0, 10))
        XCTAssertEqual(transform(point(y: 0)), PointColorizer.heightRamp(0))
        XCTAssertEqual(transform(point(y: 10)), PointColorizer.heightRamp(1))
        XCTAssertEqual(transform(point(y: 5)), PointColorizer.heightRamp(0.5))
    }

    func testColorTransformHeightGradientFlatRangeUsesMidpoint() {
        let transform = PointColorizer.colorTransform(mode: .heightGradient, heightRange: (4, 4))
        XCTAssertEqual(transform(point(y: 4)), PointColorizer.heightRamp(0.5))
    }

    func testDimLowConfidenceDarkensButPreservesHueDirection() {
        let bright = SIMD3<UInt8>(200, 100, 50)
        let transform = PointColorizer.colorTransform(mode: .cameraColor, dimLowConfidence: true)
        let high = transform(PointCloudPoint(position: .zero, color: bright, confidence: 2))
        let low = transform(PointCloudPoint(position: .zero, color: bright, confidence: 0))
        let mid = transform(PointCloudPoint(position: .zero, color: bright, confidence: 1))
        XCTAssertEqual(high, bright, "full confidence is untouched")
        XCTAssertLessThan(low.x, mid.x)
        XCTAssertLessThan(mid.x, high.x)
        // Still recognisably the same color, just darker (red channel stays dominant).
        XCTAssertGreaterThan(low.x, low.y)
    }

    func testDimLowConfidenceSkippedForConfidenceMode() {
        // .confidence mode already encodes confidence as color — dimming on top
        // would muddy the traffic-light read, so the flag is a no-op there.
        let base = SIMD3<UInt8>(120, 120, 120)
        let withDim = PointColorizer.colorTransform(mode: .confidence, dimLowConfidence: true)
        let without = PointColorizer.colorTransform(mode: .confidence, dimLowConfidence: false)
        let p = PointCloudPoint(position: .zero, color: base, confidence: 0)
        XCTAssertEqual(withDim(p), without(p))
    }

    func testColorTransformDefaultsToNoDimming() {
        let transform = PointColorizer.colorTransform(mode: .cameraColor)
        let color = SIMD3<UInt8>(10, 20, 30)
        XCTAssertEqual(transform(PointCloudPoint(position: .zero, color: color, confidence: 0)), color)
    }
}
