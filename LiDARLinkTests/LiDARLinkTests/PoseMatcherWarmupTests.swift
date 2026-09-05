import XCTest
import simd
@testable import LiDARLinkShared

final class PoseMatcherWarmupTests: XCTestCase {
    func testNearestTransformSelection() {
        var a = matrix_identity_float4x4
        a.columns.3 = simd_float4(1, 0, 0, 1)
        var b = matrix_identity_float4x4
        b.columns.3 = simd_float4(2, 0, 0, 1)
        let samples = [(timestamp: 10.0, transform: a), (timestamp: 20.0, transform: b)]
        let nearA = PoseMatcher.nearestTransform(to: 11.0, in: samples)
        XCTAssertEqual(nearA?.columns.3.x, 1)
        let nearB = PoseMatcher.nearestTransform(to: 19.0, in: samples)
        XCTAssertEqual(nearB?.columns.3.x, 2)
        let exact = PoseMatcher.nearestTransform(to: 20.0, in: samples)
        XCTAssertEqual(exact?.columns.3.x, 2)
    }

    func testNearestTransformEmpty() {
        XCTAssertNil(PoseMatcher.nearestTransform(to: 1.0, in: []))
    }

    func testWarmupGate() {
        let gate = WarmupGate(startMs: 1000, durationMs: 1500)
        XCTAssertFalse(gate.shouldPass(nowMs: 1000))
        XCTAssertFalse(gate.shouldPass(nowMs: 2499))
        XCTAssertTrue(gate.shouldPass(nowMs: 2500))
        XCTAssertTrue(gate.shouldPass(nowMs: 10_000))
    }

    func testWarmupGateZeroDuration() {
        let gate = WarmupGate(startMs: 500, durationMs: 0)
        XCTAssertTrue(gate.shouldPass(nowMs: 500))
    }
}
