import XCTest
import simd
@testable import LiDARLinkShared

final class MotionGateTests: XCTestCase {
    func testAcceptsStillNormalFrame() {
        XCTAssertTrue(MotionGate.shouldAccept(trackingIsNormal: true, translationDeltaMeters: 0.01, rotationDegrees: 0.5, sensitivity: .normal))
    }

    func testRejectsLimitedTracking() {
        XCTAssertFalse(MotionGate.shouldAccept(trackingIsNormal: false, translationDeltaMeters: 0.0, rotationDegrees: 0.0, sensitivity: .relaxed))
    }

    func testRejectsLargeTranslation() {
        XCTAssertFalse(MotionGate.shouldAccept(trackingIsNormal: true, translationDeltaMeters: 0.30, rotationDegrees: 1, sensitivity: .normal))
        XCTAssertTrue(MotionGate.shouldAccept(trackingIsNormal: true, translationDeltaMeters: 0.10, rotationDegrees: 1, sensitivity: .normal))
    }

    func testSensitivityThresholds() {
        // 0.10 m is fine for normal, too much for strict.
        XCTAssertTrue(MotionGate.shouldAccept(trackingIsNormal: true, translationDeltaMeters: 0.10, rotationDegrees: 1, sensitivity: .normal))
        XCTAssertFalse(MotionGate.shouldAccept(trackingIsNormal: true, translationDeltaMeters: 0.10, rotationDegrees: 1, sensitivity: .strict))
        // 0.20 m is too much for normal, fine for relaxed.
        XCTAssertFalse(MotionGate.shouldAccept(trackingIsNormal: true, translationDeltaMeters: 0.20, rotationDegrees: 1, sensitivity: .normal))
        XCTAssertTrue(MotionGate.shouldAccept(trackingIsNormal: true, translationDeltaMeters: 0.20, rotationDegrees: 1, sensitivity: .relaxed))
    }

    func testRejectsLargeRotation() {
        XCTAssertFalse(MotionGate.shouldAccept(trackingIsNormal: true, translationDeltaMeters: 0.01, rotationDegrees: 20, sensitivity: .normal))
        XCTAssertTrue(MotionGate.shouldAccept(trackingIsNormal: true, translationDeltaMeters: 0.01, rotationDegrees: 5, sensitivity: .normal))
    }

    func testRotationDegreesOfIdentity() {
        XCTAssertEqual(MotionGate.rotationDegrees(of: matrix_identity_float3x3), 0, accuracy: 0.001)
    }

    func testRotationDegreesOfQuarterTurn() {
        let rotation = simd_float3x3(rotateX: .pi / 2)
        XCTAssertEqual(MotionGate.rotationDegrees(of: rotation), 90, accuracy: 0.5)
    }

    func testCaptureGateSuppressesStationaryFramesUntilHeartbeat() {
        let gate = CaptureGate()
        XCTAssertEqual(decision(gate, pose: matrix_identity_float4x4, time: 1_000), .accept)
        XCTAssertEqual(decision(gate, pose: matrix_identity_float4x4, time: 1_100), .rejectRedundant)
        XCTAssertEqual(decision(gate, pose: matrix_identity_float4x4, time: 1_500), .accept)
    }

    func testCaptureGateAcceptsUsefulTranslationAndRotation() {
        let gate = CaptureGate()
        XCTAssertEqual(decision(gate, pose: matrix_identity_float4x4, time: 1_000), .accept)
        XCTAssertEqual(decision(gate, pose: pose(x: 0.02), time: 1_100), .accept)

        var rotated = pose(x: 0.02)
        rotated.columns.0 = SIMD4<Float>(cos(.pi / 90), 0, -sin(.pi / 90), 0)
        rotated.columns.2 = SIMD4<Float>(sin(.pi / 90), 0, cos(.pi / 90), 0)
        XCTAssertEqual(decision(gate, pose: rotated, time: 1_200), .accept)
    }

    func testCaptureGateRejectsTrackingAndRecoversAfterFastMotion() {
        let gate = CaptureGate()
        XCTAssertEqual(decision(gate, pose: matrix_identity_float4x4, time: 1_000), .accept)
        XCTAssertEqual(decision(gate, pose: pose(x: 0.30), time: 1_100), .rejectMotion)
        // Motion is measured from the previous attempt, not the old accepted
        // pose, so a stable frame at the new location can be accepted.
        XCTAssertEqual(decision(gate, pose: pose(x: 0.30), time: 1_200), .accept)
        XCTAssertEqual(decision(gate, pose: pose(x: 0.31), time: 1_300, tracking: false), .rejectTracking)
    }

    func testCaptureGateRotationDoesNotCreateFalseTranslationAwayFromOrigin() {
        let gate = CaptureGate()
        var initial = pose(x: 10)
        XCTAssertEqual(decision(gate, pose: initial, time: 1_000), .accept)
        initial.columns.0 = SIMD4<Float>(cos(.pi / 90), 0, -sin(.pi / 90), 0)
        initial.columns.2 = SIMD4<Float>(sin(.pi / 90), 0, cos(.pi / 90), 0)
        XCTAssertEqual(decision(gate, pose: initial, time: 1_100), .accept)
    }

    private func decision(_ gate: CaptureGate,
                          pose: simd_float4x4,
                          time: UInt64,
                          tracking: Bool = true) -> CaptureGateDecision {
        gate.evaluate(pose: pose,
                      timestampMs: time,
                      trackingIsNormal: tracking,
                      sensitivity: .normal,
                      minimumTranslationMeters: 0.015,
                      minimumRotationDegrees: 1.5,
                      heartbeatIntervalMs: 500)
    }

    private func pose(x: Float) -> simd_float4x4 {
        var result = matrix_identity_float4x4
        result.columns.3.x = x
        return result
    }
}

private extension simd_float3x3 {
    init(rotateX angle: Float) {
        let c = cos(angle)
        let s = sin(angle)
        self.init(columns: (SIMD3<Float>(1, 0, 0),
                            SIMD3<Float>(0, c, s),
                            SIMD3<Float>(0, -s, c)))
    }
}
