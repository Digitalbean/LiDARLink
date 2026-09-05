import Foundation
import simd

/// How aggressively frames are rejected during camera motion.
public enum MotionSensitivity: String, CaseIterable, Sendable, Hashable {
    case relaxed
    case normal
    case strict

    public var translationThresholdMeters: Float {
        switch self {
        case .relaxed: return 0.25
        case .normal: return 0.15
        case .strict: return 0.08
        }
    }

    public var rotationThresholdDegrees: Float {
        switch self {
        case .relaxed: return 12
        case .normal: return 8
        case .strict: return 5
        }
    }

    public var label: String {
        switch self {
        case .relaxed: return "Relaxed"
        case .normal: return "Normal"
        case .strict: return "Strict"
        }
    }
}

/// Motion-aware frame quality gate: rejects frames when AR tracking is limited
/// or the camera moved too far/fast since the last accepted frame (blur and
/// misalignment risk). Rejected frames never reach the map.
public enum MotionGate {
    public static func shouldAccept(trackingIsNormal: Bool,
                                    translationDeltaMeters: Float,
                                    rotationDegrees: Float,
                                    sensitivity: MotionSensitivity) -> Bool {
        guard trackingIsNormal else { return false }
        guard translationDeltaMeters <= sensitivity.translationThresholdMeters else { return false }
        guard rotationDegrees <= sensitivity.rotationThresholdDegrees else { return false }
        return true
    }

    /// Rotation angle in degrees of a relative rotation matrix (identity → 0).
    public static func rotationDegrees(of relative: simd_float3x3) -> Float {
        let trace = relative.columns.0.x + relative.columns.1.y + relative.columns.2.z
        let clamped = min(max((trace - 1) / 2, -1), 1)
        return acos(clamped) * 180 / .pi
    }
}

public enum CaptureGateDecision: Equatable, Sendable {
    case accept
    case rejectTracking
    case rejectMotion
    case rejectRedundant
}

/// Stateful two-pose capture gate. Consecutive attempts are used to detect
/// motion spikes, while the last emitted pose is used to decide whether a new
/// spatial keyframe is useful. Keeping those references separate lets capture
/// recover immediately after a fast movement.
public final class CaptureGate {
    private var previousAttemptPose: simd_float4x4?
    private var lastEmittedPose: simd_float4x4?
    private var lastEmittedTimestampMs: UInt64 = 0

    public init() {}

    public func evaluate(pose: simd_float4x4,
                         timestampMs: UInt64,
                         trackingIsNormal: Bool,
                         sensitivity: MotionSensitivity,
                         minimumTranslationMeters: Float,
                         minimumRotationDegrees: Float,
                         heartbeatIntervalMs: UInt64) -> CaptureGateDecision {
        let priorAttempt = previousAttemptPose
        previousAttemptPose = pose

        guard trackingIsNormal else { return .rejectTracking }
        if let priorAttempt {
            let delta = Self.poseDelta(from: priorAttempt, to: pose)
            guard MotionGate.shouldAccept(trackingIsNormal: true,
                                          translationDeltaMeters: delta.translation,
                                          rotationDegrees: delta.rotationDegrees,
                                          sensitivity: sensitivity) else {
                return .rejectMotion
            }
        }

        guard let lastEmittedPose else {
            markEmitted(pose: pose, timestampMs: timestampMs)
            return .accept
        }

        let sinceLastEmission = timestampMs &- lastEmittedTimestampMs
        let delta = Self.poseDelta(from: lastEmittedPose, to: pose)
        guard delta.translation >= max(minimumTranslationMeters, 0)
                || delta.rotationDegrees >= max(minimumRotationDegrees, 0)
                || sinceLastEmission >= heartbeatIntervalMs else {
            return .rejectRedundant
        }

        markEmitted(pose: pose, timestampMs: timestampMs)
        return .accept
    }

    public func reset() {
        previousAttemptPose = nil
        lastEmittedPose = nil
        lastEmittedTimestampMs = 0
    }

    private func markEmitted(pose: simd_float4x4, timestampMs: UInt64) {
        lastEmittedPose = pose
        lastEmittedTimestampMs = timestampMs
    }

    private static func poseDelta(from oldPose: simd_float4x4,
                                  to newPose: simd_float4x4) -> (translation: Float, rotationDegrees: Float) {
        let oldPosition = SIMD3<Float>(oldPose.columns.3.x, oldPose.columns.3.y, oldPose.columns.3.z)
        let newPosition = SIMD3<Float>(newPose.columns.3.x, newPose.columns.3.y, newPose.columns.3.z)
        let translation = simd_length(newPosition - oldPosition)
        let oldRotation = rotationMatrix(of: oldPose)
        let newRotation = rotationMatrix(of: newPose)
        return (translation, MotionGate.rotationDegrees(of: newRotation * oldRotation.transpose))
    }

    private static func rotationMatrix(of pose: simd_float4x4) -> simd_float3x3 {
        simd_float3x3(
            SIMD3<Float>(pose.columns.0.x, pose.columns.0.y, pose.columns.0.z),
            SIMD3<Float>(pose.columns.1.x, pose.columns.1.y, pose.columns.1.z),
            SIMD3<Float>(pose.columns.2.x, pose.columns.2.y, pose.columns.2.z))
    }
}
