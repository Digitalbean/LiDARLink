import Foundation
import simd

/// A fully assembled frame as seen by the receiver: synchronized pose, intrinsics,
/// depth, and optional color. This is the unit of display, recording, and export.
public struct ScanFrame: Sendable, Equatable {
    public var sequence: UInt32
    public var captureTimestampMs: UInt64
    public var pose: simd_float4x4
    public var intrinsics: CameraIntrinsics
    public var depth: DepthPayload?
    public var color: ColorPayload?
    public var depthIsSmoothed: Bool
    public var depthScale: Float

    public init(sequence: UInt32,
                captureTimestampMs: UInt64,
                pose: simd_float4x4,
                intrinsics: CameraIntrinsics,
                depth: DepthPayload?,
                color: ColorPayload?,
                depthIsSmoothed: Bool,
                depthScale: Float) {
        self.sequence = sequence
        self.captureTimestampMs = captureTimestampMs
        self.pose = pose
        self.intrinsics = intrinsics
        self.depth = depth
        self.color = color
        self.depthIsSmoothed = depthIsSmoothed
        self.depthScale = depthScale
    }
}
