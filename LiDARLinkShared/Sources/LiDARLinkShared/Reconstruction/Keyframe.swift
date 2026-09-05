import Foundation
import simd

/// The atomic unit of a v2 scan: one posed RGB-D observation the phone chose to
/// keep. Everything downstream — the pose graph, the TSDF, the texture bake —
/// is built from keyframes.
public struct Keyframe: Sendable, Identifiable, Equatable {
    /// Sequential id assigned by the capturer.
    public let id: Int32
    public let capturedAtMs: UInt64

    /// Downsampled scene depth + per-pixel confidence, and its intrinsics.
    public var depth: DepthPayload
    public var depthScale: Float
    public var depthIntrinsics: CameraIntrinsics

    /// Full-resolution colour still (`captureHighResolutionFrame`), and its
    /// intrinsics. Optional — a keyframe can be depth-only.
    public var colorJPEG: Data?
    public var colorIntrinsics: CameraIntrinsics?

    /// ARKit camera-to-world at capture — the pose-graph prior, never trusted
    /// as final.
    public var arkitPose: simd_float4x4
    /// ARKit's relative transform from the previous keyframe to this one — the
    /// sequential odometry constraint.
    public var relativePose: simd_float4x4

    /// Pose-independent "what does this keyframe see" signature — computed
    /// once here so loop-closure candidate search never re-derives it (or the
    /// full unprojected cloud) just to decide whether a pair is worth trying.
    public let descriptor: KeyframeDescriptor

    public init(id: Int32,
                capturedAtMs: UInt64,
                depth: DepthPayload,
                depthScale: Float,
                depthIntrinsics: CameraIntrinsics,
                colorJPEG: Data? = nil,
                colorIntrinsics: CameraIntrinsics? = nil,
                arkitPose: simd_float4x4,
                relativePose: simd_float4x4) {
        self.id = id
        self.capturedAtMs = capturedAtMs
        self.depth = depth
        self.depthScale = depthScale
        self.depthIntrinsics = depthIntrinsics
        self.colorJPEG = colorJPEG
        self.colorIntrinsics = colorIntrinsics
        self.arkitPose = arkitPose
        self.relativePose = relativePose
        self.descriptor = KeyframeDescriptor.compute(depth: depth, depthScale: depthScale,
                                                      confidence: depth.confidenceArray())
    }
}
