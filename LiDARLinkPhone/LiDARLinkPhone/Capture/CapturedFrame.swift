import ARKit
import CoreVideo
import Foundation
import simd
import LiDARLinkShared

enum CaptureTrackingIssue: Equatable, Sendable {
    case none
    case initializing
    case excessiveMotion
    case insufficientFeatures
    case relocalizing
    case unavailable

    var message: String {
        switch self {
        case .none: return ""
        case .initializing: return "Initializing"
        case .excessiveMotion: return "Excessive motion"
        case .insufficientFeatures: return "Not enough visual features"
        case .relocalizing: return "Relocalizing"
        case .unavailable: return "Unavailable"
        }
    }
}

/// A single ARKit frame snapshot, produced on the capture queue and consumed
/// synchronously by the frame processor on the same queue.
struct CapturedFrame {
    let cameraImage: CVPixelBuffer
    let depthBuffer: CVPixelBuffer?
    let confidenceBuffer: CVPixelBuffer?
    let depthIsSmoothed: Bool
    let pose: simd_float4x4
    let intrinsics: CameraIntrinsics
    let timestampMs: UInt64
    let meshAnchors: [ARMeshAnchor]
    /// Mesh-anchor IDs ARKit removed since the previous frame (merged/discarded).
    var removedMeshAnchorIDs: [UUID] = []
    /// Detailed ARKit tracking state for capture gating and user guidance.
    let trackingIssue: CaptureTrackingIssue
    /// Approximate visible-light intensity in lumens. LiDAR depth still works in
    /// darkness, but camera color quality degrades.
    let ambientIntensity: Double?

    var trackingIsNormal: Bool { trackingIssue == .none }

    /// Camera translation in world space (for motion-gated temporal smoothing).
    var poseTranslation: SIMD3<Float> {
        SIMD3<Float>(pose.columns.3.x, pose.columns.3.y, pose.columns.3.z)
    }
}
