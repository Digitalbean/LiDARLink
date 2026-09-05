import Foundation
import simd

/// A colored 3D point in world space.
public struct PointCloudPoint: Sendable, Equatable {
    public var position: SIMD3<Float>
    public var color: SIMD3<UInt8>
    /// ARKit scene-depth confidence: 0 = low, 1 = medium, 2 = high.
    public var confidence: UInt8

    public init(position: SIMD3<Float>, color: SIMD3<UInt8>, confidence: UInt8 = 2) {
        self.position = position
        self.color = color
        self.confidence = min(confidence, 2)
    }
}
