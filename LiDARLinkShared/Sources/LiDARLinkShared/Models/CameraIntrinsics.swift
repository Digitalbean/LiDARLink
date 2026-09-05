import Foundation
import simd

public struct CameraIntrinsics: Codable, Equatable, Sendable {
    public var fx: Float
    public var fy: Float
    public var cx: Float
    public var cy: Float
    public var imageWidth: Int
    public var imageHeight: Int

    public init(fx: Float, fy: Float, cx: Float, cy: Float, imageWidth: Int, imageHeight: Int) {
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.imageWidth = imageWidth
        self.imageHeight = imageHeight
    }

    /// Builds intrinsics from an ARKit 3x3 intrinsic matrix (pixels, camera image space).
    public init(matrix: simd_float3x3, imageWidth: Int, imageHeight: Int) {
        self.fx = matrix[0][0]
        self.fy = matrix[1][1]
        self.cx = matrix[2][0]
        self.cy = matrix[2][1]
        self.imageWidth = imageWidth
        self.imageHeight = imageHeight
    }

    /// Intrinsics scaled for a different image resolution.
    public func scaled(to width: Int, height: Int) -> CameraIntrinsics {
        let sx = Float(width) / Float(imageWidth)
        let sy = Float(height) / Float(imageHeight)
        return CameraIntrinsics(fx: fx * sx, fy: fy * sy, cx: cx * sx, cy: cy * sy, imageWidth: width, imageHeight: height)
    }

    /// Vertical field of view in degrees, from `fy` and the image height.
    public var verticalFieldOfViewDegrees: Float {
        guard fy > 0, imageHeight > 0 else { return 60 }
        return 2 * atan(Float(imageHeight) / (2 * fy)) * 180 / .pi
    }
}
