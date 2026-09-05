import Foundation
import simd
import CoreGraphics
import ImageIO

extension simd_float4x4: Codable {
    public init(from decoder: Decoder) throws {
        var container = try decoder.unkeyedContainer()
        var columns = [simd_float4]()
        columns.reserveCapacity(4)
        for _ in 0..<4 {
            var column = simd_float4()
            for i in 0..<4 {
                column[i] = try container.decode(Float.self)
            }
            columns.append(column)
        }
        self.init(columns)
    }

    public func encode(to encoder: Encoder) throws {
        var container = encoder.unkeyedContainer()
        for i in 0..<4 {
            let column = self[i]
            try container.encode(column.x)
            try container.encode(column.y)
            try container.encode(column.z)
            try container.encode(column.w)
        }
    }
}

/// A closure that returns the RGB color for a pixel coordinate (origin top-left).
public typealias ColorSampler = (Int, Int) -> SIMD3<UInt8>

/// Depth-to-point and point-cloud construction math (shared and unit-tested).
public enum PoseMath {
    /// Unprojects a depth pixel into camera space. ARKit camera space is +x right,
    /// +y up, -z forward, so the depth value maps to -z.
    public static func cameraPoint(pixelX: Float, pixelY: Float, depth: Float, intrinsics: CameraIntrinsics) -> SIMD3<Float> {
        let x = (pixelX - intrinsics.cx) * depth / intrinsics.fx
        // Pixel coordinates grow downward, opposite camera-space +Y.
        let y = (intrinsics.cy - pixelY) * depth / intrinsics.fy
        return SIMD3<Float>(x, y, -depth)
    }

    /// Transforms a camera-space point into world space using the camera pose.
    public static func worldPoint(cameraPoint: SIMD3<Float>, pose: simd_float4x4) -> SIMD3<Float> {
        let p = pose * simd_float4(cameraPoint, 1)
        return SIMD3<Float>(p.x, p.y, p.z)
    }

    public static func buildPointCloud(depthPayload: DepthPayload,
                                       depthScale: Float,
                                       intrinsics: CameraIntrinsics,
                                       pose: simd_float4x4,
                                       step: Int = 1,
                                       minConfidence: UInt8 = 0,
                                       edgeFilter: Bool = false,
                                       colorSampler: ColorSampler? = nil) -> [PointCloudPoint] {
        let depth = depthPayload.float16Array()
        return buildPointCloud(depth: depth,
                               width: depthPayload.width,
                               height: depthPayload.height,
                               depthScale: depthScale,
                               intrinsics: intrinsics,
                               pose: pose,
                               step: step,
                               minConfidence: minConfidence,
                               edgeFilter: edgeFilter,
                               confidence: depthPayload.confidenceArray(),
                               colorSampler: colorSampler)
    }

    public static func buildPointCloud(depth: [Float16],
                                       width: Int,
                                       height: Int,
                                       depthScale: Float,
                                       intrinsics: CameraIntrinsics,
                                       pose: simd_float4x4,
                                       step: Int = 1,
                                       minConfidence: UInt8 = 0,
                                       edgeFilter: Bool = false,
                                       confidence: [UInt8]? = nil,
                                       colorSampler: ColorSampler? = nil) -> [PointCloudPoint] {
        precondition(depth.count >= width * height)
        let stride = max(step, 1)
        var points: [PointCloudPoint] = []
        points.reserveCapacity((width / stride + 1) * (height / stride + 1))

        // Optional depth-edge / flying-pixel rejection, computed once at full
        // resolution over the whole frame (independent of `step`).
        var edgeRejection: [Bool]? = nil
        if edgeFilter {
            var metric = [Float](repeating: 0, count: width * height)
            for i in 0..<(width * height) {
                metric[i] = Float(depth[i]) * depthScale
            }
            edgeRejection = DepthEdgeFilter.rejectionMask(depth: metric, width: width, height: height)
        }

        var y = 0
        while y < height {
            var x = 0
            while x < width {
                let index = y * width + x
                if minConfidence > 0, let confidence, confidence[index] < minConfidence {
                    x += stride
                    continue
                }
                if let edgeRejection, edgeRejection[index] {
                    x += stride
                    continue
                }
                let raw = Float(depth[index]) * depthScale
                if raw.isFinite && raw > 0 {
                    let cam = cameraPoint(pixelX: Float(x), pixelY: Float(y), depth: raw, intrinsics: intrinsics)
                    let world = worldPoint(cameraPoint: cam, pose: pose)
                    let color = colorSampler?(x, y) ?? SIMD3<UInt8>(150, 190, 220)
                    points.append(PointCloudPoint(position: world,
                                                  color: color,
                                                  confidence: confidence?[index] ?? 2))
                }
                x += stride
            }
            y += stride
        }
        return points
    }
}

/// Decodes a JPEG color payload once and samples pixel colors (RGBA8, origin top-left).
public final class ColorImageSampler: Sendable {
    public let width: Int
    public let height: Int
    private let pixels: [UInt8]
    private let bytesPerRow: Int

    public init?(color: ColorPayload) {
        guard let provider = CGDataProvider(data: color.jpegData as CFData),
              let source = CGImageSourceCreateWithDataProvider(provider, nil),
              let image = CGImageSourceCreateImageAtIndex(source, 0, nil) else { return nil }

        width = image.width
        height = image.height
        bytesPerRow = width * 4

        var buffer = [UInt8](repeating: 0, count: height * bytesPerRow)
        guard let context = CGContext(data: &buffer,
                                      width: width,
                                      height: height,
                                      bitsPerComponent: 8,
                                      bytesPerRow: bytesPerRow,
                                      space: CGColorSpaceCreateDeviceRGB(),
                                      bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue) else { return nil }
        // Preserve the decoded image's row order. The depth map and captured
        // camera image belong to the same ARFrame, so applying an extra vertical
        // transform here would paint geometry with colors from the opposite row.
        context.draw(image, in: CGRect(x: 0, y: 0, width: width, height: height))
        pixels = buffer
    }

    public func color(atX x: Int, y: Int) -> SIMD3<UInt8> {
        let cx = min(max(x, 0), width - 1)
        let cy = min(max(y, 0), height - 1)
        let offset = cy * bytesPerRow + cx * 4
        return SIMD3<UInt8>(pixels[offset], pixels[offset + 1], pixels[offset + 2])
    }
}
