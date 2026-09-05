import Foundation
import simd
@testable import LiDARLinkShared

/// Shared fixtures for tests.
enum TestFixtures {
    static func intrinsics(width: Int = 640, height: Int = 480) -> CameraIntrinsics {
        CameraIntrinsics(fx: 500, fy: 500, cx: 320, cy: 240, imageWidth: width, imageHeight: height)
    }

    static func pose() -> simd_float4x4 {
        var m = matrix_identity_float4x4
        m.columns.3 = simd_float4(1, 2, 3, 1)
        return m
    }

    static func depthPayload(width: Int = 4, height: Int = 4) -> DepthPayload {
        let values = (0..<(width * height)).map { Float16(Float($0 + 1) / 10) }
        let confidence = [UInt8](repeating: 2, count: width * height)
        return DepthPayload(width: width,
                            height: height,
                            depthData: Binary.data(from: values),
                            confidenceData: Binary.data(from: confidence))
    }

    static func frame(sequence: UInt32, depth: DepthPayload? = nil, color: ColorPayload? = nil) -> ScanFrame {
        ScanFrame(sequence: sequence,
                  captureTimestampMs: UInt64(sequence) * 100,
                  pose: pose(),
                  intrinsics: intrinsics(),
                  depth: depth ?? depthPayload(),
                  color: color,
                  depthIsSmoothed: false,
                  depthScale: 1.0)
    }

    static func mesh() -> MeshData {
        MeshData(anchorID: UUID(),
                 vertices: [SIMD3<Float>(0, 0, 0), SIMD3<Float>(1, 0, 0), SIMD3<Float>(0, 1, 0), SIMD3<Float>(1, 1, 0)],
                 normals: [SIMD3<Float>(0, 0, 1), SIMD3<Float>(0, 0, 1), SIMD3<Float>(0, 0, 1), SIMD3<Float>(0, 0, 1)],
                 faces: [0, 1, 2, 1, 3, 2],
                 transform: matrix_identity_float4x4,
                 updatedAtMs: 1234)
    }
}
