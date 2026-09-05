import XCTest
import simd
@testable import LiDARLinkShared

final class PoseMathTests: XCTestCase {
    func testCameraPointUnprojection() {
        let intrinsics = CameraIntrinsics(fx: 500, fy: 500, cx: 320, cy: 240, imageWidth: 640, imageHeight: 480)
        // Center pixel, depth 2 → (0, 0, -2)
        let point = PoseMath.cameraPoint(pixelX: 320, pixelY: 240, depth: 2, intrinsics: intrinsics)
        XCTAssertEqual(point.x, 0, accuracy: 0.001)
        XCTAssertEqual(point.y, 0, accuracy: 0.001)
        XCTAssertEqual(point.z, -2, accuracy: 0.001)
        // Offset pixel: x = (340 - 320) * 2 / 500 = 0.08
        let offset = PoseMath.cameraPoint(pixelX: 340, pixelY: 240, depth: 2, intrinsics: intrinsics)
        XCTAssertEqual(offset.x, 0.08, accuracy: 0.001)
        XCTAssertEqual(offset.y, 0, accuracy: 0.001)

        // Image rows increase downward, while ARKit camera-space Y increases up.
        let aboveCenter = PoseMath.cameraPoint(pixelX: 320, pixelY: 220, depth: 2, intrinsics: intrinsics)
        XCTAssertEqual(aboveCenter.y, 0.08, accuracy: 0.001)
        let belowCenter = PoseMath.cameraPoint(pixelX: 320, pixelY: 260, depth: 2, intrinsics: intrinsics)
        XCTAssertEqual(belowCenter.y, -0.08, accuracy: 0.001)
    }

    func testWorldPointAppliesTranslation() {
        var pose = matrix_identity_float4x4
        pose.columns.3 = simd_float4(1, 2, 3, 1)
        let world = PoseMath.worldPoint(cameraPoint: SIMD3<Float>(0, 0, -2), pose: pose)
        XCTAssertEqual(world, SIMD3<Float>(1, 2, 1))
    }

    func testBuildPointCloudCountAndColors() {
        let intrinsics = CameraIntrinsics(fx: 10, fy: 10, cx: 5, cy: 5, imageWidth: 10, imageHeight: 10)
        let depth = [Float16](repeating: 2, count: 100)
        let points = PoseMath.buildPointCloud(depth: depth, width: 10, height: 10, depthScale: 1, intrinsics: intrinsics, pose: matrix_identity_float4x4, step: 1)
        XCTAssertEqual(points.count, 100)
        let points2 = PoseMath.buildPointCloud(depth: depth, width: 10, height: 10, depthScale: 1, intrinsics: intrinsics, pose: matrix_identity_float4x4, step: 2)
        XCTAssertEqual(points2.count, 25)
    }

    func testInvalidDepthFilteredOut() {
        let intrinsics = CameraIntrinsics(fx: 10, fy: 10, cx: 5, cy: 5, imageWidth: 10, imageHeight: 10)
        var depth = [Float16](repeating: 2, count: 100)
        depth[0] = 0          // invalid (<= 0)
        depth[1] = .nan       // invalid
        let points = PoseMath.buildPointCloud(depth: depth, width: 10, height: 10, depthScale: 1, intrinsics: intrinsics, pose: matrix_identity_float4x4, step: 1)
        XCTAssertEqual(points.count, 98)
    }

    func testColorSamplerApplied() {
        let intrinsics = CameraIntrinsics(fx: 10, fy: 10, cx: 5, cy: 5, imageWidth: 10, imageHeight: 10)
        let depth = [Float16](repeating: 2, count: 100)
        let points = PoseMath.buildPointCloud(depth: depth, width: 10, height: 10, depthScale: 1, intrinsics: intrinsics, pose: matrix_identity_float4x4, step: 1) { _, _ in
            SIMD3<UInt8>(255, 0, 0)
        }
        XCTAssertEqual(points.first?.color, SIMD3<UInt8>(255, 0, 0))
    }

    func testMinConfidenceFiltersLowConfidencePoints() {
        let intrinsics = CameraIntrinsics(fx: 10, fy: 10, cx: 5, cy: 5, imageWidth: 10, imageHeight: 10)
        let depth = [Float16](repeating: 2, count: 4)
        let confidence: [UInt8] = [0, 1, 2, 1]
        let all = PoseMath.buildPointCloud(depth: depth, width: 2, height: 2, depthScale: 1,
                                           intrinsics: intrinsics, pose: matrix_identity_float4x4,
                                           step: 1, confidence: confidence)
        XCTAssertEqual(all.count, 4)
        let filtered = PoseMath.buildPointCloud(depth: depth, width: 2, height: 2, depthScale: 1,
                                                intrinsics: intrinsics, pose: matrix_identity_float4x4,
                                                step: 1, minConfidence: 1, confidence: confidence)
        XCTAssertEqual(filtered.count, 3)
        let highOnly = PoseMath.buildPointCloud(depth: depth, width: 2, height: 2, depthScale: 1,
                                                intrinsics: intrinsics, pose: matrix_identity_float4x4,
                                                step: 1, minConfidence: 2, confidence: confidence)
        XCTAssertEqual(highOnly.count, 1)
        XCTAssertEqual(all.map(\.confidence), [0, 1, 2, 1])
        XCTAssertEqual(filtered.map(\.confidence), [1, 2, 1])
    }

    func testMissingConfidenceDefaultsToHigh() {
        let intrinsics = CameraIntrinsics(fx: 10, fy: 10, cx: 0, cy: 0, imageWidth: 1, imageHeight: 1)
        let points = PoseMath.buildPointCloud(depth: [Float16(1)], width: 1, height: 1,
                                              depthScale: 1, intrinsics: intrinsics,
                                              pose: matrix_identity_float4x4)
        XCTAssertEqual(points.first?.confidence, 2)
    }

    func testDepthScaleApplied() {
        let intrinsics = CameraIntrinsics(fx: 10, fy: 10, cx: 5, cy: 5, imageWidth: 10, imageHeight: 10)
        let depth = [Float16](repeating: 1, count: 100)
        let points = PoseMath.buildPointCloud(depth: depth, width: 10, height: 10, depthScale: 3, intrinsics: intrinsics, pose: matrix_identity_float4x4, step: 1)
        // camera-space z = -depth * scale = -3
        XCTAssertEqual(Double(points.first?.position.z ?? 0), -3, accuracy: 0.01)
    }
}
