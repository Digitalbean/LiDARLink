import XCTest
import simd
@testable import LiDARLinkShared

final class SurfelExtractorTests: XCTestCase {

    private let k = CameraIntrinsics(fx: 120, fy: 120, cx: 64, cy: 48, imageWidth: 128, imageHeight: 96)

    func testPlaneBecomesFewLargePatches() {
        let w = 128, h = 96
        let depth = [Float16](repeating: 2.0, count: w * h)
        let surfels = SurfelExtractor.extract(depth: depth, width: w, height: h, depthScale: 1,
                                              intrinsics: k, pose: matrix_identity_float4x4,
                                              confidence: nil, keyframeID: 0)
        // 128×96 / 5² ≈ 500 tiles; a flat plane keeps them whole.
        XCTAssertGreaterThan(surfels.count, 150)
        XCTAssertLessThan(surfels.count, 560)
        for s in surfels {
            XCTAssertEqual(simd_length(s.normal), 1, accuracy: 1e-3)
            XCTAssertGreaterThan(s.normal.z, 0.9, "faces the camera")
            XCTAssertEqual(s.position.z, -2.0, accuracy: 0.05)
            XCTAssertGreaterThan(s.radius, 0.015)
        }
    }

    func testDiscontinuityTileSplitsOrDrops() {
        let w = 64, h = 64
        var depth = [Float16](repeating: 1.5, count: w * h)
        // A hard step down the middle: left half at 1.5 m, right half at 3.0 m.
        for y in 0..<h { for x in (w / 2)..<w { depth[y * w + x] = 3.0 } }
        let surfels = SurfelExtractor.extract(depth: depth, width: w, height: h, depthScale: 1,
                                              intrinsics: k, pose: matrix_identity_float4x4,
                                              confidence: nil, keyframeID: 0)
        // Every surviving patch sits cleanly on one side, never bridging the step.
        for s in surfels {
            let d = -s.position.z
            XCTAssertTrue(abs(d - 1.5) < 0.2 || abs(d - 3.0) < 0.2, "patch \(d) bridges the discontinuity")
        }
        XCTAssertGreaterThan(surfels.count, 20)
    }

    func testConfidenceGateDropsLowConfidencePixels() {
        let w = 64, h = 64
        let depth = [Float16](repeating: 1.5, count: w * h)
        let conf = [UInt8](repeating: 0, count: w * h)   // all low
        var params = SurfelExtractor.Parameters()
        params.minConfidence = 2
        let surfels = SurfelExtractor.extract(depth: depth, width: w, height: h, depthScale: 1,
                                              intrinsics: k, pose: matrix_identity_float4x4,
                                              confidence: conf, keyframeID: 0, parameters: params)
        XCTAssertTrue(surfels.isEmpty)
    }
}
