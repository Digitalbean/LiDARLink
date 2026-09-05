import XCTest
import simd
@testable import LiDARLinkShared

final class SurfelRefinementTests: XCTestCase {

    private let k = CameraIntrinsics(fx: 120, fy: 120, cx: 48, cy: 36, imageWidth: 96, imageHeight: 72)

    /// Depth image of a plane at ~1.5 m, optionally with a fine ripple that a
    /// single tile can't capture flat.
    private func depth(_ w: Int, _ h: Int, ripple: Float) -> [Float16] {
        var d = [Float16](repeating: 0, count: w * h)
        for y in 0..<h { for x in 0..<w {
            // ~25 px period — a tile reads nearly planar, but the fit residual
            // and viewpoint-dependent sampling reveal the structure over time.
            let r = ripple * sin(Float(x) * 0.25) * cos(Float(y) * 0.25)
            d[y * w + x] = Float16(1.5 + r)
        }}
        return d
    }

    private func pose(_ dx: Float) -> simd_float4x4 {
        var m = matrix_identity_float4x4
        m.columns.3.x = dx
        return m
    }

    func testFlatSurfaceDoesNotSplit() {
        let map = SurfelMap()
        let flat = depth(96, 72, ripple: 0)
        for i in 0..<12 {
            map.integrate(depth: flat, width: 96, height: 72, depthScale: 1, intrinsics: k,
                          pose: pose(Float(i) * 0.01), confidence: nil, keyframeID: Int32(i))
        }
        XCTAssertTrue(map.orientedSnapshot().allSatisfy { $0.splitLevel == 0 })
    }

    func testRoughSurfaceRefinesOverTime() {
        let map = SurfelMap()
        let rough = depth(96, 72, ripple: 0.03)   // 3 cm ripple — finer than a tile
        let before = { () -> Int in
            map.integrate(depth: rough, width: 96, height: 72, depthScale: 1, intrinsics: k,
                          pose: matrix_identity_float4x4, confidence: nil, keyframeID: 0)
            return map.count
        }()

        for i in 1..<25 {
            map.integrate(depth: rough, width: 96, height: 72, depthScale: 1, intrinsics: k,
                          pose: pose(Float(i) * 0.008), confidence: nil, keyframeID: Int32(i))
        }
        let after = map.orientedSnapshot()
        XCTAssertGreaterThan(map.count, before, "the rough surface grew finer surfels")
        XCTAssertTrue(after.contains { $0.splitLevel > 0 }, "some surfels subdivided")
        XCTAssertTrue(after.allSatisfy { $0.radius >= 0.005 }, "respects the minimum radius")
    }

    func testSurfelCapIsRespected() {
        var params = SurfelMap.Parameters()
        params.maxSurfels = 500
        let map = SurfelMap(parameters: params)
        let rough = depth(96, 72, ripple: 0.04)
        for i in 0..<30 {
            map.integrate(depth: rough, width: 96, height: 72, depthScale: 1, intrinsics: k,
                          pose: pose(Float(i) * 0.01), confidence: nil, keyframeID: Int32(i))
        }
        XCTAssertLessThanOrEqual(map.count, 500)
    }
}
