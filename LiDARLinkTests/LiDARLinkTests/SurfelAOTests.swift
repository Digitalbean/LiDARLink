import XCTest
import simd
@testable import LiDARLinkShared

final class SurfelAOTests: XCTestCase {

    private let k = CameraIntrinsics(fx: 100, fy: 100, cx: 48, cy: 36, imageWidth: 96, imageHeight: 72)

    func testOpenSurfaceStaysMostlyUnoccluded() {
        let map = SurfelMap()
        let flat = [Float16](repeating: 1.5, count: 96 * 72)
        for i in 0..<4 {
            map.integrate(depth: flat, width: 96, height: 72, depthScale: 1, intrinsics: k,
                          pose: { var m = matrix_identity_float4x4; m.columns.3.x = Float(i) * 0.01; return m }(),
                          confidence: nil, keyframeID: Int32(i))
        }
        map.updateAmbientOcclusion()
        let aos = map.orientedSnapshot().map(\.ao)
        XCTAssertTrue(aos.allSatisfy { $0 >= 0 && $0 <= 1 })
        XCTAssertGreaterThan(aos.reduce(0, +) / Float(aos.count), 0.8, "a lone flat plane is mostly open")
    }

    /// A box interior: three walls of a cube around the camera. Surfels in the
    /// concave corners must come out darker than surfels mid-wall.
    func testBoxInteriorCornersAreDarker() {
        let map = SurfelMap()
        // Depth image of a corner: two walls meeting down the middle column.
        var d = [Float16](repeating: 0, count: 96 * 72)
        for y in 0..<72 { for x in 0..<96 {
            let fx = Float(x - 48) / 100          // world-ish x offset per unit depth
            // left wall recedes to the left, right wall to the right; both ~1 m off.
            d[y * 96 + x] = Float16(1.0 + 1.6 * abs(fx))
        }}
        for i in 0..<6 {
            map.integrate(depth: d, width: 96, height: 72, depthScale: 1, intrinsics: k,
                          pose: { var m = matrix_identity_float4x4
                                  m.columns.3.x = Float(i) * 0.015; return m }(),
                          confidence: nil, keyframeID: Int32(i))
        }
        var p = SurfelMap.AOParameters()
        p.radius = 1.5
        map.updateAmbientOcclusion(parameters: p)

        let snap = map.orientedSnapshot()
        guard snap.count > 50 else { return XCTFail("too few surfels (\(snap.count))") }
        let nearSeam = snap.filter { abs($0.position.x) < 0.12 }
        let midWall = snap.filter { abs($0.position.x) > 0.35 }
        guard nearSeam.count >= 3, midWall.count >= 3 else {
            return XCTFail("regions underpopulated: seam \(nearSeam.count), wall \(midWall.count)")
        }
        let seamAO = nearSeam.map(\.ao).reduce(0, +) / Float(nearSeam.count)
        let wallAO = midWall.map(\.ao).reduce(0, +) / Float(midWall.count)
        XCTAssertLessThan(seamAO, wallAO, "the concave seam is more occluded than open wall")
        XCTAssertTrue(snap.contains { $0.ao < 0.9 }, "AO actually fires somewhere")
    }
}
