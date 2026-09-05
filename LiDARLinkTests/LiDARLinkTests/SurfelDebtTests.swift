import XCTest
import simd
@testable import LiDARLinkShared

final class SurfelDebtTests: XCTestCase {

    private let k = CameraIntrinsics(fx: 100, fy: 100, cx: 48, cy: 36, imageWidth: 96, imageHeight: 72)
    private let flat = [Float16](repeating: 1.5, count: 96 * 72)

    /// A plane tilted so a forward-looking camera sees it at a grazing angle.
    private func tilted(slope: Float) -> [Float16] {
        var d = [Float16](repeating: 0, count: 96 * 72)
        for y in 0..<72 { for x in 0..<96 {
            let fx = Float(x - 48) / 96
            d[y * 96 + x] = Float16(1.5 + slope * fx)
        }}
        return d
    }

    func testFewObservationsMeansHighDebt() {
        let map = SurfelMap()
        map.integrate(depth: flat, width: 96, height: 72, depthScale: 1, intrinsics: k,
                      pose: matrix_identity_float4x4, confidence: nil, keyframeID: 0)
        map.updateDebt()
        let d1 = map.meanDebt

        for i in 1..<20 {
            map.integrate(depth: flat, width: 96, height: 72, depthScale: 1, intrinsics: k,
                          pose: matrix_identity_float4x4, confidence: nil, keyframeID: Int32(i))
        }
        map.updateDebt()
        XCTAssertLessThan(map.meanDebt, d1 - 0.1, "more observations pay down debt")
        XCTAssertLessThan(map.meanDebt, 0.35, "a well-observed head-on plane is mostly resolved")
    }

    func testGrazingOnlyViewsKeepDebtHigh() {
        let grazing = SurfelMap()
        let steep = tilted(slope: 2.4)   // ~67° tilt → grazing
        for i in 0..<20 {
            grazing.integrate(depth: steep, width: 96, height: 72, depthScale: 1, intrinsics: k,
                              pose: matrix_identity_float4x4, confidence: nil, keyframeID: Int32(i))
        }
        grazing.updateDebt()

        let headOn = SurfelMap()
        for i in 0..<20 {
            headOn.integrate(depth: flat, width: 96, height: 72, depthScale: 1, intrinsics: k,
                             pose: matrix_identity_float4x4, confidence: nil, keyframeID: Int32(i))
        }
        headOn.updateDebt()

        XCTAssertGreaterThan(grazing.meanDebt, headOn.meanDebt + 0.04,
                             "grazing-only coverage keeps debt above a head-on scan (\(grazing.meanDebt) vs \(headOn.meanDebt))")
    }

    func testBestViewCosReflectsSurfaceTilt() {
        let head = SurfelMap()
        head.integrate(depth: flat, width: 96, height: 72, depthScale: 1, intrinsics: k,
                       pose: matrix_identity_float4x4, confidence: nil, keyframeID: 0)
        let headCos = head.orientedSnapshot().map(\.bestViewCos).reduce(0, +) / Float(max(head.count, 1))

        let tilt = SurfelMap()
        tilt.integrate(depth: tilted(slope: 2.0), width: 96, height: 72, depthScale: 1, intrinsics: k,
                       pose: matrix_identity_float4x4, confidence: nil, keyframeID: 0)
        let tiltCos = tilt.orientedSnapshot().map(\.bestViewCos).reduce(0, +) / Float(max(tilt.count, 1))

        XCTAssertGreaterThan(headCos, 0.84)
        XCTAssertLessThan(tiltCos, headCos - 0.1, "a tilted surface is seen less head-on")
    }
}
