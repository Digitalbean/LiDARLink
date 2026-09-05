import XCTest
import simd
@testable import LiDARLinkShared

final class SurfelCompactTests: XCTestCase {

    private let k = CameraIntrinsics(fx: 100, fy: 100, cx: 48, cy: 36, imageWidth: 96, imageHeight: 72)

    /// Integrate the same flat plane from a set of poses spaced far enough that
    /// each keyframe's surfels land between (not on) the previous ones and do
    /// not merge — a redundant, overlapping multi-layer pile.
    private func pileUp() -> SurfelMap {
        let map = SurfelMap()
        let flat = [Float16](repeating: 1.5, count: 96 * 72)
        let offsets: [Float] = [0, 0.031, 0.017, 0.048, 0.009, 0.038]
        for (i, dx) in offsets.enumerated() {
            map.integrate(depth: flat, width: 96, height: 72, depthScale: 1, intrinsics: k,
                          pose: { var m = matrix_identity_float4x4; m.columns.3.x = dx; return m }(),
                          confidence: nil, keyframeID: Int32(i))
        }
        return map
    }

    /// Mean areal overlap: for a sample of surfels, the summed disk-overlap
    /// with neighbours, normalised by the surfel's own area.
    private func overlap(_ m: SurfelMap) -> Float {
        let s = m.orientedSnapshot()
        guard s.count > 20 else { return 0 }
        var total: Float = 0, checked: Float = 0
        for i in stride(from: 0, to: s.count, by: 5) {
            checked += 1
            for j in 0..<s.count where j != i {
                let d = simd_length(s[i].position - s[j].position)
                let sum = s[i].radius + s[j].radius
                if d < sum { total += (sum - d) / sum }   // fractional overlap
            }
        }
        return total / max(checked, 1)
    }

    func testRelaxationReducesOverlap() {
        let map = pileUp()
        let before = map.count
        let overlapBefore = overlap(map)
        XCTAssertGreaterThan(overlapBefore, 0.5, "the raw map has heavy disk overlap")

        for _ in 0..<6 { map.compact() }

        XCTAssertLessThan(overlap(map), overlapBefore * 0.6, "overlap dropped substantially")
        // Still a covered plane, not collapsed away.
        XCTAssertGreaterThan(map.count, before / 5)
    }

    func testRelaxationKeepsSurfelsOnTheirPlane() {
        let map = pileUp()
        for _ in 0..<6 { map.compact() }
        for s in map.orientedSnapshot() {
            XCTAssertEqual(s.position.z, -1.5, accuracy: 0.03, "stayed on the z = -1.5 plane")
        }
    }

    func testDoesNotTouchAnIsolatedSurfelRegion() {
        let map = SurfelMap()
        let sparse = [Float16](repeating: 2.0, count: 96 * 72)
        map.integrate(depth: sparse, width: 96, height: 72, depthScale: 1, intrinsics: k,
                      pose: matrix_identity_float4x4, confidence: nil, keyframeID: 0)
        let before = map.orientedSnapshot()
        map.compact()
        let after = map.orientedSnapshot()
        // One clean pass over a single-layer plane removes little and moves little.
        XCTAssertGreaterThan(after.count, before.count * 8 / 10)
    }
}
