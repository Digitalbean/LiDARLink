import XCTest
import simd
@testable import LiDARLinkShared

final class SurfelMapTests: XCTestCase {

    /// A frontal plane at `z = -depth`, filling the depth image.
    private func planeDepth(_ w: Int, _ h: Int, depth: Float) -> [Float16] {
        [Float16](repeating: Float16(depth), count: w * h)
    }

    private let k = CameraIntrinsics(fx: 100, fy: 100, cx: 48, cy: 36, imageWidth: 96, imageHeight: 72)

    func testIntegratesAPlaneAndMergesRepeatObservations() {
        let map = SurfelMap()
        let depth = planeDepth(96, 72, depth: 1.5)

        map.integrate(depth: depth, width: 96, height: 72, depthScale: 1, intrinsics: k,
                      pose: matrix_identity_float4x4, confidence: nil, keyframeID: 0)
        let afterFirst = map.count
        // Superpixel patches, not per-pixel: a 96×72 plane → ~100 tile surfels.
        XCTAssertGreaterThan(afterFirst, 40)
        XCTAssertLessThan(afterFirst, 400)

        // Same frame again — should merge, not roughly double.
        map.integrate(depth: depth, width: 96, height: 72, depthScale: 1, intrinsics: k,
                      pose: matrix_identity_float4x4, confidence: nil, keyframeID: 1)
        XCTAssertLessThan(map.count, Int(Double(afterFirst) * 1.3), "second pass merges into existing surfels")

        // Normals face the camera (camera at origin looking -z → surfels' +z ≈ 1).
        for s in map.orientedSnapshot() {
            XCTAssertGreaterThan(s.normal.z, 0.7)
            XCTAssertEqual(simd_length(s.normal), 1, accuracy: 1e-3)
            XCTAssertEqual(s.position.z, -1.5, accuracy: 0.05)
        }
    }

    func testApplyCorrectionMovesOnlyThatKeyframesSurfels() {
        let map = SurfelMap()
        let depth = planeDepth(96, 72, depth: 1.5)
        map.integrate(depth: depth, width: 96, height: 72, depthScale: 1, intrinsics: k,
                      pose: matrix_identity_float4x4, confidence: nil, keyframeID: 0)

        // Keyframe 1 looks at a far-away plane (no overlap → clean ownership).
        var far = matrix_identity_float4x4
        far.columns.3.x = 10
        map.integrate(depth: depth, width: 96, height: 72, depthScale: 1, intrinsics: k,
                      pose: far, confidence: nil, keyframeID: 1)

        let kf0Before = map.orientedSnapshot().filter { $0.owner == 0 }.map { $0.position }
        let kf1MinBefore = map.orientedSnapshot().filter { $0.owner == 1 }.map { $0.position.x }.min()!

        var delta = matrix_identity_float4x4
        delta.columns.3.x = 0.2
        map.applyCorrection(keyframe: 1, delta: delta)

        let kf0After = map.orientedSnapshot().filter { $0.owner == 0 }.map { $0.position }
        let kf1MinAfter = map.orientedSnapshot().filter { $0.owner == 1 }.map { $0.position.x }.min()!

        XCTAssertEqual(kf0Before.count, kf0After.count)
        for (a, b) in zip(kf0Before.sorted { $0.x < $1.x }, kf0After.sorted { $0.x < $1.x }) {
            XCTAssertEqual(simd_length(a - b), 0, accuracy: 1e-4, "keyframe 0's surfels stay put")
        }
        XCTAssertEqual(kf1MinAfter - kf1MinBefore, 0.2, accuracy: 1e-3, "keyframe 1's surfels shifted by the correction")
    }

    func testDeformationBlendsBetweenNodesWithNoSeam() {
        let map = SurfelMap()
        // A long flat strip of surfels along x.
        let depth = planeDepth(96, 72, depth: 1.5)
        for kf in 0..<12 {
            var pose = matrix_identity_float4x4
            pose.columns.3.x = Float(kf) * 0.35
            map.integrate(depth: depth, width: 96, height: 72, depthScale: 1, intrinsics: k,
                          pose: pose, confidence: nil, keyframeID: Int32(kf))
        }
        let before = Dictionary(uniqueKeysWithValues: map.orientedSnapshot().enumerated().map { ($0.offset, $0.element.position) })

        // Nodes at x = 0…3.85; the far end ramps to +0.3 in z, the near end stays.
        func node(_ x: Float, dz: Float) -> SurfelMap.DeformNode {
            .init(anchor: SIMD3<Float>(x, 0, 0), rotation: matrix_identity_float3x3,
                  translation: SIMD3<Float>(0, 0, dz))
        }
        let nodes = (0..<12).map { node(Float($0) * 0.35, dz: $0 >= 9 ? 0.3 : 0) }
        map.applyDeformation(nodes, influence: 3)

        // dz applied per surfel, ordered by x — near node 0 ≈ 0, near node 3 ≈ 0.3,
        // and neighbouring surfels move together (no seam).
        let after = map.orientedSnapshot().enumerated()
            .map { (x: $0.element.position.x, dz: $0.element.position.z - before[$0.offset]!.z) }
            .sorted { $0.x < $1.x }
        XCTAssertLessThan(abs(after.first!.dz), 0.08)
        XCTAssertGreaterThan(after.last!.dz, 0.15)
        var maxStep: Float = 0
        for i in 1..<after.count { maxStep = max(maxStep, abs(after[i].dz - after[i - 1].dz)) }
        // A rigid per-keyframe correction would drop the full 0.3 at one seam;
        // the blend spreads it over the influence radius.
        XCTAssertLessThan(maxStep, 0.13, "no seam — the shift is spread across neighbouring surfels")
    }

    func testClear() {
        let map = SurfelMap()
        map.integrate(depth: planeDepth(96, 72, depth: 1.2), width: 96, height: 72, depthScale: 1,
                      intrinsics: k, pose: matrix_identity_float4x4, confidence: nil, keyframeID: 0)
        XCTAssertGreaterThan(map.count, 0)
        map.clear()
        XCTAssertEqual(map.count, 0)
        XCTAssertTrue(map.snapshot().isEmpty)
    }
}
