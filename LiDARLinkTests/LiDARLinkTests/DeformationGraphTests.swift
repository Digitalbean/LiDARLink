import XCTest
import simd
@testable import LiDARLinkShared

final class DeformationGraphTests: XCTestCase {

    func testSingleNodeIsRigid() {
        let node = DeformationGraph.Node(anchor: .zero,
                                         rotation: matrix_identity_float3x3,
                                         translation: SIMD3<Float>(0.5, 0, 0))
        let out = DeformationGraph.warp(SIMD3<Float>(1, 2, 3), nodes: [node])
        XCTAssertEqual(out, SIMD3<Float>(1.5, 2, 3))
    }

    func testBlendsBetweenTwoNodes() {
        // Node A at x=0 pushes +x by 0.2; node B at x=10 doesn't move.
        let a = DeformationGraph.Node(anchor: SIMD3<Float>(0, 0, 0),
                                      rotation: matrix_identity_float3x3, translation: SIMD3<Float>(0.2, 0, 0))
        let b = DeformationGraph.Node(anchor: SIMD3<Float>(10, 0, 0),
                                      rotation: matrix_identity_float3x3, translation: .zero)
        let nearA = DeformationGraph.warp(SIMD3<Float>(1, 0, 0), nodes: [a, b])
        let mid = DeformationGraph.warp(SIMD3<Float>(5, 0, 0), nodes: [a, b])
        let nearB = DeformationGraph.warp(SIMD3<Float>(9, 0, 0), nodes: [a, b])
        XCTAssertGreaterThan(nearA.x - 1, 0.12, "near A gets most of the push")
        XCTAssertLessThan(nearB.x - 9, 0.06, "near B barely moves")
        XCTAssertLessThan(mid.x - 5, nearA.x - 1, "mid-point is pushed less than near-A")
    }

    func testCorrectedPoseNodeConstruction() {
        var raw = matrix_identity_float4x4
        raw.columns.3 = SIMD4<Float>(2, 0, 0, 1)
        var corrected = matrix_identity_float4x4
        corrected.columns.3 = SIMD4<Float>(2.2, 0, 0, 1)   // 20 cm drift correction
        let node = DeformationGraph.Node(rawPose: raw, correctedPose: corrected)
        // A point at the raw camera position warps to the corrected position.
        let out = DeformationGraph.warp(SIMD3<Float>(2, 0, 0), nodes: [node])
        XCTAssertEqual(out.x, 2.2, accuracy: 1e-4)
    }

    // MARK: robustness

    /// A trajectory-only drift field must not shear a flat wall into a slab.
    func testFlatWallStaysFlatUnderRealisticDriftField() {
        // Camera walks along x from 0..3 m, 1 m off the wall (wall at z = 3).
        // ARKit drift: a slow yaw that reaches ~3° plus ~8 cm of translation by
        // the end; the pose graph corrects it back.
        var nodes: [DeformationGraph.Node] = []
        let count = 40
        for i in 0..<count {
            let t = Float(i) / Float(count - 1)
            let camPos = SIMD3<Float>(3 * t, 0, 0)
            var raw = matrix_identity_float4x4
            raw.columns.3 = SIMD4<Float>(camPos, 1)
            // corrected = raw pre-multiplied by the accumulated drift we remove.
            let yaw = 0.052 * t                     // ~3° by the end
            let drift = Lie.pose(rotation: Lie.so3Exp(SIMD3<Float>(0, yaw, 0)),
                                 translation: SIMD3<Float>(0.08 * t, 0, 0.02 * t))
            let corrected = drift * raw
            nodes.append(.init(rawPose: raw, correctedPose: corrected))
        }

        // A patch of wall in front of the middle of the path.
        var warped: [SIMD3<Float>] = []
        for gx in stride(from: Float(0.5), through: 2.5, by: 0.1) {
            for gy in stride(from: Float(-0.6), through: 0.6, by: 0.1) {
                let p = SIMD3<Float>(gx, gy, 3)
                warped.append(DeformationGraph.warp(p, nodes: nodes))
            }
        }

        // Fit a plane to the warped points, measure thickness (max abs residual).
        let centroid = warped.reduce(SIMD3<Float>.zero, +) / Float(warped.count)
        var cov = simd_float3x3(0)
        for w in warped {
            let d = w - centroid
            cov.columns.0 += d * d.x
            cov.columns.1 += d * d.y
            cov.columns.2 += d * d.z
        }
        let normal = smallestEigenvectorSym(cov)
        let thickness = warped.map { abs(simd_dot($0 - centroid, normal)) }.max() ?? 0
        XCTAssertLessThan(thickness, 0.03, "warped wall stays within 3 cm of flat (was tens of cm)")
    }

    func testFarPointFadesTowardIdentity() {
        var raw = matrix_identity_float4x4
        raw.columns.3 = SIMD4<Float>(0, 0, 0, 1)
        var corrected = matrix_identity_float4x4
        corrected.columns.3 = SIMD4<Float>(0.2, 0, 0, 1)
        let a = DeformationGraph.Node(rawPose: raw, correctedPose: corrected)
        var raw2 = matrix_identity_float4x4
        raw2.columns.3 = SIMD4<Float>(0.5, 0, 0, 1)
        var corrected2 = matrix_identity_float4x4
        corrected2.columns.3 = SIMD4<Float>(0.7, 0, 0, 1)
        let b = DeformationGraph.Node(rawPose: raw2, correctedPose: corrected2)

        let near = DeformationGraph.warp(SIMD3<Float>(0.25, 0, 0), nodes: [a, b])
        let far = DeformationGraph.warp(SIMD3<Float>(0.25, 0, 12), nodes: [a, b])
        let nearShift = simd_length(near - SIMD3<Float>(0.25, 0, 0))
        let farShift = simd_length(far - SIMD3<Float>(0.25, 0, 12))
        XCTAssertGreaterThan(nearShift, 0.1, "a point among the nodes gets the full correction")
        XCTAssertLessThan(farShift, nearShift * 0.5, "a point far from every node barely moves")
    }

    func testBadClosureDisplacementIsCapped() {
        // One node with an absurd 2 m offset (a wrong loop closure).
        var raw = matrix_identity_float4x4
        raw.columns.3 = SIMD4<Float>(0, 0, 0, 1)
        var corrected = matrix_identity_float4x4
        corrected.columns.3 = SIMD4<Float>(2, 0, 0, 1)
        let bad = DeformationGraph.Node(rawPose: raw, correctedPose: corrected)
        var raw2 = matrix_identity_float4x4
        raw2.columns.3 = SIMD4<Float>(0.3, 0, 0, 1)
        let ok = DeformationGraph.Node(rawPose: raw2, correctedPose: raw2)

        let out = DeformationGraph.warp(SIMD3<Float>(0.1, 0, 0), nodes: [bad, ok])
        XCTAssertLessThanOrEqual(simd_length(out - SIMD3<Float>(0.1, 0, 0)),
                                 DeformationGraph.limits.maxDisplacement + 1e-4)
    }

    func testNegligibleFieldIsNoOp() {
        let nodes = (0..<10).map { i -> DeformationGraph.Node in
            var pose = matrix_identity_float4x4
            pose.columns.3 = SIMD4<Float>(Float(i) * 0.1, 0, 0, 1)
            return .init(rawPose: pose, correctedPose: pose)
        }
        let pts = [PointCloudPoint(position: SIMD3<Float>(0.5, 1, 2),
                                   color: SIMD3<UInt8>(1, 2, 3), confidence: 2)]
        let out = DeformationGraph.warp(points: pts, nodes: nodes)
        XCTAssertEqual(out[0].position, pts[0].position)
    }

    private func smallestEigenvectorSym(_ m: simd_float3x3) -> SIMD3<Float> {
        // Inverse power iteration with a small ridge.
        var a = m
        let ridge = max(a.columns.0.x + a.columns.1.y + a.columns.2.z, 1) * 1e-5
        a.columns.0.x += ridge; a.columns.1.y += ridge; a.columns.2.z += ridge
        let inv = a.inverse
        var v = simd_normalize(SIMD3<Float>(1, 2, 3))
        for _ in 0..<64 { v = simd_normalize(inv * v) }
        return v
    }
}
