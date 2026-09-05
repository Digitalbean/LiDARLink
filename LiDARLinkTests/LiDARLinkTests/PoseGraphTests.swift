import XCTest
import simd
@testable import LiDARLinkShared

final class LieTests: XCTestCase {
    func testSO3RoundTrip() {
        for omega in [SIMD3<Float>(0.1, -0.2, 0.05), SIMD3<Float>(1.2, 0.3, -0.7), SIMD3<Float>(0, 0, 0)] {
            let r = Lie.so3Exp(omega)
            let back = Lie.so3Log(r)
            XCTAssertEqual(simd_length(back - omega), 0, accuracy: 1e-4)
        }
    }

    func testBetweenErrorZeroWhenConsistent() {
        var ti = matrix_identity_float4x4
        ti.columns.3 = SIMD4<Float>(1, 2, 3, 1)
        let z = Lie.pose(rotation: Lie.so3Exp(SIMD3<Float>(0.2, 0, 0)), translation: SIMD3<Float>(0.5, 0, 0.1))
        let tj = ti * z
        let (er, et) = Lie.betweenError(from: ti, to: tj, measured: z)
        XCTAssertEqual(simd_length(er), 0, accuracy: 1e-4)
        XCTAssertEqual(simd_length(et), 0, accuracy: 1e-4)
    }
}

final class PoseGraphTests: XCTestCase {

    private func translated(_ x: Float, _ y: Float, _ z: Float) -> simd_float4x4 {
        var m = matrix_identity_float4x4
        m.columns.3 = SIMD4<Float>(x, y, z, 1)
        return m
    }

    func testCleanChainStaysPut() {
        let graph = PoseGraph()
        for i in 0..<5 { graph.addNode(id: Int32(i), pose: translated(Float(i), 0, 0)) }
        for i in 0..<4 {
            graph.addConstraint(.init(from: Int32(i), to: Int32(i + 1), measured: translated(1, 0, 0)))
        }
        let (initial, final) = graph.optimize()
        XCTAssertLessThan(initial, 1e-3)
        XCTAssertLessThan(final, 1e-3)
        XCTAssertEqual(Lie.translation(graph.pose(id: 4)!).x, 4, accuracy: 1e-3)
    }

    func testLoopClosurePullsDriftBack() {
        // A square loop: 4 legs of 1 m. ARKit odometry drifts, so the estimated
        // final pose doesn't return to the start. A loop-closure constraint
        // (node 4 == node 0) should pull the drift out.
        let graph = PoseGraph()
        let legs = [translated(1, 0, 0), translated(0, 0, 1), translated(-1, 0, 0), translated(0, 0, -1)]

        // Drifted initial estimates: each leg is 10 cm too long in x.
        var pose = matrix_identity_float4x4
        graph.addNode(id: 0, pose: pose)
        for i in 0..<4 {
            var drifted = legs[i]
            drifted.columns.3.x += 0.1
            pose = pose * drifted
            graph.addNode(id: Int32(i + 1), pose: pose)
            graph.addConstraint(.init(from: Int32(i), to: Int32(i + 1), measured: legs[i], weight: 1))
        }
        // Loop closure: keyframe 4 is the same place as keyframe 0.
        graph.addConstraint(.init(from: 4, to: 0, measured: matrix_identity_float4x4, weight: 5))

        let driftBefore = simd_length(Lie.translation(graph.pose(id: 4)!) - Lie.translation(graph.pose(id: 0)!))
        let (initial, final) = graph.optimize(iterations: 20)
        let driftAfter = simd_length(Lie.translation(graph.pose(id: 4)!) - Lie.translation(graph.pose(id: 0)!))

        XCTAssertGreaterThan(driftBefore, 0.35)
        XCTAssertLessThan(driftAfter, 0.08, "loop closure should absorb most of the drift")
        XCTAssertLessThan(final, initial * 0.5)
    }

    func testSingleNodeIsANoOp() {
        let graph = PoseGraph()
        graph.addNode(id: 0, pose: translated(3, 3, 3))
        let (i, f) = graph.optimize()
        XCTAssertEqual(i, 0); XCTAssertEqual(f, 0)
        XCTAssertEqual(Lie.translation(graph.pose(id: 0)!).x, 3, accuracy: 1e-4)
    }
}
