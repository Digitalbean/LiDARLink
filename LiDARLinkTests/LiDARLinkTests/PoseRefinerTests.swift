import XCTest
import simd
@testable import LiDARLinkShared

final class PoseRefinerTests: XCTestCase {

    /// A corner — three perpendicular planes — so alignment is fully constrained
    /// (no sliding ambiguity).
    private func cornerCloud(spacing: Float = 0.02) -> [PointCloudPoint] {
        var points: [PointCloudPoint] = []
        var a: Float = 0
        while a <= 1 {
            var b: Float = 0
            while b <= 1 {
                points.append(PointCloudPoint(position: SIMD3<Float>(0, a, b), color: .zero, confidence: 2))
                points.append(PointCloudPoint(position: SIMD3<Float>(a, 0, b), color: .zero, confidence: 2))
                points.append(PointCloudPoint(position: SIMD3<Float>(a, b, 0), color: .zero, confidence: 2))
                b += spacing
            }
            a += spacing
        }
        return points
    }

    private func transformed(_ points: [PointCloudPoint], by t: simd_float4x4) -> [PointCloudPoint] {
        points.map { point in
            let w = t * SIMD4<Float>(point.position, 1)
            var moved = point
            moved.position = SIMD3<Float>(w.x, w.y, w.z)
            return moved
        }
    }

    private func map(from points: [PointCloudPoint], cell: Float = 0.02) -> ProgressiveMap {
        let map = ProgressiveMap(cellSize: cell)
        _ = map.deduplicate(points)
        return map
    }

    func testMinLifecycleExcludesUnconfirmedTargets() {
        // This target map was seeded without viewpoints (map(from:) calls
        // deduplicate with none) — nothing on it can ever reach .confirmed
        // (see ProgressiveMap.deduplicate's viewpoint parameter). Requiring
        // .confirmed correspondences must then find nothing to align against,
        // proving minLifecycle actually reaches into nearestSurfels.
        let cloud = cornerCloud()
        let target = map(from: cloud)
        let result = PoseRefiner.refine(worldPoints: cloud, against: target, minLifecycle: .confirmed)
        XCTAssertFalse(result.applied, "a map with no viewpoint-diversity evidence has nothing to offer at .confirmed")
    }

    func testRecoversSmallTranslation() {
        let cloud = cornerCloud()
        let target = map(from: cloud)
        var offset = matrix_identity_float4x4
        offset.columns.3 = SIMD4<Float>(0.02, -0.012, 0.015, 1)
        let drifted = transformed(cloud, by: offset)

        let result = PoseRefiner.refine(worldPoints: drifted, against: target)
        XCTAssertTrue(result.applied)
        let t = result.correction.columns.3
        // Correction should undo the drift (map drifted points back).
        XCTAssertEqual(t.x, -0.02, accuracy: 0.004)
        XCTAssertEqual(t.y, 0.012, accuracy: 0.004)
        XCTAssertEqual(t.z, -0.015, accuracy: 0.004)
        XCTAssertLessThan(result.rmsErrorMeters, 0.005)
    }

    func testRecoversSmallRotation() {
        let cloud = cornerCloud()
        let target = map(from: cloud)
        let angle: Float = 2 * .pi / 180
        var rot = matrix_identity_float4x4
        rot.columns.0 = SIMD4<Float>(cos(angle), 0, -sin(angle), 0)
        rot.columns.2 = SIMD4<Float>(sin(angle), 0, cos(angle), 0)
        let drifted = transformed(cloud, by: rot)

        let result = PoseRefiner.refine(worldPoints: drifted, against: target)
        XCTAssertTrue(result.applied)
        XCTAssertEqual(PoseRefiner.rotationAngleDegrees(result.correction), 2, accuracy: 0.6)
    }
    func testEmptyMapReturnsIdentity() {
        let result = PoseRefiner.refine(worldPoints: cornerCloud(), against: ProgressiveMap(cellSize: 0.02))
        XCTAssertFalse(result.applied)
        XCTAssertEqual(result.correction, matrix_identity_float4x4)
    }

    func testNoOverlapReturnsIdentity() {
        let cloud = cornerCloud()
        let target = map(from: cloud)
        var offset = matrix_identity_float4x4
        offset.columns.3 = SIMD4<Float>(3, 3, 3, 1) // frame is nowhere near the map
        let drifted = transformed(cloud, by: offset)

        let result = PoseRefiner.refine(worldPoints: drifted, against: target)
        XCTAssertFalse(result.applied)
    }

    /// A single flat plane (the z = 0 wall) — one normal direction only.
    private func planeCloud(spacing: Float = 0.02) -> [PointCloudPoint] {
        var points: [PointCloudPoint] = []
        var a: Float = 0
        while a <= 1 {
            var b: Float = 0
            while b <= 1 {
                points.append(PointCloudPoint(position: SIMD3<Float>(a, b, 0), color: .zero, confidence: 2))
                b += spacing
            }
            a += spacing
        }
        return points
    }

    func testConditioningFlagsAPlaneAsDegenerate() {
        let cloud = planeCloud()
        let target = map(from: cloud)
        var offset = matrix_identity_float4x4
        offset.columns.3 = SIMD4<Float>(0.03, -0.02, 0.004, 1) // mostly tangential slide
        let drifted = transformed(cloud, by: offset)

        let result = PoseRefiner.refine(worldPoints: drifted, against: target)
        XCTAssertTrue(result.applied, "the plane still fits (low RMS) — that's the trap")
        XCTAssertLessThan(result.translationConditioning, 0.02,
                          "a single plane leaves translation unconstrained in the plane")
    }

    /// Three perpendicular faces, de-interleaved and small enough that the
    /// refiner uses the whole cloud (a strided subsample of the interleaved
    /// `cornerCloud` can alias onto a single face and look degenerate).
    private func balancedCorner(spacing: Float = 0.02, extent: Float = 0.34) -> [PointCloudPoint] {
        var faceX: [PointCloudPoint] = [], faceY: [PointCloudPoint] = [], faceZ: [PointCloudPoint] = []
        var a: Float = 0
        while a <= extent {
            var b: Float = 0
            while b <= extent {
                faceX.append(PointCloudPoint(position: SIMD3<Float>(0, a, b), color: .zero, confidence: 2))
                faceY.append(PointCloudPoint(position: SIMD3<Float>(a, 0, b), color: .zero, confidence: 2))
                faceZ.append(PointCloudPoint(position: SIMD3<Float>(a, b, 0), color: .zero, confidence: 2))
                b += spacing
            }
            a += spacing
        }
        return faceX + faceY + faceZ
    }

    func testConditioningPassesACorner() {
        let cloud = balancedCorner()
        let target = map(from: cloud)
        var offset = matrix_identity_float4x4
        offset.columns.3 = SIMD4<Float>(0.02, -0.012, 0.015, 1)
        let drifted = transformed(cloud, by: offset)

        let result = PoseRefiner.refine(worldPoints: drifted, against: target)
        XCTAssertTrue(result.applied)
        XCTAssertGreaterThan(result.translationConditioning, 0.3,
                             "three perpendicular planes fully constrain translation")
    }

    func testEigenvaluesOfKnownMatrix() {
        // diag(3, 1, 2) -> sorted descending (3, 2, 1)
        let e = PoseRefiner.symmetricEigenvalues3x3(xx: 3, xy: 0, xz: 0, yy: 1, yz: 0, zz: 2)
        XCTAssertEqual(e.x, 3, accuracy: 1e-4)
        XCTAssertEqual(e.y, 2, accuracy: 1e-4)
        XCTAssertEqual(e.z, 1, accuracy: 1e-4)

        // A non-diagonal symmetric matrix: [[2,1,0],[1,2,0],[0,0,5]] -> 5, 3, 1
        let f = PoseRefiner.symmetricEigenvalues3x3(xx: 2, xy: 1, xz: 0, yy: 2, yz: 0, zz: 5)
        XCTAssertEqual(f.x, 5, accuracy: 1e-4)
        XCTAssertEqual(f.y, 3, accuracy: 1e-4)
        XCTAssertEqual(f.z, 1, accuracy: 1e-4)
    }

    func testConvergedFrameIsLeftAlone() {
        let cloud = cornerCloud()
        let target = map(from: cloud)
        let result = PoseRefiner.refine(worldPoints: cloud, against: target)
        // Already aligned — correction, if any, is negligible.
        let t = result.correction.columns.3
        XCTAssertLessThan(simd_length(SIMD3<Float>(t.x, t.y, t.z)), 0.003)
    }
}
