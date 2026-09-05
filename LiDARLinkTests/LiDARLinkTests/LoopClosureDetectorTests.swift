import XCTest
import simd
@testable import LiDARLinkShared

final class LoopClosureDetectorTests: XCTestCase {

    /// Two depth planes meeting at a vertical step, plus a horizontal step —
    /// gives ICP real x, y and z constraints (a flat plane slides).
    private func wallKeyframe(_ id: Int32, pose: simd_float4x4, relative: simd_float4x4) -> Keyframe {
        let w = 96, h = 72
        var d = [Float16](repeating: 1.5, count: w * h)
        for y in 0..<h { for x in 0..<w {
            let fx = Float(x) / Float(w) - 0.5
            let fy = Float(y) / Float(h) - 0.5
            // A vertical V-groove (both faces have an x-normal → x constrained)
            // on a y-tilted plane (→ y constrained), at ~1.4 m (→ z constrained).
            d[y * w + x] = Float16(1.4 + 0.7 * abs(fx) + 0.5 * fy)
        }}
        return Keyframe(id: id, capturedAtMs: UInt64(id) * 100,
                        depth: DepthPayload(width: w, height: h, depthData: Binary.data(from: d), confidenceData: nil),
                        depthScale: 1,
                        depthIntrinsics: CameraIntrinsics(fx: 70, fy: 70, cx: 48, cy: 36, imageWidth: w, imageHeight: h),
                        arkitPose: pose, relativePose: relative)
    }

    private func translated(_ x: Float) -> simd_float4x4 {
        var m = matrix_identity_float4x4
        m.columns.3.x = x
        return m
    }

    func testFindsARevisit() {
        // 0..4 drift +4 cm/leg. KF 5 is physically back at the origin but ARKit
        // thinks it's at x = 0.20 (accumulated drift).
        var keyframes: [Keyframe] = [wallKeyframe(0, pose: matrix_identity_float4x4, relative: matrix_identity_float4x4)]
        var poses: [Int32: simd_float4x4] = [0: matrix_identity_float4x4]
        for i in 1...4 {
            let p = translated(0.04 * Float(i))
            keyframes.append(wallKeyframe(Int32(i), pose: p, relative: translated(0.04)))
            poses[Int32(i)] = p
        }
        keyframes.append(wallKeyframe(5, pose: translated(0.20), relative: translated(0.04)))
        poses[5] = translated(0.20)

        var params = LoopClosureDetector.Parameters()
        params.sequentialWindow = 3
        params.minCorrespondences = 60
        let closures = LoopClosureDetector.detect(keyframes: keyframes, poses: poses, parameters: params)

        let revisit = closures.first { $0.from == 0 && $0.to == 5 }
        XCTAssertNotNil(revisit, "should recognise KF5 revisits KF0")
        // The corrected relative transform should be near identity (same place).
        XCTAssertLessThan(simd_length(Lie.translation(revisit!.measured)), 0.06)
        XCTAssertLessThan(revisit!.rmsErrorMeters, 0.03)
    }

    func testCapsClosuresPerKeyframe() {
        // A cluster of keyframes all sitting near the origin looking at the same
        // V-groove — geometrically every non-adjacent pair is a valid closure.
        var keyframes: [Keyframe] = []
        var poses: [Int32: simd_float4x4] = [:]
        for i in 0..<9 {
            let p = translated(0.02 * Float(i))
            keyframes.append(wallKeyframe(Int32(i), pose: p, relative: translated(0.02)))
            poses[Int32(i)] = p
        }
        var params = LoopClosureDetector.Parameters()
        params.sequentialWindow = 1
        params.minCorrespondences = 60
        params.maxClosuresPerKeyframe = 2

        let closures = LoopClosureDetector.detect(keyframes: keyframes, poses: poses, parameters: params)
        XCTAssertFalse(closures.isEmpty, "the cluster does overlap")
        var perNode: [Int32: Int] = [:]
        for c in closures { perNode[c.from, default: 0] += 1; perNode[c.to, default: 0] += 1 }
        XCTAssertLessThanOrEqual(perNode.values.max() ?? 0, 2, "no keyframe exceeds the cap")
    }

    // MARK: Degeneracy / aperture guard

    /// A single flat wall filling the frame — one plane, one normal direction.
    /// A depth ramp is a perfectly flat plane, so the correspondence normals are
    /// all parallel and translation along the wall is unconstrained.
    private func flatWallKeyframe(_ id: Int32, pose: simd_float4x4, relative: simd_float4x4) -> Keyframe {
        let w = 96, h = 72
        var d = [Float16](repeating: 0, count: w * h)
        for y in 0..<h { for x in 0..<w {
            let fx = Float(x) / Float(w) - 0.5
            let fy = Float(y) / Float(h) - 0.5
            // Linear in both axes => a single tilted plane (constant normal).
            d[y * w + x] = Float16(1.5 + 0.10 * fx + 0.06 * fy)
        }}
        return Keyframe(id: id, capturedAtMs: UInt64(id) * 100,
                        depth: DepthPayload(width: w, height: h, depthData: Binary.data(from: d), confidenceData: nil),
                        depthScale: 1,
                        depthIntrinsics: CameraIntrinsics(fx: 70, fy: 70, cx: 48, cy: 36, imageWidth: w, imageHeight: h),
                        arkitPose: pose, relativePose: relative)
    }

    func testRejectsSinglePlaneClosure() throws {
        // KF 0..3 drift +4 cm/leg down a blank flat wall; KF 4 is physically back
        // at the start. ICP fits the plane with a tight RMS and full overlap but
        // has slid tangentially — a bogus constraint.
        var keyframes: [Keyframe] = [flatWallKeyframe(0, pose: matrix_identity_float4x4, relative: matrix_identity_float4x4)]
        var poses: [Int32: simd_float4x4] = [0: matrix_identity_float4x4]
        for i in 1...3 {
            let p = translated(0.04 * Float(i))
            keyframes.append(flatWallKeyframe(Int32(i), pose: p, relative: translated(0.04)))
            poses[Int32(i)] = p
        }
        keyframes.append(flatWallKeyframe(4, pose: translated(0.16), relative: translated(0.04)))
        poses[4] = translated(0.16)

        var params = LoopClosureDetector.Parameters()
        params.sequentialWindow = 3
        params.minCorrespondences = 60

        // Guard on (default): the degenerate revisit is rejected.
        XCTAssertNil(LoopClosureDetector.detect(keyframes: keyframes, poses: poses, parameters: params)
                        .first { $0.from == 0 && $0.to == 4 },
                     "a single-plane overlap must not become a constraint")

        // Guard off: the flat-wall match passes every other criterion, so it
        // leaks — proving the aperture guard is what rejected it.
        var noGuard = params
        noGuard.minTranslationConditioning = 0
        let leaked = try XCTUnwrap(LoopClosureDetector.detect(keyframes: keyframes, poses: poses, parameters: noGuard)
                                    .first { $0.from == 0 && $0.to == 4 },
                                   "without the guard the flat-wall match is (wrongly) accepted")
        XCTAssertLessThan(leaked.translationConditioning, params.minTranslationConditioning,
                          "flat wall reports near-zero translation conditioning")
    }

    func testAcceptsWellConstrainedClosureWithDrift() throws {
        // The proven V-groove fixture (x, y and z all constrained). KF 0..4
        // drift +4 cm/leg; KF 5 revisits the origin. The guard must not touch it.
        var keyframes: [Keyframe] = [wallKeyframe(0, pose: matrix_identity_float4x4, relative: matrix_identity_float4x4)]
        var poses: [Int32: simd_float4x4] = [0: matrix_identity_float4x4]
        for i in 1...4 {
            let p = translated(0.04 * Float(i))
            keyframes.append(wallKeyframe(Int32(i), pose: p, relative: translated(0.04)))
            poses[Int32(i)] = p
        }
        keyframes.append(wallKeyframe(5, pose: translated(0.20), relative: translated(0.04)))
        poses[5] = translated(0.20)

        var params = LoopClosureDetector.Parameters()
        params.sequentialWindow = 3
        params.minCorrespondences = 60

        let closures = LoopClosureDetector.detect(keyframes: keyframes, poses: poses, parameters: params)
        let revisit = try XCTUnwrap(closures.first { $0.from == 0 && $0.to == 5 },
                                    "a V-groove has enough geometry — the closure survives the guard")
        XCTAssertGreaterThanOrEqual(revisit.translationConditioning, params.minTranslationConditioning)
        XCTAssertLessThan(simd_length(Lie.translation(revisit.measured)), 0.06,
                          "and it pulls KF5 back toward the origin")
    }

    // MARK: Pose-independent (descriptor) matching

    func testRecoversRevisitBeyondPoseProximityThreshold() throws {
        // Drift far enough that pose proximity alone would never even try this
        // pair (the graph's current estimate puts KF6 1.8 m from KF0, past the
        // 1.5 m gate) — but KF6 is looking at literally the same V-groove, so
        // the descriptor path finds it anyway.
        var keyframes: [Keyframe] = [wallKeyframe(0, pose: matrix_identity_float4x4, relative: matrix_identity_float4x4)]
        var poses: [Int32: simd_float4x4] = [0: matrix_identity_float4x4]
        for i in 1...5 {
            let p = translated(0.3 * Float(i))
            keyframes.append(wallKeyframe(Int32(i), pose: p, relative: translated(0.3)))
            poses[Int32(i)] = p
        }
        keyframes.append(wallKeyframe(6, pose: translated(1.8), relative: translated(0.3)))
        poses[6] = translated(1.8)

        var params = LoopClosureDetector.Parameters()
        params.sequentialWindow = 3
        params.minCorrespondences = 60
        XCTAssertGreaterThan(simd_length(Lie.translation(poses[0]!) - Lie.translation(poses[6]!)),
                             params.maxCameraDistanceMeters,
                             "sanity: pose proximity alone would reject this pair")

        let closures = LoopClosureDetector.detect(keyframes: keyframes, poses: poses, parameters: params)
        let revisit = try XCTUnwrap(closures.first { $0.from == 0 && $0.to == 6 },
                                    "identical geometry should be found by descriptor match despite the pose gap")
        XCTAssertTrue(revisit.recoveredFromDrift, "this pair was found only by looking alike, not by pose proximity")
        XCTAssertLessThan(simd_length(Lie.translation(revisit.measured)), 0.06,
                          "and the correction pulls KF6 back toward the true (shared) location")
    }

    func testDescriptorMatchStillRespectsApertureGuard() {
        // Same blank wall, far enough apart in pose that only the descriptor
        // path finds the pair at all — the aperture guard must still catch
        // it: a flat plane is a flat plane no matter how the candidate was
        // proposed. This is the perceptual-aliasing safety net: a cheap
        // descriptor match is only ever a hint to *try* ICP, never a promise
        // that the pair is actually the same place.
        var keyframes: [Keyframe] = [flatWallKeyframe(0, pose: matrix_identity_float4x4, relative: matrix_identity_float4x4)]
        var poses: [Int32: simd_float4x4] = [0: matrix_identity_float4x4]
        for i in 1...5 {
            let p = translated(0.3 * Float(i))
            keyframes.append(flatWallKeyframe(Int32(i), pose: p, relative: translated(0.3)))
            poses[Int32(i)] = p
        }
        keyframes.append(flatWallKeyframe(6, pose: translated(1.8), relative: translated(0.3)))
        poses[6] = translated(1.8)

        var params = LoopClosureDetector.Parameters()
        params.sequentialWindow = 3
        params.minCorrespondences = 60

        XCTAssertNil(LoopClosureDetector.detect(keyframes: keyframes, poses: poses, parameters: params)
                        .first { $0.from == 0 && $0.to == 6 },
                     "a blank wall must not become a closure even when found only by looking alike")
    }

    func testNoFalsePositiveForDistinctPlaces() {
        // Two keyframes looking at walls a metre apart — no overlap.
        let keyframes = [
            wallKeyframe(0, pose: matrix_identity_float4x4, relative: matrix_identity_float4x4),
            wallKeyframe(10, pose: translated(3.0), relative: translated(0.1)),
        ]
        let poses: [Int32: simd_float4x4] = [0: matrix_identity_float4x4, 10: translated(3.0)]
        var params = LoopClosureDetector.Parameters()
        params.sequentialWindow = 1
        XCTAssertTrue(LoopClosureDetector.detect(keyframes: keyframes, poses: poses, parameters: params).isEmpty)
    }
}
