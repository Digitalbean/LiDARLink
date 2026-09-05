import XCTest
import simd
@testable import LiDARLinkShared

final class V2ReconstructionTests: XCTestCase {

    /// A synthetic sweep past a slanted wall: each frame sees the wall at ~1.5 m,
    /// the camera translates along x. Enough for keyframes, a pose graph, and a
    /// marching-cubes surface.
    private func sweepFrames(_ n: Int) -> [ScanFrame] {
        let w = 64, h = 48
        let k = CameraIntrinsics(fx: 50, fy: 50, cx: 32, cy: 24, imageWidth: w, imageHeight: h)
        var frames: [ScanFrame] = []
        for i in 0..<n {
            var depth = [Float16](repeating: 0, count: w * h)
            for y in 0..<h { for x in 0..<w {
                let fx = Float(x) / Float(w) - 0.5
                depth[y * w + x] = Float16(1.5 + 0.4 * abs(fx))
            }}
            var pose = matrix_identity_float4x4
            pose.columns.3.x = Float(i) * 0.05
            frames.append(ScanFrame(sequence: UInt32(i),
                                    captureTimestampMs: UInt64(i) * 40,
                                    pose: pose,
                                    intrinsics: k,
                                    depth: DepthPayload(width: w, height: h,
                                                        depthData: Binary.data(from: depth),
                                                        confidenceData: nil),
                                    color: nil,
                                    depthIsSmoothed: false,
                                    depthScale: 1))
        }
        return frames
    }

    func testProducesASurface() async {
        var options = V2Reconstruction.Options()
        options.minTranslationMeters = 0.02
        let result = await V2Reconstruction.run(frames: sweepFrames(20), options: options)
        let unwrapped = try? XCTUnwrap(result)
        guard let result = unwrapped else { return }
        XCTAssertGreaterThan(result.keyframeCount, 2)
        XCTAssertFalse(result.mesh.isEmpty)
        XCTAssertEqual(result.mesh.faces.count % 3, 0)
        // No colour frames → no atlas.
        XCTAssertNil(result.textured)
    }

    func testTooFewFramesYieldsNil() async {
        let result = await V2Reconstruction.run(frames: sweepFrames(1))
        XCTAssertNil(result)
    }

    func testPoseGraphErrorDoesNotGrow() async {
        var options = V2Reconstruction.Options()
        options.minTranslationMeters = 0.02
        options.detectLoopClosures = false
        guard let result = await V2Reconstruction.run(frames: sweepFrames(16), options: options) else {
            return XCTFail("expected a result")
        }
        XCTAssertLessThanOrEqual(result.finalError, result.initialError + 1e-3)
    }
}
