import XCTest
import simd
@testable import LiDARLinkShared

final class LiveDriftCorrectorTests: XCTestCase {

    private func wallFrame(_ i: Int, pose: simd_float4x4) -> ScanFrame {
        let w = 96, h = 72
        var d = [Float16](repeating: 1.5, count: w * h)
        for y in 0..<h { for x in 0..<w {
            let fx = Float(x) / Float(w) - 0.5
            d[y * w + x] = Float16(1.4 + 0.7 * abs(fx) + 0.4 * (Float(y) / Float(h) - 0.5))
        }}
        return ScanFrame(sequence: UInt32(i), captureTimestampMs: UInt64(i) * 60, pose: pose,
                         intrinsics: CameraIntrinsics(fx: 70, fy: 70, cx: 48, cy: 36, imageWidth: w, imageHeight: h),
                         depth: DepthPayload(width: w, height: h, depthData: Binary.data(from: d), confidenceData: nil),
                         color: nil, depthIsSmoothed: false, depthScale: 1)
    }

    private func translated(_ x: Float) -> simd_float4x4 {
        var m = matrix_identity_float4x4; m.columns.3.x = x; return m
    }

    private func collect(_ c: LiveDriftCorrector, _ body: (LiveDriftCorrector) async -> Void) async -> [LiveDriftCorrector.Update] {
        let box = Box()
        let task = Task { for await u in c.updates { await box.append(u) } }
        await body(c)
        try? await Task.sleep(nanoseconds: 40_000_000)
        task.cancel()
        return await box.items
    }

    func testEmitsACorrectionOnARevisit() async {
        var tuning = LiveDriftCorrector.Tuning()
        tuning.minTranslationMeters = 0.02
        tuning.loopClosureEveryNKeyframes = 3
        tuning.optimiseEveryNKeyframes = 3
        tuning.loopParameters.sequentialWindow = 2
        tuning.loopParameters.minCorrespondences = 60
        let corrector = LiveDriftCorrector(tuning: tuning)

        let updates = await collect(corrector) { c in
            // 0..4 drift +4 cm/leg; KF 5 is physically back at origin but ARKit
            // reports x = 0.20.
            await c.consider(wallFrame(0, pose: matrix_identity_float4x4))
            for i in 1...4 { await c.consider(wallFrame(i, pose: translated(0.04 * Float(i)))) }
            await c.consider(wallFrame(5, pose: translated(0.20)))
        }

        XCTAssertFalse(updates.isEmpty, "the pose graph produced a correction")
        let last = updates.last!
        XCTAssertGreaterThan(last.movedKeyframes, 0)
        XCTAssertEqual(last.nodes.count, last.keyframes)
    }

    func testClearResets() async {
        let corrector = LiveDriftCorrector()
        await corrector.consider(wallFrame(0, pose: matrix_identity_float4x4))
        await corrector.consider(wallFrame(1, pose: translated(0.1)))
        await corrector.clear()
        let d = await corrector.driftCorrection()
        XCTAssertEqual(d, matrix_identity_float4x4)
    }
}

private actor Box {
    var items: [LiveDriftCorrector.Update] = []
    func append(_ u: LiveDriftCorrector.Update) { items.append(u) }
}
