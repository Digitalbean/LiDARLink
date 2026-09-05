import XCTest
import simd
@testable import LiDARLinkShared

final class SurfelEngineTests: XCTestCase {

    private func planeKeyframe(_ id: Int32, pose: simd_float4x4, relative: simd_float4x4) -> Keyframe {
        let w = 80, h = 60
        let d = [Float16](repeating: 1.6, count: w * h)
        return Keyframe(id: id, capturedAtMs: UInt64(id) * 100,
                        depth: DepthPayload(width: w, height: h, depthData: Binary.data(from: d), confidenceData: nil),
                        depthScale: 1,
                        depthIntrinsics: CameraIntrinsics(fx: 90, fy: 90, cx: 40, cy: 30, imageWidth: w, imageHeight: h),
                        arkitPose: pose, relativePose: relative)
    }

    private func collect(_ engine: SurfelEngine, _ body: (SurfelEngine) async -> Void) async -> [SurfelEngine.Event] {
        let box = Box()
        let task = Task { for await e in engine.events { await box.append(e) } }
        await body(engine)
        try? await Task.sleep(nanoseconds: 30_000_000)
        task.cancel()
        return await box.items
    }

    func testBuildsASurfelCloudLive() async {
        let engine = SurfelEngine()
        var tuning = SurfelEngine.LiveTuning()
        tuning.loopClosureEveryNKeyframes = 0
        tuning.snapshotIntervalMs = 0

        let events = await collect(engine) { engine in
            var pose = matrix_identity_float4x4
            var rel = matrix_identity_float4x4
            rel.columns.3 = SIMD4<Float>(0.04, 0, 0, 1)
            for i in 0..<10 {
                if i > 0 { pose = pose * rel }
                await engine.ingestLive(planeKeyframe(Int32(i), pose: pose, relative: i == 0 ? matrix_identity_float4x4 : rel),
                                        tuning: tuning)
            }
        }

        let clouds = events.compactMap { e -> [Surfel]? in
            if case .surfels(let s, _) = e { return s }; return nil
        }
        XCTAssertFalse(clouds.isEmpty)
        XCTAssertGreaterThan(clouds.last?.count ?? 0, 30)
        XCTAssertTrue(clouds.last?.allSatisfy { simd_length($0.normal) > 0.5 } ?? false)
        let count = await engine.keyframeCount
        XCTAssertEqual(count, 10)
    }

    func testClearEmitsAndEmpties() async {
        let engine = SurfelEngine()
        let events = await collect(engine) { engine in
            await engine.ingestLive(planeKeyframe(0, pose: matrix_identity_float4x4, relative: matrix_identity_float4x4))
            await engine.clear()
        }
        XCTAssertTrue(events.contains { if case .cleared = $0 { return true }; return false })
        let cloud = await engine.cloud()
        XCTAssertTrue(cloud.isEmpty)
    }
}

private actor Box {
    var items: [SurfelEngine.Event] = []
    func append(_ e: SurfelEngine.Event) { items.append(e) }
}
