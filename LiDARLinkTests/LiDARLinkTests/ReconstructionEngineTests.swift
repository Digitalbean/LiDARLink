import XCTest
import simd
@testable import LiDARLinkShared

final class ReconstructionEngineTests: XCTestCase {

    private func wallKeyframe(_ id: Int32, pose: simd_float4x4, relative: simd_float4x4, depthM: Float = 1.5) -> Keyframe {
        let w = 64, h = 48
        let d = [Float16](repeating: Float16(depthM), count: w * h)
        return Keyframe(id: id, capturedAtMs: UInt64(id) * 100,
                        depth: DepthPayload(width: w, height: h, depthData: Binary.data(from: d), confidenceData: nil),
                        depthScale: 1,
                        depthIntrinsics: CameraIntrinsics(fx: 50, fy: 50, cx: 32, cy: 24, imageWidth: w, imageHeight: h),
                        arkitPose: pose, relativePose: relative)
    }

    private func collect(_ engine: ReconstructionEngine, _ body: (ReconstructionEngine) async -> Void) async -> [ReconstructionEngine.Event] {
        let box = Box()
        let task = Task { for await e in engine.events { await box.append(e) } }
        await body(engine)
        try? await Task.sleep(nanoseconds: 20_000_000)
        task.cancel()
        return await box.items
    }

    func testAddingKeyframesBuildsASurface() async {
        let engine = ReconstructionEngine()
        _ = await collect(engine) { engine in
            for i in 0..<4 {
                var pose = matrix_identity_float4x4
                pose.columns.3 = SIMD4<Float>(Float(i) * 0.05, 0, 0, 1)
                var rel = matrix_identity_float4x4
                rel.columns.3 = SIMD4<Float>(0.05, 0, 0, 1)
                await engine.add(wallKeyframe(Int32(i), pose: pose, relative: rel))
            }
        }
        let mesh = await engine.currentMesh()
        XCTAssertGreaterThan(mesh.faces.count, 30)
        XCTAssertTrue(mesh.vertices.allSatisfy { abs($0.z + 1.5) < 0.12 })
        let count = await engine.keyframeCount
        XCTAssertEqual(count, 4)
    }

    func testLoopClosureOptimisesAndReintegrates() async {
        let engine = ReconstructionEngine()
        // Drifted odometry: each leg reported 6 cm but was really 5 cm.
        var pose = matrix_identity_float4x4
        var relDrift = matrix_identity_float4x4
        relDrift.columns.3 = SIMD4<Float>(0.06, 0, 0, 1)

        let events = await collect(engine) { engine in
            await engine.add(wallKeyframe(0, pose: pose, relative: matrix_identity_float4x4))
            for i in 1..<5 {
                pose = pose * relDrift
                await engine.add(wallKeyframe(Int32(i), pose: pose, relative: relDrift))
            }
            // Loop closure: node 4 is really only 20 cm from node 0, not 24.
            var trueSpan = matrix_identity_float4x4
            trueSpan.columns.3 = SIMD4<Float>(0.20, 0, 0, 1)
            await engine.addLoopClosure(from: 0, to: 4, measured: trueSpan, weight: 6)
            _ = await engine.optimise(iterations: 25)
        }

        let optimised = events.compactMap { e -> (Float, Float, Int)? in
            if case .optimised(_, _, let i, let f, let r) = e { return (i, f, r) }
            return nil
        }.last
        XCTAssertNotNil(optimised)
        XCTAssertLessThan(optimised!.1, optimised!.0, "error should drop")
        XCTAssertGreaterThan(optimised!.2, 0, "moved keyframes get reintegrated")

        let span = simd_length(Lie.translation(await engine.pose(of: 4)!) - Lie.translation(await engine.pose(of: 0)!))
        XCTAssertLessThan(span, 0.235, "pulled back toward the true 20 cm")
        let mesh = await engine.currentMesh()
        XCTAssertGreaterThan(mesh.faces.count, 20, "surface survives reintegration")
    }

    func testIngestLiveBuildsAndTightensIncrementally() async {
        let engine = ReconstructionEngine()
        var tuning = ReconstructionEngine.LiveTuning()
        tuning.optimiseEveryNKeyframes = 4
        tuning.loopClosureEveryNKeyframes = 0          // keep the test cheap
        tuning.surfaceIntervalMs = 0                   // emit every keyframe

        let events = await collect(engine) { engine in
            var pose = matrix_identity_float4x4
            var rel = matrix_identity_float4x4
            rel.columns.3 = SIMD4<Float>(0.05, 0, 0, 1)
            for i in 0..<12 {
                if i > 0 { pose = pose * rel }
                await engine.ingestLive(wallKeyframe(Int32(i), pose: pose, relative: i == 0 ? matrix_identity_float4x4 : rel),
                                        tuning: tuning)
            }
        }

        XCTAssertTrue(events.contains { if case .surface = $0 { return true }; return false },
                      "live ingest streams surface updates")
        XCTAssertTrue(events.contains { if case .optimised = $0 { return true }; return false },
                      "the pose graph is tightened on the way")
        let count = await engine.keyframeCount
        XCTAssertEqual(count, 12)
        let mesh = await engine.currentMesh()
        XCTAssertGreaterThan(mesh.faces.count, 30)
    }

    func testClearResets() async {
        let engine = ReconstructionEngine()
        let events = await collect(engine) { engine in
            await engine.add(wallKeyframe(0, pose: matrix_identity_float4x4, relative: matrix_identity_float4x4))
            await engine.clear()
        }
        XCTAssertTrue(events.contains { if case .cleared = $0 { return true }; return false })
        let mesh = await engine.currentMesh()
        XCTAssertTrue(mesh.isEmpty)
    }
}

private actor Box {
    var items: [ReconstructionEngine.Event] = []
    func append(_ e: ReconstructionEngine.Event) { items.append(e) }
}
