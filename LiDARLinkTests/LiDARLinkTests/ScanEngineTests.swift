import XCTest
import simd
@testable import LiDARLinkShared

final class ScanEngineTests: XCTestCase {

    /// Feeds frames and returns every output the engine emitted.
    private func run(_ configuration: ScanEngine.Configuration = defaultConfig(),
                     frames: [ScanFrame],
                     extra: (ScanEngine) async -> Void = { _ in }) async -> [ScanEngine.Output] {
        let engine = ScanEngine(configuration: configuration)
        let collected = Collector()
        let task = Task { for await out in engine.outputs { await collected.append(out) } }
        for frame in frames { await engine.ingest(frame) }
        await extra(engine)
        try? await Task.sleep(nanoseconds: 30_000_000)
        task.cancel()
        return await collected.items
    }

    private static func defaultConfig() -> ScanEngine.Configuration {
        var c = ScanEngine.Configuration()
        c.minConfidence = 0
        c.autoClean = false
        return c
    }

    private func frame(_ sequence: UInt32, atMs ms: UInt64, x: Float = 0) -> ScanFrame {
        var pose = matrix_identity_float4x4
        pose.columns.3 = SIMD4<Float>(x, 0, 0, 1)
        return ScanFrame(sequence: sequence, captureTimestampMs: ms, pose: pose,
                         intrinsics: CameraIntrinsics(fx: 10, fy: 10, cx: 2, cy: 2, imageWidth: 4, imageHeight: 4),
                         depth: TestFixtures.depthPayload(), color: nil,
                         depthIsSmoothed: false, depthScale: 1)
    }

    func testFirstFrameEmitsFirstPointsThenStats() async {
        let outputs = await run(frames: [frame(1, atMs: 0)])
        XCTAssertTrue(outputs.contains { if case .firstPoints(let p) = $0 { return !p.isEmpty }; return false })
        XCTAssertTrue(outputs.contains { if case .stats(let s) = $0 { return s.pointCount > 0 }; return false })
    }

    func testThrottleDropsFramesInsideTheBuildInterval() async {
        // Three frames 50 ms apart — only the first (t=0) and the third (t=100
        // is < 150) ... actually only t=0 processes; t=200 would too.
        let outputs = await run(frames: [frame(1, atMs: 0), frame(2, atMs: 50), frame(3, atMs: 100)])
        let statEvents = outputs.filter { if case .stats = $0 { return true }; return false }
        XCTAssertEqual(statEvents.count, 1)
    }

    func testProcessesAgainAfterTheThrottleInterval() async {
        let outputs = await run(frames: [frame(1, atMs: 0), frame(2, atMs: 200, x: 3)])
        let statEvents = outputs.filter { if case .stats = $0 { return true }; return false }
        XCTAssertEqual(statEvents.count, 2)
    }

    func testRescanningTheSameViewAddsNoNewPoints() async {
        // Same pose + depth twice (200 ms apart so both process). The voxel map
        // dedups the second one to nothing.
        let outputs = await run(frames: [frame(1, atMs: 0), frame(2, atMs: 200)])
        let stats = outputs.compactMap { out -> ScanEngine.Stats? in
            if case .stats(let s) = out { return s }; return nil
        }
        XCTAssertEqual(stats.count, 2)
        XCTAssertGreaterThan(stats[0].lastFrameNewPoints, 0)
        XCTAssertEqual(stats[1].lastFrameNewPoints, 0)
        XCTAssertEqual(stats[0].pointCount, stats[1].pointCount) // unchanged
    }

    func testClearEmitsClearedAndZeroStats() async {
        let outputs = await run(frames: [frame(1, atMs: 0)]) { engine in
            await engine.clear()
        }
        XCTAssertTrue(outputs.contains { if case .cleared = $0 { return true }; return false })
        if case .stats(let last)? = outputs.last(where: { if case .stats = $0 { return true }; return false }) {
            XCTAssertEqual(last.pointCount, 0)
        } else {
            XCTFail("expected a stats event")
        }
    }

    func testSnapshotReturnsTheFusedCloud() async {
        let engine = ScanEngine(configuration: Self.defaultConfig())
        await engine.ingest(frame(1, atMs: 0))
        try? await Task.sleep(nanoseconds: 20_000_000)
        let snapshot = await engine.pointSnapshot()
        XCTAssertFalse(snapshot.isEmpty)
    }

    func testLoadPointsReplacesTheCloud() async {
        let points = (0..<500).map {
            PointCloudPoint(position: SIMD3<Float>(Float($0) * 0.01, 0, 0), color: .zero, confidence: 2)
        }
        let outputs = await run(frames: []) { engine in
            await engine.loadPoints(points)
        }
        XCTAssertTrue(outputs.contains { if case .replaced(let p, _, _) = $0 { return !p.isEmpty }; return false })
    }

    // MARK: TSDF path

    /// A fronto-parallel wall frame at `depthMetres`, 64x48, pose translated by `x`.
    private func wallFrame(_ sequence: UInt32, atMs ms: UInt64, depthMetres: Float = 1.5, x: Float = 0) -> ScanFrame {
        let w = 64, h = 48
        let depth = [Float16](repeating: Float16(depthMetres), count: w * h)
        var pose = matrix_identity_float4x4
        pose.columns.3 = SIMD4<Float>(x, 0, 0, 1)
        return ScanFrame(sequence: sequence, captureTimestampMs: ms, pose: pose,
                         intrinsics: CameraIntrinsics(fx: 50, fy: 50, cx: 32, cy: 24, imageWidth: w, imageHeight: h),
                         depth: DepthPayload(width: w, height: h, depthData: Binary.data(from: depth), confidenceData: nil),
                         color: nil, depthIsSmoothed: false, depthScale: 1)
    }

    private static func tsdfConfig() -> ScanEngine.Configuration {
        var c = defaultConfig()
        c.tsdfEnabled = true
        c.voxelSizeMeters = 0.03
        return c
    }

    func testTSDFModeEmitsASurfaceMesh() async {
        // Spans past the 1.5 s extraction interval.
        let frames = (0..<12).map { wallFrame(UInt32($0), atMs: UInt64($0) * 200, depthMetres: 1.5) }
        let outputs = await run(Self.tsdfConfig(), frames: frames)
        let mesh = outputs.compactMap { out -> MarchingCubes.Mesh? in
            if case .surfaceMesh(let m?, let recenter) = out, recenter { return m }
            return nil
        }.first
        XCTAssertNotNil(mesh)
        XCTAssertGreaterThan(mesh?.faces.count ?? 0, 30)
        let zs = (mesh?.vertices ?? []).map { $0.z }
        XCTAssertEqual(zs.reduce(0, +) / Float(max(zs.count, 1)), -1.5, accuracy: 0.08)
    }

    func testTSDFClearResets() async {
        let frames = (0..<6).map { wallFrame(UInt32($0), atMs: UInt64($0) * 200) }
        let outputs = await run(Self.tsdfConfig(), frames: frames) { engine in
            await engine.clear()
        }
        XCTAssertTrue(outputs.contains { if case .cleared = $0 { return true }; return false })
        let last = outputs.compactMap { if case .stats(let s) = $0 { return s } else { return nil } }.last
        XCTAssertEqual(last?.pointCount, 0)
    }
}

private actor Collector {
    var items: [ScanEngine.Output] = []
    func append(_ item: ScanEngine.Output) { items.append(item) }
}
