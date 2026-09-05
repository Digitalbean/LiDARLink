import XCTest
import simd
@testable import LiDARLinkShared

final class StorageTests: XCTestCase {
    private var tempDir: URL!

    override func setUpWithError() throws {
        tempDir = FileManager.default.temporaryDirectory
            .appendingPathComponent("LiDARLinkTests-\(UUID().uuidString)", isDirectory: true)
        try FileManager.default.createDirectory(at: tempDir, withIntermediateDirectories: true)
    }

    override func tearDownWithError() throws {
        try? FileManager.default.removeItem(at: tempDir)
    }

    func testFrameRecordCodecRoundTrip() throws {
        let frame = TestFixtures.frame(sequence: 7)
        let data = try FrameRecordCodec.encode(frame)
        let decoded = try FrameRecordCodec.decode(data)
        XCTAssertEqual(decoded, frame)
    }

    func testFrameRecordCodecRejectsCorruption() {
        let frame = TestFixtures.frame(sequence: 1)
        let data = try! FrameRecordCodec.encode(frame)
        var corrupted = data
        corrupted[corrupted.count - 1] ^= 0xFF
        XCTAssertThrowsError(try FrameRecordCodec.decode(corrupted))
    }

    func testMeshRecordCodecRoundTrip() throws {
        let mesh = TestFixtures.mesh()
        let data = try MeshRecordCodec.encode(mesh)
        let decoded = try MeshRecordCodec.decode(data)
        XCTAssertEqual(decoded, mesh)
    }

    func testManifestSaveLoadRoundTrip() throws {
        // Whole-second dates so ISO8601 encoding round-trips exactly.
        let startedAt = Date(timeIntervalSince1970: 1_700_000_000)
        let endedAt = Date(timeIntervalSince1970: 1_700_000_100)
        let manifest = RecordingManifest(formatVersion: 1, appName: "Test", appVersion: "1.0", deviceName: "iPhone", startedAt: startedAt, endedAt: endedAt, frameCount: 10, meshCount: 2, depthWidth: 256, depthHeight: 192, colorWidth: 640, colorHeight: 480, targetFPS: 12, protocolVersion: 1, notes: "hello")
        try ScanRecordingStore.saveManifest(manifest, to: tempDir)
        let loaded = try ScanRecordingStore.loadManifest(from: tempDir)
        XCTAssertEqual(loaded, manifest)
    }

    func testAppendAndLoadFrames() throws {
        try ScanRecordingStore.createFramesFile(in: tempDir)
        let frames = (0..<5).map { TestFixtures.frame(sequence: UInt32($0)) }
        try ScanRecordingStore.appendFrames(frames, to: tempDir)
        let loaded = try ScanRecordingStore.loadFrames(from: tempDir)
        XCTAssertEqual(loaded, frames)
    }

    func testCorruptFrameRecordThrows() throws {
        try ScanRecordingStore.createFramesFile(in: tempDir)
        let frames = [TestFixtures.frame(sequence: 0)]
        try ScanRecordingStore.appendFrames(frames, to: tempDir)
        let url = tempDir.appendingPathComponent(ScanRecordingStore.framesFileName)
        var data = try Data(contentsOf: url)
        // Truncate the record length to claim more bytes than exist.
        data.replaceSubrange(data.startIndex..<data.startIndex + 4, with: Data([0xFF, 0xFF, 0xFF, 0x7F]))
        try data.write(to: url)
        XCTAssertThrowsError(try ScanRecordingStore.loadFrames(from: tempDir))
    }

    func testAppendAndLoadMeshes() throws {
        try ScanRecordingStore.createMeshesFile(in: tempDir)
        let meshes = [TestFixtures.mesh(), TestFixtures.mesh()]
        try ScanRecordingStore.appendMeshes(meshes, to: tempDir)
        let loaded = try ScanRecordingStore.loadMeshes(from: tempDir)
        XCTAssertEqual(loaded.count, 2)
        XCTAssertEqual(loaded[0], meshes[0])
    }

    func testLoadMissingRecordingThrows() {
        let missing = tempDir.appendingPathComponent("nope")
        XCTAssertThrowsError(try ScanRecordingStore.loadManifest(from: missing))
    }

    func testNewRecordingDirectoryCreatesUniqueFolders() throws {
        let a = try ScanRecordingStore.newRecordingDirectory(in: tempDir, date: Date(timeIntervalSince1970: 1_000))
        let b = try ScanRecordingStore.newRecordingDirectory(in: tempDir, date: Date(timeIntervalSince1970: 1_000))
        XCTAssertNotEqual(a, b)
        XCTAssertTrue(FileManager.default.fileExists(atPath: a.path))
        XCTAssertTrue(FileManager.default.fileExists(atPath: b.path))
    }
}
