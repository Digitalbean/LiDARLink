import XCTest
@testable import LiDARLinkShared

final class FrameAssemblerTests: XCTestCase {
    private func metadata(sequence: UInt32) -> FrameMetadata {
        FrameMetadata(sequence: sequence, captureTimestampMs: UInt64(sequence) * 100, depthWidth: 4, depthHeight: 4, depthScale: 1, hasConfidence: true, depthIsSmoothed: false, intrinsics: TestFixtures.intrinsics(), pose: TestFixtures.pose(), colorWidth: 0, colorHeight: 0, colorJpegSize: 0, downsampleFactor: 1)
    }

    func testMetaThenDepthCompletesFrame() {
        let assembler = FrameAssembler()
        XCTAssertNil(assembler.apply(metadata: metadata(sequence: 1), timestampMs: 100))
        XCTAssertEqual(assembler.incompleteCount, 1)
        let frame = assembler.apply(depth: TestFixtures.depthPayload(), for: 1, timestampMs: 100)
        XCTAssertNotNil(frame)
        XCTAssertEqual(frame?.sequence, 1)
        XCTAssertEqual(assembler.incompleteCount, 0)
        XCTAssertEqual(assembler.completedFrames, 1)
    }

    func testDepthBeforeMetaCompletesFrame() {
        let assembler = FrameAssembler()
        XCTAssertNil(assembler.apply(depth: TestFixtures.depthPayload(), for: 2, timestampMs: 100))
        let frame = assembler.apply(metadata: metadata(sequence: 2), timestampMs: 100)
        XCTAssertNotNil(frame)
        XCTAssertEqual(frame?.sequence, 2)
    }

    func testDuplicatePartsAreCountedAndIgnored() {
        let assembler = FrameAssembler()
        XCTAssertNil(assembler.apply(metadata: metadata(sequence: 3), timestampMs: 100))
        XCTAssertNil(assembler.apply(metadata: metadata(sequence: 3), timestampMs: 100))
        XCTAssertEqual(assembler.duplicateParts, 1)
        let frame = assembler.apply(depth: TestFixtures.depthPayload(), for: 3, timestampMs: 100)
        XCTAssertNotNil(frame)
        XCTAssertEqual(assembler.duplicateParts, 1)
    }

    func testColorAttachesToCompletedFrame() {
        let assembler = FrameAssembler()
        _ = assembler.apply(metadata: metadata(sequence: 4), timestampMs: 100)
        let color = ColorPayload(width: 2, height: 2, jpegData: Data([9]))
        assembler.apply(color: color, for: 4, timestampMs: 100)
        let frame = assembler.apply(depth: TestFixtures.depthPayload(), for: 4, timestampMs: 100)
        XCTAssertEqual(frame?.color, color)
    }

    func testOutOfOrderSequencesAreIndependent() {
        let assembler = FrameAssembler()
        _ = assembler.apply(metadata: metadata(sequence: 10), timestampMs: 100)
        _ = assembler.apply(depth: TestFixtures.depthPayload(), for: 5, timestampMs: 100)
        let frame5 = assembler.apply(metadata: metadata(sequence: 5), timestampMs: 100)
        XCTAssertEqual(frame5?.sequence, 5)
        let frame10 = assembler.apply(depth: TestFixtures.depthPayload(), for: 10, timestampMs: 100)
        XCTAssertEqual(frame10?.sequence, 10)
    }

    func testPruneRemovesStaleIncompleteEntries() {
        let assembler = FrameAssembler()
        _ = assembler.apply(metadata: metadata(sequence: 1), timestampMs: 100)
        let removed = assembler.prune(before: 500)
        XCTAssertEqual(removed, 1)
        XCTAssertEqual(assembler.incompleteCount, 0)
    }
}
