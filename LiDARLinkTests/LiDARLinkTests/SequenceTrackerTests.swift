import XCTest
@testable import LiDARLinkShared

final class SequenceTrackerTests: XCTestCase {
    func testExpectedSequence() {
        let tracker = SequenceTracker()
        XCTAssertEqual(tracker.record(1), .expected)
        XCTAssertEqual(tracker.record(2), .expected)
        XCTAssertEqual(tracker.record(3), .expected)
        XCTAssertEqual(tracker.receivedCount, 3)
    }

    func testDuplicate() {
        let tracker = SequenceTracker()
        _ = tracker.record(5)
        XCTAssertEqual(tracker.record(5), .duplicate)
        XCTAssertEqual(tracker.duplicateCount, 1)
    }

    func testGap() {
        let tracker = SequenceTracker()
        _ = tracker.record(1)
        XCTAssertEqual(tracker.record(5), .gap(skipped: 3))
        XCTAssertEqual(tracker.gapCount, 1)
    }

    func testReordered() {
        let tracker = SequenceTracker()
        _ = tracker.record(10)
        XCTAssertEqual(tracker.record(10), .duplicate)
        XCTAssertEqual(tracker.record(8), .reordered)
        XCTAssertEqual(tracker.reorderedCount, 1)
    }

    func testWraparound() {
        let tracker = SequenceTracker()
        _ = tracker.record(UInt32.max - 1)
        XCTAssertEqual(tracker.record(UInt32.max), .expected)
        XCTAssertEqual(tracker.record(0), .expected)
        XCTAssertEqual(tracker.record(1), .expected)
        XCTAssertEqual(tracker.gapCount, 0)
    }

    func testWraparoundDuplicate() {
        let tracker = SequenceTracker()
        _ = tracker.record(UInt32.max)
        XCTAssertEqual(tracker.record(UInt32.max), .duplicate)
    }
}
