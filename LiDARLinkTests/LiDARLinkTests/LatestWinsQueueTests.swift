import XCTest
@testable import LiDARLinkShared

final class LatestWinsQueueTests: XCTestCase {
    func testSubmitThenTake() {
        let queue = LatestWinsQueue<Int>()
        XCTAssertFalse(queue.submit(1))
        XCTAssertEqual(queue.take(), 1)
        XCTAssertNil(queue.take())
    }

    func testObsoleteFramesAreDropped() {
        let queue = LatestWinsQueue<Int>()
        _ = queue.submit(1)
        XCTAssertTrue(queue.submit(2)) // replaces 1 → drop counted
        XCTAssertTrue(queue.submit(3)) // replaces 2 → drop counted
        XCTAssertEqual(queue.droppedCount, 2)
        XCTAssertEqual(queue.take(), 3)
    }

    func testClear() {
        let queue = LatestWinsQueue<Int>()
        _ = queue.submit(1)
        queue.clear()
        XCTAssertNil(queue.take())
    }
}
