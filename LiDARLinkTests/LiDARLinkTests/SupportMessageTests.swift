import XCTest
@testable import LiDARLinkShared

final class SupportMessageTests: XCTestCase {
    func testSimulatorMessage() {
        let message = SupportMessageBuilder.explanation(lidarAvailable: false, worldTrackingAvailable: false, isSimulator: true, depthSupported: false, meshSupported: false)
        XCTAssertTrue(message.contains("Simulator"))
        XCTAssertTrue(message.contains("physical iPhone"))
    }

    func testNoLidarMessage() {
        let message = SupportMessageBuilder.explanation(lidarAvailable: false, worldTrackingAvailable: true, isSimulator: false, depthSupported: false, meshSupported: false)
        XCTAssertTrue(message.contains("LiDAR scanner"))
    }

    func testLidarWithoutMeshMessage() {
        let message = SupportMessageBuilder.explanation(lidarAvailable: true, worldTrackingAvailable: true, isSimulator: false, depthSupported: true, meshSupported: false)
        XCTAssertTrue(message.contains("point-cloud scanning will still work"))
    }

    func testFullSupportMessage() {
        let message = SupportMessageBuilder.explanation(lidarAvailable: true, worldTrackingAvailable: true, isSimulator: false, depthSupported: true, meshSupported: true)
        XCTAssertTrue(message.contains("LiDAR is available"))
    }
}
