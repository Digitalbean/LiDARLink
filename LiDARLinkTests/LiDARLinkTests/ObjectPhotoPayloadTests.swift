import XCTest
import simd
@testable import LiDARLinkShared

final class ObjectPhotoPayloadTests: XCTestCase {

    func testRoundTripsThroughTheWireCodec() throws {
        let jpeg = Data((0..<5000).map { UInt8($0 % 251) })
        let original = ObjectPhotoPayload(
            index: 7, total: 36, jpegData: jpeg,
            gravity: simd_normalize(SIMD3<Float>(0.1, -0.98, 0.05)),
            intrinsics: CameraIntrinsics(fx: 1590, fy: 1590, cx: 2016, cy: 1512,
                                         imageWidth: 4032, imageHeight: 3024))

        let encoded = try MessageCodec.encode(.objectPhoto(original), sequence: 3, timestampMs: 111)
        let items = MessageDecoder().append(encoded)
        XCTAssertEqual(items.count, 1)
        guard case .message(.objectPhoto(let decoded), _) = items[0] else {
            return XCTFail("expected objectPhoto, got \(items)")
        }
        XCTAssertEqual(decoded.index, 7)
        XCTAssertEqual(decoded.total, 36)
        XCTAssertEqual(decoded.jpegData, jpeg)
        XCTAssertEqual(decoded.intrinsics.imageWidth, 4032)
        XCTAssertEqual(simd_length(decoded.gravity - original.gravity), 0, accuracy: 1e-5)
    }

    func testRejectsTruncatedPayload() {
        XCTAssertNil(ObjectPhotoPayload.decodeBinaryPayload(Data([1, 2, 3])))
    }
}
