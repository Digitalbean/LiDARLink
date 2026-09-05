import XCTest
@testable import LiDARLinkShared

final class WireCompressionTests: XCTestCase {
    func testCompressDecompressRoundTrip() throws {
        var data = Data()
        for i in 0..<4096 {
            var value = UInt16(i % 251).littleEndian
            withUnsafeBytes(of: &value) { data.append(contentsOf: $0) }
        }
        guard let compressed = WireCompression.compress(data) else {
            XCTFail("expected compression to be beneficial")
            return
        }
        XCTAssertLessThan(compressed.count, data.count)
        let restored = WireCompression.decompress(compressed, expectedSize: data.count)
        XCTAssertEqual(restored, data)
    }

    func testCompressionNotBeneficialReturnsNil() {
        let tiny = Data([1, 2, 3, 4])
        XCTAssertNil(WireCompression.compress(tiny))
    }

    func testDecompressCorruptDataFails() {
        XCTAssertNil(WireCompression.decompress(Data([1, 2, 3]), expectedSize: 100))
    }

    func testDepthPayloadCompressedRoundTrip() throws {
        // 32x32 patterned depth so compression is beneficial.
        let count = 32 * 32
        var floats: [Float16] = []
        for i in 0..<count {
            floats.append(Float16(Float((i * 7) % 200) / 10))
        }
        let payload = DepthPayload(width: 32, height: 32,
                                   depthData: Binary.data(from: floats),
                                   confidenceData: Binary.data(from: [UInt8](repeating: 2, count: count)))
        let wire = payload.binaryPayload()
        let decoded = DepthPayload.decodeBinaryPayload(wire)
        XCTAssertEqual(decoded?.depthData, payload.depthData)
        XCTAssertEqual(decoded?.confidenceData, payload.confidenceData)
        XCTAssertEqual(decoded?.width, 32)
    }

    func testWireFrameBuilderAndCodecWithCompressedDepth() throws {
        let depth = TestFixtures.depthPayload(width: 32, height: 32)
        let messages = WireFrameBuilder.frameMessages(sequence: 1, captureTimestampMs: 1,
                                                      pose: TestFixtures.pose(),
                                                      intrinsics: TestFixtures.intrinsics(),
                                                      depth: depth, color: nil,
                                                      depthIsSmoothed: false, depthScale: 1,
                                                      downsampleFactor: 1, meshChunks: [])
        let data = try MessageCodec.encode(messages[1], sequence: 1, timestampMs: 1)
        let decoder = MessageDecoder()
        let items = decoder.append(data)
        guard case .message(.frameDepth(let decoded), _) = items[0] else {
            XCTFail("expected frameDepth")
            return
        }
        XCTAssertEqual(decoded.depthData, depth.depthData)
    }
}
