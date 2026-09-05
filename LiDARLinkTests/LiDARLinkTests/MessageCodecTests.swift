import XCTest
import simd
@testable import LiDARLinkShared

final class MessageCodecTests: XCTestCase {
    private func roundTrip(_ message: Message, file: StaticString = #filePath, line: UInt = #line) throws -> Message {
        let data = try MessageCodec.encode(message, sequence: 42, timestampMs: 987_654)
        let decoder = MessageDecoder()
        let items = decoder.append(data)
        guard case .message(let decoded, let header) = items.first else {
            XCTFail("Expected a message, got \(items)", file: file, line: line)
            throw CodecError.malformedPayload("test")
        }
        XCTAssertEqual(header.sequence, 42, file: file, line: line)
        XCTAssertEqual(header.timestampMs, 987_654, file: file, line: line)
        return decoded
    }

    func testRoundTripsEveryMessageType() throws {
        let hello = Message.hello(HelloMessage(protocolVersion: 1, appName: "app", appVersion: "1.0", deviceName: "Phone", capabilities: Capabilities(supportsDepth: true, supportsMesh: true, supportsColor: true)))
        XCTAssertEqual(try roundTrip(hello), hello)

        let ack = Message.helloAck(HelloAckMessage(accepted: true, protocolVersion: 1, serverName: "Mac", reason: nil))
        XCTAssertEqual(try roundTrip(ack), ack)

        let mismatch = Message.versionMismatch(VersionMismatchMessage(clientVersion: 2, serverVersion: 1))
        XCTAssertEqual(try roundTrip(mismatch), mismatch)

        let control = Message.control(ControlMessage(action: .pause))
        XCTAssertEqual(try roundTrip(control), control)

        let metadata = FrameMetadata(sequence: 5, captureTimestampMs: 100, depthWidth: 4, depthHeight: 4, depthScale: 1, hasConfidence: true, depthIsSmoothed: false, intrinsics: TestFixtures.intrinsics(), pose: TestFixtures.pose(), colorWidth: 640, colorHeight: 480, colorJpegSize: 1234, downsampleFactor: 2)
        XCTAssertEqual(try roundTrip(.frameMeta(metadata)), .frameMeta(metadata))

        let depth = TestFixtures.depthPayload()
        XCTAssertEqual(try roundTrip(.frameDepth(depth)), .frameDepth(depth))

        let color = ColorPayload(width: 4, height: 4, jpegData: Data([0xFF, 0xD8, 0xFF, 0xE0, 1, 2, 3]))
        XCTAssertEqual(try roundTrip(.frameColor(color)), .frameColor(color))

        let meshMeta = MeshMetadata(anchorID: UUID(), chunkIndex: 0, chunkCount: 2, timestampMs: 99)
        XCTAssertEqual(try roundTrip(.meshMeta(meshMeta)), .meshMeta(meshMeta))

        // Transform round-trips (nil and non-nil).
        var transformed = matrix_identity_float4x4
        transformed.columns.3 = simd_float4(1, 2, 3, 1)
        let meshMetaWithTransform = MeshMetadata(anchorID: UUID(), chunkIndex: 0, chunkCount: 2, timestampMs: 99, transform: transformed)
        XCTAssertEqual(try roundTrip(.meshMeta(meshMetaWithTransform)), .meshMeta(meshMetaWithTransform))

        let meshChunk = MeshChunk(metadata: meshMeta, data: MeshChunk.pack(vertices: [SIMD3<Float>(1, 2, 3)], normals: [SIMD3<Float>(0, 0, 1)], faces: [0, 0, 0]))
        // The anchorID travels in the preceding meshMeta message, so it is not
        // part of the meshData round-trip; verify index/count/data instead.
        guard case .meshData(let decodedChunk) = try roundTrip(.meshData(meshChunk)) else {
            XCTFail("expected meshData")
            return
        }
        XCTAssertEqual(decodedChunk.data, meshChunk.data)
        XCTAssertEqual(decodedChunk.metadata.chunkIndex, meshChunk.metadata.chunkIndex)
        XCTAssertEqual(decodedChunk.metadata.chunkCount, meshChunk.metadata.chunkCount)

        // A large, redundant chunk compresses on the wire and still round-trips.
        let bigVerts = (0..<2_000).map { SIMD3<Float>(Float($0 % 7), 1, 2) }
        let bigNormals = [SIMD3<Float>](repeating: SIMD3<Float>(0, 1, 0), count: 2_000)
        let bigFaces = (0..<6_000).map { UInt32($0 % 2_000) }
        let bigChunk = MeshChunk(metadata: meshMeta,
                                 data: MeshChunk.pack(vertices: bigVerts, normals: bigNormals, faces: bigFaces))
        XCTAssertLessThan(bigChunk.binaryPayload().count, bigChunk.data.count) // compressed
        guard case .meshData(let decodedBig) = try roundTrip(.meshData(bigChunk)),
              let unpacked = MeshChunk.unpack(decodedBig.data) else {
            XCTFail("big mesh chunk did not round-trip")
            return
        }
        XCTAssertEqual(unpacked.vertices.count, 2_000)
        XCTAssertEqual(unpacked.vertices[3], SIMD3<Float>(3, 1, 2))
        XCTAssertEqual(unpacked.faces.count, 6_000)

        let error = Message.error(ErrorMessage(code: .connectionDenied, message: "nope"))
        XCTAssertEqual(try roundTrip(error), error)

        let scanMode = Message.scanMode(ScanModeMessage(mode: .object, voxelCm: 0.5))
        XCTAssertEqual(try roundTrip(scanMode), scanMode)

        let meshRemove = Message.meshRemove(MeshRemoveMessage(anchorIDs: [UUID(), UUID()]))
        XCTAssertEqual(try roundTrip(meshRemove), meshRemove)

        let start = Message.control(ControlMessage(action: .start))
        XCTAssertEqual(try roundTrip(start), start)

        let remoteSettings = Message.remoteCaptureSettings(RemoteCaptureSettings(captureMode: "dense", roomPlanEnabled: true, maxDepthMeters: 8))
        XCTAssertEqual(try roundTrip(remoteSettings), remoteSettings)

        XCTAssertEqual(try roundTrip(.heartbeat), .heartbeat)
    }

    func testPartialFramesAreBufferedUntilComplete() throws {
        let data = try MessageCodec.encode(.heartbeat, sequence: 1, timestampMs: 2)
        let decoder = MessageDecoder()
        // Feed one byte at a time; nothing should decode until the final byte.
        for (index, byte) in data.enumerated() {
            let items = decoder.append(Data([byte]))
            if index < data.count - 1 {
                XCTAssertTrue(items.isEmpty, "decoded before complete at byte \(index)")
            } else {
                XCTAssertEqual(items.count, 1)
            }
        }
    }

    func testMultipleFramesInOneChunk() throws {
        let one = try MessageCodec.encode(.heartbeat, sequence: 1, timestampMs: 2)
        let two = try MessageCodec.encode(.control(ControlMessage(action: .resume)), sequence: 3, timestampMs: 4)
        let decoder = MessageDecoder()
        let items = decoder.append(one + two)
        XCTAssertEqual(items.count, 2)
    }

    func testCorruptedPayloadProducesCorruptItemAndNextFrameStillDecodes() throws {
        let good = try MessageCodec.encode(.heartbeat, sequence: 1, timestampMs: 2)
        // Flip a payload byte in a control frame (heartbeat has no payload).
        let badControl = try MessageCodec.encode(.control(ControlMessage(action: .pause)), sequence: 2, timestampMs: 3)
        var corrupted = badControl
        corrupted[corrupted.count - 5] ^= 0xFF // flip a payload byte
        let decoder = MessageDecoder()
        var items = decoder.append(corrupted)
        XCTAssertTrue(items.contains { if case .corrupt = $0 { return true }; return false })
        items = decoder.append(good)
        XCTAssertEqual(items.count, 1)
        if case .message(let message, _) = items[0] {
            XCTAssertEqual(message, .heartbeat)
        } else {
            XCTFail("expected heartbeat")
        }
    }

    func testGarbagePrefixIsResynced() throws {
        let good = try MessageCodec.encode(.heartbeat, sequence: 1, timestampMs: 2)
        let garbage = Data([0x00, 0x01, 0x02, 0x03, 0xFF, 0xFE])
        let decoder = MessageDecoder()
        let items = decoder.append(garbage + good)
        let messages = items.compactMap { item -> Message? in
            if case .message(let message, _) = item { return message }
            return nil
        }
        XCTAssertEqual(messages, [.heartbeat])
    }

    func testTruncatedHeaderDoesNotCrash() {
        let decoder = MessageDecoder()
        let items = decoder.append(Data([0x4C, 0x4C, 0x46, 0x31, 0x01]))
        XCTAssertTrue(items.isEmpty)
    }

    func testTruncatedUncompressedDepthIsRejectedWithoutCrash() throws {
        let malformed = DepthPayload(width: 4,
                                     height: 4,
                                     depthData: Data([0x00]),
                                     confidenceData: nil)
        let wire = try MessageCodec.encode(.frameDepth(malformed), sequence: 7, timestampMs: 11)
        let items = MessageDecoder().append(wire)

        XCTAssertEqual(items.count, 1)
        XCTAssertTrue(items.contains { if case .corrupt = $0 { return true }; return false })
    }

    func testOverflowingDepthDimensionsAreRejectedWithoutCrash() throws {
        let malformed = DepthPayload(width: Int(UInt32.max),
                                     height: Int(UInt32.max),
                                     depthData: Data(),
                                     confidenceData: nil)
        let wire = try MessageCodec.encode(.frameDepth(malformed), sequence: 8, timestampMs: 12)
        let items = MessageDecoder().append(wire)

        XCTAssertEqual(items.count, 1)
        XCTAssertTrue(items.contains { if case .corrupt = $0 { return true }; return false })
    }

    func testBinaryArrayReturnsEmptyForTruncatedInput() {
        let values: [UInt32] = Binary.array(from: Data([0x01]), count: 1)
        XCTAssertTrue(values.isEmpty)
    }

    func testIncompatibleVersionReported() throws {
        let data = try MessageCodec.encode(.heartbeat, sequence: 1, timestampMs: 2, version: 99)
        let decoder = MessageDecoder()
        let items = decoder.append(data)
        XCTAssertEqual(items.count, 1)
        if case .incompatibleVersion(let found, let expected) = items[0] {
            XCTAssertEqual(found, 99)
            XCTAssertEqual(expected, ProtocolVersion.current)
        } else {
            XCTFail("expected incompatibleVersion, got \(items)")
        }
    }

    func testOversizedPayloadRejectedAtEncode() {
        let huge = ColorPayload(width: 1, height: 1, jpegData: Data(repeating: 0, count: ProtocolVersion.maxPayloadSize + 1))
        XCTAssertThrowsError(try MessageCodec.encode(.frameColor(huge), sequence: 1, timestampMs: 2)) { error in
            XCTAssertEqual(error as? CodecError, .payloadTooLarge)
        }
    }

    func testOversizedLengthOnWireRejected() throws {
        // Craft a frame with an absurd payload length.
        var data = Data(ProtocolVersion.magicBytes)
        data.append(ProtocolVersion.current)
        data.append(MessageType.heartbeat.rawValue)
        data.append(0)
        data.append(0)
        Binary.append(UInt32(1), to: &data)
        Binary.append(UInt64(1), to: &data)
        Binary.append(UInt32(ProtocolVersion.maxPayloadSize + 10), to: &data)
        let decoder = MessageDecoder()
        let items = decoder.append(data)
        XCTAssertTrue(items.contains { if case .corrupt = $0 { return true }; return false })
    }

    func testUnknownMessageTypeReportedAsCorrupt() throws {
        var data = Data(ProtocolVersion.magicBytes)
        data.append(ProtocolVersion.current)
        data.append(200) // unknown type
        data.append(0)
        data.append(0)
        Binary.append(UInt32(1), to: &data)
        Binary.append(UInt64(1), to: &data)
        Binary.append(UInt32(0), to: &data)
        Binary.append(CRC32.checksum(Data()), to: &data)
        let decoder = MessageDecoder()
        let items = decoder.append(data)
        XCTAssertTrue(items.contains { if case .corrupt = $0 { return true }; return false })
    }

    func testWireFrameBuilderProducesCompleteFrame() throws {
        let depth = TestFixtures.depthPayload()
        let color = ColorPayload(width: 4, height: 4, jpegData: Data([1, 2, 3, 4]))
        let messages = WireFrameBuilder.frameMessages(sequence: 9,
                                                     captureTimestampMs: 1234,
                                                     pose: TestFixtures.pose(),
                                                     intrinsics: TestFixtures.intrinsics(),
                                                     depth: depth,
                                                     color: color,
                                                     depthIsSmoothed: true,
                                                     depthScale: 1.0,
                                                     downsampleFactor: 2,
                                                     meshChunks: [])
        XCTAssertEqual(messages.map(\.type), [.frameMeta, .frameDepth, .frameColor])

        let decoder = MessageDecoder()
        var items: [MessageDecoder.DecodedItem] = []
        for message in messages {
            items += decoder.append(try MessageCodec.encode(message, sequence: 9, timestampMs: 1234))
        }
        let assembler = FrameAssembler()
        var assembled: ScanFrame?
        for item in items {
            guard case .message(let message, let header) = item else { continue }
            switch message {
            case .frameMeta(let metadata):
                assembled = assembler.apply(metadata: metadata, timestampMs: header.timestampMs)
            case .frameDepth(let depth):
                assembled = assembler.apply(depth: depth, for: header.sequence, timestampMs: header.timestampMs)
            case .frameColor(let color):
                assembled = assembler.apply(color: color, for: header.sequence, timestampMs: header.timestampMs)
            default: break
            }
        }
        XCTAssertNotNil(assembled)
        XCTAssertEqual(assembled?.sequence, 9)
        XCTAssertEqual(assembled?.depth, depth)
        XCTAssertEqual(assembled?.color, color)
        XCTAssertTrue(assembled?.depthIsSmoothed == true)
        // Intrinsics must be expressed in depth-map coordinates.
        XCTAssertEqual(assembled?.intrinsics, TestFixtures.intrinsics().scaled(to: 4, height: 4))
    }
}
