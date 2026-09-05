import XCTest
import Darwin
import simd
@testable import LiDARLinkShared

/// End-to-end wire tests over a real POSIX socket pair. Network.framework's
/// listener cannot start inside this xctest environment (EINVAL), so the tests
/// use raw sockets — the identical byte stream our codec sees on the wire.
final class LoopbackStreamTests: XCTestCase {
    private func makeSocketPair() throws -> (client: Int32, server: Int32) {
        var fds: [Int32] = [0, 0]
        let result = Darwin.socketpair(AF_UNIX, SOCK_STREAM, 0, &fds)
        XCTAssertEqual(result, 0, "socketpair failed errno=\(errno)")
        return (fds[0], fds[1])
    }

    private func writeAll(_ fd: Int32, _ data: Data) {
        data.withUnsafeBytes { raw in
            guard let base = raw.baseAddress else { return }
            var offset = 0
            while offset < raw.count {
                let n = Darwin.write(fd, base.advanced(by: offset), raw.count - offset)
                XCTAssertGreaterThan(n, 0, "write failed errno=\(errno)")
                offset += n
            }
        }
    }

    /// Reads the socket in small chunks (exercising partial-frame handling) and
    /// delivers each chunk to `onChunk` on a background thread.
    private func startReader(_ fd: Int32, chunkSize: Int = 7, onChunk: @escaping (Data) -> Void) {
        DispatchQueue.global(qos: .userInitiated).async {
            var buffer = [UInt8](repeating: 0, count: chunkSize)
            while true {
                let n = Darwin.read(fd, &buffer, chunkSize)
                if n <= 0 { return }
                onChunk(Data(buffer[0..<n]))
            }
        }
    }

    func testHelloAndFrameExchangeOverSocket() throws {
        let pair = try makeSocketPair()
        defer { close(pair.client); close(pair.server) }

        let gotHello = DispatchSemaphore(value: 0)
        let gotFrame = DispatchSemaphore(value: 0)
        let decoder = MessageDecoder()
        var decodedHello = false
        var decodedSequence: UInt32?
        var decodedDepthCount = 0

        startReader(pair.server) { chunk in
            for item in decoder.append(chunk) {
                guard case .message(let message, let header) = item else { continue }
                switch message {
                case .hello:
                    decodedHello = true
                    gotHello.signal()
                case .frameMeta:
                    decodedSequence = header.sequence
                case .frameDepth(let depth):
                    decodedDepthCount = depth.width * depth.height
                    gotFrame.signal()
                default:
                    break
                }
            }
        }

        let hello = try MessageCodec.encode(
            .hello(HelloMessage(protocolVersion: ProtocolVersion.current,
                                appName: "LiDARLinkTests",
                                appVersion: "1.0",
                                deviceName: "TestPhone",
                                capabilities: Capabilities(supportsDepth: true, supportsMesh: true, supportsColor: true))),
            sequence: 0, timestampMs: 100)

        let depth = TestFixtures.depthPayload(width: 2, height: 2)
        let metadata = FrameMetadata(sequence: 7, captureTimestampMs: 200, depthWidth: 2, depthHeight: 2, depthScale: 1, hasConfidence: true, depthIsSmoothed: false, intrinsics: TestFixtures.intrinsics(), pose: TestFixtures.pose(), colorWidth: 0, colorHeight: 0, colorJpegSize: 0, downsampleFactor: 1)
        let metaData = try MessageCodec.encode(.frameMeta(metadata), sequence: 7, timestampMs: 200)
        let depthData = try MessageCodec.encode(.frameDepth(depth), sequence: 7, timestampMs: 200)

        // Interleave the three frames byte-by-byte to stress incremental decoding.
        let combined = hello + metaData + depthData
        for byte in combined {
            writeAll(pair.client, Data([byte]))
            usleep(200)
        }

        XCTAssertEqual(gotHello.wait(timeout: .now() + 10), .success, "hello not received")
        XCTAssertEqual(gotFrame.wait(timeout: .now() + 10), .success, "frame not received")
        XCTAssertTrue(decodedHello)
        XCTAssertEqual(decodedSequence, 7)
        XCTAssertEqual(decodedDepthCount, 4)
    }

    func testVersionMismatchOverSocket() throws {
        let pair = try makeSocketPair()
        defer { close(pair.client); close(pair.server) }

        let mismatchSent = DispatchSemaphore(value: 0)
        let serverDecoder = MessageDecoder()

        startReader(pair.server) { chunk in
            for item in serverDecoder.append(chunk) {
                guard case .message(let message, _) = item else { continue }
                if case .hello(let hello) = message, hello.protocolVersion != ProtocolVersion.current {
                    let reply = VersionMismatchMessage(clientVersion: hello.protocolVersion, serverVersion: ProtocolVersion.current)
                    if let payload = try? MessageCodec.encode(.versionMismatch(reply), sequence: 0, timestampMs: 1) {
                        self.writeAll(pair.server, payload)
                        mismatchSent.signal()
                    }
                }
            }
        }

        let clientDecoder = MessageDecoder()
        var clientGotMismatch = false
        let mismatchReceived = DispatchSemaphore(value: 0)
        startReader(pair.client) { chunk in
            for item in clientDecoder.append(chunk) {
                guard case .message(let message, _) = item else { continue }
                if case .versionMismatch = message {
                    clientGotMismatch = true
                    mismatchReceived.signal()
                }
            }
        }

        let oldHello = try MessageCodec.encode(
            .hello(HelloMessage(protocolVersion: 99, appName: "Old", appVersion: "0.1", deviceName: "OldPhone", capabilities: Capabilities(supportsDepth: true, supportsMesh: false, supportsColor: true))),
            sequence: 0, timestampMs: 1)
        writeAll(pair.client, oldHello)

        XCTAssertEqual(mismatchSent.wait(timeout: .now() + 10), .success, "server did not reply")
        XCTAssertEqual(mismatchReceived.wait(timeout: .now() + 10), .success, "client did not receive mismatch")
        XCTAssertTrue(clientGotMismatch)
    }
}
