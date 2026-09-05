import Foundation

public enum CodecError: Error, Equatable {
    case payloadTooLarge
    case unknownType(UInt8)
    case malformedPayload(String)
    case incompatibleVersion(found: UInt8, expected: UInt8)
}

/// Encodes and decodes the versioned, length-prefixed, CRC-protected wire format.
public enum MessageCodec {
    private static let log = Log.make("codec")

    /// Encodes a message into a complete wire frame (header + payload + CRC).
    public static func encode(_ message: Message, sequence: UInt32, timestampMs: UInt64, version: UInt8 = ProtocolVersion.current) throws -> Data {
        let payload: Data
        switch message {
        case .hello(let m): payload = try JSONEncoder.lidarLink.encode(m)
        case .helloAck(let m): payload = try JSONEncoder.lidarLink.encode(m)
        case .versionMismatch(let m): payload = try JSONEncoder.lidarLink.encode(m)
        case .control(let m): payload = try JSONEncoder.lidarLink.encode(m)
        case .frameMeta(let m): payload = try JSONEncoder.lidarLink.encode(m)
        case .meshMeta(let m): payload = try JSONEncoder.lidarLink.encode(m)
        case .error(let m): payload = try JSONEncoder.lidarLink.encode(m)
        case .scanMode(let m): payload = try JSONEncoder.lidarLink.encode(m)
        case .meshRemove(let m): payload = try JSONEncoder.lidarLink.encode(m)
        case .roomStructure(let m): payload = try JSONEncoder.lidarLink.encode(m)
        case .remoteCaptureSettings(let m): payload = try JSONEncoder.lidarLink.encode(m)
        case .objectPhoto(let m): payload = m.binaryPayload()
        case .frameDepth(let m): payload = m.binaryPayload()
        case .frameColor(let m): payload = m.binaryPayload()
        case .meshData(let m): payload = m.binaryPayload()
        case .heartbeat: payload = Data()
        }

        guard payload.count <= ProtocolVersion.maxPayloadSize else {
            throw CodecError.payloadTooLarge
        }

        var data = Data()
        data.append(contentsOf: ProtocolVersion.magicBytes)
        data.append(version)
        data.append(message.type.rawValue)
        data.append(0) // flags (reserved)
        data.append(0) // reserved
        Binary.append(sequence, to: &data)
        Binary.append(timestampMs, to: &data)
        Binary.append(UInt32(payload.count), to: &data)
        data.append(payload)
        Binary.append(CRC32.checksum(payload), to: &data)
        return data
    }

    static func decodePayload(type: MessageType, payload: Data, header: MessageHeader) throws -> Message {
        switch type {
        case .hello:
            return .hello(try JSONDecoder.lidarLink.decode(HelloMessage.self, from: payload))
        case .helloAck:
            return .helloAck(try JSONDecoder.lidarLink.decode(HelloAckMessage.self, from: payload))
        case .versionMismatch:
            return .versionMismatch(try JSONDecoder.lidarLink.decode(VersionMismatchMessage.self, from: payload))
        case .control:
            return .control(try JSONDecoder.lidarLink.decode(ControlMessage.self, from: payload))
        case .frameMeta:
            return .frameMeta(try JSONDecoder.lidarLink.decode(FrameMetadata.self, from: payload))
        case .meshMeta:
            return .meshMeta(try JSONDecoder.lidarLink.decode(MeshMetadata.self, from: payload))
        case .error:
            return .error(try JSONDecoder.lidarLink.decode(ErrorMessage.self, from: payload))
        case .scanMode:
            return .scanMode(try JSONDecoder.lidarLink.decode(ScanModeMessage.self, from: payload))
        case .meshRemove:
            return .meshRemove(try JSONDecoder.lidarLink.decode(MeshRemoveMessage.self, from: payload))
        case .roomStructure:
            return .roomStructure(try JSONDecoder.lidarLink.decode(RoomStructure.self, from: payload))
        case .remoteCaptureSettings:
            return .remoteCaptureSettings(try JSONDecoder.lidarLink.decode(RemoteCaptureSettings.self, from: payload))
        case .objectPhoto:
            guard let p = ObjectPhotoPayload.decodeBinaryPayload(payload) else { throw CodecError.malformedPayload("Object photo") }
            return .objectPhoto(p)
        case .frameDepth:
            guard let depth = DepthPayload.decodeBinaryPayload(payload) else {
                throw CodecError.malformedPayload("Depth payload")
            }
            return .frameDepth(depth)
        case .frameColor:
            guard let color = ColorPayload.decodeBinaryPayload(payload) else {
                throw CodecError.malformedPayload("Color payload")
            }
            return .frameColor(color)
        case .meshData:
            // The chunk is self-describing; metadata placeholders are overridden
            // by values embedded in the payload (chunkIndex, chunkCount).
            let placeholder = MeshMetadata(anchorID: UUID(), chunkIndex: 0, chunkCount: 0, timestampMs: header.timestampMs)
            guard let chunk = MeshChunk.decodeBinaryPayload(payload, metadata: placeholder) else {
                throw CodecError.malformedPayload("Mesh chunk payload")
            }
            return .meshData(chunk)
        case .heartbeat:
            return .heartbeat
        }
    }
}

// MARK: - Incremental decoder

public extension MessageDecoder {
    /// One decoded item produced from a byte stream.
    enum DecodedItem: Sendable {
        case message(Message, header: MessageHeader)
        case corrupt(String)
        case incompatibleVersion(found: UInt8, expected: UInt8)
    }
}

/// Incrementally decodes a byte stream. Handles partial frames, multiple frames per
/// chunk, garbage/corruption (with resync), and version mismatches without crashing.
public final class MessageDecoder {
    private var buffer = Data()
    private let maxPayloadSize: Int
    public private(set) var corruptCount = 0
    public private(set) var decodedCount = 0

    public init(maxPayloadSize: Int = ProtocolVersion.maxPayloadSize) {
        self.maxPayloadSize = maxPayloadSize
    }

    /// Appends raw bytes and returns any complete items decoded.
    public func append(_ data: Data) -> [DecodedItem] {
        buffer.append(data)
        var items: [DecodedItem] = []

        while true {
            guard buffer.count >= ProtocolVersion.headerSize else { break }

            // Resynchronize on the magic prefix.
            let magicOK = buffer.prefix(ProtocolVersion.magicBytes.count).elementsEqual(ProtocolVersion.magicBytes)
            guard magicOK else {
                buffer.removeFirst()
                corruptCount += 1
                items.append(.corrupt("Bad magic; resyncing"))
                continue
            }

            let version = buffer[buffer.startIndex + 4]
            let typeRaw = buffer[buffer.startIndex + 5]
            let flags = buffer[buffer.startIndex + 6]
            let sequence = Binary.readUInt32(buffer, at: 8) ?? 0
            let timestampMs = Binary.readUInt64(buffer, at: 12) ?? 0
            let payloadLengthRaw = Binary.readUInt32(buffer, at: 20) ?? 0
            let payloadLength = Int(payloadLengthRaw)

            guard payloadLength <= maxPayloadSize else {
                corruptCount += 1
                items.append(.corrupt("Payload length \(payloadLength) exceeds maximum \(maxPayloadSize)"))
                buffer.removeFirst(ProtocolVersion.headerSize)
                continue
            }

            let total = ProtocolVersion.headerSize + payloadLength + ProtocolVersion.trailerSize
            guard buffer.count >= total else { break } // wait for the rest of the frame

            let payload = buffer.subdata(in: buffer.startIndex + ProtocolVersion.headerSize ..< buffer.startIndex + ProtocolVersion.headerSize + payloadLength)
            let storedCRC = Binary.readUInt32(buffer, at: ProtocolVersion.headerSize + payloadLength) ?? 0
            if CRC32.checksum(payload) != storedCRC {
                corruptCount += 1
                items.append(.corrupt("CRC mismatch"))
                buffer.removeFirst(total)
                continue
            }
            buffer.removeFirst(total)

            if version != ProtocolVersion.current {
                items.append(.incompatibleVersion(found: version, expected: ProtocolVersion.current))
                continue
            }

            guard let type = MessageType(rawValue: typeRaw) else {
                corruptCount += 1
                items.append(.corrupt("Unknown message type \(typeRaw)"))
                continue
            }

            let header = MessageHeader(version: version, type: type, flags: flags, sequence: sequence, timestampMs: timestampMs, payloadLength: UInt32(payloadLength))
            do {
                let message = try MessageCodec.decodePayload(type: type, payload: payload, header: header)
                decodedCount += 1
                items.append(.message(message, header: header))
            } catch {
                corruptCount += 1
                items.append(.corrupt("Decode failed: \(error)"))
            }
        }
        return items
    }

    public func reset() {
        buffer.removeAll()
    }
}

// MARK: - JSON helpers

extension JSONEncoder {
    static let lidarLink: JSONEncoder = {
        let encoder = JSONEncoder()
        encoder.outputFormatting = [.sortedKeys]
        encoder.dateEncodingStrategy = .iso8601
        return encoder
    }()
}

extension JSONDecoder {
    static let lidarLink: JSONDecoder = {
        let decoder = JSONDecoder()
        decoder.dateDecodingStrategy = .iso8601
        return decoder
    }()
}
