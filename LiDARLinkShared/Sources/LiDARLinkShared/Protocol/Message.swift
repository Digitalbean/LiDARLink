import Foundation
import simd

/// Parsed frame header (also echoed to receivers for sequencing and timing).
public struct MessageHeader: Equatable, Sendable {
    public var version: UInt8
    public var type: MessageType
    public var flags: UInt8
    public var sequence: UInt32
    public var timestampMs: UInt64
    public var payloadLength: UInt32

    public init(version: UInt8, type: MessageType, flags: UInt8, sequence: UInt32, timestampMs: UInt64, payloadLength: UInt32) {
        self.version = version
        self.type = type
        self.flags = flags
        self.sequence = sequence
        self.timestampMs = timestampMs
        self.payloadLength = payloadLength
    }
}

/// A decoded protocol message.
public enum Message: Sendable, Equatable {
    case hello(HelloMessage)
    case helloAck(HelloAckMessage)
    case versionMismatch(VersionMismatchMessage)
    case control(ControlMessage)
    case frameMeta(FrameMetadata)
    case frameDepth(DepthPayload)
    case frameColor(ColorPayload)
    case meshMeta(MeshMetadata)
    case meshData(MeshChunk)
    case heartbeat
    case error(ErrorMessage)
    case scanMode(ScanModeMessage)
    case meshRemove(MeshRemoveMessage)
    case roomStructure(RoomStructure)
    case objectPhoto(ObjectPhotoPayload)
    case remoteCaptureSettings(RemoteCaptureSettings)

    public var type: MessageType {
        switch self {
        case .hello: return .hello
        case .helloAck: return .helloAck
        case .versionMismatch: return .versionMismatch
        case .control: return .control
        case .frameMeta: return .frameMeta
        case .frameDepth: return .frameDepth
        case .frameColor: return .frameColor
        case .meshMeta: return .meshMeta
        case .meshData: return .meshData
        case .heartbeat: return .heartbeat
        case .error: return .error
        case .scanMode: return .scanMode
        case .meshRemove: return .meshRemove
        case .roomStructure: return .roomStructure
        case .objectPhoto: return .objectPhoto
        case .remoteCaptureSettings: return .remoteCaptureSettings
        }
    }
}
