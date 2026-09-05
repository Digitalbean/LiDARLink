import Foundation
import simd

// MARK: - Capabilities

public struct Capabilities: Codable, Equatable, Sendable {
    public var supportsDepth: Bool
    public var supportsMesh: Bool
    public var supportsColor: Bool

    public init(supportsDepth: Bool, supportsMesh: Bool, supportsColor: Bool) {
        self.supportsDepth = supportsDepth
        self.supportsMesh = supportsMesh
        self.supportsColor = supportsColor
    }
}

// MARK: - Handshake

public struct HelloMessage: Codable, Equatable, Sendable {
    public var protocolVersion: UInt8
    public var appName: String
    public var appVersion: String
    public var deviceName: String
    public var capabilities: Capabilities

    public init(protocolVersion: UInt8, appName: String, appVersion: String, deviceName: String, capabilities: Capabilities) {
        self.protocolVersion = protocolVersion
        self.appName = appName
        self.appVersion = appVersion
        self.deviceName = deviceName
        self.capabilities = capabilities
    }
}

public struct HelloAckMessage: Codable, Equatable, Sendable {
    public var accepted: Bool
    public var protocolVersion: UInt8
    public var serverName: String
    public var reason: String?

    public init(accepted: Bool, protocolVersion: UInt8, serverName: String, reason: String?) {
        self.accepted = accepted
        self.protocolVersion = protocolVersion
        self.serverName = serverName
        self.reason = reason
    }
}

public struct VersionMismatchMessage: Codable, Equatable, Sendable {
    public var clientVersion: UInt8
    public var serverVersion: UInt8

    public init(clientVersion: UInt8, serverVersion: UInt8) {
        self.clientVersion = clientVersion
        self.serverVersion = serverVersion
    }
}

// MARK: - Control

public enum ControlAction: String, Codable, Sendable {
    /// Begin a fresh scan from idle (distinct from `resume`, which only makes
    /// sense from `paused` — sending the wrong one to a phone in the wrong
    /// state is a silent no-op on the receiving end).
    case start
    case pause
    case resume
    case stop
    case requestStats
}

public struct ControlMessage: Codable, Equatable, Sendable {
    public var action: ControlAction

    public init(action: ControlAction) {
        self.action = action
    }
}

// MARK: - Scan mode

/// Scanning intent shared by both apps. Decodes leniently (unknown strings and
/// older payloads fall back to `.room`) so a version skew can't break a scan.
public enum ScanMode: String, Codable, Sendable, CaseIterable {
    case room
    case object

    public init(from decoder: Decoder) throws {
        let raw = (try? decoder.singleValueContainer().decode(String.self)) ?? Self.room.rawValue
        self = ScanMode(rawValue: raw) ?? .room
    }
}

public struct ScanModeMessage: Codable, Equatable, Sendable {
    public var mode: ScanMode
    public var voxelCm: Float

    public init(mode: ScanMode, voxelCm: Float) {
        self.mode = mode
        self.voxelCm = voxelCm
    }
}

// MARK: - Remote capture settings

/// Mac → phone: drive the phone's capture settings without touching the
/// device. `captureMode` is the phone's own `CaptureMode.rawValue` (that type
/// stays phone-side — it's a capture-time UI concept, not a wire concept —
/// this just carries the name across; an unrecognised value is ignored rather
/// than crashing, so a version skew degrades gracefully).
public struct RemoteCaptureSettings: Codable, Equatable, Sendable {
    public var captureMode: String
    public var roomPlanEnabled: Bool
    /// Same blanket override as the phone's own "Max depth" settings picker —
    /// applied on top of whatever the capture mode itself set, not a
    /// mode-specific value.
    public var maxDepthMeters: Float

    public init(captureMode: String, roomPlanEnabled: Bool, maxDepthMeters: Float) {
        self.captureMode = captureMode
        self.roomPlanEnabled = roomPlanEnabled
        self.maxDepthMeters = maxDepthMeters
    }
}

// MARK: - Mesh lifecycle

/// Anchor IDs that ARKit removed (merged or discarded) since the last frame, so
/// the receiver can drop the matching mesh geometry instead of leaving ghosts.
public struct MeshRemoveMessage: Codable, Equatable, Sendable {
    public var anchorIDs: [UUID]

    public init(anchorIDs: [UUID]) {
        self.anchorIDs = anchorIDs
    }
}

// MARK: - Errors

public enum MessageErrorCode: String, Codable, Sendable {
    case connectionDenied
    case incompatibleVersion
    case malformedData
    case internalError
}

public struct ErrorMessage: Codable, Equatable, Sendable {
    public var code: MessageErrorCode
    public var message: String

    public init(code: MessageErrorCode, message: String) {
        self.code = code
        self.message = message
    }
}

// MARK: - Frame metadata

/// Metadata that describes a captured frame. Depth/color payloads travel in
/// separate binary messages keyed by `sequence`.
public struct FrameMetadata: Codable, Equatable, Sendable {
    public var sequence: UInt32
    public var captureTimestampMs: UInt64
    public var depthWidth: Int
    public var depthHeight: Int
    public var depthScale: Float
    public var hasConfidence: Bool
    public var depthIsSmoothed: Bool
    public var intrinsics: CameraIntrinsics
    public var pose: simd_float4x4
    public var colorWidth: Int
    public var colorHeight: Int
    public var colorJpegSize: Int
    public var downsampleFactor: Int

    public init(sequence: UInt32,
                captureTimestampMs: UInt64,
                depthWidth: Int,
                depthHeight: Int,
                depthScale: Float,
                hasConfidence: Bool,
                depthIsSmoothed: Bool,
                intrinsics: CameraIntrinsics,
                pose: simd_float4x4,
                colorWidth: Int,
                colorHeight: Int,
                colorJpegSize: Int,
                downsampleFactor: Int) {
        self.sequence = sequence
        self.captureTimestampMs = captureTimestampMs
        self.depthWidth = depthWidth
        self.depthHeight = depthHeight
        self.depthScale = depthScale
        self.hasConfidence = hasConfidence
        self.depthIsSmoothed = depthIsSmoothed
        self.intrinsics = intrinsics
        self.pose = pose
        self.colorWidth = colorWidth
        self.colorHeight = colorHeight
        self.colorJpegSize = colorJpegSize
        self.downsampleFactor = downsampleFactor
    }
}
