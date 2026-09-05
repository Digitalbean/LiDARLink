import Foundation

/// The wire message types of the LiDAR Link protocol.
public enum MessageType: UInt8, Codable, CaseIterable, Sendable {
    case hello = 1
    case helloAck = 2
    case versionMismatch = 3
    case control = 4
    case frameMeta = 5
    case frameDepth = 6
    case frameColor = 7
    case meshMeta = 8
    case meshData = 9
    case heartbeat = 10
    case error = 11
    case scanMode = 12
    case meshRemove = 13
    case roomStructure = 14
    case objectPhoto = 15
    case remoteCaptureSettings = 16
}
