import Foundation

/// Describes a saved recording (one directory with manifest.json + frames.bin + meshes.bin).
public struct RecordingManifest: Codable, Equatable, Sendable {
    public var formatVersion: Int
    public var appName: String
    public var appVersion: String
    public var deviceName: String
    public var startedAt: Date
    public var endedAt: Date?
    public var frameCount: Int
    public var meshCount: Int
    public var depthWidth: Int
    public var depthHeight: Int
    public var colorWidth: Int
    public var colorHeight: Int
    public var targetFPS: Int
    public var protocolVersion: UInt8
    public var notes: String?

    public init(formatVersion: Int,
                appName: String,
                appVersion: String,
                deviceName: String,
                startedAt: Date,
                endedAt: Date?,
                frameCount: Int,
                meshCount: Int,
                depthWidth: Int,
                depthHeight: Int,
                colorWidth: Int,
                colorHeight: Int,
                targetFPS: Int,
                protocolVersion: UInt8,
                notes: String?) {
        self.formatVersion = formatVersion
        self.appName = appName
        self.appVersion = appVersion
        self.deviceName = deviceName
        self.startedAt = startedAt
        self.endedAt = endedAt
        self.frameCount = frameCount
        self.meshCount = meshCount
        self.depthWidth = depthWidth
        self.depthHeight = depthHeight
        self.colorWidth = colorWidth
        self.colorHeight = colorHeight
        self.targetFPS = targetFPS
        self.protocolVersion = protocolVersion
        self.notes = notes
    }
}
