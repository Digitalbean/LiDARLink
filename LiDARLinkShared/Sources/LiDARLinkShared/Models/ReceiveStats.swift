import Foundation

/// Receiver-side diagnostics published to the Mac UI.
public struct ReceiveStats: Equatable, Sendable {
    public var receivedFPS: Double = 0
    public var frameCount: Int = 0
    public var colorFrameCount: Int = 0
    public var droppedFrames: Int = 0
    public var duplicateFrames: Int = 0
    public var reorderedFrames: Int = 0
    public var corruptFrames: Int = 0
    public var incompleteFrames: Int = 0
    public var bytesReceived: Int = 0
    public var throughputBytesPerSecond: Double = 0
    public var lastSequence: UInt32 = 0
    public var meshChunksReceived: Int = 0
    public var meshCount: Int = 0

    public init() {}
}
