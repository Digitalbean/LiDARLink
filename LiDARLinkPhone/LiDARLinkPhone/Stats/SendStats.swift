import Foundation

/// Phone-side transmission diagnostics shown in the overlay.
struct SendStats: Equatable {
    var sentFPS: Double = 0
    var droppedFrames: Int = 0
    var bytesSent: Int = 0
    var throughputBytesPerSecond: Double = 0
    var tier = 0
    var depthResolution = "—"
    var colorOn = true
    var colorProducedCount = 0
    var rejectedFrames = 0
    var meshOn = false
}
