import Foundation

/// Reassembles frames from independently arriving metadata/depth/color messages.
/// A frame completes when metadata + depth are present and, when the metadata
/// advertises color (`colorJpegSize > 0`), the color payload has arrived too.
/// Handles out-of-order parts (keyed by sequence), duplicates, and stale entries.
/// Thread-safe.
public final class FrameAssembler: @unchecked Sendable {
    private struct Entry {
        var metadata: FrameMetadata?
        var depth: DepthPayload?
        var color: ColorPayload?
        var updatedAtMs: UInt64
    }

    private let lock = NSLock()
    private var entries: [UInt32: Entry] = [:]
    public private(set) var duplicateParts = 0
    public private(set) var completedFrames = 0

    public init() {}

    /// Applies frame metadata. Returns the completed frame if the frame is ready.
    @discardableResult
    public func apply(metadata: FrameMetadata, timestampMs: UInt64) -> ScanFrame? {
        lock.lock()
        defer { lock.unlock() }
        var entry = entries[metadata.sequence] ?? Entry(metadata: nil, depth: nil, color: nil, updatedAtMs: timestampMs)
        if entry.metadata != nil { duplicateParts += 1 }
        entry.metadata = metadata
        entry.updatedAtMs = timestampMs
        entries[metadata.sequence] = entry
        return completeIfReady(sequence: metadata.sequence)
    }

    /// Applies a depth payload. Returns the completed frame if the frame is ready.
    @discardableResult
    public func apply(depth: DepthPayload, for sequence: UInt32, timestampMs: UInt64) -> ScanFrame? {
        lock.lock()
        defer { lock.unlock() }
        var entry = entries[sequence] ?? Entry(metadata: nil, depth: nil, color: nil, updatedAtMs: timestampMs)
        if entry.depth != nil { duplicateParts += 1 }
        entry.depth = depth
        entry.updatedAtMs = timestampMs
        entries[sequence] = entry
        return completeIfReady(sequence: sequence)
    }

    /// Applies a color payload. Returns the completed frame if the frame is ready
    /// (color can be the completing part when the metadata advertises color).
    @discardableResult
    public func apply(color: ColorPayload, for sequence: UInt32, timestampMs: UInt64) -> ScanFrame? {
        lock.lock()
        defer { lock.unlock() }
        var entry = entries[sequence] ?? Entry(metadata: nil, depth: nil, color: nil, updatedAtMs: timestampMs)
        if entry.color != nil { duplicateParts += 1 }
        entry.color = color
        entry.updatedAtMs = timestampMs
        entries[sequence] = entry
        return completeIfReady(sequence: sequence)
    }

    /// Removes entries not touched since `timestampMs`; returns the number removed.
    @discardableResult
    public func prune(before timestampMs: UInt64) -> Int {
        lock.lock()
        defer { lock.unlock() }
        let stale = entries.filter { $0.value.updatedAtMs < timestampMs }
        stale.forEach { entries.removeValue(forKey: $0.key) }
        return stale.count
    }

    public var incompleteCount: Int {
        lock.lock()
        defer { lock.unlock() }
        return entries.count
    }

    public func reset() {
        lock.lock()
        defer { lock.unlock() }
        entries.removeAll()
        duplicateParts = 0
        completedFrames = 0
    }

    private func completeIfReady(sequence: UInt32) -> ScanFrame? {
        guard let entry = entries[sequence],
              let metadata = entry.metadata,
              let depth = entry.depth else { return nil }
        let expectsColor = metadata.colorJpegSize > 0
        if expectsColor, entry.color == nil { return nil }

        let frame = ScanFrame(sequence: sequence,
                              captureTimestampMs: metadata.captureTimestampMs,
                              pose: metadata.pose,
                              intrinsics: metadata.intrinsics,
                              depth: depth,
                              color: entry.color,
                              depthIsSmoothed: metadata.depthIsSmoothed,
                              depthScale: metadata.depthScale)
        entries.removeValue(forKey: sequence)
        completedFrames += 1
        return frame
    }
}
