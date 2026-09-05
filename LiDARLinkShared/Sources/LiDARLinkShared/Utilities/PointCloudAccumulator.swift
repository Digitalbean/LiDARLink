import Foundation

/// Accumulates a growing point cloud with a bounded memory footprint.
/// When the cap is reached, new points are skipped (counted) instead of dropping
/// old ones, so previously scanned geometry stays visible (object permanence).
/// Thread-safe.
public final class PointCloudAccumulator: @unchecked Sendable {
    private let lock = NSLock()
    private var points: [PointCloudPoint] = []
    private var minY: Float?
    private var maxY: Float?
    public let maxPoints: Int
    public private(set) var addedPointCount = 0
    public private(set) var skippedPointCount = 0

    public init(maxPoints: Int = 3_000_000) {
        self.maxPoints = max(maxPoints, 1)
        points.reserveCapacity(self.maxPoints)
    }

    public func add(_ newPoints: [PointCloudPoint]) {
        lock.lock()
        defer { lock.unlock() }
        guard !newPoints.isEmpty else { return }
        guard points.count < maxPoints else {
            skippedPointCount += newPoints.count
            return
        }
        let room = maxPoints - points.count
        let accepted: ArraySlice<PointCloudPoint>
        if newPoints.count > room {
            skippedPointCount += newPoints.count - room
            accepted = newPoints.prefix(room)
        } else {
            accepted = newPoints[...]
        }
        points.append(contentsOf: accepted)
        for point in accepted { absorb(point) }
        addedPointCount += newPoints.count
    }

    private func absorb(_ point: PointCloudPoint) {
        if let m = minY { minY = min(m, point.position.y) } else { minY = point.position.y }
        if let m = maxY { maxY = max(m, point.position.y) } else { maxY = point.position.y }
    }

    public var count: Int {
        lock.lock()
        defer { lock.unlock() }
        return points.count
    }

    /// Incremental world Y range (for height-gradient coloring, no full scans).
    public var heightRange: (minY: Float, maxY: Float)? {
        lock.lock()
        defer { lock.unlock() }
        guard let minY, let maxY else { return nil }
        return (minY, maxY)
    }

    public var isFull: Bool {
        lock.lock()
        defer { lock.unlock() }
        return points.count >= maxPoints
    }

    public func snapshot() -> [PointCloudPoint] {
        lock.lock()
        defer { lock.unlock() }
        return points
    }

    /// Replaces the stored points entirely (used by periodic map cleanup).
    public func replace(with newPoints: [PointCloudPoint]) {
        lock.lock()
        defer { lock.unlock() }
        points = newPoints
        addedPointCount = newPoints.count
        skippedPointCount = 0
        minY = nil
        maxY = nil
        for point in newPoints { absorb(point) }
    }

    public func clear() {
        lock.lock()
        defer { lock.unlock() }
        points.removeAll()
        addedPointCount = 0
        skippedPointCount = 0
        minY = nil
        maxY = nil
    }
}

public enum PointCloudSampler {
    /// Returns samples spread across the full input instead of truncating one end.
    public static func evenlySample(_ points: [PointCloudPoint], limit: Int) -> [PointCloudPoint] {
        guard limit > 0, points.count > limit else { return limit > 0 ? points : [] }
        let step = Double(points.count) / Double(limit)
        var result: [PointCloudPoint] = []
        result.reserveCapacity(limit)
        for sample in 0..<limit {
            result.append(points[min(Int(Double(sample) * step), points.count - 1)])
        }
        return result
    }
}
