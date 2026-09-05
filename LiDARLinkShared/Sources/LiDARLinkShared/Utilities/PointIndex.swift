import Foundation
import simd

/// Incremental spatial grid over accumulated points for neighborhood queries.
/// Stores only cell→index maps — the point data itself lives in the caller's
/// array (shared via copy-on-write), so memory isn't duplicated. Thread-safe.
public final class PointIndex: @unchecked Sendable {
    private let lock = NSLock()
    private var cells: [UInt64: [Int]] = [:]
    public let cellSize: Float
    private let inv: Float
    private var pointCount = 0

    public init(cellSize: Float = 0.05) {
        self.cellSize = max(cellSize, 0.001)
        self.inv = 1 / self.cellSize
    }

    public var count: Int {
        lock.lock()
        defer { lock.unlock() }
        return pointCount
    }

    /// Registers new points. Must be called with the same points, in the same
    /// order, as the caller appends to its own array so indices stay aligned.
    public func add(_ newPoints: [PointCloudPoint]) {
        lock.lock()
        defer { lock.unlock() }
        for point in newPoints {
            cells[ProgressiveMap.packCell(position: point.position, invCellSize: inv), default: []].append(pointCount)
            pointCount += 1
        }
    }

    /// Indices of points within `radius` of `points[index]`, excluding itself,
    /// sorted by distance. `points` is the caller's full array.
    public func neighbors(of index: Int, radius: Float, points: [PointCloudPoint]) -> [(index: Int, distance: Float)] {
        lock.lock()
        defer { lock.unlock() }
        guard index >= 0, index < points.count, index < pointCount else { return [] }
        let position = points[index].position
        let radiusSquared = radius * radius
        let searchCells = max(Int(ceil(radius * inv)), 1)
        let cx = Int(floor(position.x * inv))
        let cy = Int(floor(position.y * inv))
        let cz = Int(floor(position.z * inv))
        var result: [(Int, Float)] = []
        for dz in -searchCells...searchCells {
            for dy in -searchCells...searchCells {
                for dx in -searchCells...searchCells {
                    let key = ProgressiveMap.packCell(ix: Int32(cx + dx), iy: Int32(cy + dy), iz: Int32(cz + dz))
                    for other in cells[key] ?? [] where other != index {
                        guard other >= 0, other < points.count else { continue }
                        let delta = position - points[other].position
                        let distanceSquared = simd_dot(delta, delta)
                        if distanceSquared <= radiusSquared {
                            result.append((other, sqrt(distanceSquared)))
                        }
                    }
                }
            }
        }
        result.sort { $0.1 < $1.1 }
        return result
    }

    public func rebuild(_ newPoints: [PointCloudPoint]) {
        lock.lock()
        defer { lock.unlock() }
        cells.removeAll(keepingCapacity: true)
        pointCount = 0
        for (index, point) in newPoints.enumerated() {
            cells[ProgressiveMap.packCell(position: point.position, invCellSize: inv), default: []].append(index)
            pointCount += 1
        }
    }

    public func clear() {
        lock.lock()
        defer { lock.unlock() }
        cells.removeAll(keepingCapacity: true)
        pointCount = 0
    }
}
