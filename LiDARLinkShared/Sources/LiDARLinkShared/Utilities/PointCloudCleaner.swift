import Foundation
import simd

/// Point-cloud cleanup, applied periodically on the Mac to remove floating noise
/// specks without eroding legitimate geometry.
public enum PointCloudCleaner {
    /// Radius-based outlier removal (PCL RadiusOutlierRemoval): keeps a point only
    /// if at least `minNeighbors` other points are within `radius`. When `radius`
    /// is nil it is auto-scaled to the cloud's estimated point spacing, so it
    /// adapts to Fine/Standard/Coarse voxel settings. Sparse thin geometry with
    /// few neighbors survives because it keeps its own sparse neighborhood.
    public static func radiusOutlierRemoval(_ points: [PointCloudPoint],
                                            radius: Float? = nil,
                                            minNeighbors: Int = 8) -> [PointCloudPoint] {
        guard points.count > minNeighbors + 2 else { return points }

        // Estimate mean spacing from the bounding box to size the radius.
        var minPoint = points[0].position
        var maxPoint = points[0].position
        for point in points {
            minPoint = simd_min(minPoint, point.position)
            maxPoint = simd_max(maxPoint, point.position)
        }
        let diagonal = max(simd_length(maxPoint - minPoint), 0.01)
        let estimatedSpacing = pow(diagonal / Float(points.count), 1.0 / 3.0)
        let effectiveRadius = radius ?? max(estimatedSpacing * 4, 0.03)

        // Grid cells of radius/2 so every point within `radius` is inside the
        // 3x3x3 cell neighborhood of the query point.
        let cellSize = effectiveRadius / 2
        let inv = 1 / cellSize
        let radiusSquared = effectiveRadius * effectiveRadius

        var cells: [UInt64: [Int]] = [:]
        for (index, point) in points.enumerated() {
            cells[ProgressiveMap.packCell(position: point.position, invCellSize: inv), default: []].append(index)
        }

        var keep = [Bool](repeating: true, count: points.count)
        for index in points.indices {
            let position = points[index].position
            let cx = Int(floor(position.x * inv))
            let cy = Int(floor(position.y * inv))
            let cz = Int(floor(position.z * inv))
            var count = 0
            outer: for dz in -1...1 {
                for dy in -1...1 {
                    for dx in -1...1 {
                        let key = ProgressiveMap.packCell(ix: Int32(cx + dx), iy: Int32(cy + dy), iz: Int32(cz + dz))
                        for other in cells[key] ?? [] where other != index {
                            let delta = position - points[other].position
                            if simd_dot(delta, delta) <= radiusSquared {
                                count += 1
                                if count >= minNeighbors { break outer }
                            }
                        }
                    }
                }
            }
            keep[index] = count >= minNeighbors
        }

        return points.enumerated().compactMap { keep[$0.offset] ? $0.element : nil }
    }
}
