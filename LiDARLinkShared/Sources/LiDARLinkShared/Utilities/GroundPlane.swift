import Foundation
import simd

/// Fits the dominant floor plane of a scan and produces a transform that levels
/// it — rotates the floor flat and drops it to y = 0 — so exports come out
/// upright without hand-reorientation.
public enum GroundPlane {
    /// Estimates the floor plane from the lowest band of points (scans are
    /// gravity-aligned, so the floor is already roughly horizontal). Returns the
    /// upward plane normal and a point on the plane, or nil when there is no
    /// convincing near-horizontal floor.
    public static func fit(_ points: [PointCloudPoint]) -> (normal: SIMD3<Float>, point: SIMD3<Float>)? {
        guard points.count >= 64 else { return nil }
        let positions = points.map(\.position).sorted { $0.y < $1.y }
        let bandCount = max(32, positions.count * 12 / 100)
        let band = Array(positions.prefix(bandCount))
        // A floor band should be thin; a staircase or a low wall is not a floor.
        guard band.last!.y - band.first!.y < 0.3 else { return nil }

        var centroid = SIMD3<Float>(0, 0, 0)
        for p in band { centroid += p }
        centroid /= Float(band.count)

        var cxx: Float = 0, cxy: Float = 0, cxz: Float = 0
        var cyy: Float = 0, cyz: Float = 0, czz: Float = 0
        for p in band {
            let d = p - centroid
            cxx += d.x * d.x; cxy += d.x * d.y; cxz += d.x * d.z
            cyy += d.y * d.y; cyz += d.y * d.z; czz += d.z * d.z
        }
        let covariance = simd_float3x3(columns: (SIMD3<Float>(cxx, cxy, cxz),
                                                 SIMD3<Float>(cxy, cyy, cyz),
                                                 SIMD3<Float>(cxz, cyz, czz)))
        guard let axis = PointNormalEstimator.smallestEigenvector(of: covariance) else { return nil }
        var normal = simd_normalize(axis)
        if normal.y < 0 { normal = -normal }
        guard normal.y > 0.85 else { return nil } // must be within ~32° of vertical
        return (normal, centroid)
    }

    /// A transform that rotates `normal` onto +Y and translates so the plane
    /// through `point` lands on y = 0.
    public static func levelingTransform(normal: SIMD3<Float>, point: SIMD3<Float>) -> simd_float4x4 {
        let up = SIMD3<Float>(0, 1, 0)
        let n = simd_normalize(normal)
        let rotation: simd_float3x3
        let cosAngle = simd_clamp(simd_dot(n, up), -1, 1)
        if cosAngle > 0.99999 {
            rotation = matrix_identity_float3x3
        } else {
            let axis = simd_normalize(simd_cross(n, up))
            let angle = acos(cosAngle)
            rotation = simd_float3x3(simd_quatf(angle: angle, axis: axis))
        }
        let rotatedPoint = rotation * point
        return simd_float4x4(SIMD4<Float>(rotation.columns.0, 0),
                             SIMD4<Float>(rotation.columns.1, 0),
                             SIMD4<Float>(rotation.columns.2, 0),
                             SIMD4<Float>(0, -rotatedPoint.y, 0, 1))
    }

    /// Convenience: level transform for a cloud, or nil when no floor is found.
    public static func levelingTransform(for points: [PointCloudPoint]) -> simd_float4x4? {
        guard let plane = fit(points) else { return nil }
        return levelingTransform(normal: plane.normal, point: plane.point)
    }

    /// Applies a transform to every point (position only; colours/confidence kept).
    public static func apply(_ transform: simd_float4x4, to points: [PointCloudPoint]) -> [PointCloudPoint] {
        points.map { point in
            let w = transform * SIMD4<Float>(point.position, 1)
            var moved = point
            moved.position = SIMD3<Float>(w.x, w.y, w.z)
            return moved
        }
    }
}
