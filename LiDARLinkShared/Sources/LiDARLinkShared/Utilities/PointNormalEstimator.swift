import Foundation
import simd

/// Per-point normal estimation via PCA over local neighborhoods (for lit shading).
public enum PointNormalEstimator {
    /// Estimates the surface normal at a point from its neighborhood in `index`.
    /// The normal is the eigenvector of the smallest eigenvalue of the 3x3
    /// covariance of neighbor offsets (smallest-variance direction).
    public static func estimateNormal(at index: Int,
                                      in pointIndex: PointIndex,
                                      points: [PointCloudPoint],
                                      radius: Float,
                                      minNeighbors: Int = 8,
                                      orientToward viewpoint: SIMD3<Float>? = nil) -> SIMD3<Float>? {
        let neighbors = pointIndex.neighbors(of: index, radius: radius, points: points)
        guard neighbors.count >= minNeighbors else { return nil }
        guard index >= 0, index < points.count else { return nil }
        let position = points[index].position

        var centroid = SIMD3<Float>(0, 0, 0)
        for (neighborIndex, _) in neighbors {
            guard neighborIndex >= 0, neighborIndex < points.count else { continue }
            centroid += points[neighborIndex].position
        }
        centroid /= Float(neighbors.count)

        var cxx: Float = 0, cxy: Float = 0, cxz: Float = 0
        var cyy: Float = 0, cyz: Float = 0, czz: Float = 0
        for (neighborIndex, _) in neighbors {
            guard neighborIndex >= 0, neighborIndex < points.count else { continue }
            let d = points[neighborIndex].position - centroid
            cxx += d.x * d.x
            cxy += d.x * d.y
            cxz += d.x * d.z
            cyy += d.y * d.y
            cyz += d.y * d.z
            czz += d.z * d.z
        }
        let covariance = simd_float3x3(columns: (SIMD3<Float>(cxx, cxy, cxz),
                                                 SIMD3<Float>(cxy, cyy, cyz),
                                                 SIMD3<Float>(cxz, cyz, czz)))
        guard let smallest = Self.smallestEigenvector(of: covariance) else { return nil }
        var normal = simd_normalize(smallest)
        // PCA gives an unsigned axis. Flip it to face the viewpoint (the camera
        // shading the cloud) so lit shading is consistent instead of random.
        if let viewpoint, simd_dot(normal, viewpoint - position) < 0 {
            normal = -normal
        }
        return normal
    }

    /// Normals for a list of point indices (nil entries where estimation failed).
    public static func normals(indices: [Int],
                               in pointIndex: PointIndex,
                               points: [PointCloudPoint],
                               radius: Float,
                               minNeighbors: Int = 8,
                               orientToward viewpoint: SIMD3<Float>? = nil) -> [SIMD3<Float>?] {
        indices.map { estimateNormal(at: $0, in: pointIndex, points: points, radius: radius, minNeighbors: minNeighbors, orientToward: viewpoint) }
    }

    /// Jacobi eigensolver for symmetric 3x3 matrices; returns the eigenvector for
    /// the smallest eigenvalue.
    static func smallestEigenvector(of matrix: simd_float3x3) -> SIMD3<Float>? {
        var a = matrix
        var eigenvectors = matrix_identity_float3x3
        for _ in 0..<12 {
            let offDiag = abs(a[0][1]) + abs(a[0][2]) + abs(a[1][2])
            if offDiag < 1e-8 { break }
            // Find largest off-diagonal.
            var p = 0, q = 1
            var largest = abs(a[0][1])
            if abs(a[0][2]) > largest { p = 0; q = 2; largest = abs(a[0][2]) }
            if abs(a[1][2]) > largest { p = 1; q = 2 }
            let app = a[p][p], aqq = a[q][q], apq = a[p][q]
            let angle = 0.5 * atan2(2 * apq, aqq - app)
            let c = cos(angle)
            let s = sin(angle)
            // Rotation J(p,q,theta): a' = J^T a J
            var rotated = a
            for i in 0..<3 {
                let aip = a[i][p]
                let aiq = a[i][q]
                rotated[i][p] = c * aip - s * aiq
                rotated[i][q] = s * aip + c * aiq
            }
            for i in 0..<3 {
                let api = rotated[p][i]
                let aqi = rotated[q][i]
                rotated[p][i] = c * api - s * aqi
                rotated[q][i] = s * api + c * aqi
            }
            rotated[p][q] = 0
            rotated[q][p] = 0
            a = rotated
            // Accumulate eigenvectors.
            for i in 0..<3 {
                let eip = eigenvectors[i][p]
                let eiq = eigenvectors[i][q]
                eigenvectors[i][p] = c * eip - s * eiq
                eigenvectors[i][q] = s * eip + c * eiq
            }
        }
        // Smallest eigenvalue → smallest diagonal entry.
        var smallestIndex = 0
        var smallestValue = a[0][0]
        for i in 1..<3 where a[i][i] < smallestValue {
            smallestValue = a[i][i]
            smallestIndex = i
        }
        return SIMD3<Float>(eigenvectors[0][smallestIndex],
                            eigenvectors[1][smallestIndex],
                            eigenvectors[2][smallestIndex])
    }
}
