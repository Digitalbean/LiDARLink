import Foundation
import CoreGraphics
import simd

/// Camera-navigation math (unit-tested, shared by the Mac renderer).
public enum ViewNavigationMath {
    /// Builds a ray through a viewport point, origin at the camera.
    /// `viewportPoint` is in pixels with origin top-left; `viewportSize` in pixels.
    public static func viewRay(cameraPosition: SIMD3<Float>,
                               cameraForward: SIMD3<Float>,
                               cameraUp: SIMD3<Float>,
                               viewportPoint: CGPoint,
                               viewportSize: CGSize,
                               fieldOfViewY: Float) -> (origin: SIMD3<Float>, direction: SIMD3<Float>) {
        let fovY = fieldOfViewY * .pi / 180
        let aspect = Float(viewportSize.width / max(viewportSize.height, 1))
        let tanFov = tan(fovY / 2)
        let ndcX = (Float(viewportPoint.x) / Float(max(viewportSize.width, 1))) * 2 - 1
        let ndcY = 1 - (Float(viewportPoint.y) / Float(max(viewportSize.height, 1))) * 2
        let right = simd_normalize(simd_cross(cameraForward, cameraUp))
        let trueUp = simd_cross(right, cameraForward)
        let direction = simd_normalize(cameraForward
            + right * (ndcX * tanFov * aspect)
            + trueUp * (ndcY * tanFov))
        return (cameraPosition, direction)
    }

    /// The nearest point to a ray within `maxDistance` (perpendicular distance);
    /// nil when the cloud is empty or nothing is close enough.
    public static func nearestPoint(to origin: SIMD3<Float>,
                                    direction: SIMD3<Float>,
                                    in points: [PointCloudPoint],
                                    maxDistance: Float) -> PointCloudPoint? {
        let dir = simd_normalize(direction)
        var best: PointCloudPoint?
        var bestDistance = maxDistance
        for point in points {
            let toPoint = point.position - origin
            let t = simd_dot(toPoint, dir)
            guard t > 0 else { continue }
            let projected = origin + dir * t
            let distance = simd_length(point.position - projected)
            if distance < bestDistance {
                bestDistance = distance
                best = point
            }
        }
        return best
    }

    /// Spherical camera position orbiting `target` (yaw around Y, pitch elevation).
    public static func orbitCameraPosition(target: SIMD3<Float>,
                                           yaw: Float,
                                           pitch: Float,
                                           distance: Float) -> SIMD3<Float> {
        let offset = SIMD3<Float>(distance * cos(pitch) * sin(yaw),
                                  distance * sin(pitch),
                                  distance * cos(pitch) * cos(yaw))
        return target + offset
    }

    /// Derives (yaw, pitch, distance) from a camera position relative to a target,
    /// so switching pivots does not make the camera jump.
    public static func orbitComponents(cameraPosition: SIMD3<Float>, target: SIMD3<Float>) -> (yaw: Float, pitch: Float, distance: Float) {
        let toCamera = cameraPosition - target
        let distance = simd_length(toCamera)
        let normalized = distance > 0.0001 ? toCamera / distance : SIMD3<Float>(0, 0, 1)
        let pitch = asin(min(max(normalized.y, -1), 1))
        let yaw = atan2(normalized.x, normalized.z)
        return (yaw, pitch, distance)
    }

    // MARK: Walk (first-person) navigation

    /// Max look-up/down angle for walk mode (just short of vertical).
    public static let walkPitchLimit: Float = 1.5

    /// The direction the walk camera faces. `yaw` 0 looks toward +Z; positive
    /// `yaw` turns toward +X; positive `pitch` looks up.
    public static func walkLookDirection(yaw: Float, pitch: Float) -> SIMD3<Float> {
        SIMD3<Float>(cos(pitch) * sin(yaw), sin(pitch), cos(pitch) * cos(yaw))
    }

    /// Advances a walk-mode camera position by a movement input.
    /// `forward`/`strafe`/`lift` are in [-1, 1]; movement is along the yaw-only
    /// horizontal basis so looking up or down never changes walking speed.
    /// When `floorY` is set the result is pinned to `floorY + eyeHeight`
    /// (gravity/eye-height lock); otherwise `lift` moves vertically.
    public static func walkStep(position: SIMD3<Float>,
                                yaw: Float,
                                forward: Float,
                                strafe: Float,
                                lift: Float,
                                distance: Float,
                                floorY: Float?,
                                eyeHeight: Float) -> SIMD3<Float> {
        let forwardAxis = SIMD3<Float>(sin(yaw), 0, cos(yaw))
        // Screen-right for a camera looking along `forwardAxis` with +Y up.
        let rightAxis = SIMD3<Float>(-cos(yaw), 0, sin(yaw))
        var next = position + (forwardAxis * forward + rightAxis * strafe) * distance
        if let floorY {
            next.y = floorY + eyeHeight
        } else {
            next.y += lift * distance
        }
        return next
    }

    /// A standpoint for the walk camera: near one end of the cloud, at eye
    /// height, looking down its longest horizontal axis toward the fuller half
    /// (so you start looking *into* the scan, not out of it).
    public static func walkStandpoint(for points: [PointCloudPoint],
                                      eyeHeight: Float,
                                      floorY: Float?) -> (position: SIMD3<Float>, yaw: Float) {
        guard let first = points.first else {
            return (SIMD3<Float>(0, (floorY ?? 0) + eyeHeight, 2.5), .pi)
        }
        var minP = first.position
        var maxP = minP
        for point in points {
            minP = simd_min(minP, point.position)
            maxP = simd_max(maxP, point.position)
        }
        let centre = (minP + maxP) * 0.5
        let ground = floorY ?? minP.y

        let extentX = maxP.x - minP.x
        let extentZ = maxP.z - minP.z
        let alongZ = extentZ >= extentX

        // Face the half (split at the centre of the long axis) that holds more
        // points, so you look into the bulk of the scan rather than out of it.
        var negativeCount = 0
        var positiveCount = 0
        for point in points {
            let value = alongZ ? point.position.z : point.position.x
            let mid = alongZ ? centre.z : centre.x
            if value < mid { negativeCount += 1 } else { positiveCount += 1 }
        }
        let faceNegative = negativeCount > positiveCount

        let yaw: Float
        let backAxis: SIMD3<Float>
        if alongZ {
            yaw = faceNegative ? .pi : 0
            backAxis = SIMD3<Float>(0, 0, faceNegative ? 1 : -1)
        } else {
            yaw = faceNegative ? -.pi / 2 : .pi / 2
            backAxis = SIMD3<Float>(faceNegative ? 1 : -1, 0, 0)
        }
        // Stand ~35% of the long extent back from centre toward the near end.
        let backOff = max(extentX, extentZ) * 0.35
        let position = SIMD3<Float>(centre.x, ground + eyeHeight, centre.z) + backAxis * backOff
        return (SIMD3<Float>(position.x, ground + eyeHeight, position.z), yaw)
    }

    /// Robust floor height for a cloud: approximately a low percentile of world
    /// Y, so a few stray low points don't drop the floor through the basement.
    /// Single pass + a fixed histogram — safe to call on large clouds.
    public static func estimatedFloorY(of points: [PointCloudPoint], percentile: Float = 0.02) -> Float? {
        guard !points.isEmpty else { return nil }
        var minY = points[0].position.y
        var maxY = minY
        for point in points {
            minY = min(minY, point.position.y)
            maxY = max(maxY, point.position.y)
        }
        let span = maxY - minY
        guard span > 0.001 else { return minY }

        let binCount = 256
        var bins = [Int](repeating: 0, count: binCount)
        let scale = Float(binCount - 1) / span
        for point in points {
            let bin = Int((point.position.y - minY) * scale)
            bins[min(max(bin, 0), binCount - 1)] += 1
        }
        // Walk up from the bottom until enough of the cloud has accumulated to
        // clear isolated low outliers.
        let threshold = max(1, Int(Float(points.count) * percentile))
        var running = 0
        for (index, count) in bins.enumerated() {
            running += count
            if running >= threshold {
                return minY + (Float(index) + 0.5) / scale
            }
        }
        return minY
    }
}
