import Foundation
import simd

/// How points in the cloud are colored.
public enum PointColorMode: Sendable, Equatable, Hashable {
    case cameraColor
    case heightGradient
    /// ARKit depth confidence: low = red, medium = yellow, high = green.
    case confidence
    case solid(SIMD3<UInt8>)
    /// Camera color shaded by estimated surface normals. The renderer bakes the
    /// lit color at chunk-build time (normals are not part of the wire format).
    case lit
}

/// Pure, unit-tested color mapping for point clouds.
public enum PointColorizer {
    /// Cool→warm ramp over t in [0, 1]: blue → cyan → green → yellow → red.
    public static func heightRamp(_ t: Float) -> SIMD3<UInt8> {
        let clamped = min(max(t, 0), 1)
        let stops: [(Float, SIMD3<UInt8>)] = [
            (0.00, SIMD3<UInt8>(30, 60, 220)),
            (0.25, SIMD3<UInt8>(60, 200, 220)),
            (0.50, SIMD3<UInt8>(90, 220, 90)),
            (0.75, SIMD3<UInt8>(240, 200, 60)),
            (1.00, SIMD3<UInt8>(230, 60, 50))
        ]
        for i in 0..<(stops.count - 1) {
            let (t0, c0) = stops[i]
            let (t1, c1) = stops[i + 1]
            if clamped <= t1 {
                let s = (clamped - t0) / max(t1 - t0, 0.0001)
                return lerp(c0, c1, s)
            }
        }
        return stops[stops.count - 1].1
    }

    public static func lerp(_ a: SIMD3<UInt8>, _ b: SIMD3<UInt8>, _ t: Float) -> SIMD3<UInt8> {
        SIMD3<UInt8>(UInt8(Float(a.x) + (Float(b.x) - Float(a.x)) * t),
                     UInt8(Float(a.y) + (Float(b.y) - Float(a.y)) * t),
                     UInt8(Float(a.z) + (Float(b.z) - Float(a.z)) * t))
    }

    /// Bakes simple directional lighting into a color (unlit ambient + diffuse).
    public static func litColor(color: SIMD3<UInt8>,
                                normal: SIMD3<Float>,
                                lightDirection: SIMD3<Float> = SIMD3<Float>(0.35, 1, 0.25)) -> SIMD3<UInt8> {
        let light = simd_normalize(lightDirection)
        let intensity = max(simd_dot(simd_normalize(normal), light), 0)
        let factor: Float = 0.35 + 0.65 * intensity
        return SIMD3<UInt8>(UInt8(min(Float(color.x) * factor, 255)),
                            UInt8(min(Float(color.y) * factor, 255)),
                            UInt8(min(Float(color.z) * factor, 255)))
    }

    /// Height bounds of a cloud (for height-gradient coloring).
    public static func heightRange(of points: [PointCloudPoint]) -> (minY: Float, maxY: Float)? {
        guard let first = points.first else { return nil }
        var minY = first.position.y
        var maxY = first.position.y
        for point in points {
            minY = min(minY, point.position.y)
            maxY = max(maxY, point.position.y)
        }
        return (minY, maxY)
    }

    /// Brightness multiplier for ARKit depth confidence — low-confidence points
    /// dim toward the background so noise recedes visually without needing to
    /// switch to the diagnostic `.confidence` color mode.
    public static func confidenceDimFactor(_ confidence: UInt8) -> Float {
        switch confidence {
        case 0: return 0.4
        case 1: return 0.7
        default: return 1.0
        }
    }

    /// A per-point color transform for a mode, used by the chunked renderer.
    /// `heightRange` is the global cloud range so height coloring stays consistent
    /// across incrementally added chunks. `dimLowConfidence` darkens low- and
    /// medium-confidence points toward black; skipped for `.confidence` mode,
    /// which already encodes confidence as color.
    public static func colorTransform(mode: PointColorMode,
                                      heightRange: (minY: Float, maxY: Float)? = nil,
                                      dimLowConfidence: Bool = false) -> (PointCloudPoint) -> SIMD3<UInt8> {
        let base = baseColorTransform(mode: mode, heightRange: heightRange)
        guard dimLowConfidence, mode != .confidence else { return base }
        return { point in
            let factor = confidenceDimFactor(point.confidence)
            guard factor < 1 else { return base(point) }
            let color = base(point)
            return SIMD3<UInt8>(UInt8(Float(color.x) * factor),
                                UInt8(Float(color.y) * factor),
                                UInt8(Float(color.z) * factor))
        }
    }

    private static func baseColorTransform(mode: PointColorMode,
                                           heightRange: (minY: Float, maxY: Float)?) -> (PointCloudPoint) -> SIMD3<UInt8> {
        switch mode {
        case .cameraColor, .lit:
            return { $0.color }
        case .confidence:
            // Traffic-light tint blended over the camera color, so reliability
            // reads without losing the scene context.
            return { point in
                let tint: SIMD3<Float>
                switch point.confidence {
                case 0: tint = SIMD3<Float>(235, 70, 60)
                case 1: tint = SIMD3<Float>(245, 190, 55)
                default: tint = SIMD3<Float>(70, 220, 100)
                }
                let base = SIMD3<Float>(Float(point.color.x), Float(point.color.y), Float(point.color.z))
                let mixed = base * 0.4 + tint * 0.6
                return SIMD3<UInt8>(UInt8(min(max(mixed.x, 0), 255)),
                                    UInt8(min(max(mixed.y, 0), 255)),
                                    UInt8(min(max(mixed.z, 0), 255)))
            }
        case .solid(let color):
            return { _ in color }
        case .heightGradient:
            guard let range = heightRange, range.maxY - range.minY > 0.001 else {
                return { _ in heightRamp(0.5) }
            }
            return { point in
                heightRamp((point.position.y - range.minY) / (range.maxY - range.minY))
            }
        }
    }

    /// Applies the mode to a point cloud. Height gradient maps world Y across
    /// the cloud's min/max; a flat range yields the ramp midpoint.
    public static func apply(_ mode: PointColorMode, to points: [PointCloudPoint]) -> [PointCloudPoint] {
        switch mode {
        case .cameraColor, .lit:
            // `.lit` colors are baked by the renderer (normals are renderer-side).
            return points
        case .solid, .confidence:
            let transform = colorTransform(mode: mode)
            return points.map { PointCloudPoint(position: $0.position, color: transform($0), confidence: $0.confidence) }
        case .heightGradient:
            let transform = colorTransform(mode: mode, heightRange: heightRange(of: points))
            return points.map { PointCloudPoint(position: $0.position, color: transform($0), confidence: $0.confidence) }
        }
    }
}
