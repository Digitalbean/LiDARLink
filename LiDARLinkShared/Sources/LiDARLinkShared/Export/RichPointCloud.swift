import Foundation
import simd

/// A survey-grade point cloud: every point carries an orientation, colour,
/// confidence, and a semantic class. Built from the v2 surfel map (each surfel
/// is already an oriented, weighted, classifiable sample). This is what the
/// LAS / enhanced-PLY exporters serialise.
public struct RichPointCloud: Sendable {

    /// ASPRS-compatible where one exists, user codes (20+) otherwise. Consumers
    /// (CloudCompare, ReCap, Revit) filter/colour by the raw code.
    public enum Classification: UInt8, Sendable, CaseIterable {
        case unclassified = 1
        case floor = 2            // ASPRS "ground"
        case wall = 6            // ASPRS "building"
        case ceiling = 9
        case door = 20
        case window = 21
        case furniture = 22
        case clutter = 23

        public var label: String {
            switch self {
            case .unclassified: return "Unclassified"
            case .floor: return "Floor"
            case .wall: return "Wall"
            case .ceiling: return "Ceiling"
            case .door: return "Door"
            case .window: return "Window"
            case .furniture: return "Furniture"
            case .clutter: return "Clutter"
            }
        }
    }

    public struct Point: Sendable {
        public var position: SIMD3<Float>
        public var normal: SIMD3<Float>
        public var color: SIMD3<UInt8>
        /// 0…1 — folds surfel observation weight and view quality.
        public var confidence: Float
        public var classification: Classification

        public init(position: SIMD3<Float>, normal: SIMD3<Float>, color: SIMD3<UInt8>,
                    confidence: Float, classification: Classification) {
            self.position = position
            self.normal = normal
            self.color = color
            self.confidence = confidence
            self.classification = classification
        }
    }

    public var points: [Point]
    /// Camera positions along the scan path — carried into E57 / for context.
    public var trajectory: [simd_float4x4]

    public init(points: [Point], trajectory: [simd_float4x4] = []) {
        self.points = points
        self.trajectory = trajectory
    }

    public var bounds: (min: SIMD3<Float>, max: SIMD3<Float>) {
        guard let first = points.first?.position else { return (.zero, .zero) }
        var lo = first, hi = first
        for p in points { lo = simd_min(lo, p.position); hi = simd_max(hi, p.position) }
        return (lo, hi)
    }

    public func histogram() -> [Classification: Int] {
        var h: [Classification: Int] = [:]
        for p in points { h[p.classification, default: 0] += 1 }
        return h
    }
}
