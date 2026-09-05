import Foundation
import simd

/// A portable snapshot of Apple RoomPlan's `CapturedRoom` — the parametric room:
/// walls, floor, ceiling, doors/windows/openings, and detected objects, each as
/// an oriented box (transform + dimensions). The Mac uses the surfaces as plane
/// priors for `PlaneQuantizer` and draws the whole thing as a clean overlay.
public struct RoomStructure: Codable, Equatable, Sendable {

    public enum SurfaceCategory: String, Codable, Sendable {
        case wall, floor, ceiling, door, window, opening
    }

    public enum ObjectCategory: String, Codable, Sendable {
        case storage, refrigerator, stove, bed, sink, washerDryer, toilet, bathtub
        case oven, dishwasher, table, sofa, chair, fireplace, television, stairs
        case unknown
    }

    /// One planar element. `transform` is the element centre in world space; its
    /// local +z is the surface normal, and `dimensions` is (width, height, depth).
    public struct Surface: Codable, Equatable, Sendable {
        public var id: String
        public var category: SurfaceCategory
        public var transform: simd_float4x4
        public var dimensions: SIMD3<Float>
        public var confidence: Float
        /// World-space outline of the surface — RoomPlan's `polygonCorners`
        /// (non-rectangular walls, alcoves) with curved walls tessellated. Empty
        /// means a plain rectangle, reconstructable from `transform`+`dimensions`.
        public var polygon: [SIMD3<Float>]

        public init(id: String, category: SurfaceCategory, transform: simd_float4x4,
                    dimensions: SIMD3<Float>, confidence: Float, polygon: [SIMD3<Float>] = []) {
            self.id = id; self.category = category; self.transform = transform
            self.dimensions = dimensions; self.confidence = confidence
            self.polygon = polygon
        }

        /// World-space plane: unit normal + offset (`n·p == offset`).
        public var plane: (normal: SIMD3<Float>, offset: Float) {
            let n = simd_normalize(SIMD3<Float>(transform.columns.2.x, transform.columns.2.y, transform.columns.2.z))
            let c = SIMD3<Float>(transform.columns.3.x, transform.columns.3.y, transform.columns.3.z)
            return (n, simd_dot(n, c))
        }

        /// The outline to draw — `polygon` if RoomPlan gave one, else the four
        /// rectangle corners from `transform` + `dimensions`.
        public var outline: [SIMD3<Float>] {
            if polygon.count >= 3 { return polygon }
            let hw = dimensions.x / 2, hh = dimensions.y / 2
            return [SIMD3<Float>(-hw, -hh, 0), SIMD3<Float>(hw, -hh, 0),
                    SIMD3<Float>(hw, hh, 0), SIMD3<Float>(-hw, hh, 0)].map {
                let w = transform * SIMD4<Float>($0, 1)
                return SIMD3<Float>(w.x, w.y, w.z)
            }
        }
    }

    public struct Object: Codable, Equatable, Sendable {
        public var id: String
        public var category: ObjectCategory
        public var transform: simd_float4x4
        public var dimensions: SIMD3<Float>
        public var confidence: Float

        public init(id: String, category: ObjectCategory, transform: simd_float4x4,
                    dimensions: SIMD3<Float>, confidence: Float) {
            self.id = id; self.category = category; self.transform = transform
            self.dimensions = dimensions; self.confidence = confidence
        }
    }

    public var surfaces: [Surface]
    public var objects: [Object]
    public var capturedAtMs: UInt64
    /// True once RoomPlan has finished and returned its final model.
    public var isFinal: Bool

    public init(surfaces: [Surface], objects: [Object], capturedAtMs: UInt64, isFinal: Bool) {
        self.surfaces = surfaces
        self.objects = objects
        self.capturedAtMs = capturedAtMs
        self.isFinal = isFinal
    }

    public var walls: [Surface] { surfaces.filter { $0.category == .wall } }
    public var openings: [Surface] { surfaces.filter { $0.category == .door || $0.category == .window || $0.category == .opening } }
}
