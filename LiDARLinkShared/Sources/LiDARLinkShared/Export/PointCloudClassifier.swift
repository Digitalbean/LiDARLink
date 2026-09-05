import Foundation
import simd

/// Turns the v2 surfel map into a classified `RichPointCloud`: each surfel
/// becomes a point, tagged floor / wall / ceiling / opening / furniture /
/// clutter from the detected planes, corner edges and (optionally) RoomPlan.
public enum PointCloudClassifier {

    public struct Parameters: Sendable {
        /// A surfel is "on" a plane within this signed distance and normal angle.
        public var planeDistance: Float = 0.05
        public var planeNormalDegrees: Float = 20
        /// Near an opening's box → door/window instead of wall.
        public var openingMargin: Float = 0.15
        public init() {}
    }

    /// Floor / wall / ceiling / clutter for one oriented sample, from plane
    /// orientation + membership. The shared core of live and export classing.
    public static func classOf(normal n: SIMD3<Float>, position: SIMD3<Float>,
                               planes: [PlaneQuantizer.Plane],
                               parameters p: Parameters = Parameters()) -> RichPointCloud.Classification {
        let up = SIMD3<Float>(0, 1, 0)
        let normCos = cos(p.planeNormalDegrees * .pi / 180)
        var onPlane = planes.isEmpty
        for plane in planes {
            if abs(simd_dot(n, plane.normal)) >= normCos,
               abs(plane.signedDistance(position)) <= p.planeDistance,
               plane.contains(position, margin: 0.2) { onPlane = true; break }
        }
        guard onPlane else { return .clutter }
        let vy = simd_dot(n, up)
        if abs(vy) > 0.8 { return vy > 0 ? .floor : .ceiling }
        if abs(vy) < 0.35 { return .wall }
        return planes.isEmpty ? .clutter : .wall
    }

    public static func classify(surfels: [Surfel],
                                planes: [PlaneQuantizer.Plane] = [],
                                room: RoomStructure? = nil,
                                parameters p: Parameters = Parameters()) -> RichPointCloud {
        // Pre-index RoomPlan openings as world-space oriented boxes.
        struct Box { var inv: simd_float4x4; var half: SIMD3<Float>; var kind: RichPointCloud.Classification }
        var openings: [Box] = []
        var furniture: [Box] = []
        if let room {
            for s in room.openings {
                openings.append(Box(inv: s.transform.inverse,
                                    half: s.dimensions / 2 + SIMD3<Float>(repeating: p.openingMargin),
                                    kind: s.category == .window ? .window : .door))
            }
            for o in room.objects {
                furniture.append(Box(inv: o.transform.inverse, half: o.dimensions / 2,
                                     kind: .furniture))
            }
        }
        func boxHit(_ boxes: [Box], _ pos: SIMD3<Float>) -> RichPointCloud.Classification? {
            for b in boxes {
                let l = b.inv * SIMD4<Float>(pos, 1)
                if abs(l.x) <= b.half.x && abs(l.y) <= b.half.y && abs(l.z) <= b.half.z { return b.kind }
            }
            return nil
        }

        var points: [RichPointCloud.Point] = []
        points.reserveCapacity(surfels.count)
        for s in surfels where s.weight > 0 {
            let n = simd_normalize(s.normal)
            var cls = classOf(normal: n, position: s.position, planes: planes, parameters: p)

            if let opening = boxHit(openings, s.position) { cls = opening }
            else if cls == .clutter, let f = boxHit(furniture, s.position) { cls = f }

            let confidence = simd_clamp(0.5 * (s.weight / 12) + 0.5 * s.bestViewCos, 0, 1)
            points.append(.init(position: s.position, normal: n, color: s.colorBytes,
                                confidence: confidence, classification: cls))
        }
        return RichPointCloud(points: points)
    }
}
