import Foundation
import simd

/// Bakes a texture atlas for a reconstructed mesh by projecting posed colour
/// keyframes onto its triangles. Geometry stays coarse; the surface gets the
/// full colour resolution of the camera frames.
///
/// Per-triangle chart packing (a shelf packer): each triangle gets its own cell
/// in the atlas, sized by its world area. No UV unwrapping, no seam blending
/// beyond a small gutter dilation, no occlusion test — it targets "clearly
/// better than per-vertex colour", not film VFX.
public enum TextureBaker {

    public struct View: Sendable {
        public let sampler: ColorImageSampler
        /// ARKit camera-to-world (GL convention: +x right, +y up, -z forward).
        public let pose: simd_float4x4
        /// Intrinsics already scaled to `sampler`'s resolution.
        public let intrinsics: CameraIntrinsics
        public init(sampler: ColorImageSampler, pose: simd_float4x4, intrinsics: CameraIntrinsics) {
            self.sampler = sampler
            self.pose = pose
            self.intrinsics = intrinsics
        }
    }

    public struct BakedMesh: Sendable {
        public var vertices: [SIMD3<Float>] = []
        public var normals: [SIMD3<Float>] = []
        public var uvs: [SIMD2<Float>] = []
        public var faces: [UInt32] = []
        public var atlasRGBA: [UInt8] = []
        public var atlasSize: Int = 0
        public var texturedTriangleFraction: Float = 0
    }

    public static func bake(mesh: MarchingCubes.Mesh,
                            views: [View],
                            atlasSize: Int = 2048,
                            texelsPerMetre: Float = 600,
                            gutter: Int = 2) -> BakedMesh {
        let triangleCount = mesh.faces.count / 3
        guard triangleCount > 0, !views.isEmpty else { return BakedMesh() }

        // 1. Cell size per triangle from its world area.
        struct Tri { var v: (SIMD3<Float>, SIMD3<Float>, SIMD3<Float>); var n: (SIMD3<Float>, SIMD3<Float>, SIMD3<Float>); var side: Int }
        var tris: [Tri] = []
        tris.reserveCapacity(triangleCount)
        var maxSide = 8
        for t in 0..<triangleCount {
            let i0 = Int(mesh.faces[t * 3]), i1 = Int(mesh.faces[t * 3 + 1]), i2 = Int(mesh.faces[t * 3 + 2])
            let v = (mesh.vertices[i0], mesh.vertices[i1], mesh.vertices[i2])
            let n = (mesh.normals[i0], mesh.normals[i1], mesh.normals[i2])
            let area = simd_length(simd_cross(v.1 - v.0, v.2 - v.0)) * 0.5
            let side = min(max(Int((sqrt(2 * area) * texelsPerMetre).rounded()), 6), 220) + gutter * 2
            maxSide = max(maxSide, side)
            tris.append(Tri(v: v, n: n, side: side))
        }

        // 2. Shelf-pack, shrinking uniformly if it overflows the atlas.
        var scale: Float = 1
        var placements: [(x: Int, y: Int, side: Int)] = []
        for _ in 0..<8 {
            placements.removeAll(keepingCapacity: true)
            var penX = 0, penY = 0, shelfH = 0
            var fits = true
            for tri in tris {
                let side = max(Int(Float(tri.side) * scale), gutter * 2 + 4)
                if penX + side > atlasSize { penX = 0; penY += shelfH; shelfH = 0 }
                if penY + side > atlasSize { fits = false; break }
                placements.append((penX, penY, side))
                penX += side
                shelfH = max(shelfH, side)
            }
            if fits { break }
            scale *= 0.75
        }
        guard placements.count == tris.count else { return BakedMesh() }

        // 3. Rasterise each triangle's cell from its best view.
        var atlas = [UInt8](repeating: 0, count: atlasSize * atlasSize * 4)
        var covered = [Bool](repeating: false, count: atlasSize * atlasSize)
        var out = BakedMesh()
        out.atlasSize = atlasSize
        var textured = 0

        for (index, tri) in tris.enumerated() {
            let (x0, y0, side) = placements[index]
            let inner = max(side - gutter * 2, 1)
            let faceNormal = simd_normalize(tri.n.0 + tri.n.1 + tri.n.2)
            let centroid = (tri.v.0 + tri.v.1 + tri.v.2) / 3
            let best = bestView(centroid: centroid, normal: faceNormal, verts: tri.v, views: views)

            // UVs: right-triangle in the inner cell (v0 → TL, v1 → TR, v2 → BL).
            let uvScale = 1 / Float(atlasSize)
            let uv0 = SIMD2<Float>(Float(x0 + gutter), Float(y0 + gutter)) * uvScale
            let uv1 = SIMD2<Float>(Float(x0 + gutter + inner), Float(y0 + gutter)) * uvScale
            let uv2 = SIMD2<Float>(Float(x0 + gutter), Float(y0 + gutter + inner)) * uvScale
            let base = UInt32(out.vertices.count)
            out.vertices.append(contentsOf: [tri.v.0, tri.v.1, tri.v.2])
            out.normals.append(contentsOf: [tri.n.0, tri.n.1, tri.n.2])
            out.uvs.append(contentsOf: [uv0, uv1, uv2])
            out.faces.append(contentsOf: [base, base + 1, base + 2])

            let fallback = averageColour(centroid: centroid, verts: tri.v, view: best) ?? SIMD3<UInt8>(150, 150, 150)
            if best != nil { textured += 1 }

            for ay in 0..<side {
                for ax in 0..<side {
                    let px = x0 + ax, py = y0 + ay
                    guard px < atlasSize, py < atlasSize else { continue }
                    let s = (Float(ax) - Float(gutter)) / Float(inner)
                    let t = (Float(ay) - Float(gutter)) / Float(inner)
                    let b0 = 1 - s - t
                    guard s >= -0.15, t >= -0.15, b0 >= -0.15 else { continue }
                    let bc = normalisedBary(b0, s, t)
                    let world = tri.v.0 * bc.x + tri.v.1 * bc.y + tri.v.2 * bc.z
                    let colour = best.flatMap { sample($0, world: world) }
                        ?? best.flatMap { sampleClamped($0, world: world) }
                        ?? fallback
                    let o = (py * atlasSize + px) * 4
                    atlas[o] = colour.x; atlas[o + 1] = colour.y; atlas[o + 2] = colour.z; atlas[o + 3] = 255
                    covered[py * atlasSize + px] = true
                }
            }
        }

        dilate(&atlas, covered: &covered, size: atlasSize, passes: gutter + 1)
        out.atlasRGBA = atlas
        out.texturedTriangleFraction = Float(textured) / Float(tris.count)
        return out
    }

    // MARK: View selection & sampling

    private static func bestView(centroid: SIMD3<Float>,
                                 normal: SIMD3<Float>,
                                 verts: (SIMD3<Float>, SIMD3<Float>, SIMD3<Float>),
                                 views: [View]) -> View? {
        var best: View?
        var bestScore: Float = 0
        for view in views {
            let camPos = SIMD3<Float>(view.pose.columns.3.x, view.pose.columns.3.y, view.pose.columns.3.z)
            let toCam = camPos - centroid
            let dist = simd_length(toCam)
            guard dist > 0.05 else { continue }
            let facing = simd_dot(normal, toCam / dist)
            guard facing > 0.1 else { continue }                  // triangle faces the camera
            guard project(view, centroid) != nil else { continue } // centre visible
            let inFrame = (project(view, verts.0) != nil ? 1 : 0)
                        + (project(view, verts.1) != nil ? 1 : 0)
                        + (project(view, verts.2) != nil ? 1 : 0)
            guard inFrame >= 2 else { continue }                   // most of the triangle in frame
            let score = facing / (dist * dist) * (0.6 + 0.4 * Float(inFrame) / 3)
            if score > bestScore { bestScore = score; best = view }
        }
        return best
    }

    private static func project(_ view: View, _ world: SIMD3<Float>) -> (Int, Int)? {
        let cam = view.pose.inverse * SIMD4<Float>(world, 1)
        let z = -cam.z
        guard z > 0.05 else { return nil }
        let k = view.intrinsics
        let u = Int((k.fx * cam.x / z + k.cx).rounded())
        let v = Int((k.cy - k.fy * cam.y / z).rounded())
        guard u >= 0, u < view.sampler.width, v >= 0, v < view.sampler.height else { return nil }
        return (u, v)
    }

    private static func sample(_ view: View, world: SIMD3<Float>) -> SIMD3<UInt8>? {
        guard let (u, v) = project(view, world) else { return nil }
        return view.sampler.color(atX: u, y: v)
    }

    /// Like `sample` but clamps the projection into the frame instead of failing
    /// — used for chart-edge texels so they get a plausible colour, not grey.
    private static func sampleClamped(_ view: View, world: SIMD3<Float>) -> SIMD3<UInt8>? {
        let cam = view.pose.inverse * SIMD4<Float>(world, 1)
        let z = -cam.z
        guard z > 0.05 else { return nil }
        let k = view.intrinsics
        let u = min(max(Int((k.fx * cam.x / z + k.cx).rounded()), 0), view.sampler.width - 1)
        let v = min(max(Int((k.cy - k.fy * cam.y / z).rounded()), 0), view.sampler.height - 1)
        return view.sampler.color(atX: u, y: v)
    }

    private static func averageColour(centroid: SIMD3<Float>,
                                      verts: (SIMD3<Float>, SIMD3<Float>, SIMD3<Float>),
                                      view: View?) -> SIMD3<UInt8>? {
        guard let view else { return nil }
        var sum = SIMD3<Int>(0, 0, 0)
        var count = 0
        for p in [centroid, verts.0, verts.1, verts.2] {
            if let c = sample(view, world: p) { sum &+= SIMD3<Int>(Int(c.x), Int(c.y), Int(c.z)); count += 1 }
        }
        guard count > 0 else { return nil }
        return SIMD3<UInt8>(UInt8(sum.x / count), UInt8(sum.y / count), UInt8(sum.z / count))
    }

    private static func normalisedBary(_ a: Float, _ b: Float, _ c: Float) -> SIMD3<Float> {
        let clamped = SIMD3<Float>(max(a, 0), max(b, 0), max(c, 0))
        let s = clamped.x + clamped.y + clamped.z
        return s > 1e-5 ? clamped / s : SIMD3<Float>(1, 0, 0)
    }

    /// Spreads covered texels outward so bilinear filtering doesn't sample the
    /// black background at chart edges.
    private static func dilate(_ atlas: inout [UInt8], covered: inout [Bool], size: Int, passes: Int) {
        guard passes > 0 else { return }
        for _ in 0..<passes {
            let snapshot = covered
            for y in 0..<size {
                for x in 0..<size {
                    let i = y * size + x
                    if snapshot[i] { continue }
                    for (dx, dy) in [(-1, 0), (1, 0), (0, -1), (0, 1)] {
                        let nx = x + dx, ny = y + dy
                        guard nx >= 0, nx < size, ny >= 0, ny < size, snapshot[ny * size + nx] else { continue }
                        let src = (ny * size + nx) * 4
                        let dst = i * 4
                        atlas[dst] = atlas[src]; atlas[dst + 1] = atlas[src + 1]
                        atlas[dst + 2] = atlas[src + 2]; atlas[dst + 3] = 255
                        covered[i] = true
                        break
                    }
                }
            }
        }
    }
}
