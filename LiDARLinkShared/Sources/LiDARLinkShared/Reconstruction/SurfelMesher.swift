import Foundation
import simd

/// Bakes a surfel map into a triangle mesh: each surfel is splatted as a small
/// oriented signed-distance sample into a hashed voxel grid, then marching cubes
/// pulls the zero-crossing surface out. This is the "finalize" step — the live
/// map stays surfels, export produces a watertight mesh.
public enum SurfelMesher {

    public struct Parameters: Sendable {
        public var voxelSize: Float = 0.015
        /// SDF truncation as a multiple of the voxel size.
        public var truncationVoxels: Float = 4
        /// A surfel contributes out to this multiple of its radius, tangentially.
        public var tangentialReachFactor: Float = 1.3
        public var minWeight: Float = 1.5
        public init() {}
    }

    public static func mesh(from surfels: [Surfel], parameters: Parameters = Parameters()) -> MarchingCubes.Mesh {
        guard !surfels.isEmpty else { return MarchingCubes.Mesh() }
        let vs = max(parameters.voxelSize, 0.005)
        let inv = 1 / vs
        let trunc = vs * max(parameters.truncationVoxels, 1)

        struct Voxel { var sdf: Float = 0; var weight: Float = 0; var color: SIMD3<Float> = .zero }
        var grid: [Int64: Voxel] = [:]
        grid.reserveCapacity(surfels.count * 16)

        func key(_ x: Int64, _ y: Int64, _ z: Int64) -> Int64 {
            let m: Int64 = 0x1F_FFFF
            return (x & m) | ((y & m) << 21) | ((z & m) << 42)
        }

        for s in surfels {
            guard s.weight >= parameters.minWeight, simd_length(s.normal) > 1e-5 else { continue }
            let n = simd_normalize(s.normal)
            let reach = max(s.radius, vs) * parameters.tangentialReachFactor
            let lo = s.position - SIMD3<Float>(repeating: reach + trunc)
            let hi = s.position + SIMD3<Float>(repeating: reach + trunc)
            var iz = Int64(floor(lo.z * inv))
            let ez = Int64(floor(hi.z * inv))
            while iz <= ez {
                var iy = Int64(floor(lo.y * inv))
                let ey = Int64(floor(hi.y * inv))
                while iy <= ey {
                    var ix = Int64(floor(lo.x * inv))
                    let ex = Int64(floor(hi.x * inv))
                    while ix <= ex {
                        let center = (SIMD3<Float>(Float(ix), Float(iy), Float(iz)) + 0.5) * vs
                        let rel = center - s.position
                        let along = simd_dot(rel, n)
                        if abs(along) <= trunc {
                            let tangential = simd_length(rel - along * n)
                            if tangential <= reach {
                                let w = s.weight * (1 - tangential / reach)
                                if w > 1e-4 {
                                    let k = key(ix, iy, iz)
                                    var v = grid[k] ?? Voxel()
                                    let nw = v.weight + w
                                    let sample = simd_clamp(along / trunc, -1, 1)
                                    v.sdf = (v.sdf * v.weight + sample * w) / nw
                                    v.color = (v.color * v.weight + s.color * w) / nw
                                    v.weight = nw
                                    grid[k] = v
                                }
                            }
                        }
                        ix += 1
                    }
                    iy += 1
                }
                iz += 1
            }
        }
        guard !grid.isEmpty else { return MarchingCubes.Mesh() }

        var cells: [SIMD3<Int32>] = []
        cells.reserveCapacity(grid.count)
        for k in grid.keys {
            // unpack 21-bit signed
            func s21(_ raw: Int64) -> Int32 {
                let masked = raw & 0x1F_FFFF
                return Int32(masked >= 0x10_0000 ? masked - 0x20_0000 : masked)
            }
            cells.append(SIMD3<Int32>(s21(k), s21(k >> 21), s21(k >> 42)))
        }

        let sampler = MarchingCubes.Sampler(
            value: { grid[key(Int64($0.x), Int64($0.y), Int64($0.z))].map { $0.weight >= 1 ? $0.sdf : nil } ?? nil },
            colour: {
                let v = grid[key(Int64($0.x), Int64($0.y), Int64($0.z))]
                return v?.color ?? SIMD3<Float>(repeating: 160)
            },
            worldPosition: { ($0 + 0.5) * vs })
        return MarchingCubes.surface(cells: cells, isoLevel: 0, sampler: sampler)
    }
}
