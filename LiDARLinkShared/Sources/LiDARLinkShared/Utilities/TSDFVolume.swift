import Foundation
import simd

/// Voxel-hashed truncated signed-distance fusion.
///
/// Instead of averaging observed points into voxel centroids (which blends pose
/// drift into a soft blur), each voxel stores its signed distance to the nearest
/// surface. Integrating a depth frame carves everything between the camera and
/// the measured surface as free space, so floaters delete themselves, and the
/// surface sits at the zero-crossing with sub-voxel precision.
///
/// Storage is 8³-voxel blocks in a hash map, allocated on demand near observed
/// surfaces. Thread-safe.
public final class TSDFVolume: @unchecked Sendable {

    public struct Voxel: Equatable {
        var sdf: Float = 1          // truncated signed distance, normalised to [-1, 1]
        var weight: Float = 0
        var color: SIMD3<Float> = .zero
    }

    public let voxelSize: Float
    public let truncation: Float          // metres; SDF is normalised by this
    public let maxWeight: Float
    public let maxBlocks: Int

    static let blockEdge = 8
    static let blockVoxels = 512          // 8³

    private let lock = NSLock()
    private var blocks: [Int64: [Voxel]] = [:]
    private let invVoxel: Float
    public private(set) var integratedFrames = 0

    public init(voxelSize: Float = 0.025,
                truncationVoxels: Float = 5,
                maxWeight: Float = 64,
                maxBlocks: Int = 60_000) {
        // Below ~1.2 cm the grid mostly resolves depth noise, not geometry, and
        // the block count explodes. The truncation band still has to stay a few
        // voxels wide so many noisy observations average toward the true surface
        // rather than punching holes.
        self.voxelSize = max(voxelSize, 0.012)
        self.truncation = max(self.voxelSize * max(truncationVoxels, 1), 0.045)
        self.maxWeight = max(maxWeight, 1)
        self.maxBlocks = max(maxBlocks, 1)
        self.invVoxel = 1 / self.voxelSize
    }

    // MARK: Integration

    /// Fuses one depth frame. `pose` is ARKit camera-to-world (GL convention:
    /// +x right, +y up, -z forward). `depthScale` converts stored depth to metres.
    /// `sign` = -1 removes a previously-integrated frame (deintegration) so a
    /// pose-graph update can re-place a keyframe.
    public func integrate(depth: [Float16],
                          width: Int,
                          height: Int,
                          depthScale: Float,
                          intrinsics: CameraIntrinsics,
                          pose: simd_float4x4,
                          confidence: [UInt8]? = nil,
                          minConfidence: UInt8 = 0,
                          sign: Float = 1,
                          colorFor: ((Int, Int) -> SIMD3<UInt8>)? = nil) {
        guard depth.count >= width * height, width > 0, height > 0 else { return }
        lock.lock()
        defer { lock.unlock() }

        let k = intrinsics.scaled(to: width, height: height)
        let worldToCamera = pose.inverse
        let cameraPositionWorld = SIMD3<Float>(pose.columns.3.x, pose.columns.3.y, pose.columns.3.z)

        // 1. Allocate blocks around the observed surface (subsampled).
        let allocStride = max(1, min(width, height) / 96)
        var y = 0
        while y < height {
            var x = 0
            while x < width {
                let index = y * width + x
                let measured = Float(depth[index]) * depthScale
                if measured.isFinite, measured > 0.05,
                   !(minConfidence > 0 && (confidence?[index] ?? 2) < minConfidence) {
                    let camPoint = Self.unproject(px: Float(x), py: Float(y), depth: measured, k: k)
                    let world = pose * SIMD4<Float>(camPoint, 1)
                    allocateBlocks(around: SIMD3<Float>(world.x, world.y, world.z))
                }
                x += allocStride
            }
            y += allocStride
        }

        guard blocks.count <= maxBlocks * 2 else { return }

        // 2. Integrate: project each voxel of each block into the depth image.
        let fx = k.fx, fy = k.fy, cx = k.cx, cy = k.cy
        let blockSpan = Float(Self.blockEdge) * voxelSize
        let blockRadius = blockSpan * 0.87           // ~half the diagonal
        let margin = Int(blockRadius * fx / 1.0) + Self.blockEdge
        for (key, var voxels) in blocks {
            let (bx, by, bz) = Self.unpack(key)
            let base = SIMD3<Float>(Float(bx * Int64(Self.blockEdge)),
                                    Float(by * Int64(Self.blockEdge)),
                                    Float(bz * Int64(Self.blockEdge))) * voxelSize

            // Frustum + range cull: skip blocks the camera can't see this frame.
            let centreH = worldToCamera * SIMD4<Float>(base + SIMD3<Float>(repeating: blockSpan * 0.5), 1)
            let centreZ = -centreH.z
            if centreZ < -blockRadius || centreZ > 6 + blockRadius { continue }
            if centreZ > 0.05 {
                let cu = Int(fx * centreH.x / centreZ + cx)
                let cv = Int(cy - fy * centreH.y / centreZ)
                if cu < -margin || cu > width + margin || cv < -margin || cv > height + margin { continue }
            }

            var changed = false
            for vi in 0..<Self.blockVoxels {
                let lx = vi & 7, ly = (vi >> 3) & 7, lz = (vi >> 6) & 7
                let worldPos = base + SIMD3<Float>(Float(lx) + 0.5, Float(ly) + 0.5, Float(lz) + 0.5) * voxelSize
                let camH = worldToCamera * SIMD4<Float>(worldPos, 1)
                let camZ = -camH.z                      // metres in front of camera
                guard camZ > 0.05 else { continue }
                let u = Int((fx * camH.x / camZ + cx).rounded())
                let v = Int((cy - fy * camH.y / camZ).rounded())
                guard u >= 0, u < width, v >= 0, v < height else { continue }
                let measured = Float(depth[v * width + u]) * depthScale
                guard measured.isFinite, measured > 0.05 else { continue }
                if minConfidence > 0, (confidence?[v * width + u] ?? 2) < minConfidence { continue }

                let sdf = measured - camZ              // + = voxel in free space
                guard sdf >= -truncation else { continue }  // behind the surface: unknown
                let tsdf = min(sdf / truncation, 1)

                // KinectFusion-style: +1 per observation (capped), with a mild
                // range falloff so far, noisy returns count for a bit less.
                let w: Float = min(1, 4 / max(measured * measured, 1)) * sign

                var voxel = voxels[vi]
                let newWeight = voxel.weight + w
                if newWeight <= 1e-4 {
                    // Fully removed — reset the voxel.
                    voxel = Voxel()
                    voxels[vi] = voxel
                    changed = true
                    continue
                }
                voxel.sdf = (voxel.sdf * voxel.weight + tsdf * w) / newWeight
                if abs(sdf) < truncation, let colorFor {
                    let c = colorFor(u, v)
                    let cf = SIMD3<Float>(Float(c.x), Float(c.y), Float(c.z))
                    voxel.color = (voxel.color * voxel.weight + cf * w) / newWeight
                }
                voxel.weight = min(newWeight, maxWeight)
                voxels[vi] = voxel
                changed = true
            }
            if changed { blocks[key] = voxels }
        }
        integratedFrames += 1
    }

    // MARK: Extraction

    /// Surface points at the zero-crossing of the field, one per sign-changing
    /// voxel edge (sub-voxel interpolated). `minWeight` filters unstable voxels.
    public func surfacePoints(minWeight: Float = 1) -> [PointCloudPoint] {
        lock.lock()
        defer { lock.unlock() }
        var result: [PointCloudPoint] = []
        result.reserveCapacity(blocks.count * 64)

        for (key, voxels) in blocks {
            let (bx, by, bz) = Self.unpack(key)
            let originVoxel = SIMD3<Int64>(bx * Int64(Self.blockEdge), by * Int64(Self.blockEdge), bz * Int64(Self.blockEdge))
            for vi in 0..<Self.blockVoxels {
                let lx = vi & 7, ly = (vi >> 3) & 7, lz = (vi >> 6) & 7
                let here = voxels[vi]
                guard here.weight >= minWeight, abs(here.sdf) < 1 else { continue }
                let gx = originVoxel.x + Int64(lx), gy = originVoxel.y + Int64(ly), gz = originVoxel.z + Int64(lz)

                for (dx, dy, dz) in [(1, 0, 0), (0, 1, 0), (0, 0, 1)] {
                    guard let neighbour = voxelUnlocked(gx + Int64(dx), gy + Int64(dy), gz + Int64(dz)),
                          neighbour.weight >= minWeight, abs(neighbour.sdf) < 1 else { continue }
                    guard (here.sdf <= 0) != (neighbour.sdf <= 0) else { continue }
                    let t = here.sdf / (here.sdf - neighbour.sdf)   // 0..1 along the edge
                    let p0 = SIMD3<Float>(Float(gx), Float(gy), Float(gz))
                    let edge = SIMD3<Float>(Float(dx), Float(dy), Float(dz))
                    let world = (p0 + edge * t + 0.5) * voxelSize
                    let c = simd_mix(here.color, neighbour.color, SIMD3<Float>(repeating: t))
                    result.append(PointCloudPoint(
                        position: world,
                        color: SIMD3<UInt8>(UInt8(min(max(c.x, 0), 255)),
                                            UInt8(min(max(c.y, 0), 255)),
                                            UInt8(min(max(c.z, 0), 255))),
                        confidence: 2))
                }
            }
        }
        return result
    }

    /// A triangle mesh of the zero-level surface via Marching Cubes, with
    /// gradient normals and averaged colour. Vertices are in world space.
    public func mesh(minWeight: Float = 1) -> MarchingCubes.Mesh {
        lock.lock()
        defer { lock.unlock() }

        // Grid points are voxel centres; a cell spans centre c .. c+(1,1,1).
        var cells: [SIMD3<Int32>] = []
        cells.reserveCapacity(blocks.count * 64)
        for (key, voxels) in blocks {
            let (bx, by, bz) = Self.unpack(key)
            let origin = SIMD3<Int32>(Int32(bx * Int64(Self.blockEdge)),
                                      Int32(by * Int64(Self.blockEdge)),
                                      Int32(bz * Int64(Self.blockEdge)))
            for vi in 0..<Self.blockVoxels where voxels[vi].weight >= minWeight {
                let lx = Int32(vi & 7), ly = Int32((vi >> 3) & 7), lz = Int32((vi >> 6) & 7)
                cells.append(origin &+ SIMD3<Int32>(lx, ly, lz))
            }
        }

        let vs = voxelSize
        let sampler = MarchingCubes.Sampler(
            value: { [self] coord in
                guard let v = voxelUnlocked(Int64(coord.x), Int64(coord.y), Int64(coord.z)),
                      v.weight >= minWeight else { return nil }
                return v.sdf
            },
            colour: { [self] coord in
                voxelUnlocked(Int64(coord.x), Int64(coord.y), Int64(coord.z))?.color ?? .zero
            },
            worldPosition: { grid in (grid + 0.5) * vs })
        return MarchingCubes.surface(cells: cells, isoLevel: 0, sampler: sampler)
    }

    public var occupiedBlockCount: Int {
        lock.lock(); defer { lock.unlock() }
        return blocks.count
    }

    public func clear() {
        lock.lock(); defer { lock.unlock() }
        blocks.removeAll(keepingCapacity: true)
        integratedFrames = 0
    }

    // MARK: Internals

    private func allocateBlocks(around world: SIMD3<Float>) {
        let reach = Int(ceil(truncation * invVoxel / Float(Self.blockEdge)))
        let edge = Int64(Self.blockEdge)
        let cx = Self.floorDiv(Int64(floor(world.x * invVoxel)), edge)
        let cy = Self.floorDiv(Int64(floor(world.y * invVoxel)), edge)
        let cz = Self.floorDiv(Int64(floor(world.z * invVoxel)), edge)
        for bz in -reach...reach {
            for by in -reach...reach {
                for bx in -reach...reach {
                    let key = Self.pack(cx + Int64(bx), cy + Int64(by), cz + Int64(bz))
                    if blocks[key] == nil, blocks.count < maxBlocks {
                        blocks[key] = [Voxel](repeating: Voxel(), count: Self.blockVoxels)
                    }
                }
            }
        }
    }

    private func voxelUnlocked(_ gx: Int64, _ gy: Int64, _ gz: Int64) -> Voxel? {
        let edge = Int64(Self.blockEdge)
        let bx = Self.floorDiv(gx, edge), by = Self.floorDiv(gy, edge), bz = Self.floorDiv(gz, edge)
        guard let block = blocks[Self.pack(bx, by, bz)] else { return nil }
        let lx = Int(gx - bx * edge), ly = Int(gy - by * edge), lz = Int(gz - bz * edge)
        return block[lx + ly * 8 + lz * 64]
    }

    /// Floor division (Swift's `/` truncates toward zero, which breaks the
    /// block/voxel mapping for negative world coordinates).
    static func floorDiv(_ a: Int64, _ b: Int64) -> Int64 {
        let q = a / b
        return (a % b != 0 && (a < 0) != (b < 0)) ? q - 1 : q
    }

    static func unproject(px: Float, py: Float, depth: Float, k: CameraIntrinsics) -> SIMD3<Float> {
        SIMD3<Float>((px - k.cx) * depth / k.fx,
                     (k.cy - py) * depth / k.fy,
                     -depth)
    }

    // 21-bit signed pack, matching ProgressiveMap's scheme.
    static func pack(_ x: Int64, _ y: Int64, _ z: Int64) -> Int64 {
        let mask: Int64 = 0x1F_FFFF
        return (x & mask) | ((y & mask) << 21) | ((z & mask) << 42)
    }

    static func unpack(_ key: Int64) -> (Int64, Int64, Int64) {
        func signed(_ v: Int64) -> Int64 { v >= 0x10_0000 ? v - 0x20_0000 : v }
        return (signed(key & 0x1F_FFFF), signed((key >> 21) & 0x1F_FFFF), signed((key >> 42) & 0x1F_FFFF))
    }
}
