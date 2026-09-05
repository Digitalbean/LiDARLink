import XCTest
import simd
@testable import LiDARLinkShared

final class MarchingCubesTests: XCTestCase {

    func testExtractsASphere() {
        let voxel: Float = 0.05
        let centre = SIMD3<Float>(0.5, 0.5, 0.5)
        let radius: Float = 0.35

        var cells: [SIMD3<Int32>] = []
        for z in -2...22 { for y in -2...22 { for x in -2...22 {
            cells.append(SIMD3<Int32>(Int32(x), Int32(y), Int32(z)))
        }}}

        let sampler = MarchingCubes.Sampler(
            value: { coord in
                let world = (SIMD3<Float>(coord) + 0.5) * voxel
                return simd_length(world - centre) - radius   // SDF of a sphere
            },
            colour: { _ in SIMD3<Float>(255, 255, 255) },
            worldPosition: { grid in (grid + 0.5) * voxel })

        let mesh = MarchingCubes.surface(cells: cells, isoLevel: 0, sampler: sampler)

        XCTAssertGreaterThan(mesh.faces.count, 300)
        XCTAssertEqual(mesh.faces.count % 3, 0)
        XCTAssertEqual(mesh.vertices.count, mesh.normals.count)

        // Every vertex sits on the sphere within a voxel.
        for v in mesh.vertices {
            XCTAssertEqual(simd_length(v - centre), radius, accuracy: voxel)
        }
        // Normals point outward.
        for (v, n) in zip(mesh.vertices, mesh.normals) {
            let outward = simd_normalize(v - centre)
            XCTAssertGreaterThan(simd_dot(outward, n), 0.5)
        }
        // Closed surface: every edge shared by exactly two triangles.
        var edgeCount: [Int64: Int] = [:]
        var f = 0
        while f < mesh.faces.count {
            let tri = [mesh.faces[f], mesh.faces[f + 1], mesh.faces[f + 2]]
            for i in 0..<3 {
                let a = min(tri[i], tri[(i + 1) % 3]), b = max(tri[i], tri[(i + 1) % 3])
                edgeCount[Int64(a) << 32 | Int64(b), default: 0] += 1
            }
            f += 3
        }
        let boundaryEdges = edgeCount.values.filter { $0 != 2 }.count
        XCTAssertLessThan(boundaryEdges, edgeCount.count / 20, "surface should be nearly watertight")
    }

    func testTSDFVolumeProducesAWallMesh() {
        let volume = TSDFVolume(voxelSize: 0.03)
        let w = 96, h = 72
        let depth = [Float16](repeating: 1.4, count: w * h)
        let k = CameraIntrinsics(fx: 70, fy: 70, cx: 48, cy: 36, imageWidth: w, imageHeight: h)
        for _ in 0..<5 {
            volume.integrate(depth: depth, width: w, height: h, depthScale: 1,
                             intrinsics: k, pose: matrix_identity_float4x4,
                             colorFor: { _, _ in SIMD3<UInt8>(60, 160, 220) })
        }
        let mesh = volume.mesh()
        XCTAssertGreaterThan(mesh.faces.count, 60)
        XCTAssertTrue(mesh.vertices.allSatisfy { abs($0.z + 1.4) < 0.12 })
        XCTAssertGreaterThan(mesh.colours.map { $0.z }.reduce(0, +) / Float(max(mesh.colours.count, 1)), 120)
    }
}
