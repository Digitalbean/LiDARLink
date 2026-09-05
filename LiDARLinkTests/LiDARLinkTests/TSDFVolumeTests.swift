import XCTest
import simd
@testable import LiDARLinkShared

final class TSDFVolumeTests: XCTestCase {

    private let width = 128
    private let height = 96
    private var intrinsics: CameraIntrinsics {
        CameraIntrinsics(fx: 100, fy: 100, cx: 64, cy: 48, imageWidth: width, imageHeight: height)
    }

    /// A fronto-parallel wall: every pixel reads the same planar depth.
    private func wall(depthMetres: Float, centrePatch: (size: Int, depth: Float)? = nil) -> [Float16] {
        var d = [Float16](repeating: Float16(depthMetres), count: width * height)
        if let patch = centrePatch {
            let half = patch.size / 2
            for y in (height / 2 - half)..<(height / 2 + half) {
                for x in (width / 2 - half)..<(width / 2 + half) {
                    d[y * width + x] = Float16(patch.depth)
                }
            }
        }
        return d
    }

    func testRecoversAFlatWall() {
        let volume = TSDFVolume(voxelSize: 0.02, truncationVoxels: 4)
        for _ in 0..<4 {
            volume.integrate(depth: wall(depthMetres: 1.5), width: width, height: height,
                             depthScale: 1, intrinsics: intrinsics, pose: matrix_identity_float4x4)
        }
        let points = volume.surfacePoints()
        XCTAssertGreaterThan(points.count, 200)
        // Camera at origin looking -z → wall surface sits at world z ≈ -1.5.
        let zs = points.map { $0.position.z }
        let meanZ = zs.reduce(0, +) / Float(zs.count)
        XCTAssertEqual(meanZ, -1.5, accuracy: 0.03)
        XCTAssertLessThan(zs.map { abs($0 - meanZ) }.max() ?? 1, 0.05)
    }

    func testFreeSpaceCarvingDeletesAFloater() {
        let volume = TSDFVolume(voxelSize: 0.02, truncationVoxels: 4)
        // A few frames that also see a bogus surface 0.5 m in front of the wall
        // (e.g. a reflection), enough to build weight past the stability floor.
        for _ in 0..<3 {
            volume.integrate(depth: wall(depthMetres: 1.5, centrePatch: (size: 30, depth: 1.0)),
                             width: width, height: height, depthScale: 1,
                             intrinsics: intrinsics, pose: matrix_identity_float4x4)
        }
        let withFloater = volume.surfacePoints()
        XCTAssertTrue(withFloater.contains { $0.position.z > -1.2 }, "floater near z=-1.0 should exist")

        // Clean wall frames carve the free space in front of the wall.
        for _ in 0..<10 {
            volume.integrate(depth: wall(depthMetres: 1.5), width: width, height: height,
                             depthScale: 1, intrinsics: intrinsics, pose: matrix_identity_float4x4)
        }
        let carved = volume.surfacePoints()
        XCTAssertFalse(carved.contains { $0.position.z > -1.3 }, "floater should be carved away")
        XCTAssertTrue(carved.allSatisfy { abs($0.position.z + 1.5) < 0.06 }, "only the wall remains")
    }

    func testColorIsIntegrated() {
        let volume = TSDFVolume(voxelSize: 0.02)
        for _ in 0..<4 {
            volume.integrate(depth: wall(depthMetres: 1.2), width: width, height: height,
                             depthScale: 1, intrinsics: intrinsics, pose: matrix_identity_float4x4,
                             colorFor: { _, _ in SIMD3<UInt8>(200, 40, 40) })
        }
        let points = volume.surfacePoints()
        XCTAssertGreaterThan(points.count, 100)
        let meanRed = points.map { Float($0.color.x) }.reduce(0, +) / Float(points.count)
        XCTAssertGreaterThan(meanRed, 150)
    }

    func testClearAndEmptyInput() {
        let volume = TSDFVolume(voxelSize: 0.02)
        volume.integrate(depth: wall(depthMetres: 1.0), width: width, height: height,
                         depthScale: 1, intrinsics: intrinsics, pose: matrix_identity_float4x4)
        XCTAssertGreaterThan(volume.occupiedBlockCount, 0)
        volume.clear()
        XCTAssertEqual(volume.occupiedBlockCount, 0)
        volume.integrate(depth: [], width: 0, height: 0, depthScale: 1,
                         intrinsics: intrinsics, pose: matrix_identity_float4x4)
        XCTAssertEqual(volume.occupiedBlockCount, 0)
    }
}
