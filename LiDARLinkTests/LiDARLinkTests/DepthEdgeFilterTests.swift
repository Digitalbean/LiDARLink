import XCTest
import simd
@testable import LiDARLinkShared

final class DepthEdgeFilterTests: XCTestCase {

    // MARK: - Fixtures

    /// Flat background at `far` m with a centred square patch at `near` m.
    private func squareScene(size: Int = 40,
                             lo: Int = 15,
                             hi: Int = 24,
                             near: Float = 1.0,
                             far: Float = 3.0) -> [Float] {
        var depth = [Float](repeating: far, count: size * size)
        for y in lo...hi {
            for x in lo...hi {
                depth[y * size + x] = near
            }
        }
        return depth
    }

    /// Depth ramps smoothly across the columns with no steps (a wall seen at a
    /// grazing angle). Deliberately steep enough that the *first* difference
    /// trips near the near end — only the curvature guard keeps it.
    private func grazingRamp(width: Int = 32,
                             height: Int = 24,
                             nearMeters: Float = 1.0,
                             farMeters: Float = 4.0) -> [Float] {
        var depth = [Float](repeating: 0, count: width * height)
        for y in 0..<height {
            for x in 0..<width {
                let t = Float(x) / Float(width - 1)
                depth[y * width + x] = nearMeters + (farMeters - nearMeters) * t
            }
        }
        return depth
    }

    // MARK: - Square patch: border removed, interiors kept

    func testSquarePatchBorderRejectedInteriorsKept() {
        let size = 40
        let depth = squareScene(size: size)
        let mask = DepthEdgeFilter.rejectionMask(depth: depth, width: size, height: size)

        let rejected = mask.filter { $0 }.count
        XCTAssertGreaterThan(rejected, 0, "the square's border must be detected")
        XCTAssertLessThan(rejected, 400,
                          "rejection must stay near the border, not decimate the frame")

        // Deep background interior (far from the patch) is untouched.
        for y in 2...8 {
            for x in 2...8 {
                XCTAssertFalse(mask[y * size + x], "background interior (\(x),\(y)) kept")
            }
        }
        // Square interior is untouched.
        for y in 18...21 {
            for x in 18...21 {
                XCTAssertFalse(mask[y * size + x], "square interior (\(x),\(y)) kept")
            }
        }

        // Pixels straddling the left edge of the square (col 14 background,
        // col 15 square) are removed.
        XCTAssertTrue(mask[20 * size + 14], "background pixel against the step is removed")
        XCTAssertTrue(mask[20 * size + 15], "square pixel against the step is removed")
    }

    func testSquarePatchInteriorCountsWithinAFewPercent() {
        let size = 40
        let depth = squareScene(size: size)
        let mask = DepthEdgeFilter.rejectionMask(depth: depth, width: size, height: size)

        // Count kept pixels in the flat interior of each surface.
        func keptInBlock(_ xr: ClosedRange<Int>, _ yr: ClosedRange<Int>) -> (kept: Int, total: Int) {
            var kept = 0, total = 0
            for y in yr { for x in xr { total += 1; if !mask[y * size + x] { kept += 1 } } }
            return (kept, total)
        }
        let bg = keptInBlock(1...10, 1...10)     // background interior
        let sq = keptInBlock(18...21, 18...21)   // square interior
        XCTAssertEqual(bg.kept, bg.total, "background interior kept in full")
        XCTAssertEqual(sq.kept, sq.total, "square interior kept in full")
    }

    // MARK: - Grazing plane: ramp is NOT mistaken for edges

    func testGrazingRampMostlyKept() {
        let w = 32, h = 24
        let depth = grazingRamp(width: w, height: h)
        let mask = DepthEdgeFilter.rejectionMask(depth: depth, width: w, height: h)
        let rejected = mask.filter { $0 }.count
        let fraction = Double(rejected) / Double(w * h)
        XCTAssertLessThan(fraction, 0.05,
                          "a smooth depth ramp must not be read as edges (rejected \(rejected)/\(w * h))")
    }

    // MARK: - buildPointCloud integration

    func testBuildPointCloudEdgeFilterOffIsUnchanged() {
        let size = 40
        let depth16 = squareScene(size: size).map { Float16($0) }
        let intrinsics = CameraIntrinsics(fx: 50, fy: 50, cx: 20, cy: 20, imageWidth: size, imageHeight: size)
        let baseline = PoseMath.buildPointCloud(depth: depth16, width: size, height: size,
                                                depthScale: 1, intrinsics: intrinsics,
                                                pose: matrix_identity_float4x4, step: 1)
        XCTAssertEqual(baseline.count, size * size)

        let defaulted = PoseMath.buildPointCloud(depth: depth16, width: size, height: size,
                                                 depthScale: 1, intrinsics: intrinsics,
                                                 pose: matrix_identity_float4x4, step: 1,
                                                 edgeFilter: false)
        XCTAssertEqual(defaulted.count, baseline.count, "edgeFilter defaults to off / no-op")
    }

    func testBuildPointCloudEdgeFilterOnDropsBorder() {
        let size = 40
        let depth16 = squareScene(size: size).map { Float16($0) }
        let intrinsics = CameraIntrinsics(fx: 50, fy: 50, cx: 20, cy: 20, imageWidth: size, imageHeight: size)

        let unfiltered = PoseMath.buildPointCloud(depth: depth16, width: size, height: size,
                                                  depthScale: 1, intrinsics: intrinsics,
                                                  pose: matrix_identity_float4x4, step: 1,
                                                  edgeFilter: false)
        let filtered = PoseMath.buildPointCloud(depth: depth16, width: size, height: size,
                                                depthScale: 1, intrinsics: intrinsics,
                                                pose: matrix_identity_float4x4, step: 1,
                                                edgeFilter: true)

        XCTAssertEqual(unfiltered.count, size * size)
        XCTAssertLessThan(filtered.count, unfiltered.count, "border points dropped")
        XCTAssertGreaterThan(filtered.count, unfiltered.count - 400, "interiors retained")
    }

    func testBuildPointCloudEdgeFilterKeepsGrazingRamp() {
        let w = 32, h = 24
        let depth16 = grazingRamp(width: w, height: h).map { Float16($0) }
        let intrinsics = CameraIntrinsics(fx: 40, fy: 40, cx: 16, cy: 12, imageWidth: w, imageHeight: h)

        let unfiltered = PoseMath.buildPointCloud(depth: depth16, width: w, height: h,
                                                  depthScale: 1, intrinsics: intrinsics,
                                                  pose: matrix_identity_float4x4, step: 1,
                                                  edgeFilter: false)
        let filtered = PoseMath.buildPointCloud(depth: depth16, width: w, height: h,
                                                depthScale: 1, intrinsics: intrinsics,
                                                pose: matrix_identity_float4x4, step: 1,
                                                edgeFilter: true)
        let dropped = unfiltered.count - filtered.count
        XCTAssertLessThan(Double(dropped) / Double(unfiltered.count), 0.05)
    }
}
