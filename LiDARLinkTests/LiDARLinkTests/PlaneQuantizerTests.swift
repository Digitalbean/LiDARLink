import XCTest
import simd
@testable import LiDARLinkShared

final class PlaneQuantizerTests: XCTestCase {

    private func surfel(_ p: SIMD3<Float>, _ n: SIMD3<Float>, w: Float = 3) -> Surfel {
        Surfel(position: p, normal: simd_normalize(n), color: SIMD3<Float>(repeating: 150),
               radius: 0.05, weight: w, owner: 0, lastSeen: 0)
    }

    /// A noisy vertical wall at x = 2, plus a noisy floor at y = 0.
    private func room() -> [Surfel] {
        var s: [Surfel] = []
        for i in 0..<20 { for j in 0..<20 {
            let y = Float(i) * 0.12, z = Float(j) * 0.12
            let jitterN = SIMD3<Float>(1, .random(in: -0.08...0.08), .random(in: -0.08...0.08))
            s.append(surfel(SIMD3<Float>(2 + .random(in: -0.02...0.02), y, z), jitterN))
        }}
        for i in 0..<20 { for j in 0..<20 {
            let x = Float(i) * 0.12, z = Float(j) * 0.12
            let jitterN = SIMD3<Float>(.random(in: -0.08...0.08), 1, .random(in: -0.08...0.08))
            s.append(surfel(SIMD3<Float>(x, 0 + .random(in: -0.02...0.02), z), jitterN))
        }}
        return s
    }

    func testDetectsTheWallAndFloor() {
        let planes = PlaneQuantizer.detect(room())
        XCTAssertGreaterThanOrEqual(planes.count, 2)
        XCTAssertTrue(planes.contains { abs(abs($0.normal.x) - 1) < 0.05 && abs($0.offset - 2) < 0.06 }, "vertical wall at x≈2")
        XCTAssertTrue(planes.contains { abs(abs($0.normal.y) - 1) < 0.05 && abs($0.offset) < 0.06 }, "floor at y≈0")
    }

    func testSnapFlattensAndAlignsSurfels() {
        let raw = room()
        let quantized = PlaneQuantizer.quantize(raw)
        XCTAssertEqual(quantized.count, raw.count)

        // Wall surfels all collapse onto one consistent plane near x = 2 —
        // same x, same normal, no jitter.
        let wall = quantized.filter { abs($0.normal.x) > 0.95 }
        XCTAssertGreaterThan(wall.count, 200)
        let raw2 = raw.filter { abs(simd_normalize($0.normal).x) > 0.9 }
        let rawSpread = raw2.map { $0.position.x }.max()! - raw2.map { $0.position.x }.min()!
        let xs = wall.map { $0.position.x }
        let spread = xs.max()! - xs.min()!
        XCTAssertLessThan(spread, rawSpread * 0.5, "wall is flatter than it was")
        XCTAssertLessThan(spread, 0.02)
        XCTAssertEqual(xs.reduce(0, +) / Float(xs.count), 2, accuracy: 0.05)
        let n0 = wall[0].normal
        for s in wall { XCTAssertGreaterThan(abs(simd_dot(s.normal, n0)), 0.98, "essentially one shared normal") }
    }

    func testGravityAlignSnapsNearVerticalToHorizontalFacing() {
        let n = PlaneQuantizer.gravityAlign(SIMD3<Float>(0.98, 0.1, 0.1), degrees: 12)
        XCTAssertEqual(n.y, 0, accuracy: 1e-4)
        XCTAssertEqual(simd_length(n), 1, accuracy: 1e-4)
    }

    func testGravityAlignSnapsNearHorizontalToUp() {
        let n = PlaneQuantizer.gravityAlign(SIMD3<Float>(0.05, 0.99, -0.03), degrees: 12)
        XCTAssertEqual(n.x, 0, accuracy: 1e-4)
        XCTAssertEqual(n.y, 1, accuracy: 1e-4)
    }

    func testEmptyAndNoPlanesArePassThrough() {
        XCTAssertTrue(PlaneQuantizer.quantize([]).isEmpty)
        let lone = [surfel(SIMD3<Float>(0, 0, 0), SIMD3<Float>(0.3, 0.4, 0.5), w: 1)]
        XCTAssertEqual(PlaneQuantizer.quantize(lone).count, 1)
    }
}
