import XCTest
import simd
@testable import LiDARLinkShared

final class SurfelDelightTests: XCTestCase {

    private let k = CameraIntrinsics(fx: 100, fy: 100, cx: 48, cy: 36, imageWidth: 96, imageHeight: 72)

    /// A uniform-albedo wall captured with a bright hotspot on one side. After
    /// de-lighting, the albedo should be far more even than the raw colour.
    func testFlattensAHotspot() {
        let map = SurfelMap()
        let flat = [Float16](repeating: 1.5, count: 96 * 72)
        map.integrate(depth: flat, width: 96, height: 72, depthScale: 1, intrinsics: k,
                      pose: matrix_identity_float4x4, confidence: nil, keyframeID: 0,
                      colorFor: { x, _ in
                          // Left third bright (hotspot), rest mid-grey.
                          let v: UInt8 = x < 32 ? 235 : 120
                          return SIMD3<UInt8>(v, v, v)
                      })

        func spread(_ pick: (Surfel) -> Float) -> Float {
            let vals = map.orientedSnapshot().map(pick)
            let mean = vals.reduce(0, +) / Float(vals.count)
            let variance = vals.map { ($0 - mean) * ($0 - mean) }.reduce(0, +) / Float(vals.count)
            return sqrt(variance)
        }

        let rawSpread = spread { 0.299 * $0.color.x + 0.587 * $0.color.y + 0.114 * $0.color.z }
        map.updateAmbientOcclusion()
        map.updateDelighting()
        let albedoSpread = spread { 0.299 * $0.albedo.x + 0.587 * $0.albedo.y + 0.114 * $0.albedo.z }

        XCTAssertGreaterThan(rawSpread, 30, "the raw capture has a strong bright/dark split")
        XCTAssertLessThan(albedoSpread, rawSpread * 0.6, "de-lighting evens it out")
    }

    func testKeepsOverallColourLevel() {
        let map = SurfelMap()
        let flat = [Float16](repeating: 1.4, count: 96 * 72)
        map.integrate(depth: flat, width: 96, height: 72, depthScale: 1, intrinsics: k,
                      pose: matrix_identity_float4x4, confidence: nil, keyframeID: 0,
                      colorFor: { _, _ in SIMD3<UInt8>(160, 130, 90) })
        map.updateAmbientOcclusion()
        map.updateDelighting()
        let a = map.orientedSnapshot().map(\.albedo)
        let mean = a.reduce(SIMD3<Float>.zero, +) / Float(a.count)
        // Uniform input → albedo ≈ the input colour, not blown out or crushed.
        XCTAssertEqual(mean.x, 160, accuracy: 40)
        XCTAssertEqual(mean.y, 130, accuracy: 40)
        XCTAssertEqual(mean.z, 90, accuracy: 40)
    }
}
