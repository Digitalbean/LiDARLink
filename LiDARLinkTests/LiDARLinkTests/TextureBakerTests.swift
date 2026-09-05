import XCTest
import CoreGraphics
import ImageIO
import UniformTypeIdentifiers
import simd
@testable import LiDARLinkShared

final class TextureBakerTests: XCTestCase {

    private func sampler(width: Int, height: Int, colour: SIMD3<UInt8>) -> ColorImageSampler {
        var rgba = [UInt8](repeating: 255, count: width * height * 4)
        for i in 0..<(width * height) {
            rgba[i * 4] = colour.x; rgba[i * 4 + 1] = colour.y; rgba[i * 4 + 2] = colour.z
        }
        let ctx = CGContext(data: &rgba, width: width, height: height, bitsPerComponent: 8,
                            bytesPerRow: width * 4, space: CGColorSpaceCreateDeviceRGB(),
                            bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue)!
        let image = ctx.makeImage()!
        let data = NSMutableData()
        let dest = CGImageDestinationCreateWithData(data, UTType.jpeg.identifier as CFString, 1, nil)!
        CGImageDestinationAddImage(dest, image, [kCGImageDestinationLossyCompressionQuality: 0.95] as CFDictionary)
        CGImageDestinationFinalize(dest)
        return ColorImageSampler(color: ColorPayload(width: width, height: height, jpegData: data as Data))!
    }

    /// A quad at z = -1.5 facing +z (toward a camera at the origin).
    private func quad() -> MarchingCubes.Mesh {
        var m = MarchingCubes.Mesh()
        m.vertices = [SIMD3(-0.3, -0.3, -1.5), SIMD3(0.3, -0.3, -1.5), SIMD3(0.3, 0.3, -1.5), SIMD3(-0.3, 0.3, -1.5)]
        m.normals = Array(repeating: SIMD3<Float>(0, 0, 1), count: 4)
        m.colours = Array(repeating: SIMD3<Float>(0, 0, 0), count: 4)
        m.faces = [0, 1, 2, 0, 2, 3]
        return m
    }

    func testBakesTextureFromAView() {
        let view = TextureBaker.View(
            sampler: sampler(width: 128, height: 128, colour: SIMD3<UInt8>(220, 30, 30)),
            pose: matrix_identity_float4x4,
            intrinsics: CameraIntrinsics(fx: 100, fy: 100, cx: 64, cy: 64, imageWidth: 128, imageHeight: 128))

        let baked = TextureBaker.bake(mesh: quad(), views: [view], atlasSize: 256)

        XCTAssertEqual(baked.faces.count, 6)
        XCTAssertEqual(baked.vertices.count, 6, "vertices split per triangle for unique UVs")
        XCTAssertEqual(baked.uvs.count, 6)
        XCTAssertEqual(baked.atlasRGBA.count, 256 * 256 * 4)
        XCTAssertGreaterThan(baked.texturedTriangleFraction, 0.9)
        XCTAssertTrue(baked.uvs.allSatisfy { $0.x >= 0 && $0.x <= 1 && $0.y >= 0 && $0.y <= 1 })

        var redTexels = 0
        for i in stride(from: 0, to: baked.atlasRGBA.count, by: 4)
        where baked.atlasRGBA[i] > 150 && baked.atlasRGBA[i + 1] < 110 && baked.atlasRGBA[i + 3] == 255 {
            redTexels += 1
        }
        XCTAssertGreaterThan(redTexels, 80, "atlas should carry the view's colour")
    }

    func testTriangleFacingAwayGetsNoView() {
        var mesh = quad()
        mesh.normals = Array(repeating: SIMD3<Float>(0, 0, -1), count: 4)   // faces away from the camera
        let view = TextureBaker.View(
            sampler: sampler(width: 64, height: 64, colour: SIMD3<UInt8>(10, 200, 10)),
            pose: matrix_identity_float4x4,
            intrinsics: CameraIntrinsics(fx: 60, fy: 60, cx: 32, cy: 32, imageWidth: 64, imageHeight: 64))
        let baked = TextureBaker.bake(mesh: mesh, views: [view], atlasSize: 128)
        XCTAssertEqual(baked.texturedTriangleFraction, 0, accuracy: 0.001)
        XCTAssertEqual(baked.faces.count, 6, "still meshed, just untextured")
    }

    func testEmptyInputs() {
        XCTAssertEqual(TextureBaker.bake(mesh: MarchingCubes.Mesh(), views: []).faces.count, 0)
        XCTAssertEqual(TextureBaker.bake(mesh: quad(), views: []).faces.count, 0)
    }

    /// A dense wall (~5000 triangles) into a modest atlas: the per-triangle
    /// charts must not collapse to zero-size cells — the regression that turned
    /// the atlas into 1px slivers.
    func testDenseMeshDoesNotCollapseCharts() {
        var m = MarchingCubes.Mesh()
        let n = 50
        for j in 0...n { for i in 0...n {
            let x = Float(i) / Float(n) * 1.2 - 0.6
            let y = Float(j) / Float(n) * 1.2 - 0.6
            m.vertices.append(SIMD3<Float>(x, y, -1.5))
            m.normals.append(SIMD3<Float>(0, 0, 1))
            m.colours.append(.zero)
        }}
        let row = n + 1
        for j in 0..<n { for i in 0..<n {
            let a = UInt32(j * row + i)
            m.faces.append(contentsOf: [a, a + 1, a + UInt32(row), a + 1, a + 1 + UInt32(row), a + UInt32(row)])
        }}
        XCTAssertGreaterThan(m.faces.count / 3, 4000)

        let view = TextureBaker.View(
            sampler: sampler(width: 256, height: 256, colour: SIMD3<UInt8>(40, 90, 210)),
            pose: matrix_identity_float4x4,
            intrinsics: CameraIntrinsics(fx: 200, fy: 200, cx: 128, cy: 128, imageWidth: 256, imageHeight: 256))
        let baked = TextureBaker.bake(mesh: m, views: [view], atlasSize: 1024)

        XCTAssertEqual(baked.faces.count, m.faces.count, "every triangle placed")
        XCTAssertGreaterThan(baked.texturedTriangleFraction, 0.8)
        var blueTexels = 0
        for i in stride(from: 0, to: baked.atlasRGBA.count, by: 4)
        where baked.atlasRGBA[i + 2] > 150 && baked.atlasRGBA[i] < 100 && baked.atlasRGBA[i + 3] == 255 {
            blueTexels += 1
        }
        XCTAssertGreaterThan(blueTexels, 2000, "atlas carries real coverage, not slivers")
    }
}
