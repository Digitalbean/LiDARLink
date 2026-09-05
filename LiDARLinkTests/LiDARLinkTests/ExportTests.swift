import XCTest
import Foundation
import CoreGraphics
import ImageIO
import simd
@testable import LiDARLinkShared

final class ExportTests: XCTestCase {
    private var tempDir: URL!

    override func setUpWithError() throws {
        tempDir = FileManager.default.temporaryDirectory
            .appendingPathComponent("LiDARLinkExportTests-\(UUID().uuidString)", isDirectory: true)
        try FileManager.default.createDirectory(at: tempDir, withIntermediateDirectories: true)
    }

    override func tearDownWithError() throws {
        try? FileManager.default.removeItem(at: tempDir)
    }

    private func points() -> [PointCloudPoint] {
        [
            PointCloudPoint(position: SIMD3<Float>(1, 2, 3), color: SIMD3<UInt8>(255, 0, 0)),
            PointCloudPoint(position: SIMD3<Float>(4, 5, 6), color: SIMD3<UInt8>(0, 255, 0)),
            PointCloudPoint(position: SIMD3<Float>(7, 8, 9), color: SIMD3<UInt8>(0, 0, 255))
        ]
    }

    func testPLYExportContent() throws {
        let url = tempDir.appendingPathComponent("cloud.ply")
        try PLYExporter.export(points: points(), to: url)
        let text = try String(contentsOf: url, encoding: .utf8)
        XCTAssertTrue(text.hasPrefix("ply\nformat ascii 1.0\n"))
        XCTAssertTrue(text.contains("element vertex 3\n"))
        XCTAssertTrue(text.contains("property float x\n"))
        XCTAssertTrue(text.contains("property uchar red\n"))
        XCTAssertTrue(text.contains("end_header\n"))
        XCTAssertTrue(text.contains("1.0 2.0 3.0 255 0 0\n"))
        let bodyLines = text.split(separator: "\n").filter { !$0.hasPrefix("property") && !$0.hasPrefix("element") && !$0.hasPrefix("format") && $0 != "ply" && $0 != "end_header" }
        XCTAssertEqual(bodyLines.count, 3)
    }

    func testPLYExportEmptyThrows() {
        let url = tempDir.appendingPathComponent("empty.ply")
        XCTAssertThrowsError(try PLYExporter.export(points: [], to: url))
    }

    func testOBJExportContent() throws {
        let url = tempDir.appendingPathComponent("mesh.obj")
        try OBJExporter.export(mesh: TestFixtures.mesh(), to: url)
        let text = try String(contentsOf: url, encoding: .utf8)
        XCTAssertTrue(text.contains("v 0.0 0.0 0.0\n"))
        XCTAssertTrue(text.contains("vn 0.0 0.0 1.0\n"))
        XCTAssertTrue(text.contains("f 1//1 2//2 3//3\n"))
    }

    func testUSDTriangleExportSucceeds() throws {
        let url = tempDir.appendingPathComponent("mesh.usd")
        do {
            try USDZExporter.export(mesh: TestFixtures.mesh(), to: url)
        } catch {
            XCTFail("USD export failed: \(error)")
        }
        XCTAssertTrue(FileManager.default.fileExists(atPath: url.path))
    }

    func testMergedPointCloudCapsSize() {
        let frames = (0..<20).map { TestFixtures.frame(sequence: UInt32($0)) }
        let cloud = ScanExporter.mergedPointCloud(frames: frames, maxPoints: 500)
        XCTAssertLessThanOrEqual(cloud.count, 500)
        XCTAssertGreaterThan(cloud.count, 0)
    }

    private func makeColorPayload(width: Int, height: Int, buffer: [SIMD3<UInt8>]) -> ColorPayload {
        var rgba = [UInt8](repeating: 0, count: width * height * 4)
        for i in 0..<min(buffer.count, width * height) {
            let c = buffer[i]
            rgba[i * 4] = c.x
            rgba[i * 4 + 1] = c.y
            rgba[i * 4 + 2] = c.z
            rgba[i * 4 + 3] = 255
        }
        let context = CGContext(data: &rgba, width: width, height: height,
                                bitsPerComponent: 8, bytesPerRow: width * 4,
                                space: CGColorSpaceCreateDeviceRGB(),
                                bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue)!
        let image = context.makeImage()!
        let data = NSMutableData()
        let destination = CGImageDestinationCreateWithData(data, "public.jpeg" as CFString, 1, nil)!
        CGImageDestinationAddImage(destination, image, [kCGImageDestinationLossyCompressionQuality: 1.0] as CFDictionary)
        CGImageDestinationFinalize(destination)
        return ColorPayload(width: width, height: height, jpegData: data as Data)
    }

    func testPointCloudColorSamplingScalesDepthToColorPixels() throws {
        // 4x4 color, all blue except top-row pixel (2, 0), which is red.
        // Depth and color rows must retain the same orientation.
        var buffer = [SIMD3<UInt8>](repeating: SIMD3<UInt8>(0, 0, 255), count: 16)
        buffer[2] = SIMD3<UInt8>(255, 0, 0)
        let color = makeColorPayload(width: 4, height: 4, buffer: buffer)

        let depth = DepthPayload(width: 2, height: 2,
                                 depthData: Binary.data(from: [Float16](repeating: 2, count: 4)),
                                 confidenceData: nil)
        let frame = ScanFrame(sequence: 1, captureTimestampMs: 1,
                              pose: matrix_identity_float4x4,
                              intrinsics: CameraIntrinsics(fx: 10, fy: 10, cx: 1, cy: 1, imageWidth: 2, imageHeight: 2),
                              depth: depth, color: color,
                              depthIsSmoothed: false, depthScale: 1)
        let cloud = ScanExporter.pointCloud(from: frame, step: 1)
        XCTAssertEqual(cloud.count, 4)
        // Pixel order: (0,0), (1,0), (0,1), (1,1). Depth pixel (1,0) → color (2,0) → red.
        XCTAssertGreaterThan(cloud[1].color.x, 200, "depth pixel (1,0) should sample red")
        XCTAssertLessThan(cloud[0].color.x, 100, "depth pixel (0,0) should sample blue")
    }

    func testScanExporterPointCloudFromFrame() {
        let frame = TestFixtures.frame(sequence: 1)
        let cloud = ScanExporter.pointCloud(from: frame, step: 1)
        XCTAssertEqual(cloud.count, 16) // 4x4 depth
        // Pixel (0,0), depth 0.1: image Y is inverted into camera-space +Y.
        // then translated by the pose translation (1, 2, 3).
        XCTAssertEqual(Double(cloud.first?.position.x ?? 0), -0.064 + 1, accuracy: 0.01)
        XCTAssertEqual(Double(cloud.first?.position.y ?? 0), 0.048 + 2, accuracy: 0.01)
        XCTAssertEqual(Double(cloud.first?.position.z ?? 0), -0.1 + 3, accuracy: 0.01)
    }
}
