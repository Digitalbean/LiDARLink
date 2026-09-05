import XCTest
import simd
@testable import LiDARLinkShared

final class RichPointCloudTests: XCTestCase {

    private func surfel(_ p: SIMD3<Float>, _ n: SIMD3<Float>, w: Float = 10) -> Surfel {
        Surfel(position: p, normal: simd_normalize(n), color: SIMD3<Float>(180, 150, 120),
               radius: 0.03, weight: w, owner: 0, lastSeen: 0, bestViewCos: 0.9)
    }

    func testClassifiesFloorWallCeiling() {
        let surfels = [
            surfel(SIMD3<Float>(0, 0, 0), SIMD3<Float>(0, 1, 0)),      // floor
            surfel(SIMD3<Float>(0, 2.4, 0), SIMD3<Float>(0, -1, 0)),   // ceiling
            surfel(SIMD3<Float>(1.5, 1.2, 0), SIMD3<Float>(-1, 0, 0)), // wall
            surfel(SIMD3<Float>(0.3, 0.5, 0.3), SIMD3<Float>(0.3, 0.6, 0.7)), // clutter
        ]
        let cloud = PointCloudClassifier.classify(surfels: surfels)
        let classes = cloud.points.map(\.classification)
        XCTAssertEqual(classes[0], .floor)
        XCTAssertEqual(classes[1], .ceiling)
        XCTAssertEqual(classes[2], .wall)
        XCTAssertEqual(classes[3], .clutter)
    }

    func testLASHeaderIsWellFormed() {
        let cloud = PointCloudClassifier.classify(surfels: [
            surfel(SIMD3<Float>(1, 0.5, -2), SIMD3<Float>(0, 1, 0)),
            surfel(SIMD3<Float>(-1, 0.5, -3), SIMD3<Float>(0, 1, 0)),
        ])
        let data = LASExporter.data(from: cloud)

        XCTAssertEqual(Array(data.prefix(4)), Array("LASF".utf8))
        // version 1.2 at bytes 24, 25
        XCTAssertEqual(data[24], 1)
        XCTAssertEqual(data[25], 2)
        // point data record format 2, length 26 (offsets 104, 105..106)
        XCTAssertEqual(data[104], 2)
        let recLen = UInt16(data[105]) | (UInt16(data[106]) << 8)
        XCTAssertEqual(recLen, 26)
        // point count at 107..110
        let count = data.withUnsafeBytes { $0.loadUnaligned(fromByteOffset: 107, as: UInt32.self) }
        XCTAssertEqual(count, 2)
        XCTAssertEqual(data.count, 227 + 2 * 26)
    }

    func testRichPLYRoundTripsClassificationAndConfidence() {
        let cloud = RichPointCloud(points: [
            .init(position: SIMD3<Float>(0.1, 0.2, 0.3), normal: SIMD3<Float>(0, 0, 1),
                  color: SIMD3<UInt8>(10, 20, 30), confidence: 0.75, classification: .window),
        ])
        let data = RichPLYExporter.data(from: cloud)
        let text = String(decoding: data.prefix(400), as: UTF8.self)
        XCTAssertTrue(text.contains("property uchar classification"))
        XCTAssertTrue(text.contains("property float confidence"))
        XCTAssertTrue(text.contains("format binary_little_endian"))

        // Last record: 6 floats (24) + 3 bytes RGB + 1 byte class + 1 float conf = 32 bytes.
        let record = Array(data.suffix(32))
        XCTAssertEqual(record[27], RichPointCloud.Classification.window.rawValue)
        let conf = record[28...31].withUnsafeBytes { $0.loadUnaligned(as: Float.self) }
        XCTAssertEqual(conf, 0.75, accuracy: 1e-6)
    }
}
