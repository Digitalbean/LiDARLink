import XCTest
import simd
@testable import LiDARLinkShared

final class RoomStructureCodecTests: XCTestCase {

    private func sample() -> RoomStructure {
        var wallT = matrix_identity_float4x4
        wallT.columns.3 = SIMD4<Float>(1, 1.2, -2, 1)
        var doorT = matrix_identity_float4x4
        doorT.columns.3 = SIMD4<Float>(0.5, 1.0, -2, 1)
        return RoomStructure(
            surfaces: [
                .init(id: "w1", category: .wall, transform: wallT,
                      dimensions: SIMD3<Float>(3.2, 2.4, 0.1), confidence: 1.0),
                .init(id: "d1", category: .door, transform: doorT,
                      dimensions: SIMD3<Float>(0.9, 2.0, 0.1), confidence: 0.6),
            ],
            objects: [
                .init(id: "o1", category: .table, transform: matrix_identity_float4x4,
                      dimensions: SIMD3<Float>(1.4, 0.75, 0.8), confidence: 0.6),
            ],
            capturedAtMs: 123_456, isFinal: false)
    }

    func testRoundTripsThroughTheWireCodec() throws {
        let original = sample()
        let encoded = try MessageCodec.encode(.roomStructure(original), sequence: 7, timestampMs: 999)
        let decoder = MessageDecoder()
        let items = decoder.append(encoded)
        XCTAssertEqual(items.count, 1)
        guard case .message(.roomStructure(let decoded), _) = items[0] else {
            return XCTFail("expected a roomStructure message, got \(items)")
        }
        XCTAssertEqual(decoded, original)
        XCTAssertEqual(decoded.walls.count, 1)
        XCTAssertEqual(decoded.openings.count, 1)
        XCTAssertEqual(decoded.objects.first?.category, .table)
    }

    func testSurfacePlaneMatchesTheTransformNormal() {
        // A wall facing +x at x = 2.
        var t = matrix_identity_float4x4
        t.columns.0 = SIMD4<Float>(0, 0, -1, 0)   // local x
        t.columns.2 = SIMD4<Float>(1, 0, 0, 0)    // local z (= normal) → +x
        t.columns.3 = SIMD4<Float>(2, 1, 0, 1)
        let s = RoomStructure.Surface(id: "w", category: .wall, transform: t,
                                      dimensions: SIMD3<Float>(2, 2, 0.1), confidence: 1)
        let plane = s.plane
        XCTAssertEqual(plane.normal.x, 1, accuracy: 1e-4)
        XCTAssertEqual(plane.offset, 2, accuracy: 1e-4)
    }
}
