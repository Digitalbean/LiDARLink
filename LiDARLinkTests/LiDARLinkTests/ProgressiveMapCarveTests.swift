import XCTest
import simd
@testable import LiDARLinkShared

final class ProgressiveMapCarveTests: XCTestCase {

    /// A wall of points at z = 3, seen from the origin.
    private func wall(z: Float = 3, confidence: UInt8 = 2) -> [PointCloudPoint] {
        var pts: [PointCloudPoint] = []
        for gx in stride(from: Float(0.0), through: 1.0, by: 0.02) {
            for gy in stride(from: Float(0.0), through: 1.0, by: 0.02) {
                pts.append(PointCloudPoint(position: SIMD3<Float>(gx, gy, z),
                                           color: SIMD3<UInt8>(200, 200, 200),
                                           confidence: confidence))
            }
        }
        return pts
    }

    func testCarvesAFloaterButKeepsTheSurface() {
        let map = ProgressiveMap(cellSize: 0.02)
        _ = map.deduplicate(wall())
        // A stray point floating in mid-air between camera and wall.
        _ = map.deduplicate([PointCloudPoint(position: SIMD3<Float>(0.5, 0.5, 1.5),
                                             color: SIMD3<UInt8>(255, 0, 0), confidence: 2)])
        let wallCellsBefore = map.occupiedCellCount

        let origin = SIMD3<Float>(0.5, 0.5, 0)
        for _ in 0..<5 { map.carveFreeSpace(from: origin, towards: wall()) }

        let floaterKey = ProgressiveMap.packCell(position: SIMD3<Float>(0.5, 0.5, 1.5), invCellSize: 1 / 0.02)
        let remaining = map.mergedPoints()
        XCTAssertFalse(remaining.contains { ProgressiveMap.packCell(position: $0.position, invCellSize: 1 / 0.02) == floaterKey },
                       "the floating point was carved away")
        XCTAssertGreaterThan(map.occupiedCellCount, Int(Double(wallCellsBefore) * 0.95),
                             "the wall surface is essentially untouched")
    }

    func testEstablishedGeometryIsImmuneToStrayRays() {
        // A cell observed many times (a real thin wall) sits on the see-through
        // path to the far wall. A dozen carve passes must not touch it.
        let map = ProgressiveMap(cellSize: 0.02)
        _ = map.deduplicate(wall())
        let thinWall = SIMD3<Float>(0.5, 0.5, 1.5)
        for _ in 0..<20 {
            _ = map.deduplicate([PointCloudPoint(position: thinWall, color: .init(repeating: 180), confidence: 2)])
        }
        let key = ProgressiveMap.packCell(position: thinWall, invCellSize: 1 / 0.02)

        for _ in 0..<12 { map.carveFreeSpace(from: SIMD3<Float>(0, 0, 0), towards: wall()) }

        XCTAssertTrue(map.mergedPoints().contains { ProgressiveMap.packCell(position: $0.position, invCellSize: 1 / 0.02) == key },
                      "well-observed geometry is not eroded by a see-through ray")
    }

    func testDoesNotCarveAPointJustInFrontOfTheSurface() {
        let map = ProgressiveMap(cellSize: 0.02)
        _ = map.deduplicate(wall())
        // 4 cm in front of the wall — inside the 10 cm margin, must survive.
        let near = SIMD3<Float>(0.5, 0.5, 2.96)
        _ = map.deduplicate([PointCloudPoint(position: near, color: SIMD3<UInt8>(0, 255, 0), confidence: 2)])
        let nearKey = ProgressiveMap.packCell(position: near, invCellSize: 1 / 0.02)

        for _ in 0..<8 { map.carveFreeSpace(from: SIMD3<Float>(0.5, 0.5, 0), towards: wall()) }

        XCTAssertTrue(map.mergedPoints().contains { ProgressiveMap.packCell(position: $0.position, invCellSize: 1 / 0.02) == nearKey },
                      "a point within the margin of the surface is not carved")
    }

    func testLowConfidenceRaysDoNotCarve() {
        let map = ProgressiveMap(cellSize: 0.02)
        _ = map.deduplicate(wall())
        _ = map.deduplicate([PointCloudPoint(position: SIMD3<Float>(0.5, 0.5, 1.5),
                                             color: SIMD3<UInt8>(255, 0, 0), confidence: 2)])
        let before = map.occupiedCellCount

        for _ in 0..<5 {
            map.carveFreeSpace(from: SIMD3<Float>(0.5, 0.5, 0), towards: wall(confidence: 0))
        }
        XCTAssertEqual(map.occupiedCellCount, before, "confidence-0 observations carve nothing")
    }

    func testCarvedCellRecoversWhenReobserved() {
        // A cell that is legitimately re-measured every frame should not vanish
        // even if a stray ray clips it — its weight is topped back up.
        let map = ProgressiveMap(cellSize: 0.02)
        _ = map.deduplicate(wall())
        let target = SIMD3<Float>(0.5, 0.5, 2.0)
        _ = map.deduplicate([PointCloudPoint(position: target, color: .init(repeating: 100), confidence: 2)])
        let key = ProgressiveMap.packCell(position: target, invCellSize: 1 / 0.02)

        for _ in 0..<6 {
            map.carveFreeSpace(from: SIMD3<Float>(0.0, 0.0, 0), towards: wall())
            _ = map.deduplicate([PointCloudPoint(position: target, color: .init(repeating: 100), confidence: 2)])
        }
        let survivor = map.mergedPoints().first {
            ProgressiveMap.packCell(position: $0.position, invCellSize: 1 / 0.02) == key
        }
        XCTAssertNotNil(survivor, "a continuously re-observed cell survives incidental carving")
        // And its position is intact — decay must not drag the centroid.
        if let survivor {
            XCTAssertLessThan(simd_length(survivor.position - target), 0.02)
        }
    }
}
