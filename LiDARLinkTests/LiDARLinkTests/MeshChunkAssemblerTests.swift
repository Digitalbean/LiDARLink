import XCTest
import simd
@testable import LiDARLinkShared

final class MeshChunkAssemblerTests: XCTestCase {
    func testAssemblesChunksArrivingOutOfOrder() throws {
        let assembler = MeshChunkAssembler()
        let anchorID = UUID()
        let chunk0 = MeshChunk.pack(vertices: [SIMD3<Float>(0, 0, 0), SIMD3<Float>(1, 0, 0)],
                                    normals: [SIMD3<Float>(0, 0, 1), SIMD3<Float>(0, 0, 1)],
                                    faces: [0, 1, 2])
        let chunk1 = MeshChunk.pack(vertices: [SIMD3<Float>(0, 1, 0), SIMD3<Float>(1, 1, 0)],
                                    normals: [SIMD3<Float>(0, 0, 1), SIMD3<Float>(0, 0, 1)],
                                    faces: [1, 3, 2])
        let meta1 = MeshMetadata(anchorID: anchorID, chunkIndex: 1, chunkCount: 2, timestampMs: 50)
        let meta0 = MeshMetadata(anchorID: anchorID, chunkIndex: 0, chunkCount: 2, timestampMs: 50)

        XCTAssertNil(assembler.apply(metadata: meta1, chunkData: chunk1))
        let mesh = assembler.apply(metadata: meta0, chunkData: chunk0)
        XCTAssertNotNil(mesh)
        XCTAssertEqual(mesh?.vertices.count, 4)
        XCTAssertEqual(mesh?.faces, [0, 1, 2, 1, 3, 2])
        XCTAssertEqual(mesh?.anchorID, anchorID)
        XCTAssertEqual(assembler.completedMeshCount, 1)
    }

    func testForgetDropsInProgressAssemblyForRemovedAnchor() {
        let assembler = MeshChunkAssembler()
        let anchorID = UUID()
        let chunk0 = MeshChunk.pack(vertices: [SIMD3<Float>(0, 0, 0), SIMD3<Float>(1, 0, 0)],
                                    normals: [SIMD3<Float>(0, 0, 1), SIMD3<Float>(0, 0, 1)],
                                    faces: [0, 1, 2])
        let meta0 = MeshMetadata(anchorID: anchorID, chunkIndex: 0, chunkCount: 2, timestampMs: 1)
        XCTAssertNil(assembler.apply(metadata: meta0, chunkData: chunk0))
        XCTAssertEqual(assembler.incompleteChunkCount(for: anchorID), 1)

        assembler.forget(anchorIDs: [anchorID])
        XCTAssertEqual(assembler.incompleteChunkCount(for: anchorID), 0)
    }

    func testMergedBakesTransformsAndRebasesFaces() {
        let a = MeshData(anchorID: UUID(),
                         vertices: [SIMD3<Float>(0, 0, 0), SIMD3<Float>(1, 0, 0), SIMD3<Float>(0, 1, 0)],
                         normals: [SIMD3<Float>(0, 0, 1), SIMD3<Float>(0, 0, 1), SIMD3<Float>(0, 0, 1)],
                         faces: [0, 1, 2],
                         transform: matrix_identity_float4x4,
                         updatedAtMs: 10)
        var shifted = matrix_identity_float4x4
        shifted.columns.3 = SIMD4<Float>(5, 0, 0, 1)
        let b = MeshData(anchorID: UUID(),
                         vertices: [SIMD3<Float>(0, 0, 0), SIMD3<Float>(1, 0, 0), SIMD3<Float>(0, 1, 0)],
                         normals: [SIMD3<Float>(0, 0, 1), SIMD3<Float>(0, 0, 1), SIMD3<Float>(0, 0, 1)],
                         faces: [0, 1, 2],
                         transform: shifted,
                         updatedAtMs: 20)

        let merged = MeshData.merged([a, b])
        XCTAssertEqual(merged?.vertices.count, 6)
        XCTAssertEqual(merged?.faces, [0, 1, 2, 3, 4, 5])       // b's faces re-based by 3
        XCTAssertEqual(merged?.vertices[3].x ?? 0, 5, accuracy: 0.001) // b shifted +5 in X
        XCTAssertEqual(merged?.updatedAtMs, 20)
    }

    func testMergedNilForEmpty() {
        XCTAssertNil(MeshData.merged([]))
    }

    func testDuplicateChunksAreCounted() {
        let assembler = MeshChunkAssembler()
        let anchorID = UUID()
        let chunk0 = MeshChunk.pack(vertices: [SIMD3<Float>(0, 0, 0)],
                                    normals: [SIMD3<Float>(0, 0, 1)],
                                    faces: [0, 0, 0])
        let chunk1 = MeshChunk.pack(vertices: [SIMD3<Float>(1, 0, 0)],
                                    normals: [SIMD3<Float>(0, 0, 1)],
                                    faces: [0, 0, 0])
        let meta0 = MeshMetadata(anchorID: anchorID, chunkIndex: 0, chunkCount: 2, timestampMs: 10)
        let meta1 = MeshMetadata(anchorID: anchorID, chunkIndex: 1, chunkCount: 2, timestampMs: 10)
        XCTAssertNil(assembler.apply(metadata: meta0, chunkData: chunk0))
        // Duplicate of chunk 0 is ignored and counted.
        XCTAssertNil(assembler.apply(metadata: meta0, chunkData: chunk0))
        XCTAssertEqual(assembler.duplicateChunks, 1)
        XCTAssertNotNil(assembler.apply(metadata: meta1, chunkData: chunk1))
    }

    func testMissingChunkNeverCompletes() {
        let assembler = MeshChunkAssembler()
        let anchorID = UUID()
        let chunk = MeshChunk.pack(vertices: [SIMD3<Float>(0, 0, 0)],
                                   normals: [SIMD3<Float>(0, 0, 1)],
                                   faces: [0, 0, 0])
        let meta = MeshMetadata(anchorID: anchorID, chunkIndex: 0, chunkCount: 3, timestampMs: 10)
        XCTAssertNil(assembler.apply(metadata: meta, chunkData: chunk))
        XCTAssertEqual(assembler.incompleteChunkCount(for: anchorID), 2)
    }
}

// MARK: - Mesh streaming hardening

extension MeshChunkAssemblerTests {
    func testTransformPassthrough() {
        let assembler = MeshChunkAssembler()
        var transform = matrix_identity_float4x4
        transform.columns.3 = simd_float4(1, 2, 3, 1)
        let anchorID = UUID()
        let chunk = MeshChunk.pack(vertices: [SIMD3<Float>(0, 0, 0), SIMD3<Float>(1, 0, 0), SIMD3<Float>(0, 1, 0)],
                                   normals: [SIMD3<Float>(0, 0, 1), SIMD3<Float>(0, 0, 1), SIMD3<Float>(0, 0, 1)],
                                   faces: [0, 1, 2])
        let meta = MeshMetadata(anchorID: anchorID, chunkIndex: 0, chunkCount: 1, timestampMs: 5, transform: transform)
        let mesh = assembler.apply(metadata: meta, chunkData: chunk)
        XCTAssertEqual(mesh?.transform, transform)
    }

    func testOutOfRangeFacesDropped() {
        let assembler = MeshChunkAssembler()
        let anchorID = UUID()
        // Face references vertex 5 which does not exist in the 3-vertex mesh.
        let chunk = MeshChunk.pack(vertices: [SIMD3<Float>(0, 0, 0), SIMD3<Float>(1, 0, 0), SIMD3<Float>(0, 1, 0)],
                                   normals: [SIMD3<Float>(0, 0, 1), SIMD3<Float>(0, 0, 1), SIMD3<Float>(0, 0, 1)],
                                   faces: [0, 1, 2, 0, 1, 5])
        let meta = MeshMetadata(anchorID: anchorID, chunkIndex: 0, chunkCount: 1, timestampMs: 5)
        let mesh = assembler.apply(metadata: meta, chunkData: chunk)
        XCTAssertEqual(mesh?.faces, [0, 1, 2], "out-of-range triangle must be dropped")
    }

    func testSanitizerDropsDegenerateAndPadsNormals() {
        let result = MeshSanitizer.sanitize(vertices: [SIMD3<Float>(0, 0, 0), SIMD3<Float>(1, 0, 0), SIMD3<Float>(0, 1, 0)],
                                            normals: [SIMD3<Float>(0, 0, 1)],
                                            faces: [0, 1, 2, 0, 0, 1])
        XCTAssertEqual(result.faces, [0, 1, 2], "degenerate triangle dropped")
        XCTAssertEqual(result.normals.count, 3, "normals padded to vertex count")
    }
}
