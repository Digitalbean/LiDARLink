import Foundation
import simd

/// Reassembles a mesh update from its chunks. Chunks may arrive out of order;
/// duplicates are ignored. Thread-safe.
public final class MeshChunkAssembler: @unchecked Sendable {
    private struct Assembly {
        var chunkCount: Int
        var chunks: [Data?]
    }

    private let lock = NSLock()
    private var assemblies: [UUID: Assembly] = [:]
    public private(set) var completedMeshCount = 0
    public private(set) var duplicateChunks = 0

    public init() {}

    /// Applies one chunk. Returns the assembled mesh once all chunks are present.
    public func apply(metadata: MeshMetadata, chunkData: Data) -> MeshData? {
        lock.lock()
        defer { lock.unlock() }
        guard metadata.chunkCount > 0 else { return nil }

        var assembly = assemblies[metadata.anchorID]
        if assembly == nil || assembly!.chunkCount != metadata.chunkCount {
            assembly = Assembly(chunkCount: metadata.chunkCount, chunks: Array(repeating: nil, count: metadata.chunkCount))
        }
        guard var current = assembly,
              metadata.chunkIndex >= 0, metadata.chunkIndex < current.chunkCount else { return nil }

        if current.chunks[metadata.chunkIndex] != nil { duplicateChunks += 1 }
        current.chunks[metadata.chunkIndex] = chunkData
        assemblies[metadata.anchorID] = current

        guard current.chunks.allSatisfy({ $0 != nil }) else { return nil }

        // Concatenate chunks in order; each chunk is self-describing.
        var vertices: [SIMD3<Float>] = []
        var normals: [SIMD3<Float>] = []
        var faces: [UInt32] = []
        for chunk in current.chunks {
            guard let chunk, let unpacked = MeshChunk.unpack(chunk) else { continue }
            vertices.append(contentsOf: unpacked.vertices)
            normals.append(contentsOf: unpacked.normals)
            faces.append(contentsOf: unpacked.faces)
        }
        assemblies.removeValue(forKey: metadata.anchorID)
        completedMeshCount += 1
        let sanitized = MeshSanitizer.sanitize(vertices: vertices, normals: normals, faces: faces)
        return MeshData(anchorID: metadata.anchorID,
                        vertices: sanitized.vertices,
                        normals: sanitized.normals,
                        faces: sanitized.faces,
                        transform: metadata.transform ?? matrix_identity_float4x4,
                        updatedAtMs: metadata.timestampMs)
    }

    /// Number of chunks still missing for an anchor (0 if unknown or complete).
    public func incompleteChunkCount(for anchorID: UUID) -> Int {
        lock.lock()
        defer { lock.unlock() }
        guard let assembly = assemblies[anchorID] else { return 0 }
        return assembly.chunks.filter { $0 == nil }.count
    }

    /// Drops any in-progress assembly for anchors that were removed.
    public func forget(anchorIDs: [UUID]) {
        lock.lock()
        defer { lock.unlock() }
        for id in anchorIDs { assemblies.removeValue(forKey: id) }
    }

    public func reset() {
        lock.lock()
        defer { lock.unlock() }
        assemblies.removeAll()
        completedMeshCount = 0
        duplicateChunks = 0
    }
}
