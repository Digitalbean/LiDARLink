import ARKit
import Foundation
import Metal
import simd
import LiDARLinkShared

/// Converts ARMeshAnchor geometry into wire chunks. Faces are emitted with global
/// vertex indices; the receiver concatenates all chunks in order.
enum MeshExtractor {
    static let chunkVertexCount = 2_000

    static func chunks(from anchors: [ARMeshAnchor],
                       timestampMs: UInt64,
                       maxVerticesPerAnchor: Int = 60_000) -> [MeshChunk] {
        var result: [MeshChunk] = []
        for anchor in anchors {
            let geometry = anchor.geometry
            guard geometry.vertices.count > 0 else { continue }

            // Every read is bounded by BOTH the reported count and the actual
            // MTLBuffer capacity, so a mismatched or mid-update ARKit buffer can
            // never cause an out-of-bounds crash and never reads garbage.
            // ARKit packs mesh vertices and normals as tightly-packed float3
            // (stride 12). `MemoryLayout<SIMD3<Float>>.stride` is 16 — SIMD3
            // carries 4 bytes of padding — so clamping the stride up to it walks
            // the buffer 4 bytes fast per element and scrambles every vertex
            // after the first. Trust ARKit's reported stride; only fall back to
            // the packed size if it reports something unusable.
            let packedFloat3 = 3 * MemoryLayout<Float>.stride
            let vertexBuffer = geometry.vertices.buffer
            let vertexStride = geometry.vertices.stride >= packedFloat3 ? geometry.vertices.stride : packedFloat3
            let vertexBytes = vertexBuffer.length - geometry.vertices.offset
            let vertexCount = min(geometry.vertices.count, maxVerticesPerAnchor, vertexBytes / vertexStride)
            guard vertexCount > 0 else { continue }

            let normalBuffer = geometry.normals.buffer
            let normalStride = geometry.normals.stride >= packedFloat3 ? geometry.normals.stride : packedFloat3
            let normalBytes = normalBuffer.length - geometry.normals.offset
            let normalCount = min(geometry.normals.count, maxVerticesPerAnchor, normalBytes / normalStride)
            guard normalCount > 0 else { continue }

            let vertexRaw = vertexBuffer.contents().advanced(by: geometry.vertices.offset)
            let normalRaw = normalBuffer.contents().advanced(by: geometry.normals.offset)

            var vertices: [SIMD3<Float>] = []
            vertices.reserveCapacity(vertexCount)
            for i in 0..<vertexCount {
                vertices.append(Self.readFloat3(vertexRaw, byteOffset: i * vertexStride))
            }
            var normals: [SIMD3<Float>] = []
            normals.reserveCapacity(normalCount)
            for i in 0..<normalCount {
                normals.append(Self.readFloat3(normalRaw, byteOffset: i * normalStride))
            }

            let faceBuffer = geometry.faces.buffer
            let bytesPerIndex = geometry.faces.bytesPerIndex
            guard bytesPerIndex == 2 || bytesPerIndex == 4 else { continue }
            let faceBytes = faceBuffer.length
            // Reported count is primitives per ARKit, but clamp to what the
            // buffer can actually hold so either semantic is safe.
            let maxTrianglesByBuffer = faceBytes / (3 * bytesPerIndex)
            let triangleCount = min(geometry.faces.count, maxTrianglesByBuffer)
            guard triangleCount > 0 else { continue }
            let faceRaw = faceBuffer.contents()
            var faces: [UInt32] = []
            faces.reserveCapacity(triangleCount * 3)
            for t in 0..<triangleCount {
                let i0 = readIndex(faceRaw, index: t * 3, bytesPerIndex: bytesPerIndex)
                let i1 = readIndex(faceRaw, index: t * 3 + 1, bytesPerIndex: bytesPerIndex)
                let i2 = readIndex(faceRaw, index: t * 3 + 2, bytesPerIndex: bytesPerIndex)
                guard i0 < vertexCount, i1 < vertexCount, i2 < vertexCount else { continue }
                faces.append(i0)
                faces.append(i1)
                faces.append(i2)
            }
            guard !vertices.isEmpty, !faces.isEmpty else { continue }

            let chunkCount = max(1, (vertices.count + chunkVertexCount - 1) / chunkVertexCount)
            let facesPerChunk = max(1, (faces.count + chunkCount - 1) / chunkCount)
            for chunkIndex in 0..<chunkCount {
                let vStart = chunkIndex * chunkVertexCount
                let vEnd = min(vStart + chunkVertexCount, vertices.count)
                // Faces carry global indices and the receiver just concatenates
                // every chunk's face list, so the split needn't line up with the
                // vertex chunks — clamp both ends so a vertex-heavy, face-light
                // anchor can't produce an inverted range.
                let fStart = min(chunkIndex * facesPerChunk, faces.count)
                let fEnd = min(fStart + facesPerChunk, faces.count)
                guard vStart < vEnd else { continue }
                let payload = MeshChunk.pack(vertices: Array(vertices[vStart..<vEnd]),
                                             normals: Array(normals[vStart..<vEnd]),
                                             faces: Array(faces[fStart..<fEnd]))
                let metadata = MeshMetadata(anchorID: anchor.identifier,
                                            chunkIndex: chunkIndex,
                                            chunkCount: chunkCount,
                                            timestampMs: timestampMs,
                                            transform: anchor.transform)
                result.append(MeshChunk(metadata: metadata, data: payload))
            }
        }
        return result
    }

    /// Reads a tightly-packed float3 (12 bytes) as three unaligned Float loads,
    /// so a stride that is not 16-aligned never triggers alignment UB.
    private static func readFloat3(_ pointer: UnsafeMutableRawPointer, byteOffset: Int) -> SIMD3<Float> {
        let base = pointer.advanced(by: byteOffset)
        return SIMD3<Float>(base.loadUnaligned(as: Float.self),
                            base.advanced(by: 4).loadUnaligned(as: Float.self),
                            base.advanced(by: 8).loadUnaligned(as: Float.self))
    }

    private static func readIndex(_ pointer: UnsafeMutableRawPointer, index: Int, bytesPerIndex: Int) -> UInt32 {
        if bytesPerIndex == 2 {
            return UInt32(pointer.advanced(by: index * 2).assumingMemoryBound(to: UInt16.self).pointee)
        }
        return pointer.advanced(by: index * 4).assumingMemoryBound(to: UInt32.self).pointee
    }
}
