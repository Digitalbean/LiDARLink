import Foundation
import simd

/// A scene-reconstruction mesh (from an ARMeshAnchor) assembled from wire chunks.
public struct MeshData: Sendable, Equatable {
    public var anchorID: UUID
    public var vertices: [SIMD3<Float>]
    public var normals: [SIMD3<Float>]
    public var faces: [UInt32]          // triangle index triplets
    public var transform: simd_float4x4
    public var updatedAtMs: UInt64

    public init(anchorID: UUID, vertices: [SIMD3<Float>], normals: [SIMD3<Float>], faces: [UInt32], transform: simd_float4x4, updatedAtMs: UInt64) {
        self.anchorID = anchorID
        self.vertices = vertices
        self.normals = normals
        self.faces = faces
        self.transform = transform
        self.updatedAtMs = updatedAtMs
    }

    public var triangleCount: Int { faces.count / 3 }

    /// Merges per-anchor meshes into one world-space mesh: each anchor's
    /// transform is baked into its vertices/normals and the face indices are
    /// re-based. Returns nil when there is nothing to merge.
    public static func merged(_ meshes: [MeshData]) -> MeshData? {
        let usable = meshes.filter { !$0.vertices.isEmpty && $0.faces.count >= 3 }
        guard !usable.isEmpty else { return nil }

        var vertices: [SIMD3<Float>] = []
        var normals: [SIMD3<Float>] = []
        var faces: [UInt32] = []
        vertices.reserveCapacity(usable.reduce(0) { $0 + $1.vertices.count })
        faces.reserveCapacity(usable.reduce(0) { $0 + $1.faces.count })

        for mesh in usable {
            let offset = UInt32(vertices.count)
            let rotation = simd_float3x3(SIMD3(mesh.transform.columns.0.x, mesh.transform.columns.0.y, mesh.transform.columns.0.z),
                                        SIMD3(mesh.transform.columns.1.x, mesh.transform.columns.1.y, mesh.transform.columns.1.z),
                                        SIMD3(mesh.transform.columns.2.x, mesh.transform.columns.2.y, mesh.transform.columns.2.z))
            for v in mesh.vertices {
                let w = mesh.transform * SIMD4<Float>(v, 1)
                vertices.append(SIMD3<Float>(w.x, w.y, w.z))
            }
            for n in mesh.normals {
                normals.append(simd_normalize(rotation * n))
            }
            while normals.count < vertices.count { normals.append(SIMD3<Float>(0, 0, 1)) }
            // Re-base and drop any face that references a vertex the anchor
            // didn't actually include.
            var i = 0
            while i + 2 < mesh.faces.count {
                let a = mesh.faces[i], b = mesh.faces[i + 1], c = mesh.faces[i + 2]
                if a < UInt32(mesh.vertices.count), b < UInt32(mesh.vertices.count), c < UInt32(mesh.vertices.count) {
                    faces.append(offset + a)
                    faces.append(offset + b)
                    faces.append(offset + c)
                }
                i += 3
            }
        }

        return MeshData(anchorID: UUID(),
                        vertices: vertices,
                        normals: normals,
                        faces: faces,
                        transform: matrix_identity_float4x4,
                        updatedAtMs: usable.map(\.updatedAtMs).max() ?? 0)
    }
}

/// Metadata describing one chunk of a mesh update.
public struct MeshMetadata: Codable, Equatable, Sendable {
    public var anchorID: UUID
    public var chunkIndex: Int
    public var chunkCount: Int
    public var timestampMs: UInt64
    /// Anchor-to-world transform. Nil means identity (older payloads).
    public var transform: simd_float4x4?

    public init(anchorID: UUID, chunkIndex: Int, chunkCount: Int, timestampMs: UInt64, transform: simd_float4x4? = nil) {
        self.anchorID = anchorID
        self.chunkIndex = chunkIndex
        self.chunkCount = chunkCount
        self.timestampMs = timestampMs
        self.transform = transform
    }
}

/// One chunk of a mesh update. Each chunk is self-describing:
/// vertexCount, normalCount, faceCount, vertex data, normal data, face index data.
public struct MeshChunk: Sendable, Equatable {
    public var metadata: MeshMetadata
    public var data: Data

    public init(metadata: MeshMetadata, data: Data) {
        self.metadata = metadata
        self.data = data
    }

    // MARK: Wire encoding
    // Payload: chunkIndex(4) chunkCount(4) compression(1) packedLength(4) blob.
    // `blob` is the packed geometry (see `pack`), optionally LZFSE-compressed.
    // Geometry itself is tightly packed at 12 bytes per float3, not 16.

    public func binaryPayload() -> Data {
        var out = Data()
        Binary.append(UInt32(metadata.chunkIndex), to: &out)
        Binary.append(UInt32(metadata.chunkCount), to: &out)
        if let compressed = WireCompression.compress(data) {
            Binary.append(UInt8(1), to: &out)
            Binary.append(UInt32(data.count), to: &out)
            out.append(compressed)
        } else {
            Binary.append(UInt8(0), to: &out)
            Binary.append(UInt32(data.count), to: &out)
            out.append(data)
        }
        return out
    }

    public static func decodeBinaryPayload(_ payload: Data, metadata: MeshMetadata) -> MeshChunk? {
        guard let chunkIndexRaw = Binary.readUInt32(payload, at: 0),
              let chunkCountRaw = Binary.readUInt32(payload, at: 4),
              let compression = Binary.readUInt8(payload, at: 8),
              let packedLengthRaw = Binary.readUInt32(payload, at: 9),
              compression <= 1 else { return nil }
        let packedLength = Int(packedLengthRaw)
        guard packedLength >= 0, packedLength <= ProtocolVersion.maxPayloadSize,
              let blob = Binary.readData(payload, at: 13, length: payload.count - 13) else { return nil }
        let data: Data
        if compression == 1 {
            guard let decoded = WireCompression.decompress(blob, expectedSize: packedLength),
                  decoded.count == packedLength else { return nil }
            data = decoded
        } else {
            guard blob.count == packedLength else { return nil }
            data = blob
        }
        var meta = metadata
        meta.chunkIndex = Int(chunkIndexRaw)
        meta.chunkCount = Int(chunkCountRaw)
        return MeshChunk(metadata: meta, data: data)
    }

    /// Packs a slice of mesh geometry: counts, then tightly-packed float3
    /// vertices and normals (12 bytes each) and UInt32 face indices.
    public static func pack(vertices: [SIMD3<Float>], normals: [SIMD3<Float>], faces: [UInt32]) -> Data {
        var data = Data()
        Binary.append(UInt32(vertices.count), to: &data)
        Binary.append(UInt32(normals.count), to: &data)
        Binary.append(UInt32(faces.count), to: &data)
        for v in vertices {
            Binary.append(v.x, to: &data); Binary.append(v.y, to: &data); Binary.append(v.z, to: &data)
        }
        for n in normals {
            Binary.append(n.x, to: &data); Binary.append(n.y, to: &data); Binary.append(n.z, to: &data)
        }
        for f in faces { Binary.append(f, to: &data) }
        return data
    }

    /// Parses a packed chunk payload into geometry slices.
    public static func unpack(_ data: Data) -> (vertices: [SIMD3<Float>], normals: [SIMD3<Float>], faces: [UInt32])? {
        guard let vertexCountRaw = Binary.readUInt32(data, at: 0),
              let normalCountRaw = Binary.readUInt32(data, at: 4),
              let faceCountRaw = Binary.readUInt32(data, at: 8) else { return nil }
        let vertexCount = Int(vertexCountRaw)
        let normalCount = Int(normalCountRaw)
        let faceCount = Int(faceCountRaw)
        let verticesBytes = vertexCount * 12
        let normalsBytes = normalCount * 12
        let facesBytes = faceCount * 4
        var offset = 12
        guard offset + verticesBytes + normalsBytes + facesBytes <= data.count else { return nil }

        func readFloat3(_ base: Int) -> SIMD3<Float> {
            SIMD3<Float>(Binary.readFloat32(data, at: base) ?? 0,
                         Binary.readFloat32(data, at: base + 4) ?? 0,
                         Binary.readFloat32(data, at: base + 8) ?? 0)
        }
        var vertices: [SIMD3<Float>] = []
        vertices.reserveCapacity(vertexCount)
        for i in 0..<vertexCount { vertices.append(readFloat3(offset + i * 12)) }
        offset += verticesBytes
        var normals: [SIMD3<Float>] = []
        normals.reserveCapacity(normalCount)
        for i in 0..<normalCount { normals.append(readFloat3(offset + i * 12)) }
        offset += normalsBytes
        var faces: [UInt32] = []
        faces.reserveCapacity(faceCount)
        for i in 0..<faceCount { faces.append(Binary.readUInt32(data, at: offset + i * 4) ?? 0) }
        return (vertices, normals, faces)
    }
}
