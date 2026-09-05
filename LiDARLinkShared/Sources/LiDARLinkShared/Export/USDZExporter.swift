import Foundation
import simd
import ModelIO

/// Exports a mesh to USDZ (or USD fallback) via ModelIO.
public enum USDZExporter {
    public enum ExportError: Error, LocalizedError, Equatable {
        case emptyMesh
        case modelIOFailed(String)

        public var errorDescription: String? {
            switch self {
            case .emptyMesh: return "There is no mesh data to export."
            case .modelIOFailed(let reason): return "USDZ export failed: \(reason)"
            }
        }
    }

    public static func export(mesh: MeshData, to url: URL) throws {
        guard !mesh.vertices.isEmpty, !mesh.faces.isEmpty else { throw ExportError.emptyMesh }

        // Interleave positions + normals into a single vertex buffer.
        var vertexData = Data(capacity: mesh.vertices.count * 24)
        for (vertex, normal) in zip(mesh.vertices, mesh.normals) {
            Binary.append(vertex, to: &vertexData)
            Binary.append(normal, to: &vertexData)
        }
        var indexData = Data(capacity: mesh.faces.count * 4)
        for face in mesh.faces {
            Binary.append(face, to: &indexData)
        }

        let descriptor = MDLVertexDescriptor()
        descriptor.attributes = [
            MDLVertexAttribute(name: MDLVertexAttributePosition, format: .float3, offset: 0, bufferIndex: 0),
            MDLVertexAttribute(name: MDLVertexAttributeNormal, format: .float3, offset: 12, bufferIndex: 0)
        ]
        descriptor.layouts = [MDLVertexBufferLayout(stride: 24)]

        let vertexBuffer = MDLMeshBufferData(type: .vertex, data: vertexData)
        let indexBuffer = MDLMeshBufferData(type: .index, data: indexData)
        let submesh = MDLSubmesh(indexBuffer: indexBuffer,
                                 indexCount: mesh.faces.count,
                                 indexType: .uInt32,
                                 geometryType: .triangles,
                                 material: nil)
        let mdlMesh = MDLMesh(vertexBuffer: vertexBuffer,
                              vertexCount: mesh.vertices.count,
                              descriptor: descriptor,
                              submeshes: [submesh])

        let asset = MDLAsset()
        asset.add(mdlMesh)
        do {
            try asset.export(to: url)
        } catch {
            throw ExportError.modelIOFailed(error.localizedDescription)
        }
    }
}
