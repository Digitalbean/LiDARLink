import Foundation

/// Exports a mesh to ASCII Wavefront OBJ (vertices, normals, faces).
public enum OBJExporter {
    public enum ExportError: Error, LocalizedError, Equatable {
        case emptyMesh

        public var errorDescription: String? {
            switch self {
            case .emptyMesh: return "There is no mesh data to export."
            }
        }
    }

    /// Writes a marching-cubes mesh with inline per-vertex colour
    /// (`v x y z r g b`, 0–1) — read by MeshLab, Blender, CloudCompare, SceneKit.
    public static func export(marchingCubes mesh: MarchingCubes.Mesh, to url: URL) throws {
        guard !mesh.vertices.isEmpty, !mesh.faces.isEmpty else { throw ExportError.emptyMesh }
        var text = "# LiDAR Link v2 surface (vertex-coloured)\no LiDARLinkSurface\n"
        for i in 0..<mesh.vertices.count {
            let p = mesh.vertices[i]
            let c = i < mesh.colours.count ? mesh.colours[i] / 255 : SIMD3<Float>(repeating: 0.63)
            text += "v \(p.x) \(p.y) \(p.z) \(max(0, min(1, c.x))) \(max(0, min(1, c.y))) \(max(0, min(1, c.z)))\n"
        }
        for n in mesh.normals { text += "vn \(n.x) \(n.y) \(n.z)\n" }
        var f = 0
        while f + 2 < mesh.faces.count {
            let a = mesh.faces[f] + 1, b = mesh.faces[f + 1] + 1, c = mesh.faces[f + 2] + 1
            text += "f \(a)//\(a) \(b)//\(b) \(c)//\(c)\n"
            f += 3
        }
        guard let data = text.data(using: .utf8) else { throw CocoaError(.fileWriteUnknown) }
        try data.write(to: url)
    }

    public static func export(mesh: MeshData, to url: URL) throws {
        guard !mesh.vertices.isEmpty, !mesh.faces.isEmpty else { throw ExportError.emptyMesh }

        var text = "# LiDAR Link mesh export\n"
        text += "o LiDARLinkMesh\n"
        for vertex in mesh.vertices {
            text += "v \(vertex.x) \(vertex.y) \(vertex.z)\n"
        }
        for normal in mesh.normals {
            text += "vn \(normal.x) \(normal.y) \(normal.z)\n"
        }
        var face = 0
        while face + 2 < mesh.faces.count {
            let a = mesh.faces[face] + 1
            let b = mesh.faces[face + 1] + 1
            let c = mesh.faces[face + 2] + 1
            text += "f \(a)//\(a) \(b)//\(b) \(c)//\(c)\n"
            face += 3
        }
        guard let data = text.data(using: .utf8) else {
            throw CocoaError(.fileWriteUnknown)
        }
        try data.write(to: url)
    }

    /// The OBJ + MTL text for a texture-baked mesh. The caller writes the atlas
    /// PNG as `<name>.png` alongside these.
    public static func texturedFiles(_ baked: TextureBaker.BakedMesh, name: String) throws -> (obj: Data, mtl: Data) {
        guard !baked.vertices.isEmpty, !baked.faces.isEmpty else { throw ExportError.emptyMesh }

        var obj = "# LiDAR Link textured mesh\n"
        obj += "mtllib \(name).mtl\n"
        obj += "o \(name)\n"
        for v in baked.vertices { obj += "v \(v.x) \(v.y) \(v.z)\n" }
        for n in baked.normals { obj += "vn \(n.x) \(n.y) \(n.z)\n" }
        for uv in baked.uvs { obj += "vt \(uv.x) \(1 - uv.y)\n" }   // OBJ V axis points up
        obj += "usemtl \(name)\n"
        var f = 0
        while f + 2 < baked.faces.count {
            let a = baked.faces[f] + 1, b = baked.faces[f + 1] + 1, c = baked.faces[f + 2] + 1
            obj += "f \(a)/\(a)/\(a) \(b)/\(b)/\(b) \(c)/\(c)/\(c)\n"
            f += 3
        }

        let mtl = """
        newmtl \(name)
        Ka 0 0 0
        Kd 1 1 1
        Ks 0 0 0
        d 1
        illum 1
        map_Kd \(name).png
        """

        guard let objData = obj.data(using: .utf8), let mtlData = mtl.data(using: .utf8) else {
            throw CocoaError(.fileWriteUnknown)
        }
        return (objData, mtlData)
    }
}
