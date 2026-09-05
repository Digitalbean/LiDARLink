import AppKit
import Foundation
import UniformTypeIdentifiers
import LiDARLinkShared

/// Save-panel + export orchestration. The heavy work (point-cloud construction,
/// PLY/OBJ/USDZ writing) runs on a background queue via the shared package.
final class ExportController {
    static func exportPointCloud(points: [PointCloudPoint], completion: @escaping (Result<URL, Error>) -> Void) {
        let panel = NSSavePanel()
        panel.allowedContentTypes = [UTType(filenameExtension: "ply") ?? .data]
        panel.nameFieldStringValue = "LiDARLinkPointCloud.ply"
        panel.begin { response in
            guard response == .OK, let url = panel.url else { return }
            DispatchQueue.global(qos: .userInitiated).async {
                do {
                    try ScanExporter.exportPointCloud(points, to: url)
                    DispatchQueue.main.async { completion(.success(url)) }
                } catch {
                    DispatchQueue.main.async { completion(.failure(error)) }
                }
            }
        }
    }

    static func exportMeshOBJ(mesh: MeshData, completion: @escaping (Result<URL, Error>) -> Void) {
        exportMesh(mesh: mesh, filename: "LiDARLinkMesh.obj", extension: "obj", export: { try ScanExporter.exportMeshOBJ(mesh, to: $0) }, completion: completion)
    }

    /// The scan as a classified LAS 1.2 + enhanced binary PLY.
    static func exportClassifiedCloud(_ cloud: RichPointCloud,
                                      completion: @escaping (Result<URL, Error>) -> Void) {
        let save = NSSavePanel()
        save.nameFieldStringValue = "LiDARLink-classified"
        save.message = "Where to write the LAS + PLY point cloud"
        save.begin { s in
            guard s == .OK, let output = save.url else { return }
            DispatchQueue.global(qos: .userInitiated).async {
                do {
                    let folder = output.deletingPathExtension()
                    try FileManager.default.createDirectory(at: folder, withIntermediateDirectories: true)
                    try LASExporter.data(from: cloud).write(to: folder.appendingPathComponent("cloud.las"))
                    try RichPLYExporter.data(from: cloud).write(to: folder.appendingPathComponent("cloud.ply"))
                    let summary = cloud.histogram().sorted { $0.value > $1.value }
                        .map { "\($0.key.label): \($0.value)" }.joined(separator: "\n")
                    try summary.write(to: folder.appendingPathComponent("classes.txt"), atomically: true, encoding: .utf8)
                    DispatchQueue.main.async { completion(.success(folder)) }
                } catch {
                    DispatchQueue.main.async { completion(.failure(error)) }
                }
            }
        }
    }

    static func exportMeshUSDZ(mesh: MeshData, completion: @escaping (Result<URL, Error>) -> Void) {
        exportMesh(mesh: mesh, filename: "LiDARLinkMesh.usdz", extension: "usdz") { url in
            _ = try ScanExporter.exportMeshUSDZ(mesh, to: url, fallbackToUSD: true)
        } completion: { result in
            completion(result)
        }
    }

    private static func exportMesh(mesh: MeshData,
                                   filename: String,
                                   extension fileExtension: String,
                                   export: @escaping (URL) throws -> Void,
                                   completion: @escaping (Result<URL, Error>) -> Void) {
        let panel = NSSavePanel()
        panel.allowedContentTypes = [UTType(filenameExtension: fileExtension) ?? .data]
        panel.nameFieldStringValue = filename
        panel.begin { response in
            guard response == .OK, let url = panel.url else { return }
            DispatchQueue.global(qos: .userInitiated).async {
                do {
                    try export(url)
                    DispatchQueue.main.async { completion(.success(url)) }
                } catch {
                    DispatchQueue.main.async { completion(.failure(error)) }
                }
            }
        }
    }
}
