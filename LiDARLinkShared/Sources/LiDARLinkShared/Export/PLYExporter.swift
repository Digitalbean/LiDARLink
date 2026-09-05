import Foundation

/// Exports a point cloud to ASCII PLY.
public enum PLYExporter {
    public enum ExportError: Error, LocalizedError, Equatable {
        case emptyPointCloud

        public var errorDescription: String? {
            switch self {
            case .emptyPointCloud: return "There is no point-cloud data to export."
            }
        }
    }

    public static func export(points: [PointCloudPoint], to url: URL) throws {
        guard !points.isEmpty else { throw ExportError.emptyPointCloud }

        var text = "ply\n"
        text += "format ascii 1.0\n"
        text += "element vertex \(points.count)\n"
        text += "property float x\n"
        text += "property float y\n"
        text += "property float z\n"
        text += "property uchar red\n"
        text += "property uchar green\n"
        text += "property uchar blue\n"
        text += "end_header\n"

        var body = ""
        body.reserveCapacity(points.count * 48)
        for point in points {
            body += "\(point.position.x) \(point.position.y) \(point.position.z) \(point.color.x) \(point.color.y) \(point.color.z)\n"
        }
        guard let data = (text + body).data(using: .utf8) else {
            throw CocoaError(.fileWriteUnknown)
        }
        try data.write(to: url)
    }
}
