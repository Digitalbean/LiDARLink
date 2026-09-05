import Foundation

/// High-level export operations used by the Mac app (and unit-tested).
public enum ScanExporter {
    public static let defaultMaxExportPoints = 400_000

    /// Builds a single point cloud from one frame.
    /// Depth pixel coordinates are scaled to the color image resolution so
    /// colors are sampled from the matching location.
    public static func pointCloud(from frame: ScanFrame, step: Int = 1, minConfidence: UInt8 = 0, edgeFilter: Bool = false) -> [PointCloudPoint] {
        guard let depth = frame.depth else { return [] }
        let sampler = frame.color.flatMap { ColorImageSampler(color: $0) }
        return PoseMath.buildPointCloud(depthPayload: depth,
                                        depthScale: frame.depthScale,
                                        intrinsics: frame.intrinsics,
                                        pose: frame.pose,
                                        step: step,
                                        minConfidence: minConfidence,
                                        edgeFilter: edgeFilter,
                                        colorSampler: sampler.map { c in { x, y in
                                            c.color(atX: x * c.width / max(depth.width, 1),
                                                    y: y * c.height / max(depth.height, 1))
                                        } })
    }

    /// Merges all frames into one world-space point cloud, stopping at `maxPoints`.
    /// `step` increases per frame when merging many frames to bound the output size.
    public static func mergedPointCloud(frames: [ScanFrame], maxPoints: Int = defaultMaxExportPoints) -> [PointCloudPoint] {
        var result: [PointCloudPoint] = []
        result.reserveCapacity(min(maxPoints, 65_536))
        var framesProcessed = 0
        outer: for frame in frames {
            framesProcessed += 1
            guard frame.depth != nil else { continue }
            let step = max(1, (framesProcessed + 7) / 8) // coarser sampling as more frames merge
            let points = pointCloud(from: frame, step: step)
            for point in points {
                result.append(point)
                if result.count >= maxPoints { break outer }
            }
        }
        return result
    }

    public static func exportPointCloud(_ points: [PointCloudPoint], to url: URL) throws {
        try PLYExporter.export(points: points, to: url)
    }

    public static func exportMeshOBJ(_ mesh: MeshData, to url: URL) throws {
        try OBJExporter.export(mesh: mesh, to: url)
    }

    /// Exports a mesh to USDZ; on failure falls back to USD when requested.
    @discardableResult
    public static func exportMeshUSDZ(_ mesh: MeshData, to url: URL, fallbackToUSD: Bool = true) throws -> URL {
        do {
            try USDZExporter.export(mesh: mesh, to: url)
            return url
        } catch {
            guard fallbackToUSD, url.pathExtension.lowercased() == "usdz" else { throw error }
            let usdURL = url.deletingPathExtension().appendingPathExtension("usd")
            try USDZExporter.export(mesh: mesh, to: usdURL)
            return usdURL
        }
    }
}
