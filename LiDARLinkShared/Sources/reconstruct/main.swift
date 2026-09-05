import Foundation
import simd
import CoreGraphics
import ImageIO
import UniformTypeIdentifiers
import LiDARLinkShared

// Usage: reconstruct <recording-folder> <output-folder> [voxelCm]
//
// Runs the v2 finalize pipeline (keyframes → pose graph → loop closure →
// re-integrable TSDF → marching cubes → texture atlas) on a v1 recording and
// writes model.obj / model.mtl / model.png plus a vertex-coloured PLY.
//
// Env: SKIP_LOOPS=1, LOOP_WINDOW, LOOP_MAXDIST, LOOP_MAXRMS, LOOP_MINCORR.

let args = CommandLine.arguments
guard args.count >= 3 else {
    FileHandle.standardError.write(Data("usage: reconstruct <recording> <output> [voxelCm]\n".utf8))
    exit(2)
}
let recordingURL = URL(fileURLWithPath: args[1])
let outputURL = URL(fileURLWithPath: args[2])
let voxelOverride = (args.count > 3 ? Float(args[3]) : nil).map { $0 / 100 }

func log(_ s: String) { print(s); fflush(stdout) }

do {
    log("Loading \(recordingURL.lastPathComponent) …")
    let (manifest, frames, _) = try ScanRecordingStore.loadRecording(from: recordingURL)
    log("  \(frames.count) frames, started \(manifest.startedAt)")

    let env = ProcessInfo.processInfo.environment
    var options = V2Reconstruction.Options()
    if let v = voxelOverride { options.configuration.voxelSizeMeters = v }
    options.bakeTexture = true
    options.detectLoopClosures = env["SKIP_LOOPS"] == nil
    if let w = env["LOOP_WINDOW"].flatMap(Int.init) { options.loopParameters.sequentialWindow = w }
    if let d = env["LOOP_MAXDIST"].flatMap(Float.init) { options.loopParameters.maxCameraDistanceMeters = d }
    if let r = env["LOOP_MAXRMS"].flatMap(Float.init) { options.loopParameters.maxRMSMeters = r }
    if let c = env["LOOP_MINCORR"].flatMap(Int.init) { options.loopParameters.minCorrespondences = c }
    log("  voxel \(Int(options.configuration.voxelSizeMeters * 1000)) mm, truncation \(options.configuration.truncationVoxels) voxels")

    guard let result = await V2Reconstruction.run(frames: frames, options: options, progress: { log("  \($0)") }) else {
        FileHandle.standardError.write(Data("reconstruction produced nothing — recording needs depth\n".utf8))
        exit(1)
    }
    log("  \(result.keyframeCount) keyframes, \(result.loopClosures) closures, error \(result.initialError) → \(result.finalError), reintegrated \(result.reintegrated)")

    let fm = FileManager.default
    try fm.createDirectory(at: outputURL, withIntermediateDirectories: true)

    // Vertex-coloured raw surface — TSDF voxel colour straight through marching cubes.
    let vc = result.mesh
    var ply = "ply\nformat ascii 1.0\nelement vertex \(vc.vertices.count)\n"
    ply += "property float x\nproperty float y\nproperty float z\n"
    ply += "property uchar red\nproperty uchar green\nproperty uchar blue\n"
    ply += "element face \(vc.faces.count / 3)\nproperty list uchar int vertex_indices\nend_header\n"
    for i in 0..<vc.vertices.count {
        let p = vc.vertices[i]
        let c = i < vc.colours.count ? vc.colours[i] : SIMD3<Float>(repeating: 150)   // already 0…255
        ply += "\(p.x) \(p.y) \(p.z) \(UInt8(max(0, min(255, c.x)))) \(UInt8(max(0, min(255, c.y)))) \(UInt8(max(0, min(255, c.z))))\n"
    }
    var f = 0
    while f + 2 < vc.faces.count { ply += "3 \(vc.faces[f]) \(vc.faces[f+1]) \(vc.faces[f+2])\n"; f += 3 }
    try ply.data(using: .utf8)!.write(to: outputURL.appendingPathComponent("vertexcolor.ply"))
    log("  vertexcolor.ply: \(vc.vertices.count) verts, \(vc.faces.count / 3) tris")

    guard let baked = result.textured else {
        FileHandle.standardError.write(Data("no textured mesh produced\n".utf8))
        exit(1)
    }
    log("  \(baked.faces.count / 3) triangles, \(Int(baked.texturedTriangleFraction * 100))% textured")

    let name = "model"
    let (obj, mtl) = try OBJExporter.texturedFiles(baked, name: name)
    try obj.write(to: outputURL.appendingPathComponent("\(name).obj"))
    try mtl.write(to: outputURL.appendingPathComponent("\(name).mtl"))

    let size = baked.atlasSize
    var rgba = baked.atlasRGBA
    let ctx = rgba.withUnsafeMutableBytes { ptr in
        CGContext(data: ptr.baseAddress, width: size, height: size, bitsPerComponent: 8,
                  bytesPerRow: size * 4, space: CGColorSpaceCreateDeviceRGB(),
                  bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue)
    }
    if let cg = ctx?.makeImage(),
       let dest = CGImageDestinationCreateWithURL(outputURL.appendingPathComponent("\(name).png") as CFURL,
                                                  "public.png" as CFString, 1, nil) {
        CGImageDestinationAddImage(dest, cg, nil)
        CGImageDestinationFinalize(dest)
    }
    log("Wrote \(outputURL.path)")
} catch {
    FileHandle.standardError.write(Data("error: \(error.localizedDescription)\n".utf8))
    exit(1)
}
