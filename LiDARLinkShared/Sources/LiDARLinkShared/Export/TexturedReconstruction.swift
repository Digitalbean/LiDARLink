import Foundation
import simd

/// End-to-end: a recording's frames → TSDF fusion → marching-cubes mesh →
/// texture atlas baked from the colour keyframes. The output is a coarse,
/// photo-textured mesh ready to export or display.
public enum TexturedReconstruction {

    public static func build(frames: [ScanFrame],
                             voxelSize: Float = 0.035,
                             truncationVoxels: Float = 5,
                             maxKeyframes: Int = 60,
                             minTranslationMeters: Float = 0.06,
                             minRotationDegrees: Float = 6,
                             atlasSize: Int = 2048) -> TextureBaker.BakedMesh? {
        let volume = TSDFVolume(voxelSize: voxelSize, truncationVoxels: truncationVoxels)
        for frame in frames {
            guard let depth = frame.depth else { continue }
            let scale = frame.depthScale == 0 ? 1 : frame.depthScale
            volume.integrate(depth: depth.float16Array(), width: depth.width, height: depth.height,
                             depthScale: scale, intrinsics: frame.intrinsics, pose: frame.pose,
                             confidence: depth.confidenceArray(), minConfidence: 1)
        }
        let mesh = volume.mesh()
        guard !mesh.isEmpty else { return nil }

        // Colour keyframes, spaced by camera motion.
        var views: [TextureBaker.View] = []
        var lastPose: simd_float4x4?
        for frame in frames {
            guard let colour = frame.color, colour.width > 0 else { continue }
            if let last = lastPose {
                let dt = simd_length(translation(frame.pose) - translation(last))
                let dr = rotationDegrees(from: last, to: frame.pose)
                if dt < minTranslationMeters && dr < minRotationDegrees { continue }
            }
            guard let sampler = ColorImageSampler(color: colour) else { continue }
            let k = frame.intrinsics.scaled(to: sampler.width, height: sampler.height)
            views.append(TextureBaker.View(sampler: sampler, pose: frame.pose, intrinsics: k))
            lastPose = frame.pose
            if views.count >= maxKeyframes { break }
        }
        guard !views.isEmpty else { return nil }

        return TextureBaker.bake(mesh: mesh, views: views, atlasSize: atlasSize)
    }

    private static func translation(_ m: simd_float4x4) -> SIMD3<Float> {
        SIMD3<Float>(m.columns.3.x, m.columns.3.y, m.columns.3.z)
    }

    private static func rotationDegrees(from a: simd_float4x4, to b: simd_float4x4) -> Float {
        func basis(_ m: simd_float4x4) -> simd_float3x3 {
            simd_float3x3(SIMD3(m.columns.0.x, m.columns.0.y, m.columns.0.z),
                          SIMD3(m.columns.1.x, m.columns.1.y, m.columns.1.z),
                          SIMD3(m.columns.2.x, m.columns.2.y, m.columns.2.z))
        }
        let r = basis(b) * basis(a).transpose
        let trace = r.columns.0.x + r.columns.1.y + r.columns.2.z
        return acos(simd_clamp((trace - 1) / 2, -1, 1)) * 180 / .pi
    }
}
