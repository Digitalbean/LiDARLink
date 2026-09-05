import Foundation
import simd

/// End-to-end v2 finalize: a recording's frames → motion-spaced keyframes →
/// pose graph + loop closure → re-integrable TSDF → marching-cubes surface, and
/// optionally a texture atlas baked at the graph-optimised keyframe poses.
///
/// This is the single path shared by the CLI bridge and the Mac app's Finalize
/// action, so both produce identical output.
public enum V2Reconstruction {

    public struct Result: Sendable {
        public var mesh: MarchingCubes.Mesh
        public var textured: TextureBaker.BakedMesh?
        public var keyframeCount: Int
        public var loopClosures: Int
        public var initialError: Float
        public var finalError: Float
        public var reintegrated: Int
    }

    public struct Options: Sendable {
        public var configuration = ReconstructionEngine.Configuration()
        /// Keep a keyframe only once the camera has moved this far from the last.
        public var minTranslationMeters: Float = 0.05
        public var minRotationRadians: Float = 0.05
        /// Hard cap on keyframes; extras are dropped by even subsampling. 0 = no
        /// cap. Loop-closure cost is O(n²) in keyframes, so a cap keeps a long
        /// recording's finalize bounded.
        public var maxKeyframes: Int = 0
        public var detectLoopClosures = true
        public var loopParameters = LoopClosureDetector.Parameters()
        public var optimiseIterations = 25
        /// Bake a texture atlas. Off by default — it's the slow part.
        public var bakeTexture = false
        public var atlasSize = 4096
        public init() {}
    }

    public static func run(frames: [ScanFrame],
                           options: Options = Options(),
                           progress: (@Sendable (String) -> Void)? = nil) async -> Result? {
        var config = options.configuration
        config.emitSurfaceOnAdd = false
        let engine = ReconstructionEngine(configuration: config)

        // Pass 1: pick motion-spaced frames.
        var picked: [ScanFrame] = []
        var lastPose: simd_float4x4?
        for frame in frames {
            guard frame.depth != nil else { continue }
            if let last = lastPose {
                let t = simd_length(Lie.translation(frame.pose) - Lie.translation(last))
                let r = simd_length(Lie.so3Log(Lie.rotation(last).transpose * Lie.rotation(frame.pose)))
                if t < options.minTranslationMeters && r < options.minRotationRadians { continue }
            }
            picked.append(frame)
            lastPose = frame.pose
        }
        // Even subsample down to the cap (always keep first and last).
        if options.maxKeyframes > 1 && picked.count > options.maxKeyframes {
            let stride = Double(picked.count - 1) / Double(options.maxKeyframes - 1)
            var thinned: [ScanFrame] = []
            for i in 0..<options.maxKeyframes { thinned.append(picked[Int((Double(i) * stride).rounded())]) }
            picked = thinned
        }
        guard picked.count >= 2 else { return nil }

        // Pass 2: feed the engine, relative pose measured between the picks we keep.
        var kept = 0
        var prevPose: simd_float4x4?
        for (index, frame) in picked.enumerated() {
            let relative = prevPose.map { $0.inverse * frame.pose } ?? matrix_identity_float4x4
            let keyframe = Keyframe(id: Int32(index),
                                    capturedAtMs: frame.captureTimestampMs,
                                    depth: frame.depth!,
                                    depthScale: frame.depthScale == 0 ? 1 : frame.depthScale,
                                    depthIntrinsics: frame.intrinsics,
                                    colorJPEG: frame.color?.jpegData,
                                    colorIntrinsics: frame.color != nil ? frame.intrinsics : nil,
                                    arkitPose: frame.pose,
                                    relativePose: relative)
            await engine.add(keyframe)
            prevPose = frame.pose
            kept += 1
            if kept % 25 == 0 { progress?("fusing keyframe \(kept)/\(picked.count)") }
        }

        var loops = 0
        if options.detectLoopClosures {
            progress?("detecting loop closures (\(picked.count) keyframes)")
            loops = await engine.detectLoopClosures(parameters: options.loopParameters)
        }

        progress?("optimising pose graph (\(loops) closures)")
        let event = await engine.optimise(iterations: options.optimiseIterations)
        var initial: Float = 0, final: Float = 0, reintegrated = 0
        if case let .optimised(_, _, i, f, r) = event { initial = i; final = f; reintegrated = r }

        let mesh = await engine.currentMesh()
        guard !mesh.isEmpty else { return nil }

        var textured: TextureBaker.BakedMesh?
        if options.bakeTexture {
            progress?("baking texture")
            textured = await engine.bakeTexturedMesh(atlasSize: options.atlasSize)
        }

        return Result(mesh: mesh, textured: textured,
                      keyframeCount: kept, loopClosures: loops,
                      initialError: initial, finalError: final, reintegrated: reintegrated)
    }
}
