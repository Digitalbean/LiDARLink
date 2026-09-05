import ARKit
import Foundation
import LiDARLinkShared

/// Owns the ARSession. All delegate callbacks run on a dedicated serial queue so
/// that capture work never blocks the main thread.
final class ARSessionController: NSObject, ARSessionDelegate {
    let session = ARSession()
    private let delegateQueue = DispatchQueue(label: "com.lidarlink.capture", qos: .userInitiated)
    private let support: DeviceSupport
    private var lastConfiguration: ARWorldTrackingConfiguration?
    private var didLogCaptureInfo = false
    private var cameraPosition: SIMD3<Float>?
    private let positionLock = NSLock()
    /// Mesh anchors ARKit removed since the last `didUpdate` frame. Read and
    /// cleared on the delegate queue, same as the frame callbacks.
    private var removedMeshAnchorIDs: [UUID] = []

    var onFrame: ((CapturedFrame) -> Void)?
    var onError: ((Error) -> Void)?
    /// Raw ARKit frame + session — the Object (Photo) mode needs
    /// `captureHighResolutionFrame`.
    var onRawFrame: ((ARSession, ARFrame) -> Void)?

    /// True when observing a session owned by RoomPlan — `start`/`resume`/`pause`
    /// become no-ops because `RoomCaptureSession` drives the lifecycle.
    private(set) var isAttached = false

    /// Latest camera translation, safe to read from any thread.
    var latestCameraPosition: SIMD3<Float>? {
        positionLock.lock()
        defer { positionLock.unlock() }
        return cameraPosition
    }

    init(support: DeviceSupport) {
        self.support = support
        super.init()
        session.delegate = self
        session.delegateQueue = delegateQueue
    }

    /// Observe frames from a session RoomPlan owns, instead of running our own.
    func attach(to externalSession: ARSession) {
        isAttached = true
        externalSession.delegateQueue = delegateQueue
        externalSession.delegate = self
        Log.info("ARSessionController attached to RoomPlan session", category: "capture")
    }

    func start(configuration: ScanConfiguration) {
        guard !isAttached else { return }
        let config = ARWorldTrackingConfiguration()
        config.worldAlignment = .gravity

        // Pick the sharpest camera format that still stays near 1080p on the
        // short side — enough to feed the 1440-px wire colour at full quality
        // without paying the on-device cost of downscaling a 4K buffer every
        // frame.
        let formats = ARWorldTrackingConfiguration.supportedVideoFormats
        let affordable = formats.filter { min($0.imageResolution.width, $0.imageResolution.height) <= 1440 }
        if let best = (affordable.isEmpty ? formats : affordable).max(by: {
            $0.imageResolution.width * $0.imageResolution.height
                < $1.imageResolution.width * $1.imageResolution.height
        }) {
            config.videoFormat = best
        }

        if support.depth {
            if configuration.useSmoothedDepth, ARWorldTrackingConfiguration.supportsFrameSemantics(.smoothedSceneDepth) {
                config.frameSemantics.insert(.smoothedSceneDepth)
            } else {
                config.frameSemantics.insert(.sceneDepth)
            }
        }
        if configuration.meshStreamingEnabled, support.mesh {
            // iOS 26: scene reconstruction is enabled solely via this property;
            // ARMeshAnchor output is produced when set to a non-none value.
            config.sceneReconstruction = .meshWithClassification
        }
        lastConfiguration = config
        session.run(config)
        Log.info("ARSession started (smoothedDepth=\(configuration.useSmoothedDepth), mesh=\(configuration.meshStreamingEnabled))", category: "capture")
    }

    func resume() {
        guard !isAttached, let lastConfiguration else { return }
        session.run(lastConfiguration, options: [.removeExistingAnchors])
    }

    func pause() {
        guard !isAttached else { return }
        session.pause()
    }

    func stop() {
        guard !isAttached else { return }
        session.pause()
    }

    // MARK: ARSessionDelegate

    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        onRawFrame?(session, frame)
        let transform = frame.camera.transform
        positionLock.lock()
        cameraPosition = SIMD3<Float>(transform.columns.3.x, transform.columns.3.y, transform.columns.3.z)
        positionLock.unlock()
        if !didLogCaptureInfo, let sceneDepth = frame.sceneDepth {
            didLogCaptureInfo = true
            let depthW = CVPixelBufferGetWidth(sceneDepth.depthMap)
            let depthH = CVPixelBufferGetHeight(sceneDepth.depthMap)
            let intrinsics = frame.camera.intrinsics
            // Persisted so it survives in the unified log for verification.
            Log.error("Capture info: native depth \(depthW)x\(depthH) "
                      + "camera image \(Int(frame.camera.imageResolution.width))x\(Int(frame.camera.imageResolution.height)) "
                      + "intrinsics fx=\(intrinsics[0][0]) fy=\(intrinsics[1][1]) "
                      + "cx=\(intrinsics[2][0]) cy=\(intrinsics[2][1])",
                      category: "capture")
        }
        let smoothed = frame.smoothedSceneDepth
        let sceneDepth = smoothed ?? frame.sceneDepth

        let trackingIssue: CaptureTrackingIssue
        switch frame.camera.trackingState {
        case .normal:
            trackingIssue = .none
        case .notAvailable:
            trackingIssue = .unavailable
        case .limited(let reason):
            switch reason {
            case .initializing: trackingIssue = .initializing
            case .excessiveMotion: trackingIssue = .excessiveMotion
            case .insufficientFeatures: trackingIssue = .insufficientFeatures
            case .relocalizing: trackingIssue = .relocalizing
            @unknown default: trackingIssue = .unavailable
            }
        }

        let removed = removedMeshAnchorIDs
        removedMeshAnchorIDs.removeAll(keepingCapacity: true)

        let captured = CapturedFrame(
            cameraImage: frame.capturedImage,
            depthBuffer: sceneDepth?.depthMap,
            confidenceBuffer: sceneDepth?.confidenceMap,
            depthIsSmoothed: smoothed != nil,
            // `sceneDepth` and `smoothedSceneDepth` belong to this ARFrame. Using
            // a guessed older pose creates motion-dependent ghost surfaces.
            pose: frame.camera.transform,
            intrinsics: CameraIntrinsics(matrix: frame.camera.intrinsics,
                                         imageWidth: Int(frame.camera.imageResolution.width),
                                         imageHeight: Int(frame.camera.imageResolution.height)),
            timestampMs: UInt64(frame.timestamp * 1000),
            meshAnchors: frame.anchors.compactMap { $0 as? ARMeshAnchor },
            removedMeshAnchorIDs: removed,
            trackingIssue: trackingIssue,
            ambientIntensity: frame.lightEstimate.map { Double($0.ambientIntensity) })
        onFrame?(captured)
    }

    func session(_ session: ARSession, didRemove anchors: [ARAnchor]) {
        for anchor in anchors where anchor is ARMeshAnchor {
            removedMeshAnchorIDs.append(anchor.identifier)
        }
    }

    func session(_ session: ARSession, didFailWithError error: Error) {
        Log.error("ARSession failed: \(error.localizedDescription)", category: "capture")
        onError?(error)
    }

    func sessionWasInterrupted(_ session: ARSession) {
        Log.warning("ARSession interrupted", category: "capture")
    }

    func sessionInterruptionEnded(_ session: ARSession) {
        Log.info("ARSession interruption ended", category: "capture")
    }
}
