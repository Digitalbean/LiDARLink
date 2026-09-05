import Foundation
import LiDARLinkShared

/// User-configurable scanning parameters. Adaptive performance lowers these
/// automatically when the connection cannot keep up.
struct ScanConfiguration: Equatable {
    var targetFPS = 12
    var depthWidth = 256
    var depthHeight = 192
    var colorEnabled = true
    // Colour every keyframe: a colour-less frame makes every one of its points
    // fall back to a flat grey, which then fuses into the voxel map and washes
    // the whole cloud out. Bandwidth is managed by resolution/quality, not by
    // skipping frames.
    var colorEveryNFrames = 1
    // ARKit hands over a 1920×1440 frame for free; 640 was a streaming
    // throttle. Keep near-native colour so points, recordings and any future
    // photoreal reconstruction get real pixels. Adaptive tiers scale this down.
    var colorJPEGQuality = 0.85
    var colorMaxDimension = 1440
    var useSmoothedDepth = true
    // LiDAR is usable to ~5 m; 4 clamped a metre of real coverage.
    var maxDepthMeters: Float = 5.0
    var extraTemporalSmoothing = false
    var bilateralSmoothing = true
    var warmUpDuration: Double = 1.5
    var motionSensitivity: MotionSensitivity = .normal
    var keyframeMinTranslationMeters: Float = 0.015
    var keyframeMinRotationDegrees: Float = 1.5
    var keyframeHeartbeatMs: UInt64 = 500
    var meshStreamingEnabled = false
    var meshMaxVerticesPerAnchor = 60_000
    var adaptivePerformanceEnabled = true
    /// Pin discovery + streaming to a wired interface (iPhone↔Mac USB) instead
    /// of Wi-Fi. Much higher throughput, no dropped frames.
    var preferWiredLink = false

    var resolutionLabel: String { "\(depthWidth)×\(depthHeight)" }

    /// USB has bandwidth to spare that Wi-Fi doesn't, so a wired link trades it
    /// for color fidelity: near-lossless JPEG and the full native capture
    /// resolution instead of a downscale. Depth geometry and `targetFPS` are
    /// untouched — the Mac's fusion pipeline paces itself independently of
    /// arrival rate, so sending frames faster wouldn't currently do anything
    /// with them; raising just the color budget is the safe, unconditional win.
    static func wiredBoosted(_ base: ScanConfiguration) -> ScanConfiguration {
        var boosted = base
        boosted.colorJPEGQuality = max(boosted.colorJPEGQuality, 0.95)
        boosted.colorMaxDimension = max(boosted.colorMaxDimension, 1920)
        return boosted
    }

    /// Adaptive bandwidth ladder (index 0 = highest quality). Depth is the
    /// geometry — it stays at the LiDAR's native 256×192 on every tier; only
    /// fps and colour size/quality/cadence give under load. `applyTier` layers
    /// these onto the active capture mode so its intent (depth range, mesh,
    /// keyframe policy) is never lost.
    static let tiers: [ScanConfiguration] = [
        ScanConfiguration(targetFPS: 15, depthWidth: 256, depthHeight: 192,
                          colorEveryNFrames: 1, colorJPEGQuality: 0.85, colorMaxDimension: 1440),
        ScanConfiguration(targetFPS: 12, depthWidth: 256, depthHeight: 192,
                          colorEveryNFrames: 1, colorJPEGQuality: 0.8, colorMaxDimension: 1080),
        ScanConfiguration(targetFPS: 10, depthWidth: 256, depthHeight: 192,
                          colorEveryNFrames: 2, colorJPEGQuality: 0.7, colorMaxDimension: 900),
        ScanConfiguration(targetFPS: 8, depthWidth: 256, depthHeight: 192,
                          colorEveryNFrames: 2, colorJPEGQuality: 0.6, colorMaxDimension: 720)
    ]
}

/// Named capture presets. Individual settings below still override after a preset
/// is applied.
enum CaptureMode: String, CaseIterable, Identifiable {
    case standard
    case dense
    case fast
    case mesh
    case object
    case objectPhoto

    var id: String { rawValue }

    var label: String {
        switch self {
        case .standard: return "Standard"
        case .dense: return "Dense"
        case .fast: return "Fast"
        case .mesh: return "Mesh"
        case .object: return "Object"
        case .objectPhoto: return "Object (Photo)"
        }
    }

    var configuration: ScanConfiguration {
        switch self {
        case .standard:
            return ScanConfiguration(targetFPS: 12, depthWidth: 256, depthHeight: 192)
        case .dense:
            // ARKit scene depth is fixed at 256×192; "dense" spends its budget
            // on colour and a slower, more thorough sweep instead.
            return ScanConfiguration(targetFPS: 8, depthWidth: 256, depthHeight: 192,
                                     colorJPEGQuality: 0.9, colorMaxDimension: 1920,
                                     keyframeMinTranslationMeters: 0.01,
                                     keyframeMinRotationDegrees: 1.0)
        case .fast:
            return ScanConfiguration(targetFPS: 20, depthWidth: 256, depthHeight: 192,
                                     colorJPEGQuality: 0.7, colorMaxDimension: 960)
        case .mesh:
            return ScanConfiguration(targetFPS: 10, depthWidth: 256, depthHeight: 192,
                                     meshStreamingEnabled: true)
        case .object:
            return ScanConfiguration(targetFPS: 15, depthWidth: 256, depthHeight: 192,
                                     colorEveryNFrames: 1, colorJPEGQuality: 0.9, colorMaxDimension: 1920,
                                     maxDepthMeters: 1.2,
                                     keyframeMinTranslationMeters: 0.005,
                                     keyframeMinRotationDegrees: 1.0,
                                     meshStreamingEnabled: true)
        case .objectPhoto:
            // Photogrammetry mode — the depth stream is unused; the phone
            // captures 12 MP stills as the user orbits the object.
            return ScanConfiguration(targetFPS: 6, depthWidth: 256, depthHeight: 192,
                                     colorEnabled: false, maxDepthMeters: 1.5)
        }
    }

    var isPhotogrammetry: Bool { self == .objectPhoto }
}

/// Thread-safe holder so the capture pipeline can read the latest configuration
/// from a background queue without racing the UI.
final class ScanConfigBox: @unchecked Sendable {
    private let lock = NSLock()
    private var value: ScanConfiguration

    init(_ initial: ScanConfiguration) {
        self.value = initial
    }

    var current: ScanConfiguration {
        lock.lock()
        defer { lock.unlock() }
        return value
    }

    func update(_ newValue: ScanConfiguration) {
        lock.lock()
        defer { lock.unlock() }
        value = newValue
    }
}
