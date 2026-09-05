import ARKit
import CoreImage
import CoreVideo
import Foundation
import ImageIO
import simd
import LiDARLinkShared

/// A processed frame ready for the wire (all heavy pixel work already done).
struct ProcessedFrame: Sendable {
    let captureTimestampMs: UInt64
    let pose: simd_float4x4
    let intrinsics: CameraIntrinsics
    let depth: DepthPayload?
    let color: ColorPayload?
    let meshChunks: [MeshChunk]?
    let removedMeshAnchorIDs: [UUID]
    let depthIsSmoothed: Bool
    let depthScale: Float
    let downsampleFactor: Int
}

enum CaptureGuidance: Equatable, Sendable {
    case calibrating
    case trackingLimited(String)
    case moveSlower
    case lowLight
    case scanningWell
}

struct CaptureQualitySnapshot: Equatable, Sendable {
    var guidance: CaptureGuidance = .calibrating
    var isWarmingUp = true
    var highConfidencePercent = 0
    var calibrationRejections = 0
    var trackingRejections = 0
    var motionRejections = 0
    var redundantRejections = 0

    var totalRejections: Int {
        calibrationRejections + trackingRejections + motionRejections + redundantRejections
    }
}

/// Converts raw ARKit buffers into wire-ready payloads: downsamples depth and
/// confidence, JPEG-compresses color, and chunks mesh geometry. Runs on the
/// capture queue, never on the main thread.
final class FrameProcessor {
    private let ciContext = CIContext(options: [.cacheIntermediates: false])
    private var colorCounter = 0
    private var lastAttemptMs: UInt64 = 0
    private let lock = NSLock()
    private var producedColorCount = 0
    private var smoothedDepth: [Float]?
    private var lastTranslation: SIMD3<Float>?
    private var lastMeshSendMs: UInt64 = 0
    private var lastAnchorCounts: [UUID: Int] = [:]
    private var lastAnchorSendMs: [UUID: UInt64] = [:]
    private var warmupGate: WarmupGate?
    private let captureGate = CaptureGate()
    private let qualityLock = NSLock()
    private var quality = CaptureQualitySnapshot()
    private var stickyGuidanceUntilMs: UInt64 = 0
    private var lowLightActive = false
    /// Anchor removals seen since the last emitted frame (accumulated across
    /// rejected frames so a removal is never dropped).
    private var pendingRemovedMeshAnchors: Set<UUID> = []

    /// Frames rejected by the motion/quality gate (thread-safe).
    var rejectedFrameCount: Int {
        qualityLock.lock()
        defer { qualityLock.unlock() }
        return quality.trackingRejections + quality.motionRejections
    }

    /// True while the calibration warm-up is withholding frames.
    var isWarmingUp: Bool {
        captureQualitySnapshot.isWarmingUp
    }

    var captureQualitySnapshot: CaptureQualitySnapshot {
        qualityLock.lock()
        defer { qualityLock.unlock() }
        return quality
    }

    /// How many JPEG color frames have been produced (thread-safe).
    var colorProducedCount: Int {
        lock.lock()
        defer { lock.unlock() }
        return producedColorCount
    }

    func process(_ captured: CapturedFrame, config: ScanConfiguration) -> ProcessedFrame? {
        if config.meshStreamingEnabled {
            pendingRemovedMeshAnchors.formUnion(captured.removedMeshAnchorIDs)
        }
        // Startup calibration: skip the first moments of a scan while ARKit's
        // world tracking settles (early poses anchor points to empty space).
        if config.warmUpDuration > 0 {
            if warmupGate == nil {
                warmupGate = WarmupGate(startMs: captured.timestampMs,
                                        durationMs: UInt64(config.warmUpDuration * 1000))
            }
            let elapsed = warmupGate?.shouldPass(nowMs: captured.timestampMs) ?? true
            if !elapsed {
                mutateQuality {
                    $0.isWarmingUp = true
                    $0.guidance = .calibrating
                    $0.calibrationRejections += 1
                }
                return nil
            }
            mutateQuality { $0.isWarmingUp = false }
            if !captured.trackingIsNormal {
                stickyGuidanceUntilMs = captured.timestampMs + 1_000
                mutateQuality {
                    $0.guidance = .trackingLimited(captured.trackingIssue.message)
                    $0.trackingRejections += 1
                }
                return nil
            }
        } else {
            mutateQuality { $0.isWarmingUp = false }
        }
        // Frame pacing: only produce a frame once per target interval.
        let intervalMs = UInt64(1000 / max(config.targetFPS, 1))
        let sinceLast = captured.timestampMs &- lastAttemptMs
        if lastAttemptMs != 0, sinceLast < intervalMs { return nil }
        lastAttemptMs = captured.timestampMs

        let decision = captureGate.evaluate(pose: captured.pose,
                                            timestampMs: captured.timestampMs,
                                            trackingIsNormal: captured.trackingIsNormal,
                                            sensitivity: config.motionSensitivity,
                                            minimumTranslationMeters: config.keyframeMinTranslationMeters,
                                            minimumRotationDegrees: config.keyframeMinRotationDegrees,
                                            heartbeatIntervalMs: config.keyframeHeartbeatMs)
        switch decision {
        case .accept:
            break
        case .rejectTracking:
            stickyGuidanceUntilMs = captured.timestampMs + 1_000
            mutateQuality {
                $0.guidance = .trackingLimited(captured.trackingIssue.message)
                $0.trackingRejections += 1
            }
            return nil
        case .rejectMotion:
            stickyGuidanceUntilMs = captured.timestampMs + 1_000
            mutateQuality {
                $0.guidance = .moveSlower
                $0.motionRejections += 1
            }
            return nil
        case .rejectRedundant:
            mutateQuality {
                $0.redundantRejections += 1
                if captured.timestampMs >= self.stickyGuidanceUntilMs {
                    $0.guidance = self.guidanceForLighting(captured.ambientIntensity, colorEnabled: config.colorEnabled)
                }
            }
            return nil
        }

        let depth = makeDepthPayload(from: captured, config: config, poseTranslation: captured.poseTranslation)
        let color = makeColorPayload(from: captured, config: config)
        let meshChunks = config.meshStreamingEnabled
            ? meshUpdates(from: captured, config: config)
            : nil
        let removedMeshAnchorIDs = Array(pendingRemovedMeshAnchors)
        pendingRemovedMeshAnchors.removeAll(keepingCapacity: true)

        mutateQuality {
            if captured.timestampMs >= self.stickyGuidanceUntilMs {
                $0.guidance = self.guidanceForLighting(captured.ambientIntensity, colorEnabled: config.colorEnabled)
            }
        }

        return ProcessedFrame(captureTimestampMs: captured.timestampMs,
                              pose: captured.pose,
                              intrinsics: captured.intrinsics,
                              depth: depth,
                              color: color,
                              meshChunks: meshChunks,
                              removedMeshAnchorIDs: removedMeshAnchorIDs,
                              depthIsSmoothed: captured.depthIsSmoothed,
                              depthScale: 1.0,
                              downsampleFactor: 1)
    }

    /// Mesh updates are throttled (~2 Hz) and only anchors whose vertex count
    /// changed since their last send (or that are stale) are re-transmitted, so a
    /// moving camera does not flood the link with the entire mesh every frame.
    private func meshUpdates(from captured: CapturedFrame, config: ScanConfiguration) -> [MeshChunk] {
        let now = captured.timestampMs
        guard now - lastMeshSendMs >= 500 else { return [] }
        lastMeshSendMs = now
        var changed: [ARMeshAnchor] = []
        for anchor in captured.meshAnchors {
            let count = anchor.geometry.vertices.count
            let changedSinceLast = lastAnchorCounts[anchor.identifier] != count
            let stale = (now - (lastAnchorSendMs[anchor.identifier] ?? 0)) > 2_000
            if changedSinceLast || stale {
                lastAnchorCounts[anchor.identifier] = count
                lastAnchorSendMs[anchor.identifier] = now
                changed.append(anchor)
            }
        }
        guard !changed.isEmpty else { return [] }
        return MeshExtractor.chunks(from: changed,
                                    timestampMs: now,
                                    maxVerticesPerAnchor: config.meshMaxVerticesPerAnchor)
    }

    private func makeDepthPayload(from captured: CapturedFrame, config: ScanConfiguration, poseTranslation: SIMD3<Float>) -> DepthPayload? {
        guard let depthBuffer = captured.depthBuffer else { return nil }
        CVPixelBufferLockBaseAddress(depthBuffer, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthBuffer, .readOnly) }
        guard let baseAddress = CVPixelBufferGetBaseAddress(depthBuffer) else { return nil }
        let width = CVPixelBufferGetWidth(depthBuffer)
        let height = CVPixelBufferGetHeight(depthBuffer)
        guard width > 0, height > 0 else { return nil }
        let bytesPerRow = CVPixelBufferGetBytesPerRow(depthBuffer)
        // Uniform downscale that preserves the depth map's aspect ratio, so the
        // scaled intrinsics sent in the metadata stay exact.
        let scale = min(Float(config.depthWidth) / Float(width), Float(config.depthHeight) / Float(height))
        let targetWidth = max(1, Int(Float(width) * scale))
        let targetHeight = max(1, Int(Float(height) * scale))
        let depthPointer = baseAddress.assumingMemoryBound(to: Float.self)
        let rowStride = bytesPerRow / MemoryLayout<Float>.stride
        var depthArray = [Float](repeating: 0, count: width * height)
        for row in 0..<height {
            let src = depthPointer.advanced(by: row * rowStride)
            let dst = row * width
            for col in 0..<width {
                depthArray[dst + col] = src[col]
            }
        }
        // Clean the full-resolution depth before downsampling: cut unreliable far
        // values and remove flying pixels at surface edges.
        DepthCleaner.applyMaxDistance(depth: &depthArray, width: width, height: height, maxDistanceMeters: config.maxDepthMeters)
        DepthCleaner.removeFlyingPixels(depth: &depthArray, width: width, height: height, maxNeighborDelta: 0.15)
        // Edge-preserving bilateral smoothing: reduces sensor noise without
        // blurring object silhouettes.
        if config.bilateralSmoothing {
            DepthSmoother.bilateralSmooth(depth: &depthArray, width: width, height: height)
        }

        // Optional extra temporal smoothing: motion-gated EMA. Reset when the
        // camera moved enough that pixels no longer track the same surface.
        if config.extraTemporalSmoothing {
            let motion = simd_length(poseTranslation - (lastTranslation ?? poseTranslation))
            if motion > 0.05 || smoothedDepth?.count != depthArray.count {
                smoothedDepth = depthArray
            } else if var previous = smoothedDepth {
                _ = DepthTemporal.blend(current: &depthArray, previous: previous, alpha: 0.3, maxDelta: 0.15)
                previous = depthArray
                smoothedDepth = previous
            }
            lastTranslation = poseTranslation
        } else {
            smoothedDepth = nil
            lastTranslation = nil
        }

        let depth16 = DepthDownsampler.downsample(depth: depthArray,
                                                  width: width,
                                                  height: height,
                                                  targetWidth: targetWidth,
                                                  targetHeight: targetHeight)

        var confidence: Data?
        if let confidenceBuffer = captured.confidenceBuffer {
            CVPixelBufferLockBaseAddress(confidenceBuffer, .readOnly)
            defer { CVPixelBufferUnlockBaseAddress(confidenceBuffer, .readOnly) }
            if let confBase = CVPixelBufferGetBaseAddress(confidenceBuffer) {
                let confWidth = CVPixelBufferGetWidth(confidenceBuffer)
                let confHeight = CVPixelBufferGetHeight(confidenceBuffer)
                let confBytesPerRow = CVPixelBufferGetBytesPerRow(confidenceBuffer)
                let conf = DepthDownsampler.downsampleConfidence(pointer: confBase.assumingMemoryBound(to: UInt8.self),
                                                                 width: confWidth,
                                                                 height: confHeight,
                                                                 bytesPerRow: confBytesPerRow,
                                                                 targetWidth: targetWidth,
                                                                 targetHeight: targetHeight)
                var validCount = 0
                var highCount = 0
                for index in conf.indices where index < depth16.count && depth16[index] > 0 {
                    validCount += 1
                    if conf[index] >= 2 { highCount += 1 }
                }
                mutateQuality {
                    $0.highConfidencePercent = validCount == 0 ? 0 : Int((Double(highCount) / Double(validCount) * 100).rounded())
                }
                confidence = Binary.data(from: conf)
            }
        }

        return DepthPayload(width: targetWidth,
                            height: targetHeight,
                            depthData: Binary.data(from: depth16),
                            confidenceData: confidence)
    }

    private func guidanceForLighting(_ ambientIntensity: Double?, colorEnabled: Bool) -> CaptureGuidance {
        if let ambientIntensity {
            if lowLightActive {
                lowLightActive = ambientIntensity < 150
            } else {
                lowLightActive = ambientIntensity < 100
            }
        }
        if colorEnabled, lowLightActive {
            return .lowLight
        }
        return .scanningWell
    }

    private func mutateQuality(_ mutation: (inout CaptureQualitySnapshot) -> Void) {
        qualityLock.lock()
        mutation(&quality)
        qualityLock.unlock()
    }

    private func makeColorPayload(from captured: CapturedFrame, config: ScanConfiguration) -> ColorPayload? {
        guard config.colorEnabled else { return nil }
        colorCounter += 1
        guard colorCounter >= config.colorEveryNFrames else { return nil }
        colorCounter = 0

        let ciImage = CIImage(cvPixelBuffer: captured.cameraImage)
        let largestSide = max(ciImage.extent.width, ciImage.extent.height)
        guard largestSide > 0 else { return nil }
        let scale = min(1, CGFloat(config.colorMaxDimension) / largestSide)
        let scaled = ciImage.transformed(by: CGAffineTransform(scaleX: scale, y: scale))
        guard let cgImage = ciContext.createCGImage(scaled, from: scaled.extent) else { return nil }

        let mutableData = NSMutableData()
        guard let destination = CGImageDestinationCreateWithData(mutableData, "public.jpeg" as CFString, 1, nil) else { return nil }
        CGImageDestinationAddImage(destination, cgImage, [kCGImageDestinationLossyCompressionQuality: config.colorJPEGQuality] as CFDictionary)
        guard CGImageDestinationFinalize(destination) else { return nil }

        lock.lock()
        producedColorCount += 1
        lock.unlock()
        return ColorPayload(width: cgImage.width, height: cgImage.height, jpegData: mutableData as Data)
    }
}
