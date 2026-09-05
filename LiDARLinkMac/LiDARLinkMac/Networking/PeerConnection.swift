import Foundation
import Network
import simd
import LiDARLinkShared

/// Decodes the stream from one iPhone and publishes assembled frames, meshes,
/// and diagnostics. All decoding runs on a dedicated serial queue.
final class PeerConnection {
    enum PeerState: Equatable {
        case waiting
        case connected
        case disconnected(String?)
        case failed(String)
    }

    let channel: ByteChannel
    private let queue = DispatchQueue(label: "com.lidarlink.peer", qos: .userInitiated)
    private let decoder = MessageDecoder()
    private let assembler = FrameAssembler()
    private let meshAssembler = MeshChunkAssembler()
    private let sequenceTracker = SequenceTracker()
    private let fpsMeter = FPSMeter()
    private let throughputMeter = ThroughputMeter()
    private var stats = ReceiveStats()
    private var isApproved = false
    private var hasEnded = false
    private var statsTimer: DispatchSourceTimer?
    private var heartbeatTimer: DispatchSourceTimer?
    private var lastPruneMs: UInt64 = 0
    private var pendingMeshMetadata: MeshMetadata?
    private var didLogFirstMeta = false
    private var didLogFirstMesh = false

    var onHello: ((String, Capabilities) -> Void)?
    var onFrame: ((ScanFrame) -> Void)?
    var onMesh: ((MeshData) -> Void)?
    var onMeshRemove: (([UUID]) -> Void)?
    var onStats: ((ReceiveStats) -> Void)?
    var onStateChange: ((PeerState) -> Void)?
    var onControl: ((ControlMessage) -> Void)?
    var onScanMode: ((ScanMode, Float) -> Void)?
    var onRoomStructure: ((RoomStructure) -> Void)?
    var onObjectPhoto: ((ObjectPhotoPayload) -> Void)?

    init(connection: NWConnection) {
        self.channel = NWByteChannel(connection)
    }

    init(channel: ByteChannel) {
        self.channel = channel
    }

    func start() {
        channel.onReady = { Log.info("Peer connection ready", category: "peer") }
        channel.onFailed = { [weak self] reason in
            self?.handleDisconnect(reason: "Connection failed: \(reason)")
        }
        channel.start(queue: queue)
        receiveLoop()
        startTimers()
    }

    func approve(serverName: String) {
        queue.async { [weak self] in
            guard let self else { return }
            self.isApproved = true
            self.send(.helloAck(HelloAckMessage(accepted: true,
                                                protocolVersion: ProtocolVersion.current,
                                                serverName: serverName,
                                                reason: nil)))
            self.onStateChange?(.connected)
        }
    }

    func deny(reason: String = "Connection denied by user") {
        queue.async { [weak self] in
            guard let self else { return }
            self.send(.error(ErrorMessage(code: .connectionDenied, message: reason)))
            self.channel.cancel()
        }
    }

    func cancel() {
        queue.async { [weak self] in
            self?.channel.cancel()
        }
    }

    func send(_ message: Message) {
        guard let data = try? MessageCodec.encode(message, sequence: 0, timestampMs: Time.monotonicMilliseconds()) else { return }
        channel.send(data)
    }

    /// Drives the phone's scan lifecycle remotely — start/pause/resume/stop,
    /// same as tapping the equivalent button on the phone.
    func sendControl(_ action: ControlAction) {
        send(.control(ControlMessage(action: action)))
    }

    /// Drives the phone's capture mode / RoomPlan toggle remotely.
    func sendRemoteCaptureSettings(_ settings: RemoteCaptureSettings) {
        send(.remoteCaptureSettings(settings))
    }

    // MARK: Receive loop

    private func receiveLoop() {
        channel.receive { [weak self] data, isComplete, error in
            guard let self else { return }
            if let error {
                self.handleDisconnect(reason: "Connection error: \(error)")
                return
            }
            if let data { self.handle(data: data) }
            if isComplete {
                self.handleDisconnect(reason: "iPhone closed the connection")
                return
            }
            self.receiveLoop()
        }
    }

    private func handle(data: Data) {
        throughputMeter.record(bytes: data.count, timestampMs: Time.monotonicMilliseconds())
        for item in decoder.append(data) {
            switch item {
            case .message(let message, let header):
                handle(message, header: header)
            case .corrupt(let reason):
                Log.warning("Corrupt frame from iPhone: \(reason)", category: "peer")
            case .incompatibleVersion(let found, let expected):
                send(.versionMismatch(VersionMismatchMessage(clientVersion: found, serverVersion: expected)))
                channel.cancel()
            }
        }
        let now = Time.monotonicMilliseconds()
        if now - lastPruneMs > 3_000 {
            lastPruneMs = now
            _ = assembler.prune(before: now - 5_000)
        }
    }

    private func handle(_ message: Message, header: MessageHeader) {
        switch message {
        case .hello(let hello):
            guard hello.protocolVersion == ProtocolVersion.current else {
                send(.versionMismatch(VersionMismatchMessage(clientVersion: hello.protocolVersion, serverVersion: ProtocolVersion.current)))
                channel.cancel()
                return
            }
            onHello?(hello.deviceName, hello.capabilities)

        case .frameMeta(let metadata):
            if !didLogFirstMeta {
                didLogFirstMeta = true
                // Persisted so it survives in the unified log for verification.
                Log.error("First frame meta: depth \(metadata.depthWidth)x\(metadata.depthHeight) "
                          + "intrinsics fx=\(metadata.intrinsics.fx) fy=\(metadata.intrinsics.fy) "
                          + "cx=\(metadata.intrinsics.cx) cy=\(metadata.intrinsics.cy) "
                          + "scale=\(metadata.depthScale) smoothed=\(metadata.depthIsSmoothed)",
                          category: "peer")
            }
            _ = assembler.apply(metadata: metadata, timestampMs: header.timestampMs)

        case .frameDepth(let depth):
            if let frame = assembler.apply(depth: depth, for: header.sequence, timestampMs: header.timestampMs) {
                handleCompleted(frame)
            }

        case .frameColor(let color):
            if let frame = assembler.apply(color: color, for: header.sequence, timestampMs: header.timestampMs) {
                handleCompleted(frame)
            }

        case .meshMeta(let metadata):
            pendingMeshMetadata = metadata

        case .meshData(let chunk):
            stats.meshChunksReceived += 1
            var meta = chunk.metadata
            if let pending = pendingMeshMetadata {
                meta.anchorID = pending.anchorID
                meta.timestampMs = pending.timestampMs
                meta.transform = pending.transform
                pendingMeshMetadata = nil
            }
            if let mesh = meshAssembler.apply(metadata: meta, chunkData: chunk.data) {
                if !didLogFirstMesh {
                    didLogFirstMesh = true
                    let tx = meta.transform ?? matrix_identity_float4x4
                    Log.error("First mesh: vertices=\(mesh.vertices.count) faces=\(mesh.faces.count) "
                              + "translation=(\(tx.columns.3.x),\(tx.columns.3.y),\(tx.columns.3.z))",
                              category: "peer")
                }
                onMesh?(mesh)
            }

        case .meshRemove(let removal):
            meshAssembler.forget(anchorIDs: removal.anchorIDs)
            onMeshRemove?(removal.anchorIDs)

        case .control(let control):
            onControl?(control)

        case .scanMode(let scanMode):
            onScanMode?(scanMode.mode, scanMode.voxelCm)

        case .roomStructure(let room):
            onRoomStructure?(room)

        case .objectPhoto(let photo):
            onObjectPhoto?(photo)

        case .heartbeat:
            break

        case .helloAck, .versionMismatch, .error, .remoteCaptureSettings:
            Log.info("Unexpected \(message.type) from iPhone", category: "peer")
        }
    }

    private func handleCompleted(_ frame: ScanFrame) {
        if frame.color != nil { stats.colorFrameCount += 1 }
        let disposition = sequenceTracker.record(frame.sequence)
        switch disposition {
        case .gap(let skipped): Log.warning("Frame gap: skipped \(skipped)", category: "peer")
        case .duplicate: Log.info("Duplicate frame \(frame.sequence)", category: "peer")
        case .reordered: Log.info("Reordered frame \(frame.sequence)", category: "peer")
        case .expected: break
        }
        fpsMeter.record(timestampMs: Time.monotonicMilliseconds())
        onFrame?(frame)
    }

    // MARK: Timers

    private func startTimers() {
        let statsTimer = DispatchSource.makeTimerSource(queue: queue)
        statsTimer.schedule(deadline: .now() + 1, repeating: 1)
        statsTimer.setEventHandler { [weak self] in
            guard let self else { return }
            self.publishStats()
        }
        statsTimer.resume()
        self.statsTimer = statsTimer

        let heartbeatTimer = DispatchSource.makeTimerSource(queue: queue)
        heartbeatTimer.schedule(deadline: .now() + 2, repeating: 2)
        heartbeatTimer.setEventHandler { [weak self] in
            guard let self, self.isApproved else { return }
            self.send(.heartbeat)
        }
        heartbeatTimer.resume()
        self.heartbeatTimer = heartbeatTimer
    }

    private func publishStats() {
        stats.receivedFPS = fpsMeter.fps
        stats.frameCount = sequenceTracker.receivedCount
        stats.droppedFrames = sequenceTracker.gapCount
        stats.duplicateFrames = sequenceTracker.duplicateCount
        stats.reorderedFrames = sequenceTracker.reorderedCount
        stats.corruptFrames = decoder.corruptCount
        stats.incompleteFrames = assembler.incompleteCount
        stats.bytesReceived = throughputMeter.totalBytes
        stats.throughputBytesPerSecond = throughputMeter.bytesPerSecond
        stats.lastSequence = sequenceTracker.lastSequence ?? 0
        stats.meshCount = meshAssembler.completedMeshCount
        onStats?(stats)
    }

    private func handleDisconnect(reason: String) {
        guard !hasEnded else { return }
        hasEnded = true
        statsTimer?.cancel()
        heartbeatTimer?.cancel()
        statsTimer = nil
        heartbeatTimer = nil
        channel.cancel()
        onStateChange?(.disconnected(reason))
        Log.info("Peer disconnected: \(reason)", category: "peer")
    }
}
