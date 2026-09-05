import Foundation
import Network
import LiDARLinkShared

enum ClientError: Error, LocalizedError {
    case acknowledgementTimeout
    case rejected(reason: String)
    case versionMismatch(found: UInt8)
    case connectionClosed(String)

    var errorDescription: String? {
        switch self {
        case .acknowledgementTimeout:
            return "The Mac did not respond to the connection request. Make sure the LiDAR Link Mac app is running and the connection was approved."
        case .rejected(let reason):
            return "The Mac rejected the connection: \(reason)"
        case .versionMismatch(let found):
            return "Protocol version mismatch (phone \(found), Mac \(ProtocolVersion.current)). Update both apps."
        case .connectionClosed(let reason):
            return "Connection closed: \(reason)"
        }
    }
}

/// Owns the NWConnection to the Mac and applies backpressure with a
/// single-slot latest-wins queue (obsolete frames are dropped, never queued).
actor StreamClient {
    enum State: Equatable {
        case connecting
        case connected(String)
        case failed(String)
        case disconnected(String?)
    }

    private(set) var state: State = .connecting
    private var onStats: ((SendStats) -> Void)?
    private var onStateChange: ((State) -> Void)?
    /// A remote start/pause/resume/stop from the Mac — lets it drive scanning
    /// without touching the phone.
    private var onControl: ((ControlAction) -> Void)?
    /// A remote capture-mode / RoomPlan change from the Mac.
    private var onRemoteSettings: ((RemoteCaptureSettings) -> Void)?

    /// Sets the UI-facing callbacks (must be called before `connect`).
    func setCallbacks(onStats: @escaping (SendStats) -> Void,
                      onStateChange: @escaping (State) -> Void,
                      onControl: @escaping (ControlAction) -> Void = { _ in },
                      onRemoteSettings: @escaping (RemoteCaptureSettings) -> Void = { _ in }) {
        self.onStats = onStats
        self.onStateChange = onStateChange
        self.onControl = onControl
        self.onRemoteSettings = onRemoteSettings
    }

    private var connection: NWConnection?
    private let decoder = MessageDecoder()
    private let queue = LatestWinsQueue<ProcessedFrame>()
    private let fpsMeter = FPSMeter()
    private let throughput = ThroughputMeter()
    private var isSending = false
    private var sequence: UInt32 = 0
    private var isConnected = false
    private var ackContinuation: CheckedContinuation<HelloAckMessage?, Never>?
    private var ackTimeoutTask: Task<Void, Never>?
    private var heartbeatTask: Task<Void, Never>?
    private var statsTask: Task<Void, Never>?

    /// Connects out (Wi-Fi), sends `hello`, and waits for the Mac's approval.
    func connect(to endpoint: NWEndpoint, deviceName: String, appVersion: String,
                 capabilities: Capabilities, preferWired: Bool = false) async throws {
        let connection = NWConnection(to: endpoint, using: LinkParameters.tcp(preferWired: preferWired))
        connection.stateUpdateHandler = { [weak self] newState in
            Task { await self?.handleConnectionState(newState) }
        }
        connection.start(queue: .main)
        try await handshake(over: connection, deviceName: deviceName, appVersion: appVersion, capabilities: capabilities)
    }

    /// Runs the handshake over an inbound connection the Mac opened through the
    /// usbmuxd cable tunnel. The phone still sends `hello` first.
    func serve(over connection: NWConnection, deviceName: String, appVersion: String,
               capabilities: Capabilities) async throws {
        connection.stateUpdateHandler = { [weak self] newState in
            Task { await self?.handleConnectionState(newState) }
        }
        connection.start(queue: .main)
        try await handshake(over: connection, deviceName: deviceName, appVersion: appVersion, capabilities: capabilities)
    }

    private func handshake(over connection: NWConnection, deviceName: String, appVersion: String,
                           capabilities: Capabilities) async throws {
        self.connection = connection

        let hello = HelloMessage(protocolVersion: ProtocolVersion.current,
                                 appName: "LiDARLinkPhone",
                                 appVersion: appVersion,
                                 deviceName: deviceName,
                                 capabilities: capabilities)
        let helloData = try MessageCodec.encode(.hello(hello), sequence: 0, timestampMs: Time.monotonicMilliseconds())

        let ack: HelloAckMessage? = await withCheckedContinuation { continuation in
            ackContinuation = continuation
            ackTimeoutTask = Task { [weak self] in
                try? await Task.sleep(for: .seconds(8))
                guard let self else { return }
                await self.expireAckWait()
            }
            connection.send(content: helloData, completion: .contentProcessed { error in
                if let error {
                    Task { await self.failConnection("Failed to send handshake: \(error.localizedDescription)") }
                }
            })
        }

        guard let ack else {
            throw ClientError.connectionClosed("No handshake response from the Mac")
        }
        guard ack.accepted else {
            throw ClientError.rejected(reason: ack.reason ?? "Connection denied")
        }
        guard ack.protocolVersion == ProtocolVersion.current else {
            throw ClientError.versionMismatch(found: ack.protocolVersion)
        }

        isConnected = true
        state = .connected(ack.serverName)
        onStateChange?(.connected(ack.serverName))
        startBackgroundTasks()
    }

    /// Submits a processed frame. If a send is in flight, the newest frame
    /// replaces the pending one (the replaced frame is counted as dropped).
    func submit(_ frame: ProcessedFrame) {
        guard isConnected else { return }
        if isSending {
            _ = queue.submit(frame)
            return
        }
        sendFrame(frame)
    }

    func sendControl(_ action: ControlAction) {
        guard isConnected else { return }
        sendRaw(.control(ControlMessage(action: action)))
    }

    func sendScanMode(_ message: ScanModeMessage) {
        guard isConnected else { return }
        sendRaw(.scanMode(message))
    }

    func sendRoomStructure(_ room: RoomStructure) {
        guard isConnected else { return }
        sendRaw(.roomStructure(room))
    }

    func sendObjectPhoto(_ photo: ObjectPhotoPayload) {
        guard isConnected else { return }
        sendRaw(.objectPhoto(photo))
    }

    func disconnect() {
        teardown(reason: "Disconnected by user", wasConnected: isConnected)
    }

    // MARK: Sending

    private func sendFrame(_ frame: ProcessedFrame) {
        isSending = true
        sequence &+= 1
        let seq = sequence
        let timestamp = frame.captureTimestampMs
        let messages = WireFrameBuilder.frameMessages(sequence: seq,
                                                      captureTimestampMs: timestamp,
                                                      pose: frame.pose,
                                                      intrinsics: frame.intrinsics,
                                                      depth: frame.depth,
                                                      color: frame.color,
                                                      depthIsSmoothed: frame.depthIsSmoothed,
                                                      depthScale: frame.depthScale,
                                                      downsampleFactor: frame.downsampleFactor,
                                                      meshChunks: frame.meshChunks ?? [],
                                                      removedMeshAnchorIDs: frame.removedMeshAnchorIDs)
        let chunks: [Data] = messages.compactMap { try? MessageCodec.encode($0, sequence: seq, timestampMs: timestamp) }
        sendChunks(chunks) { [weak self] in
            Task { await self?.frameSendFinished() }
        }
    }

    private func frameSendFinished() {
        isSending = false
        if let next = queue.take() {
            sendFrame(next)
        }
    }

    private func sendChunks(_ chunks: [Data], completion: @escaping () -> Void) {
        guard let connection else { completion(); return }
        guard !chunks.isEmpty else { completion(); return }
        var remaining = chunks
        let chunk = remaining.removeFirst()
        let bytes = chunk.count
        let remainingChunks = remaining
        connection.send(content: chunk, completion: .contentProcessed { [weak self] _ in
            Task { await self?.didSendChunk(bytes: bytes, remaining: remainingChunks, completion: completion) }
        })
    }

    private func didSendChunk(bytes: Int, remaining: [Data], completion: @escaping () -> Void) {
        throughput.record(bytes: bytes, timestampMs: Time.monotonicMilliseconds())
        sendChunks(remaining, completion: completion)
    }

    private func sendRaw(_ message: Message) {
        guard let connection,
              let data = try? MessageCodec.encode(message, sequence: sequence, timestampMs: Time.monotonicMilliseconds()) else { return }
        connection.send(content: data, completion: .contentProcessed { _ in })
    }

    // MARK: Receiving

    private func receiveLoop() {
        guard let connection else { return }
        connection.receive(minimumIncompleteLength: 1, maximumLength: 16 * 1024 * 1024) { [weak self] data, _, isComplete, error in
            Task { await self?.handleReceive(data: data, isComplete: isComplete, error: error) }
        }
    }

    private func handleReceive(data: Data?, isComplete: Bool, error: Error?) {
        if let error {
            teardown(reason: "Connection error: \(error.localizedDescription)", wasConnected: isConnected)
            return
        }
        if let data {
            for item in decoder.append(data) {
                switch item {
                case .message(let message, _):
                    switch message {
                    case .helloAck(let ack):
                        ackTimeoutTask?.cancel()
                        ackContinuation?.resume(returning: ack)
                        ackContinuation = nil
                    case .versionMismatch(let mismatch):
                        ackTimeoutTask?.cancel()
                        ackContinuation?.resume(returning: nil)
                        ackContinuation = nil
                        Log.warning("Server reported version mismatch \(mismatch.clientVersion) vs \(mismatch.serverVersion)", category: "client")
                    case .error(let errorMessage):
                        Log.error("Server error: \(errorMessage.message)", category: "client")
                    case .control(let control):
                        onControl?(control.action)
                    case .remoteCaptureSettings(let settings):
                        onRemoteSettings?(settings)
                    case .heartbeat, .hello, .frameMeta, .frameDepth, .frameColor, .meshMeta, .meshData, .scanMode, .meshRemove, .roomStructure, .objectPhoto:
                        break
                    }
                case .corrupt(let reason):
                    Log.warning("Corrupt message from server: \(reason)", category: "client")
                case .incompatibleVersion(let found, let expected):
                    ackTimeoutTask?.cancel()
                    ackContinuation?.resume(returning: nil)
                    ackContinuation = nil
                    teardown(reason: "Protocol version mismatch (found \(found), expected \(expected))", wasConnected: isConnected)
                }
            }
        }
        if isComplete {
            teardown(reason: "Connection closed by the Mac", wasConnected: isConnected)
            return
        }
        receiveLoop()
    }

    // MARK: State

    private func handleConnectionState(_ newState: NWConnection.State) {
        switch newState {
        case .ready:
            receiveLoop()
        case .failed(let error):
            teardown(reason: "Connection failed: \(error.localizedDescription)", wasConnected: isConnected)
        case .cancelled:
            break
        default:
            break
        }
    }

    private func failConnection(_ reason: String) {
        guard !isConnected else { return }
        teardown(reason: reason, wasConnected: false)
    }

    private func expireAckWait() {
        ackContinuation?.resume(returning: nil)
        ackContinuation = nil
    }

    private func teardown(reason: String, wasConnected: Bool) {
        ackTimeoutTask?.cancel()
        ackTimeoutTask = nil
        heartbeatTask?.cancel()
        heartbeatTask = nil
        statsTask?.cancel()
        statsTask = nil
        ackContinuation?.resume(returning: nil)
        ackContinuation = nil
        isConnected = false
        connection?.cancel()
        connection = nil
        if wasConnected {
            state = .disconnected(reason)
            onStateChange?(.disconnected(reason))
        } else if state == .connecting {
            state = .failed(reason)
            onStateChange?(.failed(reason))
        }
    }

    private func startBackgroundTasks() {
        statsTask?.cancel()
        statsTask = Task { [weak self] in
            while !Task.isCancelled {
                try? await Task.sleep(for: .seconds(1))
                guard let self else { return }
                await self.publishStats()
            }
        }
        heartbeatTask?.cancel()
        heartbeatTask = Task { [weak self] in
            while !Task.isCancelled {
                try? await Task.sleep(for: .seconds(2))
                guard let self else { return }
                await self.sendHeartbeatIfConnected()
            }
        }
    }

    private func publishStats() {
        let stats = SendStats(sentFPS: fpsMeter.fps,
                              droppedFrames: queue.droppedCount,
                              bytesSent: throughput.totalBytes,
                              throughputBytesPerSecond: throughput.bytesPerSecond)
        onStats?(stats)
    }

    private func sendHeartbeatIfConnected() {
        guard isConnected else { return }
        sendRaw(.heartbeat)
    }
}
