import ARKit
import Foundation
import Network
import simd
import SwiftUI
import UIKit
import LiDARLinkShared

struct ErrorItem: Identifiable, Equatable {
    let id = UUID()
    let message: String
}

@MainActor
final class PhoneAppState: ObservableObject {
    enum ConnectionState: Equatable {
        case idle
        case discovering
        case connecting(String)
        case connected(String)
        case handshakeFailed(String)
        case reconnecting(String)
        case disconnected(String)
    }

    enum ScanState: Equatable {
        case idle
        case running
        case paused
    }

    @Published var connectionState: ConnectionState = .idle
    @Published var scanState: ScanState = .idle
    @Published var isCalibrating = false
    @Published var discoveredMacs: [DiscoveredMac] = []
    @Published var sendStats = SendStats()
    @Published var errorItem: ErrorItem?
    @Published var supportMessage: String?
    @Published var isScanningSupported = false
    @Published var config = ScanConfiguration()
    @Published var captureMode: CaptureMode = .standard
    @Published var currentTier = 0
    /// Run Apple RoomPlan alongside the depth stream — the Mac gets a parametric
    /// room (walls/openings/objects) as a structural prior and clean overlay.
    @Published var roomPlanEnabled = false
    var roomPlanSupported: Bool { RoomCaptureController.isSupported }
    @Published var coveredSectors: Set<Int> = []
    @Published var captureQuality = CaptureQualitySnapshot()
    /// True once the active connection came in over the USB tunnel (via
    /// `USBListener`), rather than Wi-Fi/Bonjour — drives `wiredBoosted`.
    @Published private(set) var isWiredTransport = false

    private let support: DeviceSupport
    private let configBox: ScanConfigBox
    private var browser: BonjourBrowser?
    private var client: StreamClient?
    private var arController: ARSessionController?
    private var roomController: RoomCaptureController?
    private var usbListener: USBListener?
    private var objectPhotoController: ObjectPhotoController?
    @Published var objectPhotoProgress: (taken: Int, target: Int)?
    @Published var objectPhotoSending = false
    private var processor: FrameProcessor?
    private var adaptiveTask: Task<Void, Never>?
    private var reconnectTask: Task<Void, Never>?
    private var lastEndpoint: NWEndpoint?
    private var lastConnectedName: String?
    private var reconnectAttempts = 0
    private var scanStartPosition: SIMD3<Float>?

    var arSession: ARSession? { roomController?.roomSession.arSession ?? arController?.session }

    init(support: DeviceSupport = .evaluate()) {
        self.support = support
        let initialConfig = ScanConfiguration()
        self.configBox = ScanConfigBox(initialConfig)
        self.config = initialConfig
    }

    static var appVersion: String {
        Bundle.main.infoDictionary?["CFBundleShortVersionString"] as? String ?? "1.0"
    }

    // MARK: Lifecycle

    func onAppear() {
        supportMessage = SupportMessageBuilder.explanation(lidarAvailable: support.lidar,
                                                           worldTrackingAvailable: support.worldTracking,
                                                           isSimulator: support.isSimulator,
                                                           depthSupported: support.depth,
                                                           meshSupported: support.mesh)
        isScanningSupported = support.depth
        if isScanningSupported {
            startBrowsing()
            startUSBListening()   // harmless if no cable — the Mac dials in
        }
    }

    // MARK: USB (cable) link

    func startUSBListening() {
        guard usbListener == nil else { return }
        let listener = USBListener()
        listener.onConnection = { [weak self] connection in
            Task { @MainActor in self?.acceptUSBConnection(connection) }
        }
        listener.onError = { message in Log.warning("USB listener: \(message)", category: "usb") }
        listener.start()
        usbListener = listener
    }

    private func acceptUSBConnection(_ connection: NWConnection) {
        guard case .connected = connectionState else {
            isWiredTransport = true
            applyTier()   // pick up wiredBoosted color settings immediately
            establishConnection(inbound: connection, name: "Mac (USB)")
            return
        }
        connection.cancel()   // already linked over Wi-Fi
    }

    // MARK: Discovery

    func startBrowsing() {
        if case .discovering = connectionState { return }
        let browser = BonjourBrowser()
        browser.onResults = { [weak self] macs in
            Task { @MainActor in self?.discoveredMacs = macs }
        }
        browser.start(preferWired: config.preferWiredLink)
        self.browser = browser
        connectionState = .discovering
    }

    func stopBrowsing() {
        browser?.stop()
        browser = nil
    }

    // MARK: Connection

    func connect(to mac: DiscoveredMac) {
        stopBrowsing()
        lastEndpoint = mac.endpoint
        lastConnectedName = mac.name
        reconnectAttempts = 0
        if isWiredTransport { isWiredTransport = false; applyTier() }
        establishConnection(to: mac.endpoint, name: mac.name)
    }

    private func establishConnection(to endpoint: NWEndpoint, name: String) {
        establishConnection(endpoint: endpoint, inbound: nil, name: name)
    }

    private func establishConnection(inbound connection: NWConnection, name: String) {
        stopBrowsing()
        establishConnection(endpoint: nil, inbound: connection, name: name)
    }

    private func establishConnection(endpoint: NWEndpoint?, inbound: NWConnection?, name: String) {
        connectionState = .connecting(name)
        let client = StreamClient()
        self.client = client
        Task {
            await client.setCallbacks(
                onStats: { [weak self] stats in
                    Task { @MainActor in
                        guard let self else { return }
                        var merged = stats
                        merged.tier = self.currentTier
                        merged.depthResolution = self.config.resolutionLabel
                        merged.colorOn = self.config.colorEnabled
                        merged.colorProducedCount = self.processor?.colorProducedCount ?? 0
                        merged.rejectedFrames = self.processor?.rejectedFrameCount ?? 0
                        self.isCalibrating = self.processor?.isWarmingUp ?? false
                        self.captureQuality = self.processor?.captureQualitySnapshot ?? CaptureQualitySnapshot()
                        merged.meshOn = self.config.meshStreamingEnabled
                        self.sendStats = merged
                        self.updateCoverage()
                    }
                },
                onStateChange: { [weak self] state in
                    Task { @MainActor in self?.handleClientState(state) }
                },
                onControl: { [weak self] action in
                    Task { @MainActor in self?.handleRemoteControl(action) }
                },
                onRemoteSettings: { [weak self] settings in
                    Task { @MainActor in self?.handleRemoteSettings(settings) }
                })
            do {
                let capabilities = Capabilities(supportsDepth: support.depth,
                                                supportsMesh: support.mesh,
                                                supportsColor: true)
                if let inbound {
                    try await client.serve(over: inbound,
                                           deviceName: UIDevice.current.name,
                                           appVersion: Self.appVersion,
                                           capabilities: capabilities)
                } else if let endpoint {
                    try await client.connect(to: endpoint,
                                             deviceName: UIDevice.current.name,
                                             appVersion: Self.appVersion,
                                             capabilities: capabilities,
                                             preferWired: config.preferWiredLink)
                }
            } catch {
                self.connectionState = .handshakeFailed(error.localizedDescription)
                self.errorItem = ErrorItem(message: error.localizedDescription)
            }
        }
    }

    private func sendControl(_ action: ControlAction) {
        guard let client else { return }
        Task { await client.sendControl(action) }
    }

    private func disconnectClient() {
        guard let client else { return }
        Task { await client.disconnect() }
    }

    private func handleClientState(_ state: StreamClient.State) {
        switch state {
        case .connecting:
            connectionState = .connecting(lastConnectedName ?? "Mac")
        case .connected(let serverName):
            connectionState = .connected(serverName)
            reconnectAttempts = 0
            startAdaptiveController()
        case .failed(let reason):
            connectionState = .handshakeFailed(reason)
            errorItem = ErrorItem(message: reason)
        case .disconnected(let reason):
            if scanState != .idle {
                scheduleReconnect(reason: reason ?? "Connection lost")
            } else {
                connectionState = .disconnected(reason ?? "Connection closed")
                if case .running = scanState { scanState = .idle }
            }
        }
    }

    private func scheduleReconnect(reason: String) {
        guard reconnectTask == nil, let lastEndpoint else { return }
        reconnectTask = Task { [weak self] in
            try? await Task.sleep(for: .seconds(2))
            guard let self, !Task.isCancelled else { return }
            self.reconnectAttempts += 1
            if self.reconnectAttempts > 3 {
                self.connectionState = .disconnected("Could not reconnect to the Mac")
                self.scanState = .idle
                self.reconnectTask = nil
                return
            }
            self.connectionState = .reconnecting(self.lastConnectedName ?? "Mac")
            self.establishConnection(to: lastEndpoint, name: self.lastConnectedName ?? "Mac")
            self.reconnectTask = nil
        }
    }

    func disconnect() {
        reconnectTask?.cancel()
        reconnectTask = nil
        adaptiveTask?.cancel()
        adaptiveTask = nil
        arController?.stop()
        arController = nil
        processor = nil
        scanState = .idle
        captureQuality = CaptureQualitySnapshot()
        coveredSectors.removeAll()
        scanStartPosition = nil
        disconnectClient()
        client = nil
        connectionState = .disconnected("Disconnected by user")
    }

    // MARK: Object (Photo) capture

    private func handleObjectPhoto(taken: Int, target: Int) {
        objectPhotoProgress = (taken, target)
        if taken >= target { sendObjectPhotos() }
    }

    /// Stop early and send whatever's captured.
    func finishObjectPhotoCapture() { sendObjectPhotos() }

    private func sendObjectPhotos() {
        guard let photos = objectPhotoController?.photos, !photos.isEmpty,
              let client, !objectPhotoSending else { return }
        objectPhotoSending = true
        arController?.pause()
        Task {
            for photo in photos {
                var p = photo
                p = ObjectPhotoPayload(index: photo.index, total: photos.count,
                                       jpegData: photo.jpegData, gravity: photo.gravity,
                                       intrinsics: photo.intrinsics)
                await client.sendObjectPhoto(p)
                try? await Task.sleep(for: .milliseconds(40))
            }
            await MainActor.run {
                self.objectPhotoSending = false
                self.stopScan()
            }
        }
    }

    // MARK: Scanning

    func startScan() {
        guard isScanningSupported else {
            errorItem = ErrorItem(message: supportMessage ?? "LiDAR is not available on this device.")
            return
        }
        guard case .connected = connectionState else {
            errorItem = ErrorItem(message: "Connect to the LiDAR Link Mac app before scanning.")
            return
        }
        guard scanState == .idle else { return }

        let controller = ARSessionController(support: support)
        arController = controller
        let processor = FrameProcessor()
        self.processor = processor
        captureQuality = CaptureQualitySnapshot()

        if captureMode.isPhotogrammetry {
            let photo = ObjectPhotoController()
            self.objectPhotoController = photo
            objectPhotoProgress = (0, photo.targetPhotos)
            photo.onProgress = { [weak self] taken, target in
                Task { @MainActor in self?.handleObjectPhoto(taken: taken, target: target) }
            }
            controller.onRawFrame = { [weak photo] session, frame in
                photo?.consider(session: session, frame: frame)
            }
        } else {
            controller.onFrame = { [weak self, weak processor, weak client] captured in
                guard let self, let processor, let client else { return }
                let frame = processor.process(captured, config: self.configBox.current)
                if let frame {
                    Task { await client.submit(frame) }
                }
            }
        }
        controller.onError = { [weak self] error in
            Task { @MainActor in
                self?.errorItem = ErrorItem(message: "ARSession error: \(error.localizedDescription)")
            }
        }

        if roomPlanEnabled, RoomCaptureController.isSupported {
            let room = RoomCaptureController()
            self.roomController = room
            room.onRoom = { [weak client] structure in
                Task { await client?.sendRoomStructure(structure) }
            }
            room.start()
            controller.attach(to: room.roomSession.arSession)   // RoomPlan owns the session
        } else {
            controller.start(configuration: config)
        }
        scanState = .running
        coveredSectors.removeAll()
        scanStartPosition = nil
        sendControl(.start)
        Log.info("Scan started", category: "app")
    }

    /// A start/pause/resume/stop sent by the Mac — mirrors tapping the
    /// equivalent button here, so the Mac can drive scanning hands-off.
    private func handleRemoteControl(_ action: ControlAction) {
        switch action {
        case .start: startScan()
        case .pause: pauseScan()
        case .resume: resumeScan()
        case .stop: stopScan()
        case .requestStats: break
        }
    }

    /// A capture-mode / RoomPlan change sent by the Mac. Applied the same way
    /// the on-phone settings UI would; an unrecognised mode name is ignored
    /// (version skew) rather than crashing.
    private func handleRemoteSettings(_ settings: RemoteCaptureSettings) {
        if let mode = CaptureMode(rawValue: settings.captureMode), mode != captureMode {
            applyCaptureMode(mode)
        }
        if roomPlanEnabled != settings.roomPlanEnabled {
            roomPlanEnabled = settings.roomPlanEnabled
        }
        if config.maxDepthMeters != settings.maxDepthMeters {
            updateConfig(config.with { $0.maxDepthMeters = settings.maxDepthMeters })
        }
    }

    func pauseScan() {
        guard scanState == .running else { return }
        arController?.pause()
        scanState = .paused
        sendControl(.pause)
    }

    func resumeScan() {
        guard scanState == .paused else { return }
        arController?.resume()
        scanState = .running
        sendControl(.resume)
    }

    func stopScan() {
        arController?.stop()
        arController = nil
        roomController?.stop()
        roomController = nil
        objectPhotoController = nil
        objectPhotoProgress = nil
        processor = nil
        isCalibrating = false
        captureQuality = CaptureQualitySnapshot()
        scanState = .idle
        coveredSectors.removeAll()
        scanStartPosition = nil
        sendControl(.stop)
    }

    // MARK: Settings

    func updateConfig(_ newConfig: ScanConfiguration) {
        config = newConfig
        configBox.update(newConfig)
    }

    // MARK: Capture modes

    func applyCaptureMode(_ mode: CaptureMode) {
        captureMode = mode
        let preset = isWiredTransport ? ScanConfiguration.wiredBoosted(mode.configuration) : mode.configuration
        config = preset
        configBox.update(preset)
        let modeMessage = (mode == .object) ? ScanModeMessage(mode: .object, voxelCm: 0.5) : ScanModeMessage(mode: .room, voxelCm: 1.0)
        sendScanModeMessage(modeMessage)
        coveredSectors.removeAll()
        scanStartPosition = nil
        Log.info("Capture mode set to \(mode.rawValue)", category: "app")
    }

    private func sendScanModeMessage(_ message: ScanModeMessage) {
        guard let client else { return }
        Task { await client.sendScanMode(message) }
    }

    private func updateCoverage() {
        guard captureMode == .object, scanState == .running, let pos = arController?.latestCameraPosition else { return }
        if scanStartPosition == nil { scanStartPosition = pos; return }
        guard let start = scanStartPosition else { return }
        let delta = pos - start
        let bearing = atan2(delta.x, delta.z)
        var sector = Int(((bearing + .pi) / (.pi * 2) * 12).rounded()) % 12
        if sector < 0 { sector += 12 }
        coveredSectors.insert(sector)
    }

    // MARK: Adaptive performance

    private func startAdaptiveController() {
        guard config.adaptivePerformanceEnabled else { return }
        adaptiveTask?.cancel()
        adaptiveTask = Task { [weak self] in
            var lastDropped = 0
            var cleanSeconds = 0
            while !Task.isCancelled {
                try? await Task.sleep(for: .seconds(2))
                guard let self else { return }
                let dropped = self.sendStats.droppedFrames
                let delta = dropped - lastDropped
                lastDropped = dropped
                if delta >= 4, self.currentTier < ScanConfiguration.tiers.count - 1 {
                    self.currentTier += 1
                    self.applyTier()
                    cleanSeconds = 0
                    Log.info("Adaptive: reduced quality to tier \(self.currentTier)", category: "app")
                } else if delta == 0 {
                    cleanSeconds += 2
                    if cleanSeconds >= 12, self.currentTier > 0 {
                        self.currentTier -= 1
                        self.applyTier()
                        cleanSeconds = 0
                        Log.info("Adaptive: increased quality to tier \(self.currentTier)", category: "app")
                    }
                } else {
                    cleanSeconds = 0
                }
            }
        }
    }

    private func applyTier() {
        // Rebuild from the active capture mode so its intent — depth range, mesh
        // streaming, keyframe policy, motion sensitivity — is never lost, then
        // carry the user's own toggles across. Tier 0 is unthrottled (full mode
        // quality); tiers 1+ only dial the streaming knobs, so recovering from a
        // downgrade restores the mode's real quality, not a tier's.
        var merged = isWiredTransport ? ScanConfiguration.wiredBoosted(captureMode.configuration) : captureMode.configuration
        merged.colorEnabled = config.colorEnabled
        merged.useSmoothedDepth = config.useSmoothedDepth
        merged.adaptivePerformanceEnabled = config.adaptivePerformanceEnabled

        if currentTier > 0 {
            // Bandwidth pressure showed up even on the wired boost (more likely
            // encode/CPU-bound than link-bound) — the ladder still degrades from
            // wherever the boosted baseline sits, same as over Wi-Fi.
            let tier = ScanConfiguration.tiers[currentTier]
            merged.targetFPS = tier.targetFPS
            merged.colorEveryNFrames = max(merged.colorEveryNFrames, tier.colorEveryNFrames)
            merged.colorJPEGQuality = min(merged.colorJPEGQuality, tier.colorJPEGQuality)
            merged.colorMaxDimension = min(merged.colorMaxDimension, tier.colorMaxDimension)
            if tier.depthWidth * tier.depthHeight < merged.depthWidth * merged.depthHeight {
                merged.depthWidth = tier.depthWidth
                merged.depthHeight = tier.depthHeight
            }
        }
        config = merged
        configBox.update(merged)
    }
}
