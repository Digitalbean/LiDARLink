import AppKit
import Foundation
import SceneKit
import simd
import SwiftUI
import UniformTypeIdentifiers
import LiDARLinkShared

struct MacAlertItem: Identifiable {
    let id = UUID()
    let title: String
    let message: String
}

struct ConnectionInfo: Equatable {
    let phoneName: String
    let serverName: String
    let startedAt: Date
}

@MainActor
final class MacAppState: ObservableObject {
    @Published var serverState: Server.ServerState = .idle
    @Published var pendingConnections: [PendingConnection] = []
    @Published var connectionInfo: ConnectionInfo?
    @Published var stats = ReceiveStats()
    /// Most-recently-updated anchor mesh (kept for the "first mesh" log and quick
    /// checks); export uses `meshStore`.
    @Published var mesh: MeshData?
    @Published var meshAnchorCount = 0
    /// All live anchor meshes by ID — the source of truth for display and export.
    private var meshStore: [UUID: MeshData] = [:]
    @Published var alertItem: MacAlertItem?
    @Published var deviceName = ProcessInfo.processInfo.hostName.replacingOccurrences(of: ".local", with: "")
    @Published var showMesh = true
    @Published var autoApprove = false
    @Published var pointColorMode: PointColorMode = .cameraColor
    @Published var solidColor = Color.white
    @Published var pointSize: Double = 2.0
    @Published var backgroundColor = Color(white: 0.07)
    @Published var cloudFull = false
    @Published var showWorldOverlay = true
    /// Screen-space shading from the depth buffer alone (see EyeDomeLighting.metal).
    /// Off by default — first custom render pass in the app.
    @Published var eyeDomeLightingEnabled = false {
        didSet { sceneController.setEyeDomeLightingEnabled(eyeDomeLightingEnabled) }
    }
    @Published var progressiveMapEnabled = true
    @Published var objectMode = false
    @Published var voxelSizeCm: Double = 1.0
    @Published var confidenceFilter: ConfidenceFilter = .medium
    @Published var lastFrameNewPoints = 0
    @Published var mapSaturated = false
    @Published var autoClean = true
    @Published var poseRefinementEnabled = false
    @Published private(set) var poseCorrectionMillimetres: Float = 0
    @Published var tsdfEnabled = false
    /// Run the keyframe pose graph + loop closure and warp the dense cloud when
    /// drift is corrected.
    @Published var poseGraphEnabled = true
    @Published private(set) var poseGraphKeyframes = 0
    @Published private(set) var poseGraphLoops = 0
    /// v1 dense cloud: drop flying pixels at depth edges before unprojection.
    @Published var edgeFilterEnabled = true { didSet { applyEngineConfig() } }
    /// v1 dense cloud: carve free space along depth rays (floaters / ghosts fade).
    @Published var freeSpaceCarvingEnabled = true { didSet { applyEngineConfig() } }
    /// Latest RoomPlan structure from the phone (walls / openings / objects).
    @Published private(set) var roomStructure: RoomStructure?
    @Published var showRoomStructure = true {
        didSet { sceneController.setRoomStructureVisible(showRoomStructure) }
    }
    @Published var floorLocked = true
    @Published var eyeHeightMeters: Double = 1.6
    @Published var meshWireframe = false
    @Published var measuring = false
    @Published var measureDistanceMeters: Float?

    @Published private(set) var mapPointCount = 0
    @Published private(set) var progressiveMapOccupiedCells = 0

    let sceneController = PointCloudSceneController()

    private var server: Server?
    private var peer: PeerConnection?

    /// Headless fusion pipeline. The app forwards frames in and turns the
    /// display-point batches it emits into SceneKit geometry.
    private let engine: ScanEngine
    private var engineOutputTask: Task<Void, Never>?
    private var heightRange: (minY: Float, maxY: Float)?
    private var recenterOnNextReplace = false

    nonisolated static var appVersion: String {
        Bundle.main.infoDictionary?["CFBundleShortVersionString"] as? String ?? "1.0"
    }

    init() {
        // Defaults match the @Published defaults below (room voxel 1 cm,
        // progressive map on, medium confidence, auto-clean on).
        engine = ScanEngine()
    }

    // MARK: Lifecycle

    func onAppear() {
        startEngineConsumer()
        startServer()
        sceneController.onFollowDeviceChanged = { [weak self] on in
            Task { @MainActor in self?.followingiPhone = on }
        }
        let config = engineConfiguration
        Task { await engine.updateConfiguration(config) }
    }

    private func startEngineConsumer() {
        guard engineOutputTask == nil else { return }
        engineOutputTask = Task { [weak self] in
            guard let self else { return }
            for await output in self.engine.outputs {
                await self.handleEngineOutput(output)
            }
        }
    }

    private var engineConfiguration: ScanEngine.Configuration {
        var config = ScanEngine.Configuration()
        config.progressiveMapEnabled = progressiveMapEnabled
        config.voxelSizeMeters = Float(voxelSizeCm) / 100
        config.minConfidence = confidenceFilter.minConfidence
        config.autoClean = autoClean
        config.poseRefinementEnabled = poseRefinementEnabled
        config.tsdfEnabled = tsdfEnabled
        config.poseGraphEnabled = poseGraphEnabled
        config.edgeFilter = edgeFilterEnabled
        config.freeSpaceCarving = freeSpaceCarvingEnabled
        return config
    }

    private func handleEngineOutput(_ output: ScanEngine.Output) async {
        switch output {
        case .stats(let stats):
            mapPointCount = stats.pointCount
            progressiveMapOccupiedCells = stats.occupiedCellCount
            lastFrameNewPoints = stats.lastFrameNewPoints
            cloudFull = stats.cloudFull
            mapSaturated = stats.mapSaturated
            poseCorrectionMillimetres = stats.poseCorrectionMillimetres
            poseGraphKeyframes = stats.poseGraphKeyframes
            poseGraphLoops = stats.poseGraphLoopClosures
        case .firstPoints(let points):
            await appendDisplay(points, recenter: true)
        case .appended(let points):
            await appendDisplay(points, recenter: false)
        case .replaced(let points, let floorY, let recenter):
            if recenter { recenterOnNextReplace = true }
            await installDisplay(points, floorY: floorY)
        case .surfaceMesh(let mesh, let recenter):
            installSurfaceMesh(mesh, recenter: recenter)
        case .surfelCloud:
            break   // v2 surfel mapping is shelved — the Mac app never enables it, so this never fires
        case .cleared:
            sceneController.clear()
            heightRange = nil
        }
    }

    private func appendDisplay(_ points: [PointCloudPoint], recenter: Bool) async {
        guard !points.isEmpty else { return }
        let mode: PointColorMode = pointColorMode == .lit ? .cameraColor : pointColorMode
        let material = sceneController.pointMaterial
        let range = heightRange
        let chunks = await Task.detached(priority: .userInitiated) {
            PointCloudSceneController.makeChunkGeometries(
                points: points, material: material,
                colorFor: PointColorizer.colorTransform(mode: mode, heightRange: range, dimLowConfidence: true))
        }.value
        sceneController.appendPointChunks(chunks)
        if recenter { sceneController.recenter(to: points) }
    }

    private func installSurfaceMesh(_ mesh: MarchingCubes.Mesh?, recenter: Bool) {
        guard let mesh, !mesh.isEmpty else {
            sceneController.clearReconstructedSurface()
            return
        }
        sceneController.installReconstructedSurface(vertices: mesh.vertices,
                                                    normals: mesh.normals,
                                                    colours: mesh.colours,
                                                    faces: mesh.faces,
                                                    wireframe: meshWireframe)
        if recenter {
            sceneController.recenter(to: mesh.vertices.map { PointCloudPoint(position: $0, color: .zero) })
        }
    }

    private func installDisplay(_ points: [PointCloudPoint], floorY: Float?) async {
        let mode = pointColorMode
        let material = sceneController.pointMaterial
        let litRadius = max(Float(voxelSizeCm) / 100 * 4, 0.03)
        let viewpoint = sceneController.cameraWorldPosition
        let (chunks, range) = await Task.detached(priority: .userInitiated) { () -> ([(geometry: SCNGeometry, count: Int)], (minY: Float, maxY: Float)?) in
            let index = PointIndex(cellSize: 0.05)
            index.rebuild(points)
            let (prepared, displayMode) = Self.litPrepared(points: points, allPoints: points,
                                                           startIndex: 0, mode: mode,
                                                           pointIndex: index, radius: litRadius,
                                                           viewpoint: viewpoint)
            let range = PointColorizer.heightRange(of: points)
            let chunks = PointCloudSceneController.makeChunkGeometries(
                points: prepared, material: material,
                colorFor: PointColorizer.colorTransform(mode: displayMode, heightRange: range, dimLowConfidence: true))
            return (chunks, range)
        }.value
        heightRange = range
        sceneController.installPointChunks(chunks)
        sceneController.setLatestPoints(points)
        if let floorY { sceneController.setFloorY(floorY) }
        if recenterOnNextReplace {
            recenterOnNextReplace = false
            sceneController.recenter(to: points)
        }
    }

    // MARK: Object photogrammetry

    @Published private(set) var objectStage: ObjectReconstructor.Stage = .idle
    private lazy var objectReconstructor: ObjectReconstructor = {
        let r = ObjectReconstructor()
        r.onStage = { [weak self] stage in
            guard let self else { return }
            self.objectStage = stage
            if case .done(let url) = stage { self.sceneController.installObjectModel(url) }
        }
        return r
    }()

    // MARK: Remote scan control

    /// The phone's scan lifecycle as last reported over the wire — the phone
    /// already sends `.start`/`.pause`/`.resume`/`.stop` on every local state
    /// change, this just gives the Mac's own UI something to reflect instead
    /// of guessing from what it last commanded.
    enum RemotePhoneScanState: Equatable {
        case unknown, idle, running, paused
    }
    @Published private(set) var remotePhoneScanState: RemotePhoneScanState = .unknown

    /// Capture modes the phone understands, by `CaptureMode.rawValue` — kept
    /// in sync with `LiDARLinkPhone/App/ScanConfiguration.swift`'s `CaptureMode`
    /// by hand, since that type is phone-side (a capture-time UI concept, not
    /// a wire concept) and isn't shared.
    static let remoteCaptureModes: [(name: String, label: String)] = [
        ("standard", "Standard"), ("dense", "Dense"), ("fast", "Fast"),
        ("mesh", "Mesh"), ("object", "Object"), ("objectPhoto", "Object (Photo)"),
    ]
    @Published var remoteCaptureModeName = "standard"
    @Published var remoteRoomPlanEnabled = false
    /// Mirrors the phone's own "Max depth" settings picker (2 / 4 / 8 m).
    @Published var remoteMaxDepthMeters: Float = 4

    func startRemoteScan() { peer?.sendControl(.start) }
    func pauseRemoteScan() { peer?.sendControl(.pause) }
    func resumeRemoteScan() { peer?.sendControl(.resume) }
    func stopRemoteScan() { peer?.sendControl(.stop) }

    private func sendRemoteCaptureSettings() {
        peer?.sendRemoteCaptureSettings(RemoteCaptureSettings(captureMode: remoteCaptureModeName,
                                                               roomPlanEnabled: remoteRoomPlanEnabled,
                                                               maxDepthMeters: remoteMaxDepthMeters))
    }

    func applyRemoteCaptureMode(_ name: String) {
        remoteCaptureModeName = name
        sendRemoteCaptureSettings()
    }

    func applyRemoteRoomPlan(_ enabled: Bool) {
        remoteRoomPlanEnabled = enabled
        sendRemoteCaptureSettings()
    }

    func applyRemoteMaxDepth(_ meters: Float) {
        remoteMaxDepthMeters = meters
        sendRemoteCaptureSettings()
    }

    // MARK: USB bridge

    @Published private(set) var usbBridgeActive = false
    @Published private(set) var usbStatus = ""
    private var usbBridge: USBBridge?

    func toggleUSBBridge() {
        if usbBridgeActive {
            usbBridge?.stop()
            usbBridge = nil
            usbBridgeActive = false
            usbStatus = ""
            return
        }
        let bridge = USBBridge()
        usbBridge = bridge
        usbBridgeActive = true
        bridge.onStatus = { [weak self] status in
            Task { @MainActor in self?.usbStatus = status }
        }
        bridge.onPeer = { [weak self] peer, serial in
            Task { @MainActor in
                self?.usbStatus = "Connected over USB (\(serial.prefix(8))…)"
                self?.handleIncoming(peer)
            }
        }
        bridge.start()
    }

    // MARK: Server

    func startServer() {
        let server = Server()
        self.server = server
        server.onIncomingPeer = { [weak self] peer in
            Task { @MainActor in self?.handleIncoming(peer) }
        }
        server.onStateChange = { [weak self] state in
            Task { @MainActor in self?.serverState = state }
        }
        do {
            try server.start(serviceName: "LiDARLink-\(deviceName)")
            FileLog.append("Server advertising as LiDARLink-\(deviceName)", category: "server")
        } catch {
            alertItem = MacAlertItem(title: "Server Error", message: "Could not start the local server: \(error.localizedDescription)")
        }
    }

    private func handleIncoming(_ peer: PeerConnection) {
        FileLog.append("Incoming connection", category: "server")
        peer.onHello = { [weak self] name, _ in
            Task { @MainActor in
                guard let self else { return }
                for pending in self.pendingConnections where pending.peer === peer {
                    pending.deviceName = name
                }
                if self.autoApprove, let pending = self.pendingConnections.first(where: { $0.peer === peer }) {
                    self.approve(pending)
                }
            }
        }
        peer.start()
        pendingConnections.append(PendingConnection(peer: peer))
    }

    func approve(_ pending: PendingConnection) {
        pendingConnections.removeAll { $0.id == pending.id }
        peer?.cancel()
        let peer = pending.peer
        self.peer = peer
        wire(peer)
        peer.approve(serverName: deviceName)
        connectionInfo = ConnectionInfo(phoneName: pending.deviceName, serverName: deviceName, startedAt: Date())
        Log.info("Approved connection from \(pending.deviceName)", category: "app")
    }

    func deny(_ pending: PendingConnection) {
        pendingConnections.removeAll { $0.id == pending.id }
        pending.peer.deny()
    }

    private func wire(_ peer: PeerConnection) {
        peer.onFrame = { [weak self, weak peer] frame in
            Task { @MainActor in
                guard let self, let peer, self.peer === peer else { return }
                self.handleFrame(frame)
            }
        }
        peer.onMesh = { [weak self, weak peer] mesh in
            Task { @MainActor in
                guard let self, let peer, self.peer === peer else { return }
                self.handleMesh(mesh)
            }
        }
        peer.onMeshRemove = { [weak self, weak peer] ids in
            Task { @MainActor in
                guard let self, let peer, self.peer === peer else { return }
                self.handleMeshRemove(ids)
            }
        }
        peer.onStats = { [weak self, weak peer] stats in
            Task { @MainActor in
                guard let self, let peer, self.peer === peer else { return }
                self.stats = stats
            }
        }
        peer.onStateChange = { [weak self, weak peer] state in
            Task { @MainActor in
                guard let self, let peer, self.peer === peer else { return }
                self.handlePeerState(state)
            }
        }
        peer.onControl = { [weak self, weak peer] control in
            Task { @MainActor in
                guard let self, let peer, self.peer === peer else { return }
                Log.info("Control from iPhone: \(control.action.rawValue)", category: "app")
                switch control.action {
                case .start, .resume: self.remotePhoneScanState = .running
                case .pause: self.remotePhoneScanState = .paused
                case .stop: self.remotePhoneScanState = .idle
                case .requestStats: break
                }
                // Beyond tracking state for the UI: nothing else to do — the
                // engine simply stops receiving frames, and its next fused
                // refresh settles the display.
            }
        }
        peer.onScanMode = { [weak self, weak peer] mode, voxel in
            Task { @MainActor in
                guard let self, let peer, self.peer === peer else { return }
                self.handleScanMode(mode, voxelCm: voxel)
            }
        }
        peer.onObjectPhoto = { [weak self, weak peer] photo in
            Task { @MainActor in
                guard let self, let peer, self.peer === peer else { return }
                self.objectReconstructor.receive(photo)
            }
        }
        peer.onRoomStructure = { [weak self, weak peer] room in
            Task { @MainActor in
                guard let self, let peer, self.peer === peer else { return }
                self.roomStructure = room
                let correction = await self.engine.driftCorrection()
                self.sceneController.installRoomStructure(room, correction: correction)
            }
        }
    }

    private func handlePeerState(_ state: PeerConnection.PeerState) {
        switch state {
        case .connected:
            break
        case .disconnected(let reason):
            connectionInfo = nil
            remotePhoneScanState = .unknown
            if let reason, !reason.isEmpty {
                alertItem = MacAlertItem(title: "Disconnected", message: "iPhone disconnected: \(reason)")
            }
        case .failed(let reason):
            connectionInfo = nil
            remotePhoneScanState = .unknown
            alertItem = MacAlertItem(title: "Connection Failed", message: reason)
        case .waiting:
            break
        }
    }

    private func handleScanMode(_ mode: ScanMode, voxelCm: Float) {
        objectMode = (mode == .object)
        voxelSizeCm = Double(voxelCm)
        applyVoxelSize(Double(voxelCm))
    }

    // MARK: Incoming data

    private var didLogFrameInfo = false
    private var didLogFirstMesh = false

    private func handleFrame(_ frame: ScanFrame) {
        if !didLogFrameInfo {
            didLogFrameInfo = true
            FileLog.append("First frame: depth \(frame.depth?.width ?? 0)x\(frame.depth?.height ?? 0) "
                           + "intrinsics fx=\(frame.intrinsics.fx) fy=\(frame.intrinsics.fy) "
                           + "cx=\(frame.intrinsics.cx) cy=\(frame.intrinsics.cy)",
                           category: "frame")
        }
        sceneController.setDevicePose(frame.pose, verticalFOV: frame.intrinsics.verticalFieldOfViewDegrees)
        Task { await engine.ingest(frame) }
    }

    /// Bakes lit colors for `.lit` mode using normals estimated from the spatial
    /// index (which contains all accumulated points; `points` must be its tail).
    nonisolated private static func litPrepared(points: [PointCloudPoint],
                                                allPoints: [PointCloudPoint],
                                                startIndex: Int,
                                                mode: PointColorMode,
                                                pointIndex: PointIndex,
                                                radius: Float,
                                                viewpoint: SIMD3<Float>? = nil) -> ([PointCloudPoint], PointColorMode) {
        guard mode == .lit, !points.isEmpty else { return (points, mode) }
        var baked = points
        for (offset, point) in points.enumerated() {
            let globalIndex = startIndex + offset
            if let normal = PointNormalEstimator.estimateNormal(at: globalIndex,
                                                                in: pointIndex,
                                                                points: allPoints,
                                                                radius: radius,
                                                                orientToward: viewpoint) {
                baked[offset] = PointCloudPoint(position: point.position,
                                                color: PointColorizer.litColor(color: point.color, normal: normal),
                                                confidence: point.confidence)
            }
        }
        return (baked, .cameraColor)
    }

    private func handleMesh(_ mesh: MeshData) {
        if !didLogFirstMesh {
            didLogFirstMesh = true
            let t = mesh.transform
            FileLog.append("First mesh: vertices=\(mesh.vertices.count) faces=\(mesh.faces.count) "
                           + "translation=(\(t.columns.3.x),\(t.columns.3.y),\(t.columns.3.z))",
                           category: "mesh")
            FileLog.append("Cloud point count at first mesh: \(mapPointCount)", category: "mesh")
        }
        meshStore[mesh.anchorID] = mesh
        self.mesh = mesh
        meshAnchorCount = meshStore.count
        // Always build the node (it honours the visibility flag); the toggle just
        // shows/hides, so meshes captured while hidden still appear when shown.
        sceneController.updateMesh(mesh)
    }

    private func handleMeshRemove(_ ids: [UUID]) {
        guard !ids.isEmpty else { return }
        for id in ids {
            meshStore.removeValue(forKey: id)
            sceneController.removeMesh(id)
        }
        meshAnchorCount = meshStore.count
        if let latest = meshStore.values.max(by: { $0.updatedAtMs < $1.updatedAtMs }) {
            self.mesh = latest
        } else {
            self.mesh = nil
        }
    }

    /// Copy-on-write snapshot of the fused cloud (for export). Async — the
    /// engine owns the data now.
    func mapSnapshot() async -> [PointCloudPoint] {
        await engine.pointSnapshot()
    }

    /// Re-drops the walk camera into the current scan.
    func recenterCamera() {
        sceneController.recenterToLatest()
    }

    @Published private(set) var followingiPhone = false

    /// Toggle the view camera riding the live iPhone pose.
    func toggleFollowiPhone() {
        sceneController.setFollowsDevice(!sceneController.followsDevice)
        followingiPhone = sceneController.followsDevice
    }

    // MARK: Map settings

    private func applyEngineConfig() {
        let config = engineConfiguration
        Task { await engine.updateConfiguration(config) }
    }

    func applyAutoClean(_ enabled: Bool) {
        autoClean = enabled
        let config = engineConfiguration
        Task { await engine.updateConfiguration(config) }
    }

    func applyPoseRefinement(_ enabled: Bool) {
        poseRefinementEnabled = enabled
        if !enabled { poseCorrectionMillimetres = 0 }
        let config = engineConfiguration
        Task { await engine.updateConfiguration(config) }
    }

    func applyTSDF(_ enabled: Bool) {
        tsdfEnabled = enabled
        let config = engineConfiguration
        Task { await engine.updateConfiguration(config) }
    }

    func applyPoseGraph(_ enabled: Bool) {
        poseGraphEnabled = enabled
        poseGraphKeyframes = 0
        poseGraphLoops = 0
        clearScan()
        let config = engineConfiguration
        Task { await engine.updateConfiguration(config) }
    }

    func applyProgressiveMap(_ enabled: Bool) {
        progressiveMapEnabled = enabled
        let config = engineConfiguration
        Task { await engine.updateConfiguration(config) }
    }

    func applyVoxelSize(_ cm: Double) {
        voxelSizeCm = cm
        let config = engineConfiguration
        Task { await engine.updateConfiguration(config) }
    }

    func applyConfidenceFilter(_ filter: ConfidenceFilter) {
        confidenceFilter = filter
        let config = engineConfiguration
        Task { await engine.updateConfiguration(config) }
    }

    // MARK: Point style

    /// The solid-point color as RGB bytes (from the SwiftUI color picker).
    var solidColorRGB: SIMD3<UInt8> {
        let ns = NSColor(solidColor).usingColorSpace(.deviceRGB) ?? NSColor.white
        return SIMD3<UInt8>(UInt8(ns.redComponent * 255),
                            UInt8(ns.greenComponent * 255),
                            UInt8(ns.blueComponent * 255))
    }

    func applyPointColorMode(_ mode: PointColorMode) {
        var effective = mode
        if case .solid = mode {
            effective = .solid(solidColorRGB)
        }
        pointColorMode = effective
        Task { await engine.refresh() }
    }

    func applySolidColor(_ color: Color) {
        solidColor = color
        guard case .solid = pointColorMode else { return }
        pointColorMode = .solid(solidColorRGB)
        Task { await engine.refresh() }
    }

    func applyWorldOverlay(_ visible: Bool) {
        showWorldOverlay = visible
        sceneController.setWorldOverlayVisible(visible)
    }

    // MARK: Camera navigation

    func applyFloorLocked(_ locked: Bool) {
        floorLocked = locked
        sceneController.setFloorLocked(locked)
    }

    func applyEyeHeight(_ metres: Double) {
        eyeHeightMeters = metres
        sceneController.setEyeHeight(Float(metres))
    }

    func applyMeshWireframe(_ wireframe: Bool) {
        meshWireframe = wireframe
        sceneController.setMeshWireframe(wireframe)
    }

    func applyMeasuring(_ on: Bool) {
        measuring = on
        measureDistanceMeters = nil
        sceneController.onMeasurementDistance = { [weak self] distance in
            Task { @MainActor in self?.measureDistanceMeters = distance }
        }
        sceneController.setMeasuring(on)
    }

    func clearMeasurement() {
        sceneController.clearMeasurement()
    }

    /// Levels the whole scan to its floor plane (rotates the floor flat, drops it
    /// to y = 0) so exports come out upright. One-shot; bakes into the data.
    func levelToFloor() {
        Task {
            guard let transform = await engine.levelToFloor() else {
                alertItem = MacAlertItem(title: "Level to Floor",
                                         message: "Couldn't find a clear floor. Scan more of the ground and try again.")
                return
            }
            for (id, mesh) in meshStore {
                var moved = mesh
                moved.transform = transform * mesh.transform
                meshStore[id] = moved
                sceneController.updateMesh(moved)
            }
            recenterOnNextReplace = true // the engine's post-level .replaced recentres the walker
            alertItem = MacAlertItem(title: "Level to Floor", message: "Scan leveled to the floor plane.")
        }
    }

    func applyPointSize(_ size: Double) {
        pointSize = size
        sceneController.setPointSize(Float(size))
    }

    func applyBackgroundColor(_ color: Color) {
        backgroundColor = color
        sceneController.setBackground(NSColor(color))
    }

    func toggleMeshDisplay() {
        showMesh.toggle()
        sceneController.setMeshVisible(showMesh)
    }

    // MARK: Scan lifecycle

    func clearScan() {
        Task { await engine.clear() } // the .cleared output clears the scene
        cloudFull = false
        mapSaturated = false
        lastFrameNewPoints = 0
        mapPointCount = 0
        progressiveMapOccupiedCells = 0
        mesh = nil
        meshStore.removeAll()
        meshAnchorCount = 0
        didLogFrameInfo = false
        didLogFirstMesh = false
        sceneController.clear() // also removes mesh nodes; the .cleared output would too
    }

    // MARK: Export

    func exportPointCloud() {
        Task {
            let points = await engine.pointSnapshot()
            guard !points.isEmpty else {
                alertItem = MacAlertItem(title: "Export", message: "There is no scan data to export.")
                return
            }
            ExportController.exportPointCloud(points: points) { [weak self] result in
                self?.handleExportResult(result, label: "Point cloud")
            }
        }
    }

    /// The scan as a classified, survey-grade point cloud (LAS + enhanced PLY):
    /// downsample → normals → planes → floor/wall/ceiling/opening/furniture codes.
    func exportClassifiedCloud() {
        Task {
            guard let cloud = await engine.classifiedPointCloud(), !cloud.points.isEmpty else {
                alertItem = MacAlertItem(title: "Export", message: "There is no scan data to export.")
                return
            }
            ExportController.exportClassifiedCloud(cloud) { [weak self] result in
                self?.handleExportResult(result, label: "Classified point cloud (\(cloud.points.count) pts)")
            }
        }
    }

    /// The whole reconstruction as one world-space mesh (all live anchors merged).
    private func mergedMeshForExport() -> MeshData? {
        guard let merged = MeshData.merged(Array(meshStore.values)) else { return nil }
        return merged.vertices.count > 100_000
            ? MeshSimplifier.simplified(merged, cellSizeMeters: 0.01)
            : merged
    }

    func exportMeshOBJ() {
        guard let meshToExport = mergedMeshForExport() else {
            alertItem = MacAlertItem(title: "Export", message: "No mesh data available. Enable mesh streaming on the iPhone and rescan.")
            return
        }
        ExportController.exportMeshOBJ(mesh: meshToExport) { [weak self] result in
            self?.handleExportResult(result, label: "Mesh")
        }
    }

    func exportMeshUSDZ() {
        guard let meshToExport = mergedMeshForExport() else {
            alertItem = MacAlertItem(title: "Export", message: "No mesh data available. Enable mesh streaming on the iPhone and rescan.")
            return
        }
        ExportController.exportMeshUSDZ(mesh: meshToExport) { [weak self] result in
            self?.handleExportResult(result, label: "Mesh")
        }
    }

    private func handleExportResult(_ result: Result<URL, Error>, label: String) {
        switch result {
        case .success(let url):
            NSWorkspace.shared.activateFileViewerSelecting([url])
            alertItem = MacAlertItem(title: "\(label) Exported", message: "Saved to \(url.path) and revealed in Finder.")
        case .failure(let error):
            alertItem = MacAlertItem(title: "Export Failed", message: error.localizedDescription)
        }
    }

}

/// User-facing depth-confidence filter (maps to ARConfidenceLevel thresholds).
enum ConfidenceFilter: String, CaseIterable, Identifiable {
    case low
    case medium
    case high

    var id: String { rawValue }

    var minConfidence: UInt8 {
        switch self {
        case .low: return 0
        case .medium: return 1
        case .high: return 2
        }
    }

    var label: String {
        switch self {
        case .low: return "Low (all points)"
        case .medium: return "Medium"
        case .high: return "High (cleanest)"
        }
    }
}
