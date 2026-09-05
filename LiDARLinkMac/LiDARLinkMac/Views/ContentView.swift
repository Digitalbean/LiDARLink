import AppKit
import SwiftUI
import simd
import LiDARLinkShared

struct ContentView: View {
    @EnvironmentObject private var appState: MacAppState

    var body: some View {
        HSplitView {
            sidebar
                .frame(minWidth: 300, idealWidth: 340)
            scenePanel
                .frame(minWidth: 480)
        }
        .frame(minWidth: 900, minHeight: 640)
        .task { appState.onAppear() }
        .alert(item: $appState.alertItem) { item in
            Alert(title: Text(item.title),
                  message: Text(item.message),
                  dismissButton: .default(Text("OK")))
        }
    }

    // MARK: Sidebar (glass)

    private var sidebar: some View {
        ZStack {
            VisualEffectView(material: .sidebar, blendingMode: .behindWindow)
            ScrollView {
                VStack(alignment: .leading, spacing: 12) {
                    glassSection("Connection") { connectionContent }
                    glassSection("Diagnostics") { diagnosticsContent }
                    glassSection("Scan") { scanContent }
                    glassSection("View & Style") { viewStyleContent }
                    glassSection("Map") { mapContent }
                }
                .padding(12)
            }
        }
    }

    private func glassSection<Content: View>(_ title: String, @ViewBuilder content: () -> Content) -> some View {
        VStack(alignment: .leading, spacing: 8) {
            Text(title)
                .font(.caption.weight(.semibold))
                .foregroundStyle(.secondary)
                .textCase(.uppercase)
                .kerning(0.5)
            content()
        }
        .padding(10)
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 12))
    }

    // MARK: Connection

    private var connectionContent: some View {
        VStack(alignment: .leading, spacing: 8) {
            HStack(spacing: 8) {
                Circle()
                    .fill(serverColor)
                    .frame(width: 8, height: 8)
                Text(serverText)
                    .font(.callout)
            }
            if !appState.pendingConnections.isEmpty {
                VStack(alignment: .leading, spacing: 6) {
                    Text("Incoming connections")
                        .font(.caption)
                        .foregroundStyle(.secondary)
                    ForEach(appState.pendingConnections) { pending in
                        HStack {
                            VStack(alignment: .leading) {
                                Text(pending.deviceName)
                                    .font(.callout)
                                Text("wants to connect")
                                    .font(.caption)
                                    .foregroundStyle(.secondary)
                            }
                            Spacer()
                            Button {
                                appState.deny(pending)
                            } label: {
                                Label("Deny", systemImage: "xmark")
                            }
                            .buttonStyle(.borderless)
                            Button {
                                appState.approve(pending)
                            } label: {
                                Label("Approve", systemImage: "checkmark")
                            }
                            .buttonStyle(.borderedProminent)
                            .tint(.green)
                        }
                    }
                }
            }
            if let info = appState.connectionInfo {
                LabeledContent("iPhone", value: info.phoneName)
                LabeledContent("Since", value: info.startedAt.formatted(date: .omitted, time: .standard))
            } else {
                Text("Waiting for an iPhone to connect…")
                    .font(.caption)
                    .foregroundStyle(.secondary)
            }
            Toggle("Auto-approve iPhone", isOn: $appState.autoApprove)
                .toggleStyle(.switch)

            Divider()
            Button {
                appState.toggleUSBBridge()
            } label: {
                Label(appState.usbBridgeActive ? "Stop USB Bridge" : "Connect via USB",
                      systemImage: "cable.connector")
            }
            if appState.usbBridgeActive, !appState.usbStatus.isEmpty {
                Text(appState.usbStatus)
                    .font(.caption)
                    .foregroundStyle(.secondary)
            }
        }
    }

    private var serverText: String {
        switch appState.serverState {
        case .idle: return "Server stopped"
        case .advertising(let name): return "Advertising as \(name)"
        case .failed(let reason): return "Server failed: \(reason)"
        }
    }

    private var serverColor: Color {
        switch appState.serverState {
        case .advertising: return .green
        case .failed: return .red
        case .idle: return .gray
        }
    }

    // MARK: Diagnostics

    private var diagnosticsContent: some View {
        VStack(alignment: .leading, spacing: 6) {
            LabeledContent("Receive rate", value: String(format: "%.1f fps", appState.stats.receivedFPS))
            LabeledContent("Frames received", value: "\(appState.stats.frameCount)")
            LabeledContent("Color frames", value: "\(appState.stats.colorFrameCount)")
            LabeledContent("Cloud points", value: "\(appState.mapPointCount)")
            LabeledContent("New pts / frame", value: "\(appState.lastFrameNewPoints)")
            LabeledContent("Map occupancy", value: "\(appState.progressiveMapOccupiedCells) cells")
            LabeledContent("Dropped frames", value: "\(appState.stats.droppedFrames)")
            LabeledContent("Duplicates", value: "\(appState.stats.duplicateFrames)")
            LabeledContent("Reordered", value: "\(appState.stats.reorderedFrames)")
            LabeledContent("Corrupt frames", value: "\(appState.stats.corruptFrames)")
            LabeledContent("Incomplete frames", value: "\(appState.stats.incompleteFrames)")
            LabeledContent("Throughput", value: formatBytes(appState.stats.throughputBytesPerSecond, perSecond: true))
            LabeledContent("Bytes received", value: formatBytes(Double(appState.stats.bytesReceived), perSecond: false))
            LabeledContent("Mesh chunks", value: "\(appState.stats.meshChunksReceived)")
            LabeledContent("Mesh anchors", value: "\(appState.meshAnchorCount)")
        }
        .monospacedDigit()
    }

    // MARK: Scan

    private var scanContent: some View {
        VStack(alignment: .leading, spacing: 8) {
            if appState.connectionInfo != nil {
                remoteScanControls
                Divider()
            }
            switch appState.objectStage {
            case .idle:
                EmptyView()
            case .receiving(let n, let total):
                Label("Receiving object photos \(n)/\(total)", systemImage: "photo.stack").font(.caption)
            case .processing(let f):
                Label("Photogrammetry \(Int(f * 100))%", systemImage: "cube.transparent").font(.caption)
            case .done:
                Label("Object model ready", systemImage: "checkmark.circle").font(.caption).foregroundStyle(.green)
            case .failed(let reason):
                Label(reason, systemImage: "exclamationmark.triangle").font(.caption).foregroundStyle(.orange)
            }
            Button {
                appState.clearScan()
            } label: {
                Label("Clear Scan", systemImage: "trash")
            }
            Button {
                appState.exportPointCloud()
            } label: {
                Label("Export Point Cloud (PLY)", systemImage: "square.and.arrow.up")
            }
            Button {
                appState.exportClassifiedCloud()
            } label: {
                Label("Export Classified Cloud (LAS)…", systemImage: "square.3.layers.3d")
            }
            .help("The scan as a classified, oriented point cloud — LAS 1.2 + enhanced PLY, with floor/wall/ceiling/opening/furniture codes for ReCap, CloudCompare, Revit.")
            Button {
                appState.exportMeshOBJ()
            } label: {
                Label("Export Mesh (OBJ)", systemImage: "square.and.arrow.up")
            }
            Button {
                appState.exportMeshUSDZ()
            } label: {
                Label("Export Mesh (USDZ)", systemImage: "square.and.arrow.up")
            }
        }
    }

    /// Drives the phone's scan lifecycle and a couple of its capture settings
    /// remotely — the phone still has the final say (it applies whatever it
    /// receives the same way its own on-device controls would).
    private var remoteScanControls: some View {
        VStack(alignment: .leading, spacing: 8) {
            HStack(spacing: 8) {
                switch appState.remotePhoneScanState {
                case .running:
                    Button { appState.pauseRemoteScan() } label: { Label("Pause", systemImage: "pause.fill") }
                    Button { appState.stopRemoteScan() } label: { Label("Stop", systemImage: "stop.fill") }
                case .paused:
                    Button { appState.resumeRemoteScan() } label: { Label("Resume", systemImage: "play.fill") }
                        .buttonStyle(.borderedProminent)
                    Button { appState.stopRemoteScan() } label: { Label("Stop", systemImage: "stop.fill") }
                case .idle, .unknown:
                    Button { appState.startRemoteScan() } label: { Label("Start Scan", systemImage: "play.fill") }
                        .buttonStyle(.borderedProminent)
                }
            }
            .help("Same as tapping the equivalent button on the phone.")

            Picker("Capture mode", selection: Binding(
                get: { appState.remoteCaptureModeName },
                set: { appState.applyRemoteCaptureMode($0) }
            )) {
                ForEach(MacAppState.remoteCaptureModes, id: \.name) { mode in
                    Text(mode.label).tag(mode.name)
                }
            }
            .pickerStyle(.menu)
            .help("Sent to the phone; takes effect immediately if it's mid-scan, same as switching modes there.")

            Toggle("RoomPlan on phone", isOn: Binding(
                get: { appState.remoteRoomPlanEnabled },
                set: { appState.applyRemoteRoomPlan($0) }
            ))
            .toggleStyle(.switch)

            Picker("Max depth", selection: Binding(
                get: { appState.remoteMaxDepthMeters },
                set: { appState.applyRemoteMaxDepth($0) }
            )) {
                Text("2 m").tag(Float(2))
                Text("4 m").tag(Float(4))
                Text("8 m").tag(Float(8))
            }
            .pickerStyle(.menu)
            .help("Same blanket range clamp as the phone's own Max depth setting — applied on top of whatever the capture mode set.")
        }
    }

    // MARK: View & Style

    private var viewStyleContent: some View {
        VStack(alignment: .leading, spacing: 8) {
            Toggle("Show mesh", isOn: Binding(
                get: { appState.showMesh },
                set: { _ in appState.toggleMeshDisplay() }
            ))
            .toggleStyle(.switch)

            Toggle("Mesh wireframe", isOn: Binding(
                get: { appState.meshWireframe },
                set: { appState.applyMeshWireframe($0) }
            ))
            .toggleStyle(.switch)
            .disabled(!appState.showMesh)

            HStack(spacing: 8) {
                Button {
                    appState.recenterCamera()
                } label: {
                    Label("Recenter", systemImage: "viewfinder")
                }
                Button {
                    appState.sceneController.resetView()
                } label: {
                    Label("Reset View", systemImage: "arrow.counterclockwise")
                }
            }

            Toggle(isOn: Binding(
                get: { appState.followingiPhone },
                set: { _ in appState.toggleFollowiPhone() }
            )) {
                Label("Follow iPhone", systemImage: "iphone.gen3")
            }
            .toggleStyle(.switch)
            .disabled(appState.connectionInfo == nil)
            .help("Ride the live iPhone pose — see the reconstruction from where the phone is. Any WASD/mouse input hands control back.")

            Button {
                appState.levelToFloor()
            } label: {
                Label("Level to floor", systemImage: "level")
            }

            Toggle("Measure", isOn: Binding(
                get: { appState.measuring },
                set: { appState.applyMeasuring($0) }
            ))
            .toggleStyle(.switch)
            if appState.measuring {
                HStack {
                    if let d = appState.measureDistanceMeters {
                        Text(d < 1 ? String(format: "%.0f cm", d * 100) : String(format: "%.2f m", d))
                            .font(.headline.monospacedDigit())
                    } else {
                        Text("Click two points").font(.caption).foregroundStyle(.secondary)
                    }
                    Spacer()
                    Button("Clear") { appState.clearMeasurement() }
                        .buttonStyle(.borderless)
                }
            }

            Toggle("Lock to floor", isOn: Binding(
                get: { appState.floorLocked },
                set: { appState.applyFloorLocked($0) }
            ))
            .toggleStyle(.switch)

            HStack(spacing: 8) {
                Text("Eye height")
                Slider(value: Binding(
                    get: { appState.eyeHeightMeters },
                    set: { appState.applyEyeHeight($0) }
                ), in: 0.3...2.5, step: 0.1)
                .disabled(!appState.floorLocked)
                Text(String(format: "%.1f m", appState.eyeHeightMeters))
                    .monospacedDigit()
            }
            Text("Click the scene to look · WASD move · Q/E down/up · Shift faster · scroll speed · Esc release")
                .font(.caption2)
                .foregroundStyle(.secondary)

            Toggle("World axes & grid", isOn: Binding(
                get: { appState.showWorldOverlay },
                set: { appState.applyWorldOverlay($0) }
            ))
            .toggleStyle(.switch)

            Toggle("Eye-dome lighting", isOn: $appState.eyeDomeLightingEnabled)
                .toggleStyle(.switch)
                .help("Screen-space shading from the depth buffer alone — makes the point cloud read as a solid shape instead of scattered dots. Experimental: first custom render pass in the app.")

            Divider()

            Picker("Color mode", selection: Binding(
                get: { appState.pointColorMode },
                set: { appState.applyPointColorMode($0) }
            )) {
                Text("Camera color").tag(PointColorMode.cameraColor)
                Text("Height gradient").tag(PointColorMode.heightGradient)
                Text("Confidence").tag(PointColorMode.confidence)
                Text("Solid color").tag(PointColorMode.solid(appState.solidColorRGB))
                Text("Lit (shaded)").tag(PointColorMode.lit)
            }
            .pickerStyle(.menu)

            if case .solid = appState.pointColorMode {
                ColorPicker("Point color", selection: Binding(
                    get: { appState.solidColor },
                    set: { appState.applySolidColor($0) }
                ))
            }

            HStack(spacing: 8) {
                Text("Point size")
                Slider(value: Binding(
                    get: { appState.pointSize },
                    set: { appState.applyPointSize($0) }
                ), in: 1...10, step: 0.5)
                Text(String(format: "%.1f", appState.pointSize))
                    .monospacedDigit()
                    .frame(width: 30, alignment: .trailing)
            }

            ColorPicker("Background", selection: Binding(
                get: { appState.backgroundColor },
                set: { appState.applyBackgroundColor($0) }
            ))
        }
    }

    // MARK: Map

    private var mapContent: some View {
        VStack(alignment: .leading, spacing: 8) {
            Toggle("Loop closure (pose graph)", isOn: Binding(
                get: { appState.poseGraphEnabled },
                set: { appState.applyPoseGraph($0) }
            ))
            .toggleStyle(.switch)
            .help("Runs a keyframe pose graph alongside the dense cloud and warps it when a revisit is detected — the point cloud stops drifting on big scans. Resets the current scan.")
            if appState.poseGraphEnabled, appState.poseGraphKeyframes > 0 {
                Text("\(appState.poseGraphKeyframes) keyframes · \(appState.poseGraphLoops) loop closures")
                    .font(.caption).foregroundStyle(.secondary).monospacedDigit()
            }

            Toggle("Edge filter", isOn: $appState.edgeFilterEnabled)
                .toggleStyle(.switch)
                .help("Drops flying / mixed pixels at depth discontinuities before they enter the cloud. Conservative — only step edges, never smooth gradients.")

            Toggle("Free-space carving", isOn: $appState.freeSpaceCarvingEnabled)
                .toggleStyle(.switch)
                .help("Removes points the camera later sees straight through — drift ghosts, and anything that has moved away since (a person, a shifted blanket). A continuously-observed surface is unaffected.")

            Picker("Confidence filter", selection: Binding(
                get: { appState.confidenceFilter },
                set: { appState.applyConfidenceFilter($0) }
            )) {
                ForEach(ConfidenceFilter.allCases) { filter in
                    Text(filter.label).tag(filter)
                }
            }
            .pickerStyle(.menu)

            if let room = appState.roomStructure {
                Toggle("RoomPlan overlay", isOn: Binding(
                    get: { appState.showRoomStructure },
                    set: { appState.showRoomStructure = $0 }
                ))
                .toggleStyle(.switch)
                Text("\(room.walls.count) walls · \(room.openings.count) openings · \(room.objects.count) objects\(room.isFinal ? " · final" : "")")
                    .font(.caption).foregroundStyle(.secondary)
            }

            Divider()

            Toggle("Progressive map (deduplicate)", isOn: Binding(
                get: { appState.progressiveMapEnabled },
                set: { appState.applyProgressiveMap($0) }
            ))
            .toggleStyle(.switch)

            Picker("Voxel size", selection: Binding(
                get: { appState.voxelSizeCm },
                set: { appState.applyVoxelSize($0) }
            )) {
                Text("Fine (0.5 cm)").tag(0.5)
                Text("Standard (1 cm)").tag(1.0)
                Text("Coarse (2 cm)").tag(2.0)
            }
            .pickerStyle(.menu)

            Toggle("Auto-clean (remove noise)", isOn: Binding(
                get: { appState.autoClean },
                set: { appState.applyAutoClean($0) }
            ))
            .toggleStyle(.switch)

            Toggle("Frame alignment (ICP)", isOn: Binding(
                get: { appState.poseRefinementEnabled },
                set: { appState.applyPoseRefinement($0) }
            ))
            .toggleStyle(.switch)
            .disabled(!appState.progressiveMapEnabled)

            Toggle("TSDF fusion (experimental)", isOn: Binding(
                get: { appState.tsdfEnabled },
                set: { appState.applyTSDF($0) }
            ))
            .toggleStyle(.switch)

            if appState.poseRefinementEnabled, appState.poseCorrectionMillimetres > 0 {
                Text(String(format: "correcting %.1f mm", appState.poseCorrectionMillimetres))
                    .font(.caption.monospacedDigit())
                    .foregroundStyle(.secondary)
            }
        }
    }

    // MARK: Scene panel

    private var scenePanel: some View {
        ZStack(alignment: .topLeading) {
            PointCloudSceneView(controller: appState.sceneController)
            HStack(alignment: .top) {
                statusPill
                Spacer()
                floatingControls
            }
            .padding(10)
            VStack {
                Spacer()
                bottomHint
            }
            .padding(8)
        }
    }

    private var statusPill: some View {
        HStack(spacing: 10) {
            Text("\(appState.mapPointCount) pts")
                .monospacedDigit()
            Text("+\(appState.lastFrameNewPoints) new")
                .monospacedDigit()
            Text(String(format: "%.1f fps", appState.stats.receivedFPS))
                .monospacedDigit()
            if appState.cloudFull {
                Label("Point limit reached — enlarge voxel size or Clear", systemImage: "exclamationmark.circle")
                    .foregroundStyle(.orange)
            } else if appState.mapSaturated {
                Label("Map saturating — decimating", systemImage: "arrow.down.right")
                    .foregroundStyle(.orange)
            }
        }
        .font(.caption)
        .padding(.horizontal, 10)
        .padding(.vertical, 6)
        .glassPanel()
    }

    private var floatingControls: some View {
        HStack(spacing: 6) {
            sceneButton("Recenter", systemImage: "viewfinder") {
                appState.recenterCamera()
            }
            sceneButton("Reset", systemImage: "arrow.counterclockwise") {
                appState.sceneController.resetView()
            }
            Menu {
                Button("Camera color") { appState.applyPointColorMode(.cameraColor) }
                Button("Height gradient") { appState.applyPointColorMode(.heightGradient) }
                Button("Confidence") { appState.applyPointColorMode(.confidence) }
                Button("Solid color") { appState.applyPointColorMode(.solid(appState.solidColorRGB)) }
                Button("Lit (shaded)") { appState.applyPointColorMode(.lit) }
            } label: {
                Label("Color", systemImage: "paintpalette")
            }
            .menuStyle(.borderlessButton)
            .fixedSize()
        }
        .font(.callout)
        .padding(8)
        .glassPanel()
    }

    private func sceneButton(_ title: String, systemImage: String, action: @escaping () -> Void) -> some View {
        Button(action: action) {
            Label(title, systemImage: systemImage)
        }
        .buttonStyle(.plain)
    }

    private var bottomHint: some View {
        HStack {
            Text("Click to look around · WASD to walk · Shift to move faster · Esc to release the mouse")
                .font(.caption)
                .foregroundStyle(.secondary)
            Spacer()
        }
        .padding(.horizontal, 10)
        .padding(.vertical, 6)
        .glassPanel()
    }

    // MARK: Formatting

    private func formatBytes(_ value: Double, perSecond: Bool) -> String {
        let units = ["B", "KB", "MB", "GB"]
        var v = value
        var unit = 0
        while v >= 1024, unit < units.count - 1 {
            v /= 1024
            unit += 1
        }
        let suffix = perSecond ? "\(units[unit])/s" : units[unit]
        return String(format: "%.1f %@", v, suffix)
    }
}

// MARK: - Glass helpers

private struct VisualEffectView: NSViewRepresentable {
    var material: NSVisualEffectView.Material
    var blendingMode: NSVisualEffectView.BlendingMode

    func makeNSView(context: Context) -> NSVisualEffectView {
        let view = NSVisualEffectView()
        view.material = material
        view.blendingMode = blendingMode
        view.state = .active
        return view
    }

    func updateNSView(_ nsView: NSVisualEffectView, context: Context) {
        nsView.material = material
        nsView.blendingMode = blendingMode
    }
}

private struct GlassPanel: ViewModifier {
    // macOS 26 is the deployment floor — Liquid Glass everywhere.
    func body(content: Content) -> some View {
        content.glassEffect()
    }
}

private extension View {
    func glassPanel() -> some View {
        modifier(GlassPanel())
    }
}
