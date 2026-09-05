import SwiftUI
import LiDARLinkShared

struct ContentView: View {
    @EnvironmentObject private var appState: PhoneAppState
    @State private var showingSettings = false

    var body: some View {
        ZStack {
            CameraPreviewView(session: appState.arSession)
                .ignoresSafeArea()

            VStack(spacing: 0) {
                statusHeader
                if appState.scanState == .running {
                    qualityBanner
                }
                if appState.captureMode == .object && appState.scanState == .running {
                    coverageOverlay
                }
                Spacer()
                if !appState.isScanningSupported {
                    unsupportedBanner
                } else if case .discovering = appState.connectionState {
                    macList
                }
                controlBar
            }
        }
        .task { appState.onAppear() }
        .alert(item: $appState.errorItem) { item in
            Alert(title: Text("LiDAR Link"),
                  message: Text(item.message),
                  dismissButton: .default(Text("OK")))
        }
        .sheet(isPresented: $showingSettings) {
            SettingsView()
                .environmentObject(appState)
        }
    }

    // MARK: Status header

    private var statusHeader: some View {
        ScrollView(.horizontal, showsIndicators: false) {
            HStack(spacing: Design.spaceS) {
                StatusChip(text: connectionText, color: connectionColor)
                StatusChip(text: scanText, color: scanColor)
                StatusChip(text: String(format: "%.1f fps", appState.sendStats.sentFPS),
                           color: Design.Camera.chipFill)
                StatusChip(text: "dropped \(appState.sendStats.droppedFrames)",
                           color: Design.Camera.chipFill)
                if appState.sendStats.rejectedFrames > 0 {
                    StatusChip(text: "rejected \(appState.sendStats.rejectedFrames)",
                               color: .yellow.opacity(0.7))
                }
                StatusChip(text: appState.sendStats.depthResolution,
                           color: Design.Camera.chipFill)
            }
            .padding(.horizontal, Design.spaceM)
            .padding(.top, Design.spaceS)
        }
    }

    private var connectionText: String {
        switch appState.connectionState {
        case .idle: return "Not connected"
        case .discovering: return "Searching for Mac…"
        case .connecting(let name): return "Connecting to \(name)…"
        case .connected(let name): return "Connected: \(name)"
        case .handshakeFailed: return "Connection failed"
        case .reconnecting: return "Reconnecting…"
        case .disconnected: return "Disconnected"
        }
    }

    private var connectionColor: Color {
        if case .connected = appState.connectionState { return Design.success }
        if case .handshakeFailed = appState.connectionState { return Design.error }
        return Design.warning
    }

    private var scanText: String {
        switch appState.scanState {
        case .idle: return "scan: idle"
        case .running: return "scan: live"
        case .paused: return "scan: paused"
        }
    }

    private var scanColor: Color {
        switch appState.scanState {
        case .running: return Design.success
        case .paused: return Design.warning
        case .idle: return Design.Camera.chipFill
        }
    }

    private var qualityBanner: some View {
        let presentation = qualityPresentation
        return HStack(spacing: Design.spaceS) {
            Image(systemName: presentation.symbol)
            Text(presentation.text)
                .font(.subheadline.weight(.semibold))
            Spacer()
            if appState.captureQuality.highConfidencePercent > 0 {
                Text("\(appState.captureQuality.highConfidencePercent)% high confidence")
                    .font(.caption.monospacedDigit())
            }
        }
        .foregroundStyle(.white)
        .padding(.horizontal, Design.spaceM)
        .padding(.vertical, Design.spaceS)
        .background(presentation.color.opacity(0.9), in: Capsule())
        .padding(.horizontal, Design.spaceM)
        .padding(.top, Design.spaceS)
        .accessibilityElement(children: .combine)
    }

    private var qualityPresentation: (text: String, symbol: String, color: Color) {
        switch appState.captureQuality.guidance {
        case .calibrating:
            return ("Calibrating", "scope", Design.warning)
        case .trackingLimited(let reason):
            return ("Tracking limited: \(reason)", "exclamationmark.triangle.fill", Design.warning)
        case .moveSlower:
            return ("Move slower", "tortoise.fill", Design.warning)
        case .lowLight:
            return ("Low light — depth active, color limited", "moon.fill", .indigo)
        case .scanningWell:
            return ("Scanning well", "checkmark.circle.fill", Design.success)
        }
    }

    // MARK: Unsupported banner

    private var unsupportedBanner: some View {
        VStack(spacing: Design.spaceS) {
            Image(systemName: "exclamationmark.triangle.fill")
                .font(.title2)
                .foregroundStyle(Design.warning)
            Text(appState.supportMessage ?? "LiDAR is not available on this device.")
                .font(.footnote)
                .foregroundStyle(Design.Camera.text)
                .multilineTextAlignment(.center)
        }
        .padding(Design.spaceL)
        .frame(maxWidth: .infinity)
        .background(Design.Camera.panel, in: RoundedRectangle(cornerRadius: Design.radiusL, style: .continuous))
        .padding(.horizontal, Design.spaceM)
    }

    // MARK: Mac list

    private var macList: some View {
        VStack(spacing: Design.spaceS) {
            Text("LiDAR Link Mac apps found:")
                .font(.footnote.weight(.medium))
                .foregroundStyle(Design.Camera.text)
            if appState.discoveredMacs.isEmpty {
                Text("None yet — make sure the Mac app is open and its server is running.")
                    .font(.caption)
                    .foregroundStyle(Design.Camera.textSecondary)
                    .multilineTextAlignment(.center)
            } else {
                ScrollView {
                    VStack(spacing: Design.spaceS) {
                        ForEach(appState.discoveredMacs) { mac in
                            Button {
                                appState.connect(to: mac)
                            } label: {
                                HStack(spacing: Design.spaceM) {
                                    Image(systemName: "macmini")
                                        .font(.title3)
                                        .foregroundStyle(Design.accent)
                                    Text(mac.name)
                                        .font(.callout.weight(.medium))
                                        .foregroundStyle(Design.Camera.text)
                                    Spacer()
                                    Image(systemName: "chevron.right")
                                        .font(.caption.weight(.semibold))
                                        .foregroundStyle(Design.Camera.textSecondary)
                                }
                                .padding(Design.spaceM)
                                .background(Design.Camera.scrim, in: RoundedRectangle(cornerRadius: Design.radiusM, style: .continuous))
                            }
                            .buttonStyle(.plain)
                        }
                    }
                }
                .frame(maxHeight: 220)
            }
        }
        .padding(Design.spaceM)
        .background(Design.Camera.panel, in: RoundedRectangle(cornerRadius: Design.radiusL, style: .continuous))
        .padding(.horizontal, Design.spaceM)
    }

    // MARK: Control bar

    private var canScan: Bool {
        if case .connected = appState.connectionState { return appState.isScanningSupported }
        return false
    }

    private var controlBar: some View {
        VStack(spacing: Design.spaceM) {
            connectionControls
            scanControls
        }
        .padding(Design.spaceM)
        .padding(.bottom, Design.spaceS)
        .background(.ultraThinMaterial)
    }

    private var connectionControls: some View {
        HStack(spacing: Design.spaceS) {
            switch appState.connectionState {
            case .connected:
                Button("Disconnect") { appState.disconnect() }
            case .idle, .disconnected:
                Button("Discover Mac") { appState.startBrowsing() }
                    .buttonStyle(.borderedProminent)
                    .tint(Design.accent)
            case .discovering, .connecting, .reconnecting:
                HStack(spacing: Design.spaceS) {
                    ProgressView()
                        .controlSize(.small)
                    Text("Working…")
                        .font(.caption)
                        .foregroundStyle(.secondary)
                }
            case .handshakeFailed:
                Button("Try Again") { appState.startBrowsing() }
                    .buttonStyle(.borderedProminent)
                    .tint(Design.accent)
            }
            Spacer()
            if appState.isCalibrating {
                StatusChip(text: "Calibrating…", color: .orange.opacity(0.9))
            }
            if appState.sendStats.colorOn {
                StatusChip(text: "color \(appState.sendStats.colorProducedCount)", color: .blue.opacity(0.85))
            }
            if appState.sendStats.meshOn {
                StatusChip(text: "mesh on", color: .purple.opacity(0.85))
            }
            Button {
                showingSettings = true
            } label: {
                Image(systemName: "gearshape.fill")
                    .font(.title3)
                    .foregroundStyle(.primary)
            }
            .buttonStyle(.plain)
            .accessibilityLabel("Scan Settings")
        }
    }

    private var scanControls: some View {
        VStack(spacing: Design.spaceS) {
            HStack(spacing: Design.spaceXL) {
                if appState.scanState == .idle {
                    startScanButton
                } else if appState.scanState == .running {
                    Button {
                        appState.pauseScan()
                    } label: {
                        Label("Pause", systemImage: "pause.fill")
                            .font(.callout.weight(.medium))
                            .frame(minWidth: 90)
                    }
                    .buttonStyle(.bordered)
                    .tint(.white)
                    stopButton
                } else {
                    Button {
                        appState.resumeScan()
                    } label: {
                        Label("Resume", systemImage: "play.fill")
                            .font(.callout.weight(.medium))
                            .frame(minWidth: 90)
                    }
                    .buttonStyle(.borderedProminent)
                    .tint(Design.success)
                    stopButton
                }
                Spacer()
            }
            if !canScan, appState.isScanningSupported {
                Text("Connect to a Mac to scan")
                    .font(.caption)
                    .foregroundStyle(.secondary)
            }
        }
    }

    private var startScanButton: some View {
        Button {
            appState.startScan()
        } label: {
            ZStack {
                Circle()
                    .strokeBorder(.white.opacity(0.9), lineWidth: 3)
                    .frame(width: 72, height: 72)
                Circle()
                    .fill(Design.error)
                    .frame(width: 54, height: 54)
            }
        }
        .buttonStyle(.plain)
        .disabled(!canScan)
        .opacity(canScan ? 1 : 0.45)
        .accessibilityLabel("Start Scan")
    }

    // MARK: Coverage ring

    private var coverageOverlay: some View {
        VStack(spacing: Design.spaceS) {
            CoverageRing(covered: appState.coveredSectors)
            Text("orbit the object")
                .font(Design.statLabel)
                .foregroundStyle(Design.Camera.textSecondary)
        }
        .frame(maxWidth: .infinity)
        .padding(.top, Design.spaceM)
    }

    private var stopButton: some View {
        Button {
            appState.stopScan()
        } label: {
            Label("Stop", systemImage: "stop.fill")
                .font(.callout.weight(.medium))
                .frame(minWidth: 90)
        }
        .buttonStyle(.borderedProminent)
        .tint(Design.error)
    }
}

// MARK: - Helpers

extension ScanConfiguration {
    /// Returns a copy with the mutation applied (used by Binding setters).
    func with(_ mutate: (inout ScanConfiguration) -> Void) -> ScanConfiguration {
        var copy = self
        mutate(&copy)
        return copy
    }
}

// MARK: - Coverage ring

/// 12-sector orbit coverage indicator shown during object scans.
private struct CoverageRing: View {
    let covered: Set<Int>

    var body: some View {
        ZStack {
            ForEach(0..<12, id: \.self) { i in
                Circle()
                    .trim(from: Double(i) / 12, to: Double(i + 1) / 12)
                    .stroke(covered.contains(i) ? Color.green : Color.white.opacity(0.35),
                            style: StrokeStyle(lineWidth: 6, lineCap: .butt))
                    .frame(width: 64, height: 64)
                    .rotationEffect(.degrees(-90))
            }
        }
    }
}
