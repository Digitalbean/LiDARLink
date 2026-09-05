import SwiftUI
import LiDARLinkShared

/// Scan settings presented as a sheet from the phone's control bar.
struct SettingsView: View {
    @EnvironmentObject private var appState: PhoneAppState
    @Environment(\.dismiss) private var dismiss

    var body: some View {
        NavigationStack {
            Form {
                Section("Capture") {
                    Picker("Capture mode", selection: captureModeBinding) {
                        ForEach(CaptureMode.allCases) { mode in
                            Text(mode.label).tag(mode)
                        }
                    }
                    Picker("Target FPS", selection: fpsBinding) {
                        ForEach([5, 8, 10, 12, 15, 20], id: \.self) { fps in
                            Text("\(fps) fps").tag(fps)
                        }
                    }
                    Picker("Depth resolution", selection: resolutionBinding) {
                        Text("256×192").tag(256)
                        Text("192×144").tag(192)
                        Text("128×96").tag(128)
                    }
                    Toggle("Transmit color", isOn: colorBinding)
                    Toggle("Smoothed depth", isOn: smoothedDepthBinding)
                    Picker("Max depth", selection: maxDepthBinding) {
                        Text("2 m").tag(Float(2))
                        Text("4 m").tag(Float(4))
                        Text("8 m").tag(Float(8))
                    }
                    Toggle("Skip first seconds (calibration)", isOn: warmupBinding)
                    Toggle("Edge-preserving smoothing", isOn: bilateralBinding)
                    Toggle("Extra smoothing (still camera)", isOn: temporalBinding)
                }
                Section("Connection") {
                    Toggle("Prefer USB (wired)", isOn: wiredLinkBinding)
                    Text("Plug the iPhone into the Mac. Discovery and streaming use the cable instead of Wi-Fi — far more bandwidth, no dropped frames. Reconnect after changing.")
                        .font(.caption)
                        .foregroundStyle(.secondary)
                }
                Section("Mesh") {
                    Toggle("Mesh streaming (experimental)", isOn: meshBinding)
                }
                if appState.roomPlanSupported {
                    Section("Room structure") {
                        Toggle("RoomPlan (walls, doors, objects)", isOn: Binding(
                            get: { appState.roomPlanEnabled },
                            set: { appState.roomPlanEnabled = $0 }
                        ))
                        Text("Streams a parametric room to the Mac as a structural guide.")
                            .font(.caption)
                            .foregroundStyle(.secondary)
                    }
                }
                Section("Performance") {
                    Picker("Motion rejection", selection: motionSensitivityBinding) {
                        ForEach(MotionSensitivity.allCases, id: \.self) { sensitivity in
                            Text(sensitivity.label).tag(sensitivity)
                        }
                    }
                    Toggle("Adaptive performance", isOn: adaptiveBinding)
                    if appState.config.adaptivePerformanceEnabled {
                        LabeledContent("Quality tier",
                                       value: "\(appState.currentTier + 1) of \(ScanConfiguration.tiers.count)")
                            .font(.caption)
                            .foregroundStyle(.secondary)
                    }
                }
                if appState.scanState != .idle {
                    Section("Capture quality") {
                        LabeledContent("High-confidence depth", value: "\(appState.captureQuality.highConfidencePercent)%")
                        LabeledContent("Calibration filtered", value: "\(appState.captureQuality.calibrationRejections)")
                        LabeledContent("Tracking filtered", value: "\(appState.captureQuality.trackingRejections)")
                        LabeledContent("Fast-motion filtered", value: "\(appState.captureQuality.motionRejections)")
                        LabeledContent("Redundant filtered", value: "\(appState.captureQuality.redundantRejections)")
                    }
                    .font(.caption)
                }
                Section("About") {
                    LabeledContent("App version", value: PhoneAppState.appVersion)
                }
            }
            .navigationTitle("Scan Settings")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .confirmationAction) {
                    Button("Done") { dismiss() }
                }
            }
        }
        .presentationDetents([.medium, .large])
        .presentationDragIndicator(.visible)
    }

    // MARK: Bindings

    private var captureModeBinding: Binding<CaptureMode> {
        Binding(
            get: { appState.captureMode },
            set: { appState.applyCaptureMode($0) }
        )
    }

    private var fpsBinding: Binding<Int> {
        Binding(
            get: { appState.config.targetFPS },
            set: { fps in appState.updateConfig(appState.config.with { $0.targetFPS = fps }) }
        )
    }

    private var resolutionBinding: Binding<Int> {
        Binding(
            get: { appState.config.depthWidth },
            set: { newWidth in
                let newHeight = newWidth * 3 / 4
                appState.updateConfig(appState.config.with { $0.depthWidth = newWidth; $0.depthHeight = newHeight })
            }
        )
    }

    private var colorBinding: Binding<Bool> {
        Binding(
            get: { appState.config.colorEnabled },
            set: { enabled in appState.updateConfig(appState.config.with { $0.colorEnabled = enabled }) }
        )
    }

    private var smoothedDepthBinding: Binding<Bool> {
        Binding(
            get: { appState.config.useSmoothedDepth },
            set: { on in appState.updateConfig(appState.config.with { $0.useSmoothedDepth = on }) }
        )
    }

    private var maxDepthBinding: Binding<Float> {
        Binding(
            get: { appState.config.maxDepthMeters },
            set: { maxDepth in appState.updateConfig(appState.config.with { $0.maxDepthMeters = maxDepth }) }
        )
    }

    private var warmupBinding: Binding<Bool> {
        Binding(
            get: { appState.config.warmUpDuration > 0 },
            set: { on in appState.updateConfig(appState.config.with { $0.warmUpDuration = on ? 1.5 : 0 }) }
        )
    }

    private var bilateralBinding: Binding<Bool> {
        Binding(
            get: { appState.config.bilateralSmoothing },
            set: { on in appState.updateConfig(appState.config.with { $0.bilateralSmoothing = on }) }
        )
    }

    private var temporalBinding: Binding<Bool> {
        Binding(
            get: { appState.config.extraTemporalSmoothing },
            set: { on in appState.updateConfig(appState.config.with { $0.extraTemporalSmoothing = on }) }
        )
    }

    private var wiredLinkBinding: Binding<Bool> {
        Binding(
            get: { appState.config.preferWiredLink },
            set: { on in appState.updateConfig(appState.config.with { $0.preferWiredLink = on }) }
        )
    }

    private var meshBinding: Binding<Bool> {
        Binding(
            get: { appState.config.meshStreamingEnabled },
            set: { enabled in appState.updateConfig(appState.config.with { $0.meshStreamingEnabled = enabled }) }
        )
    }

    private var motionSensitivityBinding: Binding<MotionSensitivity> {
        Binding(
            get: { appState.config.motionSensitivity },
            set: { sensitivity in appState.updateConfig(appState.config.with { $0.motionSensitivity = sensitivity }) }
        )
    }

    private var adaptiveBinding: Binding<Bool> {
        Binding(
            get: { appState.config.adaptivePerformanceEnabled },
            set: { enabled in appState.updateConfig(appState.config.with { $0.adaptivePerformanceEnabled = enabled }) }
        )
    }
}
