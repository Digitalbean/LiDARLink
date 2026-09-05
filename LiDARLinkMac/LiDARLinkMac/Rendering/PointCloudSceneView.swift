import AppKit
import SceneKit
import SwiftUI

/// Hosts the SceneKit view and wires first-person walk controls:
///  - Hold the RIGHT mouse button and move to look around.
///  - WASD to move, Q/E for down/up (when not floor-locked), Shift to move faster.
///  - Scroll to change move speed. The left mouse button is unused by the camera.
struct PointCloudSceneView: NSViewRepresentable {
    let controller: PointCloudSceneController

    func makeNSView(context: Context) -> PointCloudSCNView {
        let view = PointCloudSCNView(frame: .zero, options: nil)
        view.scene = controller.scene
        view.allowsCameraControl = false
        view.autoenablesDefaultLighting = true
        view.antialiasingMode = .multisampling4X
        view.backgroundColor = NSColor(calibratedWhite: 0.07, alpha: 1)
        view.controller = controller
        view.technique = controller.eyeDomeLightingEnabled ? PointCloudSceneController.eyeDomeLightingTechnique : nil
        controller.onEyeDomeLightingChanged = { [weak view] technique in
            view?.technique = technique
        }
        return view
    }

    func updateNSView(_ nsView: PointCloudSCNView, context: Context) {
        nsView.controller = controller
    }
}

final class PointCloudSCNView: SCNView {
    weak var controller: PointCloudSceneController?

    /// Held movement keys (by keyCode): W A S D Q E.
    private static let moveKeys: Set<UInt16> = [13, 0, 1, 2, 12, 14]
    private var heldKeys: Set<UInt16> = []
    private var sprinting = false
    private var looking = false
    private var walkTimer: Timer?
    private var lastTick = CFAbsoluteTimeGetCurrent()

    override var acceptsFirstResponder: Bool { true }

    override func viewDidMoveToWindow() {
        super.viewDidMoveToWindow()
        if window != nil {
            window?.makeFirstResponder(self)
            startWalkLoop()
        } else {
            stopInput()
            walkTimer?.invalidate()
            walkTimer = nil
        }
    }

    // MARK: Mouse — look on right-drag only

    override func mouseDown(with event: NSEvent) {
        window?.makeFirstResponder(self)
        // Left button is inert for the camera; in measure mode it drops a point.
        guard let controller, controller.isMeasuring else { return }
        let local = convert(event.locationInWindow, from: nil)
        let topLeft = CGPoint(x: local.x, y: bounds.height - local.y)
        controller.placeMeasurePoint(viewportPoint: topLeft, viewportSize: bounds.size)
    }

    override func rightMouseDown(with event: NSEvent) {
        window?.makeFirstResponder(self)
        looking = true
        NSCursor.hide()
    }

    override func rightMouseDragged(with event: NSEvent) {
        guard looking, let controller else { return }
        controller.walkLook(deltaX: Float(event.deltaX), deltaY: Float(event.deltaY))
    }

    override func rightMouseUp(with event: NSEvent) {
        looking = false
        NSCursor.unhide()
    }

    override func scrollWheel(with event: NSEvent) {
        guard let controller else { return }
        let factor = Float(1 - event.scrollingDeltaY * 0.02)
        controller.walkSpeed = min(max(controller.walkSpeed * factor, 0.15), 20)
    }

    // Swallow trackpad pinch/twist/swipe so they can never nudge the camera.
    override func magnify(with event: NSEvent) {}
    override func rotate(with event: NSEvent) {}
    override func swipe(with event: NSEvent) {}

    // MARK: Keyboard

    override func keyDown(with event: NSEvent) {
        if Self.moveKeys.contains(event.keyCode) {
            heldKeys.insert(event.keyCode)
            updateWalkInput()
            return
        }
        super.keyDown(with: event)
    }

    override func keyUp(with event: NSEvent) {
        if heldKeys.remove(event.keyCode) != nil {
            updateWalkInput()
            return
        }
        super.keyUp(with: event)
    }

    override func flagsChanged(with event: NSEvent) {
        let nowSprinting = event.modifierFlags.contains(.shift)
        if nowSprinting != sprinting {
            sprinting = nowSprinting
            updateWalkInput()
        }
        super.flagsChanged(with: event)
    }

    override func resignFirstResponder() -> Bool {
        stopInput()
        return super.resignFirstResponder()
    }

    private func stopInput() {
        heldKeys.removeAll()
        sprinting = false
        if looking { NSCursor.unhide() }
        looking = false
        controller?.setWalkInput(strafe: 0, forward: 0, lift: 0, sprint: false)
    }

    private func updateWalkInput() {
        let forward: Float = (heldKeys.contains(13) ? 1 : 0) - (heldKeys.contains(1) ? 1 : 0)  // W - S
        let strafe: Float = (heldKeys.contains(2) ? 1 : 0) - (heldKeys.contains(0) ? 1 : 0)    // D - A
        let lift: Float = (heldKeys.contains(14) ? 1 : 0) - (heldKeys.contains(12) ? 1 : 0)    // E - Q
        controller?.setWalkInput(strafe: strafe, forward: forward, lift: lift, sprint: sprinting)
    }

    // MARK: Walk render loop

    private func startWalkLoop() {
        guard walkTimer == nil else { return }
        lastTick = CFAbsoluteTimeGetCurrent()
        let timer = Timer(timeInterval: 1.0 / 60.0, repeats: true) { [weak self] _ in
            guard let self else { return }
            let now = CFAbsoluteTimeGetCurrent()
            let dt = Float(now - self.lastTick)
            self.lastTick = now
            self.controller?.stepWalk(deltaSeconds: dt)
        }
        RunLoop.main.add(timer, forMode: .common)
        walkTimer = timer
    }

    deinit {
        walkTimer?.invalidate()
        if looking { NSCursor.unhide() }
    }
}
