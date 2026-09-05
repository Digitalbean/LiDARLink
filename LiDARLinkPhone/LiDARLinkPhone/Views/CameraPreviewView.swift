import ARKit
import SceneKit
import SwiftUI

/// Live camera preview backed by ARSCNView (which renders the AR camera feed).
struct CameraPreviewView: UIViewRepresentable {
    let session: ARSession?

    func makeUIView(context: Context) -> ARSCNView {
        let view = ARSCNView()
        view.scene = SCNScene()
        view.automaticallyUpdatesLighting = false
        view.preferredFramesPerSecond = 60
        return view
    }

    func updateUIView(_ uiView: ARSCNView, context: Context) {
        if uiView.session !== session {
            uiView.session = session ?? ARSession()
        }
    }
}
