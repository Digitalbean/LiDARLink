import Foundation

/// An incoming iPhone connection waiting for the user to approve or deny it.
final class PendingConnection: Identifiable {
    let id = UUID()
    let peer: PeerConnection
    var deviceName: String

    init(peer: PeerConnection) {
        self.peer = peer
        self.deviceName = "Unknown iPhone"
    }
}
