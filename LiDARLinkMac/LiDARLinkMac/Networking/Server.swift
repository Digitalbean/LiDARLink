import Foundation
import Network
import LiDARLinkShared

/// Advertises `_lidarlink._tcp` on the local network and hands incoming
/// connections to the app for explicit user approval.
final class Server {
    enum ServerState: Equatable {
        case idle
        case advertising(String)
        case failed(String)
    }

    private var listener: NWListener?
    var onIncomingPeer: ((PeerConnection) -> Void)?
    var onStateChange: ((ServerState) -> Void)?

    func start(serviceName: String) throws {
        let params = NWParameters.tcp
        if let tcp = params.defaultProtocolStack.transportProtocol as? NWProtocolTCP.Options {
            tcp.noDelay = true   // ship small control messages without Nagle delay
        }
        let listener = try NWListener(using: params)
        listener.service = NWListener.Service(name: serviceName, type: "_lidarlink._tcp", domain: "local")
        self.listener = listener
        listener.stateUpdateHandler = { [weak self] state in
            switch state {
            case .ready:
                self?.onStateChange?(.advertising(serviceName))
            case .failed(let error):
                self?.onStateChange?(.failed(error.localizedDescription))
            default:
                break
            }
        }
        listener.newConnectionHandler = { [weak self] connection in
            let peer = PeerConnection(connection: connection)
            self?.onIncomingPeer?(peer)
        }
        listener.start(queue: .main)
        Log.info("Server listening as \(serviceName)", category: "server")
    }

    func stop() {
        listener?.cancel()
        listener = nil
    }
}
