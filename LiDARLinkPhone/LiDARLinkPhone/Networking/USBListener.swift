import Foundation
import Network
import LiDARLinkShared

/// Listens on a loopback TCP port for the connection the Mac opens through the
/// usbmuxd cable tunnel. The Mac side (`USBMuxClient`) dials `localhost:<port>`
/// on the device; usbmuxd delivers it here as a normal inbound connection.
final class USBListener {
    static let port: UInt16 = 51703

    private var listener: NWListener?
    var onConnection: ((NWConnection) -> Void)?
    var onError: ((String) -> Void)?

    func start() {
        do {
            let params = NWParameters.tcp
            params.allowLocalEndpointReuse = true
            let listener = try NWListener(using: params, on: NWEndpoint.Port(rawValue: Self.port)!)
            self.listener = listener
            listener.newConnectionHandler = { [weak self] connection in
                self?.onConnection?(connection)
            }
            listener.stateUpdateHandler = { [weak self] state in
                if case .failed(let error) = state {
                    self?.onError?(error.localizedDescription)
                }
            }
            listener.start(queue: .main)
            Log.info("USB listener started on \(Self.port)", category: "usb")
        } catch {
            onError?(error.localizedDescription)
        }
    }

    func stop() {
        listener?.cancel()
        listener = nil
    }
}
