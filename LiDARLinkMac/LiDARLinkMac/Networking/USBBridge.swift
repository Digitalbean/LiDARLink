import Foundation
import LiDARLinkShared

/// Opens a usbmuxd tunnel to the LiDARLink app on a cable-attached iPhone and
/// hands the resulting byte stream to a `PeerConnection`, exactly like an
/// incoming Wi-Fi peer.
final class USBBridge {
    /// Must match `USBListener.port` on the phone.
    static let phonePort: UInt16 = 51703

    private var pollTimer: DispatchSourceTimer?
    private var activeFD: Int32?

    var onPeer: ((PeerConnection, _ deviceSerial: String) -> Void)?
    var onStatus: ((String) -> Void)?

    /// Polls usbmuxd until a device is present, tunnels to it, and produces a peer.
    /// Retries every 2 s while nothing connects.
    func start() {
        stop()
        let timer = DispatchSource.makeTimerSource(queue: DispatchQueue(label: "com.lidarlink.usbbridge"))
        timer.schedule(deadline: .now(), repeating: 2.0)
        timer.setEventHandler { [weak self] in self?.attempt() }
        pollTimer = timer
        timer.resume()
    }

    func stop() {
        pollTimer?.cancel()
        pollTimer = nil
    }

    private func attempt() {
        guard activeFD == nil else { return }
        do {
            let devices = try USBMuxClient.listDevices()
            guard let device = devices.first else {
                onStatus?("No cable-attached iPhone")
                return
            }
            let fd = try USBMuxClient.connect(device: device, port: Self.phonePort)
            activeFD = fd
            onStatus?("Tunnel open to \(device.serialNumber.prefix(8))…")

            let channel = RawSocketChannel(fd: fd)
            channel.onFailed = { [weak self] _ in self?.activeFD = nil }
            let peer = PeerConnection(channel: channel)
            DispatchQueue.main.async { [weak self] in
                self?.onPeer?(peer, device.serialNumber)
            }
        } catch {
            onStatus?((error as? LocalizedError)?.errorDescription ?? error.localizedDescription)
        }
    }
}
