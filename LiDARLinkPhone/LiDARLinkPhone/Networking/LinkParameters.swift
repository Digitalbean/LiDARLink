import Foundation
import Network

/// TCP parameters for the Mac link. When `preferWired` is set the browser and
/// connection are pinned to a wired interface — the iPhone-to-Mac USB link shows
/// up as `.wiredEthernet` — so the stream leaves congested Wi-Fi behind.
enum LinkParameters {
    static func tcp(preferWired: Bool) -> NWParameters {
        let params = NWParameters.tcp
        if preferWired {
            params.requiredInterfaceType = .wiredEthernet
            params.prohibitedInterfaceTypes = [.cellular]
        } else {
            params.prohibitedInterfaceTypes = [.cellular]
        }
        // Low-latency: disable Nagle so small control messages ship immediately.
        if let tcp = params.defaultProtocolStack.transportProtocol as? NWProtocolTCP.Options {
            tcp.noDelay = true
        }
        return params
    }
}
