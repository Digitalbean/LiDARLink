import Foundation
import Network
import LiDARLinkShared

/// Browses for the LiDAR Link Mac app over Bonjour (`_lidarlink._tcp`).
final class BonjourBrowser {
    private var browser: NWBrowser?
    var onResults: (([DiscoveredMac]) -> Void)?

    func start(preferWired: Bool = false) {
        let browser = NWBrowser(for: .bonjour(type: "_lidarlink._tcp", domain: "local"),
                                using: LinkParameters.tcp(preferWired: preferWired))
        self.browser = browser
        browser.stateUpdateHandler = { state in
            Log.info("Browser state: \(state)", category: "browser")
        }
        browser.browseResultsChangedHandler = { [weak self] results, _ in
            let macs = results.compactMap { result -> DiscoveredMac? in
                guard let name = Self.displayName(for: result.endpoint) else { return nil }
                return DiscoveredMac(name: name, endpoint: result.endpoint)
            }
            .sorted { $0.name < $1.name }
            self?.onResults?(macs)
        }
        browser.start(queue: .main)
    }

    func stop() {
        browser?.cancel()
        browser = nil
    }

    static func displayName(for endpoint: NWEndpoint) -> String? {
        switch endpoint {
        case .service(let name, _, _, _): return name
        case .hostPort(let host, let port): return "\(host):\(port)"
        default: return nil
        }
    }
}
