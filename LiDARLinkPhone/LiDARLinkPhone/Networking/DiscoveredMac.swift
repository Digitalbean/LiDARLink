import Foundation
import Network

struct DiscoveredMac: Identifiable, Equatable {
    let id = UUID()
    let name: String
    let endpoint: NWEndpoint
}
