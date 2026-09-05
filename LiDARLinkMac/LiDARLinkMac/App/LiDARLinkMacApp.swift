import SwiftUI

@main
struct LiDARLinkMacApp: App {
    @StateObject private var appState = MacAppState()

    var body: some Scene {
        WindowGroup {
            ContentView()
                .environmentObject(appState)
        }
        .defaultSize(width: 1120, height: 700)
    }
}
