import SwiftUI

@main
struct LiDARLinkPhoneApp: App {
    @StateObject private var appState = PhoneAppState()

    var body: some Scene {
        WindowGroup {
            ContentView()
                .environmentObject(appState)
        }
    }
}
