import Foundation

/// File-based diagnostics for the macOS companion app. Writes to the first
/// writable location (workspace debug file, then ~/Library/Logs).
public enum FileLog {
    private static let lock = NSLock()
    private static var chosenURL: URL?

    public static func append(_ message: String, category: String) {
        #if os(macOS)
        lock.lock()
        defer { lock.unlock() }
        guard let url = writableURL() else { return }
        let line = "[\(Date())] \(category): \(message)\n"
        if let handle = try? FileHandle(forWritingTo: url) {
            defer { try? handle.close() }
            try? handle.seekToEnd()
            if let data = line.data(using: .utf8) {
                try? handle.write(contentsOf: data)
            }
        } else {
            try? line.data(using: .utf8)?.write(to: url)
        }
        #endif
    }

    #if os(macOS)
    private static func writableURL() -> URL? {
        if let chosenURL { return chosenURL }
        let home = FileManager.default.homeDirectoryForCurrentUser
        let candidates = [
            home.appendingPathComponent("Documents/LiDARLink/lidar-debug.log"),
            home.appendingPathComponent("Library/Logs/LiDARLink.log")
        ]
        for url in candidates {
            if FileManager.default.fileExists(atPath: url.path) || (try? Data().write(to: url)) != nil {
                chosenURL = url
                return url
            }
        }
        return nil
    }
    #endif
}
