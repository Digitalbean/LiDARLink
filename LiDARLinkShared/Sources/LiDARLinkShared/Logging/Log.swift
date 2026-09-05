import Foundation
import os

/// Structured logging helpers. All output goes to the unified logging system
/// (visible in Console.app / `log stream`) under the `com.lidarlink` subsystem.
public enum Log {
    public static let subsystem = "com.lidarlink"

    public static func make(_ category: String) -> Logger {
        Logger(subsystem: subsystem, category: category)
    }

    public static func debug(_ message: @autoclosure @escaping () -> String, category: String = "general", file: String = #fileID, line: Int = #line) {
        make(category).debug("\(message()) [\(file):\(line)]")
    }

    public static func info(_ message: @autoclosure @escaping () -> String, category: String = "general", file: String = #fileID, line: Int = #line) {
        make(category).info("\(message()) [\(file):\(line)]")
    }

    public static func warning(_ message: @autoclosure @escaping () -> String, category: String = "general", file: String = #fileID, line: Int = #line) {
        make(category).notice("\(message()) [\(file):\(line)]")
    }

    public static func error(_ message: @autoclosure @escaping () -> String, category: String = "general", file: String = #fileID, line: Int = #line) {
        make(category).error("\(message()) [\(file):\(line)]")
    }
}
