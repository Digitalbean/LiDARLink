import Foundation

/// Builds clear user-facing explanations about device capability.
public enum SupportMessageBuilder {
    public static func explanation(lidarAvailable: Bool,
                                   worldTrackingAvailable: Bool,
                                   isSimulator: Bool,
                                   depthSupported: Bool,
                                   meshSupported: Bool) -> String {
        if isSimulator {
            return "Running in the Simulator. LiDAR capture requires a physical iPhone with a LiDAR scanner (iPhone 12 Pro or later, including iPhone 16 Pro Max). Connect a real device to test scanning."
        }
        if !worldTrackingAvailable {
            return "This device does not support ARKit world tracking, which LiDAR Link requires for scanning."
        }
        if !lidarAvailable {
            return "This iPhone does not have a LiDAR scanner. LiDAR Link requires iPhone 12 Pro or later (or an iPad Pro with LiDAR) for depth and mesh capture. You can still explore the UI, but scanning is unavailable."
        }
        var parts = ["LiDAR is available."]
        if !depthSupported {
            parts.append("Scene depth is not supported on this device, so point-cloud capture is unavailable.")
        }
        if !meshSupported {
            parts.append("Scene reconstruction (mesh) is not supported; point-cloud scanning will still work.")
        }
        return parts.joined(separator: " ")
    }
}
