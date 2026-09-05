import ARKit
import Foundation

struct DeviceSupport: Equatable {
    let worldTracking: Bool
    let lidar: Bool
    let depth: Bool
    let mesh: Bool
    let isSimulator: Bool

    static func evaluate() -> DeviceSupport {
        #if targetEnvironment(simulator)
        return DeviceSupport(worldTracking: false, lidar: false, depth: false, mesh: false, isSimulator: true)
        #else
        let worldTracking = ARWorldTrackingConfiguration.isSupported
        // iOS 26 removed the `supportsSceneDepth` class property; the frame-semantic
        // check is the current API and works back to iOS 14.
        let depth = ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth)
        let mesh = ARWorldTrackingConfiguration.supportsSceneReconstruction(.meshWithClassification)
        let lidar = depth || mesh
        return DeviceSupport(worldTracking: worldTracking, lidar: lidar, depth: depth, mesh: mesh, isSimulator: false)
        #endif
    }
}
