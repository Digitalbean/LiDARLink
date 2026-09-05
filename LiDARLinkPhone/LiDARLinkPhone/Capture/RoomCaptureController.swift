import Foundation
import RoomPlan
import ARKit
import simd
import LiDARLinkShared

/// Runs Apple RoomPlan alongside the depth stream. `RoomCaptureSession` owns the
/// ARSession; `ARSessionController` attaches to `roomSession.arSession` to keep
/// pulling depth frames. Incremental `CapturedRoom` updates are converted to the
/// portable `RoomStructure` and streamed to the Mac.
final class RoomCaptureController: NSObject, RoomCaptureSessionDelegate {

    let roomSession = RoomCaptureSession()
    /// Throttled: RoomPlan updates many times a second; ~1 Hz is plenty.
    private var lastEmitMs: UInt64 = 0
    private let minIntervalMs: UInt64 = 900

    var onRoom: ((RoomStructure) -> Void)?

    static var isSupported: Bool { RoomCaptureSession.isSupported }

    override init() {
        super.init()
        roomSession.delegate = self
    }

    func start() {
        roomSession.run(configuration: RoomCaptureSession.Configuration())
        Log.info("RoomCaptureSession started", category: "roomplan")
    }

    func stop() {
        roomSession.stop()
    }

    // MARK: RoomCaptureSessionDelegate

    func captureSession(_ session: RoomCaptureSession, didUpdate room: CapturedRoom) {
        let now = UInt64(Date().timeIntervalSince1970 * 1000)
        guard now &- lastEmitMs >= minIntervalMs else { return }
        lastEmitMs = now
        onRoom?(Self.convert(room, capturedAtMs: now, isFinal: false))
    }

    func captureSession(_ session: RoomCaptureSession, didEndWith data: CapturedRoomData, error: Error?) {
        Task {
            guard let room = try? await RoomBuilder(options: [.beautifyObjects]).capturedRoom(from: data) else { return }
            let now = UInt64(Date().timeIntervalSince1970 * 1000)
            onRoom?(Self.convert(room, capturedAtMs: now, isFinal: true))
        }
    }

    // MARK: Conversion

    private static func convert(_ room: CapturedRoom, capturedAtMs: UInt64, isFinal: Bool) -> RoomStructure {
        var surfaces: [RoomStructure.Surface] = []
        func add(_ s: CapturedRoom.Surface, _ category: RoomStructure.SurfaceCategory) {
            surfaces.append(.init(id: s.identifier.uuidString, category: category,
                                  transform: s.transform, dimensions: s.dimensions,
                                  confidence: confidenceValue(s.confidence),
                                  polygon: worldPolygon(of: s)))
        }
        for w in room.walls { add(w, .wall) }
        for d in room.doors { add(d, .door) }
        for w in room.windows { add(w, .window) }
        for o in room.openings { add(o, .opening) }
        if #available(iOS 17.0, *) {
            for f in room.floors { add(f, .floor) }
        }

        let objects: [RoomStructure.Object] = room.objects.map { obj in
            .init(id: obj.identifier.uuidString, category: objectCategory(obj.category),
                  transform: obj.transform, dimensions: obj.dimensions,
                  confidence: confidenceValue(obj.confidence))
        }
        return RoomStructure(surfaces: surfaces, objects: objects,
                             capturedAtMs: capturedAtMs, isFinal: isFinal)
    }

    /// RoomPlan's `polygonCorners` (local space) transformed to world. Non-empty
    /// only for non-rectangular / curved surfaces on iOS 17+.
    private static func worldPolygon(of s: CapturedRoom.Surface) -> [SIMD3<Float>] {
        guard #available(iOS 17.0, *) else { return [] }
        let corners = s.polygonCorners
        guard corners.count >= 3 else { return [] }
        return corners.map {
            let w = s.transform * SIMD4<Float>($0.x, $0.y, $0.z, 1)
            return SIMD3<Float>(w.x, w.y, w.z)
        }
    }

    private static func confidenceValue(_ c: CapturedRoom.Confidence) -> Float {
        switch c {
        case .high: return 1.0
        case .medium: return 0.6
        case .low: return 0.3
        @unknown default: return 0.5
        }
    }

    private static func objectCategory(_ c: CapturedRoom.Object.Category) -> RoomStructure.ObjectCategory {
        switch c {
        case .storage: return .storage
        case .refrigerator: return .refrigerator
        case .stove: return .stove
        case .bed: return .bed
        case .sink: return .sink
        case .washerDryer: return .washerDryer
        case .toilet: return .toilet
        case .bathtub: return .bathtub
        case .oven: return .oven
        case .dishwasher: return .dishwasher
        case .table: return .table
        case .sofa: return .sofa
        case .chair: return .chair
        case .fireplace: return .fireplace
        case .television: return .television
        case .stairs: return .stairs
        @unknown default: return .unknown
        }
    }
}
