import Foundation
import RealityKit
import LiDARLinkShared

/// Collects the phone's "Object (Photo)" bundle and runs a macOS
/// `PhotogrammetrySession` to produce a textured USDZ.
@MainActor
final class ObjectReconstructor {

    enum Stage: Equatable {
        case idle
        case receiving(Int, Int)
        case processing(Double)
        case done(URL)
        case failed(String)
    }

    private(set) var stage: Stage = .idle { didSet { onStage?(stage) } }
    var onStage: ((Stage) -> Void)?

    private var dir: URL?
    private var expected = 0
    private var received = 0

    static var isSupported: Bool { PhotogrammetrySession.isSupported }

    func reset() {
        if let dir { try? FileManager.default.removeItem(at: dir) }
        dir = nil; expected = 0; received = 0
        stage = .idle
    }

    func receive(_ photo: ObjectPhotoPayload) {
        if dir == nil {
            let d = FileManager.default.temporaryDirectory
                .appendingPathComponent("lidarlink-object-\(UUID().uuidString)")
            try? FileManager.default.createDirectory(at: d, withIntermediateDirectories: true)
            dir = d
            expected = max(photo.total, 1)
            received = 0
        }
        guard let dir else { return }
        let url = dir.appendingPathComponent(String(format: "photo_%03d.jpg", photo.index))
        try? photo.jpegData.write(to: url)
        received += 1
        stage = .receiving(received, expected)
        if received >= expected { process() }
    }

    private func process() {
        guard let dir, PhotogrammetrySession.isSupported else {
            stage = .failed("Photogrammetry isn't supported on this Mac")
            return
        }
        let output = dir.appendingPathComponent("model.usdz")
        do {
            let session = try PhotogrammetrySession(input: dir)
            stage = .processing(0)
            Task {
                do {
                    for try await update in session.outputs {
                        switch update {
                        case .requestProgress(_, let fraction):
                            stage = .processing(fraction)
                        case .requestComplete(_, let result):
                            if case .modelFile(let modelURL) = result { stage = .done(modelURL) }
                        case .requestError(_, let error):
                            stage = .failed(error.localizedDescription)
                        default:
                            break
                        }
                    }
                } catch {
                    stage = .failed(error.localizedDescription)
                }
            }
            try session.process(requests: [.modelFile(url: output, detail: .medium)])
        } catch {
            stage = .failed(error.localizedDescription)
        }
    }
}
