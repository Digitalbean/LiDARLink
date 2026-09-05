import ARKit
import CoreImage
import ImageIO
import UniformTypeIdentifiers
import simd
import LiDARLinkShared

/// "Object (Photo)" capture: as the user orbits a small object, grab a
/// high-resolution still every ~10° of rotation. The bundle feeds a macOS
/// `PhotogrammetrySession` — the LiDAR can't resolve small-object detail, the
/// camera can.
final class ObjectPhotoController {
    var targetPhotos = 36
    var minRotationDegrees: Float = 10

    private(set) var photos: [ObjectPhotoPayload] = []
    private var lastPose: simd_float4x4?
    private var capturing = false
    private let ciContext = CIContext(options: [.cacheIntermediates: false])

    var onProgress: ((_ taken: Int, _ target: Int) -> Void)?

    var isComplete: Bool { photos.count >= targetPhotos }

    /// Call from the AR frame callback. Captures when the camera has rotated far
    /// enough and a slot is free.
    func consider(session: ARSession, frame: ARFrame) {
        guard !capturing, photos.count < targetPhotos else { return }
        let pose = frame.camera.transform
        if let last = lastPose {
            let dr = simd_length(Lie.so3Log(Lie.rotation(last).transpose * Lie.rotation(pose))) * 180 / .pi
            guard dr >= minRotationDegrees else { return }
        }
        capturing = true

        session.captureHighResolutionFrame { [weak self] captured, error in
            guard let self else { return }
            defer { self.capturing = false }
            let source = captured ?? frame
            guard error == nil || captured == nil,
                  let jpeg = self.jpeg(from: source.capturedImage) else { return }

            // Gravity (world -y) expressed in the camera frame.
            let worldToCam = source.camera.transform.inverse
            let g4 = worldToCam * SIMD4<Float>(0, -1, 0, 0)
            let gravity = simd_normalize(SIMD3<Float>(g4.x, g4.y, g4.z))

            let res = source.camera.imageResolution
            let payload = ObjectPhotoPayload(
                index: self.photos.count, total: self.targetPhotos, jpegData: jpeg, gravity: gravity,
                intrinsics: CameraIntrinsics(matrix: source.camera.intrinsics,
                                             imageWidth: Int(res.width), imageHeight: Int(res.height)))
            self.photos.append(payload)
            self.lastPose = pose
            self.onProgress?(self.photos.count, self.targetPhotos)
        }
    }

    func reset() {
        photos.removeAll()
        lastPose = nil
        capturing = false
    }

    private func jpeg(from pixelBuffer: CVPixelBuffer) -> Data? {
        let image = CIImage(cvPixelBuffer: pixelBuffer)
        guard let cg = ciContext.createCGImage(image, from: image.extent) else { return nil }
        let data = NSMutableData()
        guard let dest = CGImageDestinationCreateWithData(data, UTType.jpeg.identifier as CFString, 1, nil) else { return nil }
        CGImageDestinationAddImage(dest, cg, [kCGImageDestinationLossyCompressionQuality: 0.92] as CFDictionary)
        guard CGImageDestinationFinalize(dest) else { return nil }
        return data as Data
    }
}
