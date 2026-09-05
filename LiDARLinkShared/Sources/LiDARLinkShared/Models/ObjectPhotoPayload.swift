import Foundation
import simd

/// One high-resolution still from the phone's "Object (Photo)" capture — the
/// input to a macOS `PhotogrammetrySession`. Sent one per wire message.
public struct ObjectPhotoPayload: Sendable, Equatable {
    public var index: Int
    public var total: Int
    /// Full-resolution JPEG (`captureHighResolutionFrame`).
    public var jpegData: Data
    /// Gravity direction in the image frame — PhotogrammetrySession orientation hint.
    public var gravity: SIMD3<Float>
    public var intrinsics: CameraIntrinsics

    public init(index: Int, total: Int, jpegData: Data, gravity: SIMD3<Float>, intrinsics: CameraIntrinsics) {
        self.index = index
        self.total = total
        self.jpegData = jpegData
        self.gravity = gravity
        self.intrinsics = intrinsics
    }

    // Layout: index(4) total(4) gx(4) gy(4) gz(4) fx(4) fy(4) cx(4) cy(4) w(4) h(4) jpegLen(4) jpeg
    public func binaryPayload() -> Data {
        var data = Data()
        Binary.append(UInt32(index), to: &data)
        Binary.append(UInt32(total), to: &data)
        for v in [gravity.x, gravity.y, gravity.z,
                  intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy] {
            Binary.append(Float32(v), to: &data)
        }
        Binary.append(UInt32(intrinsics.imageWidth), to: &data)
        Binary.append(UInt32(intrinsics.imageHeight), to: &data)
        Binary.append(UInt32(jpegData.count), to: &data)
        data.append(jpegData)
        return data
    }

    public static func decodeBinaryPayload(_ data: Data) -> ObjectPhotoPayload? {
        guard data.count >= 48,
              let index = Binary.readUInt32(data, at: 0),
              let total = Binary.readUInt32(data, at: 4) else { return nil }
        func f(_ off: Int) -> Float? { Binary.readUInt32(data, at: off).map { Float(bitPattern: $0) } }
        guard let gx = f(8), let gy = f(12), let gz = f(16),
              let fx = f(20), let fy = f(24), let cx = f(28), let cy = f(32),
              let w = Binary.readUInt32(data, at: 36), let h = Binary.readUInt32(data, at: 40),
              let len = Binary.readUInt32(data, at: 44),
              let jpeg = Binary.readData(data, at: 48, length: Int(len)) else { return nil }
        return ObjectPhotoPayload(
            index: Int(index), total: Int(total), jpegData: jpeg,
            gravity: SIMD3<Float>(gx, gy, gz),
            intrinsics: CameraIntrinsics(fx: fx, fy: fy, cx: cx, cy: cy,
                                         imageWidth: Int(w), imageHeight: Int(h)))
    }
}
