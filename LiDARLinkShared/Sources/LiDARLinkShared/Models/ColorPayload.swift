import Foundation

/// A JPEG-compressed color frame aligned with the depth capture.
public struct ColorPayload: Sendable, Equatable {
    public var width: Int
    public var height: Int
    public var jpegData: Data

    public init(width: Int, height: Int, jpegData: Data) {
        self.width = width
        self.height = height
        self.jpegData = jpegData
    }

    // MARK: Wire encoding

    public func binaryPayload() -> Data {
        var data = Data()
        Binary.append(UInt32(width), to: &data)
        Binary.append(UInt32(height), to: &data)
        Binary.append(UInt32(jpegData.count), to: &data)
        data.append(jpegData)
        return data
    }

    public static func decodeBinaryPayload(_ data: Data) -> ColorPayload? {
        guard let widthRaw = Binary.readUInt32(data, at: 0),
              let heightRaw = Binary.readUInt32(data, at: 4),
              let lengthRaw = Binary.readUInt32(data, at: 8) else { return nil }
        guard let jpeg = Binary.readData(data, at: 12, length: Int(lengthRaw)) else { return nil }
        return ColorPayload(width: Int(widthRaw), height: Int(heightRaw), jpegData: jpeg)
    }
}
