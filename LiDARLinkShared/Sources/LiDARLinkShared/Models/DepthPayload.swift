import Foundation

/// Downsampled scene depth plus per-pixel confidence, ready for the wire.
/// Depth values are Float16 little-endian meters; confidence is UInt8
/// (0 = low … 2 = high, matching ARConfidenceLevel).
public struct DepthPayload: Sendable, Equatable {
    public var width: Int
    public var height: Int
    public var depthData: Data
    public var confidenceData: Data?

    public init(width: Int, height: Int, depthData: Data, confidenceData: Data?) {
        self.width = width
        self.height = height
        self.depthData = depthData
        self.confidenceData = confidenceData
    }

    public var depthCount: Int {
        guard width > 0, height > 0 else { return 0 }
        let (count, overflow) = width.multipliedReportingOverflow(by: height)
        return overflow ? 0 : count
    }

    public func float16Array() -> [Float16] {
        Binary.array(from: depthData, count: depthCount)
    }

    public func confidenceArray() -> [UInt8]? {
        guard let confidenceData, confidenceData.count >= depthCount else { return nil }
        return Binary.array(from: confidenceData, count: depthCount)
    }

    // MARK: Wire encoding
    // Layout: width(4) height(4) hasConfidence(1) compression(1) encodedDepthLength(4)
    //         depthBlob(compressed or raw) confidence?

    public func binaryPayload() -> Data {
        var data = Data()
        Binary.append(UInt32(width), to: &data)
        Binary.append(UInt32(height), to: &data)
        Binary.append(confidenceData != nil ? UInt8(1) : UInt8(0), to: &data)
        let blob: Data
        if let compressed = WireCompression.compress(depthData) {
            Binary.append(UInt8(1), to: &data)
            blob = compressed
        } else {
            Binary.append(UInt8(0), to: &data)
            blob = depthData
        }
        Binary.append(UInt32(blob.count), to: &data)
        data.append(blob)
        if let confidenceData { data.append(confidenceData) }
        return data
    }

    public static func decodeBinaryPayload(_ data: Data) -> DepthPayload? {
        guard let widthRaw = Binary.readUInt32(data, at: 0),
              let heightRaw = Binary.readUInt32(data, at: 4),
              let hasConfidence = Binary.readUInt8(data, at: 8),
              let compressionFlag = Binary.readUInt8(data, at: 9),
              let blobLengthRaw = Binary.readUInt32(data, at: 10) else { return nil }
        let width = Int(widthRaw)
        let height = Int(heightRaw)
        guard width > 0, height > 0,
              hasConfidence <= 1, compressionFlag <= 1 else { return nil }
        let (pixelCount, pixelCountOverflow) = width.multipliedReportingOverflow(by: height)
        let (depthBytes, depthBytesOverflow) = pixelCount.multipliedReportingOverflow(by: MemoryLayout<Float16>.stride)
        guard !pixelCountOverflow, !depthBytesOverflow,
              depthBytes <= ProtocolVersion.maxPayloadSize else { return nil }
        let blobLength = Int(blobLengthRaw)
        guard let blob = Binary.readData(data, at: 14, length: blobLength) else { return nil }
        let depth: Data
        if compressionFlag == 1 {
            guard let decoded = WireCompression.decompress(blob, expectedSize: depthBytes),
                  decoded.count == depthBytes else { return nil }
            depth = decoded
        } else {
            guard blob.count == depthBytes else { return nil }
            depth = blob
        }
        var confidence: Data?
        if hasConfidence == 1 {
            let (offset, offsetOverflow) = 14.addingReportingOverflow(blobLength)
            guard !offsetOverflow,
                  let conf = Binary.readData(data, at: offset, length: pixelCount) else { return nil }
            confidence = conf
        }
        return DepthPayload(width: width, height: height, depthData: depth, confidenceData: confidence)
    }
}
