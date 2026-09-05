import Foundation
import Compression

/// LZFSE compression for large binary payloads (depth maps).
public enum WireCompression {
    /// Compresses `data`; returns nil when compression is not beneficial or fails.
    public static func compress(_ data: Data) -> Data? {
        guard !data.isEmpty else { return nil }
        let capacity = data.count + data.count / 4 + 64
        var output = [UInt8](repeating: 0, count: capacity)
        let written = data.withUnsafeBytes { (src: UnsafeRawBufferPointer) -> Int in
            guard let base = src.baseAddress else { return 0 }
            return output.withUnsafeMutableBytes { (dst: UnsafeMutableRawBufferPointer) -> Int in
                guard let dstBase = dst.baseAddress else { return 0 }
                return compression_encode_buffer(dstBase.bindMemory(to: UInt8.self, capacity: capacity),
                                                 capacity,
                                                 base.bindMemory(to: UInt8.self, capacity: data.count),
                                                 data.count,
                                                 nil,
                                                 COMPRESSION_LZFSE)
            }
        }
        guard written > 0, written < data.count else { return nil }
        return Data(output.prefix(written))
    }

    /// Decompresses `data` into exactly `expectedSize` bytes; nil on failure.
    public static func decompress(_ data: Data, expectedSize: Int) -> Data? {
        guard expectedSize > 0, !data.isEmpty else { return nil }
        var output = [UInt8](repeating: 0, count: expectedSize)
        let written = data.withUnsafeBytes { (src: UnsafeRawBufferPointer) -> Int in
            guard let base = src.baseAddress else { return 0 }
            return output.withUnsafeMutableBytes { (dst: UnsafeMutableRawBufferPointer) -> Int in
                guard let dstBase = dst.baseAddress else { return 0 }
                return compression_decode_buffer(dstBase.bindMemory(to: UInt8.self, capacity: expectedSize),
                                                 expectedSize,
                                                 base.bindMemory(to: UInt8.self, capacity: data.count),
                                                 data.count,
                                                 nil,
                                                 COMPRESSION_LZFSE)
            }
        }
        guard written == expectedSize else { return nil }
        return Data(output)
    }
}
