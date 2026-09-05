import Foundation
import simd

/// Little-endian binary helpers used by the wire and recording codecs.
public enum Binary {
    // MARK: Writers

    public static func append(_ value: UInt8, to data: inout Data) {
        data.append(value)
    }

    public static func append(_ value: UInt16, to data: inout Data) {
        var v = value.littleEndian
        withUnsafeBytes(of: &v) { data.append(contentsOf: $0) }
    }

    public static func append(_ value: UInt32, to data: inout Data) {
        var v = value.littleEndian
        withUnsafeBytes(of: &v) { data.append(contentsOf: $0) }
    }

    public static func append(_ value: UInt64, to data: inout Data) {
        var v = value.littleEndian
        withUnsafeBytes(of: &v) { data.append(contentsOf: $0) }
    }

    public static func append(_ value: Float32, to data: inout Data) {
        var v = value.bitPattern.littleEndian
        withUnsafeBytes(of: &v) { data.append(contentsOf: $0) }
    }

    public static func append(_ value: Float16, to data: inout Data) {
        var v = value.bitPattern.littleEndian
        withUnsafeBytes(of: &v) { data.append(contentsOf: $0) }
    }

    public static func append(_ value: SIMD3<Float>, to data: inout Data) {
        var v = value
        withUnsafeBytes(of: &v) { data.append(contentsOf: $0) }
    }

    /// Converts an array of loadable values to raw little-endian bytes.
    public static func data<T>(from array: [T]) -> Data {
        array.withUnsafeBytes { Data($0) }
    }

    // MARK: Readers (return nil on truncation)

    public static func readUInt8(_ data: Data, at offset: Int) -> UInt8? {
        guard offset >= 0, offset < data.count else { return nil }
        return data[data.startIndex + offset]
    }

    public static func readUInt16(_ data: Data, at offset: Int) -> UInt16? {
        guard offset >= 0, offset + 2 <= data.count else { return nil }
        return data.withUnsafeBytes { $0.loadUnaligned(fromByteOffset: offset, as: UInt16.self) }.littleEndian
    }

    public static func readUInt32(_ data: Data, at offset: Int) -> UInt32? {
        guard offset >= 0, offset + 4 <= data.count else { return nil }
        return data.withUnsafeBytes { $0.loadUnaligned(fromByteOffset: offset, as: UInt32.self) }.littleEndian
    }

    public static func readUInt64(_ data: Data, at offset: Int) -> UInt64? {
        guard offset >= 0, offset + 8 <= data.count else { return nil }
        return data.withUnsafeBytes { $0.loadUnaligned(fromByteOffset: offset, as: UInt64.self) }.littleEndian
    }

    public static func readFloat32(_ data: Data, at offset: Int) -> Float32? {
        guard let bits = readUInt32(data, at: offset) else { return nil }
        return Float32(bitPattern: bits)
    }

    public static func readFloat16(_ data: Data, at offset: Int) -> Float16? {
        guard let bits = readUInt16(data, at: offset) else { return nil }
        return Float16(bitPattern: bits)
    }

    public static func readData(_ data: Data, at offset: Int, length: Int) -> Data? {
        guard offset >= 0, length >= 0 else { return nil }
        let (end, overflow) = offset.addingReportingOverflow(length)
        guard !overflow, end <= data.count else { return nil }
        return data.subdata(in: data.startIndex + offset ..< data.startIndex + end)
    }

    /// Rebuilds an array of loadable values from raw bytes. Invalid or truncated
    /// input produces an empty array instead of terminating the process.
    public static func array<T>(from data: Data, count: Int) -> [T] {
        guard count > 0 else { return [] }
        let (requiredBytes, overflow) = count.multipliedReportingOverflow(by: MemoryLayout<T>.stride)
        guard !overflow, requiredBytes <= data.count else { return [] }
        var result = [T]()
        result.reserveCapacity(count)
        data.withUnsafeBytes { raw in
            for i in 0..<count {
                result.append(raw.loadUnaligned(fromByteOffset: i * MemoryLayout<T>.stride, as: T.self))
            }
        }
        return result
    }
}
