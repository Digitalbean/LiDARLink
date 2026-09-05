import Foundation

/// Versioning for the LiDAR Link wire protocol.
public enum ProtocolVersion: Sendable {
    /// Current protocol version implemented by this build.
    /// v2: depth payloads may be LZFSE-compressed (flag byte in frameDepth payload).
    public static let current: UInt8 = 2

    /// Four-byte magic prefix `"LLF1"` (LiDAR Link Frame v1). Used to resynchronise
    /// the byte stream after corruption or foreign data.
    public static let magicBytes: [UInt8] = [0x4C, 0x4C, 0x46, 0x31]

    /// Maximum accepted payload size (32 MB). Larger payloads are rejected as corrupt.
    public static let maxPayloadSize: Int = 32 * 1024 * 1024

    /// Header size in bytes: magic(4) + version(1) + type(1) + flags(1) + reserved(1)
    /// + sequence(4) + timestamp(8) + payloadLength(4).
    public static let headerSize: Int = 24

    /// Trailer size in bytes: CRC-32 of the payload.
    public static let trailerSize: Int = 4
}
