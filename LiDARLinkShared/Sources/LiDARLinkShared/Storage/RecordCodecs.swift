import Foundation
import simd

/// Binary codecs for the on-disk recording format.
public enum FrameRecordCodec {
    public static let magic: [UInt8] = [0x4C, 0x4C, 0x52, 0x43] // "LLRC"
    public static let version: UInt8 = 1

    public enum RecordError: Error, Equatable {
        case badMagic
        case unsupportedVersion(UInt8)
        case truncated
        case malformed(String)
        case checksumMismatch
    }

    /// Layout: magic(4) version(1) flags(1) seq(4) timestamp(8) depthScale(4)
    /// intrinsics(6 floats + 2 uint32) pose(16 floats) depth? color?
    public static func encode(_ frame: ScanFrame) throws -> Data {
        var data = Data()
        data.append(contentsOf: magic)
        data.append(version)
        var flags: UInt8 = 0
        if frame.depth != nil { flags |= 1 }
        if frame.color != nil { flags |= 2 }
        if frame.depthIsSmoothed { flags |= 4 }
        data.append(flags)
        Binary.append(frame.sequence, to: &data)
        Binary.append(frame.captureTimestampMs, to: &data)
        Binary.append(frame.depthScale, to: &data)
        Binary.append(frame.intrinsics.fx, to: &data)
        Binary.append(frame.intrinsics.fy, to: &data)
        Binary.append(frame.intrinsics.cx, to: &data)
        Binary.append(frame.intrinsics.cy, to: &data)
        Binary.append(UInt32(frame.intrinsics.imageWidth), to: &data)
        Binary.append(UInt32(frame.intrinsics.imageHeight), to: &data)
        for i in 0..<4 {
            let column = frame.pose[i]
            for value in [column.x, column.y, column.z, column.w] {
                Binary.append(value, to: &data)
            }
        }
        if let depth = frame.depth {
            Binary.append(UInt32(depth.width), to: &data)
            Binary.append(UInt32(depth.height), to: &data)
            Binary.append(depth.confidenceData != nil ? UInt8(1) : UInt8(0), to: &data)
            data.append(depth.depthData)
            if let confidence = depth.confidenceData { data.append(confidence) }
        }
        if let color = frame.color {
            Binary.append(UInt32(color.width), to: &data)
            Binary.append(UInt32(color.height), to: &data)
            Binary.append(UInt32(color.jpegData.count), to: &data)
            data.append(color.jpegData)
        }
        Binary.append(CRC32.checksum(data), to: &data)
        return data
    }

    public static func decode(_ rawData: Data) throws -> ScanFrame {
        var data = rawData
        guard data.count >= 12 else { throw RecordError.truncated }
        let storedCRC = Binary.readUInt32(data, at: data.count - 4) ?? 0
        data.removeLast(4)
        guard CRC32.checksum(data) == storedCRC else { throw RecordError.checksumMismatch }

        var offset = 0
        guard data.count >= 8 else { throw RecordError.truncated }
        guard Array(data.prefix(4)) == magic else { throw RecordError.badMagic }
        let version = data[data.startIndex + 4]
        guard version == self.version else { throw RecordError.unsupportedVersion(version) }
        let flags = data[data.startIndex + 5]
        offset = 6

        guard let sequence = Binary.readUInt32(data, at: offset) else { throw RecordError.truncated }
        offset += 4
        guard let timestamp = Binary.readUInt64(data, at: offset) else { throw RecordError.truncated }
        offset += 8
        guard let depthScale = Binary.readFloat32(data, at: offset) else { throw RecordError.truncated }
        offset += 4
        guard let fx = Binary.readFloat32(data, at: offset) else { throw RecordError.truncated }
        guard let fy = Binary.readFloat32(data, at: offset + 4) else { throw RecordError.truncated }
        guard let cx = Binary.readFloat32(data, at: offset + 8) else { throw RecordError.truncated }
        guard let cy = Binary.readFloat32(data, at: offset + 12) else { throw RecordError.truncated }
        guard let iw = Binary.readUInt32(data, at: offset + 16) else { throw RecordError.truncated }
        guard let ih = Binary.readUInt32(data, at: offset + 20) else { throw RecordError.truncated }
        offset += 24
        let intrinsics = CameraIntrinsics(fx: fx, fy: fy, cx: cx, cy: cy, imageWidth: Int(iw), imageHeight: Int(ih))

        guard offset + 64 <= data.count else { throw RecordError.truncated }
        var columns = [simd_float4]()
        for _ in 0..<4 {
            var column = simd_float4()
            for i in 0..<4 {
                guard let value = Binary.readFloat32(data, at: offset) else { throw RecordError.truncated }
                column[i] = value
                offset += 4
            }
            columns.append(column)
        }
        let pose = simd_float4x4(columns)

        var depth: DepthPayload?
        if flags & 1 != 0 {
            guard let dw = Binary.readUInt32(data, at: offset) else { throw RecordError.truncated }
            guard let dh = Binary.readUInt32(data, at: offset + 4) else { throw RecordError.truncated }
            guard let hasConf = Binary.readUInt8(data, at: offset + 8) else { throw RecordError.truncated }
            offset += 9
            let depthBytes = Int(dw) * Int(dh) * 2
            guard let depthData = Binary.readData(data, at: offset, length: depthBytes) else { throw RecordError.truncated }
            offset += depthBytes
            var confidence: Data?
            if hasConf == 1 {
                guard let conf = Binary.readData(data, at: offset, length: Int(dw) * Int(dh)) else { throw RecordError.truncated }
                confidence = conf
                offset += Int(dw) * Int(dh)
            }
            depth = DepthPayload(width: Int(dw), height: Int(dh), depthData: depthData, confidenceData: confidence)
        }

        var color: ColorPayload?
        if flags & 2 != 0 {
            guard let cw = Binary.readUInt32(data, at: offset) else { throw RecordError.truncated }
            guard let ch = Binary.readUInt32(data, at: offset + 4) else { throw RecordError.truncated }
            guard let len = Binary.readUInt32(data, at: offset + 8) else { throw RecordError.truncated }
            offset += 12
            guard let jpeg = Binary.readData(data, at: offset, length: Int(len)) else { throw RecordError.truncated }
            color = ColorPayload(width: Int(cw), height: Int(ch), jpegData: jpeg)
            offset += Int(len)
        }

        return ScanFrame(sequence: sequence,
                         captureTimestampMs: timestamp,
                         pose: pose,
                         intrinsics: intrinsics,
                         depth: depth,
                         color: color,
                         depthIsSmoothed: flags & 4 != 0,
                         depthScale: depthScale)
    }
}

/// On-disk codec for mesh data (meshes.bin records).
public enum MeshRecordCodec {
    public static let magic: [UInt8] = [0x4C, 0x4C, 0x4D, 0x45] // "LLME"
    public static let version: UInt8 = 1

    public enum RecordError: Error, Equatable {
        case badMagic
        case unsupportedVersion(UInt8)
        case truncated
        case checksumMismatch
    }

    public static func encode(_ mesh: MeshData) throws -> Data {
        var data = Data()
        data.append(contentsOf: magic)
        data.append(version)
        let uuid = mesh.anchorID.uuid
        withUnsafeBytes(of: uuid) { data.append(contentsOf: $0) }
        Binary.append(mesh.updatedAtMs, to: &data)
        Binary.append(UInt32(mesh.vertices.count), to: &data)
        Binary.append(UInt32(mesh.normals.count), to: &data)
        Binary.append(UInt32(mesh.faces.count), to: &data)
        for v in mesh.vertices { Binary.append(v, to: &data) }
        for n in mesh.normals { Binary.append(n, to: &data) }
        for f in mesh.faces { Binary.append(f, to: &data) }
        for i in 0..<4 {
            let column = mesh.transform[i]
            for value in [column.x, column.y, column.z, column.w] {
                Binary.append(value, to: &data)
            }
        }
        Binary.append(CRC32.checksum(data), to: &data)
        return data
    }

    public static func decode(_ rawData: Data) throws -> MeshData {
        var data = rawData
        guard data.count >= 34 else { throw RecordError.truncated }
        let storedCRC = Binary.readUInt32(data, at: data.count - 4) ?? 0
        data.removeLast(4)
        guard CRC32.checksum(data) == storedCRC else { throw RecordError.checksumMismatch }

        guard data.count >= 30 else { throw RecordError.truncated }
        guard Array(data.prefix(4)) == magic else { throw RecordError.badMagic }
        let version = data[data.startIndex + 4]
        guard version == self.version else { throw RecordError.unsupportedVersion(version) }

        var offset = 5
        guard offset + 16 <= data.count else { throw RecordError.truncated }
        var uuidBytes = UUID().uuid
        withUnsafeMutableBytes(of: &uuidBytes) { raw in
            let slice = data.subdata(in: data.startIndex + offset ..< data.startIndex + offset + 16)
            slice.copyBytes(to: raw.bindMemory(to: UInt8.self))
        }
        let anchorID = UUID(uuid: uuidBytes)
        offset += 16

        guard let updatedAt = Binary.readUInt64(data, at: offset) else { throw RecordError.truncated }
        offset += 8
        guard let vc = Binary.readUInt32(data, at: offset) else { throw RecordError.truncated }
        guard let nc = Binary.readUInt32(data, at: offset + 4) else { throw RecordError.truncated }
        guard let fc = Binary.readUInt32(data, at: offset + 8) else { throw RecordError.truncated }
        offset += 12
        let vertexCount = Int(vc)
        let normalCount = Int(nc)
        let faceCount = Int(fc)
        let verticesBytes = vertexCount * MemoryLayout<SIMD3<Float>>.stride
        let normalsBytes = normalCount * MemoryLayout<SIMD3<Float>>.stride
        let facesBytes = faceCount * MemoryLayout<UInt32>.stride
        let transformBytes = 64
        guard offset + verticesBytes + normalsBytes + facesBytes + transformBytes <= data.count else { throw RecordError.truncated }
        let vertices: [SIMD3<Float>] = Binary.array(from: data.subdata(in: offset..<offset + verticesBytes), count: vertexCount)
        offset += verticesBytes
        let normals: [SIMD3<Float>] = Binary.array(from: data.subdata(in: offset..<offset + normalsBytes), count: normalCount)
        offset += normalsBytes
        let faces: [UInt32] = Binary.array(from: data.subdata(in: offset..<offset + facesBytes), count: faceCount)
        offset += facesBytes
        var columns = [simd_float4]()
        for _ in 0..<4 {
            var column = simd_float4()
            for i in 0..<4 {
                guard let value = Binary.readFloat32(data, at: offset) else { throw RecordError.truncated }
                column[i] = value
                offset += 4
            }
            columns.append(column)
        }
        return MeshData(anchorID: anchorID,
                        vertices: vertices,
                        normals: normals,
                        faces: faces,
                        transform: simd_float4x4(columns),
                        updatedAtMs: updatedAt)
    }
}
