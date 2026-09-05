import Foundation

/// Persists and reloads recordings. A recording is a directory containing:
///   manifest.json — metadata
///   frames.bin    — length-prefixed ScanFrame records
///   meshes.bin    — length-prefixed MeshData records (optional)
public enum ScanRecordingStore {
    public enum StorageError: Error, LocalizedError, Equatable {
        case notFound(String)
        case corrupt(String)

        public var errorDescription: String? {
            switch self {
            case .notFound(let path): return "Recording not found at \(path)"
            case .corrupt(let reason): return "Recording is corrupt: \(reason)"
            }
        }
    }

    public static let manifestFileName = "manifest.json"
    public static let framesFileName = "frames.bin"
    public static let meshesFileName = "meshes.bin"

    /// Where new recordings are written: `~/Documents/LiDAR Link/Recordings`, so
    /// they are visible in Finder instead of buried in Application Support.
    public static func defaultRecordingsDirectory() throws -> URL {
        let documents = try FileManager.default.url(for: .documentDirectory,
                                                    in: .userDomainMask,
                                                    appropriateFor: nil,
                                                    create: true)
        let dir = documents.appendingPathComponent("LiDAR Link", isDirectory: true)
            .appendingPathComponent("Recordings", isDirectory: true)
        try FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
        return dir
    }

    /// The legacy Application Support location — still scanned when listing so
    /// older recordings don't disappear.
    public static func legacyRecordingsDirectory() -> URL? {
        guard let base = try? FileManager.default.url(for: .applicationSupportDirectory,
                                                      in: .userDomainMask,
                                                      appropriateFor: nil,
                                                      create: false) else { return nil }
        return base.appendingPathComponent("LiDARLink", isDirectory: true)
            .appendingPathComponent("Recordings", isDirectory: true)
    }

    /// All recording directories from both the current and legacy locations,
    /// newest first.
    public static func allRecordingDirectories() -> [URL] {
        var roots: [URL] = []
        if let current = try? defaultRecordingsDirectory() { roots.append(current) }
        if let legacy = legacyRecordingsDirectory() { roots.append(legacy) }
        let fm = FileManager.default
        var result: [URL] = []
        for root in roots {
            let entries = (try? fm.contentsOfDirectory(at: root,
                                                       includingPropertiesForKeys: nil,
                                                       options: [.skipsHiddenFiles])) ?? []
            result += entries.filter {
                $0.hasDirectoryPath && fm.fileExists(atPath: $0.appendingPathComponent(manifestFileName).path)
            }
        }
        return result.sorted { $0.lastPathComponent > $1.lastPathComponent }
    }

    public static func newRecordingDirectory(in base: URL, date: Date = Date()) throws -> URL {
        let formatter = DateFormatter()
        formatter.dateFormat = "yyyyMMdd_HHmmss"
        formatter.locale = Locale(identifier: "en_US_POSIX")
        let baseName = "LiDARLink_\(formatter.string(from: date))"
        var dir = base.appendingPathComponent(baseName, isDirectory: true)
        var counter = 2
        while FileManager.default.fileExists(atPath: dir.path) {
            dir = base.appendingPathComponent("\(baseName)_\(counter)", isDirectory: true)
            counter += 1
        }
        try FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
        return dir
    }

    // MARK: Manifest

    public static func saveManifest(_ manifest: RecordingManifest, to directory: URL) throws {
        let data = try JSONEncoder.lidarLink.encode(manifest)
        try data.write(to: directory.appendingPathComponent(manifestFileName), options: .atomic)
    }

    public static func loadManifest(from directory: URL) throws -> RecordingManifest {
        let url = directory.appendingPathComponent(manifestFileName)
        guard let data = try? Data(contentsOf: url) else {
            throw StorageError.notFound(url.path)
        }
        return try JSONDecoder.lidarLink.decode(RecordingManifest.self, from: data)
    }

    // MARK: Frames

    public static func createFramesFile(in directory: URL) throws {
        FileManager.default.createFile(atPath: directory.appendingPathComponent(framesFileName).path, contents: nil)
    }

    public static func appendFrames(_ frames: [ScanFrame], to directory: URL) throws {
        let url = directory.appendingPathComponent(framesFileName)
        let handle = try FileHandle(forWritingTo: url)
        defer { try? handle.close() }
        try handle.seekToEnd()
        for frame in frames {
            let record = try FrameRecordCodec.encode(frame)
            var length = UInt32(record.count).littleEndian
            try handle.write(contentsOf: withUnsafeBytes(of: &length) { Data($0) })
            try handle.write(contentsOf: record)
        }
    }

    public static func loadFrames(from directory: URL) throws -> [ScanFrame] {
        let url = directory.appendingPathComponent(framesFileName)
        guard let data = try? Data(contentsOf: url) else {
            throw StorageError.notFound(url.path)
        }
        var frames: [ScanFrame] = []
        var offset = 0
        while offset + 4 <= data.count {
            guard let lengthRaw = Binary.readUInt32(data, at: offset) else { break }
            offset += 4
            let length = Int(lengthRaw)
            guard offset + length <= data.count else {
                throw StorageError.corrupt("Truncated frame record at byte \(offset)")
            }
            let record = data.subdata(in: offset..<offset + length)
            do {
                frames.append(try FrameRecordCodec.decode(record))
            } catch {
                throw StorageError.corrupt("Undecodable frame record: \(error)")
            }
            offset += length
        }
        return frames
    }

    // MARK: Meshes

    public static func createMeshesFile(in directory: URL) throws {
        FileManager.default.createFile(atPath: directory.appendingPathComponent(meshesFileName).path, contents: nil)
    }

    public static func appendMeshes(_ meshes: [MeshData], to directory: URL) throws {
        let url = directory.appendingPathComponent(meshesFileName)
        if !FileManager.default.fileExists(atPath: url.path) {
            FileManager.default.createFile(atPath: url.path, contents: nil)
        }
        let handle = try FileHandle(forWritingTo: url)
        defer { try? handle.close() }
        try handle.seekToEnd()
        for mesh in meshes {
            let record = try MeshRecordCodec.encode(mesh)
            var length = UInt32(record.count).littleEndian
            try handle.write(contentsOf: withUnsafeBytes(of: &length) { Data($0) })
            try handle.write(contentsOf: record)
        }
    }

    public static func loadMeshes(from directory: URL) throws -> [MeshData] {
        let url = directory.appendingPathComponent(meshesFileName)
        guard FileManager.default.fileExists(atPath: url.path),
              let data = try? Data(contentsOf: url) else { return [] }
        var meshes: [MeshData] = []
        var offset = 0
        while offset + 4 <= data.count {
            guard let lengthRaw = Binary.readUInt32(data, at: offset) else { break }
            offset += 4
            let length = Int(lengthRaw)
            guard offset + length <= data.count else {
                throw StorageError.corrupt("Truncated mesh record at byte \(offset)")
            }
            let record = data.subdata(in: offset..<offset + length)
            do {
                meshes.append(try MeshRecordCodec.decode(record))
            } catch {
                throw StorageError.corrupt("Undecodable mesh record: \(error)")
            }
            offset += length
        }
        return meshes
    }

    // MARK: Whole recording

    public static func loadRecording(from directory: URL) throws -> (RecordingManifest, [ScanFrame], [MeshData]) {
        (try loadManifest(from: directory), try loadFrames(from: directory), try loadMeshes(from: directory))
    }
}
