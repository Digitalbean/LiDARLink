import Foundation

/// Minimal client for macOS `usbmuxd` (the daemon Xcode uses to reach a
/// cable-attached iPhone). Lists attached devices and opens a raw byte tunnel to
/// a TCP port the phone is listening on — the low-latency transport.
enum USBMuxClient {

    struct Device {
        let deviceID: Int
        let serialNumber: String
        let productID: Int
    }

    enum USBMuxError: Error, LocalizedError {
        case socket(String)
        case badResponse
        case connectRefused(Int)
        case noDevice

        var errorDescription: String? {
            switch self {
            case .socket(let s): return "usbmuxd socket error: \(s)"
            case .badResponse: return "Unexpected reply from usbmuxd"
            case .connectRefused(let n): return "usbmuxd refused the tunnel (code \(n)) — is the app running on the phone?"
            case .noDevice: return "No cable-attached iPhone found"
            }
        }
    }

    private static let socketPath = "/var/run/usbmuxd"
    private static let plistMessageType: UInt32 = 8
    private static let protocolVersion: UInt32 = 1

    // MARK: Public

    static func listDevices() throws -> [Device] {
        let fd = try openSocket()
        defer { close(fd) }
        try sendPlist(fd, ["MessageType": "ListDevices",
                           "ClientVersionString": "LiDARLink",
                           "ProgName": "LiDARLink",
                           "kLibUSBMuxVersion": 3], tag: 1)
        let reply = try readPlist(fd)
        guard let list = reply["DeviceList"] as? [[String: Any]] else { return [] }
        return list.compactMap { entry in
            guard let props = entry["Properties"] as? [String: Any],
                  (props["ConnectionType"] as? String) == "USB",
                  let id = props["DeviceID"] as? Int else { return nil }
            return Device(deviceID: id,
                          serialNumber: props["SerialNumber"] as? String ?? "",
                          productID: props["ProductID"] as? Int ?? 0)
        }
    }

    /// Opens a tunnel to `port` on `device` and returns the connected socket fd.
    /// The caller owns the fd and must `close` it.
    static func connect(device: Device, port: UInt16) throws -> Int32 {
        let fd = try openSocket()
        // usbmuxd wants the port big-endian.
        let bePort = Int(port.bigEndian)
        try sendPlist(fd, ["MessageType": "Connect",
                           "DeviceID": device.deviceID,
                           "PortNumber": bePort,
                           "ClientVersionString": "LiDARLink",
                           "ProgName": "LiDARLink",
                           "kLibUSBMuxVersion": 3], tag: 2)
        let reply = try readPlist(fd)
        let number = reply["Number"] as? Int ?? -1
        guard number == 0 else {
            close(fd)
            throw USBMuxError.connectRefused(number)
        }
        return fd   // now a raw pipe to the phone's port
    }

    static func firstDevice() throws -> Device {
        guard let d = try listDevices().first else { throw USBMuxError.noDevice }
        return d
    }

    // MARK: Framing

    private static func openSocket() throws -> Int32 {
        let fd = socket(AF_UNIX, SOCK_STREAM, 0)
        guard fd >= 0 else { throw USBMuxError.socket("socket() failed") }
        var addr = sockaddr_un()
        addr.sun_family = sa_family_t(AF_UNIX)
        _ = socketPath.withCString { src in
            withUnsafeMutablePointer(to: &addr.sun_path) {
                $0.withMemoryRebound(to: CChar.self, capacity: 104) { dst in
                    strncpy(dst, src, 103)
                }
            }
        }
        let size = socklen_t(MemoryLayout<sockaddr_un>.size)
        let ok = withUnsafePointer(to: &addr) {
            $0.withMemoryRebound(to: sockaddr.self, capacity: 1) {
                Darwin.connect(fd, $0, size)
            }
        }
        guard ok == 0 else {
            close(fd)
            throw USBMuxError.socket("connect(\(socketPath)) failed (\(errno))")
        }
        return fd
    }

    private static func sendPlist(_ fd: Int32, _ dict: [String: Any], tag: UInt32) throws {
        let body = try PropertyListSerialization.data(fromPropertyList: dict, format: .xml, options: 0)
        var header = Data()
        var total = UInt32(16 + body.count).littleEndian
        var version = protocolVersion.littleEndian
        var type = plistMessageType.littleEndian
        var t = tag.littleEndian
        withUnsafeBytes(of: &total) { header.append(contentsOf: $0) }
        withUnsafeBytes(of: &version) { header.append(contentsOf: $0) }
        withUnsafeBytes(of: &type) { header.append(contentsOf: $0) }
        withUnsafeBytes(of: &t) { header.append(contentsOf: $0) }
        try writeAll(fd, header + body)
    }

    private static func readPlist(_ fd: Int32) throws -> [String: Any] {
        let header = try readExactly(fd, 16)
        let total = header.withUnsafeBytes { $0.load(as: UInt32.self).littleEndian }
        guard total >= 16, total < 8_000_000 else { throw USBMuxError.badResponse }
        let body = try readExactly(fd, Int(total) - 16)
        guard let dict = try PropertyListSerialization.propertyList(from: body, options: [], format: nil) as? [String: Any] else {
            throw USBMuxError.badResponse
        }
        return dict
    }

    private static func writeAll(_ fd: Int32, _ data: Data) throws {
        try data.withUnsafeBytes { raw in
            var off = 0
            let base = raw.bindMemory(to: UInt8.self).baseAddress!
            while off < data.count {
                let n = write(fd, base + off, data.count - off)
                if n <= 0 { throw USBMuxError.socket("write failed (\(errno))") }
                off += n
            }
        }
    }

    private static func readExactly(_ fd: Int32, _ count: Int) throws -> Data {
        var buf = Data(count: count)
        var got = 0
        try buf.withUnsafeMutableBytes { raw in
            let base = raw.bindMemory(to: UInt8.self).baseAddress!
            while got < count {
                let n = read(fd, base + got, count - got)
                if n <= 0 { throw USBMuxError.socket("read failed (\(errno))") }
                got += n
            }
        }
        return buf
    }
}
