import Foundation
import Network

/// The transport surface `PeerConnection` needs, so it can run identically over
/// Wi-Fi (`NWConnection`) or a usbmuxd cable tunnel (a raw socket fd).
protocol ByteChannel: AnyObject {
    var onReady: (() -> Void)? { get set }
    var onFailed: ((String) -> Void)? { get set }
    func start(queue: DispatchQueue)
    func send(_ data: Data)
    /// One-shot: fires once with the next chunk (or EOF/error); the caller re-arms.
    func receive(_ completion: @escaping (Data?, _ isComplete: Bool, _ error: String?) -> Void)
    func cancel()
}

// MARK: - Wi-Fi

final class NWByteChannel: ByteChannel {
    private let connection: NWConnection
    var onReady: (() -> Void)?
    var onFailed: ((String) -> Void)?

    init(_ connection: NWConnection) { self.connection = connection }

    func start(queue: DispatchQueue) {
        connection.stateUpdateHandler = { [weak self] state in
            switch state {
            case .ready: self?.onReady?()
            case .failed(let e): self?.onFailed?(e.localizedDescription)
            default: break
            }
        }
        connection.start(queue: queue)
    }

    func send(_ data: Data) {
        connection.send(content: data, completion: .contentProcessed { _ in })
    }

    func receive(_ completion: @escaping (Data?, Bool, String?) -> Void) {
        connection.receive(minimumIncompleteLength: 1, maximumLength: 16 * 1024 * 1024) { data, _, isComplete, error in
            completion(data, isComplete, error?.localizedDescription)
        }
    }

    func cancel() { connection.cancel() }
}

// MARK: - USB (raw socket fd from usbmuxd)

final class RawSocketChannel: ByteChannel {
    private let fd: Int32
    private var queue = DispatchQueue(label: "com.lidarlink.usbchannel")
    private var readSource: DispatchSourceRead?
    private var pendingReceive: ((Data?, Bool, String?) -> Void)?
    private var closed = false

    var onReady: (() -> Void)?
    var onFailed: ((String) -> Void)?

    init(fd: Int32) {
        self.fd = fd
        // Non-blocking so a partial chunk never stalls the read source.
        let flags = fcntl(fd, F_GETFL, 0)
        _ = fcntl(fd, F_SETFL, flags | O_NONBLOCK)
    }

    func start(queue: DispatchQueue) {
        self.queue = queue
        let source = DispatchSource.makeReadSource(fileDescriptor: fd, queue: queue)
        source.setEventHandler { [weak self] in self?.drain() }
        source.setCancelHandler { [weak self] in
            guard let self, !self.closed else { return }
            self.closed = true
            close(self.fd)
        }
        readSource = source
        source.resume()
        queue.async { [weak self] in self?.onReady?() }
    }

    func send(_ data: Data) {
        queue.async { [weak self] in
            guard let self, !self.closed else { return }
            data.withUnsafeBytes { raw in
                guard let base = raw.bindMemory(to: UInt8.self).baseAddress else { return }
                var off = 0
                while off < data.count {
                    let n = write(self.fd, base + off, data.count - off)
                    if n > 0 { off += n; continue }
                    if n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK) {
                        // Socket buffer full — brief spin; the stream is latest-wins upstream.
                        usleep(500)
                        continue
                    }
                    self.fail("write failed (\(errno))")
                    return
                }
            }
        }
    }

    func receive(_ completion: @escaping (Data?, Bool, String?) -> Void) {
        queue.async { [weak self] in
            guard let self else { return }
            self.pendingReceive = completion
            self.drain()
        }
    }

    private func drain() {
        guard let completion = pendingReceive, !closed else { return }
        var buffer = [UInt8](repeating: 0, count: 256 * 1024)
        let n = read(fd, &buffer, buffer.count)
        if n > 0 {
            pendingReceive = nil
            completion(Data(buffer[0..<n]), false, nil)
        } else if n == 0 {
            pendingReceive = nil
            completion(nil, true, nil)
        } else if errno == EAGAIN || errno == EWOULDBLOCK {
            // Nothing yet — the read source will fire again.
        } else {
            fail("read failed (\(errno))")
        }
    }

    func cancel() {
        queue.async { [weak self] in self?.readSource?.cancel() }
    }

    private func fail(_ reason: String) {
        guard !closed else { return }
        let cb = pendingReceive
        pendingReceive = nil
        cb?(nil, true, reason)
        onFailed?(reason)
        readSource?.cancel()
    }
}
