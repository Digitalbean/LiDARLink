import Foundation
import simd

/// Writes a `RichPointCloud` as LAS 1.2, point data record format 2 (XYZ +
/// intensity + classification + RGB) — the format ReCap, CloudCompare, Revit
/// and survey tools expect. Axes are remapped to the LAS convention (Z up).
public enum LASExporter {

    public static func data(from cloud: RichPointCloud) -> Data {
        let pts = cloud.points
        // ARKit (x right, y up, z back) → LAS (x east, y north, z up).
        func remap(_ v: SIMD3<Float>) -> SIMD3<Double> {
            SIMD3<Double>(Double(v.x), Double(-v.z), Double(v.y))
        }

        var lo = SIMD3<Double>(repeating: .greatestFiniteMagnitude)
        var hi = SIMD3<Double>(repeating: -.greatestFiniteMagnitude)
        for p in pts { let r = remap(p.position); lo = simd_min(lo, r); hi = simd_max(hi, r) }
        if pts.isEmpty { lo = .zero; hi = .zero }
        let scale = 0.001
        let offset = lo

        var out = Data()
        func u8(_ x: UInt8) { out.append(x) }
        func u16(_ x: UInt16) { withUnsafeBytes(of: x.littleEndian) { out.append(contentsOf: $0) } }
        func u32(_ x: UInt32) { withUnsafeBytes(of: x.littleEndian) { out.append(contentsOf: $0) } }
        func i32(_ x: Int32) { withUnsafeBytes(of: x.littleEndian) { out.append(contentsOf: $0) } }
        func f64(_ x: Double) { withUnsafeBytes(of: x.bitPattern.littleEndian) { out.append(contentsOf: $0) } }
        func str(_ s: String, _ n: Int) {
            var b = Array(s.utf8.prefix(n)); b.append(contentsOf: repeatElement(0, count: n - b.count)); out.append(contentsOf: b)
        }

        // --- Public Header Block (227 bytes, LAS 1.2) ---
        out.append(contentsOf: Array("LASF".utf8))   // file signature
        u16(0)                                       // file source id
        u16(0)                                       // global encoding
        out.append(contentsOf: [UInt8](repeating: 0, count: 16))   // project id GUID
        u8(1); u8(2)                                  // version 1.2
        str("LiDARLink", 32)                          // system identifier
        str("LiDARLink v2", 32)                       // generating software
        let now = Calendar(identifier: .gregorian).dateComponents([.day, .year], from: Date())
        u16(UInt16(now.day ?? 1)); u16(UInt16(now.year ?? 2026))
        u16(227)                                      // header size
        u32(227)                                      // offset to point data
        u32(0)                                        // number of VLRs
        u8(2)                                         // point data record format
        u16(26)                                       // point data record length
        u32(UInt32(pts.count))                        // number of point records
        for _ in 0..<5 { u32(UInt32(pts.count)) }     // by-return (approx: all return 1)
        f64(scale); f64(scale); f64(scale)
        f64(offset.x); f64(offset.y); f64(offset.z)
        f64(hi.x); f64(lo.x); f64(hi.y); f64(lo.y); f64(hi.z); f64(lo.z)

        // --- Point records ---
        out.reserveCapacity(227 + pts.count * 26)
        for p in pts {
            let r = remap(p.position)
            i32(Int32(((r.x - offset.x) / scale).rounded()))
            i32(Int32(((r.y - offset.y) / scale).rounded()))
            i32(Int32(((r.z - offset.z) / scale).rounded()))
            u16(UInt16(simd_clamp(p.confidence, 0, 1) * 65535))     // intensity
            u8(0b0000_1001)                                          // return 1 of 1
            u8(p.classification.rawValue)
            u8(0)                                                    // scan angle rank
            u8(0)                                                    // user data
            u16(0)                                                   // point source id
            u16(UInt16(p.color.x) * 257)
            u16(UInt16(p.color.y) * 257)
            u16(UInt16(p.color.z) * 257)
        }
        return out
    }
}
