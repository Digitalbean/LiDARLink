import Foundation

/// Dense linear-algebra fallbacks. Row-major `Float` matrices.
public enum Linear {

    /// Solves `A x = b` for a symmetric positive-definite `A` (n×n row-major)
    /// via Cholesky with a small diagonal jitter for near-singular systems.
    /// Returns nil if `A` is not positive-definite even after jittering.
    public static func solveSPD(_ a: [Float], _ b: [Float], n: Int) -> [Float]? {
        guard a.count == n * n, b.count == n else { return nil }
        var m = a
        for attempt in 0..<3 {
            if attempt > 0 {
                let jitter = powf(10, Float(attempt) - 6)
                for i in 0..<n { m[i * n + i] += jitter }
            }
            if let l = cholesky(m, n: n) {
                return backSubstitute(l, b, n: n)
            }
            m = a
        }
        return nil
    }

    private static func cholesky(_ a: [Float], n: Int) -> [Float]? {
        var l = [Float](repeating: 0, count: n * n)
        for j in 0..<n {
            var d = a[j * n + j]
            for k in 0..<j { d -= l[j * n + k] * l[j * n + k] }
            guard d > 1e-15 else { return nil }
            let ljj = sqrt(d)
            l[j * n + j] = ljj
            for i in (j + 1)..<n {
                var s = a[i * n + j]
                for k in 0..<j { s -= l[i * n + k] * l[j * n + k] }
                l[i * n + j] = s / ljj
            }
        }
        return l
    }

    private static func backSubstitute(_ l: [Float], _ b: [Float], n: Int) -> [Float] {
        var y = [Float](repeating: 0, count: n)
        for i in 0..<n {
            var s = b[i]
            for k in 0..<i { s -= l[i * n + k] * y[k] }
            y[i] = s / l[i * n + i]
        }
        var x = [Float](repeating: 0, count: n)
        for i in stride(from: n - 1, through: 0, by: -1) {
            var s = y[i]
            for k in (i + 1)..<n { s -= l[k * n + i] * x[k] }
            x[i] = s / l[i * n + i]
        }
        return x
    }
}
