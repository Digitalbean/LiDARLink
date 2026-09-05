import Foundation
import simd

/// A pose graph over keyframe poses. Nodes are camera-to-world transforms;
/// constraints are measured relative transforms (sequential odometry + loop
/// closures). `optimize()` runs damped Gauss–Newton on the SE(3) manifold with
/// numeric Jacobians — slower per iteration than analytic, but hard to get
/// wrong, and a scan's graph is small.
///
/// Node 0 is held fixed (gauge). The linear system is dense for now; a scan's
/// keyframe count is in the hundreds. Sparse Cholesky is the scaling path.
public final class PoseGraph: @unchecked Sendable {

    public struct Constraint: Sendable, Equatable {
        public var from: Int32
        public var to: Int32
        public var measured: simd_float4x4   // relative transform, from → to
        public var weight: Float             // isotropic information

        public init(from: Int32, to: Int32, measured: simd_float4x4, weight: Float = 1) {
            self.from = from; self.to = to; self.measured = measured; self.weight = weight
        }
    }

    private let lock = NSLock()
    private var order: [Int32] = []
    private var indexOf: [Int32: Int] = [:]
    private var poses: [simd_float4x4] = []
    private var constraints: [Constraint] = []

    public init() {}

    // MARK: Building

    public func addNode(id: Int32, pose: simd_float4x4) {
        lock.lock(); defer { lock.unlock() }
        guard indexOf[id] == nil else { return }
        indexOf[id] = order.count
        order.append(id)
        poses.append(pose)
    }

    public func addConstraint(_ c: Constraint) {
        lock.lock(); defer { lock.unlock() }
        guard indexOf[c.from] != nil, indexOf[c.to] != nil else { return }
        constraints.append(c)
    }

    public func pose(id: Int32) -> simd_float4x4? {
        lock.lock(); defer { lock.unlock() }
        guard let i = indexOf[id] else { return nil }
        return poses[i]
    }

    public var nodeCount: Int { lock.lock(); defer { lock.unlock() }; return order.count }
    public var constraintCount: Int { lock.lock(); defer { lock.unlock() }; return constraints.count }

    /// All optimized poses, keyed by node id.
    public func allPoses() -> [Int32: simd_float4x4] {
        lock.lock(); defer { lock.unlock() }
        var result: [Int32: simd_float4x4] = [:]
        for (i, id) in order.enumerated() { result[id] = poses[i] }
        return result
    }

    // MARK: Optimization

    @discardableResult
    public func optimize(iterations: Int = 12, epsilon: Float = 1e-4) -> (initial: Float, final: Float) {
        lock.lock(); defer { lock.unlock() }
        let n = order.count
        guard n >= 2, !constraints.isEmpty else { return (0, 0) }

        let free = n - 1                 // node 0 fixed
        let dim = free * 6
        var lambda: Float = 1e-3
        let initialError = totalError()
        var currentError = initialError

        for _ in 0..<iterations {
            var h = [Float](repeating: 0, count: dim * dim)
            var g = [Float](repeating: 0, count: dim)

            for c in constraints {
                let i = indexOf[c.from]!, j = indexOf[c.to]!
                let e = errorVector(from: i, to: j, measured: c.measured)
                let ji = i == 0 ? nil : numericJacobian(node: i, from: i, to: j, measured: c.measured, base: e)
                let jj = j == 0 ? nil : numericJacobian(node: j, from: i, to: j, measured: c.measured, base: e)

                if let ji { accumulate(&h, &g, block: i - 1, blockB: i - 1, ja: ji, jb: ji, e: e, w: c.weight, dim: dim) }
                if let jj { accumulate(&h, &g, block: j - 1, blockB: j - 1, ja: jj, jb: jj, e: e, w: c.weight, dim: dim) }
                if let ji, let jj {
                    accumulate(&h, &g, block: i - 1, blockB: j - 1, ja: ji, jb: jj, e: nil, w: c.weight, dim: dim)
                    accumulate(&h, &g, block: j - 1, blockB: i - 1, ja: jj, jb: ji, e: nil, w: c.weight, dim: dim)
                }
            }

            for d in 0..<dim { h[d * dim + d] += lambda * h[d * dim + d] + 1e-9 }
            guard let delta = Linear.solveSPD(h, negated(g), n: dim) else { lambda *= 10; continue }

            let backup = poses
            for k in 1..<n {
                let o = (k - 1) * 6
                poses[k] = Lie.retract(poses[k],
                                       rotation: SIMD3<Float>(delta[o], delta[o + 1], delta[o + 2]),
                                       translation: SIMD3<Float>(delta[o + 3], delta[o + 4], delta[o + 5]))
            }
            let newError = totalError()
            if newError < currentError {
                let improvement = currentError - newError
                currentError = newError
                lambda = max(lambda * 0.7, 1e-7)
                if improvement < epsilon { break }
            } else {
                poses = backup
                lambda *= 4
            }
        }
        return (initialError, currentError)
    }

    // MARK: Internals

    private func totalError() -> Float {
        var sum: Float = 0
        for c in constraints {
            let e = errorVector(from: indexOf[c.from]!, to: indexOf[c.to]!, measured: c.measured)
            sum += c.weight * simd_dot(SIMD3<Float>(e[0], e[1], e[2]), SIMD3<Float>(e[0], e[1], e[2]))
            sum += c.weight * simd_dot(SIMD3<Float>(e[3], e[4], e[5]), SIMD3<Float>(e[3], e[4], e[5]))
        }
        return sum
    }

    private func errorVector(from i: Int, to j: Int, measured: simd_float4x4) -> [Float] {
        let (r, t) = Lie.betweenError(from: poses[i], to: poses[j], measured: measured)
        return [r.x, r.y, r.z, t.x, t.y, t.z]
    }

    /// 6x6 numeric Jacobian of the edge error w.r.t. `node`'s tangent.
    private func numericJacobian(node: Int, from i: Int, to j: Int, measured: simd_float4x4, base e: [Float]) -> [Float] {
        let eps: Float = 1e-4
        var jac = [Float](repeating: 0, count: 36)
        let original = poses[node]
        for d in 0..<6 {
            var omega = SIMD3<Float>(0, 0, 0), rho = SIMD3<Float>(0, 0, 0)
            if d < 3 { omega[d] = eps } else { rho[d - 3] = eps }
            poses[node] = Lie.retract(original, rotation: omega, translation: rho)
            let ePert = errorVector(from: i, to: j, measured: measured)
            for r in 0..<6 { jac[r * 6 + d] = (ePert[r] - e[r]) / eps }
        }
        poses[node] = original
        return jac
    }

    private func negated(_ v: [Float]) -> [Float] { v.map { -$0 } }

    /// Adds `w · Jaᵀ Jb` into H's (block, blockB) 6x6 sub-block, and (when `e`
    /// is given) `w · Jaᵀ e` into g's `block` sub-vector.
    private func accumulate(_ h: inout [Float], _ g: inout [Float],
                            block: Int, blockB: Int, ja: [Float], jb: [Float], e: [Float]?, w: Float, dim: Int) {
        let ro = block * 6, co = blockB * 6
        for a in 0..<6 {
            for b in 0..<6 {
                var s: Float = 0
                for k in 0..<6 { s += ja[k * 6 + a] * jb[k * 6 + b] }
                h[(ro + a) * dim + (co + b)] += w * s
            }
            if let e {
                var s: Float = 0
                for k in 0..<6 { s += ja[k * 6 + a] * e[k] }
                g[ro + a] += w * s
            }
        }
    }
}
