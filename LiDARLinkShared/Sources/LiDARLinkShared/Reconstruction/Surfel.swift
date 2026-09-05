import Foundation
import simd

/// A surface element: a small oriented disk standing in for a patch of observed
/// surface. The live map is a set of these — cheaper to update than a volume and
/// they move rigidly when the pose graph corrects a keyframe, instead of needing
/// a re-integration.
public struct Surfel: Sendable, Equatable {
    /// World-space centre.
    public var position: SIMD3<Float>
    /// Unit outward normal (points back toward the observing camera).
    public var normal: SIMD3<Float>
    /// Accumulated colour, 0…255 per channel (divide by nothing — already the mean).
    public var color: SIMD3<Float>
    /// Disk radius in metres (roughly the pixel footprint at capture range).
    public var radius: Float
    /// Sum of observation weights folded in so far.
    public var weight: Float
    /// The keyframe this surfel is anchored to — a pose-graph correction for
    /// that keyframe transforms this surfel.
    public var owner: Int32
    /// Last keyframe that observed it (for stale-surfel cleanup).
    public var lastSeen: Int32
    /// EWMA of how far incoming samples land from this surfel's plane. High
    /// after many observations = real sub-surfel relief → the surfel splits.
    public var roughness: Float
    /// How many times this surfel has been subdivided. Bounds the recursion.
    public var splitLevel: UInt8
    /// Ambient occlusion, 1 = fully open, 0 = fully occluded. Filled by
    /// `SurfelMap.updateAmbientOcclusion`; 1 until then.
    public var ao: Float
    /// Average unoccluded direction (the "bent normal") — for soft key-light
    /// shading. Equals `normal` until AO is computed.
    public var bentNormal: SIMD3<Float>
    /// Estimated surface colour with the capture lighting divided out (de-lit).
    /// Equals `color` until `SurfelMap.updateDelighting` runs. This is what a
    /// relighting pass multiplies.
    public var albedo: SIMD3<Float>
    /// The most head-on this surfel has been observed, cos(angle) in 0…1.
    /// A wall only ever seen at a grazing angle is under-sampled.
    public var bestViewCos: Float
    /// Reconstruction debt in 0…1 — how much this patch still needs (few
    /// observations, grazing-only, unresolved relief). Filled by
    /// `SurfelMap.updateDebt`.
    public var debt: Float
    /// Semantic class (`RichPointCloud.Classification` raw value). 1 =
    /// unclassified until `SurfelMap.updateClassification` runs.
    public var classification: UInt8

    public init(position: SIMD3<Float>, normal: SIMD3<Float>, color: SIMD3<Float>,
                radius: Float, weight: Float, owner: Int32, lastSeen: Int32,
                roughness: Float = 0, splitLevel: UInt8 = 0,
                ao: Float = 1, bentNormal: SIMD3<Float>? = nil, albedo: SIMD3<Float>? = nil,
                bestViewCos: Float = 0, debt: Float = 0, classification: UInt8 = 1) {
        self.position = position
        self.normal = normal
        self.color = color
        self.radius = radius
        self.weight = weight
        self.owner = owner
        self.lastSeen = lastSeen
        self.roughness = roughness
        self.splitLevel = splitLevel
        self.ao = ao
        self.bentNormal = bentNormal ?? normal
        self.albedo = albedo ?? color
        self.bestViewCos = bestViewCos
        self.debt = debt
        self.classification = classification
    }

    public var colorBytes: SIMD3<UInt8> {
        SIMD3<UInt8>(UInt8(max(0, min(255, color.x))),
                     UInt8(max(0, min(255, color.y))),
                     UInt8(max(0, min(255, color.z))))
    }
}
