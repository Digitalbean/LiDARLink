#include <metal_stdlib>
using namespace metal;

// Eye-dome lighting: a screen-space shading cue computed purely from the
// depth buffer SceneKit already produces while rasterizing the point cloud.
// No per-point normals, no extra data sent anywhere — a pixel darkens when
// its on-screen neighbours are closer to the camera than it is, the same
// silhouette-shadow cue Potree/CloudCompare use to make a raw point cloud
// read as a solid shape instead of a scatter of dots.

struct EDLVertexOut {
    float4 position [[position]];
    float2 uv;
};

// SceneKit issues 4 vertices (a triangle strip covering the screen) for a
// DRAW_QUAD pass that supplies its own vertex shader.
vertex EDLVertexOut edl_vertex(uint vertexID [[vertex_id]]) {
    constexpr float2 corners[4] = { float2(-1, -1), float2(1, -1), float2(-1, 1), float2(1, 1) };
    // Metal texture coordinates are y-down; NDC is y-up, so the top NDC
    // corners map to v=0 and the bottom corners to v=1.
    constexpr float2 uvs[4]     = { float2(0, 1),   float2(1, 1),  float2(0, 0), float2(1, 0) };
    EDLVertexOut out;
    out.position = float4(corners[vertexID], 0, 1);
    out.uv = uvs[vertexID];
    return out;
}

// Matches the fixed camera planes set in PointCloudSceneController.init().
constant float kZNear = 0.02;
constant float kZFar = 1000.0;

// SceneKit's Metal depth buffer here is reversed-Z: 1.0 = the near plane,
// 0.0 = the far plane / clear value (confirmed by direct visualization —
// the standard-convention formula and background check were both backwards
// from what actually came back). Un-projects back to view-space distance.
static inline float edl_linearDepth(float d) {
    return (kZNear * kZFar) / (kZNear + d * (kZFar - kZNear));
}

fragment float4 edl_fragment(EDLVertexOut in [[stage_in]],
                              texture2d<float> colorSampler [[texture(0)]],
                              texture2d<float> depthSampler [[texture(1)]]) {
    constexpr sampler s(coord::normalized, filter::linear, address::clamp_to_edge);
    float4 color = colorSampler.sample(s, in.uv);

    float centerRaw = depthSampler.sample(s, in.uv).r;
    // Nothing rendered here (background) clears to 0 under reversed-Z.
    if (centerRaw <= 1e-6) { return color; }

    float2 texel = 1.0 / float2(depthSampler.get_width(), depthSampler.get_height());
    float centerLog = log2(edl_linearDepth(centerRaw));

    constexpr float2 offsets[8] = {
        float2(-1, 0), float2(1, 0), float2(0, -1), float2(0, 1),
        float2(-1, -1), float2(1, -1), float2(-1, 1), float2(1, 1)
    };
    constexpr float radiusPx = 3.0;

    float response = 0.0;
    for (int i = 0; i < 8; i++) {
        float2 uv = in.uv + offsets[i] * texel * radiusPx;
        float neighborRaw = depthSampler.sample(s, uv).r;
        if (neighborRaw <= 1e-6) { continue; }
        float neighborLog = log2(edl_linearDepth(neighborRaw));
        // Only a neighbour that's *closer* than the center contributes —
        // that's the silhouette-occlusion cue. A neighbour that's farther
        // away (center is the near surface) contributes nothing.
        response += max(0.0, centerLog - neighborLog);
    }
    response *= 0.125; // / 8

    constexpr float strength = 12.0;
    float shade = exp(-response * strength);
    return float4(color.rgb * shade, color.a);
}
