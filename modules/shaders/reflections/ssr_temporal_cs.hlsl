#include "ssr_rs.hlsli"
#include "cs.hlsli"
#include "camera.hlsli"
#include "math.hlsli"

ConstantBuffer<SSRTemporalCB> cb	: register(b0);
ConstantBuffer<CameraCB> camera		: register(b1);

RWTexture2D<float4> output			: register(u0);

Texture2D<float4> curr_resolve	    : register(t0);
Texture2D<float4> prev_resolve	    : register(t1);
Texture2D<float2> motion		    : register(t2);
Texture2D<float> depth_buffer	    : register(t3);
Texture2D<float4> raycast		    : register(t4);

SamplerState linear_sampler			: register(s0);
SamplerState point_sampler			: register(s1);

static const int2 sample_offset[9] = 
{ 
    int2(-1, -1), 
    int2(0, -1), 
    int2(1, -1), 
    int2(-1, 0), 
    int2(0, 0), 
    int2(1, 0), 
    int2(-1, 1), 
    int2(0, 1), 
    int2(1, 1) 
};

static float luma4(float3 color)
{
    return (color.g * 2.f) + (color.r + color.b);
}

static float hdr_weight4(float3 color, float exposure)
{
    return rcp(luma4(color) * exposure + 4.f);
}

static void resolver_aabb(Texture2D<float4> current_color, SamplerState current_sampler, float sharpness, float exposure_scale, float AABBScale, float2 uv, float2 texel_size, 
    inout float4 current_min, inout float4 current_max, inout float4 current_average, inout float4 current_output)
{
    // Modulate Luma HDR.

    float4 sample_colors[9];
    [unroll]
    for (uint i = 0; i < 9; i++)
    {
        sample_colors[i] = current_color.SampleLevel(current_sampler, uv + (sample_offset[i] * texel_size), 0.f);
    }

    float sample_weights[9];
    [unroll]
    for (uint j = 0; j < 9; j++)
    {
        sample_weights[j] = hdr_weight4(sample_colors[j].rgb, exposure_scale);
    }

    float total_weight = 0;
    [unroll]
    for (uint k = 0; k < 9; k++)
    {
        total_weight += sample_weights[k];
    }
    sample_colors[4] = (sample_colors[0] * sample_weights[0] + sample_colors[1] * sample_weights[1] + sample_colors[2] * sample_weights[2] + sample_colors[3] * sample_weights[3] + sample_colors[4] * sample_weights[4] +
        sample_colors[5] * sample_weights[5] + sample_colors[6] * sample_weights[6] + sample_colors[7] * sample_weights[7] + sample_colors[8] * sample_weights[8]) / total_weight;

    // Variance Clipping (AABB).

    float4 m1 = 0.f;
    float4 m2 = 0.f;
    [unroll]
    for (uint x = 0; x < 9; x++)
    {
        m1 += sample_colors[x];
        m2 += sample_colors[x] * sample_colors[x];
    }

    float4 mean = m1 / 9.f;
    float4 stddev = sqrt((m2 / 9.f) - mean * mean);

    current_min = mean - AABBScale * stddev;
    current_max = mean + AABBScale * stddev;

    current_output = sample_colors[4];
    current_min = min(current_min, current_output);
    current_max = max(current_max, current_output);
    current_average = mean;
}

static float4 clip_aabb(float3 aabb_min, float3 aabb_max, float4 p, float4 q)
{
    float3 p_clip = 0.5f * (aabb_max + aabb_min);
    float3 e_clip = 0.5f * (aabb_max - aabb_min) + 0.00000001f;

    float4 v_clip = q - float4(p_clip, p.w);
    float3 v_unit = v_clip.xyz / e_clip;
    float3 a_unit = abs(v_unit);
    float ma_unit = max(a_unit.x, max(a_unit.y, a_unit.z));

    if (ma_unit > 1.f)
    {
        return float4(p_clip, p.w) + v_clip / ma_unit;
    }
    else
    {
        return q; // Point inside AABB.
    }
}

static float2 camera_motion(float4 world_pos)
{
    return screen_space_velocity(mul(camera.view_proj, world_pos).xyw, mul(camera.prev_frame_view_proj, world_pos).xyw, camera.jitter, camera.prev_frame_jitter);
}

[numthreads(SSR_BLOCK_SIZE, SSR_BLOCK_SIZE, 1)]
[RootSignature(SSR_TEMPORAL_RS)]
void main(CSInput IN)
{
    const float2 uv = (IN.dispatch_thread_id.xy + 0.5f) * cb.inv_dimensions;

#if 0
    // Disable.
    output[IN.dispatch_thread_id.xy] = curr_resolve.SampleLevel(linear_sampler, uv, 0.f);
    return;
#endif

    float2 m = motion.SampleLevel(point_sampler, uv, 0);

    const float4 ray = raycast.SampleLevel(point_sampler, uv, 0);
    const float depth = depth_buffer.SampleLevel(point_sampler, uv, 0);
    if (ray.z > 0.f && depth < 1.f)
    {
        const float3 surface_pos = restore_view_space_position(camera.inv_proj, uv, depth);
        const float3 hit_pos = restore_view_space_position(camera.inv_proj, ray.xy, ray.z);
        const float3 virtual_pos = surface_pos + normalize(surface_pos) * length(hit_pos - surface_pos);

        const float4 surface_world_pos = mul(camera.inv_view, float4(surface_pos, 1.f));
        const float4 hit_world_pos = mul(camera.inv_view, float4(hit_pos, 1.f));
        const float4 virtual_world_pos = mul(camera.inv_view, float4(virtual_pos, 1.f));

        const float2 surface_camera_motion = camera_motion(surface_world_pos);
        const float2 hit_camera_motion = camera_motion(hit_world_pos);
        const float2 virtual_camera_motion = camera_motion(virtual_world_pos);

        // Own motion of the reflector and of the reflected object (motion vectors minus camera motion).
        const float2 hit_m = motion.SampleLevel(point_sampler, ray.xy, 0);
        m = virtual_camera_motion + (m - surface_camera_motion) + (hit_m - hit_camera_motion);
    }

    const float2 prev_uv = uv + m;

    float4 prev = prev_resolve.SampleLevel(linear_sampler, prev_uv, 0);

    float4 current = 0.f.xxxx;
    float4 current_min, current_max, current_average;
    resolver_aabb(curr_resolve, linear_sampler, 0.f, 10.f, 1.5f, uv, cb.inv_dimensions, current_min, current_max, current_average, current);

    prev.xyz = clip_aabb(current_min.xyz, current_max.xyz, clamp(current_average, current_min, current_max), prev).xyz;
    prev.a = clamp(prev.a, current_min.a, current_max.a);

    const float motion_pixels = length(m / cb.inv_dimensions);
    float blend_final = lerp(0.92f, 0.75f, saturate(motion_pixels / 16.f));
    blend_final = is_saturated(prev_uv) ? blend_final : 0.f;

    float4 result = lerp(current, prev, blend_final);
    output[IN.dispatch_thread_id.xy] = max(0, result);
}
