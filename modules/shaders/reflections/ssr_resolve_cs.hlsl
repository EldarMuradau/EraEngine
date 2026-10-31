#include "ssr_rs.hlsli"
#include "cs.hlsli"
#include "camera.hlsli"
#include "normal.hlsli"
#include "math.hlsli"
#include "brdf.hlsli"

ConstantBuffer<SSRResolveCB> cb	    : register(b0);
ConstantBuffer<CameraCB> camera	    : register(b1);

Texture2D<float> depth_buffer		    : register(t0);
Texture2D<float3> world_normals_roughness	: register(t1);
Texture2D<float4> reflection		    : register(t2);
Texture2D<float4> hdr_color  		    : register(t3);
Texture2D<float2> motion			    : register(t4);

RWTexture2D<float4> output              : register(u0);

SamplerState linear_sampler			    : register(s0);
SamplerState point_sampler			    : register(s1);

static const int2 resolve_offset[] =
{
    int2(0, 0),
    int2(0, 1),
    int2(1, -1),
    int2(-1, -1),
};

#define SSR_RESOLVE_RAD  4
#define SSR_RESOLVE_RAD2 (SSR_RESOLVE_RAD * 2)

groupshared uint group_r[(SSR_BLOCK_SIZE + SSR_RESOLVE_RAD2) * (SSR_BLOCK_SIZE + SSR_RESOLVE_RAD2)];
groupshared uint group_g[(SSR_BLOCK_SIZE + SSR_RESOLVE_RAD2) * (SSR_BLOCK_SIZE + SSR_RESOLVE_RAD2)];
groupshared uint group_b[(SSR_BLOCK_SIZE + SSR_RESOLVE_RAD2) * (SSR_BLOCK_SIZE + SSR_RESOLVE_RAD2)];
groupshared uint group_a[(SSR_BLOCK_SIZE + SSR_RESOLVE_RAD2) * (SSR_BLOCK_SIZE + SSR_RESOLVE_RAD2)];

static void pack_resolve_data(uint index, float4 raycast, float4 color)
{
    group_r[index] = f32tof16(raycast.r) | (f32tof16(color.r) << 16);
    group_g[index] = f32tof16(raycast.g) | (f32tof16(color.g) << 16);
    group_b[index] = f32tof16(raycast.b) | (f32tof16(color.b) << 16);
    group_a[index] = f32tof16(raycast.a) | (f32tof16(color.a) << 16);
}

static void get_resolve_data(uint index, out float4 raycast, out float4 color)
{
    uint rr = group_r[index];
    uint gg = group_g[index];
    uint bb = group_b[index];
    uint aa = group_a[index];
    raycast = float4(f16tof32(rr), f16tof32(gg), f16tof32(bb), f16tof32(aa));
    color = float4(f16tof32(rr >> 16), f16tof32(gg >> 16), f16tof32(bb >> 16), f16tof32(aa >> 16));
}

[numthreads(SSR_BLOCK_SIZE + SSR_RESOLVE_RAD2, SSR_BLOCK_SIZE + SSR_RESOLVE_RAD2, 1)]
[RootSignature(SSR_RESOLVE_RS)]
void main(CSInput IN)
{
    const float3 luminance_weights = float3(0.2126f, 0.7152f, 0.0722f);
    const float border_attenuation_distance = 0.25f;

    const uint2 uv_int = (IN.group_id.xy * SSR_BLOCK_SIZE) + IN.group_thread_id.xy - SSR_RESOLVE_RAD;
    const float2 uv = (uv_int.xy + 0.5f) * cb.inv_dimensions;

    const float2 uv_m = motion.SampleLevel(point_sampler, uv, 0);
    const float3 normal_and_roughness = world_normals_roughness.SampleLevel(point_sampler, uv + uv_m, 0);
    const float3 normal = unpack_normal(normal_and_roughness.xy);
    const float3 N = normalize(mul(camera.view, float4(normal, 0.f)).xyz);
    const float roughness = clamp(normal_and_roughness.z, 0.03f, 0.97f);
     
    const float depth = depth_buffer.SampleLevel(point_sampler, uv, 0);
    const float3 view_pos = restore_view_space_position(camera.inv_proj, uv, depth);
    const float3 V = normalize(-view_pos);

    const float ndot_v = saturate(dot(N, V));
    const float sqrt_roughness = sqrt(roughness);
    float cone_tangent = lerp(0.f, roughness * (1.f - SSR_GGX_IMPORTANCE_SAMPLE_BIAS), ndot_v * sqrt_roughness);
    cone_tangent *= lerp(saturate(ndot_v * 2.f), 1.f, sqrt_roughness);

    float4 raycast_result = reflection.SampleLevel(linear_sampler, uv, 0);
    float hit = raycast_result.z > 0.f;

    raycast_result.z = abs(raycast_result.z); // Remove sign.

    float source_mip = clamp(log2(cone_tangent * max(cb.dimensions.x, cb.dimensions.y)), 0.f, 4.f);

    const float2 m = motion.SampleLevel(point_sampler, raycast_result.xy, 0);
    float4 scene_color = hdr_color.SampleLevel(linear_sampler, raycast_result.xy + m, source_mip);
    scene_color.w = hit; // Store hit result.

    pack_resolve_data(IN.group_index, raycast_result, scene_color);

    GroupMemoryBarrierWithGroupSync();

    if (IN.group_thread_id.x < SSR_RESOLVE_RAD | IN.group_thread_id.y < SSR_RESOLVE_RAD | IN.group_thread_id.x >= SSR_BLOCK_SIZE + SSR_RESOLVE_RAD || IN.group_thread_id.y >= SSR_BLOCK_SIZE + SSR_RESOLVE_RAD)
    {
        return;
    }

    if (depth == 1.f)
    {
        output[uv_int.xy] = float4(0.f, 0.f, 0.f, 0.f);
        return;
    }


#if 0
    // Disable.
    float border_dist = min(1.f - max(raycast_result.x, raycast_result.y), min(raycast_result.x, raycast_result.y));
    float border_attenuation = saturate(border_dist / border_attenuation_distance);

    output[uv_int.xy] = float4(scene_color.rgb, border_attenuation * scene_color.a);

    output[uv_int.xy] = float4(hit.xxx, 1.f);
    return;
#endif

    float4 result = 0.f.xxxx;
    float total_weight = 0.f;

    [unroll]
    for (uint i = 0; i < 4; ++i)
    {
        int2 offset = resolve_offset[i] * 2;
        int2 neighbor_group_thread_id = IN.group_thread_id.xy + offset;
        uint neighbor_index = flatten_2d(neighbor_group_thread_id, SSR_BLOCK_SIZE + SSR_RESOLVE_RAD2);

        float4 neighbor_raycast_result;
        float4 neighbor_scene_color;
        get_resolve_data(neighbor_index, neighbor_raycast_result, neighbor_scene_color);

        float2 neighbor_uv = neighbor_raycast_result.xy;
        float neighbor_depth = neighbor_raycast_result.z;
        float neighbor_pdf = neighbor_raycast_result.w;

        float neighbor_hit = neighbor_scene_color.a;

        const float3 neighbor_view_pos = camera.restore_view_space_position(neighbor_uv, neighbor_depth);

        // BRDF weight
        float3 L = normalize(neighbor_view_pos - view_pos);
        float3 H = normalize(L + V);

        float ndot_h = saturate(dot(N, H));
        float ndot_l = saturate(dot(N, L));

        float G = geometry_smith(ndot_l, ndot_v, roughness);
        float D = distribution_ggx(ndot_h, roughness);
        float specular_light = G * D * M_PI / 4.f;

        float weight = specular_light / max(neighbor_pdf, 0.00001f);

        float border_dist = min(1.f - max(neighbor_uv.x, neighbor_uv.y), min(neighbor_uv.x, neighbor_uv.y));
        float border_attenuation = saturate(border_dist / border_attenuation_distance);
       
        neighbor_scene_color.a = border_attenuation * neighbor_hit;
        //neighborSceneColor.rgb /= 1.f + dot(neighborSceneColor.rgb, luminanceWeights);

        result += neighbor_scene_color * weight;
        total_weight += weight;
    }

    result /= total_weight;
    //result.rgb /= 1.f - dot(result.rgb, luminanceWeights);

    output[uv_int.xy] = max(1e-5f, result);
}
