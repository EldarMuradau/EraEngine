#include "ssr_rs.hlsli"
#include "cs.hlsli"
#include "camera.hlsli"
#include "normal.hlsli"
#include "brdf.hlsli"
#include "random.hlsli"
#include "math.hlsli"

ConstantBuffer<SSRRaycastCB> cb	    : register(b0);
ConstantBuffer<CameraCB> camera	    : register(b1);

Texture2D<float> depth_buffer		    : register(t0);
Texture2D<float> linear_depth_buffer	    : register(t1);
Texture2D<float3> world_normals_roughness	: register(t2);
Texture2D<float2> noise     		    : register(t3);
Texture2D<float2> motion			    : register(t4);

RWTexture2D<float4> output			    : register(u0);

SamplerState linear_sampler			    : register(s0);
SamplerState point_sampler			    : register(s1);

static float distance_squared(float2 a, float2 b) 
{
    a -= b;
    return dot(a, a);
}

static void swap(inout float a, inout float b)
{
    float t = a;
    a = b;
    b = t;
}

static bool intersects_depth_buffer(float scene_z_max, float ray_z_min, float ray_z_max)
{
    // Increase thickness along distance. 
    float thickness = max(scene_z_max * 0.2f, 1.f);

    // Effectively remove line/tiny artifacts, mostly caused by Zbuffers precision.
    float depth_scale = min(1.f, scene_z_max / 100.f);
    scene_z_max += lerp(0.05f, 0.f, depth_scale);

    return (ray_z_min >= scene_z_max) && (ray_z_max - thickness <= scene_z_max);
}

static bool trace_screen_space_ray(float3 ray_origin, float3 ray_direction, float jitter, float roughness,
    out float2 hit_pixel)
{
    hit_pixel = float2(-1.f, -1.f);

    const float camera_near_plane = -camera.projection_params.x; // Now negative.
    float ray_length = ((ray_origin.z + ray_direction.z * cb.max_distance) > camera_near_plane)
        ? (camera_near_plane - ray_origin.z) / ray_direction.z
        : cb.max_distance;

    float3 ray_end = ray_origin + ray_direction * ray_length;

    // Project into screen space.
    float4 H0 = mul(camera.proj, float4(ray_origin, 1.f));
    float4 H1 = mul(camera.proj, float4(ray_end, 1.f));
    float k0 = 1.f / H0.w;
    float k1 = 1.f / H1.w;
    float3 Q0 = ray_origin * k0;
    float3 Q1 = ray_end * k1;

    Q0.z *= -1.f;
    Q1.z *= -1.f;

    // Screen space endpoints.
    float2 P0 = H0.xy * k0;
    float2 P1 = H1.xy * k1;
    P0 = P0 * float2(0.5f, -0.5f) + float2(0.5f, 0.5f);
    P1 = P1 * float2(0.5f, -0.5f) + float2(0.5f, 0.5f);
    P0.xy *= cb.dimensions;
    P1.xy *= cb.dimensions;

    // Avoid degenerate lines.
    P1 += (distance_squared(P0, P1) < 0.0001f) ? 0.01f : 0.f;

    float2 screen_delta = P1 - P0;

    bool permute = false;
    if (abs(screen_delta.x) < abs(screen_delta.y))
    {
        permute = true;
        screen_delta = screen_delta.yx;
        P0 = P0.yx;
        P1 = P1.yx;
    }

    float step_sign = sign(screen_delta.x);
    float invdx = step_sign / screen_delta.x;

    // Derivatives of Q and k.
    float3 d_q = (Q1 - Q0) * invdx;
    float  dk = (k1 - k0) * invdx;
    float2 d_p = float2(step_sign, screen_delta.y * invdx);

    float z_min = min(-ray_end.z, -ray_origin.z);
    float z_max = max(-ray_end.z, -ray_origin.z);


    // Stride based on roughness. Matte materials will recieve higher stride.
    float alpha_roughness = roughness * roughness;
    float alpha_roughness_sq = alpha_roughness * alpha_roughness;

    float stride_scale = 1.f - min(1.f, -ray_origin.z / cb.stride_cutoff);
    float stride_roughness_scale = lerp(cb.min_stride, cb.max_stride, min(alpha_roughness_sq, 1.f)); // Climb exponentially at extreme conditions.
    float pixel_stride = 1.f + stride_scale * stride_roughness_scale;

    // Scale derivatives by stride.
    d_p *= pixel_stride; d_q *= pixel_stride; dk *= pixel_stride;
    P0 += d_p * jitter; Q0 += d_q * jitter; k0 += dk * jitter;

    // Start values and derivatives packed together -> only one operation needed to increase.
    float4 PQk = float4(P0, Q0.z, k0);
    float4 d_p_qk = float4(d_p, d_q.z, dk);

    float level = 0.f;

    float prev_z_max_estimate = -ray_origin.z;
    float ray_z_min = -ray_origin.z;
    float ray_z_max = -ray_origin.z;
    float scene_z_max = ray_z_max + 100000.0f;
    
    uint step_count = 0;
    float end = P1.x * step_sign;

    hit_pixel = float2(0.f, 0.f);

    while (((PQk.x * step_sign) <= end) &&
        (step_count < cb.num_steps) &&
        (!intersects_depth_buffer(scene_z_max, ray_z_min, ray_z_max)) &&
        (scene_z_max != 0.f))
    {
        if (any(hit_pixel < 0.f || hit_pixel > 1.f))
        {
            return false;
        }

        ray_z_min = prev_z_max_estimate;

        // Compute the value at 1/2 step into the future.
        ray_z_max = (d_p_qk.z * 0.5f + PQk.z) / (d_p_qk.w * 0.5f + PQk.w);
        ray_z_max = clamp(ray_z_max, z_min, z_max);

        prev_z_max_estimate = ray_z_max;

        if (ray_z_min > ray_z_max) { swap(ray_z_min, ray_z_max); }

        const float hzb_bias = 0.05f;
        const float hzb_min_step = 0.005f;

        // A simple HZB approach based on roughness.
        level += max(hzb_bias * roughness, hzb_min_step);
        level = min(level, 6.f);

        hit_pixel = permute ? PQk.yx : PQk.xy;
        hit_pixel *= cb.inv_dimensions;

        scene_z_max = linear_depth_buffer.SampleLevel(point_sampler, hit_pixel, level); // Already in world units.

        PQk += d_p_qk;
        ++step_count;
    }

    return intersects_depth_buffer(scene_z_max, ray_z_min, ray_z_max);
}

[numthreads(SSR_BLOCK_SIZE, SSR_BLOCK_SIZE, 1)]
[RootSignature(SSR_RAYCAST_RS)]
void main(CSInput IN)
{
    float2 uv = (IN.dispatch_thread_id.xy + 0.5f) * cb.inv_dimensions;

    const float depth = depth_buffer.SampleLevel(point_sampler, uv, 0);
    if (depth == 1.f)
    {
        output[IN.dispatch_thread_id.xy] = float4(0.f, 0.f, -1.f, 0.f);
        return;
    }

    const float2 uv_m = motion.SampleLevel(point_sampler, uv, 0);
    const float3 normal_and_roughness = world_normals_roughness.SampleLevel(point_sampler, uv + uv_m, 0);

	if (all(normal_and_roughness.xy == float2(0.f, 0.f)))
	{
		// If normal is zero length, just return no hit.
		// This happens if the window is resized, because we are sampling the last frame's normals.
		output[IN.dispatch_thread_id.xy] = float4(0.f, 0.f, -1.f, 0.f);
		return;
	}

    const float3 normal = unpack_normal(normal_and_roughness.xy);
    const float3 view_normal = mul(camera.view, float4(normal, 0.f)).xyz;
    float roughness = normal_and_roughness.z;
#if 1
    if (roughness > 0.85f)
    {
        output[IN.dispatch_thread_id.xy] = float4(0.f, 0.f, -1.f, 0.f);
        return;
    }
#endif

    const float3 view_pos = restore_view_space_position(camera.inv_proj, uv, depth);
    const float3 view_dir = normalize(view_pos);

#if 1
    roughness = clamp(roughness * roughness * roughness, 0.03f, 0.97f); // Raising the roughness to a power gets rid of some flickering.

    float2 h = halton23(cb.frame_index & 31);
    uint3 noise_dims;
    noise.GetDimensions(0, noise_dims.x, noise_dims.y, noise_dims.z);
    float2 xi = noise.SampleLevel(linear_sampler, (uv + h) * cb.dimensions / float2(noise_dims.xy), 0);
    xi.y = lerp(xi.y, 0.f, SSR_GGX_IMPORTANCE_SAMPLE_BIAS);

    float4 H = importance_sample_ggx(xi, view_normal, roughness);
    float3 refl_dir = reflect(view_dir, H.xyz);
#else
    // Perfect reflection.
    float3 refl_dir = reflect(view_dir, view_normal);
    float4 H = 1.f;
#endif

#if 0
    // Don't allow reflections towards camera.
    if (dot(refl_dir, -view_dir) > 0.f)
    {
        output[IN.dispatch_thread_id.xy] = float4(0.f, 0.f, -1.f, 0.f);
        return;
    }
#endif

    float jitter = 0.f;// interleavedGradientNoise(IN.dispatchThreadID.xy, cb.frameIndex);

    float2 hit_pixel;
    bool hit = trace_screen_space_ray(view_pos + view_normal * 0.005f, refl_dir, jitter, roughness, hit_pixel);

    float hit_depth = depth_buffer.SampleLevel(point_sampler, hit_pixel, 0);
    float hit_mul = hit ? 1.f : -1.f; // We pack the info whether we hit or not in the sign of the depth.
    output[IN.dispatch_thread_id.xy] = float4(hit_pixel, hit_depth * hit_mul, H.w);
}
