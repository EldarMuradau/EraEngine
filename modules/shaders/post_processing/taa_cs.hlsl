#include "post_processing_rs.hlsli"
#include "cs.hlsli"
#include "camera.hlsli"
#include "math.hlsli"

ConstantBuffer<TAACB> cb			: register(b0);

RWTexture2D<float4> output			: register(u0);
Texture2D<float4> current_frame		: register(t0);
Texture2D<float4> prev_frame			: register(t1);
Texture2D<float2> motion			: register(t2);
Texture2D<float> depth_buffer		: register(t3);

SamplerState linear_sampler			: register(s0);

#define TILE_BORDER 1
#define TILE_SIZE (POST_PROCESSING_BLOCK_SIZE + TILE_BORDER * 2)

#define MAX_HISTORY_LENGTH 32.f // In frames, stored in the alpha channel of the history.
#define VARIANCE_CLIP_GAMMA 1.25f

groupshared uint tile_red_green[TILE_SIZE * TILE_SIZE];
groupshared float tile_blue[TILE_SIZE * TILE_SIZE];
groupshared float tile_depth[TILE_SIZE * TILE_SIZE]; // Full precision: in half precision nearby depths become equal and pick a wrong velocity.


static float3 tonemap(float3 x)
{
	return x / (x + 1.f); // Reinhard tonemap
}

static float3 inverse_tonemap(float3 x)
{
	return x / (1.f - x);
}

// 5-tap Catmull-Rom (corner taps dropped). Bilinear history resampling blurs a little every frame, which smears fast-moving detail.
static float3 sample_catmull_rom(Texture2D<float4> tex, float2 uv, float2 dimensions)
{
	const float2 sample_pos = uv * dimensions;
	const float2 tex_pos1 = floor(sample_pos - 0.5f) + 0.5f;
	const float2 f = sample_pos - tex_pos1;

	const float2 w0 = f * (-0.5f + f * (1.f - 0.5f * f));
	const float2 w1 = 1.f + f * f * (-2.5f + 1.5f * f);
	const float2 w2 = f * (0.5f + f * (2.f - 1.5f * f));
	const float2 w3 = f * f * (-0.5f + 0.5f * f);

	const float2 w12 = w1 + w2;
	const float2 uv0 = (tex_pos1 - 1.f) / dimensions;
	const float2 uv3 = (tex_pos1 + 2.f) / dimensions;
	const float2 uv12 = (tex_pos1 + w2 / w12) / dimensions;

	const float wa = w12.x * w0.y;
	const float wb = w0.x * w12.y;
	const float wc = w12.x * w12.y;
	const float wd = w3.x * w12.y;
	const float we = w12.x * w3.y;

	float3 result = tex.SampleLevel(linear_sampler, float2(uv12.x, uv0.y), 0).rgb * wa;
	result += tex.SampleLevel(linear_sampler, float2(uv0.x, uv12.y), 0).rgb * wb;
	result += tex.SampleLevel(linear_sampler, uv12, 0).rgb * wc;
	result += tex.SampleLevel(linear_sampler, float2(uv3.x, uv12.y), 0).rgb * wd;
	result += tex.SampleLevel(linear_sampler, float2(uv12.x, uv3.y), 0).rgb * we;
	return max(result / (wa + wb + wc + wd + we), 0.f);
}

// Moves q toward the box center until it is inside the box.
static float3 clip_aabb(float3 box_min, float3 box_max, float3 q)
{
	const float3 center = 0.5f * (box_max + box_min);
	const float3 extents = 0.5f * (box_max - box_min) + 1e-5f;
	const float3 v = q - center;
	const float3 a = abs(v / extents);
	const float ma = max(a.x, max(a.y, a.z));
	return (ma > 1.f) ? center + v / ma : q;
}

[numthreads(POST_PROCESSING_BLOCK_SIZE, POST_PROCESSING_BLOCK_SIZE, 1)]
[RootSignature(TAA_RS)]
void main(CSInput IN)
{
	int2 tex_coord = IN.dispatch_thread_id.xy;
	float2 inv_dimensions = 1.f / cb.dimensions;
	float2 uv = (float2(tex_coord) + float2(0.5f, 0.5f)) * inv_dimensions;

	const int2 upper_left = IN.group_id.xy * POST_PROCESSING_BLOCK_SIZE - TILE_BORDER;

	for (uint t = IN.group_index; t < TILE_SIZE * TILE_SIZE; t += POST_PROCESSING_BLOCK_SIZE * POST_PROCESSING_BLOCK_SIZE)
	{
		const uint2 pixel = upper_left + unflatten_2d(t, TILE_SIZE);
		const float depth = depth_buffer[pixel];
		const float3 color = current_frame[pixel].rgb;
		tile_red_green[t] = f32tof16(color.r) | (f32tof16(color.g) << 16);
		tile_blue[t] = color.b;
		tile_depth[t] = depth;
	}
	GroupMemoryBarrierWithGroupSync();

	// All color math below is in tonemapped space.
	float3 neighborhood_min = 100000;
	float3 neighborhood_max = -100000;
	float3 moment1 = 0.f;
	float3 moment2 = 0.f;
	float3 current;
	float3 filtered = 0.f; // 3x3 tent, used where there is no history.

	// Search for best velocity and compute color clamping range in 3x3 neighborhood. Ties keep the center.
	int2 best_offset = 0;
	float best_depth = tile_depth[flatten_2d(IN.group_thread_id.xy + TILE_BORDER, TILE_SIZE)];
	for (int x = -1; x <= 1; ++x)
	{
		for (int y = -1; y <= 1; ++y)
		{
			const int2 offset = int2(x, y);
			const uint idx = flatten_2d(IN.group_thread_id.xy + TILE_BORDER + offset, TILE_SIZE);
			const uint red_green = tile_red_green[idx];

			const float3 neighbor = tonemap(float3(f16tof32(red_green), f16tof32(red_green >> 16), tile_blue[idx]));
			neighborhood_min = min(neighborhood_min, neighbor);
			neighborhood_max = max(neighborhood_max, neighbor);
			moment1 += neighbor;
			moment2 += neighbor * neighbor;
			if (x == 0 && y == 0)
			{
				current = neighbor;
			}
			filtered += neighbor * ((x == 0 ? 2.f : 1.f) * (y == 0 ? 2.f : 1.f) / 16.f);

			const float depth = tile_depth[idx];
			if (depth < best_depth)
			{
				best_depth = depth;
				best_offset = offset;
			}
		}
	}

	const float2 m = motion[IN.dispatch_thread_id.xy + best_offset].xy;
	const float2 prev_uv = uv + m;

	// Variance clipping: a much tighter box than min/max on textured surfaces, so smeared or ghosted history is rejected.
	const float3 mean = moment1 / 9.f;
	const float3 sigma = sqrt(max(moment2 / 9.f - mean * mean, 0.f));
	const float3 box_min = max(neighborhood_min, mean - VARIANCE_CLIP_GAMMA * sigma);
	const float3 box_max = min(neighborhood_max, mean + VARIANCE_CLIP_GAMMA * sigma);

	const float3 prev = clip_aabb(box_min, box_max, tonemap(sample_catmull_rom(prev_frame, prev_uv, cb.dimensions)));

	float subpixel_correction = frac(
		max(
			abs(m.x) * cb.dimensions.x,
			abs(m.y) * cb.dimensions.y
		)
	) * 0.5f;

	float blendfactor = saturate(lerp(0.05f, 0.8f, subpixel_correction));

	// Fast motion: the clamped history is less reliable, trust the current frame more.
	const float motion_pixels = length(m * cb.dimensions);
	blendfactor = max(blendfactor, lerp(0.05f, 0.25f, saturate(motion_pixels / 32.f)));

	const float history_length = is_saturated(prev_uv) ? prev_frame.SampleLevel(linear_sampler, prev_uv, 0).a : 0.f;

	// Young history is a running average, otherwise the first (aliased, jittered) samples linger for many frames.
	blendfactor = max(blendfactor, 1.f / (history_length + 1.f));

	if (history_length == 0.f)
	{
		current = filtered;
	}

	const float3 resolved = inverse_tonemap(lerp(prev, current, blendfactor));

	output[tex_coord] = float4(resolved, min(history_length + 1.f, MAX_HISTORY_LENGTH));
}
