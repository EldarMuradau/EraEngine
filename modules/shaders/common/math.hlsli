#ifndef MATH_HLSLI
#define MATH_HLSLI

#define M_PI 3.14159265359f
#define M_PI_OVER_2 (M_PI * 0.5f)
#define M_PI_OVER_180 (M_PI / 180.f)
#define M_180_OVER_PI (180.f / M_PI)
#define M_SQRT_PI 1.77245385090f
#define M_TAU 6.28318530718f

#define M_INV_PI 0.3183098861837906f
#define M_INV_2_PI 0.1591549430918953f
#define M_INV_ATAN float2(M_INV_2_PI, M_INV_PI)

static float inverse_lerp(float l, float u, float v) { return (v - l) / (u - l); }
static float remap(float v, float old_l, float old_u, float new_l, float new_u) { return lerp(new_l, new_u, inverse_lerp(old_l, old_u, v)); }

static float solid_angle_of_sphere(float radius, float distance)
{
	// The angular radius of a sphere is p = arcsin(radius / d). 
	// The solid angle of a circular cap (projection of sphere) is 2pi * (1 - cos(p)).
	// cos(arcsin(x)) = sqrt(1 - x*x)

	float s = radius / distance;
	return 2.f * M_PI * (1.f - sqrt(max(0.f, 1.f - s * s)));
}

static float square(float v) { return v * v; }
static float2 square(float2 v) { return v * v; }
static float3 square(float3 v) { return v * v; }
static float4 square(float4 v) { return v * v; }
static float pow3(float v) { return v * v * v; }

static float max_component(float3 a) { return max(a.x, max(a.y, a.z)); }

static uint flatten_2d(uint2 coord, uint2 dim)
{
	return coord.x + coord.y * dim.x;
}

static uint flatten_2d(uint2 coord, uint width)
{
	return coord.x + coord.y * width;
}

// Flattened array index to 2D array index.
static uint2 unflatten_2d(uint idx, uint2 dim)
{
	return uint2(idx % dim.x, idx / dim.x);
}

static uint2 unflatten_2d(uint idx, uint width)
{
	return uint2(idx % width, idx / width);
}

inline bool is_saturated(float a) { return a == saturate(a); }
inline bool is_saturated(float2 a) { return is_saturated(a.x) && is_saturated(a.y); }
inline bool is_saturated(float3 a) { return is_saturated(a.x) && is_saturated(a.y) && is_saturated(a.z); }
inline bool is_saturated(float4 a) { return is_saturated(a.x) && is_saturated(a.y) && is_saturated(a.z) && is_saturated(a.w); }

static float2 screen_space_velocity(float3 ndc, float3 prev_frame_ndc, float2 jitter, float2 prev_frame_jitter)
{
	float2 curr_ndc = (ndc.xy / ndc.z) - jitter;
	float2 prev_ndc = (prev_frame_ndc.xy / prev_frame_ndc.z) - prev_frame_jitter;

	float2 motion = (prev_ndc - curr_ndc) * float2(0.5f, -0.5f);
	return motion;
}

static float2 quad_from_vertex_id(uint vertex_id)
{
	return float2((vertex_id << 1) & 2, vertex_id & 2) - 1.f;
}

#define pack_float	4
#define pack_vec4	1
#define pack_float4	1

#define pack(num, data_type) num / pack_##data_type

// maxNumPoints must be a multiple of 4!
// data_type can currently be either float or vec4.
// The values are internally stored packed. This is because HLSL packs arrays in constant buffers
// in float4s. See https://docs.microsoft.com/en-us/windows/win32/direct3dhlsl/dx-graphics-hlsl-packing-rules.
// In order to conform to C++'s packing rules, we declare an array of float4s and cast it to an array of floats.
#define spline(data_type, max_num_points) catmull_rom_spline_##data_type##_##max_num_points

#define defineSpline(data_type, max_num_points)															\
struct spline(data_type, max_num_points)																	\
{																										\
	float4 packed_ts[max_num_points / 4];																	\
	float4 packed_values[pack(max_num_points, data_type)];													\
																										\
	inline data_type evaluate(int num_actual_points, float t)												\
	{																									\
		float ts[max_num_points] = (float[max_num_points])packed_ts;											\
		data_type values[max_num_points] = (data_type[max_num_points])packed_values;							\
																										\
		int k = 0;																						\
		[unroll(max_num_points)]																			\
		while (ts[k] < t)																				\
		{																								\
			++k;																						\
		}																								\
																										\
		const float h1 = inverse_lerp(ts[k - 1], ts[k], t);												\
		const float h2 = h1 * h1;																		\
		const float h3 = h2 * h1;																		\
		const float4 h = float4(h3, h2, h1, 1.f);														\
																										\
		data_type result = (data_type)0;																\
																										\
		int m = num_actual_points - 1;																	\
		result += values[clamp(k - 2, 0, m)] * dot(float4(-1, 2, -1, 0), h);							\
		result += values[k - 1] * dot(float4(3, -5, 0, 2), h);											\
		result += values[k] * dot(float4(-3, 4, 1, 0), h);												\
		result += values[clamp(k + 1, 0, m)] * dot(float4(1, -1, 0, 0), h);								\
																										\
		result *= 0.5f;																					\
																										\
		return result;																					\
	}																									\
};

static inline uint pack_halfs(float left, float right)
{
	return (f32tof16(left) << 16) | f32tof16(right);
}

static inline float unpack_halfs_left(uint x)
{
	return f16tof32(x >> 16);
}

static inline float unpack_halfs_right(uint x)
{
	return f16tof32(x);
}

static inline void unpack_halfs(uint x, out float left, out float right)
{
	left = unpack_halfs_left(x);
	right = unpack_halfs_right(x);
}

static inline float cross2(float2 a, float2 b)
{ 
	return a.x * b.y - a.y * b.x; 
}

//	https://www.lgdv.tf.fau.de/publications/spherical-fibonacci-mapping/
static float3 spherical_fibonacci(float i, float n)
{
	const float PHI = sqrt(5.f) * 0.5f + 0.5f;
#   define madfrac(a, b) ((a)*(b)-floor((a)*(b)))
	float phi = 2.f * M_PI * madfrac(i, PHI - 1);
	float cos_theta = 1.f - (2.f * i + 1.f) * (1.f / n);
	float sin_theta = sqrt(saturate(1.f - cos_theta * cos_theta));

	float sin_phi, cos_phi;
	sincos(phi, sin_phi, cos_phi);

	return float3(
		cos_phi * sin_theta,
		sin_phi * sin_theta,
		cos_theta);

#   undef madfrac
}
#endif
