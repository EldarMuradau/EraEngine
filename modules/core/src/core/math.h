// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#pragma once

#define ROW_MAJOR 0
#define DIRECTX_COORDINATE_SYSTEM 1

#include "core_api.h"

#include "core/simd.h"

#include <rttr/type>

#include <cmath>
#include <cfloat>

#define M_PI 3.14159265359f
#define M_PI_OVER_2 (M_PI * 0.5f)
#define M_PI_OVER_180 (M_PI / 180.f)
#define M_180_OVER_PI (180.f / M_PI)
#define SQRT_PI	1.77245385090f
#define INV_PI 0.31830988618379f
#define M_TAU 6.28318530718f
#define INV_TAU 0.159154943091895335f

#define EPSILON 1e-6f

inline constexpr float deg2rad(float  deg) 
{ 
	return deg * (M_PI_OVER_180);
}

inline constexpr float rad2deg(float rad) 
{
	return rad * (M_180_OVER_PI);
}

constexpr float oneDiv6 = 1.0f / 6.0f;
constexpr float oneDiv24 = 1.0f / 24.0f;
constexpr float oneDiv60 = 1.0f / 60.0f;
constexpr float oneDiv120 = 1.0f / 120.0f;

inline constexpr float lerp(float l, float u, float t) { return l + t * (u - l); }
inline constexpr float inverse_lerp(float l, float u, float v) { return (v - l) / (u - l); }
inline constexpr float remap(float v, float oldL, float oldU, float newL, float newU) { return lerp(newL, newU, inverse_lerp(oldL, oldU, v)); }
inline constexpr float clamp(float v, float l, float u) { float r = max(l, v); r = min(u, r); return r; }
inline constexpr uint32 clamp(uint32 v, uint32 l, uint32 u) { uint32 r = max(l, v); r = min(u, r); return r; }
inline constexpr int32 clamp(int32 v, int32 l, int32 u) { int32 r = max(l, v); r = min(u, r); return r; }
inline constexpr float clamp01(float v) { return clamp(v, 0.f, 1.f); }
inline constexpr float saturate(float v) { return clamp01(v); }
inline constexpr float smoothstep(float t) { return t * t * (3.f - 2.f * t); }
inline constexpr float smoothstep(float l, float u, float v) { return smoothstep(clamp01(inverse_lerp(l, u, v))); }
inline constexpr uint32 bucketize(uint32 problemSize, uint32 bucketSize) { return (problemSize + bucketSize - 1) / bucketSize; }
inline constexpr uint64 bucketize(uint64 problemSize, uint64 bucketSize) { return (problemSize + bucketSize - 1) / bucketSize; }

inline float frac(float v) { return fmodf(v, 1.f); }
inline void copy_sign(float from, float& to) { to = copysign(to, from); }

inline void flush_denormals_to_zero() { _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON); }

// Constexpr-version of _BitScanForward. Returns -1 if mask is zero.
inline constexpr uint32 index_of_least_significant_set_bit(uint32 i)
{
	if (i == 0)
	{
		return (uint32)-1;
	}

	uint32 count = 0;

	while ((i & 1) == 0)
	{
		i >>= 1;
		++count;
	}
	return count;
}

// Constexpr-version of _BitScanReverse. Returns -1 if mask is zero.
inline constexpr uint32 index_of_most_significant_set_bit(uint32 i)
{
	if (i == 0)
	{
		return (uint32)-1;
	}

	uint32 count = 0;

	while (i != 1)
	{
		i >>= 1;
		++count;
	}
	return count;
}

inline constexpr int32 log2(int32 i)
{
	ASSERT(i >= 0);

	uint32 mssb = index_of_most_significant_set_bit((uint32)i);
	uint32 lssb = index_of_least_significant_set_bit((uint32)i);

	if (mssb == (uint32)-1 || lssb == (uint32)-1)
	{
		return 0;
	}

	// If perfect power of two (only one set bit), return index of bit.  Otherwise round up
	// fractional log by adding 1 to most signicant set bit's index.
	return (int32)mssb + (mssb == lssb ? 0 : 1);
}

inline constexpr uint32 log2(uint32 i)
{
	uint32 mssb = index_of_most_significant_set_bit(i);
	uint32 lssb = index_of_least_significant_set_bit(i);

	if (mssb == (uint32)-1 || lssb == (uint32)-1)
	{
		return 0;
	}

	// If perfect power of two (only one set bit), return index of bit.  Otherwise round up
	// fractional log by adding 1 to most signicant set bit's index.
	return mssb + (mssb == lssb ? 0 : 1);
}

inline constexpr bool is_power_of_two(uint32 i)
{
	return (i & (i - 1)) == 0;
}

inline constexpr uint32 align_to_power_of_two(uint32 i)
{
	return i == 0u ? 0u : 1u << log2(i);
}

inline float ease_in_quadratic(float t) { return t * t; }
inline float ease_out_quadratic(float t) { return t * (2.f - t); }
inline float ease_in_out_quadratic(float t) { return (t < 0.5f) ? (2.f * t * t) : (-1.f + (4.f - 2.f * t) * t); }

inline float ease_in_cubic(float t) { return t * t * t; }
inline float ease_out_cubic(float t) { float tmin1 = t - 1.f; return tmin1 * tmin1 * tmin1 + 1.f; }
inline float ease_in_out_cubic(float t) { return (t < 0.5f) ? (4.f * t * t * t) : ((t - 1.f) * (2.f * t - 2.f) * (2.f * t - 2.f) + 1.f); }

inline float ease_in_quartic(float t) { return t * t * t * t; }
inline float ease_out_quartic(float t) { float tmin1 = t - 1.f; return 1.f - tmin1 * tmin1 * tmin1 * tmin1; }
inline float ease_in_out_quartic(float t) { float tmin1 = t - 1.f; return (t < 0.5f) ? (8.f * t * t * t * t) : (1.f - 8.f * tmin1 * tmin1 * tmin1 * tmin1); }

inline float ease_in_quintic(float t) { return t * t * t * t * t; }
inline float ease_out_quintic(float t) { float tmin1 = t - 1.f; return 1.f + tmin1 * tmin1 * tmin1 * tmin1 * tmin1; }
inline float ease_in_out_quintic(float t) { float tmin1 = t - 1.f; return t < 0.5 ? 16.f * t * t * t * t * t : 1.f + 16.f * tmin1 * tmin1 * tmin1 * tmin1 * tmin1; }

inline float ease_in_sine(float t) { return sin((t - 1.f) * M_PI_OVER_2) + 1.f; }
inline float ease_out_sine(float t) { return sin(t * M_PI_OVER_2); }
inline float ease_in_out_sine(float t) { return 0.5f * (1 - cos(t * M_PI)); }

inline float ease_in_circular(float t) { return 1.f - sqrt(1.f - (t * t)); }
inline float ease_out_circular(float t) { return sqrt((2.f - t) * t); }
inline float ease_in_out_circular(float t) { return (t < 0.5f) ? (0.5f * (1.f - sqrt(1.f - 4.f * (t * t)))) : (0.5f * (sqrt(-((2.f * t) - 3.f) * ((2.f * t) - 1.f)) + 1.f)); }

inline float ease_in_exponential(float t) { return (t == 0.f) ? t : powf(2.f, 10.f * (t - 1.f)); }
inline float ease_out_exponential(float t) { return (t == 1.f) ? t : 1.f - powf(2.f, -10.f * t); }
inline float ease_in_out_exponential(float t) { if (t == 0.f || t == 1.f) { return t; } return (t < 0.5f) ? (0.5f * powf(2.f, (20.f * t) - 10.f)) : (-0.5f * powf(2.f, (-20.f * t) + 10.f) + 1.f); }

inline float in_elastic(float t) { return sin(13.f * M_PI_OVER_2 * t) * powf(2.f, 10.f * (t - 1.f)); }
inline float out_elastic(float t) { return sin(-13.f * M_PI_OVER_2 * (t + 1.f)) * powf(2.f, -10.f * t) + 1.f; }
inline float in_out_elastic(float t) { return (t < 0.5f) ? (0.5f * sin(13.f * M_PI_OVER_2 * (2.f * t)) * powf(2.f, 10.f * ((2.f * t) - 1.f))) : (0.5f * (sin(-13.f * M_PI_OVER_2 * ((2.f * t - 1.f) + 1.f)) * powf(2.f, -10.f * (2.f * t - 1.f)) + 2.f)); }

inline float in_back(float t) { const float s = 1.70158f; return t * t * ((s + 1.f) * t - s); }
inline float out_back(float t) { const float s = 1.70158f; return --t, 1.f * (t * t * ((s + 1.f) * t + s) + 1.f); }
inline float in_out_back(float t) { const float s = 1.70158f * 1.525f; return (t < 0.5f) ? (t *= 2.f, 0.5f * t * t * (t * s + t - s)) : (t = t * 2.f - 2.f, 0.5f * (2.f + t * t * (t * s + t + s))); }

#define bounceout(p) ( \
    (t) < 4.f/11.f ? (121.f * (t) * (t))/16.f : \
    (t) < 8.f/11.f ? (363.f/40.f * (t) * (t)) - (99.f/10.f * (t)) + 17.f/5.f : \
    (t) < 9.f/10.f ? (4356.f/361.f * (t) * (t)) - (35442.f/1805.f * (t)) + 16061.f/1805.f \
                : (54.f/5.f * (t) * (t)) - (513.f/25.f * (t)) + 268.f/25.f )

inline float in_bounce(float t) { return 1.f - bounceout(1.f - t); }
inline float out_bounce(float t) { return bounceout(t); }
inline float in_out_bounce(float t) { return (t < 0.5f) ? (0.5f * (1.f - bounceout(1.f - t * 2.f))) : (0.5f * bounceout((t * 2.f - 1.f)) + 0.5f); }

#undef bounceout

inline float get_framerate_independent_t(float speed, float dt)
{
	return 1.f - expf(-speed * dt);
}

struct ERA_CORE_API half
{
	uint16 h;

	half() {}
	half(float f);
	half(uint16 i);

	operator float();

	static const half min_value;
	static const half max_value;
};

half operator+(half a, half b);
half& operator+=(half& a, half b);

half operator-(half a, half b);
half& operator-=(half& a, half b);

half operator*(half a, half b);
half& operator*=(half& a, half b);

half operator/(half a, half b);
half& operator/=(half& a, half b);

struct ERA_CORE_API vec2
{
	union
	{
		struct
		{
			float x, y;
		};
		float data[2];
	};

	vec2() {}
	vec2(float v) : vec2(v, v) {}
	vec2(float x, float y) : x(x), y(y) {}

	static const vec2 zero;
};

struct ERA_CORE_API vec3
{
	union
	{
		struct
		{
			float x, y, z;
		};
		struct
		{
			float r, g, b;
		};
		struct
		{
			vec2 xy;
			float z;
		};
		struct
		{
			float x;
			vec2 yz;
		};
		float data[3];
	};

	vec3() {}
	vec3(float v) : vec3(v, v, v) {}
	vec3(float x, float y, float z) : x(x), y(y), z(z) {}
	vec3(vec2 xy, float z) : x(xy.x), y(xy.y), z(z) {}

	static const vec3 zero;
	static const vec3 right;
	static const vec3 up;
	static const vec3 forward;

};

struct ERA_CORE_API vec4
{
	union
	{
		struct
		{
			float x, y, z, w;
		};
		struct
		{
			float r, g, b, a;
		};
		struct
		{
			vec3 xyz;
			float w;
		};
		struct
		{
			vec2 xy;
			vec2 zw;
		};
		struct
		{
			float x;
			vec3 yzw;
		};
		w4_float f4;
		float data[4];
	};

	vec4() {}
	vec4(float v) : vec4(v, v, v, v) {}
	vec4(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}
	vec4(vec3 xyz, float w) : x(xyz.x), y(xyz.y), z(xyz.z), w(w) {}
	vec4(w4_float f4) : f4(f4) {}

	static const vec4 zero;
};

struct ERA_CORE_API quat
{
	union
	{
		struct
		{
			float x, y, z, w;
		};
		struct
		{
			vec3 v;
			float cosHalfAngle;
		};
		vec4 v4;
		w4_float f4;
	};

	quat() {}
	quat(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}
	quat(vec3 axis, float angle);
	quat(vec4 v4) : v4(v4) {}
	quat(w4_float f4) : f4(f4) {}

	static const quat identity;
	static const quat zero;
};

struct ERA_CORE_API dual_quat
{
	quat real;
	quat dual;

	dual_quat() {}
	dual_quat(quat real, quat dual) : real(real), dual(dual) {}
	dual_quat(quat rotation, vec3 translation);

	vec3 get_translation() const;
	quat get_rotation() const;

	static const dual_quat identity;
	static const dual_quat zero;
};

struct ERA_CORE_API mat2
{
	union
	{
#if ROW_MAJOR
		struct
		{
			float
				m00, m01,
				m10, m11;
		};
		struct
		{
			vec2 row0;
			vec2 row1;
};
		vec2 rows[2];
#else
		struct
		{
			float
				m00, m10,
				m01, m11;
		};
		struct
		{
			vec2 col0;
			vec2 col1;
		};
		vec2 cols[2];
#endif
		float m[4];
	};

	mat2() {}
	mat2(
		float m00, float m01,
		float m10, float m11);

	static const mat2 identity;
	static const mat2 zero;
};

struct ERA_CORE_API mat3
{
	union
	{
#if ROW_MAJOR
		struct
		{
			float
				m00, m01, m02,
				m10, m11, m12,
				m20, m21, m22;
		};
		struct
		{
			vec3 row0;
			vec3 row1;
			vec3 row2;
	};
		vec3 rows[3];
#else
		struct
		{
			float
				m00, m10, m20,
				m01, m11, m21,
				m02, m12, m22;
		};
		struct
		{
			vec3 col0;
			vec3 col1;
			vec3 col2;
		};
		vec3 cols[3];
#endif
		float m[9];
	};

	mat3() {}
	mat3(
		float m00, float m01, float m02,
		float m10, float m11, float m12,
		float m20, float m21, float m22);

	static const mat3 identity;
	static const mat3 zero;
};

struct ERA_CORE_API mat4
{
	union
	{
#if ROW_MAJOR
		struct
		{
			float
				m00, m01, m02, m03,
				m10, m11, m12, m13,
				m20, m21, m22, m23,
				m30, m31, m32, m33;
		};
		struct
		{
			vec4 row0;
			vec4 row1;
			vec4 row2;
			vec4 row3;
		};
		vec4 rows[4];
#else
		struct
		{
			float
				m00, m10, m20, m30,
				m01, m11, m21, m31,
				m02, m12, m22, m32,
				m03, m13, m23, m33;
		};
		struct
		{
			vec4 col0;
			vec4 col1;
			vec4 col2;
			vec4 col3;
		};
		vec4 cols[4];
#endif
		struct
		{
			w4_float f40;
			w4_float f41;
			w4_float f42;
			w4_float f43;
		};
		float m[16];
	};

	mat4() {}
	mat4(
		float m00, float m01, float m02, float m03,
		float m10, float m11, float m12, float m13,
		float m20, float m21, float m22, float m23,
		float m30, float m31, float m32, float m33);

	static const mat4 identity;
	static const mat4 zero;
};

static_assert(sizeof(mat4) == 16 * sizeof(float), "");

#if ROW_MAJOR
inline vec2 row(const mat2& a, uint32 r) { return a.rows[r]; }
inline vec3 row(const mat3& a, uint32 r) { return a.rows[r]; }
inline vec4 row(const mat4& a, uint32 r) { return a.rows[r]; }

inline vec2 col(const mat2& a, uint32 c) { return { a.m[c], a.m[c + 2] }; }
inline vec3 col(const mat3& a, uint32 c) { return { a.m[c], a.m[c + 3], a.m[c + 6] }; }
inline vec4 col(const mat4& a, uint32 c) { return { a.m[c], a.m[c + 4], a.m[c + 8], a.m[c + 12] }; }
#else
inline vec2 row(const mat2& a, uint32 r) { return { a.m[r], a.m[r + 2] }; }
inline vec3 row(const mat3& a, uint32 r) { return { a.m[r], a.m[r + 3], a.m[r + 6] }; }
inline vec4 row(const mat4& a, uint32 r) { return { a.m[r], a.m[r + 4], a.m[r + 8], a.m[r + 12] }; }

inline vec2 col(const mat2& a, uint32 c) { return a.cols[c]; }
inline vec3 col(const mat3& a, uint32 c) { return a.cols[c]; }
inline vec4 col(const mat4& a, uint32 c) { return a.cols[c]; }
#endif

struct ERA_CORE_API trs
{
	quat rotation;
	vec3 position;
	vec3 scale;

	trs() {}
	trs(vec3 position, quat rotation = quat::identity, vec3 scale = { 1.f, 1.f, 1.f }) : position(position), rotation(rotation), scale(scale) {}

	static const trs identity;

	RTTR_ENABLE()
};

struct ERA_CORE_API ray
{
	vec3 origin;
	vec3 direction;

	RTTR_ENABLE()
};

// Vec2 operators
inline vec2 operator+(const vec2& a, const vec2& b) { vec2 result = { a.x + b.x, a.y + b.y }; return result; }
inline vec2& operator+=(vec2& a, const vec2& b) { a = a + b; return a; }
inline vec2 operator-(vec2 a, const vec2& b) { vec2 result = { a.x - b.x, a.y - b.y }; return result; }
inline vec2& operator-=(vec2& a, const vec2& b) { a = a - b; return a; }
inline vec2 operator*(vec2 a, const vec2& b) { vec2 result = { a.x * b.x, a.y * b.y }; return result; }
inline vec2& operator*=(vec2& a, const vec2& b) { a = a * b; return a; }
inline vec2 operator/(vec2 a, const vec2& b) { vec2 result = { a.x / b.x, a.y / b.y }; return result; }
inline vec2& operator/=(vec2& a, const vec2& b) { a = a / b; return a; }

inline vec2 operator*(const vec2& a, float b) { vec2 result = { a.x * b, a.y * b }; return result; }
inline vec2 operator*(float a, const vec2& b) { return b * a; }
inline vec2& operator*=(vec2& a, float b) { a = a * b; return a; }
inline vec2 operator/(vec2 a, float b) { vec2 result = { a.x / b, a.y / b }; return result; }
inline vec2& operator/=(vec2& a, float b) { a = a / b; return a; }

inline vec2 operator-(const vec2& a) { return vec2(-a.x, -a.y); }

inline bool operator==(const vec2& a, const vec2& b) { return a.x == b.x && a.y == b.y; }
inline bool operator!=(const vec2& a, const vec2& b) { return !(a == b); }

// Vec3 operators
inline vec3 operator+(const vec3& a, const vec3& b) { vec3 result = { a.x + b.x, a.y + b.y, a.z + b.z }; return result; }
inline vec3& operator+=(vec3& a, const vec3& b) { a = a + b; return a; }
inline vec3 operator-(const vec3& a, const vec3& b) { vec3 result = { a.x - b.x, a.y - b.y, a.z - b.z }; return result; }
inline vec3& operator-=(vec3& a, const vec3& b) { a = a - b; return a; }
inline vec3 operator*(const vec3& a, const vec3& b) { vec3 result = { a.x * b.x, a.y * b.y, a.z * b.z }; return result; }
inline vec3& operator*=(vec3& a, const vec3& b) { a = a * b; return a; }
inline vec3 operator/(const vec3& a, const vec3& b) { vec3 result = { a.x / b.x, a.y / b.y, a.z / b.z }; return result; }
inline vec3& operator/=(vec3& a, const vec3& b) { a = a / b; return a; }

inline vec3 operator*(const vec3& a, float b) { vec3 result = { a.x * b, a.y * b, a.z * b }; return result; }
inline vec3 operator*(float a, const vec3& b) { return b * a; }
inline vec3& operator*=(vec3& a, float b) { a = a * b; return a; }
inline vec3 operator/(vec3 a, float b) { vec3 result = { a.x / b, a.y / b, a.z / b }; return result; }
inline vec3& operator/=(vec3& a, float b) { a = a / b; return a; }

inline vec3 operator-(const vec3& a) { return vec3(-a.x, -a.y, -a.z); }

inline bool operator==(const vec3& a, const vec3& b) { return a.x == b.x && a.y == b.y && a.z == b.z; }
inline bool operator!=(const vec3& a, const vec3& b) { return !(a == b); }

// Vec4 operators
inline vec4 operator+(const vec4& a, const vec4& b) { vec4 result = { a.f4 + b.f4 }; return result; }
inline vec4& operator+=(vec4& a, const vec4& b) { a = a + b; return a; }
inline vec4 operator-(const vec4& a, const vec4& b) { vec4 result = { a.f4 - b.f4 }; return result; }
inline vec4& operator-=(vec4& a, const vec4& b) { a = a - b; return a; }
inline vec4 operator*(const vec4& a, const vec4& b) { vec4 result = { a.f4 * b.f4 }; return result; }
inline vec4& operator*=(vec4& a, const vec4& b) { a = a * b; return a; }
inline vec4 operator/(const vec4& a, const vec4& b) { vec4 result = { a.f4 / b.f4 }; return result; }
inline vec4& operator/=(vec4& a, const vec4& b) { a = a / b; return a; }

inline vec4 operator*(const vec4& a, float b) { vec4 result = { a.f4 * w4_float(b) }; return result; }
inline vec4 operator*(float a, const vec4& b) { return b * a; }
inline vec4& operator*=(vec4& a, float b) { a = a * b; return a; }
inline vec4 operator/(const vec4& a, float b) { vec4 result = { a.f4 / w4_float(b) }; return result; }
inline vec4& operator/=(vec4& a, float b) { a = a / b; return a; }

inline vec4 operator-(const vec4& a) { return vec4(-a.f4); }

inline quat operator-(const quat& a) { return quat(-a.f4); }

inline bool operator==(const vec4& a, const vec4& b) { return a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w; }
inline bool operator!=(const vec4& a, const vec4& b) { return !(a == b); }

inline vec2 diagonal(const mat2& m) { return vec2(m.m00, m.m11); }
inline vec3 diagonal(const mat3& m) { return vec3(m.m00, m.m11, m.m22); }
inline vec4 diagonal(const mat4& m) { return vec4(m.m00, m.m11, m.m22, m.m33); }

inline float dot(const vec2& a, const vec2& b) { float result = a.x * b.x + a.y * b.y; return result; }
inline float dot(const vec3& a, const vec3& b) { float result = a.x * b.x + a.y * b.y + a.z * b.z; return result; }
inline float dot(const vec4& a, const vec4& b) { w4_float m = a.f4 * b.f4; return add_elements(m); }
inline float dot(const quat& a, const quat& b) { w4_float m = a.f4 * b.f4; return add_elements(m); }

inline float cross(const vec2& a, const vec2& b) { return a.x * b.y - a.y * b.x; }
inline vec3 cross(const vec3& a, const vec3& b) { vec3 result = { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x }; return result; }

inline float squared_length(const vec2& a) { return dot(a, a); }
inline float squared_length(const vec3& a) { return dot(a, a); }
inline float squared_length(const vec4& a) { return dot(a, a); }

inline float length(const vec2& a) { return sqrt(squared_length(a)); }
inline float length(const vec3& a) { return sqrt(squared_length(a)); }
inline float length(const vec4& a) { return sqrt(squared_length(a)); }

inline float squared_distance(const vec2& a, const vec2& b) { vec2 delta = a - b; return dot(delta, delta); }
inline float squared_distance(const vec3& a, const vec3& b) { vec3 delta = a - b; return dot(delta, delta); }
inline float squared_distance(const vec4& a, const vec4& b) { vec4 delta = a - b; return dot(delta, delta); }

inline float distance(const vec2& a, const vec2& b) { return sqrt(squared_distance(a, b)); }
inline float distance(const vec3& a, const vec3& b) { return sqrt(squared_distance(a, b)); }
inline float distance(const vec4& a, const vec4& b) { return sqrt(squared_distance(a, b)); }

inline vec2 noz(const vec2& a) { float sl = squared_length(a); return (sl < 1e-8f) ? vec2(0.f, 0.f) : (a * (1.f / sqrt(sl))); }
inline vec3 noz(const vec3& a) { float sl = squared_length(a); return (sl < 1e-8f) ? vec3(0.f, 0.f, 0.f) : (a * (1.f / sqrt(sl))); }
inline vec4 noz(const vec4& a) { float sl = squared_length(a); return (sl < 1e-8f) ? vec4(0.f, 0.f, 0.f, 0.f) : (a * (1.f / sqrt(sl))); }

inline vec2 normalize(const vec2& a) { float l = length(a); return a * (1.f / l); }
inline vec3 normalize(const vec3& a) { float l = length(a); return a * (1.f / l); }
inline vec4 normalize(const vec4& a) { float l = length(a); return a * (1.f / l); }

inline void copy_sign(const vec2& from, vec2& to) { copy_sign(from.x, to.x); copy_sign(from.y, to.y); }
inline void copy_sign(const vec3& from, vec3& to) { copy_sign(from.x, to.x); copy_sign(from.y, to.y); copy_sign(from.z, to.z); }
inline void copy_sign(const vec4& from, vec4& to) { copy_sign(from.x, to.x); copy_sign(from.y, to.y); copy_sign(from.z, to.z); copy_sign(from.w, to.w); }

inline vec2 abs(const vec2& a) { return vec2(abs(a.x), abs(a.y)); }
inline vec3 abs(const vec3& a) { return vec3(abs(a.x), abs(a.y), abs(a.z)); }
inline vec4 abs(const vec4& a) { return vec4(abs(a.f4)); }
inline quat abs(const quat& a) { return a.w < 0.0 ? -a : a; }

inline vec2 floor(const vec2& a) { return vec2(floor(a.x), floor(a.y)); }
inline vec3 floor(const vec3& a) { return vec3(floor(a.x), floor(a.y), floor(a.z)); }
inline vec4 floor(const vec4& a) { return vec4(floor(a.f4)); }

inline vec2 round(vec2 a) { return vec2(round(a.x), round(a.y)); }
inline vec3 round(vec3 a) { return vec3(round(a.x), round(a.y), round(a.z)); }
inline vec4 round(vec4 a) { return vec4(round(a.f4)); }

inline vec2 frac(vec2 a) { return vec2(frac(a.x), frac(a.y)); }
inline vec3 frac(vec3 a) { return vec3(frac(a.x), frac(a.y), frac(a.z)); }
inline vec4 frac(vec4 a) { return vec4(frac(a.x), frac(a.y), frac(a.z), frac(a.w)); }

inline quat normalize(quat a) { return { normalize(a.v4).f4 }; }
inline quat conjugate(quat a) { return { -a.x, -a.y, -a.z, a.w }; }

inline quat operator+(quat a, quat b) { quat result = { a.f4 + b.f4 }; return result; }
inline quat operator-(quat a, quat b) { quat result = { a.f4 - b.f4 }; return result; }

inline quat operator*(quat a, quat b)
{
	quat result;
	result.w = a.w * b.w - dot(a.v, b.v);
	result.v = a.v * b.w + b.v * a.w + cross(a.v, b.v);
	return result;
}

inline quat operator*(quat q, float s)
{
	quat result;
	result.v4 = q.v4 * s;
	return result;
}

inline vec3 operator*(quat q, vec3 v)
{
	quat p(v.x, v.y, v.z, 0.f);
	return (q * p * conjugate(q)).v;
}

inline dual_quat operator+(const dual_quat& a, const dual_quat& b) { return { a.real + b.real, a.dual + b.dual }; }
inline dual_quat& operator+=(dual_quat& a, const dual_quat& b) { a = a + b; return a; }
inline dual_quat operator*(const dual_quat& a, const dual_quat& b) { return { a.real * b.real, a.dual * b.real + b.dual * a.real }; }
inline dual_quat operator*(const dual_quat& a, float b) { return { a.real * b, a.dual * b }; }
inline dual_quat operator*(float b, const dual_quat& a) { return a * b; }
inline dual_quat& operator*=(dual_quat& q, float v) { q = q * v; return q; }
inline dual_quat normalize(const dual_quat& q) { float n = 1.f / length(q.real.v4); return q * n; }

inline bool operator==(quat a, quat b) { return a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w; }
inline bool operator!=(quat a, quat b) { return !(a == b); }

inline vec2 operator*(mat2 a, vec2 b) { vec2 result = { dot(row(a, 0), b), dot(row(a, 1), b) }; return result; }
inline vec3 operator*(mat3 a, vec3 b) { vec3 result = { dot(row(a, 0), b), dot(row(a, 1), b), dot(row(a, 2), b) }; return result; }

#if ROW_MAJOR
inline vec4 operator*(mat4 a, vec4 b) { vec4 result = { dot(row(a, 0), b), dot(row(a, 1), b), dot(row(a, 2), b), dot(row(a, 3), b) }; return result; }
#else
inline vec4 operator*(mat4 a, vec4 b) { vec4 result = col(a, 0) * b.x + col(a, 1) * b.y + col(a, 2) * b.z + col(a, 3) * b.w; return result; }
#endif

inline vec2 lerp(vec2 l, vec2 u, float t) { return l + t * (u - l); }
inline vec3 lerp(vec3 l, vec3 u, float t) { return l + t * (u - l); }
inline vec4 lerp(vec4 l, vec4 u, float t) { return l + t * (u - l); }
inline quat lerp(quat l, quat u, float t) { quat result; result.v4 = lerp(l.v4, u.v4, t); return normalize(result); }
inline dual_quat lerp(const dual_quat& l, const dual_quat& u, float t) { return { quat(lerp(l.real.v4, u.real.v4, t)), quat(lerp(l.dual.v4, u.dual.v4, t)) }; }

inline trs lerp(const trs& l, const trs& u, float t)
{
	trs result;
	result.position = lerp(l.position, u.position, t);
	result.rotation = lerp(l.rotation, u.rotation, t);
	result.scale = lerp(l.scale, u.scale, t);
	return result;
}

inline vec2 exp(vec2 v) { return vec2(exp(v.x), exp(v.y)); }
inline vec3 exp(vec3 v) { return vec3(exp(v.x), exp(v.y), exp(v.z)); }
inline vec4 exp(vec4 v) { return vec4(exp(v.f4)); }

inline vec2 pow(vec2 v, float e) { return vec2(pow(v.x, e), pow(v.y, e)); }
inline vec3 pow(vec3 v, float e) { return vec3(pow(v.x, e), pow(v.y, e), pow(v.z, e)); }
inline vec4 pow(vec4 v, float e) { return vec4(pow(v.f4, w4_float(e))); }

inline vec2 min(vec2 a, vec2 b) { return vec2(min(a.x, b.x), min(a.y, b.y)); }
inline vec3 min(vec3 a, vec3 b) { return vec3(min(a.x, b.x), min(a.y, b.y), min(a.z, b.z)); }
inline vec4 min(vec4 a, vec4 b) { return vec4(minimum(a.f4, b.f4)); }

inline vec2 max(vec2 a, vec2 b) { return vec2(max(a.x, b.x), max(a.y, b.y)); }
inline vec3 max(vec3 a, vec3 b) { return vec3(max(a.x, b.x), max(a.y, b.y), max(a.z, b.z)); }
inline vec4 max(vec4 a, vec4 b) { return vec4(maximum(a.f4, b.f4)); }

inline float max_element(vec2 a) { return max(a.x, a.y); }
inline float max_element(vec3 a) { return max(a.x, max(a.y, a.z)); }
inline float max_element(vec4 a) { return max(a.x, max(a.y, max(a.z, a.w))); }

inline void decompose(const vec3& a, float& len, vec3& normal, float epsilon = EPSILON)
{
	len = length(a);
	if (len <= epsilon)
	{
		normal = vec3();
	}
	else
	{
		float inv = 1.0f / len;
		normal = vec3(a.x * inv, a.y * inv, a.z * inv);
	}
}

inline quat nlerp(const quat& q, const quat& p, float alpha)
{
	return normalize(quat(
		lerp(q.x, p.x, alpha),
		lerp(q.y, p.y, alpha),
		lerp(q.z, p.z, alpha),
		lerp(q.w, p.w, alpha)));
}

inline quat nlerp_shortest(const quat& q, const quat& p, float alpha)
{
	return nlerp(q, (dot(q, p) < 0.0f ? -p : p), alpha);
}

inline quat slerp_shortest(const quat& q, quat p, float alpha, float eps = 1e-5f)
{
	if (dot(q, p) < 0.0f)
	{
		p = -p;
	}

	float dot_product = dot(q, p);
	float theta = acosf(clamp(dot_product, -1.0f, 1.0f));

	if (theta < eps)
	{
		return nlerp(q, p, alpha);
	}

	quat r = normalize(p - q * dot_product);

	return q * cosf(theta * alpha) + r * sinf(theta * alpha);
}

// Taken from https://zeux.io/2015/07/23/approximating-slerp/
inline quat slerp_shortest_approx(const quat& q, quat p, float alpha)
{
	float ca = dot(q, p);

	if (ca < 0.0f)
	{
		p = -p;
	}

	float d = fabsf(ca);
	float a = 1.0904f + d * (-3.2452f + d * (3.55645f - d * 1.43519f));
	float b = 0.848013f + d * (-1.06021f + d * 0.215638f);
	float k = a * (alpha - 0.5f) * (alpha - 0.5f) + b;
	float oalpha = alpha + alpha * (alpha - 0.5f) * (alpha - 1) * k;

	return nlerp(q, p, oalpha);
}

inline float angle_between(quat q, quat p)
{
	quat diff = abs(conjugate(q) * p);
	return 2.0f * acosf(clamp(diff.w, -1.0f, 1.0f));
}

inline vec3 log(const quat& q, float eps = 1e-8f)
{
	float length = sqrtf(q.x * q.x + q.y * q.y + q.z * q.z);

	if (length < eps)
	{
		return vec3(q.x, q.y, q.z);
	}
	else
	{
		float halfangle = acosf(clamp(q.w, -1.0f, 1.0f));
		return halfangle * (vec3(q.x, q.y, q.z) / length);
	}
}

inline quat exp(const vec3& v, float eps = 1e-8f)
{
	float halfangle = sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);

	if (halfangle < eps)
	{
		return normalize(quat(v.x, v.y, v.z, 1.0f));
	}
	else
	{
		float c = cosf(halfangle);
		float s = sinf(halfangle) / halfangle;
		return quat(s * v.x, s * v.y, s * v.z, c);
	}
}

inline quat quat_from_scaled_angle_axis(const vec3& v, float eps = 1e-8f)
{
	return exp(v / 2.0f, eps);
}

inline vec3 quat_to_scaled_angle_axis(const quat& q, float eps = 1e-8f)
{
	return 2.0f * log(q, eps);
}

inline vec3 quat_differentiate_angular_velocity(
	const quat& next, const quat& curr, float dt, float eps = 1e-8f)
{
	return quat_to_scaled_angle_axis(
		abs(next * conjugate(curr)), eps) / dt;
}

inline quat quat_integrate_angular_velocity(
	const vec3& vel, const quat& cur, float dt, float eps = 1e-8f)
{
	return quat_from_scaled_angle_axis(vel * dt, eps) * cur;
}

inline uint32 max_element_index(vec2 a) { return (a.x > a.y) ? 0 : 1; }
inline uint32 max_element_index(vec3 a) { return (a.x > a.y) ? ((a.x > a.z) ? 0 : 2) : ((a.y > a.z) ? 1 : 2); }
inline uint32 max_element_index(vec4 a) {
	return (a.x > a.y) ? ((a.x > a.z) ? ((a.x > a.w) ? 0 : 3)
		: ((a.z > a.w) ? 2 : 3))
		: ((a.y > a.z) ? ((a.y > a.w) ? 1 : 3)
			: ((a.z > a.w) ? 2 : 3));
}

ERA_CORE_API float map_value(float value, float in_range_a, float in_range_b, float out_range_a, float out_range_b);

inline vec2 remap(vec2 v, vec2 oldL, vec2 oldU, vec2 newL, vec2 newU)
{
	return
	{
		remap(v.x, oldL.x, oldU.x, newL.x, newU.x),
		remap(v.y, oldL.y, oldU.y, newL.y, newU.y),
	};
}

inline vec3 remap(vec3 v, vec3 oldL, vec3 oldU, vec3 newL, vec3 newU)
{
	return
	{
		remap(v.x, oldL.x, oldU.x, newL.x, newU.x),
		remap(v.y, oldL.y, oldU.y, newL.y, newU.y),
		remap(v.z, oldL.z, oldU.z, newL.z, newU.z),
	};
}

inline vec4 remap(vec4 v, vec4 oldL, vec4 oldU, vec4 newL, vec4 newU)
{
	return
	{
		remap(v.x, oldL.x, oldU.x, newL.x, newU.x),
		remap(v.y, oldL.y, oldU.y, newL.y, newU.y),
		remap(v.z, oldL.z, oldU.z, newL.z, newU.z),
		remap(v.w, oldL.w, oldU.w, newL.w, newU.w),
	};
}

ERA_CORE_API mat2 operator*(const mat2& a, const mat2& b);
ERA_CORE_API mat3 operator*(const mat3& a, const mat3& b);
ERA_CORE_API mat3 operator+(const mat3& a, const mat3& b);
ERA_CORE_API mat3& operator+=(mat3& a, const mat3& b);
ERA_CORE_API mat3 operator-(const mat3& a, const mat3& b);
ERA_CORE_API mat4 operator*(const mat4& a, const mat4& b);
ERA_CORE_API mat2 operator*(const mat2& a, float b);
ERA_CORE_API mat3 operator*(const mat3& a, float b);
ERA_CORE_API mat4 operator*(const mat4& a, float b);
ERA_CORE_API mat2 operator*(float a, const mat2& b);
ERA_CORE_API mat3 operator*(float a, const mat3& b);
ERA_CORE_API mat4 operator*(float a, const mat4& b);
ERA_CORE_API mat2& operator*=(mat2& a, float b);
ERA_CORE_API mat3& operator*=(mat3& a, float b);
ERA_CORE_API mat4& operator*=(mat4& a, float b);

inline trs operator*(const trs& a, const trs& b)
{
	trs result;
	result.rotation = a.rotation * b.rotation;
	result.position = a.rotation * (a.scale * b.position) + a.position;
	result.scale = a.scale * b.scale;
	return result;
}

inline bool operator==(const trs& a, const trs& b) 
{ 
	return a.position == b.position && a.rotation == b.rotation && a.scale == b.scale; 
}

inline bool operator!=(const trs& a, const trs& b)
{
	return !operator==(a, b);
}

static inline quat operator-(const quat& q, const quat& p)
{
	return quat(q.x - p.x, q.y - p.y, q.z - p.z, q.w - p.w);
}

ERA_CORE_API mat2 transpose(const mat2& a);
ERA_CORE_API mat3 transpose(const mat3& a);
ERA_CORE_API mat4 transpose(const mat4& a);

ERA_CORE_API mat3 invert(const mat3& m);
ERA_CORE_API mat4 invert(const mat4& m);
ERA_CORE_API trs invert(const trs& t);
ERA_CORE_API mat2 invert(const mat2& m);

ERA_CORE_API float determinant(const mat2& m);
ERA_CORE_API float determinant(const mat3& m);
ERA_CORE_API float determinant(const mat4& m);

ERA_CORE_API float trace(const mat3& m);
ERA_CORE_API float trace(const mat4& m);

ERA_CORE_API vec3 transform_position(const mat4& m, vec3 pos);
ERA_CORE_API vec3 transform_direction(const mat4& m, vec3 dir);
ERA_CORE_API vec3 transform_position(const trs& m, vec3 pos);
ERA_CORE_API vec3 transform_direction(const trs& m, vec3 dir);
ERA_CORE_API vec3 inverse_transform_position(const trs& m, vec3 pos);
ERA_CORE_API vec3 inverse_transform_direction(const trs& m, vec3 dir);

ERA_CORE_API quat rotate_from_to(vec3 from, vec3 to);
ERA_CORE_API quat look_at_quaternion(vec3 forward, vec3 up);
ERA_CORE_API void get_axis_rotation(quat q, vec3& axis, float& angle);
ERA_CORE_API void decompose_quaternion_into_twist_and_swing(quat q, vec3 normalized_twist_axis, quat& twist, quat& swing);
ERA_CORE_API float get_twist_angle(const quat& q, const vec3& axis);
ERA_CORE_API float get_yaw_angle(const quat& q);

ERA_CORE_API quat slerp(quat from, quat to, float t);
ERA_CORE_API quat nlerp(quat* qs, float* weights, uint32 count);

ERA_CORE_API mat3 quaternion_to_mat3(quat q);
ERA_CORE_API mat4 quaternion_to_mat4(quat q);

ERA_CORE_API quat mat3_to_quaternion(const mat3& m);

ERA_CORE_API vec3 quat_to_euler(quat q);
ERA_CORE_API quat euler_to_quat(vec3 euler);

ERA_CORE_API mat3 outer_product(vec3 a, vec3 b);
ERA_CORE_API mat3 get_skew_matrix(vec3 r);

ERA_CORE_API mat4 trs_to_mat4(const trs& transform);
ERA_CORE_API trs mat4_to_trs(const mat4& m);

ERA_CORE_API mat4 create_perspective_projection_matrix(float fov, float aspect, float near_plane, float far_plane);
ERA_CORE_API mat4 create_perspective_projection_matrix(float width, float height, float fx, float fy, float cx, float cy, float near_plane, float far_plane);
ERA_CORE_API mat4 create_perspective_projection_matrix(float r, float l, float t, float b, float near_plane, float far_plane);
ERA_CORE_API mat4 create_orthographic_projection_matrix(float r, float l, float t, float b, float near_plane, float far_plane);
ERA_CORE_API mat4 invert_perspective_projection_matrix(const mat4& m);
ERA_CORE_API mat4 invert_orthographic_projection_matrix(const mat4& m);
ERA_CORE_API mat4 create_translation_matrix(vec3 position);
ERA_CORE_API mat4 create_model_matrix(vec3 position, quat rotation, vec3 scale = vec3(1.f, 1.f, 1.f));
ERA_CORE_API mat4 create_billboard_model_matrix(vec3 position, vec3 eye, vec3 scale);
ERA_CORE_API mat4 create_view_matrix(vec3 eye, float pitch, float yaw);
ERA_CORE_API mat4 create_sky_view_matrix(const mat4& v);
ERA_CORE_API mat4 look_at(vec3 eye, vec3 target, vec3 up);
ERA_CORE_API mat4 create_view_matrix(vec3 position, quat rotation);
ERA_CORE_API mat4 invert_affine(const mat4& m);

ERA_CORE_API bool point_in_triangle(vec3 point, vec3 triA, vec3 triB, vec3& triC);
ERA_CORE_API bool point_in_rectangle(vec2 p, vec2 top_left, vec2 bottom_right);
ERA_CORE_API bool point_in_box(vec3 p, vec3 min_corner, vec3 max_corner);

ERA_CORE_API vec2 direction_to_panorama_uv(vec3 dir);

ERA_CORE_API float angle_to_zero_to_two_pi(float angle);
ERA_CORE_API float angle_to_neg_pi_to_pi(float angle);

ERA_CORE_API vec2 solve_linear_system(const mat2& A, vec2 b);
ERA_CORE_API vec3 solve_linear_system(const mat3& A, vec3 b);

ERA_CORE_API vec3 get_barycentric_coordinates(vec2 a, vec2 b, vec2 c, vec2 p);
ERA_CORE_API vec3 get_barycentric_coordinates(vec3 a, vec3 b, vec3 c, vec3 p);
ERA_CORE_API bool inside_triangle(vec3 barycentrics);

ERA_CORE_API void get_tangents(vec3 normal, vec3& out_tangent, vec3& out_bitangent);
ERA_CORE_API vec3 get_tangent(vec3 normal);

// W = PDF = 1 / 4pi
ERA_CORE_API vec4 uniform_sample_sphere(vec2 E);

struct ERA_CORE_API singular_value_decomposition
{
	mat3 U;
	mat3 V;
	vec3 singular_values;
};

struct ERA_CORE_API QRDecomposition
{
	mat3 Q, R;
};

ERA_CORE_API singular_value_decomposition compute_svd(const mat3& A);
ERA_CORE_API QRDecomposition create_qr_decomposition(const mat3& mat);
ERA_CORE_API bool get_eigen(const mat3& A, vec3& out_eigen_values, mat3& out_eigen_vectors);

template <typename T>
void exclusive_prefix_sum(const T* input, T* output, uint32 count)
{
	if (count > 0)
	{
		output[0] = T(0);
		for (uint32 i = 1; i < count; ++i)
		{
			output[i] = output[i - 1] + input[i - 1];
		}
	}
}

template <typename T>
T sum(const T* input, uint32 count)
{
	T result = 0;
	for (uint32 i = 0; i < count; ++i)
	{
		result += input[i];
	}

	return result;
}

inline bool fuzzy_equals(float a, float b, float threshold = 1e-4f) { return abs(a - b) < threshold; }
inline bool fuzzy_equals(const vec2& a, const vec2& b, float threshold = 1e-4f) { return fuzzy_equals(a.x, b.x, threshold) && fuzzy_equals(a.y, b.y, threshold); }
inline bool fuzzy_equals(const vec3& a, const vec3& b, float threshold = 1e-4f) { return fuzzy_equals(a.x, b.x, threshold) && fuzzy_equals(a.y, b.y, threshold) && fuzzy_equals(a.z, b.z, threshold); }
inline bool fuzzy_equals(const vec4& a, const vec4& b, float threshold = 1e-4f) { return fuzzy_equals(a.x, b.x, threshold) && fuzzy_equals(a.y, b.y, threshold) && fuzzy_equals(a.z, b.z, threshold) && fuzzy_equals(a.w, b.w, threshold); }
inline bool fuzzy_equals(quat a, quat b, float threshold = 1e-4f) { if (dot(a.v4, b.v4) < 0.f) { a.v4 *= -1.f; } return fuzzy_equals(a.x, b.x, threshold) && fuzzy_equals(a.y, b.y, threshold) && fuzzy_equals(a.z, b.z, threshold) && fuzzy_equals(a.w, b.w, threshold); }

inline bool fuzzy_equals(const mat2& a, const mat2& b, float threshold = 1e-4f)
{
	bool result = true;
	for (uint32 i = 0; i < 4; ++i)
	{
		result &= fuzzy_equals(a.m[i], b.m[i], threshold);
	}

	return result;
}

inline bool fuzzy_equals(const mat3& a, const mat3& b, float threshold = 1e-4f)
{
	bool result = true;
	for (uint32 i = 0; i < 9; ++i)
	{
		result &= fuzzy_equals(a.m[i], b.m[i], threshold);
	}

	return result;
}

inline bool fuzzy_equals(const mat4& a, const mat4& b, float threshold = 1e-4f)
{
	bool result = true;
	for (uint32 i = 0; i < 16; ++i)
	{
		result &= fuzzy_equals(a.m[i], b.m[i], threshold);
	}

	return result;
}

inline bool fuzzy_equals(const trs& a, const trs& b, float threshold = 1e-4f)
{
	bool result = true;
	result &= fuzzy_equals(a.position, b.position, threshold);
	result &= fuzzy_equals(a.rotation, b.rotation, threshold);
	result &= fuzzy_equals(a.scale, b.scale, threshold);
	return result;
}

inline bool isUniform(const vec2& v) { return fuzzy_equals(v.x, v.y); }
inline bool isUniform(const vec3& v) { return fuzzy_equals(v.x, v.y) && fuzzy_equals(v.x, v.z); }
inline bool isUniform(const vec4& v) { return fuzzy_equals(v.x, v.y) && fuzzy_equals(v.x, v.z) && fuzzy_equals(v.x, v.w); }

inline quat::quat(vec3 axis, float angle)
{
	w = cos(angle * 0.5f);
	v = axis * sin(angle * 0.5f);
}

inline dual_quat::dual_quat(quat rotation, vec3 translation)
{
	float w = -0.5f * (translation.x * rotation.x + translation.y * rotation.y + translation.z * rotation.z);
	float x = 0.5f * (translation.x * rotation.w + translation.y * rotation.z - translation.z * rotation.y);
	float y = 0.5f * (-translation.x * rotation.z + translation.y * rotation.w + translation.z * rotation.x);
	float z = 0.5f * (translation.x * rotation.y - translation.y * rotation.x + translation.z * rotation.w);

	real = rotation;
	dual = quat(x, y, z, w);
}

inline vec3 dual_quat::get_translation() const
{
	quat tq = dual * conjugate(real);
	return 2.f * tq.v4.xyz;
}

inline quat dual_quat::get_rotation() const
{
	return real;
}

inline mat2::mat2(
	float m00, float m01,
	float m10, float m11)
	:
	m00(m00), m01(m01),
	m10(m10), m11(m11) {}

inline mat3::mat3(
	float m00, float m01, float m02,
	float m10, float m11, float m12,
	float m20, float m21, float m22)
	:
	m00(m00), m01(m01), m02(m02),
	m10(m10), m11(m11), m12(m12),
	m20(m20), m21(m21), m22(m22) {}

inline mat4::mat4(
	float m00, float m01, float m02, float m03,
	float m10, float m11, float m12, float m13,
	float m20, float m21, float m22, float m23,
	float m30, float m31, float m32, float m33)
	:
	m00(m00), m01(m01), m02(m02), m03(m03),
	m10(m10), m11(m11), m12(m12), m13(m13),
	m20(m20), m21(m21), m22(m22), m23(m23),
	m30(m30), m31(m31), m32(m32), m33(m33) {}

inline std::ostream& operator<<(std::ostream& s, vec2 v)
{
	s << "[" << v.x << ", " << v.y << "]";
	return s;
}

ERA_CORE_API quat shortest_arc(const vec3& from, const vec3& to);

inline std::ostream& operator<<(std::ostream& s, vec3 v)
{
	s << "[" << v.x << ", " << v.y << ", " << v.z << "]";
	return s;
}

inline std::ostream& operator<<(std::ostream& s, vec4 v)
{
	s << "[" << v.x << ", " << v.y << ", " << v.z << ", " << v.w << "]";
	return s;
}

inline std::ostream& operator<<(std::ostream& s, quat v)
{
	s << "[" << v.x << ", " << v.y << ", " << v.z << ", " << v.w << "]";
	return s;
}

inline std::ostream& operator<<(std::ostream& s, const mat2& m)
{
	s << "[" << m.m00 << ", " << m.m01 << "]\n";
	s << "[" << m.m10 << ", " << m.m11 << "]";
	return s;
}

inline std::ostream& operator<<(std::ostream& s, const mat3& m)
{
	s << "[" << m.m00 << ", " << m.m01 << ", " << m.m02 << "]\n";
	s << "[" << m.m10 << ", " << m.m11 << ", " << m.m12 << "]\n";
	s << "[" << m.m20 << ", " << m.m21 << ", " << m.m22 << "]";
	return s;
}

inline std::ostream& operator<<(std::ostream& s, const mat4& m)
{
	s << "[" << m.m00 << ", " << m.m01 << ", " << m.m02 << ", " << m.m03 << "]\n";
	s << "[" << m.m10 << ", " << m.m11 << ", " << m.m12 << ", " << m.m13 << "]\n";
	s << "[" << m.m20 << ", " << m.m21 << ", " << m.m22 << ", " << m.m23 << "]\n";
	s << "[" << m.m30 << ", " << m.m31 << ", " << m.m32 << ", " << m.m33 << "]";
	return s;
}

inline std::ostream& operator<<(std::ostream& s, const trs& m)
{
	s << "Position: " << m.position << '\n';
	s << "Rotation: " << m.rotation << '\n';
	s << "Scale: " << m.scale << '\n';
	return s;
}

template <typename T>
inline T evaluate_spline(const float* ts, const T* values, int32 num, float t)
{
	ASSERT(num >= 2);

	// Find key.
	int32 k = 0;
	while (k < num - 1 && ts[k + 1] >= 0.f && ts[k] < t)
		++k;

	if (ts[k + 1] < 0.f)
		num = k + 1;

	// Interpolant.
	const float h1 = clamp01(inverse_lerp(ts[k - 1], ts[k], t));
	const float h2 = h1 * h1;
	const float h3 = h2 * h1;
	const vec4 h(h3, h2, h1, 1.f);

	T result;
	if constexpr (std::is_same_v<std::remove_cv_t<T>, quat> || std::is_same_v<std::remove_cv_t<T>, dual_quat>)
	{
		result = T::zero;
	}
	else
	{
		result = 0;
	}

	int32 m = num - 1;
	result += values[clamp(k - 2, 0, m)] * dot(vec4(-1, 2, -1, 0), h);
	result += values[k - 1] * dot(vec4(3, -5, 0, 2), h);
	result += values[k] * dot(vec4(-3, 4, 1, 0), h);
	result += values[clamp(k + 1, 0, m)] * dot(vec4(1, -1, 0, 0), h);

	result *= 0.5f;

	return result;
}

inline vec2 clamp(const vec2& v, float l, float u)
{ 
	vec2 result;
	result.x = clamp(v.x, l, u);
	result.y = clamp(v.x, l, u);
	return result;
}

inline vec3 clamp(const vec3& v, float l, float u)
{
	vec3 result;
	result.x = clamp(v.x, l, u);
	result.y = clamp(v.x, l, u);
	result.z = clamp(v.z, l, u);
	return result;
}

// maxNumPoints must be a multiple of 4!
// If you want to use this in a shader constant buffer, T can currently be either float or vec4.
template <typename T, uint32 max_num_points_>
struct alignas(16) catmull_rom_spline
{
	static_assert(max_num_points_ > 0 && max_num_points_ % 4 == 0, "Spline max num points must be divisible by 4.");

	float ts[max_num_points_];
	T values[max_num_points_];

	enum { max_num_points = max_num_points_ };

	catmull_rom_spline()
	{
		ts[0] = 0;
		ts[1] = 1;
		ts[2] = -1;

		if constexpr (std::is_same_v<std::remove_cv_t<T>, quat> || std::is_same_v<std::remove_cv_t<T>, dual_quat>)
		{
			values[0] = values[1] = T::identity;
		}
		else
		{
			values[0] = T(0);
			values[1] = T(1);
		}
	}

	catmull_rom_spline(T(*func)(float))
	{
		for (int i = 0; i < max_num_points; ++i)
		{
			ts[i] = i / float(max_num_points - 1);
			values[i] = func(ts[i]);
		}
	}

	inline T evaluate(uint32 numActualPoints, float t) const
	{
		return evaluate_spline(ts, values, numActualPoints, t);
	}
};

// Interop for shaders.
#define spline(T, max_num_points) catmull_rom_spline<T, max_num_points>
#define defineSpline(T, max_num_points) template struct spline(T, max_num_points);