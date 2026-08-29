#include "motion_matching/motion/spring_motion_utils.h"

namespace era_engine
{
	float SpringMotionUtils::damper_exact(float x, float g, float halflife, float dt, float eps)
	{
		return lerpf(x, g, 1.0f - fast_negexpf((LN2f * dt) / (halflife + eps)));
	}

	vec3 SpringMotionUtils::damper_exact(const vec3& x, const vec3& g, float halflife, float dt, float eps)
	{
		return lerp(x, g, 1.0f - fast_negexpf((LN2f * dt) / (halflife + eps)));
	}

	quat SpringMotionUtils::damper_exact(const quat& x, const quat& g, float halflife, float dt, float eps)
	{
		return slerp_shortest_approx(x, g, 1.0f - fast_negexpf((LN2f * dt) / (halflife + eps)));
	}

	float SpringMotionUtils::damp_adjustment_exact(float g, float halflife, float dt, float eps)
	{
		return g * (1.0f - fast_negexpf((LN2f * dt) / (halflife + eps)));
	}

	vec3 SpringMotionUtils::damp_adjustment_exact(const vec3& g, float halflife, float dt, float eps)
	{
		return g * (1.0f - fast_negexpf((LN2f * dt) / (halflife + eps)));
	}

	quat SpringMotionUtils::damp_adjustment_exact(const quat& g, float halflife, float dt, float eps)
	{
		return slerp_shortest_approx(quat::identity, g, 1.0f - fast_negexpf((LN2f * dt) / (halflife + eps)));
	}

	void SpringMotionUtils::simple_spring_damper_exact(float& x, float& v, const float x_goal, const float halflife, const float dt)
	{
		float y = halflife_to_damping(halflife) / 2.0f;
		float j0 = x - x_goal;
		float j1 = v + j0 * y;
		float eydt = fast_negexpf(y * dt);

		x = eydt * (j0 + j1 * dt) + x_goal;
		v = eydt * (v - j1 * y * dt);
	}

	void SpringMotionUtils::simple_spring_damper_exact(vec3& x, vec3& v, const vec3& x_goal, const float halflife, const float dt)
	{
		float y = halflife_to_damping(halflife) / 2.0f;
		vec3 j0 = x - x_goal;
		vec3 j1 = v + j0 * y;
		float eydt = fast_negexpf(y * dt);

		x = eydt * (j0 + j1 * dt) + x_goal;
		v = eydt * (v - j1 * y * dt);
	}

	void SpringMotionUtils::simple_spring_damper_exact(quat& x, vec3& v, const quat& x_goal, float halflife, float dt)
	{
		float y = halflife_to_damping(halflife) / 2.0f;

		vec3 j0 = quat_to_scaled_angle_axis(abs(x * conjugate(x_goal)));
		vec3 j1 = v + j0 * y;

		float eydt = fast_negexpf(y * dt);

		x = quat_from_scaled_angle_axis(eydt * (j0 + j1 * dt)) * x_goal;
		v = eydt * (v - j1 * y * dt);
	}

	void SpringMotionUtils::decay_spring_damper_exact(float& x, float& v, float halflife, float dt)
	{
		float y = halflife_to_damping(halflife) / 2.0f;
		float j1 = v + x * y;
		float eydt = fast_negexpf(y * dt);

		x = eydt * (x + j1 * dt);
		v = eydt * (v - j1 * y * dt);
	}

	void SpringMotionUtils::decay_spring_damper_exact(vec3& x, vec3& v, float halflife, float dt)
	{
		float y = halflife_to_damping(halflife) / 2.0f;
		vec3 j1 = v + x * y;
		float eydt = fast_negexpf(y * dt);

		x = eydt * (x + j1 * dt);
		v = eydt * (v - j1 * y * dt);
	}

	void SpringMotionUtils::decay_spring_damper_exact(quat& x, vec3& v, float halflife, float dt)
	{
		float y = halflife_to_damping(halflife) / 2.0f;

		vec3 j0 = quat_to_scaled_angle_axis(x);
		vec3 j1 = v + j0 * y;

		float eydt = fast_negexpf(y * dt);

		x = quat_from_scaled_angle_axis(eydt * (j0 + j1 * dt));
		v = eydt * (v - j1 * y * dt);
	}

	void SpringMotionUtils::inertialize_transition(vec3& off_x, vec3& off_v, const vec3& src_x, const vec3& src_v, const vec3& dst_x, const vec3& dst_v)
	{
		off_x = (src_x + off_x) - dst_x;
		off_v = (src_v + off_v) - dst_v;
	}

	void SpringMotionUtils::inertialize_update(vec3& out_x, vec3& out_v, vec3& off_x, vec3& off_v, const vec3& in_x, const vec3& in_v, float halflife, float dt)
	{
		decay_spring_damper_exact(off_x, off_v, halflife, dt);
		out_x = in_x + off_x;
		out_v = in_v + off_v;
	}

	void SpringMotionUtils::inertialize_transition(quat& off_x, vec3& off_v, const quat& src_x, const vec3& src_v, const quat& dst_x, const vec3& dst_v)
	{
		off_x = abs((off_x * src_x) * conjugate(dst_x));
		off_v = (off_v + src_v) - dst_v;
	}

	void SpringMotionUtils::inertialize_update(quat& out_x, vec3& out_v, quat& off_x, vec3& off_v, const quat& in_x, const vec3& in_v, float halflife, float dt)
	{
		decay_spring_damper_exact(off_x, off_v, halflife, dt);
		out_x = off_x * in_x;
		out_v = off_v + (off_x * in_v);
	}
}
