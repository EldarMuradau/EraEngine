#pragma once 

#include "motion_matching_api.h"

#include "motion_matching/common.h"

#include <core/math.h>

namespace era_engine 
{
    static inline float halflife_to_damping(float halflife, float eps = 1e-5f)
    {
        return (4.0f * LN2f) / (halflife + eps);
    }

    static inline float damping_to_halflife(float damping, float eps = 1e-5f)
    {
        return (4.0f * LN2f) / (damping + eps);
    }

    static inline float frequency_to_stiffness(float frequency)
    {
        return squaref(2.0f * PIf * frequency);
    }

    static inline float stiffness_to_frequency(float stiffness)
    {
        return sqrtf(stiffness) / (2.0f * PIf);
    }

	class ERA_MOTION_MATCHING_API SpringMotionUtils
	{
	public:
		SpringMotionUtils() = delete;

        static float damper_exact(float x, float g, float halflife, float dt, float eps = 1e-5f);

        static vec3 damper_exact(const vec3& x, const vec3& g, float halflife, float dt, float eps = 1e-5f);

        static quat damper_exact(const quat& x, const quat& g, float halflife, float dt, float eps = 1e-5f);

        static float damp_adjustment_exact(float g, float halflife, float dt, float eps = 1e-5f);

        static vec3 damp_adjustment_exact(const vec3& g, float halflife, float dt, float eps = 1e-5f);

        static quat damp_adjustment_exact(const quat& g, float halflife, float dt, float eps = 1e-5f);

        static void simple_spring_damper_exact(
            float& x,
            float& v,
            float x_goal,
            float halflife,
            float dt);

        static void simple_spring_damper_exact(
            vec3& x,
            vec3& v,
            const vec3& x_goal,
            float halflife,
            float dt);

        static void simple_spring_damper_exact(
            quat& x,
            vec3& v,
            const quat& x_goal,
            float halflife,
            float dt);

        static void decay_spring_damper_exact(
            float& x,
            float& v,
            float halflife,
            float dt);

        static void decay_spring_damper_exact(
            vec3& x,
            vec3& v,
            float halflife,
            float dt);

        static void decay_spring_damper_exact(
            quat& x,
            vec3& v,
            float halflife,
            float dt);

        static void inertialize_transition(
            vec3& off_x,
            vec3& off_v,
            const vec3& src_x,
            const vec3& src_v,
            const vec3& dst_x,
            const vec3& dst_v);

        static void inertialize_update(
            vec3& out_x,
            vec3& out_v,
            vec3& off_x,
            vec3& off_v,
            const vec3& in_x,
            const vec3& in_v,
            float halflife,
            float dt);

        static void inertialize_transition(
            quat& off_x,
            vec3& off_v,
            const quat& src_x,
            const vec3& src_v,
            const quat& dst_x,
            const vec3& dst_v);

        static void inertialize_update(
            quat& out_x,
            vec3& out_v,
            quat& off_x,
            vec3& off_v,
            const quat& in_x,
            const vec3& in_v,
            float halflife,
            float dt);
	};
}