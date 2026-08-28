#pragma once

#include "core_api.h"
#include "core/ring_buffer.h"

#include "animation/skeleton.h"

namespace era_engine::animation
{
    class ERA_CORE_API InertialBlendSampler final
    {
    public:
        InertialBlendSampler(const Skeleton* _skeleton);

        const Skeleton* get_skeleton() const;

        void reset_blend_time(float _blend_time, float _elapsed_blend_time = 0.f);
        void process_pose(float dt, SkeletonPose* output_pose, bool should_cache_pose = true);
        void blend_pose(SkeletonPose* output_pose) const;
        void cache_pose(float dt, SkeletonPose* output_pose);
        void init_inertial_blend(const SkeletonPose& current_pose);

        std::optional<SkeletonPose> get_last_pose() const;
        float get_last_pose_anim_position() const;

        struct InertialJointSpace
        {
            vec3 axis;
            float value = 0.0f;
            float velocity = 0.0f;

            float sign = 0.0f;
            float duration = 0.0f;
            float duration_pow2 = 0.0f;
            float duration_pow3 = 0.0f;
            float duration_pow4 = 0.0f;
            float duration_pow5 = 0.0f;
            float a0 = 0.0;
            float A = 0.0f;
            float B = 0.0f;
            float C = 0.0f;
            float D = 0.0f;
        };

        struct InertialJointTransition final
        {
            InertialJointSpace translation;
            InertialJointSpace rotation;

            float duration = 0.0;

            void reset();
        };

        bool is_in_blend() const;

        const std::vector<InertialJointTransition>& get_inertialization_info() const;

        float blend_elapsed = 0.0f;

    private:
        struct InertialSkeletonPose
        {
            SkeletonPose pose;
            float dt_to_prev = 0.0f;
            float anim_position = 0.0f;
        };

        const Skeleton* skeleton = nullptr;

        std::vector<InertialJointTransition> joints_transitions;
        RingBuffer<InertialSkeletonPose> pose_buffer;

        mutable std::mutex lock;

        float blend_time = 0.0f;
        float max_blend_time = 0.0f;
    };
}