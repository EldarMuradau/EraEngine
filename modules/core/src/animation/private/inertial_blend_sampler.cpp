#include "animation/inertial_blend_sampler.h"

namespace era_engine::animation
{
    InertialBlendSampler::InertialBlendSampler(const Skeleton* _skeleton)
        : pose_buffer(2)
    {
        skeleton = _skeleton;
        joints_transitions.resize(skeleton->joints.size());
    }

    const Skeleton* InertialBlendSampler::get_skeleton() const
    {
        return skeleton;
    }

    void InertialBlendSampler::reset_blend_time(float _blend_time, float _elapsed_blend_time)
    {
        blend_elapsed = _elapsed_blend_time;
        if (pose_buffer.is_empty())
        {
            blend_time = 0.0f;
            max_blend_time = 0.0f;
        }
        else
        {
            blend_time = _blend_time;
            max_blend_time = -1.0f;
        }
    }

    void InertialBlendSampler::process_pose(float dt, SkeletonPose* output_pose, bool should_cache_pose)
    {
        blend_elapsed += dt;

        if (is_in_blend())
        {
            blend_pose(output_pose);
        }

        if (should_cache_pose)
        {
            cache_pose(dt, output_pose);
        }
    }

    void InertialBlendSampler::blend_pose(SkeletonPose* output_pose) const
    {
        if (output_pose)
        {
            auto calc_inertial_factor = [](float elapsed, const InertialJointSpace& space)
            {
                if (elapsed >= space.duration - EPSILON)
                {
                    return 0.0f;
                }

                float result = (((((space.A * elapsed) + space.B) * elapsed + space.C) * elapsed + space.D) * elapsed + space.velocity) * elapsed + space.value;

                return result * space.sign;
            };

            for (uint32 joint_index = 1; joint_index < skeleton->joints.size(); ++joint_index)
			{
				const InertialJointTransition& joint_transitions = joints_transitions[joint_index];
				JointTransform joint_transform = output_pose->get_joint_transform(joint_index);

				float translation_inertia = calc_inertial_factor(blend_elapsed, joint_transitions.translation);

				const vec3 inertial_translation = joint_transitions.translation.axis * translation_inertia + joint_transform.get_translation();

				joint_transform.set_translation(inertial_translation);

				float rotation_inertia = calc_inertial_factor(blend_elapsed, joint_transitions.rotation);
				const quat inertial_rotation = quat(joint_transitions.rotation.axis, rotation_inertia) * joint_transform.get_rotation();

				joint_transform.set_rotation(normalize(inertial_rotation));

				output_pose->set_joint_transform(joint_transform, joint_index);
            }
        }
    }

    void InertialBlendSampler::cache_pose(float dt, SkeletonPose* output_pose)
    {
        if (output_pose != nullptr && 
            dt > 0.f)
        {
            pose_buffer.push({ *output_pose, dt, blend_elapsed });
        }
    }

    void InertialBlendSampler::init_inertial_blend(const SkeletonPose& current_pose)
    {
        if (max_blend_time < 0.0f)
        {
            InertialSkeletonPose& prev1_pose = pose_buffer[pose_buffer.get_size() - 1];
            InertialSkeletonPose& prev2_pose = pose_buffer[max(int32(pose_buffer.get_size()) - 2, 0)];
            max_blend_time = -1.0f;

            for (uint32 joint_index = 0; joint_index < skeleton->joints.size(); ++joint_index)
            {
                const uint32 parent_joint_index = skeleton->joints[joint_index].parent_id;
                if (parent_joint_index == INVALID_JOINT)
                {
                    continue;
                }

                const JointTransform& current_joint_transform = current_pose.get_joint_transform(joint_index);
                const JointTransform& prev1_joint_transform = prev1_pose.pose.get_joint_transform(joint_index);
                const JointTransform& prev2_joint_transform = prev2_pose.pose.get_joint_transform(joint_index);

                InertialJointTransition& joint_transitions = joints_transitions[joint_index];
                joint_transitions.reset();

                joint_transitions.duration = blend_time;
                max_blend_time = max(max_blend_time, joint_transitions.duration);

                const float delta_time = prev1_pose.dt_to_prev;
                ASSERT(delta_time >= 0.0f);

                auto calc_intertialization_factors = [](InertialJointSpace& space)
                {
                    space.sign = 1.0f;
                    float fixed_value = space.value;
                    float fixed_velocity = space.velocity;
                    if (space.value < 0.0f)
                    {
                        fixed_value = -fixed_value;
                        fixed_velocity = -fixed_velocity;
                        space.sign = -1.0f;
                    }

                    if (fixed_velocity > 0.0f)
                    {
                        fixed_velocity = 0.0f;
                    }

                    ASSERT(fixed_value >= 0.0f);
                    ASSERT(fixed_velocity <= 0.0f);

                    if (fixed_velocity < -1e-4f)
                    {
                        space.duration = min(space.duration, -5.0f * fixed_value / fixed_velocity);
                    }

                    space.duration_pow2 = space.duration * space.duration;
                    space.duration_pow3 = space.duration * space.duration_pow2;
                    space.duration_pow4 = space.duration * space.duration_pow3;
                    space.duration_pow5 = space.duration * space.duration_pow4;

                    space.a0 = max(0.0f, (-8.0f * space.duration * fixed_velocity - 20.0f * fixed_value) / space.duration_pow2);
                    space.A = -0.5f * (space.a0 * space.duration_pow2 + 6.0f * space.duration * fixed_velocity + 12.0f * fixed_value) / space.duration_pow5;
                    space.B = 0.5f * (3.0f * space.a0 * space.duration_pow2 + 16.0f * space.duration * fixed_velocity + 30.0f * fixed_value) / space.duration_pow4;
                    space.C = -0.5f * (3.0f * space.a0 * space.duration_pow2 + 12.0f * space.duration * fixed_velocity + 20.0f * fixed_value) / space.duration_pow3;
                    space.D = 0.5f * space.a0;

                    space.value = fixed_value;
                    space.velocity = fixed_velocity;

                    ASSERT(fuzzy_equals(space.value, fixed_value));
                    ASSERT(fuzzy_equals(space.velocity, fixed_velocity));
                };

                // Translation inertialization.
                {
                    const vec3 translation_axis = prev1_joint_transform.get_translation() - current_joint_transform.get_translation();
                    decompose(translation_axis, joint_transitions.translation.value, joint_transitions.translation.axis);

                    ASSERT(!isnan(joint_transitions.translation.value) && !isinf(joint_transitions.translation.value));

                    if (delta_time > EPSILON && 
                        joint_transitions.translation.value > EPSILON)
                    {
                        const vec3 prev_translation = prev2_joint_transform.get_translation() - current_joint_transform.get_translation();
                        const float prev_translation_magnitude = dot(prev_translation, joint_transitions.translation.axis);
                        joint_transitions.translation.velocity = (joint_transitions.translation.value - prev_translation_magnitude) / delta_time;
                    }
                    joint_transitions.translation.duration = joint_transitions.duration;
                    calc_intertialization_factors(joint_transitions.translation);
                }

                // Rotation inertialization.
                {
                    const quat inversed_current_q = conjugate(current_joint_transform.get_rotation());
                    const quat q = prev1_joint_transform.get_rotation() * inversed_current_q;

                    get_axis_rotation(q, joint_transitions.rotation.axis, joint_transitions.rotation.value);

                    joint_transitions.rotation.value = angle_to_neg_pi_to_pi(joint_transitions.rotation.value);

                    if (delta_time > EPSILON && 
                        joint_transitions.rotation.value > EPSILON)
                    {
                        const quat prev_q = prev2_joint_transform.get_rotation() * inversed_current_q;
                        const float prev_angle = get_twist_angle(prev_q, joint_transitions.rotation.axis);
                        joint_transitions.rotation.velocity = angle_to_neg_pi_to_pi(joint_transitions.rotation.value - prev_angle) / delta_time;
                    }

                    joint_transitions.rotation.duration = joint_transitions.duration;
                    calc_intertialization_factors(joint_transitions.rotation);
                }
            }
        }
    }

    std::optional<SkeletonPose> InertialBlendSampler::get_last_pose() const
    {
        if (pose_buffer.is_empty())
        {
            return std::nullopt;
        }
        return pose_buffer.get_back().value().pose;
    }

    float InertialBlendSampler::get_last_pose_anim_position() const
    {
        if (pose_buffer.is_empty())
        {
            return 0.0f;
        }
        return pose_buffer.get_back().value().anim_position;
    }

    bool InertialBlendSampler::is_in_blend() const
    {
        return blend_elapsed >= 0.f && 
            (blend_elapsed + EPSILON) < max_blend_time;
    }

    const std::vector<InertialBlendSampler::InertialJointTransition>& InertialBlendSampler::get_inertialization_info() const
    {
        return joints_transitions;
    }

    void InertialBlendSampler::InertialJointTransition::reset()
    {
        translation.value = 0.0f;
        translation.velocity = 0.0f;
        rotation.value = 0.0f;
        rotation.velocity = 0.0f;
        duration = 0.0f;
    }
}
