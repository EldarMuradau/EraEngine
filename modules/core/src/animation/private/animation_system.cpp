#include "animation/animation_system.h"
#include "core/jobs/jobs.h"
#include "animation/animation.h"
#include "animation/skinning.h"
#include "animation/animation_pose_sampler.h"

#include "rendering/ecs/renderer_holder_root_component.h"

#include "core/cpu_profiling.h"
#include "core/memory.h"

#include "engine/engine.h"

#include "ecs/update_groups.h"
#include "ecs/rendering/mesh_component.h"
#include "ecs/base_components/transform_component.h"

#include <rttr/policy.h>
#include <rttr/registration>

namespace era_engine::animation
{

    RTTR_REGISTRATION
    {
        using namespace rttr;

        registration::class_<AnimationSystem>("AnimationSystem")
            .constructor<World*>()(policy::ctor::as_raw_ptr, metadata("Tag", std::string("render")))
            .method("update", &AnimationSystem::update)(metadata("update_group", update_types::GAMEPLAY_NORMAL))

            .method("update_mesh", &AnimationSystem::update_mesh)(metadata("update_group", update_types::BEFORE_RENDER));
    }

    AnimationSystem::AnimationSystem(World* _world)
        : System(_world)
    {
        renderer_holder_rc = world->add_root_component<RendererHolderRootComponent>();
        ASSERT(renderer_holder_rc != nullptr);
    }

    AnimationSystem::~AnimationSystem()
    {
        allocator->reset();
    }

    void AnimationSystem::init()
    {
        allocator = make_ref<Allocator>();
    }

    void AnimationSystem::update(float dt)
    {
        ZoneScopedN("AnimationSystem::update");

        struct AnimatedEntity
        {
            AnimationComponent* animation = nullptr;
            SkeletonComponent* skeleton = nullptr;
        };

        std::vector<AnimatedEntity> entities;
        for (auto [entity_handle, animation_component, mesh_component, skeleton_component, transform_component] :
            world->group(components_group<AnimationComponent, MeshComponent, SkeletonComponent, TransformComponent>).each())
        {
            entities.push_back({&animation_component, &skeleton_component});
        }

        JobSystem::parallel_for(0, (uint32)entities.size(), 1, [&entities, dt](uint32 index)
            {
                AnimationComponent& animation_component = *entities[index].animation;
                SkeletonComponent& skeleton_component = *entities[index].skeleton;

                if (animation_component.current_animation == nullptr ||
                    !animation_component.current_animation->is_valid() ||
                    !animation_component.play)
                {
                    return;
                }

                const float anim_duration = animation_component.current_animation->get_duration();

                if (animation_component.current_anim_position < anim_duration)
                {
                    ref<Skeleton> skeleton = skeleton_component.skeleton;

                    AnimationPoseSampler sampler;
                    sampler.init(skeleton.get(), animation_component.current_animation);

                    if (!animation_component.enable_root_motion)
                    {
                        sampler.set_joint_enabled(skeleton->joints[0].name, false);
                    }

                    SkeletonPose result_pose = SkeletonPose(skeleton->joints.size());
                    sampler.sample_pose(animation_component.current_anim_position, result_pose);
                    if (result_pose.is_valid())
                    {
                        if (animation_component.inertial_blend_inited)
                        {
                            animation_component.inertial_sampler->process_pose(dt, &result_pose);
                        }

                        if (animation_component.update_skeleton)
                        {
                            skeleton_component.apply_pose(result_pose);
                        }
                        animation_component.current_animation_pose = result_pose;
                    }
                }

                animation_component.current_anim_position += dt;

                if (animation_component.current_anim_position >= anim_duration)
                {
                    if (animation_component.loop)
                    {
                        animation_component.current_anim_position = fmod(animation_component.current_anim_position, anim_duration);
                    }
                    else
                    {
                        animation_component.current_anim_position = anim_duration;
                    }
                }
            });
    }

    void AnimationSystem::update_mesh(float dt)
    {
        ZoneScopedN("AnimationSystem::update_mesh");

        struct SkinnedEntity
        {
            AnimationComponent* animation = nullptr;
            const MeshComponent* mesh = nullptr;
            const SkeletonComponent* skeleton = nullptr;
        };

        std::vector<SkinnedEntity> entities;
        for (auto [entity_handle, animation_component, mesh_component, skeleton_component, transform_component] :
            world->group(components_group<AnimationComponent, MeshComponent, SkeletonComponent, TransformComponent>).each())
        {
            const DXMesh& dx_mesh = mesh_component.mesh->mesh;
            if (dx_mesh.vertex_buffer.positions == nullptr ||
                dx_mesh.vertex_buffer.positions->element_count == 0)
            {
                continue;
            }

            entities.push_back({&animation_component, &mesh_component, &skeleton_component});
        }

        // skin_object reserves its ranges atomically and each entity writes only its own matrices.
        JobSystem::parallel_for(0, (uint32)entities.size(), 1, [&entities](uint32 index)
            {
                const SkinnedEntity& entity = entities[index];
                const DXMesh& dx_mesh = entity.mesh->mesh->mesh;
                const uint32 num_joints = (uint32)entity.skeleton->skeleton->joints.size();

                auto [vb, skinning_matrices] = skin_object(dx_mesh.vertex_buffer, dx_mesh.vertex_buffer.positions->element_count, num_joints);

                entity.animation->prev_frame_vertex_buffer = entity.animation->current_vertex_buffer;
                entity.animation->current_vertex_buffer = vb;

                trs* global_transforms = (trs*)alloca(sizeof(trs) * num_joints);
                entity.skeleton->get_skinning_matrices_from_local_transforms(global_transforms, skinning_matrices, trs::identity);
            });
    }
}