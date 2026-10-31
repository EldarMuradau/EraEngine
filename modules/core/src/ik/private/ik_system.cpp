#include "ik/ik_system.h"
#include "core/jobs/jobs.h"
#include "ik/ik_component.h"
#include "ik/full_body_ik_component.h"

#include "animation/skeleton.h"
#include "animation/skeleton_component.h"

#include "rendering/ecs/renderer_holder_root_component.h"
#include "rendering/debug_visualization.h"

#include "core/cpu_profiling.h"

#include "ecs/update_groups.h"
#include "ecs/base_components/transform_component.h"

#include <rttr/policy.h>
#include <rttr/registration>

namespace era_engine
{
    RTTR_REGISTRATION
    {
        using namespace rttr;

        registration::class_<IKSystem>("IKSystem")
            .constructor<World*>()(policy::ctor::as_raw_ptr, metadata("Tag", std::string("base")))
            .method("update", &IKSystem::update)(metadata("update_group", update_types::GAMEPLAY_NORMAL),
                metadata("Before", std::vector<std::string>{"SkeletonSystem::update"}),
                metadata("After", std::vector<std::string>{"AnimationSystem::update"}))
            .method("debug_draw", &IKSystem::debug_draw)(metadata("update_group", update_types::RENDER));
    }

    IKSystem::IKSystem(World* _world)
        : System(_world)
    {
        renderer_holder_rc = world->add_root_component<RendererHolderRootComponent>();
        ASSERT(renderer_holder_rc != nullptr);
    }

    IKSystem::~IKSystem()
    {
    }

    void IKSystem::init()
    {
    }

    void IKSystem::update(float dt)
    {
        using namespace animation;

        ZoneScopedN("IKSystem::update");

        for (auto [entity_handle, target_component, transform_component] : world->group(components_group<IKTargetComponent, TransformComponent>).each())
        {
            IKComponent* owner_ik = world->get_entity(target_component.owner_entity).get_component<IKComponent>();
            if (!owner_ik)
            {
                continue;
            }

            if (target_component.chain_index >= owner_ik->chains.size())
            {
                continue;
            }

            auto& chain = owner_ik->chains[target_component.chain_index];

            const trs& target_transform = transform_component.get_world_transform();
            chain.target_world_position = target_transform.position;
            chain.target_world_rotation = target_transform.rotation;
            chain.weight = target_component.weight;
        }

        for (auto [entity_handle, target_component, transform_component] : world->group(components_group<FBIKTargetComponent, TransformComponent>).each())
        {
            FBIKComponent* owner_ik = world->get_entity(target_component.owner_entity).get_component_if_exists<FBIKComponent>();
            if (owner_ik == nullptr)
            {
                continue;
            }

            if (target_component.effector_index >= owner_ik->effectors.size())
            {
                continue;
            }

            IKEffector& effector = owner_ik->effectors[target_component.effector_index];

            const trs& target_transform = transform_component.get_world_transform();
            effector.target_position = target_transform.position;
            effector.target_rotation = target_transform.rotation;
            effector.position_weight = target_component.position_weight;
            effector.rotation_weight = target_component.rotation_weight;
        }

        struct FullBodyIKEntity
        {
            FBIKComponent* ik = nullptr;
            SkeletonComponent* skeleton = nullptr;
            trs world_transform;
        };

        std::vector<FullBodyIKEntity> full_body_entities;
        for (auto [handle, full_body_ik_component, skeleton_component, transform_component] :
            world->group(components_group<FBIKComponent, SkeletonComponent, TransformComponent>).each())
        {
            if (!full_body_ik_component.enabled || !full_body_ik_component.is_initialized() || full_body_ik_component.settings.weight <= 0.0f)
            {
                continue;
            }

            full_body_entities.push_back({&full_body_ik_component, &skeleton_component, transform_component.get_world_transform()});
        }

        JobSystem::parallel_for(0, (uint32)full_body_entities.size(), 1, [&full_body_entities](uint32 index)
            {
                const FullBodyIKEntity& entity = full_body_entities[index];

                SkeletonPose current_pose = entity.skeleton->get_current_pose();
                if (!current_pose.is_valid())
                {
                    return;
                }

                if (entity.ik->solver->solve(current_pose, entity.world_transform, entity.ik->effectors, entity.ik->settings))
                {
                    entity.skeleton->apply_pose(current_pose);
                }
            });

        struct ChainIKEntity
        {
            IKComponent* ik = nullptr;
            SkeletonComponent* skeleton = nullptr;
            trs world_transform;
        };

        std::vector<ChainIKEntity> chain_entities;
        for (auto [handle, ik_component, skeleton_component, transform_component] :
            world->group(components_group<IKComponent, SkeletonComponent, TransformComponent>).each())
        {
            if (!ik_component.enabled || !ik_component.is_initialized())
            {
                continue;
            }

            chain_entities.push_back({&ik_component, &skeleton_component, transform_component.get_world_transform()});
        }

        // Runs after the full body pass, so a character may have both.
        JobSystem::parallel_for(0, (uint32)chain_entities.size(), 1, [&chain_entities](uint32 index)
            {
                const ChainIKEntity& entity = chain_entities[index];

                SkeletonPose current_pose = entity.skeleton->get_current_pose();
                if (!current_pose.is_valid())
                {
                    return;
                }

                bool has_any_enabled = false;
                for (auto& chain : entity.ik->chains)
                {
                    if (!chain.enabled || chain.weight <= 0.0f)
                    {
                        continue;
                    }
                    if (chain.solve(current_pose, entity.world_transform))
                    {
                        has_any_enabled = true;
                    }
                }

                if (has_any_enabled)
                {
                    entity.skeleton->apply_pose(current_pose);
                }
            });
    }

    void IKSystem::debug_draw(float dt)
    {
        for (auto [entity_handle, target_component, transform_component] : world->group(components_group<IKTargetComponent, TransformComponent>).each())
        {
            vec3 point = transform_component.get_world_transform().position;
            render_wire_sphere(point, 0.05f, vec4(1.0f, 0.f, 1.0f, 1.0f), renderer_holder_rc->ldr_render_pass);
        }

        for (auto [entity_handle, target_component, transform_component] : world->group(components_group<FBIKTargetComponent, TransformComponent>).each())
        {
            vec3 point = transform_component.get_world_transform().position;
            render_wire_sphere(point, 0.05f, vec4(1.0f, 0.f, 1.0f, 1.0f), renderer_holder_rc->ldr_render_pass);
        }

        for (auto [entity_handle, full_body_ik_component] : world->view<FBIKComponent>().each())
        {
            if (!full_body_ik_component.enabled || !full_body_ik_component.draw_debug || !full_body_ik_component.is_initialized())
            {
                continue;
            }

            const FBIKSolver& solver = *full_body_ik_component.solver;
            const std::vector<FBIKSolver::Node>& nodes = solver.get_nodes();
            const std::vector<vec3>& positions = solver.get_solved_positions();

            if (positions.size() != nodes.size())
            {
                continue;
            }

            for (size_t i = 0; i < nodes.size(); ++i)
            {
                const vec4 color = nodes[i].effector != INVALID_JOINT ? vec4(0.0f, 1.0f, 1.0f, 1.0f) : vec4(1.0f, 1.0f, 0.0f, 1.0f);
                render_wire_sphere(positions[i], 0.03f, color, renderer_holder_rc->ldr_render_pass);

                if (nodes[i].parent != INVALID_JOINT)
                {
                    render_line(positions[nodes[i].parent], positions[i], vec4(1.0f, 1.0f, 0.0f, 1.0f), renderer_holder_rc->ldr_render_pass);
                }
            }

            for (const IKEffector& effector : full_body_ik_component.effectors)
            {
                if (effector.enabled && effector.position_weight > 0.0f)
                {
                    render_wire_sphere(effector.target_position, 0.06f, vec4(0.0f, 1.f, 1.0f, 1.0f), renderer_holder_rc->ldr_render_pass);
                }
            }
        }

        for (auto [entity_handle, ik_component] : world->view<IKComponent>().each())
        {
            for (const IKChain& chain : ik_component.chains)
            {
                if (!chain.enabled || chain.solver == nullptr)
                {
                    continue;
                }

                for (const vec3& position : chain.solver->get_solved_positions())
                {
                    render_wire_sphere(position, 0.05f, vec4(1.0f, 1.f, 0.0f, 1.0f), renderer_holder_rc->ldr_render_pass);
                }
                render_wire_sphere(chain.target_world_position, 0.06f, vec4(0.0f, 1.f, 1.0f, 1.0f), renderer_holder_rc->ldr_render_pass);
            }
        }
    }
}
