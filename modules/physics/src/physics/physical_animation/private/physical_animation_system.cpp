#include "physics/physical_animation/physical_animation_system.h"
#include "physics/core/physics.h"
#include "physics/core/physics_utils.h"
#include "physics/body_component.h"
#include "physics/joint.h"
#include "physics/core/physics_utils.h"
#include "physics/shape_utils.h"
#include "physics/collisions_holder_root_component.h"
#include "physics/cct_component.h"
#include "physics/physical_animation/physical_ragdoll_builder.h"
#include "physics/scene_queries.h"
#include "physics/shape_component.h"
#include "physics/physical_animation/ragdoll_profile.h"
#include "physics/physical_animation/ragdoll_profiles_asset.h"
#include "physics/physical_animation/physical_animation_utils.h"

#include <core/cpu_profiling.h>
#include <core/debug/debug_var.h>
#include <core/log.h>
#include <core/traits.h>
#include <core/string.h>

#include <asset/game_asset.h>

#include <rendering/debug_visualization.h>
#include <rendering/ecs/renderer_holder_root_component.h>

#include <ecs/update_groups.h>
#include <ecs/base_components/name_component.h>
#include <ecs/reflection_utils.h>

#include <rttr/policy.h>
#include <rttr/registration>

#include <imgui/imgui.h>
#include <physics/vehicles/w4_vehicle_component.h>

namespace era_engine::physics
{
    RTTR_REGISTRATION
    {
        using namespace rttr;

        registration::class_<PhysicalAnimationSystem>("PhysicalAnimationSystem")
            .constructor<World*>()(policy::ctor::as_raw_ptr, metadata("Tag", std::string("physics")))

            .method("update_profile_data", &PhysicalAnimationSystem::update_profile_data)(metadata("update_group", update_types::BEFORE_RENDER),
                metadata("After", std::vector<std::string>{"RenderSystem::before_render"}))

            .method("update_before_physics", &PhysicalAnimationSystem::update_before_physics)(metadata("update_group", update_types::PHYSICS),
                metadata("Before", std::vector<std::string>{"PhysicsSystem::update", "JointsSystem::update"}))

            .method("update_after_physics", &PhysicalAnimationSystem::update_after_physics)(metadata("update_group", update_types::PHYSICS),
                metadata("After", std::vector<std::string>{"PhysicsSystem::update"}))

            .method("update_normal", &PhysicalAnimationSystem::update_normal)(metadata("update_group", update_types::GAMEPLAY_NORMAL),
                metadata("After", std::vector<std::string>{"SkeletonSystem::update"}));
    }

    static DebugVar<bool> enable_always = DebugVar<bool>("physics.physical_animation.enable_always", false);
    static DebugVar<bool> force_simulate = DebugVar<bool>("physics.physical_animation.force_simulate", false);
    static DebugVar<bool> force_ragdoll = DebugVar<bool>("physics.physical_animation.force_ragdoll", false);
    static DebugVar<bool> force_disable_ragdoll = DebugVar<bool>("physics.physical_animation.force_disable_ragdoll", false);

    static DebugVar<bool> force_sync_profile = DebugVar<bool>("physics.physical_animation.force_sync_profile", false);
    static DebugVar<bool> force_drag_profile = DebugVar<bool>("physics.physical_animation.force_drag_profile", false);

    static constexpr const char* RAGDOLL_PROFILES_ASSET_PATH = "/resources/assets/physics/ragdoll_profiles";
    static constexpr float DRIVE_POSE_STEP_COEFF = 0.5f;

    static std::string get_ragdoll_profiles_asset_path()
    {
        return get_asset_path(RAGDOLL_PROFILES_ASSET_PATH) + RagdollProfilesAsset::get_asset_type_impl();
    }

    static const char* get_entity_name(Entity entity)
    {
        const NameComponent* name_component = entity.get_component_if_exists<NameComponent>();
        return name_component != nullptr ? name_component->name : "<unnamed>";
    }

    static trs blend_rigid_pose(const trs& from, const trs& to, float t)
    {
        return trs(lerp(from.position, to.position, t), nlerp_shortest(from.rotation, to.rotation, t), vec3(1.0f));
    }

    static float get_shortest_angle(const quat& q)
    {
        float angle = 0.0f;
        vec3 axis = vec3::zero;
        get_axis_rotation(q, axis, angle);
        return angle > M_PI ? 2.0f * M_PI - angle : angle;
    }

    PhysicalAnimationSystem::PhysicalAnimationSystem(World* _world)
        : System(_world)
    {
    }

    void PhysicalAnimationSystem::init()
    {
        using namespace animation;

        entt::registry& registry = world->get_registry();
        registry.on_construct<PhysicalAnimationComponent>().connect<&PhysicalAnimationSystem::on_pac_created>(this);
        registry.on_destroy<PhysicalAnimationComponent>().connect<&PhysicalAnimationSystem::on_pac_removed>(this);

        ragdolls_group = world->group(components_group<TransformComponent, PhysicalAnimationComponent, AnimationComponent, SkeletonComponent>);

        collisions_holder_rc = world->add_root_component<CollisionsHolderRootComponent>();
        renderer_holder_rc = world->add_root_component<RendererHolderRootComponent>();

        load_profiles_asset();

        pose_sampler = std::make_unique<RagdollDrivePoseSampler>();
    }

    void PhysicalAnimationSystem::update_before_physics(float dt)
    {
        ZoneScopedN("PhysicalAnimationSystem::update_before_physics");

        process_added_pacs();

        for (auto&& [entity_handle, transform_component, physical_animation_component, animm, skeleton] : ragdolls_group.each())
        {
            if (physical_animation_component.simulated.is_changed())
            {
                for (const EntityPtr& limb_ptr : physical_animation_component.limbs)
                {
                    Entity limb = limb_ptr.get();
                    limb.get_component<PhysicalAnimationLimbComponent>()->simulated = physical_animation_component.simulated;
                }
                physical_animation_component.simulated.sync_changes();
            }
        }

        for (auto&& [entity_handle, transform_component, physical_animation_component, animm, skeleton] : ragdolls_group.each())
        {
            // Applies inertia dependent settings once all bodies exist and sets loaded.
            PhysicalRagdollBuilder::try_finalize_ragdoll(&physical_animation_component);
        }

        update_ragdolls(dt);
    }

    void PhysicalAnimationSystem::update_after_physics(float dt)
    {
        ZoneScopedN("PhysicalAnimationSystem::update_after_physics");

        if (active_ragdolls.empty())
        {
            return;
        }

        for (PhysicalAnimationComponent* physical_animation_component : active_ragdolls)
        {
            const trs& world_transform = physical_animation_component->get_entity().get_component<TransformComponent>()->get_world_transform();
            const trs inverse_world_transform = invert(world_transform);

            for (const EntityPtr& limb_ptr : physical_animation_component->limbs)
            {
                Entity limb = limb_ptr.get();
                PhysicalAnimationLimbComponent* limb_data_component = limb.get_component<PhysicalAnimationLimbComponent>();

                ASSERT(limb_data_component != nullptr);

                const trs physics_world_pose = PhysicsUtils::get_actor_world_pose_locked(limb);

                limb_data_component->prev_physics_pose = limb_data_component->physics_pose;
                limb_data_component->physics_pose = inverse_world_transform * physics_world_pose;
                limb_data_component->deviation = compute_limb_deviation(limb_data_component, physics_world_pose);

                limb_data_component->collision.is_colliding = false;
                limb_data_component->collision.touching_dynamic = false;
                limb_data_component->collision.impulse = vec3::zero;
            }
        }

        filter_states_by_collisions(dt);
    }

    void PhysicalAnimationSystem::update_normal(float dt)
    {
        using namespace animation;

        ZoneScopedN("PhysicalAnimationSystem::update_normal");

        for (auto&& [entity_handle, transform_component, physical_animation_component, animation_component, skeleton_component] : ragdolls_group.each())
        {
            if (physical_animation_component.loaded)
            {
                physical_animation_component.push_animation_pose(skeleton_component.get_current_override_pose(), dt);
            }
        }

        if (active_ragdolls.empty())
        {
            return;
        }

        for (PhysicalAnimationComponent* physical_animation_component : active_ragdolls)
        {
            Entity ragdoll = physical_animation_component->get_entity();
            if (!ragdoll.is_valid())
            {
                continue;
            }

            SkeletonComponent* skeleton_component = ragdoll.get_component_if_exists<SkeletonComponent>();

            ref<Skeleton> skeleton = skeleton_component->skeleton;
            if (skeleton == nullptr)
            {
                continue;
            }

            pose_sampler->sample_pose(physical_animation_component, skeleton_component, skeleton_component->get_current_pose(), dt);
        }
    }

    void PhysicalAnimationSystem::update_ragdolls(float dt)
    {
        using namespace animation;

        ZoneScopedN("PhysicalAnimationSystem::update_ragdolls");

        active_ragdolls.clear();

        for (auto&& [entity_handle,
                 transform_component,
                 physical_animation_component,
                 animation_component,
                 skeleton_component] : ragdolls_group.each())
        {
            if (!physical_animation_component.loaded)
            {
                continue;
            }

            const ref<Skeleton> skeleton = skeleton_component.skeleton;
            if (skeleton == nullptr)
            {
                ASSERT(false);
                continue;
            }

            if (animation_component.current_animation == nullptr)
            {
                continue;
            }

            Entity simulated_body = world->get_entity(entity_handle);

            const trs& world_transform = transform_component.get_world_transform();

            const vec3 root_delta_motion = world_transform.position - physical_animation_component.prev_world_transform.position;
            physical_animation_component.velocity = root_delta_motion / dt;

            physical_animation_component.prev_world_transform = world_transform;

            const bool is_should_be_simulated = should_be_simulated(simulated_body);

            if (is_should_be_simulated) // Update profiles only if simulated.
            {
                update_ragdoll_profiles(&physical_animation_component, physical_animation_component.velocity, dt, false);
            }

            physical_animation_component.update_drives(dt);

            // Update states.
            {
                if (force_ragdoll)
                {
                    physical_animation_component.force_set_ragdoll(true);
                    force_ragdoll = false;
                }

                if (force_disable_ragdoll)
                {
                    physical_animation_component.force_set_ragdoll(false);
                    force_disable_ragdoll = false;
                }

                SimulationStateType desired_state_type = SimulationStateType::DISABLED;
                if (physical_animation_component.is_in_ragdoll())
                {
                    desired_state_type = SimulationStateType::RAGDOLL;
                }
                else if (is_should_be_simulated)
                {
                    desired_state_type = SimulationStateType::ENABLED;
                }

                physical_animation_component.update_states(dt, desired_state_type);
            }

            const SimulationStateType current_state_type = physical_animation_component.get_current_state_type();
            const bool is_disabled = current_state_type == SimulationStateType::DISABLED;

            if (is_disabled)
            {
                continue;
            }

            active_ragdolls.emplace_back(&physical_animation_component);

            SkeletonPose target_animation_pose;
            if (!physical_animation_component.sample_animation_pose(dt, target_animation_pose))
            {
                target_animation_pose = skeleton_component.get_current_override_pose();
            }
            update_target_pose(&physical_animation_component, target_animation_pose, skeleton.get(), world_transform);

            update_chains_states(&physical_animation_component, dt);
        }
    }

    void PhysicalAnimationSystem::update_profile_data(float)
    {
        if (profiles_asset == nullptr)
        {
            return;
        }

        if (!ImGui::Begin("Ragdoll Profile"))
        {
            ImGui::End();
            return;
        }

        bool changed = false;

        ImGui::TextUnformatted(get_ragdoll_profiles_asset_path().c_str());
        if (ImGui::Button("Save"))
        {
            save_profiles_asset();
        }
        ImGui::SameLine();
        if (ImGui::Button("Reload"))
        {
            load_profiles_asset();
            changed = true;
        }

        const ImGuiTreeNodeFlags tree_node_flags = ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_SpanAvailWidth | ImGuiTreeNodeFlags_FramePadding;

        if (ImGui::TreeNodeEx("Profile selection", tree_node_flags))
        {
            RagdollProfileSelectionSettings& selection = profiles_asset->selection_settings;
            changed |= ImGui::DragFloat("Running speed (m/s)", &selection.running_speed, 0.05f, 0.0f, 20.0f);
            changed |= ImGui::DragFloat("Sprint speed (m/s)", &selection.sprint_speed, 0.05f, 0.0f, 20.0f);
            changed |= ImGui::DragFloat("Speed hysteresis (m/s)", &selection.hysteresis, 0.01f, 0.0f, 5.0f);
            changed |= ImGui::DragFloat("Transition time (s)", &selection.transition_time, 0.01f, 0.0f, 5.0f);

            ImGui::TreePop();
        }

        auto edit_limb_details = [&changed](PhysicalLimbDetails& limb_details, const char* limb_label)
        {
            if (!limb_details.motor_drive.has_value())
            {
                return;
            }

            if (!ImGui::TreeNode(limb_label))
            {
                return;
            }

            AngularDriveSettings& angular = limb_details.motor_drive->angular;
            changed |= ImGui::DragFloat("Angular frequency (Hz)", &angular.frequency, 0.05f, 0.0f, 60.0f);
            changed |= ImGui::DragFloat("Angular damping ratio", &angular.damping_ratio, 0.01f, 0.0f, 10.0f);
            changed |= ImGui::DragFloat("Angular max acceleration (rad/s^2)", &angular.max_acceleration, 1.0f, 0.0f, 2000.0f);

            LinearDriveSettings& linear = limb_details.motor_drive->linear;
            changed |= ImGui::DragFloat("Linear frequency (Hz)", &linear.frequency, 0.05f, 0.0f, 60.0f);
            changed |= ImGui::DragFloat("Linear damping ratio", &linear.damping_ratio, 0.01f, 0.0f, 10.0f);
            changed |= ImGui::DragFloat("Linear max acceleration (m/s^2)", &linear.max_acceleration, 0.1f, 0.0f, 200.0f);

            LimbStrengthDetails& strength = limb_details.strength_details;
            changed |= ImGui::DragFloat("Strength default", &strength.default_strength_coeff, 0.01f, 0.0f, 10.0f);
            changed |= ImGui::DragFloat("Strength soft", &strength.soft_strength_coeff, 0.01f, 0.0f, 10.0f);
            changed |= ImGui::DragFloat("Strength hard", &strength.hard_strength_coeff, 0.01f, 0.0f, 10.0f);

            bool has_contact_response = limb_details.contact_response.has_value();
            if (ImGui::Checkbox("Contact response", &has_contact_response))
            {
                limb_details.contact_response = has_contact_response ? std::optional<ContactResponseSettings>(ContactResponseSettings()) : std::nullopt;
                changed = true;
            }

            if (limb_details.contact_response.has_value())
            {
                ContactResponseSettings& contact = limb_details.contact_response.value();
                changed |= ImGui::DragFloat("Contact min strength", &contact.min_strength, 0.01f, 0.0f, 1.0f);
                changed |= ImGui::DragFloat("Contact touch strength", &contact.touch_strength, 0.01f, 0.0f, 1.0f);
                changed |= ImGui::DragFloat("Contact hit damping ratio", &contact.hit_damping_ratio, 0.01f, 0.0f, 10.0f);
                changed |= ImGui::DragFloat2("Contact velocity change range (m/s)", contact.velocity_change_range.data, 0.05f, 0.0f, 50.0f);
                changed |= ImGui::DragFloat("Contact recovery time (s)", &contact.contact_recovery_time, 0.01f, 0.0f, 10.0f);
                changed |= ImGui::DragFloat("Contact chain falloff", &contact.chain_falloff_coeff, 0.01f, 0.0f, 1.0f);
                changed |= ImGui::DragFloat("Contact parent follow", &contact.parent_follow_factor, 0.01f, 0.0f, 1.0f);
                changed |= ImGui::Checkbox("Contact respond to own hits", &contact.respond_to_own_hits);
            }

            bool has_drag_force = limb_details.drag_force.has_value();
            if (ImGui::Checkbox("Drag force", &has_drag_force))
            {
                limb_details.drag_force = has_drag_force ? std::optional<DragForceDetails>(DragForceDetails()) : std::nullopt;
                changed = true;
            }

            if (limb_details.drag_force.has_value())
            {
                DragForceDetails& drag = limb_details.drag_force.value();
                changed |= ImGui::DragFloat("Drag velocity drive", &drag.partial_velocity_drive, 0.01f, 0.0f, 1.0f);
                changed |= ImGui::DragFloat("Drag velocity limit (m/s)", &drag.partial_velocity_drive_limit, 0.1f, 0.0f, 100.0f);
                changed |= ImGui::DragFloat("Drag angular drive", &drag.partial_angular_drive, 0.01f, 0.0f, 1.0f);
                changed |= ImGui::DragFloat("Drag angular limit (rad/s)", &drag.partial_angular_drive_limit, 0.1f, 0.0f, 200.0f);
                changed |= ImGui::DragFloat("Drag root velocity limit (m/s)", &drag.acceleration_limit, 0.01f, 0.0f, 10.0f);
                changed |= ImGui::DragFloat("Drag root velocity gain", &drag.acceleration_gain, 0.001f, 0.0f, 1.0f);
            }

            ImGui::TreePop();
        };

        for (uint32 profile_index = 0; profile_index < RagdollProfilesAsset::PROFILES_COUNT; ++profile_index)
        {
            const ref<RagdollProfile>& profile = profiles_asset->profiles[profile_index];
            if (profile == nullptr)
            {
                continue;
            }

            if (ImGui::TreeNodeEx(ReflectionUtils::enum_to_string(static_cast<RagdollProfileType>(profile_index)).c_str(), tree_node_flags))
            {
                for (uint32 limb_index = 0; limb_index < RagdollProfile::LIMB_TYPES_COUNT; ++limb_index)
                {
                    edit_limb_details(profile->get_limb_details_by_type(static_cast<RagdollLimbType>(limb_index)), ReflectionUtils::enum_to_string(static_cast<RagdollLimbType>(limb_index)).c_str());
                }
                ImGui::TreePop();
            }
        }

        if (ImGui::TreeNodeEx("Compiled drives", tree_node_flags))
        {
            const ImGuiTableFlags table_flags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit;

            for (const PhysicalAnimationComponent* physical_animation_component : active_ragdolls)
            {
                ImGui::PushID(physical_animation_component);
                ImGui::Text("Ragdoll '%s', profile: %s", get_entity_name(physical_animation_component->get_entity()),
                    ReflectionUtils::enum_to_string(physical_animation_component->get_ragdoll_profile()->type).c_str());

                ImGui::TextUnformatted("Design");
                if (ImGui::BeginTable("##design", 13, table_flags))
                {
                    ImGui::TableSetupColumn("Limb");
                    ImGui::TableSetupColumn("Mass (kg)");
                    ImGui::TableSetupColumn("I_pivot/I_cm");
                    ImGui::TableSetupColumn("m/m_eff");
                    ImGui::TableSetupColumn("Ang f (Hz)");
                    ImGui::TableSetupColumn("Ang zeta");
                    ImGui::TableSetupColumn("K (N*m/rad)");
                    ImGui::TableSetupColumn("Max torque (N*m)");
                    ImGui::TableSetupColumn("Lin f (Hz)");
                    ImGui::TableSetupColumn("Lin zeta");
                    ImGui::TableSetupColumn("Max force (N)");
                    ImGui::TableSetupColumn("Contact strength");
                    ImGui::TableSetupColumn("Deviation (deg / m)");
                    ImGui::TableHeadersRow();

                    for (const EntityPtr& limb_ptr : physical_animation_component->limbs)
                    {
                        const PhysicalAnimationLimbComponent* limb_component = limb_ptr.get().get_component<PhysicalAnimationLimbComponent>();
                        const LimbMassProperties& mass_properties = limb_component->mass_properties;
                        const CompiledLimbDrive& drive = limb_component->drive;

                        ImGui::TableNextRow();
                        ImGui::TableNextColumn();
                        ImGui::TextUnformatted(get_entity_name(limb_ptr.get()));
                        ImGui::TableNextColumn();
                        ImGui::Text("%.2f", mass_properties.mass);
                        ImGui::TableNextColumn();
                        ImGui::Text("%.2f", mass_properties.get_inertia_ratio());
                        ImGui::TableNextColumn();
                        ImGui::Text("%.2f", mass_properties.get_linear_mass_ratio());
                        ImGui::TableNextColumn();
                        ImGui::Text("%.2f", drive.angular_frequency);
                        ImGui::TableNextColumn();
                        ImGui::Text("%.2f", drive.angular_damping_ratio);
                        ImGui::TableNextColumn();
                        ImGui::Text("%.1f", drive.torque_stiffness);
                        ImGui::TableNextColumn();
                        ImGui::Text("%.1f", drive.max_torque);
                        ImGui::TableNextColumn();
                        ImGui::Text("%.2f", drive.linear_frequency);
                        ImGui::TableNextColumn();
                        ImGui::Text("%.2f", drive.linear_damping_ratio);
                        ImGui::TableNextColumn();
                        ImGui::Text("%.1f", drive.max_force);
                        ImGui::TableNextColumn();
                        ImGui::Text("%.2f", limb_component->contact_strength);
                        ImGui::TableNextColumn();
                        ImGui::Text("%.1f / %.3f", rad2deg(limb_component->deviation.angle), limb_component->deviation.distance);
                    }

                    ImGui::EndTable();
                }

                ImGui::TextUnformatted("PhysX (compiled -> applied)");
                if (ImGui::BeginTable("##physx", 8, table_flags))
                {
                    ImGui::TableSetupColumn("Limb");
                    ImGui::TableSetupColumn("Drive");
                    ImGui::TableSetupColumn("Slerp k");
                    ImGui::TableSetupColumn("Slerp c");
                    ImGui::TableSetupColumn("Slerp limit");
                    ImGui::TableSetupColumn("Linear k");
                    ImGui::TableSetupColumn("Linear c");
                    ImGui::TableSetupColumn("Linear limit");
                    ImGui::TableHeadersRow();

                    for (const EntityPtr& limb_ptr : physical_animation_component->limbs)
                    {
                        const PhysicalAnimationLimbComponent* limb_component = limb_ptr.get().get_component<PhysicalAnimationLimbComponent>();
                        const CompiledLimbDrive& drive = limb_component->drive;

                        struct AppliedDrive
                        {
                            bool valid = false;
                            bool enabled = false;
                            bool slerp_config = false;
                            bool improved_slerp = false;
                            physx::PxD6JointDrive slerp;
                            physx::PxD6JointDrive linear;
                        } applied;

                        if (const JointComponent* joint_component = dynamic_cast<const JointComponent*>(limb_component->drive_joint_component.get()))
                        {
                            if (physx::PxJoint* native_joint = joint_component->get_native_joint())
                            {
                                PhysicsEngine::execute_read([&]()
                                    {
                                        using namespace physx;
                                        if (PxD6Joint* d6_joint = native_joint->is<PxD6Joint>())
                                        {
                                            applied.valid = true;
                                            applied.enabled = !d6_joint->getConstraintFlags().isSet(PxConstraintFlag::eDISABLE_CONSTRAINT);
                                            applied.slerp_config = d6_joint->getAngularDriveConfig() == PxD6AngularDriveConfig::eSLERP;
                                            applied.improved_slerp = d6_joint->getConstraintFlags().isSet(PxConstraintFlag::eIMPROVED_SLERP);
                                            if (applied.slerp_config)
                                            {
                                                applied.slerp = d6_joint->getDrive(PxD6Drive::eSLERP);
                                            }
                                            applied.linear = d6_joint->getDrive(PxD6Drive::eX);
                                        }
                                    });
                            }
                        }

                        auto accel_mark = [](const physx::PxD6JointDrive& physx_drive)
                        {
                            return (physx_drive.flags & physx::PxD6JointDriveFlag::eACCELERATION) ? "a" : "F!";
                        };

                        ImGui::TableNextRow();
                        ImGui::TableNextColumn();
                        ImGui::TextUnformatted(get_entity_name(limb_ptr.get()));

                        ImGui::TableNextColumn();
                        if (!applied.valid)
                        {
                            ImGui::TextUnformatted("no joint");
                        }
                        else
                        {
                            ImGui::Text("%s%s%s", applied.enabled ? "on" : "off",
                                applied.slerp_config ? "" : " !slerp",
                                applied.improved_slerp ? " improved" : "");
                        }

                        ImGui::TableNextColumn();
                        ImGui::Text("%.1f -> %.1f %s", drive.angular.stiffness, applied.slerp.stiffness, accel_mark(applied.slerp));
                        ImGui::TableNextColumn();
                        ImGui::Text("%.1f -> %.1f", drive.angular.damping, applied.slerp.damping);
                        ImGui::TableNextColumn();
                        ImGui::Text("%.1f -> %.1f", drive.angular.force_limit, applied.slerp.forceLimit);
                        ImGui::TableNextColumn();
                        ImGui::Text("%.1f -> %.1f %s", drive.linear.stiffness, applied.linear.stiffness, accel_mark(applied.linear));
                        ImGui::TableNextColumn();
                        ImGui::Text("%.1f -> %.1f", drive.linear.damping, applied.linear.damping);
                        ImGui::TableNextColumn();
                        ImGui::Text("%.1f -> %.1f", drive.linear.force_limit, applied.linear.forceLimit);
                    }

                    ImGui::EndTable();
                }

                ImGui::TextUnformatted("Parent joints (now / limit, deg)");
                if (ImGui::BeginTable("##joint_limits", 6, table_flags))
                {
                    ImGui::TableSetupColumn("Limb");
                    ImGui::TableSetupColumn("Twist");
                    ImGui::TableSetupColumn("Swing Y");
                    ImGui::TableSetupColumn("Swing Z");
                    ImGui::TableSetupColumn("Cone");
                    ImGui::TableSetupColumn("Anim gap (cm)");
                    ImGui::TableHeadersRow();

                    const ImVec4 limit_color = ImVec4(1.0f, 0.4f, 0.4f, 1.0f);
                    const ImVec4 text_color = ImGui::GetStyleColorVec4(ImGuiCol_Text);

                    auto limit_cell = [&](float angle, D6JointComponent::Motion motion, float min_limit, float max_limit)
                    {
                        ImGui::TableNextColumn();
                        if (motion == D6JointComponent::Motion::FREE)
                        {
                            ImGui::Text("%.1f / free", rad2deg(angle));
                            return;
                        }
                        if (motion == D6JointComponent::Motion::LOCKED)
                        {
                            ImGui::Text("%.1f / locked", rad2deg(angle));
                            return;
                        }

                        const bool at_limit = angle <= min_limit * 0.95f || angle >= max_limit * 0.95f;
                        ImGui::TextColored(at_limit ? limit_color : text_color, "%.1f / [%.1f, %.1f]", rad2deg(angle), rad2deg(min_limit), rad2deg(max_limit));
                    };

                    for (const EntityPtr& limb_ptr : physical_animation_component->limbs)
                    {
                        const PhysicalAnimationLimbComponent* limb_component = limb_ptr.get().get_component<PhysicalAnimationLimbComponent>();
                        const D6JointComponent* parent_joint = dynamic_cast<const D6JointComponent*>(limb_component->parent_joint_component.get());

                        ImGui::TableNextRow();
                        ImGui::TableNextColumn();
                        ImGui::TextUnformatted(get_entity_name(limb_ptr.get()));

                        physx::PxJoint* native_joint = parent_joint != nullptr ? parent_joint->get_native_joint() : nullptr;
                        if (native_joint == nullptr)
                        {
                            continue;
                        }

                        float twist = 0.0f;
                        float swing_y = 0.0f;
                        float swing_z = 0.0f;
                        PhysicsEngine::execute_read([&]()
                            {
                                if (const physx::PxD6Joint* d6_joint = native_joint->is<physx::PxD6Joint>())
                                {
                                    // getTwistAngle does not pick the shortest rotation like the solver does, the result is in (-2pi, 2pi).
                                    twist = d6_joint->getTwistAngle();
                                    if (twist > M_PI)
                                    {
                                        twist -= 2.0f * M_PI;
                                    }
                                    else if (twist < -M_PI)
                                    {
                                        twist += 2.0f * M_PI;
                                    }
                                    swing_y = d6_joint->getSwingYAngle();
                                    swing_z = d6_joint->getSwingZAngle();
                                }
                            });

                        const float swing_y_limit = parent_joint->swing_y_limit.get();
                        const float swing_z_limit = parent_joint->swing_z_limit.get();

                        limit_cell(twist, parent_joint->twist_motion_type.get(), parent_joint->twist_min_limit.get(), parent_joint->twist_max_limit.get());
                        limit_cell(swing_y, parent_joint->swing_y_motion_type.get(), -swing_y_limit, swing_y_limit);
                        limit_cell(swing_z, parent_joint->swing_z_motion_type.get(), -swing_z_limit, swing_z_limit);

                        ImGui::TableNextColumn();
                        if (parent_joint->swing_y_motion_type.get() == D6JointComponent::Motion::LIMITED &&
                            parent_joint->swing_z_motion_type.get() == D6JointComponent::Motion::LIMITED &&
                            swing_y_limit > 0.0f && swing_z_limit > 0.0f)
                        {
                            const float y = tan(0.25f * swing_y) / tan(0.25f * swing_y_limit);
                            const float z = tan(0.25f * swing_z) / tan(0.25f * swing_z_limit);
                            const float cone = y * y + z * z;
                            ImGui::TextColored(cone >= 0.9f ? limit_color : text_color, "%.2f", cone);
                        }
                        else
                        {
                            ImGui::TextUnformatted("-");
                        }

                        // The rigid ragdoll keeps the child pivot at the joint anchor of the parent: a gap means the animation
                        // moves the child bone relative to its parent, and the drives cannot match both poses.
                        ImGui::TableNextColumn();
                        const PhysicalAnimationLimbComponent* parent_limb = parent_joint->get_first_entity_ptr().get().get_component_if_exists<PhysicalAnimationLimbComponent>();
                        if (parent_limb != nullptr)
                        {
                            const vec3 anchor = transform_position(parent_limb->animation_pose, parent_joint->get_first_local_frame().position);
                            const float gap = length(anchor - limb_component->animation_pose.position) * 100.0f;
                            ImGui::TextColored(gap >= 1.0f ? limit_color : text_color, "%.1f", gap);
                        }
                    }

                    ImGui::EndTable();
                }

                ImGui::PopID();
            }
            ImGui::TreePop();
        }

        if (changed)
        {
            ++profiles_asset->revision;
        }

        ImGui::End();
    }

    static PhysicalAnimationLimbComponent* get_active_limb(Entity entity)
    {
        PhysicalAnimationLimbComponent* limb_component = entity.get_component_if_exists<PhysicalAnimationLimbComponent>();
        if (limb_component == nullptr)
        {
            return nullptr;
        }

        const PhysicalAnimationComponent* physical_animation_component = limb_component->ragdoll_ptr.get().get_component<PhysicalAnimationComponent>();
        ASSERT(physical_animation_component != nullptr);

        const SimulationStateType state = physical_animation_component->get_current_state_type();
        return state == SimulationStateType::ENABLED || state == SimulationStateType::RAGDOLL ? limb_component : nullptr;
    }

    static bool is_same_ragdoll_pair(Entity first_entity, Entity second_entity)
    {
        return first_entity == second_entity || first_entity.get_parent_handle() == second_entity.get_parent_handle();
    }

    void PhysicalAnimationSystem::filter_states_by_collisions(float dt) const
    {
        ref<SimulationEventCallback> event_callback = PhysicsEngine::get_physics_core()->simulation_event_callback;
        for (const Collision& collision : event_callback->new_collisions)
        {
            Entity first_entity = collision.this_actor->get_entity();
            Entity second_entity = collision.other_actor->get_entity();

            if (is_same_ragdoll_pair(first_entity, second_entity))
            {
                continue;
            }

            for (Entity entity : {first_entity, second_entity})
            {
                if (PhysicalAnimationLimbComponent* limb_component = get_active_limb(entity))
                {
                    limb_component->collision.is_colliding = true;
                    limb_component->collision.impulse += physx::create_vec3(collision.impulse);
                }
            }
        }

        for (const TouchingPair& touching_pair : event_callback->touching_pairs)
        {
            Entity first_entity = touching_pair.first_actor->get_entity();
            Entity second_entity = touching_pair.second_actor->get_entity();

            if (is_same_ragdoll_pair(first_entity, second_entity))
            {
                continue;
            }

            auto mark_touching = [](Entity limb_entity, bool other_dynamic)
            {
                PhysicalAnimationLimbComponent* limb_component = other_dynamic ? get_active_limb(limb_entity) : nullptr;
                if (limb_component != nullptr)
                {
                    limb_component->collision.is_colliding = true;
                    limb_component->collision.touching_dynamic = true;
                }
            };

            mark_touching(first_entity, touching_pair.second_dynamic);
            mark_touching(second_entity, touching_pair.first_dynamic);
        }
    }

    void PhysicalAnimationSystem::update_chains_states(const PhysicalAnimationComponent* physical_animation_component, float dt) const
    {
        bool has_any_up_collided_limbs = force_simulate;
        if (!has_any_up_collided_limbs)
        {
            has_any_up_collided_limbs |= physical_animation_component->body_chain->has_any_colliding_limb();
        }
        if (!has_any_up_collided_limbs)
        {
            has_any_up_collided_limbs |= physical_animation_component->neck_chain->has_any_colliding_limb();
        }
        if (!has_any_up_collided_limbs)
        {
            has_any_up_collided_limbs |= physical_animation_component->left_arm_chain->has_any_colliding_limb();
        }
        if (!has_any_up_collided_limbs)
        {
            has_any_up_collided_limbs |= physical_animation_component->right_arm_chain->has_any_colliding_limb();
        }
        if (!has_any_up_collided_limbs)
        {
            has_any_up_collided_limbs |= physical_animation_component->left_leg_chain->has_any_colliding_limb();
        }
        if (!has_any_up_collided_limbs)
        {
            has_any_up_collided_limbs |= physical_animation_component->right_leg_chain->has_any_colliding_limb();
        }

        const bool is_in_ragdoll = physical_animation_component->is_in_ragdoll();

        update_chain(physical_animation_component->body_chain, dt, is_in_ragdoll, has_any_up_collided_limbs);
        update_chain(physical_animation_component->neck_chain, dt, is_in_ragdoll, has_any_up_collided_limbs);
        update_chain(physical_animation_component->left_arm_chain, dt, is_in_ragdoll, has_any_up_collided_limbs);
        update_chain(physical_animation_component->right_arm_chain, dt, is_in_ragdoll, has_any_up_collided_limbs);

        update_chain(physical_animation_component->left_leg_chain, dt, is_in_ragdoll, has_any_up_collided_limbs);
        update_chain(physical_animation_component->right_leg_chain, dt, is_in_ragdoll, has_any_up_collided_limbs);
    }

    void PhysicalAnimationSystem::update_chain(const ref<PhysicsLimbChain>& chain,
        float dt,
        bool is_in_ragdoll,
        bool force_simulation /* = false*/) const
    {
        for (EntityPtr limb_ptr : chain->connected_limbs)
        {
            Entity limb = limb_ptr.get();

            PhysicalAnimationLimbComponent* limb_data_component = limb.get_component<PhysicalAnimationLimbComponent>();
            ASSERT(limb_data_component != nullptr);

            PhysicalLimbStateType desired_chain_state = PhysicalLimbStateType::KINEMATIC;

            if (limb_data_component->current_state_type == PhysicalLimbStateType::DISMEMBERED ||
                limb_data_component->current_state_type == PhysicalLimbStateType::PARTIAL_DISMEMBERED ||
                limb_data_component->current_state_type == PhysicalLimbStateType::USER_STATE)
            {
                desired_chain_state = limb_data_component->current_state_type;
            }
            else if (is_in_ragdoll)
            {
                desired_chain_state = PhysicalLimbStateType::RAGDOLL;
            }
            else if (force_simulation)
            {
                desired_chain_state = PhysicalLimbStateType::SIMULATION;
            }

            limb_data_component->update_states(dt, desired_chain_state);

            if (limb_data_component->collision.was_in_collision && limb_data_component->collision.is_colliding)
            {
                const float used_collision_time = PhysicalAnimationUtils::is_arm_limb(limb_data_component->type) ? PhysicalAnimationLimbComponent::CollisionData::MAX_FREQUENT_COLLISION_TIME : PhysicalAnimationLimbComponent::CollisionData::MAX_COLLISION_TIME;
                limb_data_component->collision.collision_time = clamp(limb_data_component->collision.collision_time + dt, 0.0f, used_collision_time);
            }
            else
            {
                limb_data_component->collision.collision_time = max(limb_data_component->collision.collision_time - dt, 0.0f);
            }
            limb_data_component->collision.was_in_collision = limb_data_component->collision.is_colliding;
        }
    }

    void PhysicalAnimationSystem::update_target_pose(PhysicalAnimationComponent* physical_animation_component,
        const animation::SkeletonPose& current_animation_pose,
        const animation::Skeleton* skeleton,
        const trs& ragdoll_world_space_pose)
    {
        if (!current_animation_pose.is_valid())
        {
            return;
        }

        auto simulated_joints_end = physical_animation_component->simulated_joints.end();

        std::unordered_map<uint32, trs>& target_joint_poses = physical_animation_component->local_joint_poses_for_target_calculation;
        std::unordered_map<uint32, trs>& physics_follow_joint_poses = physical_animation_component->physics_follow_joint_poses;
        std::unordered_map<uint32, trs>& animation_joint_poses = physical_animation_component->animation_joint_poses;

        const trs inverse_ragdoll_world_space_pose = invert(ragdoll_world_space_pose);
        const ref<RagdollProfile> profile = physical_animation_component->get_ragdoll_profile();

        for (const uint32 simulation_joint : physical_animation_component->simulated_joints_set)
        {
            if (simulation_joint == 0)
            {
                continue;
            }

            const uint32 parent_id = skeleton->joints.at(simulation_joint).parent_id;

            trs limb_animation_pose = current_animation_pose.get_joint_transform(simulation_joint).get_transform();
            limb_animation_pose.scale = vec3(1.0f);

            animation_joint_poses[simulation_joint] = animation_joint_poses[parent_id] * limb_animation_pose;
            trs target_pose = target_joint_poses[parent_id] * limb_animation_pose;
            trs physics_follow_pose = physics_follow_joint_poses[parent_id] * limb_animation_pose;

            auto limb_iter = physical_animation_component->simulated_joints.find(simulation_joint);
            if (limb_iter == simulated_joints_end)
            {
                target_joint_poses[simulation_joint] = target_pose;
                physics_follow_joint_poses[simulation_joint] = physics_follow_pose;
                continue;
            }

            Entity limb = limb_iter->second.get();

            PhysicalAnimationLimbComponent* limb_component = limb.get_component<PhysicalAnimationLimbComponent>();
            const PhysicalLimbStateType current_state = limb_component->get_current_state_type();

            const std::optional<ContactResponseSettings>& contact_response = profile->get_limb_details_by_type(limb_component->type).contact_response;
            if (contact_response.has_value())
            {
                target_pose = blend_rigid_pose(target_pose, physics_follow_pose, contact_response->parent_follow_factor);
            }

            physics_follow_pose = target_pose;

            // Children follow the physics pose as much as the drive is weakened.
            const float strength_loss_coeff = contact_response.has_value() ? clamp(1.0f - limb_component->contact_strength, 0.0f, 1.0f) : 0.0f;
            if (strength_loss_coeff > 0.0f && (current_state == PhysicalLimbStateType::SIMULATION || current_state == PhysicalLimbStateType::BLEND_OUT))
            {
                const trs physics_pose = inverse_ragdoll_world_space_pose * PhysicsUtils::get_actor_world_pose_locked(limb);
                physics_follow_pose = blend_rigid_pose(target_pose, physics_pose, strength_loss_coeff);
            }

            target_joint_poses[simulation_joint] = target_pose;
            physics_follow_joint_poses[simulation_joint] = physics_follow_pose;

            limb_component->target_pose = target_pose;
            limb_component->animation_pose = animation_joint_poses[simulation_joint];

            if (limb_component->drive_joint_component.get() == nullptr)
            {
                continue;
            }

            const trs target_world_pose = ragdoll_world_space_pose * limb_component->target_pose;
            const trs target_rigid_pose = trs(target_world_pose.position, target_world_pose.rotation, vec3(1.0f));

            D6JointComponent* drive_joint = dynamic_cast<D6JointComponent*>(limb_component->drive_joint_component.get_for_write());
            Entity kinematic_target = drive_joint->get_first_entity_ptr().get();

            if (drive_joint->disabled || current_state == PhysicalLimbStateType::RAGDOLL)
            {
                drive_joint->drive_transform.get_for_write() = trs::identity;
            }
            else
            {
                // The kinematic target reaches its new pose only after the solver. TGS keeps it at the old pose and sees its
                // velocity only inside each substep, which covers about half of the step, so the drive aims half way.
                const trs kinematic_world_pose = PhysicsUtils::get_actor_world_pose_locked(kinematic_target);
                const trs kinematic_rigid_pose = trs(kinematic_world_pose.position, kinematic_world_pose.rotation, vec3(1.0f));

                trs drive_transform = blend_rigid_pose(trs::identity, invert(kinematic_rigid_pose) * target_rigid_pose, DRIVE_POSE_STEP_COEFF);
                drive_transform.rotation = normalize(drive_transform.rotation);
                drive_transform.scale = vec3(1.0f);
                drive_joint->drive_transform.get_for_write() = drive_transform;
            }

            PhysicsUtils::manual_set_physics_transform_locked(kinematic_target, target_world_pose, true);
        }

        if (!physical_animation_component->attachment_body.is_empty())
        {
            const trs attachment_world_pose = ragdoll_world_space_pose * physical_animation_component->local_joint_poses_for_target_calculation[physical_animation_component->root_joint_id];
            PhysicsUtils::manual_set_physics_transform_locked(physical_animation_component->attachment_body.get(), attachment_world_pose, true);
        }
    }

    static RagdollProfileType select_locomotion_profile_type(RagdollProfileType current_type, float speed, const RagdollProfileSelectionSettings& settings)
    {
        const float half_hysteresis = 0.5f * std::max(settings.hysteresis, 0.0f);
        const float thresholds[2] = {settings.running_speed, settings.sprint_speed};

        int32 level = 0;
        if (current_type == RagdollProfileType::RUNNING)
        {
            level = 1;
        }
        else if (current_type == RagdollProfileType::SPRINT)
        {
            level = 2;
        }

        while (level < 2 && speed > thresholds[level] + half_hysteresis)
        {
            ++level;
        }

        while (level > 0 && speed < thresholds[level - 1] - half_hysteresis)
        {
            --level;
        }

        return level == 2 ? RagdollProfileType::SPRINT : (level == 1 ? RagdollProfileType::RUNNING : RagdollProfileType::IDLE);
    }

    void PhysicalAnimationSystem::update_ragdoll_profiles(PhysicalAnimationComponent* physical_animation_component,
        const vec3& velocity,
        float dt,
        bool force_reload) const
    {
        const RagdollProfilesAsset* asset = get_profiles_asset(physical_animation_component);
        if (asset == nullptr)
        {
            return;
        }

        const ref<RagdollProfile>& current_profile = physical_animation_component->get_ragdoll_profile();
        const RagdollProfileType current_type = current_profile != nullptr ? current_profile->type : RagdollProfileType::IDLE;

        const RagdollProfileType desired_type = force_drag_profile.get()
                                                    ? RagdollProfileType::DRAG
                                                    : select_locomotion_profile_type(current_type, length(velocity), asset->selection_settings);

        ref<RagdollProfile> desired_profile = asset->get_profile(desired_type);
        if (desired_profile == nullptr)
        {
            desired_profile = asset->get_profile(RagdollProfileType::IDLE);
        }

        bool force_sync = false;

        if (force_sync_profile.get())
        {
            force_sync_profile = false;
            force_sync = true;
        }

        if (desired_profile != nullptr)
        {
            physical_animation_component->try_to_apply_ragdoll_profile(desired_profile, physical_animation_component->get_strength_type(), force_reload || force_sync);
        }
        else
        {
            ASSERT(false);
        }
    }

    bool PhysicalAnimationSystem::should_be_simulated(Entity ragdoll) const
    {
        return enable_always;
    }

    void PhysicalAnimationSystem::process_added_pacs()
    {
        using namespace physx;
        using namespace animation;

        ScopedSpinLock _lock{sync};

        for (auto iter = pacs_to_init.begin(); iter != pacs_to_init.end();)
        {
            Entity::Handle entity_handle = *iter;
            Entity entity = world->get_entity(entity_handle);

            const SkeletonComponent* skeleton_component = entity.get_component<SkeletonComponent>();

            const ref<Skeleton> skeleton = skeleton_component->skeleton;
            if (skeleton == nullptr)
            {
                ++iter;
                continue;
            }

            PhysicalAnimationComponent* physical_animation_component = entity.get_component<PhysicalAnimationComponent>();
            if (physical_animation_component->profiles_asset == nullptr)
            {
                physical_animation_component->profiles_asset = profiles_asset;
            }

            const ref<RagdollProfile> idle_profile = physical_animation_component->profiles_asset->get_profile(RagdollProfileType::IDLE);
            ASSERT(idle_profile != nullptr);

            physical_animation_component->current_profile = idle_profile;

            PhysicalRagdollBuilder::RuntimeContext context;
            context.world = world;
            context.ragdoll = entity;
            context.ragdoll_component = physical_animation_component;
            context.default_profile = idle_profile.get();
            context.enable_physical_animation = true;

            PhysicalRagdollBuilder::build_ragdoll(context);

            auto set_chain_side = [](const ref<PhysicsLimbChain>& chain, RagdollLimbSide side)
            {
                for (const EntityPtr& limb_ptr : chain->connected_limbs)
                {
                    limb_ptr.get().get_component<PhysicalAnimationLimbComponent>()->side = side;
                }
            };
            set_chain_side(physical_animation_component->left_arm_chain, RagdollLimbSide::LEFT);
            set_chain_side(physical_animation_component->left_leg_chain, RagdollLimbSide::LEFT);
            set_chain_side(physical_animation_component->right_arm_chain, RagdollLimbSide::RIGHT);
            set_chain_side(physical_animation_component->right_leg_chain, RagdollLimbSide::RIGHT);

            for (const EntityPtr& limb_ptr : physical_animation_component->limbs)
            {
                const PhysicalAnimationLimbComponent* limb_component = limb_ptr.get().get_component<PhysicalAnimationLimbComponent>();
                physical_animation_component->simulated_joints.emplace(limb_component->joint_id, limb_ptr);
            }

            physical_animation_component->root_joint_id = physical_animation_component->joint_init_ids.attachment_idx;

            for (auto& [joint_id, limb_ptr] : physical_animation_component->simulated_joints)
            {
                uint32 calculation_id = joint_id;
                physical_animation_component->simulated_joints_set.emplace(calculation_id);

                while (calculation_id != INVALID_JOINT)
                {
                    const uint32 parent_id = skeleton->joints.at(calculation_id).parent_id;
                    if (parent_id != INVALID_JOINT)
                    {
                        physical_animation_component->simulated_joints_set.emplace(parent_id);
                        calculation_id = parent_id;
                    }
                    else
                    {
                        break;
                    }
                }
            }

            const uint32 simulation_graph_size = static_cast<uint32>(physical_animation_component->simulated_joints_set.size() + 1u);

            physical_animation_component->local_joint_poses.reserve(simulation_graph_size);
            physical_animation_component->local_joint_poses_for_target_calculation.reserve(simulation_graph_size);

            physical_animation_component->local_joint_poses[INVALID_JOINT] = trs::identity;
            physical_animation_component->local_joint_poses_for_target_calculation[INVALID_JOINT] = trs::identity;
            physical_animation_component->local_joint_poses[0] = trs::identity;
            physical_animation_component->local_joint_poses_for_target_calculation[0] = trs::identity;

            for (const uint32 root_id : {static_cast<uint32>(INVALID_JOINT), 0u})
            {
                physical_animation_component->physics_follow_joint_poses[root_id] = trs::identity;
                physical_animation_component->animation_joint_poses[root_id] = trs::identity;
            }

            physical_animation_component->try_to_apply_ragdoll_profile(idle_profile, RagdollProfileStrengthType::DEFAULT, true);

            iter = pacs_to_init.erase(iter);
        }
    }

    void PhysicalAnimationSystem::load_profiles_asset()
    {
        const std::string path = get_ragdoll_profiles_asset_path();

        if (fs::exists(path))
        {
            GameAssetsProvider provider;
            ref<RagdollProfilesAsset> loaded_asset = provider.load_game_asset_from_file<RagdollProfilesAsset>(path, false);
            loaded_asset->load_job.wait_for_completion();

            if (loaded_asset->load_state == AssetLoadState::LOADED && loaded_asset->get_profile(RagdollProfileType::IDLE) != nullptr)
            {
                const ref<RagdollProfilesAsset> default_asset = RagdollProfilesAsset::create_default();
                for (uint32 profile_index = 0; profile_index < RagdollProfilesAsset::PROFILES_COUNT; ++profile_index)
                {
                    if (loaded_asset->profiles[profile_index] == nullptr)
                    {
                        loaded_asset->profiles[profile_index] = default_asset->profiles[profile_index];
                    }
                }

                if (profiles_asset == nullptr)
                {
                    profiles_asset = loaded_asset;
                }
                else
                {
                    const uint32 revision = profiles_asset->revision;
                    profiles_asset->profiles = loaded_asset->profiles;
                    profiles_asset->selection_settings = loaded_asset->selection_settings;
                    profiles_asset->revision = revision + 1;
                }
                return;
            }

            LOG_ERROR("Physical animation> Failed to load ragdoll profiles '%s', using defaults.", path.c_str());
        }

        if (profiles_asset == nullptr)
        {
            profiles_asset = RagdollProfilesAsset::create_default();
        }

        if (!fs::exists(path))
        {
            save_profiles_asset();
            LOG_MESSAGE("Physical animation> Default ragdoll profiles were written to '%s'.", path.c_str());
        }
    }

    bool PhysicalAnimationSystem::save_profiles_asset() const
    {
        if (profiles_asset == nullptr)
        {
            return false;
        }

        GameAssetsProvider provider;
        JobHandle save_job = provider.save_game_asset_to_file_async<RagdollProfilesAsset>(get_ragdoll_profiles_asset_path(), profiles_asset.get(), {}, false);
        save_job.wait_for_completion();
        return true;
    }

    const RagdollProfilesAsset* PhysicalAnimationSystem::get_profiles_asset(const PhysicalAnimationComponent* physical_animation_component) const
    {
        if (physical_animation_component->profiles_asset != nullptr)
        {
            return physical_animation_component->profiles_asset.get();
        }
        return profiles_asset.get();
    }

    void PhysicalAnimationSystem::on_pac_created(entt::registry& registry, entt::entity entity_handle)
    {
        ScopedSpinLock _lock{sync};

        pacs_to_init.push_back(static_cast<Entity::Handle>(entity_handle));
    }

    void PhysicalAnimationSystem::on_pac_removed(entt::registry& registry, entt::entity entity_handle)
    {
        Entity entity = world->get_entity(entity_handle);
        PhysicalAnimationComponent* physical_animation_component = entity.get_component<PhysicalAnimationComponent>();

        if (!physical_animation_component->loaded)
        {
            return;
        }

        for (const EntityPtr& limb_ptr : physical_animation_component->limbs)
        {
            Entity limb = limb_ptr.get();
            if (!limb.is_valid())
            {
                continue;
            }

            PhysicalAnimationLimbComponent* limb_component = limb.get_component_if_exists<PhysicalAnimationLimbComponent>();
            if (limb_component == nullptr)
            {
                continue;
            }

            if (!limb_component->drive_joint_entity_ptr.is_empty())
            {
                Entity drive_joint_entity = limb_component->drive_joint_entity_ptr.get();

                if (D6JointComponent* joint_component = drive_joint_entity.get_component_if_exists<D6JointComponent>())
                {
                    world->destroy_entity(joint_component->get_first_entity_ptr().get());
                }

                world->destroy_entity(drive_joint_entity);
            }

            if (!limb_component->joint_entity_ptr.is_empty())
            {
                world->destroy_entity(limb_component->joint_entity_ptr.get());
            }

            world->destroy_entity(limb);
        }

        if (!physical_animation_component->attachment_body.is_empty())
        {
            world->destroy_entity(physical_animation_component->attachment_body.get());
        }
    }

    LimbDeviation PhysicalAnimationSystem::compute_limb_deviation(const PhysicalAnimationLimbComponent* limb_component, const trs& limb_world_pose) const
    {
        const D6JointComponent* drive_joint = dynamic_cast<const D6JointComponent*>(limb_component->drive_joint_component.get());
        const PhysicalLimbStateType state = limb_component->get_current_state_type();
        if (drive_joint == nullptr ||
            state == PhysicalLimbStateType::DISMEMBERED ||
            state == PhysicalLimbStateType::PARTIAL_DISMEMBERED)
        {
            return LimbDeviation{};
        }

        const trs target_world_pose = PhysicsUtils::get_actor_world_pose_locked(drive_joint->get_first_entity_ptr().get());
        const trs animation_frame = target_world_pose * invert(limb_component->target_pose) * limb_component->animation_pose * drive_joint->get_first_local_frame();
        const trs limb_frame = limb_world_pose * drive_joint->get_second_local_frame();
        const trs error = invert(limb_frame) * animation_frame;

        const CompiledLimbDrive& drive = limb_component->drive;

        LimbDeviation deviation;
        deviation.angle = get_shortest_angle(normalize(error.rotation));
        deviation.distance = length(error.position);
        deviation.normalized_angle = drive.saturation_angle > 0.0f ? deviation.angle / drive.saturation_angle : 0.0f;
        deviation.normalized_distance = drive.saturation_distance > 0.0f ? deviation.distance / drive.saturation_distance : 0.0f;
        return deviation;
    }
}
