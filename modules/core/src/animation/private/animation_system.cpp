#include "animation/animation_system.h"
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

		for (auto [entity_handle, animation_component, mesh_component, skeleton_component, transform_component] :
			world->group(components_group<AnimationComponent, MeshComponent, SkeletonComponent, TransformComponent>).each())
		{
			if (animation_component.current_animation != nullptr &&
				animation_component.current_animation->is_valid() &&
				animation_component.play)
			{
				const float anim_duration = animation_component.current_animation->get_duration();

				if (animation_component.current_anim_position < anim_duration)
				{
					ref<Skeleton> skeleton = skeleton_component.skeleton;

					AnimationPoseSampler sampler;
					sampler.init(skeleton.get(), animation_component.current_animation);

					SkeletonPose result_pose = SkeletonPose(skeleton->joints.size());
					sampler.sample_pose(animation_component.current_anim_position, result_pose);
					if (result_pose.is_valid())
					{
						if (animation_component.inertial_blend_inited)
						{
							animation_component.inertial_sampler->process_pose(animation_component.current_anim_position, &result_pose);
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
			}
		}
	}

	void AnimationSystem::update_mesh(float dt)
	{
		ZoneScopedN("AnimationSystem::update_mesh");

		MemoryMarker marker = allocator->get_marker();
		for (auto [entity_handle, animation_component, mesh_component, skeleton_component, transform_component] :
			world->group(components_group<AnimationComponent, MeshComponent, SkeletonComponent, TransformComponent>).each())
		{
			const dx_mesh& dxMesh = mesh_component.mesh->mesh;
			ref<Skeleton> skeleton = skeleton_component.skeleton;

			animation_component.current_global_transforms = nullptr;

			auto [vb, skinning_matrices] = skinObject(dxMesh.vertexBuffer, dxMesh.vertexBuffer.positions->elementCount, (uint32)skeleton->joints.size());

			animation_component.prev_frame_vertex_buffer = animation_component.current_vertex_buffer;
			animation_component.current_vertex_buffer = vb;

			trs* global_transforms = allocator->allocate<trs>((uint32)skeleton->joints.size());

			skeleton_component.get_skinning_matrices_from_local_transforms(global_transforms, skinning_matrices, trs::identity);

			animation_component.current_global_transforms = global_transforms;
		}
		allocator->reset_to_marker(marker);
	}
}