#include "animation/skeleton_system.h"
#include "animation/skeleton.h"
#include "animation/skeleton_component.h"
#include "animation/skinning.h"

#include "rendering/ecs/renderer_holder_root_component.h"
#include "rendering/debug_visualization.h"

#include "core/cpu_profiling.h"
#include "core/memory.h"

#include "engine/engine.h"

#include "ecs/update_groups.h"
#include "ecs/rendering/mesh_component.h"
#include "ecs/base_components/transform_component.h"

#include "geometry/mesh.h"

#include "dx/dx_context.h"

#include <rttr/policy.h>
#include <rttr/registration>

namespace era_engine::animation
{
	RTTR_REGISTRATION
	{
		using namespace rttr;

		registration::class_<SkeletonSystem>("SkeletonSystem")
			.constructor<World*>()(policy::ctor::as_raw_ptr, metadata("Tag", std::string("render")))
			.method("update", &SkeletonSystem::update)(metadata("update_group", update_types::GAMEPLAY_NORMAL))
			.method("draw_skeletons", &SkeletonSystem::draw_skeletons)(metadata("update_group", update_types::RENDER));
	}

	const vec3 limb_type_colors[] =
	{
		vec3(1.f, 0.f, 1.f),

		vec3(1.f, 0.f, 0.f),
		vec3(0.f, 1.f, 0.f),

		vec3(1.f, 1.f, 0.f),
		vec3(0.f, 1.f, 1.f),
		vec3(1.f, 1.f, 1.f),

		vec3(1.f, 1.f, 0.f),
		vec3(0.f, 1.f, 1.f),
		vec3(1.f, 1.f, 1.f),

		vec3(1.f, 1.f, 0.f),
		vec3(0.f, 1.f, 1.f),
		vec3(1.f, 1.f, 1.f),

		vec3(1.f, 1.f, 0.f),
		vec3(0.f, 1.f, 1.f),
		vec3(1.f, 1.f, 1.f),
	};

	static void draw_skeleton(const Skeleton& skeleton, const SkeletonPose& pose, const trs& transform, ldr_render_pass* render_pass)
	{
		const uint32 num_joints = (uint32)skeleton.joints.size();

		auto [vb, vertexPtr] = dxContext.createDynamicVertexBuffer(sizeof(position_color), num_joints * 2);
		auto [ib, indexPtr] = dxContext.createDynamicIndexBuffer(sizeof(uint16), num_joints * 2);

		position_color* vertices = (position_color*)vertexPtr;
		indexed_line16* lines = (indexed_line16*)indexPtr;

		for (uint32 i = 0; i < num_joints; ++i)
		{
			const auto& joint = skeleton.joints[i];
			if (joint.parent_id != INVALID_JOINT && !joint.ik)
			{
				const auto& parent_joint = skeleton.joints[joint.parent_id];

				const trs parent_transform = SkeletonUtils::get_object_space_joint_transform(pose, &skeleton, joint.parent_id);
				const trs current_transform = parent_transform * pose.get_joint_transform(i).get_transform();

				*vertices++ = { parent_transform.position, limb_type_colors[parent_joint.limb_type] };
				*vertices++ = { current_transform.position, limb_type_colors[parent_joint.limb_type] };
			}
			else
			{
				*vertices++ = { vec3(0.f, 0.f, 0.f), vec3(1.f, 1.f, 1.f) };
				*vertices++ = { vec3(0.f, 0.f, 0.f), vec3(1.f, 1.f, 1.f) };
			}

			*lines++ = { (uint16)(2 * i), (uint16)(2 * i + 1) };
		}

		renderDebug<debug_unlit_line_pipeline::position_color>(trs_to_mat4(transform), vb, ib, vec4(1.f, 1.f, 1.f, 1.f), render_pass, true);
	}

	SkeletonSystem::SkeletonSystem(World* _world)
		: System(_world)
	{
		renderer_holder_rc = world->add_root_component<RendererHolderRootComponent>();
		ASSERT(renderer_holder_rc != nullptr);
	}

	SkeletonSystem::~SkeletonSystem()
	{
		allocator->reset();
	}

	void SkeletonSystem::init()
	{
		allocator = make_ref<Allocator>();
	}

	void SkeletonSystem::update(float dt)
	{
		ZoneScopedN("SkeletonSystem::render");

		//MemoryMarker marker = allocator->get_marker();

		// TODO
		
		//allocator->reset_to_marker(marker);
	}

	void SkeletonSystem::draw_skeletons(float dt)
	{
		ZoneScopedN("SkeletonSystem::draw_skeletons");

		for (auto [entity_handle, skeleton, transform] : world->group(components_group<SkeletonComponent, TransformComponent>).each())
		{
			if (skeleton.draw_sceleton)
			{
				draw_skeleton(*skeleton.skeleton, skeleton.get_current_pose(), transform.get_world_transform(), renderer_holder_rc->ldrRenderPass);
			}
		}
	}

}