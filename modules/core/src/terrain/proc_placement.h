// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#pragma once

#include "core_api.h"

#include "terrain/terrain.h"

#include "ecs/component.h"

namespace era_engine
{
	struct ERA_CORE_API proc_placement_layer_desc
	{
		const char* name;
		float footprint;
		ref<MultiMesh> meshes[4] = {};
	};

	class ERA_CORE_API ProcPlacementComponent : public Component
	{
	public:
		ProcPlacementComponent() = default;
		ProcPlacementComponent(ref<Entity::EcsData> _data, const std::vector<proc_placement_layer_desc>& _layers);
		virtual ~ProcPlacementComponent();

		void generate(const render_camera& camera, const TerrainComponent& terrain, const vec3& position_offset);
		void render(struct ldr_render_pass* render_pass);

		struct PlacementLayer
		{
			const char* name;

			float footprint;

			uint32 global_mesh_offset;
			uint32 num_meshes;

			ref<MultiMesh> meshes[4];
			float densities[4] = { 0.25f, 0.25f, 0.25f, 0.25f };
		};

		ERA_VIRTUAL_REFLECT(Component)

	public:
		std::vector<PlacementLayer> layers;

	private:

		bool has_valid_meshes = false;

		std::vector<uint32> submesh_to_mesh;

		ref<dx_buffer> placement_point_buffer;
		ref<dx_buffer> transform_buffer;
		ref<dx_buffer> mesh_count_buffer;
		ref<dx_buffer> mesh_offset_buffer;
		ref<dx_buffer> submesh_to_mesh_buffer;
		ref<dx_buffer> draw_indirect_buffer;
	};

	void initializeProceduralPlacementPipelines();
}