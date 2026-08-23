// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#pragma once

#include "core_api.h"

#include "ecs/component.h"

#include "rendering/render_pass.h"

#include "terrain/terrain.h"

namespace era_engine
{
	struct ERA_CORE_API GrassSettings
	{
		static inline bool depthPrepass = true;

		float bladeHeight = 1.f;
		float bladeWidth = 0.05f;
		uint32 numGrassBladesPerChunkDim = 350;

		float lodChangeStartDistance = 50.f;
		float lodChangeTransitionDistance = 75.f;

		float cullStartDistance = 350.f;
		float cullTransitionDistance = 50.f;
	};

	class ERA_CORE_API GrassComponent final : public Component
	{
	public:
		GrassComponent() = default;
		GrassComponent(ref<Entity::EcsData> _data, const GrassSettings& _settings = {});
		~GrassComponent() override;

		void generate(struct compute_pass* compute_pass, const render_camera& camera, const TerrainComponent& terrain, const vec3& position_offset, float dt);
		void render(struct opaque_render_pass* render_pass);

		ERA_VIRTUAL_REFLECT(Component)

	public:
		GrassSettings settings;

	private:
		ref<dx_buffer> draw_buffer;
		ref<dx_buffer> count_buffer;
		ref<dx_buffer> blade_buffer_LOD0;
		ref<dx_buffer> blade_buffer_LOD1;

		float time = 0.f;
		float prev_time = 0.f;
		vec2 wind_direction = normalize(vec2(1.f, 1.f));
	};

	void initializeGrassPipelines();

}