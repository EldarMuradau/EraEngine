// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#pragma once

#include "core_api.h"

#include "core/math.h"
#include "core/camera.h"

#include "ecs/component.h"

namespace era_engine
{
	struct ERA_CORE_API water_settings
	{
		vec4 deepWaterColor = vec4(0.f, 0.241f, 0.799f, 0.76f);
		vec4 shallowWaterColor = vec4(0.120f, 0.546f, 0.941f, 0.176f);
		float uvScale = 0.5f;
		float shallowDepth = 3.f;
		float transitionStrength = 0.07f;
		float normalStrength = 1.f;
	};

	class ERA_CORE_API WaterComponent : public Component
	{
	public:
		WaterComponent() = default;
		WaterComponent(ref<Entity::EcsData> _data, const water_settings& _settings = water_settings{});
		~WaterComponent() override;

		void render(const render_camera& camera, struct transparent_render_pass* render_pass, const vec3& position_offset, const vec2& scale, float dt);

		ERA_VIRTUAL_REFLECT(Component)

	public:
		water_settings settings;

	protected:
		float time = 0.f;
	};

	void initializeWaterPipelines();
}