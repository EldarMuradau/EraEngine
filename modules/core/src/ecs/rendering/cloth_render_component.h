#pragma once

#include "core_api.h"
#include "ecs/component.h"

#include "geometry/mesh.h"

namespace era_engine
{
	class ERA_CORE_API ClothRenderComponent : public Component
	{
	public:
		ClothRenderComponent(ref<Entity::EcsData> _data);

		std::tuple<dx_vertex_buffer_group_view, dx_vertex_buffer_group_view, dx_index_buffer_view, SubmeshInfo> get_render_data();

		ERA_VIRTUAL_REFLECT(Component)
	public:
		std::function<std::tuple<dx_vertex_buffer_group_view, dx_vertex_buffer_group_view, dx_index_buffer_view, SubmeshInfo>(ClothRenderComponent*)> get_data_internal;

		ref<dx_index_buffer> indexBuffer;
		dx_vertex_buffer_group_view prevFrameVB;
	};
}