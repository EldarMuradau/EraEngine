#include "ecs/rendering/cloth_render_component.h"
#include "ecs/world.h"

#include <rttr/registration>

namespace era_engine
{
	RTTR_REGISTRATION
	{
		using namespace rttr;
		registration::class_<ClothRenderComponent>("ClothRenderComponent")
			.constructor<ref<Entity::EcsData>>();
	}

	ClothRenderComponent::ClothRenderComponent(ref<Entity::EcsData> _data)
		: Component(_data)
	{
	}

	std::tuple<dx_vertex_buffer_group_view, dx_vertex_buffer_group_view, dx_index_buffer_view, submesh_info> era_engine::ClothRenderComponent::get_render_data()
	{
		if (get_data_internal != nullptr)
		{
			return get_data_internal(this);
		}
		return std::tuple<dx_vertex_buffer_group_view, dx_vertex_buffer_group_view, dx_index_buffer_view, submesh_info>();
	}
}