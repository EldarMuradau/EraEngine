#include "editor/file_utils.h"

#include <asset/model_asset.h>

#include <ecs/world.h>
#include <ecs/entity.h>
#include <ecs/base_components/base_components.h>
#include <ecs/rendering/mesh_component.h>
#include <ecs/rendering/world_renderer.h>

#include <rendering/ecs/renderer_holder_root_component.h>
#include <rendering/render_utils.h>

namespace era_engine
{
	void EditorFileUtils::handle_file_drop(const fs::path& filename, World* world)
	{
		fs::path path = filename;
		fs::path relative = fs::relative(path, fs::current_path());
		fs::path ext = relative.extension();

		if (is_mesh_extension(ext))
		{
			fs::path path = filename;
			path = path.stem();

			if (auto mesh = loadAnimatedMeshFromFileAsync(relative.string()))
			{
				auto entity = world->create_entity();
				entity.add_component<MeshComponent>(mesh);
				entity.add_component<animation::AnimationComponent>();
				entity.add_component<animation::SkeletonComponent>();

				TransformComponent* transform_component = entity.get_component<TransformComponent>();
				transform_component->type = TransformComponent::DYNAMIC;

				addRaytracingComponentAsync(entity, mesh);
			}
			else if (auto mesh = import_mesh_from_file_async(relative.string()))
			{
				auto entity = world->create_entity();
				entity.add_component<animation::AnimationComponent>();
				entity.add_component<MeshComponent>(mesh);

				addRaytracingComponentAsync(entity, mesh);
			}
		}
		else if (ext == ".hdr")
		{
			world->get_root_component<RendererHolderRootComponent>()->environment.setFromTexture(relative);
		}
	}
}