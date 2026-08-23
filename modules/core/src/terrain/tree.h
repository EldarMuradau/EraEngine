// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#pragma once

#include "core_api.h"
#include "core/job_system.h"

#include "geometry/mesh.h"

#include "rendering/material.h"
#include "rendering/pbr.h"

#include "ecs/component.h"

namespace era_engine
{
	struct ERA_CORE_API tree_settings
	{
		float bendStrength;
	};

	class ERA_CORE_API TreeComponent : public Component
	{
	public:
		TreeComponent() = default;
		TreeComponent(ref<Entity::EcsData> _data, const tree_settings& _settings);
		~TreeComponent() override;

		ERA_VIRTUAL_REFLECT(Component)

	public:
		tree_settings settings;
	};

	void renderTree(struct opaque_render_pass* renderPass, D3D12_GPU_VIRTUAL_ADDRESS transforms, uint32 numInstances, const MultiMesh* mesh, float dt);

	void initializeTreePipelines();

	ref<MultiMesh> loadTreeMeshFromFile(const fs::path& sceneFilename);
	ref<MultiMesh> loadTreeMeshFromHandle(AssetHandle handle);

	ref<MultiMesh> loadTreeMeshFromFileAsync(const fs::path& sceneFilename, JobHandle parent_job = {});
	ref<MultiMesh> loadTreeMeshFromHandleAsync(AssetHandle handle, JobHandle parent_job = {});
}