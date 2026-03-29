// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#include "geometry/mesh.h"

#include "rendering/pbr.h"

#include "core/hash.h"
#include "core/string.h"

#include "asset/file_registry.h"
#include "asset/game_asset.h"
#include "asset/model_asset.h"

#include "animation/skeleton.h"
#include "animation/animation_clip_utils.h"
#include "animation/animation_clip.h"

namespace era_engine
{
	struct mesh_key
	{
		AssetHandle handle;
		uint32 flags;
	};
}

namespace std
{
	template<>
	struct hash<era_engine::mesh_key>
	{
		size_t operator()(const era_engine::mesh_key& x) const
		{
			size_t seed = 0;
			hash_combine(seed, x.handle);
			hash_combine(seed, x.flags);
			return seed;
		}
	};
}

namespace era_engine
{
	static void meshLoaderThread(ref<multi_mesh> result, const fs::path& sceneFilename, uint32 flags, const mesh_load_callback& cb,
		bool async, JobHandle parentJob)
	{
		using namespace animation;

		result->aabb = bounding_box::negativeInfinity();

		ModelAsset asset = load_3d_model_from_file(sceneFilename);
		mesh_builder builder(flags | mesh_creation_flags_with_skin);

		for (auto& mesh : asset.meshes)
		{
			for (auto& sub : mesh.submeshes)
			{
				PbrMaterialDesc& materialDesc = asset.materials[sub.material_index];

				materialDesc.emission = vec4(0.0f, 0.0f, 0.0f, 1.0f);

				{
					if (!materialDesc.albedo.empty() && *materialDesc.albedo.c_str() != 'F')
						materialDesc.albedo = get_asset_path(convert_path(materialDesc.albedo.string()));
					if (!materialDesc.normal.empty() && *materialDesc.normal.c_str() != 'F')
						materialDesc.normal = get_asset_path(convert_path(materialDesc.normal.string()));
					if (!materialDesc.roughness.empty() && *materialDesc.roughness.c_str() != 'F')
						materialDesc.roughness = get_asset_path(convert_path(materialDesc.roughness.string()));
					if (!materialDesc.metallic.empty() && *materialDesc.metallic.c_str() != 'F')
						materialDesc.metallic = get_asset_path(convert_path(materialDesc.metallic.string()));
				}

				ref<pbr_material> material;
				if (!async)
				{
					material = createPBRMaterial(materialDesc);
				}
				else
				{
					material = createPBRMaterialAsync(materialDesc, parentJob);
				}

				bounding_box aabb;
				builder.pushMesh(sub, 1.f, &aabb);
				result->submeshes.push_back({ builder.endSubmesh(), aabb, trs::identity, material, mesh.name });

				result->aabb.grow(aabb.minCorner);
				result->aabb.grow(aabb.maxCorner);
			}
		}

		GameAssetsProvider provider;

		ref<Skeleton> default_skeleton;

		// Load skeleton
		if (!asset.skeletons.empty() && (flags & mesh_creation_flags_with_skin))
		{
			uint32 skeleton_index = 0;
			for (SkeletonAsset& skeleton_asset : asset.skeletons)
			{
				fs::path skeleton_path = sceneFilename.parent_path();
				skeleton_path.append("skeletons");
				skeleton_path.append("skeleton" + std::to_string(skeleton_index));

				if (!fs::exists(fs::path(skeleton_path.string() + AssetExtension<Skeleton>::get_asset_type())))
				{
					ref<Skeleton> imported_skeleton = make_ref<Skeleton>();

					if (default_skeleton == nullptr)
					{
						default_skeleton = imported_skeleton;
					}

					imported_skeleton->joints = std::move(skeleton_asset.joints);

					imported_skeleton->local_transforms.reserve(imported_skeleton->joints.size());
					for (size_t i = 0; i < imported_skeleton->joints.size(); ++i)
					{
						SkeletonJoint& joint = imported_skeleton->joints[i];
						JointTransform joint_transform;

						if (joint.parent_id > imported_skeleton->joints.size() || joint.parent_id == INVALID_JOINT)
						{
							if (flags & mesh_creation_flags_unreal_asset)
							{
								joint_transform.set_transform(mat4_to_trs(joint.bind_transform) * trs { vec3::zero, euler_to_quat(vec3(0.0f, -M_PI / 2.0f, 0.0f)), vec3(1.0f) });
							}
							else
							{
								joint_transform.set_transform(mat4_to_trs(joint.bind_transform));
							}
						}
						else
						{
							const auto& parent_bind = imported_skeleton->joints[joint.parent_id].bind_transform;
							joint_transform.set_transform(mat4_to_trs(invert(parent_bind) * joint.bind_transform));
						}

						imported_skeleton->local_transforms.push_back(joint_transform);
					}

					imported_skeleton->name_to_joint_id = std::move(skeleton_asset.name_to_joint_id);
					imported_skeleton->analyze_joints(builder.getPositions(), (uint8*)builder.getOthers() + builder.getSkinOffset(), builder.getOthersSize(), builder.getNumVertices());

					JobHandle skeleton_load_job = provider.save_game_asset_to_file_async<Skeleton>(skeleton_path, imported_skeleton.get(), parentJob);
					skeleton_load_job.wait_for_completion();
					skeleton_index++;
				}
			}
		}


		{
			AnimationSkeleton animation_skeleton;

			if (default_skeleton == nullptr &&
				!asset.animations.empty())
			{
				fs::path skeleton_path = sceneFilename.parent_path();
				skeleton_path.append("skeletons");
				skeleton_path.append("skeleton" + std::to_string(0));

				default_skeleton = provider.load_game_asset_from_file<Skeleton>(skeleton_path);
				default_skeleton->load_job.wait_for_completion();

			}
			animation_skeleton.skeleton = default_skeleton.get();

			uint32 anim_index = 0;
			uint32 clips_loaded = 0;

			// Load animations
			for (auto& anim : asset.animations)
			{
				fs::path clip_path = sceneFilename.parent_path();
				clip_path.append("animations");
				clip_path.append("animation_clip" + std::to_string(anim_index));

				if (!fs::exists(fs::path(clip_path.string() + AssetExtension<AnimationAssetClip>::get_asset_type())))
				{
					AnimationAsset& in = anim;

					AnimationClip& clip = animation_skeleton.clips.emplace_back();
					clip.name = std::move(in.name);
					clip.is_unreal_asset = flags & mesh_creation_flags_unreal_asset;
					clip.filename = sceneFilename;
					clip.length_in_seconds = in.duration;
					clip.joints.resize(default_skeleton->joints.size(), {});

					clip.position_keyframes = std::move(in.position_keyframes);
					clip.position_timestamps = std::move(in.position_timestamps);
					clip.rotation_keyframes = std::move(in.rotation_keyframes);
					clip.rotation_timestamps = std::move(in.rotation_timestamps);
					clip.scale_keyframes = std::move(in.scale_keyframes);
					clip.scale_timestamps = std::move(in.scale_timestamps);

					for (auto& [name, joint] : in.joints)
					{
						auto it = default_skeleton->name_to_joint_id.find(name);
						if (it != default_skeleton->name_to_joint_id.end())
						{
							AnimationJoint& j = clip.joints[it->second];
							j = joint;
						}
					}

					clip.root_motion_joint = clip.joints[0];

					ref<AnimationAssetClip> animation_clip = AnimationAssetClipUtils::make_clip(animation_skeleton, clips_loaded, default_skeleton.get());

					JobHandle animation_load_job = provider.save_game_asset_to_file_async<AnimationAssetClip>(clip_path, animation_clip.get(), parentJob);
					animation_load_job.wait_for_completion();
					clips_loaded++;
				}
				anim_index++;
			}
		}

		if (cb)
		{
			cb(builder, result->submeshes, result->aabb);
		}

		result->mesh = builder.createDXMesh();

		result->loadState.store(AssetLoadState::LOADED, std::memory_order_release);
	}

	static ref<multi_mesh> loadMeshFromFileInternal(const fs::path& sceneFilename, AssetHandle handle, uint32 flags, const mesh_load_callback& cb,
		bool async, JobHandle parentJob)
	{
		ref<multi_mesh> result = make_ref<multi_mesh>();
		result->handle = handle;
		result->flags = flags;
		result->loadState = AssetLoadState::LOADING;

		if (!async)
		{
			meshLoaderThread(result, sceneFilename, flags, cb, false, {});
			result->loadJob = {};
			return result;
		}
		else
		{
			struct mesh_loading_data
			{
				mesh_load_callback cb;
				fs::path path;
				ref<multi_mesh> mesh;
				uint32 flags;
			};

			constexpr int a = sizeof(mesh_loading_data);

			mesh_loading_data data = { cb, sceneFilename, result, flags };

			JobHandle job = low_priority_job_queue.createJob<mesh_loading_data>([](mesh_loading_data& data, JobHandle job)
				{
					meshLoaderThread(data.mesh, data.path, data.flags, data.cb, true, job);
				}, data, parentJob);
			job.submit_now();

			result->loadJob = job;

			return result;
		}
	}

	static bool operator==(const mesh_key& a, const mesh_key& b)
	{
		return a.handle == b.handle && a.flags == b.flags;
	}

	static std::unordered_map<mesh_key, weakref<multi_mesh>> meshCache;
	static std::mutex mutex;

	static ref<multi_mesh> loadMeshFromFileAndHandle(const fs::path& filename, AssetHandle handle, uint32 flags, const mesh_load_callback& cb,
		bool async = false, JobHandle parentJob = {})
	{
		if (!fs::exists(filename))
			return nullptr;

		mesh_key key = { handle, flags };

		std::lock_guard _lock{ mutex };
		ref<multi_mesh> result = { meshCache[key].lock(), {} };
		if (!result)
		{
			result = loadMeshFromFileInternal(filename, handle, flags, cb, async, parentJob);
			meshCache[key] = result;
		}

		return result;
	}

	ref<multi_mesh> loadMeshFromFile(const fs::path& filename, uint32 flags, const mesh_load_callback& cb)
	{
		fs::path path = filename.lexically_normal().make_preferred();

		AssetHandle handle = getAssetHandleFromPath(path);
		return loadMeshFromFileAndHandle(path, handle, flags, cb);
	}

	ref<multi_mesh> loadMeshFromHandle(AssetHandle handle, uint32 flags, const mesh_load_callback& cb)
	{
		fs::path sceneFilename = getPathFromAssetHandle(handle);
		return loadMeshFromFileAndHandle(sceneFilename, handle, flags, cb);
	}

	ref<multi_mesh> loadMeshFromFileAsync(const fs::path& filename, uint32 flags, JobHandle parentJob, const mesh_load_callback& cb)
	{
		fs::path path = filename.lexically_normal().make_preferred();

		AssetHandle handle = getAssetHandleFromPath(path);
		return loadMeshFromFileAndHandle(path, handle, flags, cb, true, parentJob);
	}

	ref<multi_mesh> loadMeshFromHandleAsync(AssetHandle handle, uint32 flags, JobHandle parentJob, const mesh_load_callback& cb)
	{
		fs::path sceneFilename = getPathFromAssetHandle(handle);
		return loadMeshFromFileAndHandle(sceneFilename, handle, flags, cb, true, parentJob);
	}
}