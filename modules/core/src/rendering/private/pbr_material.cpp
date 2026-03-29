// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#include "rendering/pbr_material.h"

namespace std
{
	using namespace era_engine;
	template<>
	struct hash<PbrMaterialDesc>
	{
		size_t operator()(const PbrMaterialDesc& x) const
		{
			size_t seed = 0;

			hash_combine(seed, x.albedo);
			hash_combine(seed, x.normal);
			hash_combine(seed, x.roughness);
			hash_combine(seed, x.metallic);
			hash_combine(seed, x.emission);
			hash_combine(seed, x.albedo_tint);
			hash_combine(seed, x.roughness_override);
			hash_combine(seed, x.metallic_override);
			hash_combine(seed, (uint32)x.shader);
			hash_combine(seed, x.uv_scale);
			hash_combine(seed, x.translucency);

			return seed;
		}
	};
}

namespace era_engine
{
	static bool operator==(const PbrMaterialDesc& a, const PbrMaterialDesc& b)
	{
		return a.albedo == b.albedo
			&& a.normal == b.normal
			&& a.roughness == b.roughness
			&& a.metallic == b.metallic
			&& a.emission == b.emission
			&& a.albedo_tint == b.albedo_tint
			&& a.roughness_override == b.roughness_override
			&& a.metallic_override == b.metallic_override
			&& a.shader == b.shader
			&& a.uv_scale == b.uv_scale
			&& a.translucency == b.translucency;
	}

	static std::unordered_map<PbrMaterialDesc, weakref<pbr_material>> materialCache;
	static std::mutex mutex;

	ref<pbr_material> createPBRMaterial(const PbrMaterialDesc& desc)
	{
		std::lock_guard lock{ mutex };

		auto sp = materialCache[desc].lock();
		if (!sp)
		{
			ref<pbr_material> material = make_ref<pbr_material>();

			if (!desc.albedo.empty()) material->albedo = loadTextureFromFile(desc.albedo, desc.albedo_flags);
			if (!desc.normal.empty()) material->normal = loadTextureFromFile(desc.normal, desc.normal_flags);
			if (!desc.roughness.empty()) material->roughness = loadTextureFromFile(desc.roughness, desc.roughness_flags);
			if (!desc.metallic.empty()) material->metallic = loadTextureFromFile(desc.metallic, desc.metallic_flags);
			material->emission = desc.emission;
			material->albedoTint = desc.albedo_tint;
			material->roughnessOverride = desc.roughness_override;
			material->metallicOverride = desc.metallic_override;
			material->shader = desc.shader;
			material->uvScale = desc.uv_scale;
			material->translucency = desc.translucency;

			materialCache[desc] = sp = material;
		}

		return sp;
	}

	ref<pbr_material> createPBRMaterialAsync(const PbrMaterialDesc& desc, JobHandle parentJob)
	{
		std::lock_guard lock{ mutex };

		auto sp = materialCache[desc].lock();
		if (!sp)
		{
			ref<pbr_material> material = make_ref<pbr_material>();

			if (!desc.albedo.empty()) material->albedo = loadTextureFromFileAsync(desc.albedo, desc.albedo_flags, parentJob);
			if (!desc.normal.empty()) material->normal = loadTextureFromFileAsync(desc.normal, desc.normal_flags, parentJob);
			if (!desc.roughness.empty()) material->roughness = loadTextureFromFileAsync(desc.roughness, desc.roughness_flags, parentJob);
			if (!desc.metallic.empty()) material->metallic = loadTextureFromFileAsync(desc.metallic, desc.metallic_flags, parentJob);
			material->emission = desc.emission;
			material->albedoTint = desc.albedo_tint;
			material->roughnessOverride = desc.roughness_override;
			material->metallicOverride = desc.metallic_override;
			material->shader = desc.shader;
			material->uvScale = desc.uv_scale;
			material->translucency = desc.translucency;

			materialCache[desc] = sp = material;
		}

		return sp;
	}

	ref<pbr_material> getDefaultPBRMaterial()
	{
		static ref<pbr_material> material = createPBRMaterial({});
		return material;
	}
}