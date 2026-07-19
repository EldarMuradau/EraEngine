// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#include "asset/bin.h"
#include "asset/io.h"
#include "asset/model_asset.h"
#include "asset/mesh_postprocessing.h"
#include "asset/pbr_material_desc.h"

#include "core/math.h"
#include "core/cpu_profiling.h"

#include "geometry/mesh.h"

//#define PROFILE(name) CPU_PRINT_PROFILE_BLOCK(name)
#define PROFILE(name) 

namespace era_engine
{
	static bool is_end_of_line(char c)
	{
		return c == '\r' ||
			c == '\n';
	}

	static bool is_whitespace(char c)
	{
		return c == ' ' ||
			c == '\t' ||
			is_end_of_line(c);
	}

	static bool is_white_space_no_end_of_line(char c)
	{
		return c == ' ' ||
			c == '\t';
	}

	static void discard_line(EntireFile& file)
	{
		while (file.read_offset < file.size && file.content[file.read_offset] != '\n')
		{
			++file.read_offset;
		}
		if (file.read_offset < file.size)
		{
			++file.read_offset;
		}
	}

	static void discard_whitespace(EntireFile& file, bool allow_line_skip = true)
	{
		if (allow_line_skip)
		{
			while (file.read_offset < file.size)
			{
				while (file.read_offset < file.size && is_whitespace((char)file.content[file.read_offset]))
				{
					++file.read_offset;
				}
				if (file.read_offset < file.size && file.content[file.read_offset] == '#')
				{
					discard_line(file);
				}
				else
				{
					break;
				}
			}
		}
		else
		{
			while (file.read_offset < file.size && is_white_space_no_end_of_line((char)file.content[file.read_offset]))
			{
				++file.read_offset;
			}
		}
	}

	static sized_string read_string(EntireFile& file, bool allow_line_skip = true)
	{
		discard_whitespace(file, allow_line_skip);

		sized_string result;
		result.str = (const char*)file.content + file.read_offset;
		while (file.read_offset < file.size && !is_whitespace((char)file.content[file.read_offset]))
		{
			++result.length;
			++file.read_offset;
		}

		return result;
	}

	static int32 read_int32(EntireFile& file)
	{
		sized_string str = read_string(file);
		return atoi(str.str);
	}

	static vec3 read_vec3(EntireFile& file)
	{
		sized_string xStr = read_string(file);
		sized_string yStr = read_string(file);
		sized_string zStr = read_string(file);

		float x = (float)atof(xStr.str);
		float y = (float)atof(yStr.str);
		float z = (float)atof(zStr.str);

		return vec3(x, y, z);
	}

	static vec2 read_vec2(EntireFile& file)
	{
		sized_string xStr = read_string(file);
		sized_string yStr = read_string(file);

		float x = (float)atof(xStr.str);
		float y = (float)atof(yStr.str);

		return vec2(x, y);
	}

	static float read_float(EntireFile& file)
	{
		sized_string str = read_string(file);
		float v = (float)atof(str.str);
		return v;
	}

	struct obj_vertex_indices
	{
		int32 position_index;
		int32 normal_index;
		int32 uv_index;
	};

	static obj_vertex_indices read_vertex_indices(sized_string vertex_str)
	{
		sized_string index_strs[3] = {};
		index_strs[0].str = vertex_str.str;
		uint32 curr = 0;
		for (uint32 i = 0; i < vertex_str.length; ++i)
		{
			if (vertex_str.str[i] == '/')
			{
				++curr;
				index_strs[curr].str = vertex_str.str + (i + 1);
			}
			else
			{
				++index_strs[curr].length;
			}
		}

		int32 position_index = index_strs[0].length ? atoi(index_strs[0].str) : 0;
		int32 uv_index = index_strs[1].length ? atoi(index_strs[1].str) : 0;
		int32 normal_index = index_strs[2].length ? atoi(index_strs[2].str) : 0;

		if (position_index > 0)
		{
			--position_index;
		}
		if (normal_index > 0)
		{
			--normal_index;
		}
		if (uv_index > 0)
		{
			--uv_index;
		}

		return { position_index, normal_index, uv_index };
	}

	static std::vector<std::pair<std::string, PbrMaterialDesc>> load_material_library(const fs::path& path)
	{
		EntireFile file = load_file(path);

		std::vector<std::pair<std::string, PbrMaterialDesc>> result;
		PbrMaterialDesc current_material = {};
		std::string current_name;

		{
			PROFILE("Parse OBJ material library");

			while (file.read_offset < file.size)
			{
				sized_string token = read_string(file);
				std::transform(token.str, token.str + token.length, (char*)token.str, [](char c) { return std::tolower(c); });

				if (token == "newmtl")
				{
					if (!current_name.empty())
					{
						result.push_back({ std::move(current_name), std::move(current_material) });
					}

					sized_string name = read_string(file);
					current_name = name_to_string(name);
				}
				else if (token == "ns")
				{
					float value = read_float(file);
					current_material.roughness_override = 1.f - (sqrt(value) * 0.1f);
				}
				else if (token == "pr")
				{
					float value = read_float(file);
					current_material.roughness_override = value;
				}
				else if (token == "pm")
				{
					float value = read_float(file);
					current_material.metallic_override = value;
				}
				else if (token == "ni")
				{

				}
				else if (token == "d")
				{

				}
				else if (token == "tr")
				{
					float value = read_float(file);
					float alpha = 1.f - value;
					current_material.albedo_tint.w = alpha;
					if (alpha < 1.f)
					{
						current_material.shader = pbr_material_shader_transparent;
					}
				}
				else if (token == "tf")
				{

				}
				else if (token == "illum")
				{

				}
				else if (token == "ka")
				{

				}
				else if (token == "kd")
				{
					vec3 color = read_vec3(file);
					current_material.albedo_tint.xyz = color;
				}
				else if (token == "ks")
				{

				}
				else if (token == "ke")
				{
					vec3 color = read_vec3(file);
					current_material.emission = vec4(color, 1.f);
				}
				else if (token == "map_ka" || token == "map_pm")
				{
					sized_string str = read_string(file);
					current_material.metallic = relative_filepath(str, path);
				}
				else if (token == "map_kd")
				{
					sized_string str = read_string(file);
					current_material.albedo = relative_filepath(str, path);
				}
				else if (token == "map_d")
				{

				}
				else if (token == "map_bump")
				{
					sized_string str = read_string(file);
					current_material.normal = relative_filepath(str, path);
				}
				else if (token == "map_ns" || token == "map_pr")
				{
					sized_string str = read_string(file);
					current_material.roughness = relative_filepath(str, path);
				}
				else if (token == "bump")
				{

				}
				else if (token.length == 0)
				{
					// Nothing.
				}
				else
				{
					printf("Unrecognized material start token '%.*s'\n", token.length, token.str);
				}

				discard_line(file);
			}
		}
		if (!current_name.empty())
		{
			result.push_back({ std::move(current_name), std::move(current_material) });
		}

		free_file(file);

		return result;
	}

	ModelAsset load_obj(const fs::path& path, uint32 flags)
	{
		PROFILE("Loading OBJ");

		EntireFile file = load_file(path);

		std::vector<vec3> positions; positions.reserve(1 << 16);
		std::vector<vec2> uvs; uvs.reserve(1 << 16);
		std::vector<vec3> normals; normals.reserve(1 << 16);

		std::vector<PbrMaterialDesc> materials;
		std::unordered_map<std::string, int32> name_to_material_index;
		int32 current_material_index = 0;

		std::vector<SubmeshAsset> submeshes;

		std::unordered_map<int32, per_material> material_to_mesh;

		std::vector<vec3> position_cache; position_cache.reserve(16);
		std::vector<vec2> uv_cache; uv_cache.reserve(16);
		std::vector<vec3> normal_cache; normal_cache.reserve(16);

		{
			PROFILE("Parse OBJ");

			while (file.read_offset < file.size)
			{
				sized_string token = read_string(file);

				if (token == "mtllib")
				{
					sized_string lib = read_string(file);
					auto lib_materials = load_material_library(relative_filepath(lib, path));
					for (auto& [name, mat] : lib_materials)
					{
						name_to_material_index[std::move(name)] = (int32)materials.size();
						materials.push_back(std::move(mat));
					}
				}
				else if (token == "v")
				{
					positions.push_back(read_vec3(file));
				}
				else if (token == "vn")
				{
					normals.push_back(read_vec3(file));
				}
				else if (token == "vt")
				{
					vec2 uv = read_vec2(file);
					if (flags & MESH_FLAG_FLIP_UVS_VERTICALLY)
					{
						uv.y = 1.f - uv.y;
					}
					uvs.push_back(uv);
				}
				else if (token == "g")
				{
					sized_string name = read_string(file);
				}
				else if (token == "o")
				{
					sized_string name = read_string(file);
				}
				else if (token == "s")
				{
					int32 smoothing = read_int32(file);
				}
				else if (token == "usemtl")
				{
					sized_string mtl = read_string(file);
					std::string name = name_to_string(mtl);

					auto it = name_to_material_index.find(name);
					if (it != name_to_material_index.end())
					{
						current_material_index = it->second;
					}
					else
					{
						printf("Unrecognized material '%.*s'\n", mtl.length, mtl.str);
					}
				}
				else if (token == "f")
				{
					int32 face_size = 0;
					while (file.read_offset < file.size && !is_end_of_line((char)file.content[file.read_offset]))
					{
						sized_string vertex_str = read_string(file, false);
						if (vertex_str.length == 0)
						{
							break;
						}

						obj_vertex_indices vertex_indices = read_vertex_indices(vertex_str);

						int32 curr_num_positions = (int32)positions.size();
						vertex_indices.position_index += (vertex_indices.position_index >= 0) ? 0 : curr_num_positions;
						ASSERT(vertex_indices.position_index < curr_num_positions);
						position_cache.push_back(positions[vertex_indices.position_index]);

						if (flags & MESH_FLAG_LOAD_UVS)
						{
							int32 curr_num_uvs = (int32)uvs.size();
							vertex_indices.uv_index += (vertex_indices.uv_index >= 0) ? 0 : curr_num_uvs;
							ASSERT(vertex_indices.uv_index < curr_num_uvs);
							uv_cache.push_back(uvs[vertex_indices.uv_index]);
						}

						if (flags & MESH_FLAG_LOAD_NORNALS)
						{
							int32 curr_num_normals = (int32)normals.size();
							vertex_indices.normal_index += (vertex_indices.normal_index >= 0) ? 0 : curr_num_normals;
							ASSERT(vertex_indices.normal_index < curr_num_normals);
							normal_cache.push_back(normals[vertex_indices.normal_index]);
						}

						++face_size;
					}

					per_material& per_mat = material_to_mesh[current_material_index];
					per_mat.sub.material_index = current_material_index;

					per_mat.addTriangles(position_cache, uv_cache, normal_cache, {}, {}, {}, 0, face_size, submeshes);

					position_cache.clear();
					uv_cache.clear();
					normal_cache.clear();
				}
				else if (token.length == 0)
				{
					// Nothing
				}
				else
				{
					printf("Unrecognized start token '%.*s'\n", token.length, token.str);
				}

				discard_line(file);
			}
		}

		for (auto& [i, per_mat] : material_to_mesh)
		{
			per_mat.flush(submeshes);
		}

		free_file(file);
		generateNormalsAndTangents(submeshes, flags);

		ModelAsset result;
		result.flags = flags;
		result.meshes.push_back({ path.filename().string(), std::move(submeshes), -1 });
		result.materials = std::move(materials);

		return result;
	}

}