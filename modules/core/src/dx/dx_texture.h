// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#pragma once

#include "core_api.h"
#include "core/math.h"
#include "core/job_system.h"

#include "dx/dx_descriptor.h"
#include "dx/dx_descriptor_allocation.h"

#include "asset/asset.h"
#include "asset/image.h"

namespace era_engine
{
	struct dx_texture
	{
		virtual ~dx_texture();

		dx_cpu_descriptor_handle uavAt(uint32 index) { return srvUavAllocation.cpuAt(1 + index); }
		dx_rtv_descriptor_handle rtvAt(uint32 index) { return rtvAllocation.cpuAt(index); }

		void setName(const wchar* name) const;
		std::wstring getName() const;

		dx_resource resource;
		D3D12MA::Allocation* allocation = 0;

		dx_descriptor_allocation srvUavAllocation = {};
		dx_descriptor_allocation rtvAllocation = {};
		dx_descriptor_allocation dsvAllocation = {};

		dx_cpu_descriptor_handle defaultSRV; // SRV for the whole texture (all mip levels).
		dx_cpu_descriptor_handle defaultUAV; // UAV for the first mip level.

		dx_cpu_descriptor_handle stencilSRV; // For depth stencil textures.

		dx_rtv_descriptor_handle defaultRTV;

		dx_dsv_descriptor_handle defaultDSV;

		uint32 width, height, depth;
		DXGI_FORMAT format;

		D3D12_RESOURCE_STATES initialState;

		bool supportsRTV;
		bool supportsDSV;
		bool supportsUAV;
		bool supportsSRV;

		uint32 requestedNumMipLevels;
		uint32 numMipLevels;

		AssetHandle handle;
		uint32 flags = 0;

		std::atomic<AssetLoadState> loadState = AssetLoadState::LOADED;
		JobHandle loadJob;
	};

	struct dx_texture_atlas
	{
		NODISCARD std::pair<vec2, vec2> getUVs(uint32 x, uint32 y) const
		{
			ASSERT(x < cols);
			ASSERT(y < rows);

			float width = 1.f / cols;
			float height = 1.f / rows;
			vec2 uv0 = vec2(x * width, y * height);
			vec2 uv1 = vec2((x + 1) * width, (y + 1) * height);

			return { uv0, uv1 };
		}

		NODISCARD std::pair<vec2, vec2> getUVs(uint32 i) const
		{
			uint32 x = i % cols;
			uint32 y = i / cols;
			return getUVs(x, y);
		}

		ref<dx_texture> texture;

		uint32 cols;
		uint32 rows;
	};

	struct dx_tiled_texture
	{
		inline NODISCARD bool isMipPacked(uint32 index) const { return index >= numStandard; }

		struct tiled_texture_mip_desc
		{
			CD3DX12_TILED_RESOURCE_COORDINATE startCoordinate;
			D3D12_TILE_REGION_SIZE regionSize;
		};

		ref<dx_texture> texture;
		std::vector<tiled_texture_mip_desc> mipDescs;
		D3D12_TILE_SHAPE tileShape;

		uint32 numStandard;
		uint32 numPacked;
	};

	struct texture_grave
	{
		texture_grave() {}
		texture_grave(const texture_grave& o) = delete;
		texture_grave(texture_grave&& o) = default;

		texture_grave& operator=(const texture_grave& o) = delete;
		texture_grave& operator=(texture_grave&& o) = default;

		~texture_grave();

		dx_resource resource;

		dx_descriptor_allocation srvUavAllocation = {};
		dx_descriptor_allocation rtvAllocation = {};
		dx_descriptor_allocation dsvAllocation = {};
	};

	NODISCARD D3D12_RESOURCE_ALLOCATION_INFO getTextureAllocationInfo(uint32 width, uint32 height, DXGI_FORMAT format, bool allocateMips, D3D12_RESOURCE_FLAGS flags);

	void uploadTextureSubresourceData(ref<dx_texture> texture, D3D12_SUBRESOURCE_DATA* subresourceData, uint32 firstSubresource, uint32 numSubresources);
	NODISCARD ref<dx_texture> createTexture(D3D12_RESOURCE_DESC textureDesc, D3D12_SUBRESOURCE_DATA* subresourceData, uint32 numSubresources, D3D12_RESOURCE_STATES initialState = D3D12_RESOURCE_STATE_COMMON, bool mipUAVs = false);
	NODISCARD ref<dx_texture> createTexture(const void* data, uint32 width, uint32 height, DXGI_FORMAT format, bool allocateMips = false, bool allowRenderTarget = false, bool allowUnorderedAccess = false, D3D12_RESOURCE_STATES initialState = D3D12_RESOURCE_STATE_COMMON, bool mipUAVs = false);
	NODISCARD ref<dx_texture> createDepthTexture(uint32 width, uint32 height, DXGI_FORMAT format, uint32 arrayLength = 1, D3D12_RESOURCE_STATES initialState = D3D12_RESOURCE_STATE_DEPTH_WRITE);
	NODISCARD ref<dx_texture> createCubeTexture(const void* data, uint32 width, uint32 height, DXGI_FORMAT format, bool allocateMips = false, bool allowRenderTarget = false, bool allowUnorderedAccess = false, D3D12_RESOURCE_STATES initialState = D3D12_RESOURCE_STATE_COMMON, bool mipUAVs = false);
	NODISCARD ref<dx_texture> createVolumeTexture(const void* data, uint32 width, uint32 height, uint32 depth, DXGI_FORMAT format, bool allowUnorderedAccess = false, D3D12_RESOURCE_STATES initialState = D3D12_RESOURCE_STATE_COMMON);
	void resizeTexture(ref<dx_texture> texture, uint32 newWidth, uint32 newHeight, D3D12_RESOURCE_STATES initialState = (D3D12_RESOURCE_STATES)-1);

	NODISCARD ref<dx_texture> createPlacedTexture(dx_heap heap, uint64 offset, uint32 width, uint32 height, DXGI_FORMAT format, bool allocateMips = false, bool allowRenderTarget = false, bool allowUnorderedAccess = false, D3D12_RESOURCE_STATES initialState = D3D12_RESOURCE_STATE_COMMON, bool mipUAVs = false);
	NODISCARD ref<dx_texture> createPlacedTexture(dx_heap heap, uint64 offset, D3D12_RESOURCE_DESC textureDesc, D3D12_RESOURCE_STATES initialState = D3D12_RESOURCE_STATE_COMMON, bool mipUAVs = false);

	NODISCARD ref<dx_texture> createPlacedDepthTexture(dx_heap heap, uint64 offset, uint32 width, uint32 height, DXGI_FORMAT format, uint32 arrayLength = 1, D3D12_RESOURCE_STATES initialState = D3D12_RESOURCE_STATE_DEPTH_WRITE, bool allowDepthStencil = true);
	NODISCARD ref<dx_texture> createPlacedDepthTexture(dx_heap heap, uint64 offset, D3D12_RESOURCE_DESC textureDesc, D3D12_RESOURCE_STATES initialState = D3D12_RESOURCE_STATE_DEPTH_WRITE);

	NODISCARD dx_tiled_texture createTiledTexture(D3D12_RESOURCE_DESC textureDesc, D3D12_RESOURCE_STATES initialState = D3D12_RESOURCE_STATE_COMMON, bool mipUAVs = false);
	NODISCARD dx_tiled_texture createTiledTexture(uint32 width, uint32 height, DXGI_FORMAT format, bool allocateMips = false, bool allowRenderTarget = false, bool allowUnorderedAccess = false, D3D12_RESOURCE_STATES initialState = D3D12_RESOURCE_STATE_COMMON, bool mipUAVs = false);

	// This system caches textures. It does not keep the resource alive (we store weak ptrs).
	// So if no one else has a reference, the texture gets deleted.
	// This means you should keep a reference to your textures yourself and not call this every frame.

	ERA_CORE_API ref<dx_texture> loadTextureFromFile(const fs::path& filename, uint32 flags = image_load_flags_default);
	ERA_CORE_API ref<dx_texture> loadTextureFromHandle(AssetHandle handle, uint32 flags = image_load_flags_default);
	ERA_CORE_API ref<dx_texture> loadTextureFromFileAsync(const fs::path& filename, uint32 flags = image_load_flags_default, JobHandle parent_job = {});
	ERA_CORE_API ref<dx_texture> loadTextureFromHandleAsync(AssetHandle handle, uint32 flags = image_load_flags_default, JobHandle parent_job = {});

	ERA_CORE_API ref<dx_texture> loadTextureFromMemory(const void* ptr, uint32 size, image_format imageFormat, const fs::path& cacheFilename, uint32 flags = image_load_flags_default);
	ERA_CORE_API ref<dx_texture> loadVolumeTextureFromDirectory(const fs::path& dirname, uint32 flags = image_load_flags_compress | image_load_flags_cache_to_dds | image_load_flags_noncolor);

	void copyTextureToCPUBuffer(const ref<dx_texture>& texture, void* buffer, D3D12_RESOURCE_STATES beforeAndAfterState = D3D12_RESOURCE_STATE_COMMON);
	ERA_CORE_API void saveTextureToFile(const ref<dx_texture>& texture, const fs::path& path);
	void saveTextureToFile(dx_resource texture, uint32 width, uint32 height, DXGI_FORMAT format, const fs::path& path);
}