#pragma once

#include "core_api.h"

#include "core/threading.h"

#include "dx/dx.h"
#include "dx/dx_command_queue.h"
#include "dx/dx_upload_buffer.h"
#include "dx/dx_descriptor_allocation.h"
#include "dx/dx_buffer.h"
#include "dx/dx_query.h"

namespace era_engine
{
	struct ERA_CORE_API dx_memory_usage
	{
		// In MB.
		uint32 currentlyUsed;
		uint32 available;
	};

	enum dx_raytracing_tier
	{
		dx_raytracing_not_available,
		dx_raytracing_1_0,
		dx_raytracing_1_1
	};

	enum DLSS_status
	{
		DLSS_status_not_available,
		DLSS_status_FSR_wrapper,
		DLSS_status_3_0,
		DLSS_status_3_5
	};

	enum dx_mesh_shader_tier
	{
		dx_mesh_shader_not_available,
		dx_mesh_shader_1_0
	};

	enum dx_tiling_tier
	{
		dx_tiling_not_available,
		dx_tiling_1,
		dx_tiling_2,
		dx_tiling_3,
		dx_tiling_4
	};

	struct ERA_CORE_API dx_feature_support
	{
		dx_raytracing_tier raytracingTier = dx_raytracing_1_0;
		dx_mesh_shader_tier meshShaderTier = dx_mesh_shader_1_0;
		dx_tiling_tier tilingTier = dx_tiling_1;
		DLSS_status dlss_status = DLSS_status_FSR_wrapper;

		constexpr bool raytracing() const { return raytracingTier >= dx_raytracing_1_0; }
		constexpr bool meshShaders() const { return meshShaderTier >= dx_mesh_shader_1_0; }
		constexpr bool tiling() const { return tilingTier >= dx_tiling_1; }
		constexpr bool dlss() const { return dlss_status >= DLSS_status_FSR_wrapper; }
	};

	struct dx_context
	{
		bool initialize();
		void quit();

		void newFrame(uint64 frameID);
		void endFrame(dx_command_list* cl);
		void flushApplication();

		dx_command_list* getFreeCopyCommandList();
		dx_command_list* getFreeComputeCommandList(bool async);
		dx_command_list* getFreeRenderCommandList();
		uint64 executeCommandList(dx_command_list* commandList);
		uint64 executeCommandLists(dx_command_list** commandLists, uint32 count);

		// Careful with these functions. They are not thread safe.
		dx_allocation allocateDynamicBuffer(uint32 sizeInBytes, uint32 alignment = D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT);

		std::pair<dx_dynamic_constant_buffer, void*> uploadDynamicConstantBuffer(uint32 sizeInBytes, const void* data);
		template <typename T> dx_dynamic_constant_buffer uploadDynamicConstantBuffer(const T& data)
		{
			return uploadDynamicConstantBuffer(sizeof(T), &data).first;
		}

		std::pair<dx_dynamic_vertex_buffer, void*> createDynamicVertexBuffer(uint32 elementSize, uint32 elementCount);
		std::pair<dx_dynamic_index_buffer, void*> createDynamicIndexBuffer(uint32 elementSize, uint32 elementCount);

		dx_memory_usage getMemoryUsage();

		dx_factory factory;
		dx_adapter adapter;
		dx_device device;

		dx_command_queue renderQueue;
		dx_command_queue computeQueue;
		dx_command_queue copyQueue;

		dx_feature_support featureSupport;

		uint64 frameID;
		uint32 bufferedFrameID;

		dx_descriptor_heap srvUavAllocator;
		dx_descriptor_heap srvUavAllocatorShaderVisible; // Used for resources, which can be UAV cleared.

		dx_descriptor_heap rtvAllocator;
		dx_descriptor_heap dsvAllocator;

#if ENABLE_DX_PROFILING
		volatile uint32 timestampQueryIndex[NUM_BUFFERED_FRAMES];
#endif

		dx_upload_buffer frameUploadBuffer;

		dx_frame_descriptor_allocator frameDescriptorAllocator;

		volatile bool running = true;

		uint32 descriptorHandleIncrementSize;

		D3D12MA::Allocator* memoryAllocator;

		void retire(struct texture_grave&& texture);
		void retire(struct buffer_grave&& buffer);
		void retire(dx_object obj);
		void retire(D3D12MA::Allocation* allocation);

		dx_command_signature defaultDispatchCommandSignature;

	private:
#if ENABLE_DX_PROFILING
		dx_timestamp_query_heap timestampHeaps[NUM_BUFFERED_FRAMES];
		ref<dx_buffer> resolvedTimestampBuffers[NUM_BUFFERED_FRAMES];
#endif

		dx_page_pool pagePools[NUM_BUFFERED_FRAMES];

		std::mutex mutex;

		std::vector<struct texture_grave> textureGraveyard[NUM_BUFFERED_FRAMES];
		std::vector<struct buffer_grave> bufferGraveyard[NUM_BUFFERED_FRAMES];
		std::vector<dx_object> objectGraveyard[NUM_BUFFERED_FRAMES];
		std::vector<D3D12MA::Allocation*> allocationGraveyard[NUM_BUFFERED_FRAMES];

		dx_command_queue& getQueue(D3D12_COMMAND_LIST_TYPE type);
		dx_command_list* getFreeCommandList(dx_command_queue& queue);

		friend struct dlss_feature_adapter;
	};

	extern dx_context& dxContext;

	ERA_CORE_API dx_context* get_dx_context();

	ERA_CORE_API std::pair<dx_dynamic_vertex_buffer, void*> create_dynamic_vertex_buffer(uint32 element_size, uint32 element_count);
	ERA_CORE_API std::pair<dx_dynamic_index_buffer, void*> create_dynamic_index_buffer(uint32 element_size, uint32 element_count);

	ERA_CORE_API dx_memory_usage get_dx_mem_usage();
}