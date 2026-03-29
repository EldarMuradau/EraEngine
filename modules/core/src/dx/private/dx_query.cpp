// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#include "dx/dx_query.h"
#include "dx/dx_context.h"

namespace era_engine
{
	void dx_timestamp_query_heap::initialize(uint32 maxCount)
	{
		D3D12_QUERY_HEAP_DESC desc;
		desc.Type = D3D12_QUERY_HEAP_TYPE_TIMESTAMP;
		desc.Count = maxCount;
		desc.NodeMask = 0;

		checkResult(dxContext.device->CreateQueryHeap(&desc, IID_PPV_ARGS(&heap)));
	}
}