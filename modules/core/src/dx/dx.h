// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#pragma once

#include <dx/d3dx12.h>
#include <dxgi1_6.h>
#include <d3dcompiler.h>
#include <sdkddkver.h>

#define USE_D3D12_BLOCK_ALLOCATOR 1
namespace D3D12MA
{
    class Allocator;
    class Allocation;
};

#ifdef __ID3D12GraphicsCommandList6_INTERFACE_DEFINED__
#define SDK_SUPPORTS_MESH_SHADERS
#endif

namespace era_engine
{
    typedef com<ID3D12Object> DXObject;
    typedef com<IDXGIAdapter4> DXAdapter;
    typedef com<ID3D12Device5> DXDevice;
    typedef com<IDXGIFactory4> DXFactory;
    typedef com<IDXGISwapChain4> DXSwapchain;
    typedef com<ID3D12Resource> DXResource;
    typedef com<ID3D12CommandAllocator> DXCommandAllocator;
    typedef com<ID3DBlob> DXBlob;
    typedef com<ID3D12PipelineState> DXPipelineState;
    typedef com<ID3D12Resource> DXResource;
    typedef com<ID3D12CommandSignature> DXCommandSignature;
    typedef com<ID3D12Heap> DXHeap;
    typedef com<ID3D12StateObject> DXRaytracingPipelineState;
    typedef com<ID3D12QueryHeap> DXQueryHeap;

#ifdef SDK_SUPPORTS_MESH_SHADERS
    typedef com<ID3D12GraphicsCommandList6> DXGraphicsCommandList;
#else
    typedef com<ID3D12GraphicsCommandList4> DXGraphicsCommandList;
#endif
}

#define NUM_BUFFERED_FRAMES 3

#define SET_NAME(obj, name) checkResult(obj->SetName(L##name));