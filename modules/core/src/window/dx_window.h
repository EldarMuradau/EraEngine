// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#pragma once

#include "core_api.h"

#include "window/window.h"

#include "dx/dx.h"
#include "dx/dx_descriptor.h"

#include "rendering/render_utils.h"

#include <rttr/type>

namespace era_engine
{
	struct ERA_CORE_API dx_window : win32_window
	{
		dx_window();
		dx_window(dx_window&) = delete;
		dx_window(dx_window&&) = default;

		virtual ~dx_window();

		bool initialize(const TCHAR* name, uint32 requestedClientWidth, uint32 requestedClientHeight, color_depth colorDepth = color_depth_10, bool exclusiveFullscreen = false);
		bool initialize(HINSTANCE hInst, const TCHAR* name, uint32 requestedClientWidth, uint32 requestedClientHeight, color_depth colorDepth = color_depth_10, bool exclusiveFullscreen = false);

		virtual void shutdown();

		virtual void swapBuffers();
		virtual void toggleFullscreen();
		void toggleVSync();

		virtual void onResize();
		virtual void onMove();
		virtual void onWindowDisplayChange();

		dx_resource backBuffers[NUM_BUFFERED_FRAMES];
		dx_rtv_descriptor_handle backBufferRTVs[NUM_BUFFERED_FRAMES];
		uint32 currentBackbufferIndex;

		color_depth colorDepth;

		RTTR_ENABLE()

	private:
		void updateRenderTargetViews();

		dx_swapchain swapchain;
		com<ID3D12DescriptorHeap> rtvDescriptorHeap;

		bool tearingSupported;
		bool exclusiveFullscreen;
		bool hdrSupport;
		bool vSync = false;
		bool initialized = false;
	};

}