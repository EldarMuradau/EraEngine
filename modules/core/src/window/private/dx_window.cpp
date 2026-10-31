// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#include "window/dx_window.h"

#include "dx/dx_context.h"
#include "core/cpu_profiling.h"

#include "engine/engine.h"

#include <core/debug/debug_var.h>

#include <rttr/registration>

namespace era_engine
{
    static DebugVar<bool> enable_vsynk = DebugVar<bool>("engine.enable_vsynk", false);

    RTTR_REGISTRATION
    {
        using namespace rttr;

        registration::class_<DXWindow>("DXWindow")
            .constructor<>();
    }

    static bool check_tearing_support(DXFactory factory)
    {
        BOOL allow_tearing = FALSE;

        // Rather than create the DXGI 1.5 factory interface directly, we create the
        // DXGI 1.4 interface and query for the 1.5 interface. This is to enable the
        // graphics debugging tools which will not support the 1.5 factory interface
        // until a future update
        com<IDXGIFactory5> factory5;
        if (SUCCEEDED(factory.As(&factory5)))
        {
            if (FAILED(factory5->CheckFeatureSupport(
                    DXGI_FEATURE_PRESENT_ALLOW_TEARING,
                    &allow_tearing, sizeof(allow_tearing))))
            {
                allow_tearing = FALSE;
            }
        }

        return allow_tearing == TRUE;
    }

    static DXSwapchain create_swap_chain(HWND window_handle,
        DXFactory factory, const DXCommandQueue& command_queue,
        uint32 width, uint32 height, uint32 buffer_count, bool tearing_supported, ColorDepth color_depth, bool exclusive_fullscreen)
    {
        DXSwapchain dxgi_swap_chain;

        DXGI_SWAP_CHAIN_DESC1 swap_chain_desc = {};
        swap_chain_desc.Width = width;
        swap_chain_desc.Height = height;
        swap_chain_desc.Format = color_depth_to_format(color_depth);
        swap_chain_desc.Stereo = FALSE;
        swap_chain_desc.SampleDesc = {1, 0};
        swap_chain_desc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
        swap_chain_desc.BufferCount = buffer_count;
        swap_chain_desc.Scaling = DXGI_SCALING_STRETCH;
        swap_chain_desc.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD;
        swap_chain_desc.AlphaMode = DXGI_ALPHA_MODE_UNSPECIFIED;

        // It is recommended to always allow tearing if tearing support is available
        swap_chain_desc.Flags = tearing_supported ? DXGI_SWAP_CHAIN_FLAG_ALLOW_TEARING : 0;

        com<IDXGISwapChain1> swap_chain1;
        checkResult(factory->CreateSwapChainForHwnd(
            command_queue.command_queue.Get(),
            window_handle,
            &swap_chain_desc,
            0,
            0,
            &swap_chain1));

        UINT flags = 0;
        if (!exclusive_fullscreen)
        {
            // Disable the Alt+Enter fullscreen toggle feature. Switching to fullscreen
            // will be handled manually
            flags = DXGI_MWA_NO_ALT_ENTER;
        }
        checkResult(factory->MakeWindowAssociation(window_handle, flags));

        checkResult(swap_chain1.As(&dxgi_swap_chain));

        return dxgi_swap_chain;
    }

    static int32 compute_intersection_area(int32 ax1, int32 ay1, int32 ax2, int32 ay2, int32 bx1, int32 by1, int32 bx2, int32 by2)
    {
        return max(0, min(ax2, bx2) - max(ax1, bx1)) * max(0, min(ay2, by2) - max(ay1, by1));
    }

    static bool check_for_hdr_support(DXFactory factory, RECT window_rect, ColorDepth color_depth)
    {
        if (color_depth == COLOR_DEPTH_8)
            return false;

        com<IDXGIAdapter1> dxgi_adapter;
        checkResult(factory->EnumAdapters1(0, &dxgi_adapter));

        uint32 i = 0;
        com<IDXGIOutput> current_output;
        com<IDXGIOutput> best_output;
        int32 best_intersect_area = -1;

        while (dxgi_adapter->EnumOutputs(i, &current_output) != DXGI_ERROR_NOT_FOUND)
        {
            // Get the retangle bounds of the app window.
            int ax1 = window_rect.left;
            int ay1 = window_rect.top;
            int ax2 = window_rect.right;
            int ay2 = window_rect.bottom;

            // Get the rectangle bounds of current output.
            DXGI_OUTPUT_DESC desc;
            checkResult(current_output->GetDesc(&desc));
            RECT r = desc.DesktopCoordinates;
            int bx1 = r.left;
            int by1 = r.top;
            int bx2 = r.right;
            int by2 = r.bottom;

            // Compute the intersection.
            int32 intersect_area = compute_intersection_area(ax1, ay1, ax2, ay2, bx1, by1, bx2, by2);
            if (intersect_area > best_intersect_area)
            {
                best_output = current_output;
                best_intersect_area = intersect_area;
            }

            ++i;
        }

        com<IDXGIOutput6> output6;
        checkResult(best_output.As(&output6));

        DXGI_OUTPUT_DESC1 desc1;
        checkResult(output6->GetDesc1(&desc1));

        return desc1.ColorSpace == DXGI_COLOR_SPACE_RGB_FULL_G2084_NONE_P2020;
    }

    static void set_swap_chain_color_space(DXSwapchain swapchain, ColorDepth color_depth, bool hdr_support)
    {
        // Rec2020 is the standard for UHD displays. The tonemap shader needs to apply the ST2084 curve before display.
        // Rec709 is the same as sRGB, just without the gamma curve. The tonemap shader needs to apply the gamma curve before display.
        DXGI_COLOR_SPACE_TYPE color_space = (hdr_support && color_depth == COLOR_DEPTH_10) ? DXGI_COLOR_SPACE_RGB_FULL_G2084_NONE_P2020 : DXGI_COLOR_SPACE_RGB_FULL_G22_NONE_P709;
        UINT color_space_support = 0;
        if (SUCCEEDED(swapchain->CheckColorSpaceSupport(color_space, &color_space_support)) &&
            ((color_space_support & DXGI_SWAP_CHAIN_COLOR_SPACE_SUPPORT_FLAG_PRESENT) == DXGI_SWAP_CHAIN_COLOR_SPACE_SUPPORT_FLAG_PRESENT))
        {
            checkResult(swapchain->SetColorSpace1(color_space));
        }

        if (!hdr_support)
        {
            checkResult(swapchain->SetHDRMetaData(DXGI_HDR_METADATA_TYPE_NONE, 0, 0));
            return;
        }

        struct DisplayChromaticities
        {
            float red_x;
            float red_y;
            float green_x;
            float green_y;
            float blue_x;
            float blue_y;
            float white_x;
            float white_y;
        };

        static const DisplayChromaticities chroma =
            {
                0.70800f, 0.29200f, 0.17000f, 0.79700f, 0.13100f, 0.04600f, 0.31270f, 0.32900f // Display Gamut Rec2020
            };

        float max_output_nits = 1000.f;
        float min_output_nits = 0.001f;
        float max_cll = 2000.f;
        float max_fall = 500.f;

        DXGI_HDR_METADATA_HDR10 hdr10_meta_data = {};
        hdr10_meta_data.RedPrimary[0] = (uint16)(chroma.red_x * 50000.f);
        hdr10_meta_data.RedPrimary[1] = (uint16)(chroma.red_y * 50000.f);
        hdr10_meta_data.GreenPrimary[0] = (uint16)(chroma.green_x * 50000.f);
        hdr10_meta_data.GreenPrimary[1] = (uint16)(chroma.green_y * 50000.f);
        hdr10_meta_data.BluePrimary[0] = (uint16)(chroma.blue_x * 50000.f);
        hdr10_meta_data.BluePrimary[1] = (uint16)(chroma.blue_y * 50000.f);
        hdr10_meta_data.WhitePoint[0] = (uint16)(chroma.white_x * 50000.f);
        hdr10_meta_data.WhitePoint[1] = (uint16)(chroma.white_y * 50000.f);
        hdr10_meta_data.MaxMasteringLuminance = (uint32)(max_output_nits * 10000.f);
        hdr10_meta_data.MinMasteringLuminance = (uint32)(min_output_nits * 10000.f);
        hdr10_meta_data.MaxContentLightLevel = (uint16)(max_cll);
        hdr10_meta_data.MaxFrameAverageLightLevel = (uint16)(max_fall);

        checkResult(swapchain->SetHDRMetaData(DXGI_HDR_METADATA_TYPE_HDR10, sizeof(DXGI_HDR_METADATA_HDR10), &hdr10_meta_data));
    }

    void DXWindow::update_render_target_views()
    {
        uint32 rtv_descriptor_size = dx_context.device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_RTV);

        CD3DX12_CPU_DESCRIPTOR_HANDLE rtv_handle(rtv_descriptor_heap->GetCPUDescriptorHandleForHeapStart());

        for (int i = 0; i < NUM_BUFFERED_FRAMES; ++i)
        {
            DXResource back_buffer;
            checkResult(swapchain->GetBuffer(i, IID_PPV_ARGS(&back_buffer)));

            dx_context.device->CreateRenderTargetView(back_buffer.Get(), 0, rtv_handle);

            back_buffers[i] = back_buffer;
            back_buffer_rtvs[i] = rtv_handle;

            SET_NAME(back_buffers[i], "Backbuffer");

            rtv_handle.Offset(rtv_descriptor_size);
        }
    }

    DXWindow::DXWindow()
    {
        v_sync = enable_vsynk;
    }

    DXWindow::~DXWindow()
    {
        dx_context.flush_application();

        shutdown();
    }

    bool DXWindow::initialize(const TCHAR* name, uint32 requested_client_width, uint32 requested_client_height, ColorDepth color_depth, bool exclusive_fullscreen)
    {
        if (!Win32Window::initialize(name, requested_client_width, requested_client_height))
            return false;

        this->color_depth = color_depth;
        this->exclusive_fullscreen = exclusive_fullscreen;
        tearing_supported = check_tearing_support(dx_context.factory);

        swapchain = create_swap_chain(window_handle, dx_context.factory, dx_context.render_queue, client_width, client_height, NUM_BUFFERED_FRAMES, tearing_supported, color_depth, exclusive_fullscreen);
        current_backbuffer_index = swapchain->GetCurrentBackBufferIndex();

        D3D12_DESCRIPTOR_HEAP_DESC descriptor_heap_desc = {};
        descriptor_heap_desc.NumDescriptors = NUM_BUFFERED_FRAMES;
        descriptor_heap_desc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;

        checkResult(dx_context.device->CreateDescriptorHeap(&descriptor_heap_desc, IID_PPV_ARGS(&rtv_descriptor_heap)));

        RECT window_rect = {0, 0, (LONG)client_width, (LONG)client_height};
        AdjustWindowRect(&window_rect, WS_OVERLAPPEDWINDOW, FALSE);
        GetWindowRect(window_handle, &window_rect);
        hdr_support = check_for_hdr_support(dx_context.factory, window_rect, color_depth);
        set_swap_chain_color_space(swapchain, color_depth, hdr_support);

        update_render_target_views();

        initialized = true;

        return true;
    }

    bool DXWindow::initialize(HINSTANCE h_inst, const TCHAR* name, uint32 requested_client_width, uint32 requested_client_height, ColorDepth color_depth, bool exclusive_fullscreen)
    {
        if (!Win32Window::initialize(h_inst, name, requested_client_width, requested_client_height))
            return false;

        this->color_depth = color_depth;
        this->exclusive_fullscreen = exclusive_fullscreen;
        tearing_supported = check_tearing_support(dx_context.factory);

        swapchain = create_swap_chain(window_handle, dx_context.factory, dx_context.render_queue, client_width, client_height, NUM_BUFFERED_FRAMES, tearing_supported, color_depth, exclusive_fullscreen);
        current_backbuffer_index = swapchain->GetCurrentBackBufferIndex();

        D3D12_DESCRIPTOR_HEAP_DESC descriptor_heap_desc = {};
        descriptor_heap_desc.NumDescriptors = NUM_BUFFERED_FRAMES;
        descriptor_heap_desc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;

        checkResult(dx_context.device->CreateDescriptorHeap(&descriptor_heap_desc, IID_PPV_ARGS(&rtv_descriptor_heap)));

        RECT window_rect = {0, 0, (LONG)client_width, (LONG)client_height};
        AdjustWindowRect(&window_rect, WS_OVERLAPPEDWINDOW, FALSE);
        GetWindowRect(window_handle, &window_rect);
        hdr_support = check_for_hdr_support(dx_context.factory, window_rect, color_depth);
        set_swap_chain_color_space(swapchain, color_depth, hdr_support);

        update_render_target_views();

        initialized = true;

        return true;
    }

    void DXWindow::shutdown()
    {
        // Flush the GPU queue to make sure the swap chain's back buffers
        // are not being referenced by an in-flight command list
        dx_context.flush_application();

        for (uint32 i = 0; i < NUM_BUFFERED_FRAMES; ++i)
        {
            back_buffers[i].Reset();
        }
        rtv_descriptor_heap.Reset();
        swapchain.Reset();

        initialized = false;
        Win32Window::shutdown();
    }

    void DXWindow::on_resize()
    {
        if (initialized)
        {
            // The executing frame may be recording into or presenting the back buffers.
            wait_for_frame_execution();

            // Flush the GPU queue to make sure the swap chain's back buffers
            // are not being referenced by an in-flight command list.
            dx_context.flush_application();

            for (uint32 i = 0; i < NUM_BUFFERED_FRAMES; ++i)
            {
                back_buffers[i].Reset();
            }

            DXGI_SWAP_CHAIN_DESC swap_chain_desc = {};
            checkResult(swapchain->GetDesc(&swap_chain_desc));
            checkResult(swapchain->ResizeBuffers(NUM_BUFFERED_FRAMES, client_width, client_height,
                swap_chain_desc.BufferDesc.Format, swap_chain_desc.Flags));

            RECT window_rect;
            GetWindowRect(window_handle, &window_rect);
            hdr_support = check_for_hdr_support(dx_context.factory, window_rect, color_depth);
            set_swap_chain_color_space(swapchain, color_depth, hdr_support);

            current_backbuffer_index = swapchain->GetCurrentBackBufferIndex();

            update_render_target_views();
        }
    }

    void DXWindow::on_move()
    {
        if (initialized)
        {
            RECT window_rect;
            GetWindowRect(window_handle, &window_rect);
            hdr_support = check_for_hdr_support(dx_context.factory, window_rect, color_depth);
            set_swap_chain_color_space(swapchain, color_depth, hdr_support);
        }
    }

    void DXWindow::on_window_display_change()
    {
        if (initialized)
        {
            RECT window_rect;
            GetWindowRect(window_handle, &window_rect);
            hdr_support = check_for_hdr_support(dx_context.factory, window_rect, color_depth);
            set_swap_chain_color_space(swapchain, color_depth, hdr_support);
        }
    }

    void DXWindow::swap_buffers()
    {
        ZoneScopedN("DXWindow::present");

        if (initialized)
        {
            uint32 sync_interval = v_sync ? 1 : 0;
            uint32 present_flags = tearing_supported && !v_sync && !exclusive_fullscreen ? DXGI_PRESENT_ALLOW_TEARING : 0;
            checkResult(swapchain->Present(sync_interval, present_flags));

            current_backbuffer_index = swapchain->GetCurrentBackBufferIndex();
        }
    }

    void DXWindow::toggle_fullscreen()
    {
        if (exclusive_fullscreen)
        {
            fullscreen = !fullscreen;
            swapchain->SetFullscreenState(fullscreen, 0);
        }
        else
        {
            Win32Window::toggle_fullscreen();
        }
    }

    void DXWindow::toggle_v_sync()
    {
        v_sync = !v_sync;
        enable_vsynk = v_sync;
    }
}