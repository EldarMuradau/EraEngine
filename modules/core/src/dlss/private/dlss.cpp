// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#include "dlss/dlss.h"

#include "dx/dx_command_list.h"

#include "core/project.h"
#include "core/log.h"

#include "rendering/main_renderer.h"

namespace era_engine::dlss
{
    bool check_dlss_status(IDXGIAdapter* adapter)
    {
#if ENABLE_DLSS

        NVSDK_NGX_FeatureDiscoveryInfo dlssInfo{};
        NVSDK_NGX_FeatureRequirement outSupported{};
        dlssInfo.FeatureID = NVSDK_NGX_Feature_SuperSampling;
        dlssInfo.ApplicationDataPath = (eproject::enginePath + std::wstring(L"/logs")).c_str();
        dlssInfo.Identifier.v.ProjectDesc.EngineType = NVSDK_NGX_ENGINE_TYPE_CUSTOM;

        if (NVSDK_NGX_D3D12_GetFeatureRequirements(adapter, &dlssInfo, &outSupported) != NVSDK_NGX_Result_Success)
        {
            std::cerr << "No DLSS 3.5 capable GPU/Software found.\n";
            return false;
        }
        if (outSupported.FeatureSupported != NVSDK_NGX_FeatureSupportResult_Supported)
        {
            std::cerr << "No capable GPU/Software for current DLSS 3.5 feature found.\n";
            return false;
        }

        return true;

#else

        return false;

#endif
    }

    void DLSSFeatureAdapter::initialize(MainRenderer* rnd)
    {
#ifndef ENABLE_FSR_WRAPPER

        if (check_dlss_status(dx_context.adapter.Get()))
        {
            LOG_ERROR("Graphics> DLSS 3.5 is not supported by this GPU.");
            return;
        }

#endif

#if ENABLE_DLSS

        if (rnd->dlss_inited)
            return;
        renderer = rnd;

        initialize_dlss();

        renderer->dlss_inited = true;
#ifndef ENABLE_FSR_WRAPPER
        LOG_MESSAGE("Graphics> DLSS 3.5 Initialized succefffuly.");
#else
        LOG_MESSAGE("Graphics> FSR 2.0 Initialized succefffuly.");
#endif

        tonemap_settings.A = 0.191f;
        tonemap_settings.B = 0.082f;
        tonemap_settings.C = 1.0f;
        tonemap_settings.D = 0.5f;
        tonemap_settings.E = 0.082f;
        tonemap_settings.F = 0.255f;
        tonemap_settings.exposure = -0.06f;

#endif
    }

    static void ngx_resource_alloc_callback(D3D12_RESOURCE_DESC* in_desc, int in_state,
        CD3DX12_HEAP_PROPERTIES* in_heap, ID3D12Resource** out_resource)
    {

#if ENABLE_DLSS

        *OutResource = nullptr;
        HRESULT hr = dx_context.device->CreateCommittedResource(InHeap, D3D12_HEAP_FLAG_NONE,
            InDesc, (D3D12_RESOURCE_STATES)InState,
            nullptr, IID_PPV_ARGS(OutResource));
        LOG_MESSAGE("Graphics> DLSS 3.5 Allocation.");
        if (hr != S_OK)
        {
            LOG_ERROR("Graphics> DLSS allocation failed.");
        }

#endif
    }

    void DLSSFeatureAdapter::update_dlss(ID3D12GraphicsCommandList* cmd_list, float dt)
    {

#if ENABLE_DLSS

        renderer->frame_settings.tonemap_settings = tonemap_settings;
        NVSDK_NGX_D3D12_Feature_Eval_Params featureEvalsParams{};
        featureEvalsParams.pInOutput = renderer->frame_result->resource.Get();
        featureEvalsParams.pInColor = renderer->ldr_post_processing_texture->resource.Get();
        featureEvalsParams.InSharpness = 0.15f;

        NVSDK_NGX_Dimensions dim{};
        dim.Width = renderer->render_width;
        dim.Height = renderer->render_height;

        NVSDK_NGX_D3D12_DLSS_Eval_Params evalsParams{};
        evalsParams.Feature = featureEvalsParams;
        evalsParams.InRenderSubrectDimensions = dim;
        evalsParams.InJitterOffsetX = renderer->window_x_offset;
        evalsParams.InJitterOffsetY = renderer->window_y_offset;
        evalsParams.InExposureScale = renderer->settings.tonemap_settings.exposure;
        evalsParams.pInMotionVectors = renderer->screen_velocities_texture->resource.Get();
        evalsParams.InMVScaleX = renderer->screen_velocities_texture->width;
        evalsParams.InMVScaleY = renderer->screen_velocities_texture->height;
        evalsParams.pInDepth = renderer->opaque_depth_buffer->resource.Get();
        evalsParams.InRenderSubrectDimensions.Width = renderer->render_width;
        evalsParams.InRenderSubrectDimensions.Height = renderer->render_height;
        evalsParams.InPreExposure = 0.10f;
        evalsParams.InFrameTimeDeltaInMsec = dt;

        NVSDK_NGX_Result EvalDLSSFeatureRes = NGX_D3D12_EVALUATE_DLSS_EXT(cmd_list, handle, params, &evalsParams);

#endif
    }

    static void ngx_resource_release_callback(IUnknown* in_resource)
    {
        SAFE_RELEASE(in_resource);
    }

    DLSSFeatureAdapter::~DLSSFeatureAdapter()
    {
#if ENABLE_DLSS
        NVSDK_NGX_D3D12_Shutdown1(dx_context.device.Get());
        NVSDK_NGX_D3D12_DestroyParameters(params);
#endif
    }

    void DLSSFeatureAdapter::initialize_dlss()
    {

#if ENABLE_DLSS

        auto res = NVSDK_NGX_D3D12_Init_with_ProjectID("FAF85B49-66B9-D77F-2FF4-20FD9B731C5C", NVSDK_NGX_ENGINE_TYPE_CUSTOM,
            "0.1", L"logs", dx_context.device.Get());

        NVSDK_NGX_Result CapParRes = NVSDK_NGX_D3D12_GetCapabilityParameters(&params);
        params->Set(NVSDK_NGX_Parameter_ResourceAllocCallback, ngx_resource_alloc_callback);
        params->Set(NVSDK_NGX_Parameter_ResourceReleaseCallback, ngx_resource_release_callback);

        unsigned int outRenderMaxWidth = 0;
        unsigned int outRenderMaxHeight = 0;
        unsigned int outRenderMinWidth = 0;
        unsigned int outRenderMinHeight = 0;
        float outSharpness = 0.0f;
        unsigned int w = renderer->window_width;
        unsigned int h = renderer->window_height;
        NVSDK_NGX_Result optSetRes = NGX_DLSS_GET_OPTIMAL_SETTINGS(params, w, h, NVSDK_NGX_PerfQuality_Value_Balanced,
            &render_optimal_width,
            &render_optimal_height,
            &outRenderMaxWidth,
            &outRenderMaxHeight,
            &outRenderMinWidth,
            &outRenderMinHeight,
            &outSharpness);

        NVSDK_NGX_DLSS_Create_Params dlss_c_p{};
        dlss_c_p.InFeatureCreateFlags = NVSDK_NGX_DLSS_Feature_Flags_IsHDR | NVSDK_NGX_DLSS_Feature_Flags_DoSharpening | NVSDK_NGX_DLSS_Feature_Flags_MVLowRes | NVSDK_NGX_DLSS_Feature_Flags_MVJittered | NVSDK_NGX_DLSS_Feature_Flags_DepthInverted;
        dlss_c_p.InEnableOutputSubrects = false;
        dlss_c_p.Feature.InWidth = outRenderMaxWidth;
        dlss_c_p.Feature.InHeight = outRenderMaxHeight;
        dlss_c_p.Feature.InTargetWidth = renderer->render_width;
        dlss_c_p.Feature.InTargetHeight = renderer->render_height;
        dlss_c_p.Feature.InPerfQualityValue = NVSDK_NGX_PerfQuality_Value_Balanced;
        NVSDK_NGX_Result CreatDLSSExtRes = NGX_D3D12_CREATE_DLSS_EXT(dx_context.get_free_render_command_list()->command_list.Get(), 1, 1, &handle, params, &dlss_c_p);

#endif
    }

}