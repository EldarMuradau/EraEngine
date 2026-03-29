// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#include "dlss/dlss.h"

#include "dx/dx_command_list.h"

#include "core/project.h"
#include "core/log.h"

#include "rendering/main_renderer.h"

namespace era_engine::dlss
{
	bool checkDLSSStatus(IDXGIAdapter* adapter)
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

	void dlss_feature_adapter::initialize(main_renderer* rnd)
	{
#ifndef ENABLE_FSR_WRAPPER

		if (checkDLSSStatus(dxContext.adapter.Get()))
		{
			LOG_ERROR("Graphics> DLSS 3.5 is not supported by this GPU.");
			return;
		}

#endif

#if ENABLE_DLSS

		if (rnd->dlssInited)
			return;
		renderer = rnd;

		initializeDLSS();

		renderer->dlssInited = true;
#ifndef ENABLE_FSR_WRAPPER
		LOG_MESSAGE("Graphics> DLSS 3.5 Initialized succefffuly.");
#else
		LOG_MESSAGE("Graphics> FSR 2.0 Initialized succefffuly.");
#endif

		tonemapSettings.A = 0.191f;
		tonemapSettings.B = 0.082f;
		tonemapSettings.C = 1.0f;
		tonemapSettings.D = 0.5f;
		tonemapSettings.E = 0.082f;
		tonemapSettings.F = 0.255f;
		tonemapSettings.exposure = -0.06f;

#endif

	}

	static void NGXResourceAllocCallback(D3D12_RESOURCE_DESC* InDesc, int InState,
		CD3DX12_HEAP_PROPERTIES* InHeap, ID3D12Resource** OutResource)
	{

#if ENABLE_DLSS

		*OutResource = nullptr;
		HRESULT hr = dxContext.device->CreateCommittedResource(InHeap, D3D12_HEAP_FLAG_NONE,
			InDesc, (D3D12_RESOURCE_STATES)InState,
			nullptr, IID_PPV_ARGS(OutResource));
		LOG_MESSAGE("Graphics> DLSS 3.5 Allocation.");
		if (hr != S_OK)
		{
			LOG_ERROR("Graphics> DLSS allocation failed.");
		}

#endif

	}

	void dlss_feature_adapter::updateDLSS(ID3D12GraphicsCommandList* cmdList, float dt)
	{

#if ENABLE_DLSS

		renderer->settings.tonemapSettings = tonemapSettings;
		NVSDK_NGX_D3D12_Feature_Eval_Params featureEvalsParams{};
		featureEvalsParams.pInOutput = renderer->frameResult->resource.Get();
		featureEvalsParams.pInColor = renderer->ldrPostProcessingTexture->resource.Get();
		featureEvalsParams.InSharpness = 0.15f;

		NVSDK_NGX_Dimensions dim{};
		dim.Width = renderer->renderWidth;
		dim.Height = renderer->renderHeight;

		NVSDK_NGX_D3D12_DLSS_Eval_Params evalsParams{};
		evalsParams.Feature = featureEvalsParams;
		evalsParams.InRenderSubrectDimensions = dim;
		evalsParams.InJitterOffsetX = renderer->windowXOffset;
		evalsParams.InJitterOffsetY = renderer->windowYOffset;
		evalsParams.InExposureScale = renderer->settings.tonemapSettings.exposure;
		evalsParams.pInMotionVectors = renderer->screenVelocitiesTexture->resource.Get();
		evalsParams.InMVScaleX = renderer->screenVelocitiesTexture->width;
		evalsParams.InMVScaleY = renderer->screenVelocitiesTexture->height;
		evalsParams.pInDepth = renderer->opaqueDepthBuffer->resource.Get();
		evalsParams.InRenderSubrectDimensions.Width = renderer->renderWidth;
		evalsParams.InRenderSubrectDimensions.Height = renderer->renderHeight;
		evalsParams.InPreExposure = 0.10f;
		evalsParams.InFrameTimeDeltaInMsec = dt;

		NVSDK_NGX_Result EvalDLSSFeatureRes = NGX_D3D12_EVALUATE_DLSS_EXT(cmdList, handle, params, &evalsParams);

#endif

	}

	static void NGXResourceReleaseCallback(IUnknown* InResource)
	{
		SAFE_RELEASE(InResource);
	}

	dlss_feature_adapter::~dlss_feature_adapter()
	{
#if ENABLE_DLSS
		NVSDK_NGX_D3D12_Shutdown1(dxContext.device.Get());
		NVSDK_NGX_D3D12_DestroyParameters(params);
#endif
	}

	void dlss_feature_adapter::initializeDLSS()
	{

#if ENABLE_DLSS

		auto res = NVSDK_NGX_D3D12_Init_with_ProjectID("FAF85B49-66B9-D77F-2FF4-20FD9B731C5C", NVSDK_NGX_ENGINE_TYPE_CUSTOM,
			"0.1", L"logs", dxContext.device.Get());

		NVSDK_NGX_Result CapParRes = NVSDK_NGX_D3D12_GetCapabilityParameters(&params);
		params->Set(NVSDK_NGX_Parameter_ResourceAllocCallback, NGXResourceAllocCallback);
		params->Set(NVSDK_NGX_Parameter_ResourceReleaseCallback, NGXResourceReleaseCallback);

		unsigned int outRenderMaxWidth = 0;
		unsigned int outRenderMaxHeight = 0;
		unsigned int outRenderMinWidth = 0;
		unsigned int outRenderMinHeight = 0;
		float outSharpness = 0.0f;
		unsigned int w = renderer->windowWidth;
		unsigned int h = renderer->windowHeight;
		NVSDK_NGX_Result optSetRes = NGX_DLSS_GET_OPTIMAL_SETTINGS(params, w, h, NVSDK_NGX_PerfQuality_Value_Balanced,
			&renderOptimalWidth,
			&renderOptimalHeight,
			&outRenderMaxWidth,
			&outRenderMaxHeight,
			&outRenderMinWidth,
			&outRenderMinHeight,
			&outSharpness);

		NVSDK_NGX_DLSS_Create_Params dlss_c_p{};
		dlss_c_p.InFeatureCreateFlags = NVSDK_NGX_DLSS_Feature_Flags_IsHDR
			| NVSDK_NGX_DLSS_Feature_Flags_DoSharpening
			| NVSDK_NGX_DLSS_Feature_Flags_MVLowRes
			| NVSDK_NGX_DLSS_Feature_Flags_MVJittered
			| NVSDK_NGX_DLSS_Feature_Flags_DepthInverted;
		dlss_c_p.InEnableOutputSubrects = false;
		dlss_c_p.Feature.InWidth = outRenderMaxWidth;
		dlss_c_p.Feature.InHeight = outRenderMaxHeight;
		dlss_c_p.Feature.InTargetWidth = renderer->renderWidth;
		dlss_c_p.Feature.InTargetHeight = renderer->renderHeight;
		dlss_c_p.Feature.InPerfQualityValue = NVSDK_NGX_PerfQuality_Value_Balanced;
		NVSDK_NGX_Result CreatDLSSExtRes = NGX_D3D12_CREATE_DLSS_EXT(dxContext.getFreeRenderCommandList()->commandList.Get(), 1, 1, &handle, params, &dlss_c_p);

#endif

	}

}