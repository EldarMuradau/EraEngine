// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#pragma once

#include "core/math.h"
#include "core/camera.h"
#include "core/input.h"

#include "rendering/pbr.h"
#include "rendering/render_pass.h"
#include "rendering/light_source.h"
#include "rendering/raytracer.h"
#include "rendering/render_algorithms.h"
#include "rendering/render_resources.h"
#include "rendering/render_utils.h"
#include "rendering/path_tracing.h"
#include "rendering/light_probe.h"
#include "rendering/pbr_environment.h"

#include "dlss/dlss.h"

#include "light_source.hlsli"
#include "camera.hlsli"

#define MAX_NUM_SUN_LIGHT_SHADOW_PASSES 16
#define MAX_NUM_SPOT_LIGHT_SHADOW_PASSES 16
#define MAX_NUM_POINT_LIGHT_SHADOW_PASSES 16

namespace era_engine
{
    struct RendererSettings
    {
        TonemapSettings tonemap_settings{};
        TonemapSettings default_tonemap_settings{};

        HBAOSettings ao_settings;

        SSSSettings sss_settings;

        SSRSettings ssr_settings;

        TAASettings taa_settings;

        BloomSettings bloom_settings;

        SharpenSettings sharpen_settings;

        bool enable_ao = true;
        bool enable_sss = true;
        bool enable_ssr = true;
        bool enable_taa = true;
        bool enable_dlss = false;
        bool enable_sharpen = true;
        bool cache_shadow_map = false;
        bool enable_bloom = true;
    };
    REFLECT_STRUCT(RendererSettings,
        (tonemap_settings, "Tonemap"),
        (enable_ao, "Enable AO"),
        (ao_settings, "AO"),
        (enable_sss, "Enable SSS"),
        (sss_settings, "SSS"),
        (enable_ssr, "Enable SSR"),
        (ssr_settings, "SSR"),
        (enable_taa, "Enable TAA"),
        (taa_settings, "TAA"),
        (enable_bloom, "Enable Bloom"),
        (bloom_settings, "Bloom"),
        (enable_sharpen, "Enable sharpen"),
        (sharpen_settings, "Sharpen"));

    enum AspectRatioMode
    {
        ASPECT_RATIO_FREE,
        ASPECT_RATIO_FIX_16_9,
        ASPECT_RATIO_FIX_16_10,

        ASPECT_RATIO_MODE_COUNT,
    };

    static const char* aspect_ratio_names[] =
        {
            "Free",
            "16:9",
            "16:10",
    };

    enum RendererMode
    {
        RENDERER_MODE_RASTERIZED,
        RENDERER_MODE_VISUALIZE_SUN_SHADOW_CASCADES,
        RENDERER_MODE_PATHTRACED,

        RENDERER_MODE_COUNT,
    };

    static const char* renderer_mode_names[] =
        {
            "Rasterized",
            "Visualize sun shadow cascades",
            "Path-traced",
    };

    struct RendererSpec
    {
        bool allow_object_picking = true;
        bool allow_ao = true;
        bool allow_sss = true;
        bool allow_ssr = true;
        bool allow_taa = true;
        bool allow_bloom = true;
        bool allow_dlss = true;
    };

    struct MainRendererFrameInputs
    {
        const SunShadowRenderPass* sun_shadow_render_passes[MAX_NUM_SUN_LIGHT_SHADOW_PASSES]{};
        const SpotShadowRenderPass* spot_light_shadow_render_passes[MAX_NUM_SPOT_LIGHT_SHADOW_PASSES]{};
        const PointShadowRenderPass* point_light_shadow_render_passes[MAX_NUM_POINT_LIGHT_SHADOW_PASSES]{};

        uint32 num_sun_light_shadow_render_passes{};
        uint32 num_spot_light_shadow_render_passes{};
        uint32 num_point_light_shadow_render_passes{};

        RaytracingTLAS* tlas = nullptr;
        const PBREnvironment* environment = nullptr;

        DirectionalLightCB sun{};

        ref<DXBuffer> point_lights;
        ref<DXBuffer> spot_lights;
        ref<DXBuffer> point_light_shadow_info_buffer;
        ref<DXBuffer> spot_light_shadow_info_buffer;
        ref<DXBuffer> decals;

        uint32 num_point_lights{};
        uint32 num_spot_lights{};
        uint32 num_decals{};

        ref<DXTexture> decal_texture_atlas;

        const OpaqueRenderPass* opaque_render_pass = nullptr;
        const TransparentRenderPass* transparent_render_pass = nullptr;
        const LDRRenderPass* ldr_render_pass = nullptr;
        const ComputePass* compute_pass = nullptr;

        CameraCB jittered_camera{};
        CameraCB unjittered_camera{};

        uint32 requested_width{};
        uint32 requested_height{};

        RendererSettings frame_settings{};
        RendererMode frame_mode = RENDERER_MODE_RASTERIZED;
        AspectRatioMode frame_aspect_ratio_mode = ASPECT_RATIO_FREE;
    };

    struct MainRenderer : private MainRendererFrameInputs
    {
        MainRenderer() = default;
        void initialize(ColorDepth color_depth, uint32 window_width, uint32 window_height, RendererSpec spec, bool init_dlss = false);

        void begin_frame(uint32 window_width, uint32 window_height);
        void end_collection();
        void end_frame(const UserInput* input);

        void set_camera(const RenderCamera& camera);

        void dlss_pass(float dt);

        void submit_render_pass(OpaqueRenderPass* render_pass)
        {
            ASSERT(!collected().opaque_render_pass);
            collected().opaque_render_pass = render_pass;
        }

        void submit_render_pass(TransparentRenderPass* render_pass)
        {
            ASSERT(!collected().transparent_render_pass);
            collected().transparent_render_pass = render_pass;
        }

        void submit_render_pass(LDRRenderPass* render_pass)
        {
            ASSERT(!collected().ldr_render_pass);
            collected().ldr_render_pass = render_pass;
        }

        void submit_compute_pass(ComputePass* compute_pass)
        {
            ASSERT(!collected().compute_pass);
            collected().compute_pass = compute_pass;
        }

        void submit_shadow_render_pass(SunShadowRenderPass* render_pass)
        {
            MainRendererFrameInputs& inputs = collected();
            ASSERT(inputs.num_sun_light_shadow_render_passes < MAX_NUM_SUN_LIGHT_SHADOW_PASSES);
            inputs.sun_shadow_render_passes[inputs.num_sun_light_shadow_render_passes++] = render_pass;
        }

        void submit_shadow_render_pass(SpotShadowRenderPass* render_pass)
        {
            MainRendererFrameInputs& inputs = collected();
            ASSERT(inputs.num_spot_light_shadow_render_passes < MAX_NUM_SPOT_LIGHT_SHADOW_PASSES);
            inputs.spot_light_shadow_render_passes[inputs.num_spot_light_shadow_render_passes++] = render_pass;
        }

        void submit_shadow_render_pass(PointShadowRenderPass* render_pass)
        {
            MainRendererFrameInputs& inputs = collected();
            ASSERT(inputs.num_point_light_shadow_render_passes < MAX_NUM_POINT_LIGHT_SHADOW_PASSES);
            inputs.point_light_shadow_render_passes[inputs.num_point_light_shadow_render_passes++] = render_pass;
        }

        void set_raytracing_scene(RaytracingTLAS* tlas) { collected().tlas = tlas; }

        NODISCARD ERA_CORE_API static uint32 get_num_available_modes();

        void set_environment(const PBREnvironment& environment);
        void set_sun(const DirectionalLight& light);

        void set_point_lights(const ref<DXBuffer>& lights, uint32 num_lights, const ref<DXBuffer>& shadow_info_buffer);
        void set_spot_lights(const ref<DXBuffer>& lights, uint32 num_lights, const ref<DXBuffer>& shadow_info_buffer);
        void set_decals(const ref<DXBuffer>& decals, uint32 num_decals, const ref<DXTexture>& texture_atlas);

        NODISCARD const ref<DXTexture>& get_ao_result() const { return ao_textures[ao_history_index]; }
        NODISCARD const ref<DXTexture>& get_sss_result() const { return sss_textures[sss_history_index]; }
        NODISCARD const ref<DXTexture>& get_ssr_result() const { return ssr_resolve_texture; }
        NODISCARD const ref<DXTexture>& get_bloom_result() const { return bloom_texture; }
        NODISCARD const ref<DXTexture>& get_screen_velocities() const { return screen_velocities_texture; }

        // Settings
        RendererMode mode = RENDERER_MODE_RASTERIZED;
        AspectRatioMode aspect_ratio_mode = ASPECT_RATIO_FREE;

        RendererSettings settings{};

        uint32 render_width{};
        uint32 render_height{};
        ref<DXTexture> frame_result;

        uint32 hovered_object_id = -1;

        const RendererSpec spec;

        PathTracer path_tracer{};

    private:
        template <typename func_t = std::nullptr_t>
        uint64 execute_compute_tasks(ComputePassEvent event_time, const func_t& additional_tasks_callback = nullptr);

        NODISCARD DXCommandList* render_thread0(const CommonRenderData& common_render_data);
        NODISCARD DXCommandList* render_thread1(const CommonRenderData& common_render_data, bool aspect_ratio_mode_changed);
        NODISCARD DXCommandList* render_thread2(const CommonRenderData& common_render_data, const UserInput* input);
        NODISCARD DXCommandList* render_thread3(CommonRenderData common_render_data, DXDynamicConstantBuffer unjittered_camera_cbv);

        void recalculate_viewport(bool resize_textures);


        uint32 window_width{};
        uint32 window_height{};
        int32 window_x_offset{};
        int32 window_y_offset{};

        ref<DXTexture> hdr_color_texture;
        ref<DXTexture> world_normals_roughness_texture;
        ref<DXTexture> screen_velocities_texture;
        ref<DXTexture> prev_frame_screen_velocities_texture; // Swapped with screen_velocities_texture every frame.
        ref<DXTexture> object_ids_texture;
        ref<DXTexture> depth_stencil_buffer;
        ref<DXTexture> linear_depth_buffer;
        ref<DXTexture> opaque_depth_buffer; // The depth-stencil buffer gets copied to this texture after the opaque pass.

        ref<DXTexture> ao_calculation_texture;
        ref<DXTexture> ao_blur_temp_texture;
        ref<DXTexture> ao_textures[2]; // These get flip-flopped from frame to frame.
        volatile uint32 ao_history_index = 0;

        ref<DXTexture> sss_calculation_texture;
        ref<DXTexture> sss_blur_temp_texture;
        ref<DXTexture> sss_textures[2]; // These get flip-flopped from frame to frame.
        volatile uint32 sss_history_index = 0;

        ref<DXTexture> ssr_raycast_texture;
        ref<DXTexture> ssr_resolve_texture;
        ref<DXTexture> ssr_temporal_textures[2]; // These get flip-flopped from frame to frame.
        volatile uint32 ssr_history_index = 0;

        ref<DXTexture> prev_frame_hdr_color_texture; // This is downsampled by a factor of 2 and contains up to 8 mip levels.
        ref<DXTexture> prev_frame_hdr_color_temp_texture;

        ref<DXTexture> hdr_post_processing_texture;
        ref<DXTexture> ldr_post_processing_texture;
        ref<DXTexture> taa_textures[2]; // These get flip-flopped from frame to frame.
        volatile uint32 taa_history_index = 0;

        ref<DXTexture> bloom_texture;
        ref<DXTexture> bloom_temp_texture;

        ref<DXBuffer> hovered_object_id_readback_buffer;


        LightCulling culling{};

        dlss::DLSSFeatureAdapter dlss_adapter{};

        AspectRatioMode old_aspect_ratio_mode = ASPECT_RATIO_FREE;
        RendererMode old_mode = RENDERER_MODE_RASTERIZED;

        bool dlss_inited = false;
        bool ao_was_on_last_frame = false;
        bool window_hovered = false;
        bool sss_was_on_last_frame = false;

        friend dlss::DLSSFeatureAdapter;
        MainRendererFrameInputs& collected();

        MainRendererFrameInputs frame_inputs[NUM_BUFFERED_FRAMES];

        // Collection side: the previous frame's camera buffers.
        CameraCB camera_history_jittered{};
        CameraCB camera_history_unjittered{};

        friend struct EEditor;
    };
}