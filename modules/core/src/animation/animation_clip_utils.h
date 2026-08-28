#pragma once 

#include "core_api.h"

#include "animation/animation.h"
#include "animation/animation_clip.h"

#include "asset/model_asset.h"

namespace era_engine::animation
{
    struct ERA_CORE_API AnimationClipAssetImportData
    {
        std::string name;
        float duration = 0.0f;

        std::vector<SkeletonPose> poses;
        std::vector<float> timestamps;
    };

    struct ERA_CORE_API TrackInfo final
    {
        std::vector<trs> joint_transforms;

        std::string joint_id;
        uint32 parent_index = INVALID_JOINT;

        float precision = 0.001f;
        float distance = 3.0f;
    };

    struct ERA_CORE_API CurveInfo final
    {
        std::vector<float> values;

        std::string curve_id;

        float precision = 0.001f;
    };

    struct ERA_CORE_API ClipInfo final
    {
        std::vector<TrackInfo> tracks;
        std::vector<CurveInfo> curves;

        float sample_rate = 30.0f;
        uint32 num_samples = 0;
    };

    class ERA_CORE_API AnimationAssetClipUtils final
    {
    public:
        AnimationAssetClipUtils() = delete;

        static ref<AnimationAssetClip> make_clip(const ClipInfo& info);

        static ref<AnimationAssetClip> make_clip(AnimationClipAssetImportData& anim_import_data, const Skeleton* skeleton, uint32 flags = 0);

        static std::vector<ref<AnimationAssetClip>> import_animations(std::vector<AnimationClipAssetImportData>& animations_to_import,
            const Skeleton* skeleton,
            const fs::path& file,
            uint32 flags = 0);
    };

}