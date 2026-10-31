#include "ecs/prefab/prefab_asset.h"
#include "ecs/prefab/private/prefab_asset_impl.h"

#include "ecs/serialization/private/yaml_value_codec.h"

#include "core/log.h"
#include "core/yaml.h"

#include <rttr/registration>

#include <unordered_set>

namespace era_engine
{
    static constexpr uint32 PREFAB_VERSION = 1;

    RTTR_REGISTRATION
    {
        AssetTypeRegistry::register_asset_type<PrefabAsset>();
    }

    static void wait_for_asset(const GameAsset& asset)
    {
        while (asset.load_state.load() == AssetLoadState::LOADING)
        {
            JobHandle job = asset.load_job;
            if (job.state == nullptr)
            {
                std::this_thread::yield();
                continue;
            }
            job.wait_for_completion();
        }
    }

    PrefabAsset::PrefabAsset()
        : impl(make_ref<Impl>())
    {
    }

    PrefabAsset::~PrefabAsset()
    {
    }

    bool PrefabAsset::serialize(std::ostream& os) const
    {
        YAML::Emitter emitter;
        emitter << impl->document;
        os << emitter.c_str();
        return os.good();
    }

    bool PrefabAsset::deserialize(std::istream& is)
    {
        ref<Impl> new_impl = make_ref<Impl>();

        try
        {
            new_impl->document = YAML::Load(is);
            const YAML::Node& document = new_impl->document;

            const YAML::Node version_node = document["version"];
            if (!version_node || version_node.as<uint32>() != PREFAB_VERSION)
            {
                LOG_ERROR("Prefab> Unsupported prefab version in '%s'.", source_path.string().c_str());
                return false;
            }

            const YAML::Node entities_node = document["entities"];
            if (entities_node && !entities_node.IsSequence() && !entities_node.IsNull())
            {
                LOG_ERROR("Prefab> 'entities' must be a sequence in '%s'.", source_path.string().c_str());
                return false;
            }

            std::unordered_set<uint32> known_ids;
            std::vector<AssetReference> asset_references;

            const YAML::Node entity_nodes = entities_node && entities_node.IsSequence() ? entities_node : YAML::Node(YAML::NodeType::Sequence);
            for (const YAML::Node& entity_node : entity_nodes)
            {

                PrefabEntityDesc desc;
                desc.id = entity_node["id"].as<uint32>();
                desc.parent = entity_node["parent"] ? entity_node["parent"].as<uint32>() : 0;
                if (const YAML::Node components = entity_node["components"])
                {
                    desc.components = components;
                }

                if (desc.id == 0 || known_ids.contains(desc.id))
                {
                    LOG_ERROR("Prefab> Invalid or duplicated entity id %u in '%s'.", desc.id, source_path.string().c_str());
                    return false;
                }

                if (desc.parent != 0 && !known_ids.contains(desc.parent))
                {
                    LOG_WARNING("Prefab> Parent %u of entity %u is not defined before it in '%s', entity becomes top-level.", desc.parent, desc.id, source_path.string().c_str());
                    desc.parent = 0;
                }

                known_ids.insert(desc.id);
                YamlValueCodec::collect_asset_references(desc.components, asset_references);
                new_impl->entities.push_back(std::move(desc));
            }

            for (const AssetReference& reference : asset_references)
            {
                if (new_impl->dependencies.contains(reference))
                {
                    continue;
                }

                const std::string extension = fs::path(reference.path).extension().string();
                const AssetTypeInfo* asset_type = AssetTypeRegistry::find_by_extension(extension);
                if (asset_type == nullptr)
                {
                    LOG_WARNING("Prefab> Unknown asset type of '%s' in '%s'.", reference.path.c_str(), source_path.string().c_str());
                    continue;
                }

                new_impl->dependencies.emplace(reference, asset_type->load(AssetTypeRegistry::to_absolute_path(reference.path), reference.flags));
            }
        }
        catch (const YAML::Exception& exception)
        {
            LOG_ERROR("Prefab> Failed to parse '%s': %s", source_path.string().c_str(), exception.what());
            return false;
        }

        impl = std::move(new_impl);
        return true;
    }

    bool PrefabAsset::is_ready() const
    {
        if (load_state.load() != AssetLoadState::LOADED)
        {
            return false;
        }

        for (const auto& [reference, asset] : impl->dependencies)
        {
            if (asset->load_state.load() != AssetLoadState::LOADED)
            {
                return false;
            }
        }
        return true;
    }

    bool PrefabAsset::has_failed() const
    {
        if (load_state.load() == AssetLoadState::FAILED)
        {
            return true;
        }

        if (load_state.load() != AssetLoadState::LOADED)
        {
            return false;
        }

        for (const auto& [reference, asset] : impl->dependencies)
        {
            if (asset->load_state.load() == AssetLoadState::FAILED)
            {
                return true;
            }
        }
        return false;
    }

    void PrefabAsset::wait_until_loaded() const
    {
        wait_for_asset(*this);

        if (load_state.load() != AssetLoadState::LOADED)
        {
            return;
        }

        for (const auto& [reference, asset] : impl->dependencies)
        {
            wait_for_asset(*asset);
        }
    }

    size_t PrefabAsset::get_entity_count() const
    {
        return impl->entities.size();
    }

    trs PrefabAsset::get_local_transform(uint32 local_id) const
    {
        for (const PrefabEntityDesc& desc : impl->entities)
        {
            if (desc.id != local_id)
            {
                continue;
            }

            trs result = trs::identity;
            if (desc.components && desc.components.IsMap())
            {
                const YAML::Node transform_node = desc.components["TransformComponent"];
                if (transform_node && transform_node.IsMap() && transform_node["local_transform"])
                {
                    YAML::convert<trs>::decode(transform_node["local_transform"], result);
                }
            }
            return result;
        }
        return trs::identity;
    }

    std::string PrefabAsset::get_asset_type_impl()
    {
        return std::string(".eprefab");
    }
}
