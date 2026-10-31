#include <gtest/gtest.h>

#include <ecs/world.h>
#include <ecs/base_components/base_components.h>
#include <ecs/observable_member.h>
#include <ecs/prefab/prefab_asset.h>
#include <ecs/prefab/prefab_instance_component.h>
#include <ecs/prefab/prefab_serializer.h>
#include <ecs/prefab/prefab_utils.h>
#include <ecs/prefab/transient_component.h>
#include <ecs/serialization/component_registry.h>

#include <asset/asset_type_registry.h>

#include <rttr/registration>

#include <chrono>
#include <fstream>
#include <thread>

namespace
{
    using namespace era_engine;

    class PrefabTestAsset final : public GameAsset
    {
    public:
        bool serialize(std::ostream& os) const override
        {
            os << text;
            return true;
        }

        bool deserialize(std::istream& is) override
        {
            std::getline(is, text);
            return true;
        }

        static std::string get_asset_type_impl()
        {
            return ".eptest";
        }

        std::string text;
    };

    class PrefabTestComponent final : public Component
    {
    public:
        PrefabTestComponent() = default;
        PrefabTestComponent(ref<Entity::EcsData> _data)
            : Component(_data) {}

        float speed = 0.0f;
        Entity::Handle target = Entity::null_handle;
        EntityPtr other;
        std::vector<EntityPtr> links;
        ref<PrefabTestAsset> asset;
        ObservableMember<vec3> velocity;

        bool deserialized = false;
        bool other_resolved_on_deserialize = false;

        void on_deserialized() override
        {
            deserialized = true;
            other_resolved_on_deserialize = !other.is_empty() && other.get().has_component<PrefabTestComponent>();
        }

        ERA_VIRTUAL_REFLECT(Component)
    };

    class PrefabTest : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            directory = fs::temp_directory_path() / "era_engine_unittests" / "prefabs";
            fs::remove_all(directory);
            fs::create_directories(directory);
            AssetTypeRegistry::set_assets_root(directory);

            world = new World("PrefabTestWorld");
            world->init();
        }

        void TearDown() override
        {
            delete world;

            std::error_code error;
            fs::remove_all(directory, error);
        }

        fs::path write_file(const std::string& name, const std::string& content) const
        {
            const fs::path path = directory / name;
            fs::create_directories(path.parent_path());
            std::ofstream os(path, std::ios_base::binary | std::ios_base::trunc);
            os << content;
            return path;
        }

        Entity create_named(const char* name, const vec3& position, Entity parent = Entity::null)
        {
            Entity entity = world->create_entity(name);
            entity.get_component<TransformComponent>()->set_local_transform(trs(position));
            if (parent.is_valid())
            {
                entity.set_parent(parent.get_handle());
            }
            return entity;
        }

        std::vector<Entity> childs(const Entity& entity) const
        {
            return EntityContainer::get_entity_childs(world, entity.get_handle());
        }

        static std::string name_of(Entity entity)
        {
            return entity.get_component<NameComponent>()->get_name();
        }

        static vec3 local_position(Entity entity)
        {
            return entity.get_component<TransformComponent>()->get_local_transform().position;
        }

        std::vector<Entity> spawn_async(const fs::path& path, const PrefabInstantiateParams& params = {})
        {
            std::vector<Entity> result;
            bool spawned = false;

            PrefabUtils::instantiate_async(world, path, params, [&](const std::vector<Entity>& entities)
                {
                    result = entities;
                    spawned = true;
                });

            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
            while (PrefabUtils::get_pending_spawn_count(world) > 0 && std::chrono::steady_clock::now() < deadline)
            {
                PrefabUtils::process_spawn_queue(world);
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }

            EXPECT_TRUE(spawned);
            return result;
        }

        fs::path directory;
        World* world = nullptr;
    };
}

RTTR_REGISTRATION
{
    using namespace rttr;

    registration::class_<PrefabTestComponent>("PrefabTestComponent")
        .constructor<ref<Entity::EcsData>>()
        .property("speed", &PrefabTestComponent::speed)
        .property("target", &PrefabTestComponent::target)
        .property("other", &PrefabTestComponent::other)
        .property("links", &PrefabTestComponent::links)
        .property("asset", &PrefabTestComponent::asset)
        .property("velocity", &PrefabTestComponent::velocity);
    ComponentRegistry::register_component<PrefabTestComponent>();

    AssetTypeRegistry::register_asset_type<PrefabTestAsset>();
}

TEST_F(PrefabTest, HierarchyAndValuesAreRestored)
{
    Entity a = create_named("A", vec3(1.0f, 0.0f, 0.0f));
    Entity b = create_named("B", vec3(0.0f, 2.0f, 0.0f), a);
    Entity c = create_named("C", vec3(0.0f, 0.0f, 3.0f), b);
    Entity e = create_named("E", vec3(4.0f, 0.0f, 0.0f), a);
    Entity d = create_named("D", vec3(5.0f, 0.0f, 0.0f));
    b.add_component<PrefabTestComponent>()->speed = 7.5f;
    a.get_component<TransformComponent>()->type = TransformComponent::STATIC;

    const fs::path path = directory / "hierarchy.eprefab";
    ASSERT_TRUE(PrefabSerializer::save_to_file({a, d}, path));

    const std::vector<Entity> spawned = PrefabUtils::instantiate(world, path);
    ASSERT_EQ(spawned.size(), 2u);

    const Entity new_a = spawned[0];
    const Entity new_d = spawned[1];
    EXPECT_NE(new_a, a);
    EXPECT_EQ(name_of(new_a), "A");
    EXPECT_EQ(name_of(new_d), "D");
    EXPECT_EQ(new_a.get_parent_handle(), world->get_root_entity().get_handle());
    EXPECT_EQ(new_a.get_component<TransformComponent>()->type, TransformComponent::STATIC);
    EXPECT_EQ(local_position(new_d), vec3(5.0f, 0.0f, 0.0f));

    const std::vector<Entity> a_childs = childs(new_a);
    ASSERT_EQ(a_childs.size(), 2u);
    EXPECT_EQ(name_of(a_childs[0]), "B");
    EXPECT_EQ(name_of(a_childs[1]), "E");
    EXPECT_EQ(local_position(a_childs[0]), vec3(0.0f, 2.0f, 0.0f));
    ASSERT_NE(a_childs[0].get_component_if_exists<PrefabTestComponent>(), nullptr);
    EXPECT_EQ(a_childs[0].get_component<PrefabTestComponent>()->speed, 7.5f);
    EXPECT_EQ(a_childs[1].get_component_if_exists<PrefabTestComponent>(), nullptr);

    const std::vector<Entity> b_childs = childs(a_childs[0]);
    ASSERT_EQ(b_childs.size(), 1u);
    EXPECT_EQ(name_of(b_childs[0]), "C");
    EXPECT_EQ(b_childs[0].get_component<TransformComponent>()->get_world_position(), vec3(1.0f, 2.0f, 3.0f));
}

TEST_F(PrefabTest, SavedFileIsReadableYaml)
{
    Entity a = create_named("A", vec3(1.0f, 0.0f, 0.0f));
    create_named("B", vec3(0.0f), a);

    const std::string text = PrefabSerializer::save_to_string({a});

    EXPECT_NE(text.find("version: 1"), std::string::npos);
    EXPECT_NE(text.find("NameComponent:\n"), std::string::npos);
    EXPECT_NE(text.find("name: A"), std::string::npos);
    EXPECT_NE(text.find("position: [1, 0, 0]"), std::string::npos);
    EXPECT_EQ(text.find("PrefabInstanceComponent"), std::string::npos);
    EXPECT_EQ(text.find(" transform:"), std::string::npos);
}

TEST_F(PrefabTest, InstancesAreIndependent)
{
    Entity a = create_named("A", vec3(0.0f));
    a.add_component<PrefabTestComponent>()->speed = 1.0f;

    const fs::path path = directory / "independent.eprefab";
    ASSERT_TRUE(PrefabSerializer::save_to_file({a}, path));

    const ref<PrefabAsset> prefab = PrefabUtils::load(path);
    std::vector<Entity> first = PrefabUtils::instantiate(world, prefab);
    std::vector<Entity> second = PrefabUtils::instantiate(world, prefab);
    ASSERT_EQ(first.size(), 1u);
    ASSERT_EQ(second.size(), 1u);

    EXPECT_NE(first[0], second[0]);
    first[0].get_component<PrefabTestComponent>()->speed = 9.0f;
    EXPECT_EQ(second[0].get_component<PrefabTestComponent>()->speed, 1.0f);

    const PrefabInstanceComponent* first_instance = first[0].get_component<PrefabInstanceComponent>();
    const PrefabInstanceComponent* second_instance = second[0].get_component<PrefabInstanceComponent>();
    EXPECT_EQ(first_instance->prefab, prefab);
    EXPECT_EQ(first_instance->local_id, 1u);
    EXPECT_NE(first_instance->instance_id, second_instance->instance_id);
}

TEST_F(PrefabTest, EntityReferencesAreRemapped)
{
    Entity a = create_named("A", vec3(0.0f));
    Entity b = create_named("B", vec3(0.0f), a);
    Entity c = create_named("C", vec3(0.0f), b);
    Entity outside = create_named("Outside", vec3(0.0f));

    PrefabTestComponent* component = b.add_component<PrefabTestComponent>();
    component->target = c.get_handle();
    component->other = EntityPtr(a);
    component->links = {EntityPtr(c), EntityPtr(outside)};

    const fs::path path = directory / "references.eprefab";
    ASSERT_TRUE(PrefabSerializer::save_to_file({a}, path));

    const std::vector<Entity> spawned = PrefabUtils::instantiate(world, path);
    ASSERT_EQ(spawned.size(), 1u);

    const Entity new_a = spawned[0];
    const Entity new_b = childs(new_a)[0];
    const Entity new_c = childs(new_b)[0];
    const PrefabTestComponent* new_component = new_b.get_component<PrefabTestComponent>();

    EXPECT_EQ(new_component->target, new_c.get_handle());
    EXPECT_EQ(new_component->other.get(), new_a);
    ASSERT_EQ(new_component->links.size(), 2u);
    EXPECT_EQ(new_component->links[0].get(), new_c);
    EXPECT_TRUE(new_component->links[1].is_empty());
}

TEST_F(PrefabTest, TransientSubtreeIsNotSaved)
{
    Entity a = create_named("A", vec3(0.0f));
    Entity runtime = create_named("Runtime", vec3(0.0f), a);
    create_named("RuntimeChild", vec3(0.0f), runtime);
    create_named("Kept", vec3(0.0f), a);
    runtime.add_component<TransientComponent>();

    const fs::path path = directory / "transient.eprefab";
    ASSERT_TRUE(PrefabSerializer::save_to_file({a}, path));

    const ref<PrefabAsset> prefab = PrefabUtils::load(path);
    EXPECT_EQ(prefab->get_entity_count(), 2u);

    const std::vector<Entity> spawned = PrefabUtils::instantiate(world, prefab);
    ASSERT_EQ(spawned.size(), 1u);
    const std::vector<Entity> a_childs = childs(spawned[0]);
    ASSERT_EQ(a_childs.size(), 1u);
    EXPECT_EQ(name_of(a_childs[0]), "Kept");
}

TEST_F(PrefabTest, SavingWorldRootSavesItsChildren)
{
    create_named("First", vec3(0.0f));
    create_named("Second", vec3(0.0f));

    const fs::path path = directory / "scene.eprefab";
    ASSERT_TRUE(PrefabSerializer::save_to_file({world->get_root_entity()}, path));

    EXPECT_EQ(PrefabUtils::load(path)->get_entity_count(), 2u);
}

TEST_F(PrefabTest, InstantiateParamsAreApplied)
{
    Entity a = create_named("A", vec3(1.0f, 0.0f, 0.0f));

    const fs::path path = directory / "params.eprefab";
    ASSERT_TRUE(PrefabSerializer::save_to_file({a}, path));

    Entity parent = create_named("Parent", vec3(0.0f, 10.0f, 0.0f));

    PrefabInstantiateParams params;
    params.parent = parent;
    params.transform = trs(vec3(0.0f, 0.0f, 5.0f));

    const std::vector<Entity> spawned = PrefabUtils::instantiate(world, path, params);
    ASSERT_EQ(spawned.size(), 1u);

    EXPECT_EQ(spawned[0].get_parent_handle(), parent.get_handle());
    EXPECT_EQ(local_position(spawned[0]), vec3(1.0f, 0.0f, 5.0f));
    EXPECT_EQ(spawned[0].get_component<TransformComponent>()->get_world_position(), vec3(1.0f, 10.0f, 5.0f));
}

TEST_F(PrefabTest, UnknownComponentsAndPropertiesAreSkipped)
{
    const fs::path path = write_file("unknown.eprefab",
        "version: 1\n"
        "entities:\n"
        "  - id: 1\n"
        "    parent: 0\n"
        "    components:\n"
        "      UnknownComponent:\n"
        "        value: 1\n"
        "      PrefabTestComponent:\n"
        "        unknown_property: 2\n"
        "        speed: 3.0\n");

    const std::vector<Entity> spawned = PrefabUtils::instantiate(world, path);
    ASSERT_EQ(spawned.size(), 1u);
    EXPECT_EQ(spawned[0].get_component<PrefabTestComponent>()->speed, 3.0f);
}

TEST_F(PrefabTest, EntityWithoutComponentsIsCreated)
{
    const fs::path path = write_file("empty_entity.eprefab",
        "version: 1\n"
        "entities:\n"
        "  - id: 1\n"
        "  - id: 2\n"
        "    parent: 1\n");

    const std::vector<Entity> spawned = PrefabUtils::instantiate(world, path);
    ASSERT_EQ(spawned.size(), 1u);
    EXPECT_EQ(childs(spawned[0]).size(), 1u);
}

TEST_F(PrefabTest, OnDeserializedIsCalledAfterAllComponents)
{
    Entity a = create_named("A", vec3(0.0f));
    Entity b = create_named("B", vec3(0.0f), a);
    a.add_component<PrefabTestComponent>()->other = EntityPtr(b);
    b.add_component<PrefabTestComponent>();

    const fs::path path = directory / "hook.eprefab";
    ASSERT_TRUE(PrefabSerializer::save_to_file({a}, path));

    const std::vector<Entity> spawned = PrefabUtils::instantiate(world, path);
    ASSERT_EQ(spawned.size(), 1u);

    const PrefabTestComponent* component = spawned[0].get_component<PrefabTestComponent>();
    EXPECT_TRUE(component->deserialized);
    EXPECT_TRUE(component->other_resolved_on_deserialize);
    EXPECT_FALSE(a.get_component<PrefabTestComponent>()->deserialized);
}

TEST_F(PrefabTest, ObservableMembersAreChangedAfterSpawn)
{
    const fs::path path = write_file("observable.eprefab",
        "version: 1\n"
        "entities:\n"
        "  - id: 1\n"
        "    components:\n"
        "      PrefabTestComponent:\n"
        "        velocity: [1, 2, 3]\n");

    const std::vector<Entity> spawned = PrefabUtils::instantiate(world, path);
    ASSERT_EQ(spawned.size(), 1u);

    const PrefabTestComponent* component = spawned[0].get_component<PrefabTestComponent>();
    EXPECT_EQ(component->velocity.get(), vec3(1.0f, 2.0f, 3.0f));
    EXPECT_TRUE(component->velocity.is_changed());
}

TEST_F(PrefabTest, InvalidPrefabFails)
{
    const fs::path wrong_version = write_file("wrong_version.eprefab", "version: 99\nentities: []\n");
    const fs::path duplicated_id = write_file("duplicated_id.eprefab",
        "version: 1\n"
        "entities:\n"
        "  - id: 1\n"
        "  - id: 1\n");

    EXPECT_TRUE(PrefabUtils::instantiate(world, wrong_version).empty());
    EXPECT_TRUE(PrefabUtils::instantiate(world, duplicated_id).empty());
    EXPECT_TRUE(PrefabUtils::instantiate(world, directory / "missing.eprefab").empty());
    EXPECT_EQ(PrefabUtils::load(wrong_version)->load_state.load(), AssetLoadState::FAILED);
}

TEST_F(PrefabTest, AssetReferencesAreSavedAndLoaded)
{
    const fs::path asset_path = write_file("data/item.eptest", "item content");

    GameAssetsProvider provider;
    ref<PrefabTestAsset> asset = provider.load_game_asset_from_file<PrefabTestAsset>(asset_path);
    asset->load_job.wait_for_completion();
    ASSERT_EQ(asset->load_state.load(), AssetLoadState::LOADED);

    Entity a = create_named("A", vec3(0.0f));
    a.add_component<PrefabTestComponent>()->asset = asset;

    const std::string text = PrefabSerializer::save_to_string({a});
    EXPECT_NE(text.find("asset: data/item.eptest"), std::string::npos);

    const fs::path path = write_file("asset.eprefab", text);
    const std::vector<Entity> spawned = PrefabUtils::instantiate(world, path);
    ASSERT_EQ(spawned.size(), 1u);

    const ref<PrefabTestAsset>& loaded = spawned[0].get_component<PrefabTestComponent>()->asset;
    ASSERT_NE(loaded, nullptr);
    EXPECT_EQ(loaded, asset);
    EXPECT_EQ(loaded->text, "item content");
}

TEST_F(PrefabTest, MissingAssetFailsInstantiate)
{
    const fs::path path = write_file("missing_asset.eprefab",
        "version: 1\n"
        "entities:\n"
        "  - id: 1\n"
        "    components:\n"
        "      PrefabTestComponent:\n"
        "        asset:\n"
        "          asset: data/missing.eptest\n");

    const ref<PrefabAsset> prefab = PrefabUtils::load(path);
    EXPECT_TRUE(prefab->has_failed());
    EXPECT_TRUE(PrefabUtils::instantiate(world, prefab).empty());
}

TEST_F(PrefabTest, AsyncSpawnCreatesEntities)
{
    Entity a = create_named("A", vec3(0.0f));
    create_named("B", vec3(0.0f), a);

    const fs::path path = directory / "async.eprefab";
    ASSERT_TRUE(PrefabSerializer::save_to_file({a}, path));

    const size_t entity_count = world->size();
    const std::vector<Entity> spawned = spawn_async(path);

    ASSERT_EQ(spawned.size(), 1u);
    EXPECT_EQ(name_of(spawned[0]), "A");
    EXPECT_EQ(world->size(), entity_count + 2);
}

TEST_F(PrefabTest, AsyncSpawnOfMissingPrefabIsDropped)
{
    bool spawned = false;
    PrefabUtils::instantiate_async(world, directory / "missing.eprefab", {}, [&](const std::vector<Entity>&)
        {
            spawned = true;
        });

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (PrefabUtils::get_pending_spawn_count(world) > 0 && std::chrono::steady_clock::now() < deadline)
    {
        PrefabUtils::process_spawn_queue(world);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    EXPECT_EQ(PrefabUtils::get_pending_spawn_count(world), 0u);
    EXPECT_FALSE(spawned);
}

TEST_F(PrefabTest, AsyncSpawnUsesParent)
{
    Entity a = create_named("A", vec3(0.0f));
    const fs::path path = directory / "async_parent.eprefab";
    ASSERT_TRUE(PrefabSerializer::save_to_file({a}, path));

    Entity parent = create_named("Parent", vec3(0.0f));
    PrefabInstantiateParams params;
    params.parent = parent;

    const std::vector<Entity> spawned = spawn_async(path, params);
    ASSERT_EQ(spawned.size(), 1u);
    EXPECT_EQ(spawned[0].get_parent_handle(), parent.get_handle());
}

TEST_F(PrefabTest, InstanceRootsAreTopLevelEntitiesOfInstance)
{
    Entity a = create_named("A", vec3(0.0f));
    create_named("B", vec3(0.0f), a);
    Entity c = create_named("C", vec3(0.0f));

    const fs::path path = directory / "roots.eprefab";
    ASSERT_TRUE(PrefabSerializer::save_to_file({a, c}, path));

    const std::vector<Entity> spawned = PrefabUtils::instantiate(world, path);
    ASSERT_EQ(spawned.size(), 2u);

    const Entity child = childs(spawned[0])[0];
    const std::vector<Entity> roots = PrefabUtils::get_instance_roots(child);
    ASSERT_EQ(roots.size(), 2u);
    EXPECT_EQ(roots[0], spawned[0]);
    EXPECT_EQ(roots[1], spawned[1]);
    EXPECT_TRUE(PrefabUtils::get_instance_roots(a).empty());
}

TEST_F(PrefabTest, ApplyWritesChangesAndKeepsRootTransform)
{
    Entity a = create_named("A", vec3(1.0f, 0.0f, 0.0f));
    create_named("B", vec3(0.0f), a).add_component<PrefabTestComponent>()->speed = 1.0f;

    const fs::path path = directory / "apply.eprefab";
    ASSERT_TRUE(PrefabSerializer::save_to_file({a}, path));

    PrefabInstantiateParams params;
    params.transform = trs(vec3(0.0f, 5.0f, 0.0f));
    const std::vector<Entity> spawned = PrefabUtils::instantiate(world, path, params);
    ASSERT_EQ(spawned.size(), 1u);

    Entity instance_child = childs(spawned[0])[0];
    instance_child.get_component<PrefabTestComponent>()->speed = 9.0f;
    create_named("Added", vec3(0.0f), spawned[0]);

    ASSERT_TRUE(PrefabUtils::apply_instance(instance_child));
    EXPECT_EQ(local_position(spawned[0]), vec3(1.0f, 5.0f, 0.0f));

    const std::vector<Entity> second = PrefabUtils::instantiate(world, path);
    ASSERT_EQ(second.size(), 1u);
    EXPECT_EQ(local_position(second[0]), vec3(1.0f, 0.0f, 0.0f));

    const std::vector<Entity> second_childs = childs(second[0]);
    ASSERT_EQ(second_childs.size(), 2u);
    EXPECT_EQ(second_childs[0].get_component<PrefabTestComponent>()->speed, 9.0f);
    EXPECT_EQ(name_of(second_childs[1]), "Added");
}

TEST_F(PrefabTest, RevertRestoresPrefabStateAndKeepsPlacement)
{
    Entity a = create_named("A", vec3(0.0f));
    a.add_component<PrefabTestComponent>()->speed = 2.0f;

    const fs::path path = directory / "revert.eprefab";
    ASSERT_TRUE(PrefabSerializer::save_to_file({a}, path));

    Entity parent = create_named("Parent", vec3(0.0f));
    PrefabInstantiateParams params;
    params.parent = parent;
    const std::vector<Entity> spawned = PrefabUtils::instantiate(world, path, params);
    ASSERT_EQ(spawned.size(), 1u);

    Entity instance = spawned[0];
    instance.get_component<PrefabTestComponent>()->speed = 7.0f;
    instance.get_component<TransformComponent>()->set_local_transform(trs(vec3(3.0f, 0.0f, 0.0f)));
    create_named("Added", vec3(0.0f), instance);

    const std::vector<Entity> reverted = PrefabUtils::revert_instance(instance);
    ASSERT_EQ(reverted.size(), 1u);

    EXPECT_FALSE(world->get_entity(instance.get_handle()) == instance);
    EXPECT_EQ(reverted[0].get_parent_handle(), parent.get_handle());
    EXPECT_EQ(local_position(reverted[0]), vec3(3.0f, 0.0f, 0.0f));
    EXPECT_EQ(reverted[0].get_component<PrefabTestComponent>()->speed, 2.0f);
    EXPECT_TRUE(childs(reverted[0]).empty());
}

TEST_F(PrefabTest, UnpackRemovesInstanceLink)
{
    Entity a = create_named("A", vec3(0.0f));
    create_named("B", vec3(0.0f), a);

    const fs::path path = directory / "unpack.eprefab";
    ASSERT_TRUE(PrefabSerializer::save_to_file({a}, path));

    const std::vector<Entity> spawned = PrefabUtils::instantiate(world, path);
    ASSERT_EQ(spawned.size(), 1u);

    PrefabUtils::unpack_instance(spawned[0]);

    EXPECT_FALSE(spawned[0].has_component<PrefabInstanceComponent>());
    EXPECT_FALSE(childs(spawned[0])[0].has_component<PrefabInstanceComponent>());
    EXPECT_TRUE(PrefabUtils::get_instance_roots(spawned[0]).empty());
}

TEST_F(PrefabTest, DuplicateCopiesSubtreeNextToOriginal)
{
    Entity parent = create_named("Parent", vec3(0.0f));
    Entity a = create_named("A", vec3(1.0f, 0.0f, 0.0f), parent);
    create_named("B", vec3(0.0f), a).add_component<PrefabTestComponent>()->speed = 4.0f;

    const std::vector<Entity> copies = PrefabUtils::duplicate({a, childs(a)[0]});
    ASSERT_EQ(copies.size(), 1u);

    const Entity copy = copies[0];
    EXPECT_NE(copy, a);
    EXPECT_EQ(copy.get_parent_handle(), parent.get_handle());
    EXPECT_EQ(name_of(copy), "A");
    EXPECT_EQ(local_position(copy), vec3(1.0f, 0.0f, 0.0f));
    EXPECT_FALSE(copy.has_component<PrefabInstanceComponent>());

    const std::vector<Entity> copy_childs = childs(copy);
    ASSERT_EQ(copy_childs.size(), 1u);
    EXPECT_EQ(copy_childs[0].get_component<PrefabTestComponent>()->speed, 4.0f);
    EXPECT_EQ(childs(parent).size(), 2u);
}
