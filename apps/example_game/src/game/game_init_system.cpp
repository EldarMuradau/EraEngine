#include "game/game_init_system.h"
#include "game/physics/gameplay_physics_types.h"

#include <core/ecs/input_receiver_component.h>
#include <core/ecs/input_sender_component.h>
#include <core/string.h>
#include <core/ecs/camera_holder_component.h>
#include <core/debug/debug_var.h>
#include <core/log.h>

#include <asset/game_asset.h>

#include <animation/animation_clip.h>
#include <animation/animation_clip_utils.h>

#include <rendering/ecs/renderer_holder_root_component.h>

#include <ecs/update_groups.h>
#include <ecs/base_components/transform_component.h>
#include <ecs/rendering/world_renderer.h>
#include <ecs/rendering/scene_rendering.h>
#include <ecs/rendering/mesh_component.h>
#include <ecs/prefab/prefab_utils.h>

#include <physics/body_component.h>
#include <physics/core/physics.h>
#include <physics/shape_component.h>
#include <physics/cct_component.h>
#include <physics/joint.h>
#include <physics/ragdolls/ragdoll_component.h>
#include <physics/physical_animation/physical_animation_component.h>
#include <physics/collisions_holder_root_component.h>
#include <physics/pbd/cloth_physics_component.h>
#include <physics/vehicles/w4_vehicle_component.h>
#include <physics/vehicles/vehicle_import_utils.h>
#include <physics/physical_animation/dismemberment/ragdoll_dismemberment_component.h>
#include <physics/destructions/destructible_component.h>

#include <motion_matching/trajectory/trajectory_component.h>
#include <motion_matching/motion/motion_component.h>
#include <motion_matching/features/pose_feature.h>
#include <motion_matching/features/trajectory_feature.h>
#include <motion_matching/features/phase_feature.h>
#include <motion_matching/motion_matching_database.h>
#include <motion_matching/features/motion_matching_feature_set.h>
#include <motion_matching/motion_matching_component.h>
#include <motion_matching/motion_data_component.h>

#include "game/movement/character_locomotion_component.h"
#include "game/camera/gameplay_tpp_camera_component.h"

#include <ik/ik_component.h>
#include <ik/full_body_ik_component.h>

#include <audio/audio.h>

#include <terrain/terrain.h>
#include <terrain/grass.h>

#include <animation/skinning.h>

#include <rttr/policy.h>
#include <rttr/registration>

namespace era_engine
{
    RTTR_REGISTRATION
    {
        using namespace rttr;

        registration::class_<GameInitSystem>("GameInitSystem")
            .constructor<World*>()(policy::ctor::as_raw_ptr, metadata("Tag", std::string("game")))
            .method("update", &GameInitSystem::update)(metadata("update_group", update_types::GAMEPLAY_NORMAL));
    }

    static DebugVar<float> sphere_speed = DebugVar<float>("test.sphere_speed", 1500.0f);

    GameInitSystem::GameInitSystem(World* _world)
        : System(_world)
    {
    }

    GameInitSystem::~GameInitSystem()
    {
    }

    void GameInitSystem::init()
    {
        using namespace animation;
        using namespace physics;

        {
            MeshBuilder builder;

            sphere_render_material = get_default_pbr_material();

            sphere_mesh = make_ref<MultiMesh>();
            builder.push_sphere({});
            sphere_mesh->submeshes.push_back({builder.end_submesh(), {}, trs::identity, sphere_render_material});

            sphere_mesh->mesh = builder.create_dx_mesh();
            sphere_mesh->load_state = AssetLoadState::LOADED;
        }

        RendererHolderRootComponent* renderer_holder_rc = world->add_root_component<RendererHolderRootComponent>();
        ASSERT(renderer_holder_rc != nullptr);
        renderer_holder_rc->present_mode = RendererHolderRootComponent::PRESENT_TO_MAIN_WINDOW;

        CollisionsHolderRootComponent* collisions_holder_rc = world->add_root_component<CollisionsHolderRootComponent>();
        ASSERT(collisions_holder_rc != nullptr);

        /*camera_entity = world->create_entity("CameraEntity");
        CameraHolderComponent* camera_holder_component = camera_entity.add_component<CameraHolderComponent>();
        camera_holder_component->set_camera_type(CameraHolderComponent::FREE_CAMERA_ON_HOLD);
        camera_holder_component->set_render_camera(&renderer_holder_rc->camera);
        camera_entity.add_component<InputSenderComponent>()->add_reciever(camera_entity.add_component<InputReceiverComponent>());*/

        PbrMaterialDesc default_plane_mat_desc;
        default_plane_mat_desc.albedo = get_asset_path("/resources/assets/uv.jpg");
        default_plane_mat_desc.normal = "";
        default_plane_mat_desc.roughness = "";
        default_plane_mat_desc.uv_scale = 15.0f;
        default_plane_mat_desc.metallic_override = 0.35f;
        default_plane_mat_desc.roughness_override = 0.01f;

        auto default_plane_mat = create_pbr_material(default_plane_mat_desc);

        MeshBuilder builder;

        ref<MultiMesh> ground_mesh = make_ref<MultiMesh>();
        builder.push_box({vec3(0.f), vec3(30.f, 4.f, 30.f)});
        ground_mesh->submeshes.push_back({builder.end_submesh(), {}, trs::identity, default_plane_mat});

        GameAssetsProvider provider;

        if (true)
        {
            if (true)
            {
                if (false)
                {
                    import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Change DirectionR.fbx"),
                        MESH_CREATION_FLAGS_ANIMATED | MESH_CREATION_FLAGS_SM_TO_M | MESH_CREATION_FLAGS_GENERATE_ROOT_MOTION | MESH_CREATION_FLAGS_FLIP_X);

                    import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Run To Stop.fbx"),
                        MESH_CREATION_FLAGS_ANIMATED | MESH_CREATION_FLAGS_SM_TO_M | MESH_CREATION_FLAGS_GENERATE_ROOT_MOTION | MESH_CREATION_FLAGS_FLIP_X);

                    import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Change DirectionL.fbx"),
                        MESH_CREATION_FLAGS_ANIMATED | MESH_CREATION_FLAGS_SM_TO_M | MESH_CREATION_FLAGS_GENERATE_ROOT_MOTION | MESH_CREATION_FLAGS_FLIP_X);

                    import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Run Forward Arc Left.fbx"),
                        MESH_CREATION_FLAGS_ANIMATED | MESH_CREATION_FLAGS_SM_TO_M | MESH_CREATION_FLAGS_GENERATE_ROOT_MOTION | MESH_CREATION_FLAGS_FLIP_X);

                    import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Run Forward Arc Right.fbx"),
                        MESH_CREATION_FLAGS_ANIMATED | MESH_CREATION_FLAGS_SM_TO_M | MESH_CREATION_FLAGS_GENERATE_ROOT_MOTION | MESH_CREATION_FLAGS_FLIP_X);

                    import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Running Turn 180R.fbx"),
                        MESH_CREATION_FLAGS_ANIMATED | MESH_CREATION_FLAGS_SM_TO_M | MESH_CREATION_FLAGS_GENERATE_ROOT_MOTION | MESH_CREATION_FLAGS_FLIP_X);

                    import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Running Turn 180L.fbx"),
                        MESH_CREATION_FLAGS_ANIMATED | MESH_CREATION_FLAGS_SM_TO_M | MESH_CREATION_FLAGS_GENERATE_ROOT_MOTION | MESH_CREATION_FLAGS_FLIP_X);

                    import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Turn To RunningL.fbx"),
                        MESH_CREATION_FLAGS_ANIMATED | MESH_CREATION_FLAGS_SM_TO_M | MESH_CREATION_FLAGS_GENERATE_ROOT_MOTION | MESH_CREATION_FLAGS_FLIP_X);

                    import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Turn To RunningR.fbx"),
                        MESH_CREATION_FLAGS_ANIMATED | MESH_CREATION_FLAGS_SM_TO_M | MESH_CREATION_FLAGS_GENERATE_ROOT_MOTION | MESH_CREATION_FLAGS_FLIP_X);

                    import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Running Left Turn.fbx"),
                        MESH_CREATION_FLAGS_ANIMATED | MESH_CREATION_FLAGS_SM_TO_M | MESH_CREATION_FLAGS_GENERATE_ROOT_MOTION | MESH_CREATION_FLAGS_FLIP_X);

                    import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Running Right Turn.fbx"),
                        MESH_CREATION_FLAGS_ANIMATED | MESH_CREATION_FLAGS_SM_TO_M | MESH_CREATION_FLAGS_GENERATE_ROOT_MOTION | MESH_CREATION_FLAGS_FLIP_X);

                    import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Running.fbx"),
                        MESH_CREATION_FLAGS_ANIMATED | MESH_CREATION_FLAGS_SM_TO_M | MESH_CREATION_FLAGS_GENERATE_ROOT_MOTION | MESH_CREATION_FLAGS_FLIP_X);

                    import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Unarmed Idle.fbx"),
                        MESH_CREATION_FLAGS_ANIMATED | MESH_CREATION_FLAGS_SM_TO_M | MESH_CREATION_FLAGS_GENERATE_ROOT_MOTION | MESH_CREATION_FLAGS_FLIP_X);

                    import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Catwalk Sequence 02L.fbx"),
                        MESH_CREATION_FLAGS_ANIMATED | MESH_CREATION_FLAGS_SM_TO_M | MESH_CREATION_FLAGS_GENERATE_ROOT_MOTION | MESH_CREATION_FLAGS_FLIP_X);

                    import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Catwalk Sequence 02R.fbx"),
                        MESH_CREATION_FLAGS_ANIMATED | MESH_CREATION_FLAGS_SM_TO_M | MESH_CREATION_FLAGS_GENERATE_ROOT_MOTION | MESH_CREATION_FLAGS_FLIP_X);

                    import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Catwalk Walk Start Turn 180 Left.fbx"),
                        MESH_CREATION_FLAGS_ANIMATED | MESH_CREATION_FLAGS_SM_TO_M | MESH_CREATION_FLAGS_GENERATE_ROOT_MOTION | MESH_CREATION_FLAGS_FLIP_X);

                    import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Catwalk Walk Start Turn 180 Right.fbx"),
                        MESH_CREATION_FLAGS_ANIMATED | MESH_CREATION_FLAGS_SM_TO_M | MESH_CREATION_FLAGS_GENERATE_ROOT_MOTION | MESH_CREATION_FLAGS_FLIP_X);

                    import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Catwalk Walk Forward Arc 90R.fbx"),
                        MESH_CREATION_FLAGS_ANIMATED | MESH_CREATION_FLAGS_SM_TO_M | MESH_CREATION_FLAGS_GENERATE_ROOT_MOTION | MESH_CREATION_FLAGS_FLIP_X);

                    import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Catwalk Walk Forward Arc 90L.fbx"),
                        MESH_CREATION_FLAGS_ANIMATED | MESH_CREATION_FLAGS_SM_TO_M | MESH_CREATION_FLAGS_GENERATE_ROOT_MOTION | MESH_CREATION_FLAGS_FLIP_X);

                    import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Stop Walking.fbx"),
                        MESH_CREATION_FLAGS_ANIMATED | MESH_CREATION_FLAGS_SM_TO_M | MESH_CREATION_FLAGS_GENERATE_ROOT_MOTION | MESH_CREATION_FLAGS_FLIP_X);

                    import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Walking Right Turn.fbx"),
                        MESH_CREATION_FLAGS_ANIMATED | MESH_CREATION_FLAGS_SM_TO_M | MESH_CREATION_FLAGS_GENERATE_ROOT_MOTION | MESH_CREATION_FLAGS_FLIP_X);

                    import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Walking Left Turn.fbx"),
                        MESH_CREATION_FLAGS_ANIMATED | MESH_CREATION_FLAGS_SM_TO_M | MESH_CREATION_FLAGS_GENERATE_ROOT_MOTION | MESH_CREATION_FLAGS_FLIP_X);

                    import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Jog Backward DiagonalR.fbx"),
                        MESH_CREATION_FLAGS_ANIMATED | MESH_CREATION_FLAGS_SM_TO_M | MESH_CREATION_FLAGS_GENERATE_ROOT_MOTION | MESH_CREATION_FLAGS_FLIP_X);

                    import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Jog Backward DiagonalL.fbx"),
                        MESH_CREATION_FLAGS_ANIMATED | MESH_CREATION_FLAGS_SM_TO_M | MESH_CREATION_FLAGS_GENERATE_ROOT_MOTION | MESH_CREATION_FLAGS_FLIP_X);

                    import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Walking Backwards.fbx"),
                        MESH_CREATION_FLAGS_ANIMATED | MESH_CREATION_FLAGS_SM_TO_M | MESH_CREATION_FLAGS_GENERATE_ROOT_MOTION | MESH_CREATION_FLAGS_FLIP_X);

                    import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Sprint.fbx"),
                        MESH_CREATION_FLAGS_ANIMATED | MESH_CREATION_FLAGS_SM_TO_M | MESH_CREATION_FLAGS_GENERATE_ROOT_MOTION | MESH_CREATION_FLAGS_FLIP_X);

                    import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Start Walking.fbx"),
                        MESH_CREATION_FLAGS_ANIMATED | MESH_CREATION_FLAGS_SM_TO_M | MESH_CREATION_FLAGS_GENERATE_ROOT_MOTION | MESH_CREATION_FLAGS_FLIP_X);

                    import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Standing Idle.fbx"),
                        MESH_CREATION_FLAGS_ANIMATED | MESH_CREATION_FLAGS_SM_TO_M | MESH_CREATION_FLAGS_GENERATE_ROOT_MOTION | MESH_CREATION_FLAGS_FLIP_X);

                    import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Walking.fbx"),
                        MESH_CREATION_FLAGS_ANIMATED | MESH_CREATION_FLAGS_SM_TO_M | MESH_CREATION_FLAGS_GENERATE_ROOT_MOTION | MESH_CREATION_FLAGS_FLIP_X);

                    import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Running To Turn.fbx"),
                        MESH_CREATION_FLAGS_ANIMATED | MESH_CREATION_FLAGS_SM_TO_M | MESH_CREATION_FLAGS_GENERATE_ROOT_MOTION | MESH_CREATION_FLAGS_FLIP_X);

                    import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Jog Strafe Right.fbx"),
                        MESH_CREATION_FLAGS_ANIMATED | MESH_CREATION_FLAGS_SM_TO_M | MESH_CREATION_FLAGS_GENERATE_ROOT_MOTION | MESH_CREATION_FLAGS_FLIP_X);

                    import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Jog Strafe Left.fbx"),
                        MESH_CREATION_FLAGS_ANIMATED | MESH_CREATION_FLAGS_SM_TO_M | MESH_CREATION_FLAGS_GENERATE_ROOT_MOTION | MESH_CREATION_FLAGS_FLIP_X);

                    import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Backward Right Turn.fbx"),
                        MESH_CREATION_FLAGS_ANIMATED | MESH_CREATION_FLAGS_SM_TO_M | MESH_CREATION_FLAGS_GENERATE_ROOT_MOTION | MESH_CREATION_FLAGS_FLIP_X);

                    import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Backward Left Turn.fbx"),
                        MESH_CREATION_FLAGS_ANIMATED | MESH_CREATION_FLAGS_SM_TO_M | MESH_CREATION_FLAGS_GENERATE_ROOT_MOTION | MESH_CREATION_FLAGS_FLIP_X);

                    import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Running To Turn.fbx"),
                        MESH_CREATION_FLAGS_ANIMATED | MESH_CREATION_FLAGS_SM_TO_M | MESH_CREATION_FLAGS_GENERATE_ROOT_MOTION | MESH_CREATION_FLAGS_FLIP_X);

                    import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Run Backward Arc Left.fbx"),
                        MESH_CREATION_FLAGS_ANIMATED | MESH_CREATION_FLAGS_SM_TO_M | MESH_CREATION_FLAGS_GENERATE_ROOT_MOTION | MESH_CREATION_FLAGS_FLIP_X);

                    import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Run Backward Arc Right.fbx"),
                        MESH_CREATION_FLAGS_ANIMATED | MESH_CREATION_FLAGS_SM_TO_M | MESH_CREATION_FLAGS_GENERATE_ROOT_MOTION | MESH_CREATION_FLAGS_FLIP_X);

                    import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Running Backward.fbx"),
                        MESH_CREATION_FLAGS_ANIMATED | MESH_CREATION_FLAGS_SM_TO_M | MESH_CREATION_FLAGS_GENERATE_ROOT_MOTION | MESH_CREATION_FLAGS_FLIP_X);

                    import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/StrafeR.fbx"),
                        MESH_CREATION_FLAGS_ANIMATED | MESH_CREATION_FLAGS_SM_TO_M | MESH_CREATION_FLAGS_GENERATE_ROOT_MOTION | MESH_CREATION_FLAGS_FLIP_X);

                    import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/StrafeL.fbx"),
                        MESH_CREATION_FLAGS_ANIMATED | MESH_CREATION_FLAGS_SM_TO_M | MESH_CREATION_FLAGS_GENERATE_ROOT_MOTION | MESH_CREATION_FLAGS_FLIP_X);

                    import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Slow Jog Backwards.fbx"),
                        MESH_CREATION_FLAGS_ANIMATED | MESH_CREATION_FLAGS_SM_TO_M | MESH_CREATION_FLAGS_GENERATE_ROOT_MOTION | MESH_CREATION_FLAGS_FLIP_X);

                    import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Run Backward.fbx"),
                        MESH_CREATION_FLAGS_ANIMATED | MESH_CREATION_FLAGS_SM_TO_M | MESH_CREATION_FLAGS_GENERATE_ROOT_MOTION | MESH_CREATION_FLAGS_FLIP_X);
                }

                // ref<MultiMesh> mesh = import_animated_mesh_from_file_async(get_asset_path("/resources/assets/test_character/Walking.fbx"),
                //	MESH_CREATION_FLAGS_ANIMATED | MESH_CREATION_FLAGS_SM_TO_M | MESH_CREATION_FLAGS_GENERATE_ROOT_MOTION | MESH_CREATION_FLAGS_OVERRIDE_TEXTURES_WITH_LOCAL_FOLDER | MESH_CREATION_FLAGS_FLIP_X);

                ref<MultiMesh> mesh = provider.load_game_asset_from_file<MultiMesh>(get_asset_path("/resources/assets/test_character/Character.emesh"), true, {}, MESH_CREATION_FLAGS_ANIMATED);

                character = world->create_entity("Character");
                camera_entity = character;

                character.add_component<MeshComponent>(mesh);
                character.add_component<GameplayTppCameraComponent>();
                CameraHolderComponent* camera_holder_component = character.add_component<CameraHolderComponent>();
                camera_holder_component->set_camera_type(CameraHolderComponent::USER_DEFINED);
                camera_holder_component->set_render_camera(&renderer_holder_rc->camera);
                character.add_component<InputSenderComponent>()->add_reciever(character.add_component<InputReceiverComponent>());

                TransformComponent* transform_component = character.get_component<TransformComponent>();
                transform_component->set_world_transform(trs{vec3(0.0f, 1.0f, 2.0f), quat::identity, vec3(1.0f)});

                mesh->load_job.wait_for_completion();

                SkeletonComponent* skeleton_component = character.add_component<SkeletonComponent>();

                ref<Skeleton> skeleton = provider.load_game_asset_from_file<Skeleton>(get_asset_path("/resources/assets/test_character/skeletons/Character_skeleton0.eskeleton"));
                skeleton->load_job.wait_for_completion();
                skeleton_component->skeleton = skeleton;
                // skeleton_component->draw_sceleton = true;
                skeleton_component->apply_pose(skeleton->get_default_pose());

                AnimationComponent* animation_component = character.add_component<AnimationComponent>();
                animation_component->play = true;
                animation_component->loop = true;

                {
                    ref<AnimationAssetClip> anim_clip = provider.load_game_asset_from_file<AnimationAssetClip>(get_asset_path("/resources/assets/test_character/animations/Unarmed Idle_mixamo.com_clip0.eanm"));
                    anim_clip->load_job.wait_for_completion();
                    animation_component->current_animation = anim_clip;
                    animation_component->current_anim_position = 0.0f;
                }


                animation_component->activate_inertial_blend();

                CharacterControllerComponent* cct_component = character.add_component<CharacterControllerComponent>();
                cct_component->collision_type = static_cast<CollisionType>(GameCollisionType::CCT);
                cct_component->height = 1.2f;
                cct_component->radius = 0.23f;
                cct_component->step_offset = 0.05f;
                cct_component->handle_non_cct_contacts = false;

                character.add_component<MotionComponent>();
                TrajectoryComponent* trajectory_component = character.add_component<TrajectoryComponent>();
                trajectory_component->time_offsets[0] = 0.0f;
                trajectory_component->time_offsets[1] = 0.12f;
                trajectory_component->time_offsets[2] = 0.32f;
                trajectory_component->time_offsets[3] = 0.54f;
                trajectory_component->time_offsets[4] = 0.76f;

                RagdollJointIds joint_init_ids;
                joint_init_ids.head_end_idx = skeleton->name_to_joint_id.at("mixamorig:HeadTop_End");
                joint_init_ids.head_idx = skeleton->name_to_joint_id.at("mixamorig:Head");
                joint_init_ids.neck_idx = skeleton->name_to_joint_id.at("mixamorig:Neck");

                joint_init_ids.spine_03_idx = skeleton->name_to_joint_id.at("mixamorig:Spine2");
                joint_init_ids.spine_02_idx = skeleton->name_to_joint_id.at("mixamorig:Spine1");
                joint_init_ids.spine_01_idx = skeleton->name_to_joint_id.at("mixamorig:Spine");
                joint_init_ids.pelvis_idx = skeleton->name_to_joint_id.at("mixamorig:Hips");

                joint_init_ids.clavicle_l_idx = skeleton->name_to_joint_id.at("mixamorig:LeftShoulder");
                joint_init_ids.clavicle_r_idx = skeleton->name_to_joint_id.at("mixamorig:RightShoulder");

                joint_init_ids.root_idx = skeleton->name_to_joint_id.at("EE_GeneratedRoot");
                joint_init_ids.attachment_idx = skeleton->name_to_joint_id.at("mixamorig:Hips");

                joint_init_ids.thigh_l_idx = skeleton->name_to_joint_id.at("mixamorig:LeftUpLeg");
                joint_init_ids.calf_l_idx = skeleton->name_to_joint_id.at("mixamorig:LeftLeg");
                joint_init_ids.foot_l_idx = skeleton->name_to_joint_id.at("mixamorig:LeftFoot");
                joint_init_ids.foot_end_l_idx = skeleton->name_to_joint_id.at("mixamorig:LeftToe_End");

                joint_init_ids.thigh_r_idx = skeleton->name_to_joint_id.at("mixamorig:RightUpLeg");
                joint_init_ids.calf_r_idx = skeleton->name_to_joint_id.at("mixamorig:RightLeg");
                joint_init_ids.foot_r_idx = skeleton->name_to_joint_id.at("mixamorig:RightFoot");
                joint_init_ids.foot_end_r_idx = skeleton->name_to_joint_id.at("mixamorig:RightToe_End");

                joint_init_ids.upperarm_l_idx = skeleton->name_to_joint_id.at("mixamorig:LeftArm");
                joint_init_ids.lowerarm_l_idx = skeleton->name_to_joint_id.at("mixamorig:LeftForeArm");
                joint_init_ids.hand_l_idx = skeleton->name_to_joint_id.at("mixamorig:LeftHand");
                joint_init_ids.hand_end_l_idx = skeleton->name_to_joint_id.at("mixamorig:LeftHandMiddle3");

                joint_init_ids.upperarm_r_idx = skeleton->name_to_joint_id.at("mixamorig:RightArm");
                joint_init_ids.lowerarm_r_idx = skeleton->name_to_joint_id.at("mixamorig:RightForeArm");
                joint_init_ids.hand_r_idx = skeleton->name_to_joint_id.at("mixamorig:RightHand");
                joint_init_ids.hand_end_r_idx = skeleton->name_to_joint_id.at("mixamorig:RightHandMiddle3");

                if (true)
                {
                    database = provider.load_game_asset_from_file<MotionMatchingDatabase>(get_asset_path("/resources/assets/test_character/mmdb/locomotion.emmdb"));
                    database->load_job.wait_for_completion();
                }

                if (false)
                {
                    database = make_ref<MotionMatchingDatabase>();

                    database->database_id = "LOCOMOTION";
                    database->knn_type = KnnStructureType::HNSW;

                    {
                        PoseFeature* pose_feature = new PoseFeature();
                        {
                            ref<PoseFeatureDesc> desc = make_ref<PoseFeatureDesc>();
                            desc->basis = FeatureDescBasis::XYZ;
                            desc->type = FeatureDescType::LOCATION;
                            desc->joint_id = joint_init_ids.pelvis_idx;
                            desc->name = "pelvis_location";
                            pose_feature->descriptors.push_back(desc);
                        }

                        {
                            ref<PoseFeatureDesc> desc = make_ref<PoseFeatureDesc>();
                            desc->basis = FeatureDescBasis::XYZ;
                            desc->type = FeatureDescType::LOCATION;
                            desc->joint_id = joint_init_ids.neck_idx;
                            desc->name = "neck_location";
                            pose_feature->descriptors.push_back(desc);
                        }

                        {
                            ref<PoseFeatureDesc> desc = make_ref<PoseFeatureDesc>();
                            desc->basis = FeatureDescBasis::XYZ;
                            desc->type = FeatureDescType::VELOCITY;
                            desc->joint_id = joint_init_ids.neck_idx;
                            desc->name = "neck_velocity";
                            pose_feature->descriptors.push_back(desc);
                        }

                        //{
                        //	ref<PoseFeatureDesc> desc = make_ref<PoseFeatureDesc>();
                        //	desc->basis = FeatureDescBasis::Y;
                        //	desc->type = FeatureDescType::LOCATION;
                        //	desc->joint_id = joint_init_ids.neck_idx;
                        //	desc->name = "neck_location";
                        //	pose_feature->descriptors.push_back(desc);
                        //}

                        {
                            ref<PoseFeatureDesc> desc = make_ref<PoseFeatureDesc>();
                            desc->basis = FeatureDescBasis::XYZ;
                            desc->type = FeatureDescType::LOCATION;
                            desc->joint_id = joint_init_ids.foot_l_idx;
                            desc->name = "foot_l_location";
                            pose_feature->descriptors.push_back(desc);
                        }

                        {
                            ref<PoseFeatureDesc> desc = make_ref<PoseFeatureDesc>();
                            desc->basis = FeatureDescBasis::XYZ;
                            desc->type = FeatureDescType::VELOCITY;
                            desc->joint_id = joint_init_ids.foot_l_idx;
                            desc->name = "foot_l_velocity";
                            pose_feature->descriptors.push_back(desc);
                        }

                        {
                            ref<PoseFeatureDesc> desc = make_ref<PoseFeatureDesc>();
                            desc->basis = FeatureDescBasis::XYZ;
                            desc->type = FeatureDescType::LOCATION;
                            desc->joint_id = joint_init_ids.foot_r_idx;
                            desc->name = "foot_r_location";
                            pose_feature->descriptors.push_back(desc);
                        }

                        {
                            ref<PoseFeatureDesc> desc = make_ref<PoseFeatureDesc>();
                            desc->basis = FeatureDescBasis::XYZ;
                            desc->type = FeatureDescType::VELOCITY;
                            desc->joint_id = joint_init_ids.foot_r_idx;
                            desc->name = "foot_r_velocity";
                            pose_feature->descriptors.push_back(desc);
                        }

                        {
                            ref<PoseFeatureDesc> desc = make_ref<PoseFeatureDesc>();
                            desc->basis = FeatureDescBasis::XYZ;
                            desc->type = FeatureDescType::LOCATION;
                            desc->joint_id = joint_init_ids.hand_l_idx;
                            desc->name = "hand_l_location";
                            pose_feature->descriptors.push_back(desc);
                        }

                        //{
                        //	ref<PoseFeatureDesc> desc = make_ref<PoseFeatureDesc>();
                        //	desc->basis = FeatureDescBasis::XYZ;
                        //	desc->type = FeatureDescType::VELOCITY;
                        //	desc->joint_id = joint_init_ids.hand_l_idx;
                        //	desc->name = "hand_l_velocity";
                        //	pose_feature->descriptors.push_back(desc);
                        //}

                        {
                            ref<PoseFeatureDesc> desc = make_ref<PoseFeatureDesc>();
                            desc->basis = FeatureDescBasis::XYZ;
                            desc->type = FeatureDescType::LOCATION;
                            desc->joint_id = joint_init_ids.hand_r_idx;
                            desc->name = "hand_r_location";
                            pose_feature->descriptors.push_back(desc);
                        }

                        //{
                        //	ref<PoseFeatureDesc> desc = make_ref<PoseFeatureDesc>();
                        //	desc->basis = FeatureDescBasis::XYZ;
                        //	desc->type = FeatureDescType::LOCATION;
                        //	desc->joint_id = joint_init_ids.hand_r_idx;
                        //	desc->name = "hand_r_velocity";
                        //	pose_feature->descriptors.push_back(desc);
                        //}

                        {
                            ref<PoseFeatureDesc> desc = make_ref<PoseFeatureDesc>();
                            desc->basis = FeatureDescBasis::XYZ;
                            desc->type = FeatureDescType::LOCATION;
                            desc->joint_id = joint_init_ids.calf_l_idx;
                            desc->name = "calf_l_location";
                            pose_feature->descriptors.push_back(desc);
                        }

                        {
                            ref<PoseFeatureDesc> desc = make_ref<PoseFeatureDesc>();
                            desc->basis = FeatureDescBasis::XYZ;
                            desc->type = FeatureDescType::LOCATION;
                            desc->joint_id = joint_init_ids.calf_r_idx;
                            desc->name = "calf_r_location";
                            pose_feature->descriptors.push_back(desc);
                        }

                        {
                            ref<PoseFeatureDesc> desc = make_ref<PoseFeatureDesc>();
                            desc->basis = FeatureDescBasis::XYZ;
                            desc->type = FeatureDescType::VELOCITY;
                            desc->joint_id = joint_init_ids.calf_l_idx;
                            desc->name = "calf_l_velocity";
                            pose_feature->descriptors.push_back(desc);
                        }

                        {
                            ref<PoseFeatureDesc> desc = make_ref<PoseFeatureDesc>();
                            desc->basis = FeatureDescBasis::XYZ;
                            desc->type = FeatureDescType::VELOCITY;
                            desc->joint_id = joint_init_ids.calf_r_idx;
                            desc->name = "calf_r_velocity";
                            pose_feature->descriptors.push_back(desc);
                        }

                        database->features.push_back(pose_feature);
                    }

                    {
                        TrajectoryFeature* trajectory_feature = new TrajectoryFeature();

                        // 1, 2
                        {
                            ref<TrajectoryFeatureDesc> desc = make_ref<TrajectoryFeatureDesc>();
                            desc->type = FeatureDescType::VELOCITY;
                            desc->time_offset = 0.0f;
                            desc->name = "curr_velocity";
                            trajectory_feature->descriptors.push_back(desc);
                        }

                        // 5, 6
                        {
                            ref<TrajectoryFeatureDesc> desc = make_ref<TrajectoryFeatureDesc>();
                            desc->type = FeatureDescType::LOCATION;
                            desc->time_offset = 0.12f;
                            desc->name = "next_1_location";
                            trajectory_feature->descriptors.push_back(desc);
                        }

                        {
                            ref<TrajectoryFeatureDesc> desc = make_ref<TrajectoryFeatureDesc>();
                            desc->type = FeatureDescType::DIRECTION;
                            desc->time_offset = 0.12f;
                            desc->name = "next_1_direction";
                            trajectory_feature->descriptors.push_back(desc);
                        }

                        // 9, 10
                        {
                            ref<TrajectoryFeatureDesc> desc = make_ref<TrajectoryFeatureDesc>();
                            desc->type = FeatureDescType::LOCATION;
                            desc->time_offset = 0.32f;
                            desc->name = "next_2_location";
                            trajectory_feature->descriptors.push_back(desc);
                        }

                        {
                            ref<TrajectoryFeatureDesc> desc = make_ref<TrajectoryFeatureDesc>();
                            desc->type = FeatureDescType::DIRECTION;
                            desc->time_offset = 0.32f;
                            desc->name = "next_2_direction";
                            trajectory_feature->descriptors.push_back(desc);
                        }

                        // 13, 14
                        {
                            ref<TrajectoryFeatureDesc> desc = make_ref<TrajectoryFeatureDesc>();
                            desc->type = FeatureDescType::LOCATION;
                            desc->time_offset = 0.54f;
                            desc->name = "next_3_location";
                            trajectory_feature->descriptors.push_back(desc);
                        }

                        {
                            ref<TrajectoryFeatureDesc> desc = make_ref<TrajectoryFeatureDesc>();
                            desc->type = FeatureDescType::DIRECTION;
                            desc->time_offset = 0.54f;
                            desc->name = "next_3_direction";
                            trajectory_feature->descriptors.push_back(desc);
                        }

                        {
                            ref<TrajectoryFeatureDesc> desc = make_ref<TrajectoryFeatureDesc>();
                            desc->type = FeatureDescType::VELOCITY;
                            desc->time_offset = 0.54f;
                            desc->name = "next_3_velocity";
                            trajectory_feature->descriptors.push_back(desc);
                        }

                        // 15, 16
                        {
                            ref<TrajectoryFeatureDesc> desc = make_ref<TrajectoryFeatureDesc>();
                            desc->type = FeatureDescType::LOCATION;
                            desc->time_offset = 0.76f;
                            desc->name = "next_4_location";
                            trajectory_feature->descriptors.push_back(desc);
                        }

                        {
                            ref<TrajectoryFeatureDesc> desc = make_ref<TrajectoryFeatureDesc>();
                            desc->type = FeatureDescType::DIRECTION;
                            desc->time_offset = 0.76f;
                            desc->name = "next_4_direction";
                            trajectory_feature->descriptors.push_back(desc);
                        }

                        database->features.push_back(trajectory_feature);
                    }

                    PhaseFeature* phase_feature = new PhaseFeature();
                    {
                        {
                            ref<PhaseFeatureDesc> desc = make_ref<PhaseFeatureDesc>();
                            desc->joint_id = joint_init_ids.foot_l_idx;
                            desc->name = "foot_l_phase";
                            phase_feature->descriptors.push_back(desc);
                        }

                        {
                            ref<PhaseFeatureDesc> desc = make_ref<PhaseFeatureDesc>();
                            desc->joint_id = joint_init_ids.foot_r_idx;
                            desc->name = "foot_r_phase";
                            phase_feature->descriptors.push_back(desc);
                        }

                        database->features.push_back(phase_feature);
                    }

                    auto add_animation = [&](std::string_view path)
                    {
                        ref<AnimationAssetClip> animation = provider.load_game_asset_from_file<AnimationAssetClip>(get_asset_path(path));
                        animation->load_job.wait_for_completion();
                        database->animations.push_back(animation);
                    };

                    database->sample_rate = 15.0f;
                    database->narrow_phase_params.flags = NarrowPhaseFlags::SAME_FRAME_CHECK;
                    database->narrow_phase_params.same_frame_time_threshold = 0.16f;

                    add_animation("/resources/assets/test_character/animations/Standing Idle_mixamo.com_clip0.eanm");
                    add_animation("/resources/assets/test_character/animations/Unarmed Idle_mixamo.com_clip0.eanm");
                    add_animation("/resources/assets/test_character/animations/Run To Stop_mixamo.com_clip0.eanm");
                    add_animation("/resources/assets/test_character/animations/Walking_mixamo.com_clip0.eanm");
                    add_animation("/resources/assets/test_character/animations/Walking Left Turn_mixamo.com_clip0.eanm");
                    add_animation("/resources/assets/test_character/animations/Walking Right Turn_mixamo.com_clip0.eanm");
                    add_animation("/resources/assets/test_character/animations/Running_mixamo.com_clip0.eanm");
                    add_animation("/resources/assets/test_character/animations/StrafeL_mixamo.com_clip0.eanm");
                    add_animation("/resources/assets/test_character/animations/StrafeR_mixamo.com_clip0.eanm");
                    // add_animation("/resources/assets/test_character/animations/Run Forward Arc Left_mixamo.com_clip0.eanm");
                    // add_animation("/resources/assets/test_character/animations/Run Forward Arc Right_mixamo.com_clip0.eanm");
                    add_animation("/resources/assets/test_character/animations/Running Left Turn_mixamo.com_clip0.eanm");
                    add_animation("/resources/assets/test_character/animations/Running Right Turn_mixamo.com_clip0.eanm");
                    add_animation("/resources/assets/test_character/animations/Running Turn 180L_mixamo.com_clip0.eanm");
                    add_animation("/resources/assets/test_character/animations/Running Turn 180R_mixamo.com_clip0.eanm");
                    add_animation("/resources/assets/test_character/animations/Sprint_mixamo.com_clip0.eanm");
                    add_animation("/resources/assets/test_character/animations/Jog Strafe Left_mixamo.com_clip0.eanm");
                    add_animation("/resources/assets/test_character/animations/Jog Strafe Right_mixamo.com_clip0.eanm");
                    add_animation("/resources/assets/test_character/animations/Backward Right Turn_mixamo.com_clip0.eanm");
                    add_animation("/resources/assets/test_character/animations/Backward Left Turn_mixamo.com_clip0.eanm");
                    add_animation("/resources/assets/test_character/animations/Run Backward Arc Left_mixamo.com_clip0.eanm");
                    add_animation("/resources/assets/test_character/animations/Run Backward Arc Right_mixamo.com_clip0.eanm");
                    add_animation("/resources/assets/test_character/animations/Running Backward_mixamo.com_clip0.eanm");
                    add_animation("/resources/assets/test_character/animations/Walking Backwards_mixamo.com_clip0.eanm");
                    add_animation("/resources/assets/test_character/animations/Run Backward_mixamo.com_clip0.eanm");
                    add_animation("/resources/assets/test_character/animations/Slow Jog Backwards_mixamo.com_clip0.eanm");
                    add_animation("/resources/assets/test_character/animations/Jog Backward DiagonalL_mixamo.com_clip0.eanm");
                    add_animation("/resources/assets/test_character/animations/Jog Backward DiagonalR_mixamo.com_clip0.eanm");
                    add_animation("/resources/assets/test_character/animations/Catwalk Walk Forward Arc 90L_mixamo.com_clip0.eanm");
                    add_animation("/resources/assets/test_character/animations/Catwalk Walk Forward Arc 90R_mixamo.com_clip0.eanm");
                    add_animation("/resources/assets/test_character/animations/Catwalk Walk Start Turn 180 Left_mixamo.com_clip0.eanm");
                    add_animation("/resources/assets/test_character/animations/Catwalk Walk Start Turn 180 Right_mixamo.com_clip0.eanm");
                    add_animation("/resources/assets/test_character/animations/Change DirectionL_mixamo.com_clip0.eanm");
                    add_animation("/resources/assets/test_character/animations/Change DirectionR_mixamo.com_clip0.eanm");
                    add_animation("/resources/assets/test_character/animations/Running To Turn_mixamo.com_clip0.eanm");
                    add_animation("/resources/assets/test_character/animations/Catwalk Sequence 02L_mixamo.com_clip0.eanm");
                    add_animation("/resources/assets/test_character/animations/Catwalk Sequence 02R_mixamo.com_clip0.eanm");
                    add_animation("/resources/assets/test_character/animations/Start Walking_mixamo.com_clip0.eanm");
                    add_animation("/resources/assets/test_character/animations/Stop Walking_mixamo.com_clip0.eanm");
                    add_animation("/resources/assets/test_character/animations/Turn To RunningL_mixamo.com_clip0.eanm");
                    add_animation("/resources/assets/test_character/animations/Turn To RunningR_mixamo.com_clip0.eanm");

                    database->generate_initial_data();

                    database->search_dimension = 40;
                    database->static_thresholds.resize(database->total_features_per_sample);
                    database->weights.resize(database->total_features_per_sample, 1.0f);

                    const uint32 start_from = 38;

                    database->weights[0] = 4.0f;
                    database->weights[1] = 8.0f;
                    database->weights[2] = 4.0f;

                    database->weights[3] = 20.0f;
                    database->weights[4] = 8.0f;
                    database->weights[5] = 20.0f;

                    database->weights[6] = 7.0f;
                    database->weights[7] = 4.5f;
                    database->weights[8] = 7.0f;

                    database->weights[9] = 10.0f;
                    database->weights[10] = 6.0f;
                    database->weights[11] = 10.0f;
                    database->weights[12] = 13.0f;
                    database->weights[13] = 13.0f;
                    database->weights[14] = 13.0f;

                    database->weights[15] = 10.0f;
                    database->weights[16] = 6.0f;
                    database->weights[17] = 10.0f;
                    database->weights[18] = 13.0f;
                    database->weights[19] = 13.0f;
                    database->weights[20] = 13.0f;

                    database->weights[21] = 3.0f;
                    database->weights[22] = 3.0f;
                    database->weights[23] = 3.0f;
                    database->weights[24] = 3.0f;
                    database->weights[25] = 3.0f;
                    database->weights[26] = 3.0f;

                    database->weights[27] = 6.0f;
                    database->weights[28] = 4.0f;
                    database->weights[29] = 6.0f;
                    database->weights[30] = 7.0f;
                    database->weights[31] = 7.0f;
                    database->weights[32] = 12.0f;

                    database->weights[33] = 6.0f;
                    database->weights[34] = 4.0f;
                    database->weights[35] = 6.0f;
                    database->weights[36] = 7.0f;
                    database->weights[37] = 7.0f;
                    database->weights[38] = 12.0f;

                    database->weights[start_from + 1] = 30.0f;
                    database->weights[start_from + 2] = 30.0f;

                    database->weights[start_from + 3] = 8.0f;
                    database->weights[start_from + 4] = 8.0f;
                    database->weights[start_from + 5] = 4.0f;
                    database->weights[start_from + 6] = 4.0f;

                    database->weights[start_from + 7] = 65.0f;
                    database->weights[start_from + 8] = 65.0f;
                    database->weights[start_from + 9] = 25.0f;
                    database->weights[start_from + 10] = 25.0f;

                    database->weights[start_from + 11] = 90.0f;
                    database->weights[start_from + 12] = 90.0f;
                    database->weights[start_from + 13] = 80.0f;
                    database->weights[start_from + 14] = 80.0f;
                    database->weights[start_from + 9] = 20.0f;
                    database->weights[start_from + 10] = 20.0f;

                    database->weights[start_from + 11] = 150.0f;
                    database->weights[start_from + 12] = 150.0f;
                    database->weights[start_from + 13] = 100.0f;
                    database->weights[start_from + 14] = 100.0f;

                    const uint32 phase_from = database->total_features_per_sample - phase_feature->get_feature_size();
                    for (uint32 i = phase_from; i < database->total_features_per_sample; ++i)
                    {
                        database->weights[i] = 25.0f;
                    }

                    database->mark(skeleton_component);

                    database->bake(skeleton_component->skeleton.get());

                    JobHandle save_job = provider.save_game_asset_to_file_async<MotionMatchingDatabase>(get_asset_path("/resources/assets/test_character/mmdb/locomotion"), database.get());
                    save_job.wait_for_completion();
                }

                MotionDatabaseRegistry::add_to_registry(database);

                animation_component->enable_root_motion = false;

                character.add_component<MotionDataComponent>();
                character.add_component<CharacterLocomotionComponent>();

                character.add_component<MotionMatchingComponent>();

                RagdollSettings settings;
                settings.shapes_settings.head_radius = 0.1f;
                settings.shapes_settings.neck_radius = 0.05f;
                settings.shapes_settings.clavicle_radius = 0.035f;
                settings.shapes_settings.arm_radius = 0.058f;
                settings.shapes_settings.forearm_radius = 0.058f;
                settings.shapes_settings.foot_radius = 0.035f;
                settings.shapes_settings.hand_radius = 0.035f;
                settings.shapes_settings.calf_radius = 0.065f;

                settings.local_shape_settings.abdomen_joint_adjastment = vec3(0.0f, 0.05f, 0.0f);
                settings.local_shape_settings.pelvis_joint_adjastment = vec3(0.0f, -0.03f, 0.0f);

                settings.object_space_settings.head_end_joint_adjastment = vec3(0.0f, 0.15f, 0.0f);
                settings.object_space_settings.head_joint_adjastment = vec3(0.0f, 0.12f, 0.0f);
                settings.object_space_settings.neck_joint_adjastment = vec3(0.0f, 0.05f, 0.0f);
                settings.object_space_settings.abdomen_joint_adjastment = vec3(0.0f, 0.07f, 0.0f);
                settings.object_space_settings.pelvis_joint_adjastment = vec3(0.0f, -0.03f, 0.0f);
                settings.object_space_settings.left_arm_joint_adjastment = vec3(0.0f, 0.035f, 0.0f);
                settings.object_space_settings.right_arm_joint_adjastment = vec3(0.0f, 0.035f, 0.0f);
                settings.object_space_settings.left_foot_end_joint_adjastment = vec3(0.0f, 0.0f, -0.04f);
                settings.object_space_settings.right_foot_end_joint_adjastment = vec3(0.0f, 0.0f, -0.04f);

                settings.scaler_settings.upper_body_height_modifier = 0.65f;
                settings.scaler_settings.upper_body_radius_modifier = 0.62f;
                settings.scaler_settings.middle_body_height_modifier = 0.82f;
                settings.scaler_settings.middle_body_radius_modifier = 0.82f;
                settings.scaler_settings.lower_body_height_modifier = 0.95f;
                settings.scaler_settings.lower_body_radius_modifier = 0.9f;
                settings.scaler_settings.clavicle_height_modifier = 1.0f;
                settings.scaler_settings.arm_height_modifier = 1.0f;
                settings.scaler_settings.forearm_height_modifier = 1.1f;
                settings.scaler_settings.neck_height_modifier = 1.0f;

                // RagdollComponent* ragdoll_component = character.add_component<RagdollComponent>();
                // ragdoll_component->simulated = true;
                PhysicalAnimationComponent* ragdoll_component = character.add_component<PhysicalAnimationComponent>();
                ragdoll_component->joint_init_ids = joint_init_ids;
                ragdoll_component->settings = settings;

                // character.add_component<RagdollDismembermentComponent>();

                auto make_limited_joint = [](float limit_deg, float stiffness = 0.0f)
                {
                    FBIKJointSettings joint;
                    joint.angular_stiffness = vec3(stiffness);
                    for (IKLimitType& type : joint.angular_limit_types)
                    {
                        type = IKLimitType::LIMITED;
                    }
                    joint.angular_limits_deg = vec3(limit_deg);
                    return joint;
                };

                if (false)
                {
                    IKComponent* ik_component = character.add_component<IKComponent>();

                    std::vector<FBIKJointSettings> seeds(4);
                    seeds[0] = make_limited_joint(25.0f);
                    seeds[1] = make_limited_joint(95.0f);
                    seeds[2] = make_limited_joint(145.0f);
                    seeds[3] = make_limited_joint(70.0f);

                    ik_component->add_chain_to_component({"mixamorig:LeftShoulder", "mixamorig:LeftArm", "mixamorig:LeftForeArm", "mixamorig:LeftHand"}, &seeds);

                    auto& arm = ik_component->chains[0];
                    arm.target_world_position = vec3::zero;
                    arm.pole_vector_local = -vec3::forward;
                    arm.weight = 1.0f;
                    arm.iterations = 16;
                    arm.tolerance = 0.005f;

                    ik_component->init();

                    {
                        Entity hand_target = world->create_entity("HandIkTarget");

                        IKTargetComponent* hand_ik_target = hand_target.add_component<IKTargetComponent>();
                        hand_ik_target->owner_entity = character.get_handle();
                        hand_ik_target->chain_index = 0;
                        hand_ik_target->weight = 1.0f;
                    }
                }

                if (true)
                {
                    FBIKComponent* full_body_ik = character.add_component<FBIKComponent>();
                    full_body_ik->solver_type = FBIKSolverType::JACOBIAN;
                    full_body_ik->set_root("mixamorig:Hips");
                    full_body_ik->settings.iterations = 16;
                    full_body_ik->enabled = false;

                    FBIKJointSettings hips_settings;
                    hips_settings.angular_stiffness = vec3(1.0f);
                    full_body_ik->set_joint_settings("mixamorig:Hips", hips_settings);

                    const FBIKJointSettings spine_settings = make_limited_joint(30.0f, 0.5f);
                    full_body_ik->set_joint_settings("mixamorig:Spine", spine_settings);
                    full_body_ik->set_joint_settings("mixamorig:Spine1", spine_settings);
                    full_body_ik->set_joint_settings("mixamorig:Spine2", spine_settings);

                    const FBIKJointSettings clavicle_settings = make_limited_joint(30.0f);
                    full_body_ik->set_joint_settings("mixamorig:LeftShoulder", clavicle_settings);
                    full_body_ik->set_joint_settings("mixamorig:RightShoulder", clavicle_settings);

                    const SkeletonPose default_pose = skeleton->get_default_pose();
                    const trs character_transform = transform_component->get_world_transform();

                    auto add_target = [&](const char* joint_name, float rotation_weight, const vec3& pole_vector_local)
                    {
                        const uint32 effector_index = full_body_ik->add_effector(joint_name);
                        if (effector_index == INVALID_JOINT)
                        {
                            return;
                        }

                        IKEffector& effector = full_body_ik->effectors[effector_index];
                        effector.pole_vector_local = pole_vector_local;
                        effector.pole_weight = 0.5f;

                        Entity target = world->create_entity(joint_name);
                        target.get_component<TransformComponent>()->set_world_transform(
                            character_transform * SkeletonUtils::get_object_space_joint_transform(default_pose, skeleton.get(), effector.joint_id));

                        FBIKTargetComponent* target_component = target.add_component<FBIKTargetComponent>();
                        target_component->owner_entity = character.get_handle();
                        target_component->effector_index = effector_index;
                        target_component->position_weight = 1.0f;
                        target_component->rotation_weight = rotation_weight;
                    };

                    add_target("mixamorig:RightHand", 0.0f, -vec3::forward);

                    // add_target("mixamorig:Hips", 0.0f, vec3::zero);
                    // add_target("mixamorig:LeftFoot", 1.0f, vec3::forward);
                    // add_target("mixamorig:RightFoot", 1.0f, vec3::forward);

                    full_body_ik->init();
                }

                if (true)
                {
                    Entity idle_character = world->create_entity("IdleCharacter");
                    idle_character.add_component<MeshComponent>(mesh);
                    idle_character.get_component<TransformComponent>()->set_world_transform(trs{vec3(1.5f, -5.0f, 4.0f), quat::identity, vec3(1.0f)});

                    SkeletonComponent* idle_skeleton_component = idle_character.add_component<SkeletonComponent>();
                    idle_skeleton_component->skeleton = skeleton;
                    idle_skeleton_component->apply_pose(skeleton->get_default_pose());

                    AnimationComponent* idle_animation_component = idle_character.add_component<AnimationComponent>();
                    idle_animation_component->play = true;
                    idle_animation_component->loop = true;
                    idle_animation_component->enable_root_motion = false;

                    ref<AnimationAssetClip> idle_clip = provider.load_game_asset_from_file<AnimationAssetClip>(get_asset_path("/resources/assets/test_character/animations/Unarmed Idle_mixamo.com_clip0.eanm"));
                    idle_clip->load_job.wait_for_completion();
                    idle_animation_component->current_animation = idle_clip;
                    idle_animation_component->current_anim_position = 0.0f;

                    PhysicalAnimationComponent* idle_ragdoll_component = idle_character.add_component<PhysicalAnimationComponent>();
                    idle_ragdoll_component->joint_init_ids = joint_init_ids;
                    idle_ragdoll_component->settings = settings;

                    CharacterControllerComponent* idle_cct_component = idle_character.add_component<CharacterControllerComponent>();
                    idle_cct_component->collision_type = static_cast<CollisionType>(GameCollisionType::CCT);
                    idle_cct_component->height = 1.2f;
                    idle_cct_component->radius = 0.18f;
                    idle_cct_component->step_offset = 0.05f;
                    idle_cct_component->handle_non_cct_contacts = false;
                }
            }
        }

        //{
        //	Entity cloth = world->create_entity("Cloth");

        //	TransformComponent* transform_component = cloth.get_component<TransformComponent>();
        //	transform_component->set_world_transform(trs{ vec3(0.0f, 20.0f, 10.0f), quat::identity, vec3(1.0f) });

        //	physics::ClothPhysicsComponent* cloth_component = cloth.add_component<physics::ClothPhysicsComponent>();
        //	cloth_component->num_x = 10;
        //	cloth_component->num_z = 10;
        //}

        /*if (auto mesh = import_mesh_from_file_async(get_asset_path("/resources/assets/Sponza/sponza.obj"), MESH_CREATION_FLAGS_UNREAL_ASSET))
        {
            auto sponza = world->create_entity("Sponza");
            sponza.add_component<MeshComponent>(mesh);

            TransformComponent* transform_component = sponza.get_component<TransformComponent>();
            transform_component->set_world_position(vec3(5.0f, -3.75f, 35.0f));
        }*/

        if (auto mesh = import_mesh_from_file_async(get_asset_path("/resources/assets/box.fbx"), MESH_CREATION_FLAGS_UNREAL_ASSET))
        {
            // ref<MultiMesh> mesh = provider.load_game_asset_from_file<MultiMesh>(get_asset_path("/resources/assets/box.emesh"), true, {}, MESH_CREATION_FLAGS_DEFAULT);
            mesh->load_job.wait_for_completion();

            Entity box = world->create_entity("Box");
            box.add_component<MeshComponent>(mesh)->is_hidden = true;

            TransformComponent* transform_component = box.get_component<TransformComponent>();
            transform_component->set_world_position(vec3(5.0f, -4.0f, -5.0f));

            DestructibleComponent* descructible_component = box.add_component<DestructibleComponent>(DestructibleComponent::Type::FRACTURE_BASED);
            descructible_component->fracture_desc.chunks_count = 10;
            descructible_component->fracture_desc.density = 100.0f;
            descructible_component->fracture_desc.break_force = 100.0f;
            descructible_component->material = PhysicsEngine::get_physics_core()->create_material(0.1f, 0.8f, 0.7f);
        }

        if (true)
        {
            const fs::path model_path = get_asset_path("/resources/assets/bmw-330i/source/FINAL_MODEL_25.fbx");
            const fs::path prefab_path = model_path.parent_path() / "bmw_330i.eprefab";

            Entity vehicle = Entity::null;
            if (fs::exists(prefab_path))
            {
                const std::vector<Entity> roots = PrefabUtils::instantiate(world, prefab_path);
                if (!roots.empty())
                {
                    vehicle = roots.front();
                }
            }
            else if (ref<MultiMesh> source = import_mesh_from_file(model_path, MESH_CREATION_FLAGS_DEFAULT);
                     source != nullptr && source->model_asset.has_value())
            {
                VehicleImportSettings settings;
                settings.mass = 1550.0f;

                const VehicleImportSetup setup = VehicleImportUtils::analyze(*source->model_asset, settings);
                if (setup.error.empty())
                {
                    vehicle = VehicleImportUtils::create_vehicle(world, source, setup, model_path.parent_path(), "bmw_330i");
                }
                else
                {
                    LOG_ERROR("BMW 330i import: %s", setup.error.c_str());
                }
            }

            if (vehicle.is_valid())
            {
                vehicle.get_component<TransformComponent>()->set_world_position(vec3(-8.0f, -4.7f, 0.0f));

                // camera_entity = vehicle;

                GameplayTppCameraComponent* tpp_camera_component = vehicle.add_component<GameplayTppCameraComponent>();
                tpp_camera_component->follow_target_heading = true;
                tpp_camera_component->orbit.yaw = 180.0f;
                tpp_camera_component->orbit.pitch = 12.0f;
                tpp_camera_component->orbit.distance = 7.0f;
                tpp_camera_component->orbit.height = 1.2f;

                CameraHolderComponent* camera_holder_component = vehicle.add_component<CameraHolderComponent>();
                camera_holder_component->set_camera_type(CameraHolderComponent::USER_DEFINED);
                camera_holder_component->set_render_camera(&renderer_holder_rc->camera);
                // vehicle.add_component<InputSenderComponent>()->add_reciever(vehicle.add_component<InputReceiverComponent>());
            }
        }

        Entity plane = world->create_entity("Platform");
        plane.add_component<MeshComponent>(ground_mesh);
        plane.get_component<TransformComponent>()->set_world_transform(trs{vec3(10.0f, -9.0f, 0.0f), quat(vec3(1.0f, 0.0f, 0.0f), deg2rad(0.0f)), vec3(5.0f, 1.0f, 5.0f)});
        plane.add_component<PlaneShapeComponent>()->local_position = vec3(0.0f, 4.0f, 0.0f);
        plane.add_component<StaticBodyComponent>()->simulated = true;

        ground_mesh->load_state = AssetLoadState::LOADED;
        ground_mesh->mesh = builder.create_dx_mesh();

        if (false)
        {
            ref<PhysicsMaterial> terrain_material = PhysicsEngine::get_physics_core()->create_material(0.1f, 0.5f, 0.5f);

            uint32 num_terrain_chunks = 8;
            float terrain_chunk_size = 15.f;

            PbrMaterialDesc terrain_ground_desc;
            terrain_ground_desc.albedo = get_asset_path("/resources/assets/terrain/grass.bmp");
            terrain_ground_desc.normal = get_asset_path("/resources/assets/terrain/grass_normal.bmp");
            terrain_ground_desc.albedo_flags &= ~IMAGE_LOAD_FLAGS_COMPRESS;

            PbrMaterialDesc terrain_rock_desc;
            terrain_rock_desc.albedo = get_asset_path("/resources/assets/terrain/rock.bmp");
            terrain_rock_desc.normal = get_asset_path("/resources/assets/terrain/rock_normal.bmp");

            PbrMaterialDesc terrain_mud_desc;

            auto terrain_ground_material = create_pbr_material(terrain_ground_desc);
            auto terrain_rock_material = create_pbr_material(terrain_rock_desc);
            auto terrain_mud_material = create_pbr_material(terrain_mud_desc);

            Entity terrain = world->create_entity("Terrain");

            terrain.get_component<TransformComponent>()->set_world_position(vec3(0.0f, -20.0f, 0.0f));

            terrain.add_component<TerrainComponent>(num_terrain_chunks, terrain_chunk_size, 10.f, terrain_ground_material, terrain_rock_material, terrain_mud_material);
            TerrainShapeComponent* terrain_shape = terrain.add_component<TerrainShapeComponent>();
            terrain_shape->material = terrain_material;
            terrain_shape->collision_type.get_for_write() = CollisionType::TERRAIN;
            terrain.add_component<StaticBodyComponent>()->simulated = true;
            // terrain.add_component<GrassComponent>(GrassSettings{ .bladeHeight = 0.15f, .bladeWidth = 0.0075f,.numGrassBladesPerChunkDim = 150 });

            // std::vector<proc_placement_layer_desc> layers =
            //{
            //	proc_placement_layer_desc {
            //		"Trees and rocks",
            //		5.f,
            //		{
            //			loadMeshFromFile("assets/hoewa/hoewa1.fbx"),
            //			loadMeshFromFile("assets/hoewa/hoewa2.fbx"),
            //			loadMeshFromFile("assets/desert/rock1.fbx"),
            //			loadMeshFromFile("assets/desert/rock4.fbx"),
            //		}
            //	}
            // };
            //.addComponent<heightmap_collider_component>(numTerrainChunks, terrainChunkSize, physics_material{ physics_material_type_metal, 0.1f, 1.f, 4.f })
            //.addComponent<proc_placement_component>(layers) // TODO: This could be deferred if we want to load the meshes asynchronously.
        }
    }

    void GameInitSystem::update(float dt)
    {
        using namespace physics;

        const InputReceiverComponent* input_receiver = camera_entity.get_component_if_exists<InputReceiverComponent>();
        if (input_receiver == nullptr || camera_entity.get_component_if_exists<W4VehicleComponent>() != nullptr)
        {
            return;
        }

        const UserInput& frame_input = input_receiver->get_frame_input();
        if (frame_input.keyboard[KeyCode::KEY_SPACE].press_event)
        {
            Entity sphere = world->create_entity("Sphere");

            const ref<PhysicsMaterial>& material = PhysicsEngine::get_physics_core()->get_default_material();

            SphereShapeComponent* sphere_shape_component = sphere.add_component<SphereShapeComponent>();
            sphere_shape_component->collision_type = static_cast<CollisionType>(GameCollisionType::DYNAMICS);
            sphere_shape_component->radius = 0.25f;
            sphere_shape_component->material = material;
            sphere.add_component<MeshComponent>(sphere_mesh);

            CameraHolderComponent* camera_holder_component = camera_entity.get_component<CameraHolderComponent>();

            trs camera_world_transform = trs{camera_holder_component->get_render_camera()->position, camera_holder_component->get_render_camera()->rotation, vec3(0.2f)};
            sphere.get_component<TransformComponent>()->set_world_transform(camera_world_transform);

            DynamicBodyComponent* dynamic_body_component = sphere.add_component<DynamicBodyComponent>();
            dynamic_body_component->ccd.get_for_write() = true;
            dynamic_body_component->mass.get_for_write() = 10.0f;
            dynamic_body_component->simulated.get_for_write() = true;
            dynamic_body_component->use_gravity.get_for_write() = true;
            dynamic_body_component->sleep_threshold.get_for_write() = 0.05f;

            Force& force = dynamic_body_component->forces.emplace_back();
            force.force = (camera_holder_component->get_render_camera()->rotation * quat(vec3(0.0f, 1.0f, 0.0f), M_PI)) * vec3(0.0f, 0.0f, 1.0f) * sphere_speed;
            force.mode = ForceMode::FORCE;
        }
    }
}