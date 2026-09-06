#include "game/game_init_system.h"
#include "game/physics/gameplay_physics_types.h"

#include <core/ecs/input_receiver_component.h>
#include <core/ecs/input_sender_component.h>
#include <core/string.h>
#include <core/ecs/camera_holder_component.h>
#include <core/debug/debug_var.h>

#include <asset/game_asset.h>

#include <animation/animation_clip.h>
#include <animation/animation_clip_utils.h>

#include <rendering/ecs/renderer_holder_root_component.h>

#include <ecs/update_groups.h>
#include <ecs/base_components/transform_component.h>
#include <ecs/rendering/world_renderer.h>
#include <ecs/rendering/scene_rendering.h>
#include <ecs/rendering/mesh_component.h>

#include <physics/body_component.h>
#include <physics/core/physics.h>
#include <physics/shape_component.h>
#include <physics/cct_component.h>
#include <physics/basic_objects.h>
#include <physics/joint.h>
#include <physics/ragdolls/ragdoll_component.h>
#include <physics/physical_animation/physical_animation_component.h>
#include <physics/collisions_holder_root_component.h>
#include <physics/pbd/pbd_cloth_component.h>
#include <physics/vehicles/w4_vehicle_component.h>
#include <physics/physical_animation/dismemberment/ragdoll_dismemberment_component.h>
#include <physics/destructions/destructible_component.h>

#include <motion_matching/trajectory/trajectory_component.h>
#include <motion_matching/motion/motion_component.h>
#include <motion_matching/features/pose_feature.h>
#include <motion_matching/features/trajectory_feature.h>
#include <motion_matching/motion_matching_database.h>
#include <motion_matching/features/motion_matching_feature_set.h>
#include <motion_matching/motion_matching_component.h>
#include <motion_matching/motion_data_component.h>

#include "game/movement/character_locomotion_component.h"
#include "game/camera/gameplay_tpp_camera_component.h"

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

	static DebugVar<float> sphere_speed = DebugVar<float>("test.sphere_speed", 15000.0f);

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
			mesh_builder builder;

			sphere_render_material = get_default_pbr_material();

			sphere_mesh = make_ref<MultiMesh>();
			builder.pushSphere({ });
			sphere_mesh->submeshes.push_back({ builder.endSubmesh(), {}, trs::identity, sphere_render_material });

			sphere_mesh->mesh = builder.createDXMesh();
			sphere_mesh->load_state = AssetLoadState::LOADED;
		}

		RendererHolderRootComponent* renderer_holder_rc = world->add_root_component<RendererHolderRootComponent>();
		ASSERT(renderer_holder_rc != nullptr);

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

		mesh_builder builder;

		ref<MultiMesh> ground_mesh = make_ref<MultiMesh>();
		builder.pushBox({ vec3(0.f), vec3(30.f, 4.f, 30.f) });
		ground_mesh->submeshes.push_back({ builder.endSubmesh(), {}, trs::identity, default_plane_mat });

		GameAssetsProvider provider;

		{
			if (true)
			{
				if (false)
				{
					import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Change DirectionR.fbx"),
						mesh_creation_flags_animated | mesh_creation_flags_sm_to_m | mesh_creation_flags_generate_root_motion);

					import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Run To Stop.fbx"),
						mesh_creation_flags_animated | mesh_creation_flags_sm_to_m | mesh_creation_flags_generate_root_motion);

					import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Change DirectionL.fbx"),
						mesh_creation_flags_animated | mesh_creation_flags_sm_to_m | mesh_creation_flags_generate_root_motion);

					import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Run Forward Arc Left.fbx"),
						mesh_creation_flags_animated | mesh_creation_flags_sm_to_m | mesh_creation_flags_generate_root_motion);

					import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Run Forward Arc Right.fbx"),
						mesh_creation_flags_animated | mesh_creation_flags_sm_to_m | mesh_creation_flags_generate_root_motion);

					import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Running Turn 180R.fbx"),
						mesh_creation_flags_animated | mesh_creation_flags_sm_to_m | mesh_creation_flags_generate_root_motion);

					import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Running Turn 180L.fbx"),
						mesh_creation_flags_animated | mesh_creation_flags_sm_to_m | mesh_creation_flags_generate_root_motion);

					import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Turn To RunningL.fbx"),
						mesh_creation_flags_animated | mesh_creation_flags_sm_to_m | mesh_creation_flags_generate_root_motion);

					import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Turn To RunningR.fbx"),
						mesh_creation_flags_animated | mesh_creation_flags_sm_to_m | mesh_creation_flags_generate_root_motion);

					import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Running Left Turn.fbx"),
						mesh_creation_flags_animated | mesh_creation_flags_sm_to_m | mesh_creation_flags_generate_root_motion);

					import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Running Right Turn.fbx"),
						mesh_creation_flags_animated | mesh_creation_flags_sm_to_m | mesh_creation_flags_generate_root_motion);

					import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Running.fbx"),
						mesh_creation_flags_animated | mesh_creation_flags_sm_to_m | mesh_creation_flags_generate_root_motion);

					import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Unarmed Idle.fbx"),
						mesh_creation_flags_animated | mesh_creation_flags_sm_to_m | mesh_creation_flags_generate_root_motion);

					import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Catwalk Sequence 02L.fbx"),
						mesh_creation_flags_animated | mesh_creation_flags_sm_to_m | mesh_creation_flags_generate_root_motion);

					import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Catwalk Sequence 02R.fbx"),
						mesh_creation_flags_animated | mesh_creation_flags_sm_to_m | mesh_creation_flags_generate_root_motion);

					import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Catwalk Walk Start Turn 180 Left.fbx"),
						mesh_creation_flags_animated | mesh_creation_flags_sm_to_m | mesh_creation_flags_generate_root_motion);

					import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Catwalk Walk Start Turn 180 Right.fbx"),
						mesh_creation_flags_animated | mesh_creation_flags_sm_to_m | mesh_creation_flags_generate_root_motion);

					import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Catwalk Walk Forward Arc 90R.fbx"),
						mesh_creation_flags_animated | mesh_creation_flags_sm_to_m | mesh_creation_flags_generate_root_motion);

					import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Catwalk Walk Forward Arc 90L.fbx"),
						mesh_creation_flags_animated | mesh_creation_flags_sm_to_m | mesh_creation_flags_generate_root_motion);

					import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Stop Walking.fbx"),
						mesh_creation_flags_animated | mesh_creation_flags_sm_to_m | mesh_creation_flags_generate_root_motion);

					import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Walking Right Turn.fbx"),
						mesh_creation_flags_animated | mesh_creation_flags_sm_to_m | mesh_creation_flags_generate_root_motion);

					import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Walking Left Turn.fbx"),
						mesh_creation_flags_animated | mesh_creation_flags_sm_to_m | mesh_creation_flags_generate_root_motion);

					import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Jog Backward DiagonalR.fbx"),
						mesh_creation_flags_animated | mesh_creation_flags_sm_to_m | mesh_creation_flags_generate_root_motion);

					import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Jog Backward DiagonalL.fbx"),
						mesh_creation_flags_animated | mesh_creation_flags_sm_to_m | mesh_creation_flags_generate_root_motion);

					import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Walking Backwards.fbx"),
						mesh_creation_flags_animated | mesh_creation_flags_sm_to_m | mesh_creation_flags_generate_root_motion);

					import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Sprint.fbx"),
						mesh_creation_flags_animated | mesh_creation_flags_sm_to_m | mesh_creation_flags_generate_root_motion);

					import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Start Walking.fbx"),
						mesh_creation_flags_animated | mesh_creation_flags_sm_to_m | mesh_creation_flags_generate_root_motion);

					import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Standing Idle.fbx"),
						mesh_creation_flags_animated | mesh_creation_flags_sm_to_m | mesh_creation_flags_generate_root_motion);

					import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Walking.fbx"),
						mesh_creation_flags_animated | mesh_creation_flags_sm_to_m | mesh_creation_flags_generate_root_motion);

					import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Running To Turn.fbx"),
						mesh_creation_flags_animated | mesh_creation_flags_sm_to_m | mesh_creation_flags_generate_root_motion);

					import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Jog Strafe Right.fbx"),
						mesh_creation_flags_animated | mesh_creation_flags_sm_to_m | mesh_creation_flags_generate_root_motion);

					import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Jog Strafe Left.fbx"),
						mesh_creation_flags_animated | mesh_creation_flags_sm_to_m | mesh_creation_flags_generate_root_motion);

					import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Backward Right Turn.fbx"),
						mesh_creation_flags_animated | mesh_creation_flags_sm_to_m | mesh_creation_flags_generate_root_motion);

					import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Backward Left Turn.fbx"),
						mesh_creation_flags_animated | mesh_creation_flags_sm_to_m | mesh_creation_flags_generate_root_motion);

					import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Running To Turn.fbx"),
						mesh_creation_flags_animated | mesh_creation_flags_sm_to_m | mesh_creation_flags_generate_root_motion);

					import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Run Backward Arc Left.fbx"),
						mesh_creation_flags_animated | mesh_creation_flags_sm_to_m | mesh_creation_flags_generate_root_motion);

					import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Run Backward Arc Right.fbx"),
						mesh_creation_flags_animated | mesh_creation_flags_sm_to_m | mesh_creation_flags_generate_root_motion);

					import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Running Backward.fbx"),
						mesh_creation_flags_animated | mesh_creation_flags_sm_to_m | mesh_creation_flags_generate_root_motion);

					import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/StrafeR.fbx"),
						mesh_creation_flags_animated | mesh_creation_flags_sm_to_m | mesh_creation_flags_generate_root_motion);

					import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/StrafeL.fbx"),
						mesh_creation_flags_animated | mesh_creation_flags_sm_to_m | mesh_creation_flags_generate_root_motion);

					import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Slow Jog Backwards.fbx"),
						mesh_creation_flags_animated | mesh_creation_flags_sm_to_m | mesh_creation_flags_generate_root_motion);

					import_animations_and_skeletons(get_asset_path("/resources/assets/test_character/Run Backward.fbx"),
						mesh_creation_flags_animated | mesh_creation_flags_sm_to_m | mesh_creation_flags_generate_root_motion);
				}

				//ref<MultiMesh> mesh = import_animated_mesh_from_file_async(get_asset_path("/resources/assets/test_character/Walking.fbx"),
				//	mesh_creation_flags_animated | mesh_creation_flags_sm_to_m | mesh_creation_flags_generate_root_motion | mesh_creation_flags_override_textures_with_local_folder);

				ref<MultiMesh> mesh = provider.load_game_asset_from_file<MultiMesh>(get_asset_path("/resources/assets/test_character/Character.emesh"), true, {}, mesh_creation_flags_animated);

				character = world->create_entity("Character");
				camera_entity = character;

				character.add_component<MeshComponent>(mesh);
				character.add_component<GameplayTppCameraComponent>();
				CameraHolderComponent* camera_holder_component = character.add_component<CameraHolderComponent>();
				camera_holder_component->set_camera_type(CameraHolderComponent::USER_DEFINED);
				camera_holder_component->set_render_camera(&renderer_holder_rc->camera);
				character.add_component<InputSenderComponent>()->add_reciever(character.add_component<InputReceiverComponent>());

				TransformComponent* transform_component = character.get_component<TransformComponent>();
				transform_component->set_world_transform(trs{ vec3(0.0f, 1.0f, 2.0f), quat::identity, vec3(1.0f) });

				mesh->load_job.wait_for_completion();

				SkeletonComponent* skeleton_component = character.add_component<SkeletonComponent>();

				ref<Skeleton> skeleton = provider.load_game_asset_from_file<Skeleton>(get_asset_path("/resources/assets/test_character/skeletons/Character_skeleton0.eskeleton"));
				skeleton->load_job.wait_for_completion();
				skeleton_component->skeleton = skeleton;
				//skeleton_component->draw_sceleton = true;
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
				cct_component->radius = 0.3f;
				cct_component->step_offset = 0.05f;
				cct_component->handle_non_cct_contacts = false;

				character.add_component<MotionComponent>();
				TrajectoryComponent* trajectory_component = character.add_component<TrajectoryComponent>();
				trajectory_component->time_offsets(0) = 0.0f;
				trajectory_component->time_offsets(1) = 0.12f;
				trajectory_component->time_offsets(2) = 0.32f;
				trajectory_component->time_offsets(3) = 0.54f;
				trajectory_component->time_offsets(4) = 0.76f;

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

				if (false)
				{
					database = provider.load_game_asset_from_file<MotionMatchingDatabase>(get_asset_path("/resources/assets/test_character/mmdb/locomotion.emmdb"));
					database->load_job.wait_for_completion();
				}

				if (true)
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

					auto add_animation = [&](std::string_view path)
						{
							ref<AnimationAssetClip> animation = provider.load_game_asset_from_file<AnimationAssetClip>(get_asset_path(path));
							animation->load_job.wait_for_completion();
							database->animations.push_back(animation);
						};

					database->sample_rate = 30.0f;
					database->narrow_phase_params.flags = NarrowPhaseFlags::SAME_FRAME_CHECK;
					database->narrow_phase_params.same_frame_time_threshold = 0.16f;

					add_animation("/resources/assets/test_character/animations/Standing Idle_mixamo.com_clip0.eanm");
					add_animation("/resources/assets/test_character/animations/Unarmed Idle_mixamo.com_clip0.eanm");
					add_animation("/resources/assets/test_character/animations/Catwalk Sequence 02L_mixamo.com_clip0.eanm");
					add_animation("/resources/assets/test_character/animations/Catwalk Sequence 02R_mixamo.com_clip0.eanm");
					add_animation("/resources/assets/test_character/animations/Catwalk Walk Forward Arc 90L_mixamo.com_clip0.eanm");
					add_animation("/resources/assets/test_character/animations/Catwalk Walk Forward Arc 90R_mixamo.com_clip0.eanm");
					add_animation("/resources/assets/test_character/animations/Catwalk Walk Start Turn 180 Left_mixamo.com_clip0.eanm");
					add_animation("/resources/assets/test_character/animations/Catwalk Walk Start Turn 180 Right_mixamo.com_clip0.eanm");
					add_animation("/resources/assets/test_character/animations/Change DirectionL_mixamo.com_clip0.eanm");
					add_animation("/resources/assets/test_character/animations/Change DirectionR_mixamo.com_clip0.eanm");
					add_animation("/resources/assets/test_character/animations/Jog Backward DiagonalL_mixamo.com_clip0.eanm");
					add_animation("/resources/assets/test_character/animations/Jog Backward DiagonalR_mixamo.com_clip0.eanm");
					add_animation("/resources/assets/test_character/animations/Run Forward Arc Left_mixamo.com_clip0.eanm");
					add_animation("/resources/assets/test_character/animations/Run Forward Arc Right_mixamo.com_clip0.eanm");
					add_animation("/resources/assets/test_character/animations/Run To Stop_mixamo.com_clip0.eanm");
					add_animation("/resources/assets/test_character/animations/Running Left Turn_mixamo.com_clip0.eanm");
					add_animation("/resources/assets/test_character/animations/Running Right Turn_mixamo.com_clip0.eanm");
					add_animation("/resources/assets/test_character/animations/Running Turn 180L_mixamo.com_clip0.eanm");
					add_animation("/resources/assets/test_character/animations/Running Turn 180R_mixamo.com_clip0.eanm");
					add_animation("/resources/assets/test_character/animations/Running_mixamo.com_clip0.eanm");
					add_animation("/resources/assets/test_character/animations/Sprint_mixamo.com_clip0.eanm");
					//add_animation("/resources/assets/test_character/animations/Start Walking_mixamo.com_clip0.eanm");
					//add_animation("/resources/assets/test_character/animations/Stop Walking_mixamo.com_clip0.eanm");
					add_animation("/resources/assets/test_character/animations/Turn To RunningL_mixamo.com_clip0.eanm");
					add_animation("/resources/assets/test_character/animations/Turn To RunningR_mixamo.com_clip0.eanm");
					add_animation("/resources/assets/test_character/animations/Walking Backwards_mixamo.com_clip0.eanm");
					add_animation("/resources/assets/test_character/animations/Walking Left Turn_mixamo.com_clip0.eanm");
					add_animation("/resources/assets/test_character/animations/Walking Right Turn_mixamo.com_clip0.eanm");
					add_animation("/resources/assets/test_character/animations/Walking_mixamo.com_clip0.eanm");
					add_animation("/resources/assets/test_character/animations/Backward Right Turn_mixamo.com_clip0.eanm");
					add_animation("/resources/assets/test_character/animations/Backward Left Turn_mixamo.com_clip0.eanm");
					add_animation("/resources/assets/test_character/animations/Jog Strafe Left_mixamo.com_clip0.eanm");
					add_animation("/resources/assets/test_character/animations/Jog Strafe Right_mixamo.com_clip0.eanm");
					add_animation("/resources/assets/test_character/animations/Running To Turn_mixamo.com_clip0.eanm");
					add_animation("/resources/assets/test_character/animations/Run Backward Arc Left_mixamo.com_clip0.eanm");
					add_animation("/resources/assets/test_character/animations/Run Backward Arc Right_mixamo.com_clip0.eanm");
					add_animation("/resources/assets/test_character/animations/Running Backward_mixamo.com_clip0.eanm");
					add_animation("/resources/assets/test_character/animations/StrafeL_mixamo.com_clip0.eanm");
					add_animation("/resources/assets/test_character/animations/StrafeR_mixamo.com_clip0.eanm");
					add_animation("/resources/assets/test_character/animations/Run Backward_mixamo.com_clip0.eanm");
					add_animation("/resources/assets/test_character/animations/Slow Jog Backwards_mixamo.com_clip0.eanm");

					database->generate_initial_data();

					database->search_dimension = database->total_features_per_sample;
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

					database->weights[start_from + 1] = 6.0f;
					database->weights[start_from + 2] = 6.0f;

					database->weights[start_from + 3] = 8.0f;
					database->weights[start_from + 4] = 8.0f;
					database->weights[start_from + 5] = 4.0f;
					database->weights[start_from + 6] = 4.0f;

					database->weights[start_from + 7] = 65.0f;
					database->weights[start_from + 8] = 65.0f;
					database->weights[start_from + 9] = 25.0f;
					database->weights[start_from + 10] = 25.0f;

					database->weights[start_from + 11] = 80.0f;
					database->weights[start_from + 12] = 80.0f;
					database->weights[start_from + 13] = 60.0f;
					database->weights[start_from + 14] = 60.0f;
					database->weights[start_from + 9] = 30.0f;
					database->weights[start_from + 10] = 30.0f;

					database->weights[start_from + 11] = 150.0f;
					database->weights[start_from + 12] = 150.0f;
					database->weights[start_from + 13] = 80.0f;
					database->weights[start_from + 14] = 80.0f;

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

				settings.scaler_settings.upper_body_height_modifier = 0.55f;
				settings.scaler_settings.upper_body_radius_modifier = 0.5f;
				settings.scaler_settings.middle_body_height_modifier = 0.75f;
				settings.scaler_settings.middle_body_radius_modifier = 0.75f;
				//settings.scaler_settings.lower_body_height_modifier = 1.0f;
				settings.scaler_settings.lower_body_radius_modifier = 1.0f;
				settings.scaler_settings.clavicle_height_modifier = 1.0f;
				settings.scaler_settings.arm_height_modifier = 1.6f;
				settings.scaler_settings.forearm_height_modifier = 1.3f;

				//RagdollComponent* ragdoll_component = character.add_component<RagdollComponent>();
				//ragdoll_component->simulated = true;
				PhysicalAnimationComponent* ragdoll_component = character.add_component<PhysicalAnimationComponent>();
				ragdoll_component->joint_init_ids = joint_init_ids;
				ragdoll_component->settings = settings;

				//character.add_component<RagdollDismembermentComponent>();
			}
		}

		//{
		//	Entity cloth = world->create_entity("Cloth");

		//	TransformComponent* transform_component = cloth.get_component<TransformComponent>();
		//	transform_component->set_world_transform(trs{ vec3(0.0f, 20.0f, 10.0f), quat::identity, vec3(1.0f) });

		//	physics::PBDClothComponent* cloth_component = cloth.add_component<physics::PBDClothComponent>();
		//	cloth_component->num_x = 30;
		//	cloth_component->num_z = 30;
		//}

		/*if (auto mesh = import_mesh_from_file_async(get_asset_path("/resources/assets/Sponza/sponza.obj"), mesh_creation_flags_unreal_asset))
		{
			auto sponza = world->create_entity("Sponza");
			sponza.add_component<MeshComponent>(mesh);

			TransformComponent* transform_component = sponza.get_component<TransformComponent>();
			transform_component->set_world_position(vec3(5.0f, -3.75f, 35.0f));
		}*/

		//if (auto mesh = import_mesh_from_file_async(get_asset_path("/resources/assets/box.fbx"), mesh_creation_flags_default))
		{
			/*ref<MultiMesh> mesh = provider.load_game_asset_from_file<MultiMesh>(get_asset_path("/resources/assets/box.emesh"), true, {}, mesh_creation_flags_default);
			mesh->load_job.wait_for_completion();

			Entity box = world->create_entity("Box");
			box.add_component<MeshComponent>(mesh)->is_hidden = true;

			TransformComponent* transform_component = box.get_component<TransformComponent>();
			transform_component->set_world_position(vec3(5.0f, -4.0f, -5.0f));

			DestructibleComponent* descructible_component = box.add_component<DestructibleComponent>(DestructibleComponent::Type::FRACTURE_BASED);
			descructible_component->fracture_desc.chunks_count = 10;
			descructible_component->fracture_desc.density = 100.0f;
			descructible_component->fracture_desc.break_force = 200.0f;
			descructible_component->material = PhysicsEngine::get_physics_core()->create_material(0.1f, 0.8f, 0.7f);*/
		}

		//{
		//	Entity vehicle = world->create_entity("Vehicle");
		//	vehicle.get_component<TransformComponent>()->set_world_position(vec3(0.0f, 5.0f, 0.0f));
		//	vehicle.add_component<physics::W4VehicleComponent>();
		//	camera_entity.get_component<InputSenderComponent>()->add_reciever(vehicle.add_component<InputReceiverComponent>());
		//}

		Entity plane = world->create_entity("Platform");
		plane.add_component<PlanePhysicsComponent>(CollisionType::TERRAIN, vec3(0.0f, -5.0f, 0.0f));
		plane.add_component<MeshComponent>(ground_mesh);
		plane.get_component<TransformComponent>()->set_world_transform(trs{vec3(10.0f, -9.0f, 0.0f), quat(vec3(1.0f, 0.0f, 0.0f), deg2rad(0.0f)), vec3(5.0f, 1.0f, 5.0f)});
		
		ground_mesh->load_state = AssetLoadState::LOADED;
		ground_mesh->mesh = builder.createDXMesh();

		{
			/*uint32 num_terrain_chunks = 4;
			float terrain_chunk_size = 10.f;

			PbrMaterialDesc terrain_ground_desc;
			terrain_ground_desc.albedo = get_asset_path("/resources/assets/terrain/grass.bmp");
			terrain_ground_desc.normal = get_asset_path("/resources/assets/terrain/grass_normal.bmp");
			terrain_ground_desc.albedo_flags &= ~image_load_flags_compress;

			PbrMaterialDesc terrain_rock_desc;
			terrain_rock_desc.albedo = get_asset_path("/resources/assets/terrain/rock.bmp");
			terrain_rock_desc.normal = get_asset_path("/resources/assets/terrain/rock_normal.bmp");

			PbrMaterialDesc terrain_mud_desc;

			auto terrain_ground_material = create_pbr_material(terrain_ground_desc);
			auto terrain_rock_material = create_pbr_material(terrain_rock_desc);
			auto terrain_mud_material = create_pbr_material(terrain_mud_desc);

			Entity terrain = world->create_entity("Terrain");

			terrain.get_component<TransformComponent>()->set_world_position(vec3(0.0f, 25.0f, 0.0f));

			terrain.add_component<TerrainComponent>(num_terrain_chunks, terrain_chunk_size, 10.f, terrain_ground_material, terrain_rock_material, terrain_mud_material)->update();*/
			//terrain.add_component<TerrainPhysicsComponent>(CollisionType::TERRAIN, vec3(0.0f, 25.0, 0.0f));
			//terrain.add_component<GrassComponent>(GrassSettings{ .bladeHeight = 0.15f, .bladeWidth = 0.0075f,.numGrassBladesPerChunkDim = 150 });

			//std::vector<proc_placement_layer_desc> layers =
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
			//};
			//.addComponent<heightmap_collider_component>(numTerrainChunks, terrainChunkSize, physics_material{ physics_material_type_metal, 0.1f, 1.f, 4.f })
			//.addComponent<proc_placement_component>(layers) // TODO: This could be deferred if we want to load the meshes asynchronously.
		}
	}

	void GameInitSystem::update(float dt)
	{
		using namespace physics;

		const InputReceiverComponent* input_receiver = camera_entity.get_component<InputReceiverComponent>();

		const UserInput& frame_input = input_receiver->get_frame_input();
		if (frame_input.keyboard[key_code::key_space].press_event)
		{
			Entity sphere = world->create_entity("Sphere");

			const ref<PhysicsMaterial>& material = PhysicsEngine::get_physics_core()->get_default_material();

			SphereShapeComponent* sphere_shape_component = sphere.add_component<SphereShapeComponent>();
			sphere_shape_component->collision_type = static_cast<CollisionType>(GameCollisionType::DYNAMICS);
			sphere_shape_component->radius = 0.25f;
			sphere_shape_component->material = material;
			sphere.add_component<MeshComponent>(sphere_mesh);

			CameraHolderComponent* camera_holder_component = camera_entity.get_component<CameraHolderComponent>();

			trs camera_world_transform = trs{ camera_holder_component->get_render_camera()->position, camera_holder_component->get_render_camera()->rotation, vec3(0.2f)};
			sphere.get_component<TransformComponent>()->set_world_transform(camera_world_transform);

			DynamicBodyComponent* dynamic_body_component = sphere.add_component<DynamicBodyComponent>();
			dynamic_body_component->ccd.get_for_write() = true;
			dynamic_body_component->mass.get_for_write() = 50.0f;
			dynamic_body_component->simulated.get_for_write() = true;
			dynamic_body_component->use_gravity.get_for_write() = true;
			dynamic_body_component->sleep_threshold.get_for_write() = 0.05f;

			Force& force = dynamic_body_component->forces.emplace_back();
			force.force = (camera_holder_component->get_render_camera()->rotation * quat(vec3(0.0f, 1.0f, 0.0f), M_PI)) * vec3(0.0f, 0.0f, 1.0f) * sphere_speed;
			force.mode = ForceMode::FORCE;
		}
	}
}