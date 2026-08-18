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
#include "physics/destructions/destructible_component.h"

#include <motion_matching/trajectory/trajectory_component.h>
#include <motion_matching/motion/motion_component.h>
#include <motion_matching/features/pose_feature.h>
#include <motion_matching/features/trajectory_feature.h>
#include <motion_matching/motion_matching_database.h>

#include <audio/audio.h>

#include <terrain/terrain.h>

#include <animation/skinning.h>

#include <rttr/policy.h>
#include <rttr/registration>
#include <motion_matching/features/motion_matching_feature_set.h>

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

		camera_entity = world->create_entity("CameraEntity");
		CameraHolderComponent* camera_holder_component = camera_entity.add_component<CameraHolderComponent>();
		camera_holder_component->set_camera_type(CameraHolderComponent::FREE_CAMERA);
		camera_holder_component->set_render_camera(&renderer_holder_rc->camera);
		camera_entity.add_component<InputSenderComponent>()->add_reciever(camera_entity.add_component<InputReceiverComponent>());

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
		
		//if (ref<MultiMesh> mesh = import_animated_mesh_from_file_async(get_asset_path("/resources/assets/springtrap/source/Springtrap.fbx"),
		//	mesh_creation_flags_unreal_animated_asset))
		{
			ref<MultiMesh> mesh = provider.load_game_asset_from_file<MultiMesh>(get_asset_path("/resources/assets/springtrap/source/Springtrap"), true, {}, mesh_creation_flags_animated | mesh_creation_flags_compact);

			tiran = world->create_entity("Tiran");

			tiran.add_component<MeshComponent>(mesh);

			TransformComponent* transform_component = tiran.get_component<TransformComponent>();
			transform_component->set_world_transform(trs{vec3(-5.0f, -4.7f, 5.0f), quat::identity, vec3(1.0f)});

			mesh->load_job.wait_for_completion();

			SkeletonComponent* skeleton_component = tiran.add_component<SkeletonComponent>();

			AnimationComponent* animation_component = tiran.add_component<AnimationComponent>();
			animation_component->play = true;
			animation_component->loop = true;

			{
				ref<Skeleton> tiran_skeleton = provider.load_game_asset_from_file<Skeleton>(get_asset_path("/resources/assets/springtrap/source/skeletons/skeleton0"));
				tiran_skeleton->load_job.wait_for_completion();
				skeleton_component->skeleton = tiran_skeleton;
				skeleton_component->draw_sceleton = true;
				skeleton_component->apply_pose(tiran_skeleton->get_default_pose());
			}

			{
				ref<AnimationAssetClip> anim_clip = provider.load_game_asset_from_file<AnimationAssetClip>(get_asset_path("/resources/assets/springtrap/source/animations/animation_clip74"));
				anim_clip->load_job.wait_for_completion();
				animation_component->current_animation = anim_clip;
				animation_component->current_anim_position = 0.0f;
			}

			//animation_component->activate_inertial_blend();

			const ref<Skeleton> skeleton = skeleton_component->skeleton;

			RagdollJointIds joint_init_ids;
			joint_init_ids.head_end_idx = skeleton->name_to_joint_id.at("joint_Head_01");
			joint_init_ids.head_idx = skeleton->name_to_joint_id.at("joint_Head_01");
			joint_init_ids.neck_idx = skeleton->name_to_joint_id.at("joint_NeckA_01");

			joint_init_ids.spine_03_idx = skeleton->name_to_joint_id.at("joint_TorsoC_01");
			joint_init_ids.spine_02_idx = skeleton->name_to_joint_id.at("joint_TorsoB_01");
			joint_init_ids.spine_01_idx = skeleton->name_to_joint_id.at("joint_TorsoA_01");
			joint_init_ids.pelvis_idx = skeleton->name_to_joint_id.at("joint_Pelvis_01");

			joint_init_ids.clavicle_l_idx = skeleton->name_to_joint_id.at("joint_ClavicleLT_01");
			joint_init_ids.clavicle_r_idx = skeleton->name_to_joint_id.at("joint_ClavicleRT_01");

			joint_init_ids.root_idx = skeleton->name_to_joint_id.at("joint_Char");
			joint_init_ids.attachment_idx = skeleton->name_to_joint_id.at("joint_Pelvis_01");

			joint_init_ids.thigh_l_idx = skeleton->name_to_joint_id.at("joint_HipLT_01");
			joint_init_ids.calf_l_idx = skeleton->name_to_joint_id.at("joint_KneeLT_01");
			joint_init_ids.foot_l_idx = skeleton->name_to_joint_id.at("joint_FootLT_01");
			joint_init_ids.foot_end_l_idx = skeleton->name_to_joint_id.at("joint_ToeLT_01");

			joint_init_ids.thigh_r_idx = skeleton->name_to_joint_id.at("joint_HipRT_01");
			joint_init_ids.calf_r_idx = skeleton->name_to_joint_id.at("joint_KneeRT_01");
			joint_init_ids.foot_r_idx = skeleton->name_to_joint_id.at("joint_FootRT_01");
			joint_init_ids.foot_end_r_idx = skeleton->name_to_joint_id.at("joint_ToeRT_01");

			joint_init_ids.upperarm_l_idx = skeleton->name_to_joint_id.at("joint_ShoulderLT_01");
			joint_init_ids.lowerarm_l_idx = skeleton->name_to_joint_id.at("joint_ElbowLT_01");
			joint_init_ids.hand_l_idx = skeleton->name_to_joint_id.at("joint_HandLT_01");
			joint_init_ids.hand_end_l_idx = skeleton->name_to_joint_id.at("joint_FingerBLT_01");

			joint_init_ids.upperarm_r_idx = skeleton->name_to_joint_id.at("joint_ShoulderRT_01");
			joint_init_ids.lowerarm_r_idx = skeleton->name_to_joint_id.at("joint_ElbowRT_01");
			joint_init_ids.hand_r_idx = skeleton->name_to_joint_id.at("joint_HandRT_01");
			joint_init_ids.hand_end_r_idx = skeleton->name_to_joint_id.at("joint_FingerBRT_01");

			RagdollSettings settings;
			settings.shapes_settings.head_radius = 0.13f;
			settings.shapes_settings.clavicle_radius = 0.071f;
			settings.shapes_settings.arm_radius = 0.065f;
			settings.shapes_settings.foot_radius = 0.05f;
			settings.shapes_settings.hand_radius = 0.05f;

			settings.local_shape_settings.head_joint_adjastment = vec3(0.0f, 0.05f, 0.0f);
			//settings.local_shape_settings.neck_joint_adjastment = vec3(0.0f, 0.0f, 0.0f);
			settings.local_shape_settings.thorax_joint_adjastment = vec3(0.0f, 0.1f, 0.0f);
			//settings.local_shape_settings.abdomen_joint_adjastment = vec3(0.0f, 0.05f, 0.0f);
			settings.local_shape_settings.pelvis_joint_adjastment = vec3(0.0f, -0.05f, 0.0f);
			settings.local_shape_settings.left_clavicle_joint_adjastment = vec3(0.0f, 0.06f, 0.0f);
			settings.local_shape_settings.left_arm_joint_adjastment = vec3(0.05f, 0.06f, 0.0f);
			settings.local_shape_settings.left_arm_joint_spin = quat(vec3(0.0f, 1.0f, 0.0f), deg2rad(-10.0f));
			settings.local_shape_settings.left_forearm_joint_adjastment = vec3(0.05f, 0.0f, 0.0f);
			settings.local_shape_settings.right_clavicle_joint_adjastment = vec3(0.0f, 0.06f, 0.0f);
			settings.local_shape_settings.right_arm_joint_adjastment = vec3(-0.05f, 0.06f, 0.0f);
			settings.local_shape_settings.right_arm_joint_spin = quat(vec3(0.0f, 1.0f, 0.0f), deg2rad(10.0f));
			settings.local_shape_settings.right_forearm_joint_adjastment = vec3(-0.05f, 0.0f, 0.0f);

			settings.object_space_settings.head_end_joint_adjastment = vec3(0.0f, 0.35f, 0.0f);
			settings.object_space_settings.head_joint_adjastment = vec3(0.0f, 0.05f, 0.0f);
			settings.object_space_settings.neck_joint_adjastment = vec3(0.0f, 0.0f, 0.0f);
			settings.object_space_settings.thorax_joint_adjastment = vec3(0.0f, 0.25f, 0.0f);
			settings.object_space_settings.abdomen_joint_adjastment = vec3(0.0f, 0.1f, 0.0f);
			settings.object_space_settings.pelvis_joint_adjastment = vec3(0.0f, -0.1f, 0.0f);
			settings.object_space_settings.left_clavicle_joint_adjastment = vec3(-0.075f, 0.1f, 0.0f);
			settings.object_space_settings.left_arm_joint_adjastment = vec3(0.0f, 0.075f, 0.0f);
			settings.object_space_settings.right_clavicle_joint_adjastment = vec3(0.075f, 0.1f, 0.0f);
			settings.object_space_settings.right_arm_joint_adjastment = vec3(0.0f, 0.075f, 0.0f);
			settings.object_space_settings.left_foot_end_joint_adjastment = vec3(0.0f, 0.0f, 0.04f);
			settings.object_space_settings.right_foot_end_joint_adjastment = vec3(0.0f, 0.0f, 0.04f);

			settings.scaler_settings.upper_body_height_modifier = 0.32f;
			settings.scaler_settings.upper_body_radius_modifier = 0.8f;
			settings.scaler_settings.middle_body_height_modifier = 0.45f;
			settings.scaler_settings.middle_body_radius_modifier = 0.45f;
			settings.scaler_settings.lower_body_height_modifier = 1.0f;
			settings.scaler_settings.lower_body_radius_modifier = 1.1f;
			settings.scaler_settings.clavicle_height_modifier = 1.2f;
			settings.scaler_settings.arm_height_modifier = 1.0f;
			settings.scaler_settings.forearm_height_modifier = 1.3f;

			CharacterControllerComponent* cct_component = tiran.add_component<CharacterControllerComponent>();
			cct_component->collision_type = static_cast<CollisionType>(GameCollisionType::CCT);
			cct_component->height = 1.2f;
			cct_component->radius = 0.3f;
			cct_component->step_offset = 0.05f;

			tiran.add_component<MotionComponent>();
			TrajectoryComponent* trajectory_component = tiran.add_component<TrajectoryComponent>();
			trajectory_component->time_offsets(0) = 0.0f;
			trajectory_component->time_offsets(1) = 0.06f;
			trajectory_component->time_offsets(2) = 0.12f;
			trajectory_component->time_offsets(3) = 0.18f;
			camera_entity.get_component<InputSenderComponent>()->add_reciever(tiran.add_component<InputReceiverComponent>());

			//RagdollComponent* ragdoll_component = tiran.add_component<RagdollComponent>();
			//ragdoll_component->simulated = true;
			PhysicalAnimationComponent* ragdoll_component = tiran.add_component<PhysicalAnimationComponent>();
			ragdoll_component->joint_init_ids = joint_init_ids;
			ragdoll_component->settings = settings;

			//tiran.add_component<RagdollDismembermentComponent>();

			{
				//database = provider.load_game_asset_from_file<MotionMatchingDatabase>(get_asset_path("/resources/assets/springtrap/source/mmdb/locomotion"));
				//database->load_job.wait_for_completion();
			}

			//{
			//	database = make_ref<MotionMatchingDatabase>();

			//	database->database_id = "LOCOMOTION";
			//	database->knn_type = KnnStructureType::HNSW;

			//	{
			//		PoseFeature* pose_feature = new PoseFeature();
			//		{
			//			ref<PoseFeatureDesc> desc = make_ref<PoseFeatureDesc>();
			//			desc->basis = FeatureDescBasis::XYZ;
			//			desc->type = FeatureDescType::LOCATION;
			//			desc->joint_id = joint_init_ids.pelvis_idx;
			//			desc->name = "pelvis_location";
			//			pose_feature->descriptors.push_back(desc);
			//		}

			//		{
			//			ref<PoseFeatureDesc> desc = make_ref<PoseFeatureDesc>();
			//			desc->basis = FeatureDescBasis::XYZ;
			//			desc->type = FeatureDescType::LOCATION;
			//			desc->joint_id = joint_init_ids.pelvis_idx;
			//			desc->name = "pelvis_velocity";
			//			pose_feature->descriptors.push_back(desc);
			//		}

			//		{
			//			ref<PoseFeatureDesc> desc = make_ref<PoseFeatureDesc>();
			//			desc->basis = FeatureDescBasis::Y;
			//			desc->type = FeatureDescType::LOCATION;
			//			desc->joint_id = joint_init_ids.neck_idx;
			//			desc->name = "neck_location";
			//			pose_feature->descriptors.push_back(desc);
			//		}

			//		{
			//			ref<PoseFeatureDesc> desc = make_ref<PoseFeatureDesc>();
			//			desc->basis = FeatureDescBasis::XYZ;
			//			desc->type = FeatureDescType::LOCATION;
			//			desc->joint_id = joint_init_ids.foot_l_idx;
			//			desc->name = "foot_l_location";
			//			pose_feature->descriptors.push_back(desc);
			//		}

			//		{
			//			ref<PoseFeatureDesc> desc = make_ref<PoseFeatureDesc>();
			//			desc->basis = FeatureDescBasis::XYZ;
			//			desc->type = FeatureDescType::VELOCITY;
			//			desc->joint_id = joint_init_ids.foot_l_idx;
			//			desc->name = "foot_l_velocity";
			//			pose_feature->descriptors.push_back(desc);
			//		}

			//		{
			//			ref<PoseFeatureDesc> desc = make_ref<PoseFeatureDesc>();
			//			desc->basis = FeatureDescBasis::XYZ;
			//			desc->type = FeatureDescType::LOCATION;
			//			desc->joint_id = joint_init_ids.foot_r_idx;
			//			desc->name = "foot_r_location";
			//			pose_feature->descriptors.push_back(desc);
			//		}

			//		{
			//			ref<PoseFeatureDesc> desc = make_ref<PoseFeatureDesc>();
			//			desc->basis = FeatureDescBasis::XYZ;
			//			desc->type = FeatureDescType::VELOCITY;
			//			desc->joint_id = joint_init_ids.foot_r_idx;
			//			desc->name = "foot_r_velocity";
			//			pose_feature->descriptors.push_back(desc);
			//		}

			//		{
			//			ref<PoseFeatureDesc> desc = make_ref<PoseFeatureDesc>();
			//			desc->basis = FeatureDescBasis::XYZ;
			//			desc->type = FeatureDescType::LOCATION;
			//			desc->joint_id = joint_init_ids.hand_l_idx;
			//			desc->name = "hand_l_location";
			//			pose_feature->descriptors.push_back(desc);
			//		}

			//		{
			//			ref<PoseFeatureDesc> desc = make_ref<PoseFeatureDesc>();
			//			desc->basis = FeatureDescBasis::XYZ;
			//			desc->type = FeatureDescType::VELOCITY;
			//			desc->joint_id = joint_init_ids.hand_l_idx;
			//			desc->name = "hand_l_velocity";
			//			pose_feature->descriptors.push_back(desc);
			//		}

			//		{
			//			ref<PoseFeatureDesc> desc = make_ref<PoseFeatureDesc>();
			//			desc->basis = FeatureDescBasis::XYZ;
			//			desc->type = FeatureDescType::LOCATION;
			//			desc->joint_id = joint_init_ids.hand_r_idx;
			//			desc->name = "hand_r_location";
			//			pose_feature->descriptors.push_back(desc);
			//		}

			//		{
			//			ref<PoseFeatureDesc> desc = make_ref<PoseFeatureDesc>();
			//			desc->basis = FeatureDescBasis::XYZ;
			//			desc->type = FeatureDescType::LOCATION;
			//			desc->joint_id = joint_init_ids.hand_r_idx;
			//			desc->name = "hand_r_velocity";
			//			pose_feature->descriptors.push_back(desc);
			//		}

			//		//{
			//		//	ref<PoseFeatureDesc> desc = make_ref<PoseFeatureDesc>();
			//		//	desc->basis = FeatureDescBasis::XYZ;
			//		//	desc->type = FeatureDescType::LOCATION;
			//		//	desc->joint_id = joint_init_ids.calf_l_idx;
			//		//	desc->name = "calf_l_location";
			//		//	pose_feature->descriptors.push_back(desc);
			//		//}

			//		//{
			//		//	ref<PoseFeatureDesc> desc = make_ref<PoseFeatureDesc>();
			//		//	desc->basis = FeatureDescBasis::XYZ;
			//		//	desc->type = FeatureDescType::LOCATION;
			//		//	desc->joint_id = joint_init_ids.calf_r_idx;
			//		//	desc->name = "calf_r_location";
			//		//	pose_feature->descriptors.push_back(desc);
			//		//}

			//		database->features.push_back(pose_feature);
			//	}

			//	{
			//		TrajectoryFeature* trajectory_feature = new TrajectoryFeature();

			//		{
			//			ref<TrajectoryFeatureDesc> desc = make_ref<TrajectoryFeatureDesc>();
			//			desc->type = FeatureDescType::LOCATION;
			//			desc->time_offset = 0.0f;
			//			desc->name = "curr_location";
			//			trajectory_feature->descriptors.push_back(desc);
			//		}

			//		{
			//			ref<TrajectoryFeatureDesc> desc = make_ref<TrajectoryFeatureDesc>();
			//			desc->type = FeatureDescType::DIRECTION;
			//			desc->time_offset = 0.0f;
			//			desc->name = "curr_direction";
			//			trajectory_feature->descriptors.push_back(desc);
			//		}

			//		{
			//			ref<TrajectoryFeatureDesc> desc = make_ref<TrajectoryFeatureDesc>();
			//			desc->type = FeatureDescType::LOCATION;
			//			desc->time_offset = 0.06f;
			//			desc->name = "next_1_location";
			//			trajectory_feature->descriptors.push_back(desc);
			//		}

			//		{
			//			ref<TrajectoryFeatureDesc> desc = make_ref<TrajectoryFeatureDesc>();
			//			desc->type = FeatureDescType::DIRECTION;
			//			desc->time_offset = 0.06f;
			//			desc->name = "next_1_direction";
			//			trajectory_feature->descriptors.push_back(desc);
			//		}

			//		{
			//			ref<TrajectoryFeatureDesc> desc = make_ref<TrajectoryFeatureDesc>();
			//			desc->type = FeatureDescType::LOCATION;
			//			desc->time_offset = 0.12f;
			//			desc->name = "next_2_location";
			//			trajectory_feature->descriptors.push_back(desc);
			//		}

			//		{
			//			ref<TrajectoryFeatureDesc> desc = make_ref<TrajectoryFeatureDesc>();
			//			desc->type = FeatureDescType::DIRECTION;
			//			desc->time_offset = 0.12f;
			//			desc->name = "next_2_direction";
			//			trajectory_feature->descriptors.push_back(desc);
			//		}

			//		{
			//			ref<TrajectoryFeatureDesc> desc = make_ref<TrajectoryFeatureDesc>();
			//			desc->type = FeatureDescType::LOCATION;
			//			desc->time_offset = 0.18f;
			//			desc->name = "next_3_location";
			//			trajectory_feature->descriptors.push_back(desc);
			//		}

			//		{
			//			ref<TrajectoryFeatureDesc> desc = make_ref<TrajectoryFeatureDesc>();
			//			desc->type = FeatureDescType::DIRECTION;
			//			desc->time_offset = 0.18f;
			//			desc->name = "next_3_direction";
			//			trajectory_feature->descriptors.push_back(desc);
			//		}

			//		database->features.push_back(trajectory_feature);
			//	}

			//	auto add_animation = [&](std::string_view path)
			//		{
			//			ref<AnimationAssetClip> animation = provider.load_game_asset_from_file<AnimationAssetClip>(get_asset_path(path));
			//			animation->load_job.wait_for_completion();
			//			database->animations.push_back(animation);
			//		};

			//	database->sample_rate = 30.0f;
			//	database->narrow_phase_params.flags = NarrowPhaseFlags::SAME_FRAME_CHECK | NarrowPhaseFlags::EUCLIDIAN_DISTANCE_CHECK;

			//	add_animation("/resources/assets/springtrap/source/animations/animation_clip50");
			//	add_animation("/resources/assets/springtrap/source/animations/animation_clip71");
			//	add_animation("/resources/assets/springtrap/source/animations/animation_clip72");
			//	add_animation("/resources/assets/springtrap/source/animations/animation_clip73");
			//	add_animation("/resources/assets/springtrap/source/animations/animation_clip74");
			//	add_animation("/resources/assets/springtrap/source/animations/animation_clip75");
			//	add_animation("/resources/assets/springtrap/source/animations/animation_clip77");
			//	add_animation("/resources/assets/springtrap/source/animations/animation_clip78");
			//	add_animation("/resources/assets/springtrap/source/animations/animation_clip79");
			//	add_animation("/resources/assets/springtrap/source/animations/animation_clip80");
			//	add_animation("/resources/assets/springtrap/source/animations/animation_clip87");
			//	add_animation("/resources/assets/springtrap/source/animations/animation_clip88");
			//	add_animation("/resources/assets/springtrap/source/animations/animation_clip89");
			//	add_animation("/resources/assets/springtrap/source/animations/animation_clip90");
			//	add_animation("/resources/assets/springtrap/source/animations/animation_clip91");
			//	add_animation("/resources/assets/springtrap/source/animations/animation_clip92");
			//	add_animation("/resources/assets/springtrap/source/animations/animation_clip93");
			//	add_animation("/resources/assets/springtrap/source/animations/animation_clip94");
			//	add_animation("/resources/assets/springtrap/source/animations/animation_clip95");
			//	add_animation("/resources/assets/springtrap/source/animations/animation_clip96");

			//	database->generate_initial_data();

			//	database->search_dimension = database->total_features_per_sample;
			//	database->static_thresholds.resize(database->total_features_per_sample);
			//	database->weights.resize(database->total_features_per_sample, 1.0f);
			//	database->mark(skeleton_component);

			//	database->bake(skeleton.get());

			//	JobHandle save_job = provider.save_game_asset_to_file_async<MotionMatchingDatabase>(get_asset_path("/resources/assets/springtrap/source/mmdb/locomotion"), database.get());
			//	save_job.wait_for_completion();
			//}
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
			ref<MultiMesh> mesh = provider.load_game_asset_from_file<MultiMesh>(get_asset_path("/resources/assets/box"), true, {}, mesh_creation_flags_default);
			mesh->load_job.wait_for_completion();

			Entity box = world->create_entity("Box");
			box.add_component<MeshComponent>(mesh)->is_hidden = true;

			TransformComponent* transform_component = box.get_component<TransformComponent>();
			transform_component->set_world_position(vec3(5.0f, -4.0f, -5.0f));

			DestructibleComponent* descructible_component = box.add_component<DestructibleComponent>(DestructibleComponent::Type::FRACTURE_BASED);
			descructible_component->fracture_desc.chunks_count = 10;
			descructible_component->fracture_desc.density = 100.0f;
			descructible_component->fracture_desc.break_force = 20.0f;
			descructible_component->material = PhysicsEngine::get_physics_core()->create_material(0.1f, 0.8f, 0.8f);
		}

		//{
		//	Entity vehicle = world->create_entity("Vehicle");
		//	vehicle.get_component<TransformComponent>()->set_world_position(vec3(0.0f, 5.0f, 0.0f));
		//	vehicle.add_component<physics::W4VehicleComponent>();
		//	camera_entity.get_component<InputSenderComponent>()->add_reciever(vehicle.add_component<InputReceiverComponent>());
		//}

		Entity plane = world->create_entity("Platform");
		plane.add_component<PlaneComponent>(CollisionType::TERRAIN, vec3(0.f, -5.0, 0.0f));
		plane.add_component<MeshComponent>(ground_mesh);
		plane.get_component<TransformComponent>()->set_world_transform(trs{vec3(10, -9.f, 0.f), quat(vec3(1.f, 0.f, 0.f), deg2rad(0.f)), vec3(5.0f, 1.0f, 5.0f)});
		
		ground_mesh->load_state = AssetLoadState::LOADED;
		ground_mesh->mesh = builder.createDXMesh();
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

		/*if (frame_input.keyboard['U'].press_event &&
			tiran.is_valid())
		{
			using namespace animation;

			FeatureComputationContext context;
			context.fill_context(tiran, world->get_fixed_update_dt());

			MotionMatchingFeatureSet feature_set;
			for (const MotionMatchingFeature* feature : database->features)
			{
				std::vector<float> feature_vector = feature->compute_features(context);
				feature_set.add_feature(feature->get_type(), std::move(feature_vector));
			}

			AnimationComponent* animation_component = tiran.get_component<AnimationComponent>();

			SearchParams search_params;
			search_params.current_anim_position = animation_component->current_anim_position;
			search_params.current_animation = animation_component->current_animation;
			search_params.query = feature_set.get_all_feature_values();
			search_params.current_features = search_params.query;

			SearchResult result = database->search(search_params);

			animation_component->current_animation = result.animation;
			animation_component->current_anim_position = result.anim_position;
		}*/
	}
}