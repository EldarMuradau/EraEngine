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

#include <rendering/mesh_shader.h>
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

#include <motion_matching/trajectory/trajectory_component.h>
#include <motion_matching/motion/motion_component.h>

#include <audio/audio.h>

#include <terrain/terrain.h>

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
			.method("update", &GameInitSystem::update)(metadata("update_group", update_types::GAMEPLAY_NORMAL_CONCURRENT));
	}

	static DebugVar<float> sphere_speed = DebugVar<float>("test.sphere_speed", 50000.0f);

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

		RendererHolderRootComponent* renderer_holder_rc = world->add_root_component<RendererHolderRootComponent>();
		ASSERT(renderer_holder_rc != nullptr);

		camera_entity = world->create_entity("CameraEntity");
		CameraHolderComponent* camera_holder_component = camera_entity.add_component<CameraHolderComponent>();
		camera_holder_component->set_camera_type(CameraHolderComponent::FREE_CAMERA);
		camera_holder_component->set_render_camera(&renderer_holder_rc->camera);
		camera_entity.add_component<InputSenderComponent>()->add_reciever(camera_entity.add_component<InputReceiverComponent>());

		PbrMaterialDesc defaultPlaneMatDesc;
		defaultPlaneMatDesc.albedo = get_asset_path("/resources/assets/uv.jpg");
		defaultPlaneMatDesc.normal = "";
		defaultPlaneMatDesc.roughness = "";
		defaultPlaneMatDesc.uv_scale = 15.0f;
		defaultPlaneMatDesc.metallic_override = 0.35f;
		defaultPlaneMatDesc.roughness_override = 0.01f;

		auto defaultPlaneMat = createPBRMaterialAsync(defaultPlaneMatDesc);

		mesh_builder builder;

		auto groundMesh = make_ref<multi_mesh>();
		builder.pushBox({ vec3(0.f), vec3(30.f, 4.f, 30.f) });
		groundMesh->submeshes.push_back({ builder.endSubmesh(), {}, trs::identity, defaultPlaneMat });

		Entity tiran;

		if (auto mesh = loadAnimatedMeshFromFileAsync(get_asset_path("/resources/assets/springtrap/source/Springtrap.fbx"), 
			mesh_creation_flags_unreal_animated_asset))
		{
			tiran = world->create_entity("Tiran");

			tiran.add_component<MeshComponent>(mesh);

			TransformComponent* transform_component = tiran.get_component<TransformComponent>();
			transform_component->set_world_transform(trs{vec3(-5.0f, -4.95f, 5.0f), quat::identity, vec3(1.0f)});

			mesh->loadJob.wait_for_completion();

			SkeletonComponent* skeleton_component = tiran.add_component<SkeletonComponent>();

			AnimationComponent* animation_component = tiran.add_component<AnimationComponent>();
			animation_component->play = true;
			animation_component->loop = true;

			GameAssetsProvider provider;

			{
				ref<Skeleton> tiran_skeleton = provider.load_game_asset_from_file<Skeleton>(get_asset_path("/resources/assets/springtrap/source/skeletons/skeleton0"));
				tiran_skeleton->load_job.wait_for_completion();
				skeleton_component->skeleton = tiran_skeleton;
				tiran_skeleton->apply_pose(tiran_skeleton->get_default_pose());
			}

			{
				ref<AnimationAssetClip> anim_clip = provider.load_game_asset_from_file<AnimationAssetClip>(get_asset_path("/resources/assets/springtrap/source/animations/animation_clip74"));
				anim_clip->load_job.wait_for_completion();
				animation_component->current_animation = anim_clip;
				animation_component->current_anim_position = 0.0f;
			}

			const ref<Skeleton> skeleton = skeleton_component->skeleton;

			RagdollJointIds joint_init_ids;
			joint_init_ids.head_end_idx = skeleton->name_to_joint_id.at("joint_HeadA_01");
			joint_init_ids.head_idx = skeleton->name_to_joint_id.at("joint_HeadA_01");
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
			settings.shapes_settings.arm_radius = 0.071f;

			settings.local_shape_settings.head_joint_adjastment = vec3(0.0f, 0.05f, 0.0f);
			//settings.local_shape_settings.neck_joint_adjastment = vec3(0.0f, 0.0f, 0.0f);
			settings.local_shape_settings.thorax_joint_adjastment = vec3(0.0f, 0.1f, 0.0f);
			//settings.local_shape_settings.abdomen_joint_adjastment = vec3(0.0f, 0.05f, 0.0f);
			settings.local_shape_settings.pelvis_joint_adjastment = vec3(0.0f, -0.05f, 0.0f);
			settings.local_shape_settings.left_clavicle_joint_adjastment = vec3(0.0f, 0.06f, 0.0f);
			settings.local_shape_settings.left_arm_joint_adjastment = vec3(0.05f, 0.06f, 0.0f);
			settings.local_shape_settings.left_forearm_joint_adjastment = vec3(0.03f, 0.0f, 0.0f);
			settings.local_shape_settings.right_clavicle_joint_adjastment = vec3(0.0f, 0.06f, 0.0f);
			settings.local_shape_settings.right_arm_joint_adjastment = vec3(-0.05f, 0.06f, 0.0f);
			settings.local_shape_settings.right_forearm_joint_adjastment = vec3(-0.03f, 0.0f, 0.0f);

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

			settings.shapes_settings.hand_x = 0.12f;
			settings.shapes_settings.hand_y = 0.08f;
			settings.shapes_settings.hand_z = 0.06f;

			settings.scaler_settings.upper_body_height_modifier = 0.32f;
			settings.scaler_settings.upper_body_radius_modifier = 0.8f;
			settings.scaler_settings.middle_body_height_modifier = 0.45f;
			settings.scaler_settings.middle_body_radius_modifier = 0.45f;
			settings.scaler_settings.lower_body_height_modifier = 1.0f;
			settings.scaler_settings.lower_body_radius_modifier = 1.0f;

			CharacterControllerComponent* cct_component = tiran.add_component<CharacterControllerComponent>();
			cct_component->collision_type = static_cast<CollisionType>(GameCollisionType::CCT);
			cct_component->height = 1.2f;
			cct_component->radius = 0.3f;
			cct_component->step_offset = 0.05f;

			tiran.add_component<MotionComponent>();
			tiran.add_component<TrajectoryComponent>();
			camera_entity.get_component<InputSenderComponent>()->add_reciever(tiran.add_component<InputReceiverComponent>());

			//RagdollComponent* ragdoll_component = tiran.add_component<RagdollComponent>();
			//ragdoll_component->simulated = true;
			PhysicalAnimationComponent* ragdoll_component = tiran.add_component<PhysicalAnimationComponent>();
			ragdoll_component->joint_init_ids = joint_init_ids;
			ragdoll_component->settings = settings;
		}

		/*if (auto mesh = loadMeshFromFileAsync(get_asset_path("/resources/assets/Sponza/sponza.obj"), mesh_creation_flags_unreal_asset))
		{
			auto sponza = world->create_entity("Sponza");
			sponza.add_component<MeshComponent>(mesh);

			TransformComponent* transform_component = sponza.get_component<TransformComponent>();
			transform_component->set_world_position(vec3(5.0f, -3.75f, 35.0f));
		}*/

		auto plane = world->create_entity("Platform");
		plane.add_component<PlaneComponent>(CollisionType::TERRAIN, vec3(0.f, -5.0, 0.0f));
		plane.add_component<MeshComponent>(groundMesh);
		plane.get_component<TransformComponent>()->set_world_transform(trs{vec3(10, -9.f, 0.f), quat(vec3(1.f, 0.f, 0.f), deg2rad(0.f)), vec3(5.0f, 1.0f, 5.0f)});
		
		groundMesh->mesh = builder.createDXMesh();
	}

	void GameInitSystem::update(float dt)
	{
		using namespace physics;
		const InputReceiverComponent* input_receiver = camera_entity.get_component<InputReceiverComponent>();

		const UserInput& frame_input = input_receiver->get_frame_input();
		if (frame_input.keyboard[key_code::key_space].press_event)
		{
			mesh_builder builder;

			auto defaultmat = createPBRMaterialAsync({ "", "" });
			defaultmat->shader = pbr_material_shader_double_sided;

			auto sphere_mesh = make_ref<multi_mesh>();
			builder.pushSphere({ });
			sphere_mesh->submeshes.push_back({ builder.endSubmesh(), {}, trs::identity, defaultmat });

			sphere_mesh->mesh = builder.createDXMesh();

			Entity sphere = world->create_entity("Sphere");

			ref<PhysicsMaterial> material = PhysicsHolder::physics_ref->get_default_material();

			SphereShapeComponent* sphere_shape_component = sphere.add_component<SphereShapeComponent>();
			sphere_shape_component->collision_type = static_cast<CollisionType>(GameCollisionType::DYNAMICS);
			sphere_shape_component->radius = 0.2f;
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
			dynamic_body_component->sleep_threshold.get_for_write() = 0.1f;

			Force& force = dynamic_body_component->forces.emplace_back();
			force.force = (camera_holder_component->get_render_camera()->rotation * quat(vec3(0.0f, 1.0f, 0.0f), M_PI)) * vec3(0.0f, 0.0f, 1.0f) * sphere_speed;
			force.mode = ForceMode::FORCE;
		}
	}
}