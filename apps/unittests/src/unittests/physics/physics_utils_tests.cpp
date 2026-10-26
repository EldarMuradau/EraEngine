#include <gtest/gtest.h>

#include <physics/collision_types.h>
#include <physics/collisions_holder_root_component.h>
#include <physics/physx_api.h>
#include <physics/ragdolls/ragdoll_component.h>
#include <physics/physical_animation/ragdoll_drive_compiler.h>
#include <physics/physical_animation/ragdoll_profile.h>
#include <physics/physical_animation/ragdoll_profiles_asset.h>

#include <core/math.h>

#include <sstream>

namespace
{
    using namespace era_engine;
    using namespace era_engine::physics;

    constexpr uint32 to_bits(CollisionType type)
    {
        return static_cast<uint32>(type);
    }

    LimbMassProperties make_mass_properties(float mass, float inertia_cm, float inertia_pivot, float linear_effective_mass)
    {
        LimbMassProperties properties;
        properties.valid = true;
        properties.mass = mass;
        properties.inertia_cm = inertia_cm;
        properties.inertia_pivot = inertia_pivot;
        properties.linear_effective_mass = linear_effective_mass;
        return properties;
    }
}

TEST(Physics_Collisions, CollisionTypeFlags)
{
    const CollisionType combined = CollisionType::COLLISION_0 | CollisionType::TERRAIN;

    EXPECT_EQ(to_bits(combined), (1u << 0) | (1u << 30));
    EXPECT_EQ(to_bits(combined & CollisionType::TERRAIN), to_bits(CollisionType::TERRAIN));
    EXPECT_EQ(to_bits(combined & CollisionType::RAGDOLL), 0u);
    EXPECT_EQ(to_bits(CollisionType::RAGDOLL), 1u << 31);
    EXPECT_EQ(to_bits(CollisionType::ALL), 0xFFFFFFFFu);
    EXPECT_EQ(TYPES_COUNT, 32u);
}

TEST(Physics_Collisions, FilterMatrixIsSymmetric)
{
    CollisionsHolderRootComponent filter;

    EXPECT_FALSE(filter.check_collision(to_bits(CollisionType::COLLISION_0), to_bits(CollisionType::TERRAIN)));

    filter.set_collision_filter(to_bits(CollisionType::COLLISION_0), to_bits(CollisionType::TERRAIN), true);

    EXPECT_TRUE(filter.check_collision(to_bits(CollisionType::COLLISION_0), to_bits(CollisionType::TERRAIN)));
    EXPECT_TRUE(filter.check_collision(to_bits(CollisionType::TERRAIN), to_bits(CollisionType::COLLISION_0)));
    EXPECT_FALSE(filter.check_collision(to_bits(CollisionType::COLLISION_0), to_bits(CollisionType::COLLISION_0)));
    EXPECT_FALSE(filter.check_collision(to_bits(CollisionType::TERRAIN), to_bits(CollisionType::TERRAIN)));

    EXPECT_EQ(filter.get_collision_filter(to_bits(CollisionType::COLLISION_0)), to_bits(CollisionType::TERRAIN));
    EXPECT_EQ(filter.get_collision_filter(to_bits(CollisionType::TERRAIN)), to_bits(CollisionType::COLLISION_0));
}

TEST(Physics_Collisions, FilterMasksAndRemoval)
{
    CollisionsHolderRootComponent filter;

    const uint32 ragdoll = to_bits(CollisionType::RAGDOLL);
    const uint32 statics = to_bits(CollisionType::TERRAIN | CollisionType::COLLISION_1);

    filter.set_collision_filter(ragdoll, statics | ragdoll, true);

    EXPECT_EQ(filter.get_collision_filter(ragdoll), statics | ragdoll);
    EXPECT_TRUE(filter.check_collision(ragdoll, to_bits(CollisionType::COLLISION_1)));
    EXPECT_TRUE(filter.check_collision(ragdoll, ragdoll));
    EXPECT_TRUE(filter.check_collision(to_bits(CollisionType::COLLISION_1), ragdoll));
    EXPECT_FALSE(filter.check_collision(to_bits(CollisionType::COLLISION_1), to_bits(CollisionType::TERRAIN)));

    filter.set_collision_filter(ragdoll, to_bits(CollisionType::TERRAIN), false);

    EXPECT_FALSE(filter.check_collision(ragdoll, to_bits(CollisionType::TERRAIN)));
    EXPECT_FALSE(filter.check_collision(to_bits(CollisionType::TERRAIN), ragdoll));
    EXPECT_TRUE(filter.check_collision(ragdoll, to_bits(CollisionType::COLLISION_1)));

    EXPECT_EQ(filter.get_collision_filter(ragdoll | to_bits(CollisionType::TERRAIN)), to_bits(CollisionType::COLLISION_1) | ragdoll);
    EXPECT_TRUE(filter.check_collision(ragdoll | to_bits(CollisionType::COLLISION_5), to_bits(CollisionType::COLLISION_1)));
}

TEST(Physics_Conversions, RoundTrip)
{
    const vec3 position(1.0f, -2.0f, 3.5f);
    const quat rotation(normalize(vec3(1.0f, 2.0f, 3.0f)), 0.7f);

    const physx::PxVec3 px_position = physx::create_PxVec3(position);
    EXPECT_FLOAT_EQ(px_position.x, 1.0f);
    EXPECT_FLOAT_EQ(px_position.y, -2.0f);
    EXPECT_FLOAT_EQ(px_position.z, 3.5f);
    EXPECT_EQ(physx::create_vec3(px_position), position);

    const physx::PxQuat px_rotation = physx::create_PxQuat(rotation);
    EXPECT_FLOAT_EQ(px_rotation.w, rotation.w);
    EXPECT_EQ(physx::create_quat(px_rotation), rotation);

    const trs transform(position, rotation, vec3(1.0f));
    const physx::PxTransform px_transform = physx::create_PxTransform(transform);
    EXPECT_TRUE(fuzzy_equals(physx::create_trs(px_transform), transform));

    const vec3 point(0.3f, 0.2f, -0.1f);
    EXPECT_TRUE(fuzzy_equals(physx::create_vec3(px_transform.transform(physx::create_PxVec3(point))), transform_position(transform, point)));

    EXPECT_FLOAT_EQ(PX_GRAVITY.y, -9.81f);
}

TEST(Physics_Ragdoll, DefaultMassSettingsSumToOne)
{
    const RagdollSettings::MassSettings mass_settings;

    EXPECT_NEAR(mass_settings.get_total_percentage(), 1.0f, 1e-3f);
}

TEST(Physics_Ragdoll, CompileLimbDrive)
{
    MotorDriveSettings settings;
    settings.angular.frequency = 2.0f;
    settings.angular.damping_ratio = 1.0f;
    settings.angular.max_acceleration = 100.0f;
    settings.linear.frequency = 1.0f;
    settings.linear.damping_ratio = 0.8f;
    settings.linear.max_acceleration = 5.0f;

    LimbMassProperties mass_properties = make_mass_properties(4.0f, 0.1f, 0.4f, 2.0f);
    const CompiledLimbDrive drive = RagdollDriveCompiler::compile_limb_drive(settings, mass_properties, 1.0f);

    const float angular_omega = 2.0f * M_PI * settings.angular.frequency;
    EXPECT_NEAR(drive.angular.stiffness, angular_omega * angular_omega * 4.0f, 1e-2f);
    EXPECT_NEAR(drive.angular.damping, 2.0f * angular_omega * 4.0f * 1.0f, 1e-3f);
    EXPECT_NEAR(drive.torque_stiffness, 0.4f * angular_omega * angular_omega, 1e-3f);
    EXPECT_NEAR(drive.max_torque, 0.4f * 100.0f, 1e-3f);
    EXPECT_NEAR(drive.saturation_angle, drive.max_torque / drive.torque_stiffness, 1e-5f);
    EXPECT_NEAR(drive.angular.force_limit, 2.0f * drive.max_torque, 1e-3f);
    EXPECT_NEAR(drive.angular_frequency, 2.0f, 1e-4f);
    EXPECT_TRUE(drive.angular.accelerated);

    const float linear_omega = 2.0f * M_PI * settings.linear.frequency;
    EXPECT_NEAR(drive.linear.stiffness, linear_omega * linear_omega * 2.0f, 1e-3f);
    EXPECT_NEAR(drive.linear.damping, 2.0f * linear_omega * 2.0f * 0.8f, 1e-3f);
    EXPECT_NEAR(drive.max_force, 4.0f * 5.0f, 1e-3f);
    EXPECT_NEAR(drive.linear.force_limit, drive.max_force, 1e-3f);
    EXPECT_NEAR(drive.linear_frequency, 1.0f, 1e-4f);
    EXPECT_NEAR(drive.saturation_distance, 5.0f / (linear_omega * linear_omega), 1e-5f);
}

TEST(Physics_Ragdoll, DriveStrengthKeepsDampingRatio)
{
    const MotorDriveSettings settings;
    const LimbMassProperties mass_properties = make_mass_properties(3.0f, 0.05f, 0.2f, 1.5f);

    const CompiledLimbDrive normal = RagdollDriveCompiler::compile_limb_drive(settings, mass_properties, 1.0f);
    const CompiledLimbDrive strong = RagdollDriveCompiler::compile_limb_drive(settings, mass_properties, 4.0f);

    EXPECT_NEAR(strong.angular.stiffness, normal.angular.stiffness * 4.0f, 1e-2f);
    EXPECT_NEAR(strong.angular.damping, normal.angular.damping * 2.0f, 1e-3f);
    EXPECT_NEAR(strong.linear.stiffness, normal.linear.stiffness * 4.0f, 1e-2f);
    EXPECT_NEAR(strong.max_torque, normal.max_torque * 4.0f, 1e-2f);
    EXPECT_NEAR(strong.angular_frequency, normal.angular_frequency * 2.0f, 1e-4f);
    EXPECT_EQ(strong.angular_damping_ratio, normal.angular_damping_ratio);

    const CompiledLimbDrive disabled = RagdollDriveCompiler::compile_limb_drive(settings, mass_properties, -1.0f);
    EXPECT_FLOAT_EQ(disabled.angular.stiffness, 0.0f);
    EXPECT_FLOAT_EQ(disabled.linear.stiffness, 0.0f);
}

TEST(Physics_Ragdoll, InvalidMassPropertiesGiveEmptyDrive)
{
    const LimbMassProperties invalid;
    EXPECT_FLOAT_EQ(invalid.get_inertia_ratio(), 0.0f);
    EXPECT_FLOAT_EQ(invalid.get_linear_mass_ratio(), 0.0f);

    const CompiledLimbDrive drive = RagdollDriveCompiler::compile_limb_drive(MotorDriveSettings(), invalid, 1.0f);
    EXPECT_FLOAT_EQ(drive.angular.stiffness, 0.0f);
    EXPECT_FLOAT_EQ(drive.max_torque, 0.0f);
    EXPECT_FLOAT_EQ(drive.max_force, 0.0f);
}

TEST(Physics_Ragdoll, CompiledDriveLerp)
{
    const LimbMassProperties mass_properties = make_mass_properties(2.0f, 0.1f, 0.3f, 1.0f);
    const CompiledLimbDrive from = RagdollDriveCompiler::compile_limb_drive(MotorDriveSettings(), mass_properties, 0.5f);
    const CompiledLimbDrive to = RagdollDriveCompiler::compile_limb_drive(MotorDriveSettings(), mass_properties, 1.5f);

    const CompiledLimbDrive start = CompiledLimbDrive::lerp(from, to, 0.0f);
    const CompiledLimbDrive end = CompiledLimbDrive::lerp(from, to, 1.0f);
    const CompiledLimbDrive middle = CompiledLimbDrive::lerp(from, to, 0.5f);

    EXPECT_FLOAT_EQ(start.angular.stiffness, from.angular.stiffness);
    EXPECT_FLOAT_EQ(end.angular.stiffness, to.angular.stiffness);
    EXPECT_FLOAT_EQ(end.max_force, to.max_force);
    EXPECT_NEAR(middle.angular.stiffness, 0.5f * (from.angular.stiffness + to.angular.stiffness), 1e-3f);
    EXPECT_NEAR(middle.linear.force_limit, 0.5f * (from.linear.force_limit + to.linear.force_limit), 1e-3f);
}

TEST(Physics_Ragdoll, ContactHitStrength)
{
    ContactResponseSettings settings;
    settings.min_strength = 0.25f;
    settings.velocity_change_range = vec2(1.0f, 3.0f);

    EXPECT_FLOAT_EQ(RagdollDriveCompiler::compute_hit_strength(settings, 0.0f), 1.0f);
    EXPECT_FLOAT_EQ(RagdollDriveCompiler::compute_hit_strength(settings, 1.0f), 1.0f);
    EXPECT_NEAR(RagdollDriveCompiler::compute_hit_strength(settings, 2.0f), 0.625f, 1e-5f);
    EXPECT_FLOAT_EQ(RagdollDriveCompiler::compute_hit_strength(settings, 3.0f), 0.25f);
    EXPECT_FLOAT_EQ(RagdollDriveCompiler::compute_hit_strength(settings, 100.0f), 0.25f);

    EXPECT_FLOAT_EQ(RagdollDriveCompiler::get_contact_softness(settings, 1.0f), 0.0f);
    EXPECT_NEAR(RagdollDriveCompiler::get_contact_softness(settings, 0.625f), 0.5f, 1e-5f);
    EXPECT_FLOAT_EQ(RagdollDriveCompiler::get_contact_softness(settings, 0.25f), 1.0f);
    EXPECT_FLOAT_EQ(RagdollDriveCompiler::get_contact_softness(settings, 0.0f), 1.0f);
}

TEST(Physics_Ragdoll, ApplyContactStrength)
{
    MotorDriveSettings drive_settings;
    drive_settings.angular.damping_ratio = 0.5f;
    drive_settings.linear.damping_ratio = 1.0f;

    const LimbMassProperties mass_properties = make_mass_properties(2.0f, 0.1f, 0.3f, 1.0f);
    const CompiledLimbDrive drive = RagdollDriveCompiler::compile_limb_drive(drive_settings, mass_properties, 1.0f);

    ContactResponseSettings settings;
    settings.min_strength = 0.25f;
    settings.hit_damping_ratio = 1.0f;

    const CompiledLimbDrive unchanged = RagdollDriveCompiler::apply_contact_strength(drive, settings, 1.0f);
    EXPECT_FLOAT_EQ(unchanged.angular.stiffness, drive.angular.stiffness);
    EXPECT_FLOAT_EQ(unchanged.angular.damping, drive.angular.damping);
    EXPECT_FLOAT_EQ(unchanged.linear.force_limit, drive.linear.force_limit);

    const CompiledLimbDrive soft = RagdollDriveCompiler::apply_contact_strength(drive, settings, 0.25f);
    EXPECT_NEAR(soft.angular.stiffness, drive.angular.stiffness * 0.25f, 1e-2f);
    EXPECT_NEAR(soft.angular.damping, drive.angular.damping * 0.5f * 2.0f, 1e-3f);
    EXPECT_NEAR(soft.angular.force_limit, drive.angular.force_limit * 0.25f, 1e-3f);
    EXPECT_NEAR(soft.linear.damping, drive.linear.damping * 0.5f, 1e-3f);

    const float omega = std::sqrt(soft.angular.stiffness / mass_properties.get_inertia_ratio());
    EXPECT_NEAR(soft.angular.damping / (2.0f * omega * mass_properties.get_inertia_ratio()), settings.hit_damping_ratio, 1e-3f);
}

TEST(Physics_Ragdoll, ProfileStrength)
{
    RagdollProfile profile;
    PhysicalLimbDetails& arm = profile.get_limb_details_by_type(RagdollLimbType::ARM);
    arm.strength_details.default_strength_coeff = 1.0f;
    arm.strength_details.soft_strength_coeff = 0.25f;
    arm.strength_details.hard_strength_coeff = 3.0f;

    EXPECT_FLOAT_EQ(profile.get_strength(RagdollLimbType::ARM, RagdollProfileStrengthType::DEFAULT), 1.0f);
    EXPECT_FLOAT_EQ(profile.get_strength(RagdollLimbType::ARM, RagdollProfileStrengthType::SOFT), 0.25f);
    EXPECT_FLOAT_EQ(profile.get_strength(RagdollLimbType::ARM, RagdollProfileStrengthType::HARD), 3.0f);
    EXPECT_EQ(&profile.get_limb_details_by_type(RagdollLimbType::ARM), &profile.arm_limb_details);
    EXPECT_EQ(&profile.get_limb_details_by_type(RagdollLimbType::FOOT), &profile.foot_limb_details);
}

TEST(Physics_Ragdoll, DefaultProfilesAsset)
{
    const ref<RagdollProfilesAsset> asset = RagdollProfilesAsset::create_default();
    ASSERT_NE(asset, nullptr);

    for (RagdollProfileType type : {RagdollProfileType::IDLE, RagdollProfileType::RUNNING, RagdollProfileType::SPRINT, RagdollProfileType::RAGDOLL, RagdollProfileType::DRAG})
    {
        const ref<RagdollProfile> profile = asset->get_profile(type);
        ASSERT_NE(profile, nullptr);
        EXPECT_EQ(profile->type, type);
    }

    const ref<RagdollProfile> idle = asset->get_profile(RagdollProfileType::IDLE);
    ASSERT_TRUE(idle->leg_limb_details.contact_response.has_value());
    EXPECT_FALSE(idle->foot_limb_details.contact_response->respond_to_own_hits);
    EXPECT_TRUE(idle->hand_limb_details.contact_response->respond_to_own_hits);
    EXPECT_FALSE(asset->get_profile(RagdollProfileType::RAGDOLL)->hand_limb_details.contact_response.has_value());

    const ref<RagdollProfile> drag = asset->get_profile(RagdollProfileType::DRAG);
    EXPECT_FALSE(drag->body_upper_limb_details.drag_force.has_value());
    EXPECT_FLOAT_EQ(drag->body_upper_limb_details.motor_drive->angular.frequency, idle->body_upper_limb_details.motor_drive->angular.frequency);
    ASSERT_TRUE(drag->arm_limb_details.drag_force.has_value());
    ASSERT_TRUE(drag->arm_limb_details.motor_drive.has_value());
    EXPECT_LT(drag->arm_limb_details.motor_drive->angular.frequency, idle->arm_limb_details.motor_drive->angular.frequency);

    EXPECT_LT(asset->selection_settings.running_speed, asset->selection_settings.sprint_speed);
}

TEST(Physics_Ragdoll, ProfilesAssetSerialization)
{
    const ref<RagdollProfilesAsset> source = RagdollProfilesAsset::create_default();
    source->selection_settings.running_speed = 2.5f;
    source->selection_settings.hysteresis = 0.4f;

    PhysicalLimbDetails& arm = source->get_profile(RagdollProfileType::RUNNING)->get_limb_details_by_type(RagdollLimbType::ARM);
    if (!arm.motor_drive.has_value())
    {
        arm.motor_drive = MotorDriveSettings();
    }
    arm.motor_drive->angular.frequency = 7.5f;
    arm.strength_details.hard_strength_coeff = 2.25f;
    arm.contact_response = ContactResponseSettings();
    arm.contact_response->min_strength = 0.4f;
    arm.contact_response->velocity_change_range = vec2(0.75f, 5.0f);
    arm.contact_response->parent_follow = 0.6f;
    arm.contact_response->touch_strength = 0.35f;
    arm.contact_response->respond_to_own_hits = false;
    arm.blend_type = PhysicalLimbBlendType::BLEND_WITH_PREV_POSE | PhysicalLimbBlendType::PURE_PHYSICS;
    arm.drag_force = DragForceDetails();
    arm.drag_force->acceleration_gain = 0.05f;

    std::stringstream stream;
    ASSERT_TRUE(source->serialize(stream));

    RagdollProfilesAsset result;
    ASSERT_TRUE(result.deserialize(stream));

    EXPECT_FLOAT_EQ(result.selection_settings.running_speed, 2.5f);
    EXPECT_FLOAT_EQ(result.selection_settings.hysteresis, 0.4f);
    EXPECT_FLOAT_EQ(result.selection_settings.sprint_speed, source->selection_settings.sprint_speed);

    const ref<RagdollProfile> running = result.get_profile(RagdollProfileType::RUNNING);
    ASSERT_NE(running, nullptr);

    const PhysicalLimbDetails& result_arm = running->get_limb_details_by_type(RagdollLimbType::ARM);
    ASSERT_TRUE(result_arm.motor_drive.has_value());
    EXPECT_FLOAT_EQ(result_arm.motor_drive->angular.frequency, 7.5f);
    EXPECT_FLOAT_EQ(result_arm.strength_details.hard_strength_coeff, 2.25f);
    EXPECT_EQ(result_arm.blend_type, arm.blend_type);
    ASSERT_TRUE(result_arm.contact_response.has_value());
    EXPECT_FLOAT_EQ(result_arm.contact_response->min_strength, 0.4f);
    EXPECT_EQ(result_arm.contact_response->velocity_change_range, vec2(0.75f, 5.0f));
    EXPECT_FLOAT_EQ(result_arm.contact_response->parent_follow, 0.6f);
    EXPECT_FLOAT_EQ(result_arm.contact_response->touch_strength, 0.35f);
    EXPECT_FALSE(result_arm.contact_response->respond_to_own_hits);
    ASSERT_TRUE(result_arm.drag_force.has_value());
    EXPECT_FLOAT_EQ(result_arm.drag_force->acceleration_gain, 0.05f);

    const ref<RagdollProfile> ragdoll = result.get_profile(RagdollProfileType::RAGDOLL);
    ASSERT_NE(ragdoll, nullptr);
    EXPECT_FALSE(ragdoll->arm_limb_details.motor_drive.has_value());
    EXPECT_FALSE(ragdoll->arm_limb_details.contact_response.has_value());
    EXPECT_EQ(result.get_profile(RagdollProfileType::CLIMBING), nullptr);

    std::stringstream broken("not: [valid");
    RagdollProfilesAsset broken_result;
    EXPECT_FALSE(broken_result.deserialize(broken));
}
