#include "tumble/util/error.h"
#include "tumble/math/mat4f.h"
#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/dof.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/body/rigid_body_step.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/dispatch.h"
#include "tumble/physics/shapes/empty.h"
#include "tumble/physics/shapes/sphere.h"
#include "tumble/physics/world.h"
#include "tumble/util/alloc.h"
#include "test.h"
#include "test_math.h"

using rigid_body::RigidBody;

struct Fixture {
    world::World *world;
    uint32_t moving;
};

static Fixture make_fixture() {
    dispatch::init();
    Fixture f;
    world_settings::WorldSettings settings = world_settings::create();
    f.moving = world_settings::add_object_layer(settings, world_settings::add_broadphase_layer(settings));
    world_settings::enable_collision(settings, f.moving, f.moving);
    f.world = world::create(settings);
    return f;
}

static void release_fixture(Fixture &f) { world::destroy(f.world); }

/** within eps of (x, y, z) by distance */
static bool near_point(Vec3f a, float x, float y, float z, float eps = 1e-5f) { return vec3f::distance(a, vec3f::make(x, y, z)) <= eps; }

static bool near_point(const Float3 &a, float x, float y, float z, float eps = 1e-5f) { return near_point(vec3f::load(a), x, y, z, eps); }

static bool active_list_consistent(const bodies::Bodies &bodies) {
    for (uint32_t i = 0; i < bodies.active_body_count; i++) {
        const RigidBody *body = bodies.pool[bodies.active_body_indices[i]];
        if (body->active_index != i) return false;
    }
    return true;
}

void test_rigid_body_create_box_mass(TestCtx &ctx) {
    Fixture f = make_fixture();
    // a 1 x 2 x 4 box at 500 kg/m^3: m = 4000, I = m/12 (b^2 + c^2) about each axis
    shapes::Shape *shape = box::create({.half_extents = {0.5f, 1.0f, 2.0f}, .density = 500.0f});
    RigidBody *body = rigid_body::create(*f.world,
                                         {.shape = shape, .object_layer = f.moving, .motion_type = motion_type::MOTION_DYNAMIC, .user_data = 0x100000007ull});
    CHECK(ctx, body != nullptr);
    const float mass = 4000.0f;
    CHECK_NEAR(ctx, body->mass_properties.mass, mass, 1e-2f);
    CHECK_NEAR(ctx, body->motion_properties.inv_mass, 1.0f / mass, 1e-9f);
    const float ixx = mass / 12.0f * (4.0f + 16.0f);
    const float iyy = mass / 12.0f * (1.0f + 16.0f);
    const float izz = mass / 12.0f * (1.0f + 4.0f);
    // unrotated, the world inverse inertia is the inverse of the diagonal
    Float4x4 inverse_inertia;
    rigid_body::get_inverse_inertia(inverse_inertia, body);
    const float *m = inverse_inertia.m;
    CHECK_NEAR(ctx, m[0] * ixx, 1.0f, 1e-4f);
    CHECK_NEAR(ctx, m[5] * iyy, 1.0f, 1e-4f);
    CHECK_NEAR(ctx, m[10] * izz, 1.0f, 1e-4f);
    CHECK(ctx, scalar::abs(m[1]) < 1e-9f && scalar::abs(m[2]) < 1e-9f && scalar::abs(m[6]) < 1e-9f);

    // the settings' defaults
    // all 64 bits of the user data come through
    CHECK(ctx, body->user_data == 0x100000007ull && body->shape == shape && body->object_layer == f.moving);
    CHECK(ctx, body->friction == 0.2f && body->restitution == 0.0f);
    CHECK(ctx, body->friction_combine_mode == combine_material::GEOMETRIC_MEAN && body->restitution_combine_mode == combine_material::MAX);
    CHECK(ctx, body->collision_groups == 0xffffffffu && body->collision_mask == 0xffffffffu);
    CHECK(ctx, body->motion_properties.linear_damping == 0.05f && body->motion_properties.angular_damping == 0.05f);
    CHECK(ctx, body->motion_properties.max_linear_velocity == 500.0f);
    CHECK_NEAR(ctx, body->motion_properties.max_angular_velocity, 0.25f * scalar::PI * 60.0f, 1e-5f);
    CHECK(ctx, body->motion_properties.allowed_degrees_of_freedom == dof::DOF_ALL && body->motion_properties.allow_sleeping);
    CHECK(ctx, body->use_manifold_reduction && !body->sensor && !body->sleeping);
    // in the world: in its layer's broadphase tree and the move set, for the next find_pairs to pair
    CHECK(ctx, body->broadphase_node != -1 && body->broadphase_layer == 0 && body->in_move_set);
    CHECK(ctx, body->island_index == -1 && body->ccd_body_index == -1 && body->head_pair_key == -1 && body->constraint_ids.length == 0);
    // zero velocities and accumulators; the box is centred, so the centre of mass is the position
    CHECK(ctx, vec3f::length(vec3f::load(body->motion_properties.linear_velocity)) == 0.0f && vec3f::length(vec3f::load(body->motion_properties.force)) == 0.0f);
    CHECK(ctx, near_point(body->center_of_mass_position, 0.0f, 0.0f, 0.0f));
    // dynamic and awake: on the active list
    CHECK(ctx, body->active_index == 0 && f.world->bodies.active_body_count == 1);

    // a static body gets no inverse mass or inertia and stays off the active list
    RigidBody *ground = rigid_body::create(*f.world, {.shape = shape, .object_layer = f.moving, .motion_type = motion_type::MOTION_STATIC});
    CHECK(ctx, ground->motion_properties.inv_mass == 0.0f);
    CHECK(ctx, vec3f::length(vec3f::load(ground->motion_properties.inv_inertia_diagonal)) == 0.0f);
    rigid_body::get_inverse_inertia(inverse_inertia, ground);
    bool zero = true;
    for (int i = 0; i < 16; i++) zero = zero && inverse_inertia.m[i] == 0.0f;
    CHECK(ctx, zero);
    CHECK(ctx, ground->active_index == sleep::INACTIVE_BODY_INDEX && f.world->bodies.active_body_count == 1);

    release_fixture(f);
    box::destroy(shape);
}

void test_rigid_body_mass_overrides(TestCtx &ctx) {
    Fixture f = make_fixture();
    shapes::Shape *shape = sphere::create({.radius = 1.0f, .density = 1000.0f});

    // computed: (4/3) pi r^3 density
    RigidBody *computed = rigid_body::create(*f.world, {.shape = shape, .object_layer = f.moving, .motion_type = motion_type::MOTION_DYNAMIC});
    CHECK_NEAR(ctx, 1.0f / computed->motion_properties.inv_mass, 4.0f / 3.0f * scalar::PI * 1000.0f, 0.5f);
    CHECK(ctx, computed->mass_properties_override == rigid_body::CALCULATE_MASS_AND_INERTIA);

    // full override: mass and inertia as given
    mass_properties::MassProperties provided = mass_properties::create();
    provided.mass = 500.0f;
    mat4f::store(provided.inertia, mat4f::identity());
    provided.inertia.m[0] = 100.0f;
    provided.inertia.m[5] = 100.0f;
    provided.inertia.m[10] = 100.0f;
    RigidBody *overridden = rigid_body::create(
        *f.world,
        {.shape = shape, .object_layer = f.moving, .motion_type = motion_type::MOTION_DYNAMIC, .mass = 3.0f, .mass_properties_override = &provided});
    CHECK(ctx, overridden->mass_properties_override == rigid_body::MASS_AND_INERTIA_PROVIDED);
    CHECK_NEAR(ctx, overridden->motion_properties.inv_mass, 1.0f / 500.0f, 1e-8f);
    CHECK_NEAR(ctx, overridden->motion_properties.inv_inertia_diagonal.x, 1.0f / 100.0f, 1e-7f);

    // mass only: the sphere's inertia 2/5 m r^2, scaled to the mass
    RigidBody *mass_only =
        rigid_body::create(*f.world, {.shape = shape, .object_layer = f.moving, .motion_type = motion_type::MOTION_DYNAMIC, .mass = 10.0f});
    CHECK(ctx, mass_only->mass_properties_override == rigid_body::CALCULATE_INERTIA);
    CHECK_NEAR(ctx, mass_only->motion_properties.inv_mass, 0.1f, 1e-7f);
    CHECK_NEAR(ctx, mass_only->mass_properties.inertia.m[0], 0.4f * 10.0f, 1e-4f);

    release_fixture(f);
    sphere::destroy(shape);
}

void test_rigid_body_null_shape_is_empty(TestCtx &ctx) {
    Fixture f = make_fixture();
    RigidBody *body = rigid_body::create(*f.world, {.object_layer = f.moving, .motion_type = motion_type::MOTION_STATIC, .position = {1.0f, 2.0f, 3.0f}});
    CHECK(ctx, body->shape != nullptr && body->shape->type == shapes::SHAPE_EMPTY);
    CHECK(ctx, near_point(body->center_of_mass_position, 1.0f, 2.0f, 3.0f));
    // the pool's one shared empty shape, which a second shapeless body and a removed body hold too; the pool frees it
    CHECK(ctx, body->shape == f.world->bodies.pooled_body_shape);
    RigidBody *other = rigid_body::create(*f.world, {.object_layer = f.moving, .motion_type = motion_type::MOTION_STATIC});
    CHECK(ctx, other->shape == body->shape);
    release_fixture(f);
}

void test_rigid_body_aabb(TestCtx &ctx) {
    Fixture f = make_fixture();
    shapes::Shape *shape = box::create({.half_extents = {1.0f, 0.5f, 0.25f}});

    // 30 degrees about z: the extents in x and y mix by |cos| and |sin|, z keeps its own
    const float angle = scalar::PI / 6.0f;
    const Float4 turned = stored_axis_angle(vec3f::make(0.0f, 0.0f, 1.0f), angle);
    RigidBody *body = rigid_body::create(*f.world,
                                         {.shape = shape, .object_layer = f.moving, .motion_type = motion_type::MOTION_DYNAMIC, .position = {1.0f, 2.0f, 3.0f},
                                          .quaternion = turned});
    const float c = scalar::cos(angle);
    const float s = scalar::sin(angle);
    const float ex = c * 1.0f + s * 0.5f;
    const float ey = s * 1.0f + c * 0.5f;
    CHECK(ctx, near_point(body->aabb.min, 1.0f - ex, 2.0f - ey, 3.0f - 0.25f));
    CHECK(ctx, near_point(body->aabb.max, 1.0f + ex, 2.0f + ey, 3.0f + 0.25f));

    // any rotation: the box of the box through the rotation and translation matrix
    const Float4 tumbled = stored_axis_angle(vec3f::make(0.3f, -0.8f, 0.5f), 1.3f);
    rigid_body::set_transform(*f.world, body, vec3f::make(-4.0f, 0.5f, 9.0f), quatf::load(tumbled), false);
    const Mat4f transform = mat4f::from_rotation_translation(quatf::load(tumbled), vec3f::make(-4.0f, 0.5f, 9.0f));
    const AABox expected = aabox::transform_mat4(aabox::load(shape->aabb), transform);
    CHECK(ctx, vec3f::distance(vec3f::load(body->aabb.min), expected.min) < 1e-5f && vec3f::distance(vec3f::load(body->aabb.max), expected.max) < 1e-5f);

    // set_position moves the aabb with the body
    rigid_body::set_position(*f.world, body, vec3f::make(0.0f, 0.0f, 0.0f), false);
    CHECK(ctx, vec3f::distance(vec3f::load(body->aabb.min), vec3f::add(expected.min, vec3f::make(4.0f, -0.5f, -9.0f))) < 1e-5f);

    release_fixture(f);
    box::destroy(shape);
}

void test_rigid_body_remove_and_ids(TestCtx &ctx) {
    Fixture f = make_fixture();
    shapes::Shape *shape = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});
    const rigid_body::RigidBodySettings settings = {.shape = shape, .object_layer = f.moving, .motion_type = motion_type::MOTION_DYNAMIC};
    RigidBody *a = rigid_body::create(*f.world, settings);
    RigidBody *b = rigid_body::create(*f.world, settings);
    CHECK(ctx, a->index == 0 && b->index == 1 && body_id::index(b->id) == 1 && body_id::sequence(b->id) == b->sequence);
    CHECK(ctx, rigid_body::get(*f.world, a->id) == a && rigid_body::get(*f.world, b->id) == b);

    // remove frees the slot and takes the body off the active list
    const body_id::BodyId old_id = a->id;
    CHECK(ctx, rigid_body::remove(*f.world, a));
    CHECK(ctx, rigid_body::get(*f.world, old_id) == nullptr);
    CHECK(ctx, f.world->bodies.active_body_count == 1 && b->active_index == 0 && active_list_consistent(f.world->bodies));
    CHECK(ctx, !rigid_body::remove(*f.world, a));

    // the next body takes the same slot and record, under a fresh id the old one does not match
    RigidBody *c = rigid_body::create(*f.world, settings);
    CHECK(ctx, c == a && c->index == 0);
    CHECK(ctx, c->id != old_id);
    CHECK(ctx, rigid_body::get(*f.world, c->id) == c && rigid_body::get(*f.world, old_id) == nullptr);
    CHECK(ctx, f.world->bodies.active_body_count == 2 && active_list_consistent(f.world->bodies));

    // a reused slot carries nothing of the old body's motion state: a static made where a dynamic with
    // step overrides was has no inverse mass and no overrides, and a dynamic after it inherits none
    c->motion_properties.num_velocity_steps_override = 12;
    c->motion_properties.num_position_steps_override = 7;
    CHECK(ctx, c->motion_properties.inv_mass > 0.0f);
    CHECK(ctx, rigid_body::remove(*f.world, c));
    RigidBody *ground = rigid_body::create(*f.world, {.shape = shape, .object_layer = f.moving, .motion_type = motion_type::MOTION_STATIC});
    CHECK(ctx, ground == c && ground->index == 0);
    CHECK(ctx, ground->motion_properties.inv_mass == 0.0f);
    CHECK(ctx, vec3f::length(vec3f::load(ground->motion_properties.inv_inertia_diagonal)) == 0.0f);
    CHECK(ctx, ground->motion_properties.num_velocity_steps_override == 0 && ground->motion_properties.num_position_steps_override == 0);
    CHECK(ctx, ground->motion_properties.world_inverse_inertia_stamp == motion_properties::STEP_STAMP_NONE);
    CHECK(ctx, rigid_body::remove(*f.world, ground));
    RigidBody *d = rigid_body::create(*f.world, settings);
    CHECK(ctx, d == c && d->motion_properties.num_velocity_steps_override == 0 && d->motion_properties.inv_mass > 0.0f);

    release_fixture(f);
    box::destroy(shape);
}

void test_rigid_body_remove_drops_shape_and_step_overrides(TestCtx &ctx) {
    const size_t blocks_before = util::allocated_blocks();
    Fixture f = make_fixture();
    shapes::Shape *shape = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});

    // the step overrides in the settings land in the motion properties
    RigidBody *a = rigid_body::create(*f.world,
                                      {.shape = shape, .object_layer = f.moving, .motion_type = motion_type::MOTION_DYNAMIC,
                                       .num_velocity_steps_override = 12, .num_position_steps_override = 7});
    RigidBody *b = rigid_body::create(*f.world, {.shape = shape, .object_layer = f.moving, .motion_type = motion_type::MOTION_DYNAMIC});
    CHECK(ctx, a->motion_properties.num_velocity_steps_override == 12 && a->motion_properties.num_position_steps_override == 7);
    CHECK(ctx, b->motion_properties.num_velocity_steps_override == 0 && b->motion_properties.num_position_steps_override == 0);

    // removed bodies let go of the shape they were given and hold one shared empty shape
    CHECK(ctx, f.world->bodies.pooled_body_shape == nullptr);
    CHECK(ctx, rigid_body::remove(*f.world, a));
    CHECK(ctx, a->shape != shape && a->shape != nullptr && a->shape->type == shapes::SHAPE_EMPTY);
    CHECK(ctx, rigid_body::remove(*f.world, b));
    CHECK(ctx, b->shape == a->shape && b->shape == f.world->bodies.pooled_body_shape);

    // a reused slot created without overrides reads 0, and takes its new shape
    RigidBody *c = rigid_body::create(*f.world, {.shape = shape, .object_layer = f.moving, .motion_type = motion_type::MOTION_DYNAMIC});
    RigidBody *d = rigid_body::create(*f.world, {.shape = shape, .object_layer = f.moving, .motion_type = motion_type::MOTION_DYNAMIC});
    CHECK(ctx, (c == a || d == a) && c->shape == shape && d->shape == shape);
    CHECK(ctx, a->motion_properties.num_velocity_steps_override == 0 && a->motion_properties.num_position_steps_override == 0);

    // the shared empty shape goes with the world: nothing is left once the box goes too
    release_fixture(f);
    box::destroy(shape);
    CHECK(ctx, util::allocated_blocks() == blocks_before);
    CHECK(ctx, util::error() == 0);
}

void test_rigid_body_forces_and_impulses(TestCtx &ctx) {
    Fixture f = make_fixture();
    shapes::Shape *shape = sphere::create({.radius = 1.0f});
    RigidBody *body = rigid_body::create(*f.world, {.shape = shape, .object_layer = f.moving, .motion_type = motion_type::MOTION_DYNAMIC});
    motion_properties::MotionProperties &mp = body->motion_properties;

    // forces accumulate
    rigid_body::add_force(*f.world, body, vec3f::make(5.0f, 0.0f, 0.0f), true);
    rigid_body::add_force(*f.world, body, vec3f::make(3.0f, -1.0f, 0.0f), true);
    CHECK(ctx, near_point(mp.force, 8.0f, -1.0f, 0.0f));
    rigid_body::add_torque(*f.world, body, vec3f::make(0.0f, 2.0f, 0.0f), true);
    CHECK(ctx, near_point(mp.torque, 0.0f, 2.0f, 0.0f));
    rigid_body::clear_forces(body);
    CHECK(ctx, vec3f::length(vec3f::load(mp.force)) == 0.0f && vec3f::length(vec3f::load(mp.torque)) == 0.0f);

    // a force off the centre of mass adds r x f
    rigid_body::add_force_at_position(*f.world, body, vec3f::make(1.0f, 0.0f, 0.0f), vec3f::make(0.0f, -2.0f, 0.0f), true);
    CHECK(ctx, near_point(mp.force, 1.0f, 0.0f, 0.0f) && near_point(mp.torque, 0.0f, 0.0f, 2.0f));

    // an impulse changes velocity by impulse x inverse mass
    rigid_body::add_impulse(*f.world, body, vec3f::make(100.0f, 0.0f, -50.0f));
    CHECK(ctx, near_point(mp.linear_velocity, 100.0f * mp.inv_mass, 0.0f, -50.0f * mp.inv_mass, 1e-7f));

    // an angular impulse goes through the inverse inertia: 1 / (2/5 m r^2) for a sphere
    mp.angular_velocity = {0.0f, 0.0f, 0.0f};
    rigid_body::add_angular_impulse(*f.world, body, vec3f::make(0.0f, 1.0f, 0.0f));
    CHECK_NEAR(ctx, mp.angular_velocity.y, mp.inv_mass / 0.4f, 1e-7f);

    // an impulse off the centre of mass spins as well as pushes
    mp.linear_velocity = {0.0f, 0.0f, 0.0f};
    mp.angular_velocity = {0.0f, 0.0f, 0.0f};
    rigid_body::add_impulse_at_position(*f.world, body, vec3f::make(1.0f, 0.0f, 0.0f), vec3f::make(0.0f, 1.0f, 0.0f));
    CHECK(ctx, mp.linear_velocity.x > 0.0f && mp.angular_velocity.z < 0.0f);

    // velocity clamps to the maximum
    mp.max_linear_velocity = 10.0f;
    rigid_body::add_impulse(*f.world, body, vec3f::make(1.0e7f, 0.0f, 0.0f));
    CHECK(ctx, vec3f::length(vec3f::load(mp.linear_velocity)) <= 10.0f + 1e-4f);

    // static and kinematic bodies take no forces; static takes no impulses or velocities
    RigidBody *ground = rigid_body::create(*f.world, {.shape = shape, .object_layer = f.moving, .motion_type = motion_type::MOTION_STATIC});
    RigidBody *mover = rigid_body::create(*f.world, {.shape = shape, .object_layer = f.moving, .motion_type = motion_type::MOTION_KINEMATIC});
    rigid_body::add_force(*f.world, ground, vec3f::make(10.0f, 0.0f, 0.0f), true);
    rigid_body::add_force(*f.world, mover, vec3f::make(10.0f, 0.0f, 0.0f), true);
    rigid_body::add_impulse(*f.world, ground, vec3f::make(10.0f, 0.0f, 0.0f));
    rigid_body::add_angular_impulse(*f.world, ground, vec3f::make(10.0f, 0.0f, 0.0f));
    rigid_body::set_linear_velocity(*f.world, ground, vec3f::make(10.0f, 0.0f, 0.0f));
    CHECK(ctx, vec3f::length(vec3f::load(ground->motion_properties.force)) == 0.0f && vec3f::length(vec3f::load(mover->motion_properties.force)) == 0.0f);
    CHECK(ctx, vec3f::length(vec3f::load(ground->motion_properties.linear_velocity)) == 0.0f);
    CHECK(ctx, vec3f::length(vec3f::load(ground->motion_properties.angular_velocity)) == 0.0f);
    // a kinematic body takes velocities
    rigid_body::set_linear_velocity(*f.world, mover, vec3f::make(0.0f, 3.0f, 0.0f));
    rigid_body::add_angular_velocity(*f.world, mover, vec3f::make(0.0f, 0.0f, 1.0f));
    CHECK(ctx, near_point(mover->motion_properties.linear_velocity, 0.0f, 3.0f, 0.0f));
    CHECK(ctx, near_point(mover->motion_properties.angular_velocity, 0.0f, 0.0f, 1.0f));

    release_fixture(f);
    sphere::destroy(shape);
}

void test_rigid_body_velocity_at_point(TestCtx &ctx) {
    Fixture f = make_fixture();
    shapes::Shape *shape = sphere::create({.radius = 1.0f});
    RigidBody *body = rigid_body::create(*f.world,
                                         {.shape = shape, .object_layer = f.moving, .motion_type = motion_type::MOTION_DYNAMIC, .position = {5.0f, 0.0f, 0.0f}});
    rigid_body::set_linear_velocity(*f.world, body, vec3f::make(5.0f, 0.0f, 0.0f));
    rigid_body::set_angular_velocity(*f.world, body, vec3f::make(0.0f, 0.0f, 2.0f));

    // v = v_linear + w x r: (5, 0, 0) + (0, 0, 2) x (0, 1, 0) = (3, 0, 0)
    Vec3f velocity = rigid_body::get_velocity_at_point_com(body, vec3f::make(0.0f, 1.0f, 0.0f));
    CHECK(ctx, near_point(velocity, 3.0f, 0.0f, 0.0f));
    // at the centre of mass only the linear part
    velocity = rigid_body::get_velocity_at_point_com(body, vec3f::make(0.0f, 0.0f, 0.0f));
    CHECK(ctx, near_point(velocity, 5.0f, 0.0f, 0.0f));
    // the world point (6, 0, 0) is r = (1, 0, 0) from the centre of mass at (5, 0, 0): + (0, 2, 0)
    velocity = rigid_body::get_velocity_at_point(body, vec3f::make(6.0f, 0.0f, 0.0f));
    CHECK(ctx, near_point(velocity, 5.0f, 2.0f, 0.0f));

    // a static body has none anywhere
    RigidBody *ground = rigid_body::create(*f.world, {.shape = shape, .object_layer = f.moving, .motion_type = motion_type::MOTION_STATIC});
    ground->motion_properties.linear_velocity = {1.0f, 1.0f, 1.0f};
    velocity = rigid_body::get_velocity_at_point(ground, vec3f::make(15.0f, 10.0f, 2.0f));
    CHECK(ctx, vec3f::length(velocity) == 0.0f);

    release_fixture(f);
    sphere::destroy(shape);
}

void test_rigid_body_sleep_and_active_list(TestCtx &ctx) {
    Fixture f = make_fixture();
    shapes::Shape *shape = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});
    const rigid_body::RigidBodySettings settings = {.shape = shape, .object_layer = f.moving, .motion_type = motion_type::MOTION_DYNAMIC};
    RigidBody *bodies[4];
    for (int i = 0; i < 4; i++) bodies[i] = rigid_body::create(*f.world, settings);
    CHECK(ctx, f.world->bodies.active_body_count == 4 && active_list_consistent(f.world->bodies));

    // sleeping a body in the middle swaps the last one into its place
    rigid_body::set_linear_velocity(*f.world, bodies[1], vec3f::make(1.0f, 0.0f, 0.0f));
    bodies[1]->island_index = 3;
    rigid_body::sleep(*f.world, bodies[1]);
    CHECK(ctx, bodies[1]->sleeping && bodies[1]->active_index == sleep::INACTIVE_BODY_INDEX && bodies[1]->island_index == -1);
    CHECK(ctx, vec3f::length(vec3f::load(bodies[1]->motion_properties.linear_velocity)) == 0.0f);
    CHECK(ctx, f.world->bodies.active_body_count == 3);
    CHECK(ctx, bodies[3]->active_index == 1 && f.world->bodies.active_body_indices[1] == bodies[3]->index);
    CHECK(ctx, active_list_consistent(f.world->bodies));
    // sleeping again changes nothing
    rigid_body::sleep(*f.world, bodies[1]);
    CHECK(ctx, f.world->bodies.active_body_count == 3);

    // sleeping the last needs no swap
    rigid_body::sleep(*f.world, bodies[2]);
    CHECK(ctx, f.world->bodies.active_body_count == 2 && active_list_consistent(f.world->bodies));
    CHECK(ctx, bodies[0]->active_index == 0 && bodies[3]->active_index == 1);

    // waking appends at the end
    rigid_body::wake(*f.world, bodies[1]);
    CHECK(ctx, !bodies[1]->sleeping && bodies[1]->active_index == 2 && f.world->bodies.active_body_count == 3);
    CHECK(ctx, active_list_consistent(f.world->bodies));

    // set_position wakes a sleeping body only when asked
    rigid_body::set_position(*f.world, bodies[2], vec3f::make(0.0f, 3.0f, 0.0f), false);
    CHECK(ctx, bodies[2]->sleeping && near_point(bodies[2]->center_of_mass_position, 0.0f, 3.0f, 0.0f));
    rigid_body::set_position(*f.world, bodies[2], vec3f::make(0.0f, 4.0f, 0.0f), true);
    CHECK(ctx, !bodies[2]->sleeping && bodies[2]->active_index == 3 && active_list_consistent(f.world->bodies));
    // a wake resets the sleep timer and recentres the test spheres
    CHECK(ctx, bodies[2]->motion_properties.sleep_test_timer == 0.0f);
    CHECK(ctx, near_point(bodies[2]->motion_properties.sleep_test_spheres[0].center, 0.0f, 4.0f, 0.0f));

    // static bodies neither sleep nor join the list
    RigidBody *ground = rigid_body::create(*f.world, {.shape = shape, .object_layer = f.moving, .motion_type = motion_type::MOTION_STATIC});
    rigid_body::sleep(*f.world, ground);
    CHECK(ctx, !ground->sleeping);
    rigid_body::wake(*f.world, ground);
    CHECK(ctx, ground->active_index == sleep::INACTIVE_BODY_INDEX && f.world->bodies.active_body_count == 4);

    release_fixture(f);
    box::destroy(shape);
}

void test_rigid_body_set_motion_type(TestCtx &ctx) {
    Fixture f = make_fixture();
    shapes::Shape *shape = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});
    RigidBody *body = rigid_body::create(*f.world, {.shape = shape, .object_layer = f.moving, .motion_type = motion_type::MOTION_STATIC});
    CHECK(ctx, body->motion_properties.inv_mass == 0.0f && f.world->bodies.active_body_count == 0);

    // static to dynamic: on the active list, with the box's inverse mass (1 m^3 at 1000 kg/m^3)
    rigid_body::set_motion_type(*f.world, body, motion_type::MOTION_DYNAMIC, false);
    CHECK(ctx, body->active_index == 0 && f.world->bodies.active_body_count == 1);
    CHECK_NEAR(ctx, body->motion_properties.inv_mass, 1.0f / 1000.0f, 1e-8f);

    // dynamic to static: off the list and stopped
    rigid_body::set_linear_velocity(*f.world, body, vec3f::make(1.0f, 2.0f, 3.0f));
    rigid_body::set_angular_velocity(*f.world, body, vec3f::make(1.0f, 0.0f, 0.0f));
    body->motion_properties.world_inverse_inertia_stamp = 5;
    rigid_body::set_motion_type(*f.world, body, motion_type::MOTION_STATIC, true);
    CHECK(ctx, body->active_index == sleep::INACTIVE_BODY_INDEX && f.world->bodies.active_body_count == 0);
    CHECK(ctx, vec3f::length(vec3f::load(body->motion_properties.linear_velocity)) == 0.0f && vec3f::length(vec3f::load(body->motion_properties.angular_velocity)) == 0.0f);
    CHECK(ctx, body->motion_properties.world_inverse_inertia_stamp == motion_properties::STEP_STAMP_NONE);

    // a sleeping static body made kinematic stays off the list until woken
    body->sleeping = true;
    rigid_body::set_motion_type(*f.world, body, motion_type::MOTION_KINEMATIC, false);
    CHECK(ctx, body->active_index == sleep::INACTIVE_BODY_INDEX);
    rigid_body::set_motion_type(*f.world, body, motion_type::MOTION_DYNAMIC, true);
    CHECK(ctx, !body->sleeping && body->active_index == 0);

    // the object layer is a plain write here; the broadphase is the caller's
    const uint32_t other = layers::add_object_layer(f.world->settings.layers, 0);
    rigid_body::set_object_layer(*f.world, body, other);
    CHECK(ctx, body->object_layer == other);

    release_fixture(f);
    box::destroy(shape);
}

void test_rigid_body_move_kinematic(TestCtx &ctx) {
    Fixture f = make_fixture();
    // a centre of mass off the origin, so the linear part has to follow the rotated offset
    shapes::Shape *shape = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});
    shape->center_of_mass = {1.0f, 0.0f, 0.0f};
    RigidBody *body = rigid_body::create(*f.world, {.shape = shape, .object_layer = f.moving, .motion_type = motion_type::MOTION_KINEMATIC});
    CHECK(ctx, near_point(body->center_of_mass_position, 1.0f, 0.0f, 0.0f));

    // to (1, 2, 3), a quarter turn about y, in half a second
    const float dt = 0.5f;
    const Float3 target_position = {1.0f, 2.0f, 3.0f};
    const Float4 target_quaternion = stored_axis_angle(vec3f::make(0.0f, 1.0f, 0.0f), scalar::PI * 0.5f);
    rigid_body::move_kinematic(body, vec3f::load(target_position), quatf::load(target_quaternion), dt);
    // the offset (1, 0, 0) turned a quarter about y is (0, 0, -1): the com goes (1, 0, 0) -> (1, 2, 2)
    CHECK(ctx, near_point(body->motion_properties.linear_velocity, 0.0f, 4.0f, 4.0f, 1e-4f));
    CHECK(ctx, near_point(body->motion_properties.angular_velocity, 0.0f, scalar::PI, 0.0f, 1e-4f));

    // stepping those velocities over dt lands on the target
    const Vec3f linear_step = vec3f::scale(vec3f::load(body->motion_properties.linear_velocity), dt);
    const Vec3f angular_step = vec3f::scale(vec3f::load(body->motion_properties.angular_velocity), dt);
    rigid_body_step::add_position_step(body, linear_step);
    rigid_body_step::add_rotation_step(body, angular_step);
    rigid_body_step::derive_transform(body);
    CHECK(ctx, vec3f::distance(vec3f::load(body->position), vec3f::load(target_position)) < 1e-4f);
    CHECK(ctx, scalar::abs(quatf::dot(quatf::load(body->quaternion), quatf::load(target_quaternion))) > 1.0f - 1e-5f);

    release_fixture(f);
    box::destroy(shape);
}

void test_rigid_body_inverse_inertia_outside_the_step(TestCtx &ctx) {
    // the in-step setup, without the step: a rotated 1 x 2 x 4 box with rotation about y locked
    Fixture f = make_fixture();
    shapes::Shape *shape = box::create({.half_extents = {0.5f, 1.0f, 2.0f}});
    const Float4 rotation = stored_axis_angle(vec3f::make(0.3f, 0.8f, -0.5f), 0.9f);
    RigidBody *body = rigid_body::create(*f.world,
                                         {.shape = shape, .object_layer = f.moving, .motion_type = motion_type::MOTION_DYNAMIC, .position = {0.0f, 10.0f, 0.0f},
                                          .quaternion = rotation, .allowed_degrees_of_freedom = dof::DOF_ALL & ~dof::ROTATION_Y,
                                          .angular_damping = 0.0f});
    Float4x4 inverse_inertia;
    rigid_body::get_inverse_inertia(inverse_inertia, body);
    Float4x4 rotation_matrix;
    mat4f::store(rotation_matrix, mat4f::from_quat(quatf::load(rotation)));
    Float4x4 fresh;
    mat4f::store(fresh, motion_properties::get_inverse_inertia_for_rotation(body->motion_properties, mat4f::load(rotation_matrix)));
    CHECK(ctx, same_bits(mat4f::load(inverse_inertia), mat4f::load(fresh)));
    // a torque gets no response about the locked world y axis
    const Vec3f response = mat4f::multiply3x3_vec(mat4f::load(inverse_inertia), vec3f::make(3.0f, -2.0f, 5.0f));
    CHECK(ctx, vec3f::y(response) == 0.0f && (vec3f::x(response) != 0.0f || vec3f::z(response) != 0.0f));
    // the public getter does not touch the memo
    CHECK(ctx, body->motion_properties.world_inverse_inertia_stamp == motion_properties::STEP_STAMP_NONE);

    // a rotation written between steps resets the memo; a position does not need to
    body->motion_properties.world_inverse_inertia_stamp = 3;
    rigid_body::set_position(*f.world, body, vec3f::make(0.0f, 0.0f, 0.0f), false);
    CHECK(ctx, body->motion_properties.world_inverse_inertia_stamp == 3);
    rigid_body::set_quaternion(*f.world, body, axis_angle(vec3f::make(0.0f, 1.0f, 0.0f), scalar::PI * 0.5f), true);
    CHECK(ctx, body->motion_properties.world_inverse_inertia_stamp == motion_properties::STEP_STAMP_NONE);
    body->motion_properties.world_inverse_inertia_stamp = 4;
    rigid_body::set_transform(*f.world, body, vec3f::make(0.0f, 1.0f, 0.0f), quatf::load(rotation), false);
    CHECK(ctx, body->motion_properties.world_inverse_inertia_stamp == motion_properties::STEP_STAMP_NONE);

    release_fixture(f);
    box::destroy(shape);
}

void test_rigid_body_center_of_mass_and_surface_normal(TestCtx &ctx) {
    Fixture f = make_fixture();
    shapes::Shape *shape = box::create({.half_extents = {1.0f, 1.0f, 1.0f}});
    shape->center_of_mass = {2.0f, 0.0f, 0.0f};
    // a quarter turn about z at (5, 10, 0): the com offset (2, 0, 0) turns to (0, 2, 0)
    const Float4 turned = stored_axis_angle(vec3f::make(0.0f, 0.0f, 1.0f), scalar::PI * 0.5f);
    RigidBody *body = rigid_body::create(*f.world,
                                         {.shape = shape, .object_layer = f.moving, .motion_type = motion_type::MOTION_DYNAMIC, .position = {5.0f, 10.0f, 0.0f},
                                          .quaternion = turned});
    CHECK(ctx, near_point(body->center_of_mass_position, 5.0f, 12.0f, 0.0f));
    body->position = {0.0f, 0.0f, 0.0f};
    rigid_body::update_center_of_mass_position(body);
    CHECK(ctx, near_point(body->center_of_mass_position, 0.0f, 2.0f, 0.0f));

    // the local +x face turned onto world +y: its normal comes back in world space
    Float3 normal;
    Float3 &returned = rigid_body::get_surface_normal(normal, body, {0.0f, 1.0f, 0.0f}, sub_shape::EMPTY_SUB_SHAPE_ID);
    CHECK(ctx, &returned == &normal);
    CHECK(ctx, near_point(normal, 0.0f, 1.0f, 0.0f));
    // and relative to the shape origin, not the centre of mass
    rigid_body::set_position(*f.world, body, vec3f::make(5.0f, 10.0f, 0.0f), false);
    rigid_body::get_surface_normal(normal, body, {5.0f, 9.0f, 0.0f}, sub_shape::EMPTY_SUB_SHAPE_ID);
    CHECK(ctx, near_point(normal, 0.0f, -1.0f, 0.0f));

    // a leaf shape is itself; the sub-shape transform starts from the one given
    shapes::GetLeafShapeResult leaf = rigid_body::create_get_leaf_shape_result();
    CHECK(ctx, leaf.shape == nullptr && leaf.remainder == sub_shape::EMPTY_SUB_SHAPE_ID);
    rigid_body::get_leaf_shape(leaf, shape, sub_shape::EMPTY_SUB_SHAPE_ID);
    CHECK(ctx, leaf.shape == shape && leaf.remainder == sub_shape::EMPTY_SUB_SHAPE_ID);
    shapes::GetSubShapeTransformedShapeResult transformed = rigid_body::create_get_sub_shape_transformed_shape_result();
    rigid_body::get_sub_shape_transformed_shape(transformed, shape, sub_shape::EMPTY_SUB_SHAPE_ID, vec3f::make(1.0f, 2.0f, 3.0f), quatf::load(turned), vec3f::make(2.0f, 2.0f, 2.0f));
    CHECK(ctx, transformed.shape == shape && near_point(transformed.position, 1.0f, 2.0f, 3.0f) && near_point(transformed.scale, 2.0f, 2.0f, 2.0f));
    CHECK(ctx, same_bits(transformed.rotation, quatf::load(turned)));

    release_fixture(f);
    box::destroy(shape);
}

void test_rigid_body_bodies_share_constraint(TestCtx &ctx) {
    RigidBody a = {};
    RigidBody b = {};
    // no constraints on either: not connected
    CHECK(ctx, !rigid_body::bodies_share_constraint(&a, &b));
    const constraint_id::ConstraintId hinge = constraint_id::ser(0, constraint_id::HINGE, 1);
    const constraint_id::ConstraintId point = constraint_id::ser(0, constraint_id::POINT, 1);
    const constraint_id::ConstraintId fixed = constraint_id::ser(4, constraint_id::FIXED, 2);
    a.constraint_ids.push(hinge);
    CHECK(ctx, !rigid_body::bodies_share_constraint(&a, &b));
    // same index, different type: a different constraint
    b.constraint_ids.push(point);
    b.constraint_ids.push(fixed);
    CHECK(ctx, !rigid_body::bodies_share_constraint(&a, &b) && !rigid_body::bodies_share_constraint(&b, &a));
    // a shared id connects them, whichever list is shorter
    b.constraint_ids.push(hinge);
    CHECK(ctx, rigid_body::bodies_share_constraint(&a, &b) && rigid_body::bodies_share_constraint(&b, &a));
    a.constraint_ids.release();
    b.constraint_ids.release();
}

void test_rigid_body_filter_from_body(TestCtx &ctx) {
    Fixture f = make_fixture();
    // a second broadphase layer with a static layer the moving layer collides with, and a third it does not
    const uint32_t broadphase_static = layers::add_broadphase_layer(f.world->settings.layers);
    const uint32_t still = layers::add_object_layer(f.world->settings.layers, broadphase_static);
    const uint32_t ghost = layers::add_object_layer(f.world->settings.layers, broadphase_static);
    layers::enable_collision(f.world->settings.layers, f.moving, still);
    RigidBody body = {};
    body.object_layer = f.moving;
    body.collision_groups = 0b0110;
    body.collision_mask = 0b0011;
    filter::Filter query = filter::create(f.world->settings.layers);
    filter::set_from_body(query, f.world->settings.layers, &body);
    CHECK(ctx, filter::filter_object_layer(query, f.moving) && filter::filter_object_layer(query, still));
    CHECK(ctx, !filter::filter_object_layer(query, ghost));
    CHECK(ctx, filter::filter_broadphase_layer(query, 0) && filter::filter_broadphase_layer(query, broadphase_static));
    CHECK(ctx, query.collision_groups == 0b0110 && query.collision_mask == 0b0011);
    filter::release(query);
    release_fixture(f);
}
