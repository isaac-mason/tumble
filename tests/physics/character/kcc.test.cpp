#include "tumble/physics/character/kcc.h"

#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/query.h"
#include "tumble/physics/shapes/triangle_mesh.h"
#include "tumble/physics/shapes/utils/triangle_mesh_builder.h"
#include "../world_test_scene.h"
#include "test_math.h"

using rigid_body::RigidBody;

namespace {

constexpr float DT = 1.0f / 60.0f;
const Float3 GRAVITY = {0.0f, -10.0f, 0.0f};
const Float4 IDENTITY = {0.0f, 0.0f, 0.0f, 1.0f};

RigidBody *add_static_box(WorldScene &scene, const Float3 &half_extents, const Float3 &position, const Float4 &quaternion = IDENTITY) {
    rigid_body::RigidBodySettings settings =
        body_settings(box::create({.half_extents = half_extents}), scene.not_moving, motion_type::MOTION_STATIC, position);
    settings.quaternion = quaternion;
    return rigid_body::create(*scene.world, settings);
}

RigidBody *add_floor(WorldScene &scene) { return add_static_box(scene, {50.0f, 0.5f, 50.0f}, {0.0f, -0.5f, 0.0f}); }

/** a capsule character (half height 0.5, radius 0.3) at its centre */
kcc::KCC *make_capsule_character(const Float3 &position, kcc::BackFaceMode back_face_mode = kcc::BACK_FACE_COLLIDE) {
    kcc::KCCSettings settings;
    settings.shape = capsule::create({.half_height_of_cylinder = 0.5f, .radius = 0.3f});
    settings.back_face_mode = back_face_mode;
    return kcc::create(settings, vec3f::load(position), quatf::load(IDENTITY));
}

/** a capsule character whose position is its feet: the shape sits 0.8 above it */
kcc::KCC *make_footed_character(const Float3 &position, float max_slope_angle = 50.0f * scalar::DEGREES_TO_RADIANS) {
    kcc::KCCSettings settings;
    settings.shape = capsule::create({.half_height_of_cylinder = 0.5f, .radius = 0.3f});
    settings.shape_offset = {0.0f, 0.8f, 0.0f};
    settings.max_slope_angle = max_slope_angle;
    return kcc::create(settings, vec3f::load(position), quatf::load(IDENTITY));
}

kcc::UpdateSettings no_stairs_no_stick() {
    kcc::UpdateSettings settings;
    settings.stick_to_floor_step_down = {0.0f, 0.0f, 0.0f};
    settings.walk_stairs_step_up = {0.0f, 0.0f, 0.0f};
    return settings;
}

/** what the character reported, by body id */
struct ContactLog {
    body_id::BodyId added[64];
    uint32_t added_count;
    body_id::BodyId persisted[64];
    uint32_t persisted_count;
    body_id::BodyId removed[64];
    uint32_t removed_count;
};

bool log_has(const body_id::BodyId *ids, uint32_t count, body_id::BodyId id) {
    for (uint32_t i = 0; i < count; i++) {
        if (ids[i] == id) return true;
    }
    return false;
}

/** the events of the character's last update into the log */
void log_events(ContactLog &log, const kcc::KCC &character) {
    for (uint32_t i = 0; i < character.contact_events.length; i++) {
        const kcc::ContactEvent &event = character.contact_events[i];
        if (event.kind == kcc::CONTACT_ADDED && log.added_count < 64) log.added[log.added_count++] = event.body_id;
        if (event.kind == kcc::CONTACT_PERSISTED && log.persisted_count < 64) log.persisted[log.persisted_count++] = event.body_id;
        if (event.kind == kcc::CONTACT_REMOVED && log.removed_count < 64) log.removed[log.removed_count++] = event.body_id;
    }
}

/** gravity into the velocity, the character, then the world */
void step_falling(world::World &world, kcc::KCC &character, const kcc::UpdateSettings &settings, const filter::Filter &filter,
                  ContactLog *log = nullptr) {
    character.linear_velocity = vec3f::scale_and_add(character.linear_velocity, vec3f::load(GRAVITY), DT);
    kcc::update(world, character, DT, vec3f::load(GRAVITY), settings, nullptr, filter);
    if (log != nullptr) log_events(*log, character);
    step_world(world, DT);
}

/**
 * a character game loop: on ground the velocity starts from the ground's, otherwise from the vertical part
 * of the last; then gravity and the horizontal input
 */
void step_walking(world::World &world, kcc::KCC &character, const Float3 &input, const kcc::UpdateSettings &settings,
                  const filter::Filter &filter, ContactLog *log = nullptr) {
    Vec3f velocity = character.ground.state == kcc::ON_GROUND
                         ? character.ground.velocity
                         : vec3f::scale(character.up, vec3f::dot(character.linear_velocity, character.up));
    velocity = vec3f::scale_and_add(velocity, vec3f::load(GRAVITY), DT);
    character.linear_velocity = vec3f::add(velocity, vec3f::load(input));
    kcc::update(world, character, DT, vec3f::load(GRAVITY), settings, nullptr, filter);
    if (log != nullptr) log_events(*log, character);
    step_world(world, DT);
}

uint32_t contacts_with(const kcc::KCC &character, body_id::BodyId id) {
    uint32_t count = 0;
    for (uint32_t i = 0; i < character.contacts.length; i++) {
        if (character.contacts[i].body_id == id) count++;
    }
    return count;
}

}  // namespace

// a steep slope escape, starting intersecting
void test_kcc_escapes_upward_from_steep_slopes(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    world::World &world = *scene.world;

    add_floor(scene);
    // a box intersecting the character
    add_static_box(scene, {0.5f, 0.5f, 0.5f}, {-0.5f, 0.5f, 0.0f});
    // two 75 degree slopes intersecting the character, steeper than its 50 degree max
    const float slope_angle = 75.0f * scalar::DEGREES_TO_RADIANS;
    add_static_box(scene, {1.0f, 0.1f, 1.0f}, {0.0f, 0.0f, 0.0f}, stored_axis_angle(vec3f::axis_z(), slope_angle));
    add_static_box(scene, {1.0f, 0.1f, 1.0f}, {0.0f, 0.0f, 0.0f}, stored_axis_angle(vec3f::axis_z(), -slope_angle));

    step_world(world, DT);

    kcc::KCC *character = make_capsule_character({0.0f, 0.0f, 0.0f});
    filter::Filter filter = filter::create(world.settings.layers);
    const kcc::UpdateSettings settings = no_stairs_no_stick();

    for (int step = 0; step < 10; step++) step_falling(world, *character, settings, filter);

    // without the fix y stays at 0; the asymmetric box drifts it sideways a little
    CHECK(ctx, vec3f::y(character->position) > 0.05f);
    CHECK(ctx, scalar::abs(vec3f::x(character->position)) < 2.0f);
    CHECK(ctx, scalar::abs(vec3f::z(character->position)) < 2.0f);

    kcc::destroy(character);
    filter::release(filter);
    release_world_scene(scene);
}

// conflicting contacts
void test_kcc_wedged_between_two_bodies_keeps_both_contacts(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    world::World &world = *scene.world;

    add_floor(scene);
    // a corridor narrower than the capsule: inner faces at x = +-0.25, 0.05 penetration per side, past the 1.25 x
    // padding conflict threshold
    RigidBody *wall_a = add_static_box(scene, {0.5f, 1.0f, 1.0f}, {-0.75f, 0.5f, 0.0f});
    RigidBody *wall_b = add_static_box(scene, {0.5f, 1.0f, 1.0f}, {0.75f, 0.5f, 0.0f});

    step_world(world, DT);

    kcc::KCC *character = make_capsule_character({0.0f, 0.0f, 0.0f});
    filter::Filter filter = filter::create(world.settings.layers);
    const kcc::UpdateSettings settings = no_stairs_no_stick();

    // after the first update, still deep in both walls: discarding one would push the character through the other
    step_falling(world, *character, settings, filter);
    CHECK(ctx, kcc::has_collided_with(*character, wall_a->id));
    CHECK(ctx, kcc::has_collided_with(*character, wall_b->id));

    // it settles centred in the corridor
    for (int step = 0; step < 5; step++) step_falling(world, *character, settings, filter);
    CHECK(ctx, scalar::abs(vec3f::x(character->position)) < 0.1f);

    kcc::destroy(character);
    filter::release(filter);
    release_world_scene(scene);
}

// contact identity in the events
void test_kcc_recycled_body_slot_is_a_new_contact(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    world::World &world = *scene.world;

    RigidBody *floor_a = add_floor(scene);
    step_world(world, DT);

    kcc::KCC *character = make_capsule_character({0.0f, 0.01f, 0.0f});
    filter::Filter filter = filter::create(world.settings.layers);
    const kcc::UpdateSettings settings;
    ContactLog log = {};

    // settle onto floor a: added once, then persisted
    for (int i = 0; i < 5; i++) step_falling(world, *character, settings, filter, &log);
    CHECK(ctx, log_has(log.added, log.added_count, floor_a->id));
    CHECK(ctx, log_has(log.persisted, log.persisted_count, floor_a->id));

    // floor a goes and floor b takes its slot: same index, a new sequence
    const body_id::BodyId floor_a_id = floor_a->id;
    CHECK(ctx, rigid_body::remove(world, floor_a));
    RigidBody *floor_b = add_floor(scene);
    CHECK(ctx, floor_b->index == body_id::index(floor_a_id));
    CHECK(ctx, floor_b->id != floor_a_id);

    log = {};
    for (int i = 0; i < 3; i++) step_falling(world, *character, settings, filter, &log);

    // the recycled slot is a new contact, and the old id never persists
    CHECK(ctx, log_has(log.added, log.added_count, floor_b->id));
    CHECK(ctx, !log_has(log.persisted, log.persisted_count, floor_a_id));
    CHECK(ctx, log_has(log.removed, log.removed_count, floor_a_id));

    kcc::destroy(character);
    filter::release(filter);
    release_world_scene(scene);
}

// a contact removed for a destroyed body
void test_kcc_destroying_the_ground_reports_its_removal(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    world::World &world = *scene.world;

    RigidBody *floor = add_floor(scene);
    step_world(world, DT);

    kcc::KCC *character = make_capsule_character({0.0f, 0.01f, 0.0f});
    filter::Filter filter = filter::create(world.settings.layers);
    const kcc::UpdateSettings settings;
    ContactLog log = {};

    for (int i = 0; i < 5; i++) step_falling(world, *character, settings, filter, &log);
    CHECK(ctx, log.removed_count == 0);
    CHECK(ctx, character->ground.state == kcc::ON_GROUND);
    CHECK(ctx, character->ground.body_id == floor->id);

    const body_id::BodyId floor_id = floor->id;
    CHECK(ctx, rigid_body::remove(world, floor));
    step_falling(world, *character, settings, filter, &log);
    CHECK(ctx, log_has(log.removed, log.removed_count, floor_id));
    CHECK(ctx, character->ground.state == kcc::IN_AIR);

    kcc::destroy(character);
    filter::release(filter);
    release_world_scene(scene);
}

namespace {

/** contacts with a quad whose front faces down, the capsule resting 0.05 into it from above (on its back face) */
uint32_t contacts_with_down_facing_quad(TestCtx &ctx, kcc::BackFaceMode back_face_mode) {
    WorldScene scene = make_world_scene(ctx);
    world::World &world = *scene.world;

    const float positions[] = {-2.0f, 0.0f, -2.0f, 2.0f, 0.0f, -2.0f, 2.0f, 0.0f, 2.0f, -2.0f, 0.0f, 2.0f};
    const uint32_t indices[] = {0, 1, 2, 0, 2, 3};
    triangle_mesh_builder::TriangleMeshBuilderSettings mesh;
    mesh.positions = positions;
    mesh.num_positions = 4;
    mesh.indices = indices;
    mesh.num_indices = 6;
    RigidBody *quad = add_body(scene, triangle_mesh::create({.data = triangle_mesh_builder::build(mesh)}), motion_type::MOTION_STATIC, {0.0f, 0.0f, 0.0f});

    step_world(world, DT);

    kcc::KCC *character = make_capsule_character({0.0f, 0.75f, 0.0f}, back_face_mode);
    filter::Filter filter = filter::create(world.settings.layers);
    kcc::refresh_contacts(world, *character, filter, nullptr);
    const uint32_t count = contacts_with(*character, quad->id);

    kcc::destroy(character);
    filter::release(filter);
    release_world_scene(scene);
    return count;
}

}  // namespace

// back face mode
void test_kcc_back_face_collide_finds_the_back_face(TestCtx &ctx) { CHECK(ctx, contacts_with_down_facing_quad(ctx, kcc::BACK_FACE_COLLIDE) > 0); }

void test_kcc_back_face_ignore_does_not(TestCtx &ctx) { CHECK(ctx, contacts_with_down_facing_quad(ctx, kcc::BACK_FACE_IGNORE) == 0); }

void test_kcc_defaults_and_slope_test(TestCtx &ctx) {
    kcc::KCCSettings settings;
    settings.shape = capsule::create({.half_height_of_cylinder = 0.5f, .radius = 0.3f});
    settings.up = {0.0f, 2.0f, 0.0f};
    kcc::KCC *character = kcc::create(settings, vec3f::make(1.0f, 2.0f, 3.0f), quatf::load(IDENTITY));

    // up normalised, the supporting plane below everything, in the air
    CHECK_NEAR(ctx, vec3f::y(character->up), 1.0f, 1e-6f);
    CHECK_NEAR(ctx, vec4f::y(character->supporting_volume_plane), 1.0f, 1e-6f);
    CHECK(ctx, vec4f::w(character->supporting_volume_plane) == -1.0e10f);
    CHECK_NEAR(ctx, character->cos_max_slope_angle, 0.6427876f, 1e-6f);
    CHECK(ctx, character->ground.state == kcc::IN_AIR);
    CHECK(ctx, !kcc::is_supported(*character));
    CHECK(ctx, character->inner_rigid_body_id == body_id::INVALID_BODY_ID);

    // 40 degrees is walkable, 60 is not
    const float walkable = 40.0f * scalar::DEGREES_TO_RADIANS;
    const float steep = 60.0f * scalar::DEGREES_TO_RADIANS;
    CHECK(ctx, !kcc::is_slope_too_steep(*character, vec3f::make(scalar::sin(walkable), scalar::cos(walkable), 0.0f)));
    CHECK(ctx, kcc::is_slope_too_steep(*character, vec3f::make(scalar::sin(steep), scalar::cos(steep), 0.0f)));

    // a max slope angle near 0 turns the test off
    character->cos_max_slope_angle = 1.0f;
    CHECK(ctx, !kcc::is_slope_too_steep(*character, vec3f::make(1.0f, 0.0f, 0.0f)));

    // the shape transform carries the offset rotated by the character
    character->shape_offset = vec3f::make(0.0f, 0.0f, 1.0f);
    const Float3 y_axis = {0.0f, 1.0f, 0.0f};
    character->quaternion = quatf::set_axis_angle(vec3f::load(y_axis), 0.5f * scalar::PI);
    Vec3f com_position;
    Quatf com_quaternion;
    kcc::get_center_of_mass_transform(*character, com_position, com_quaternion);
    const Vec3f position = com_position;
    const Quatf quaternion = com_quaternion;
    CHECK_NEAR(ctx, vec3f::x(position), 2.0f, 1e-5f);
    CHECK_NEAR(ctx, vec3f::y(position), 2.0f, 1e-5f);
    CHECK_NEAR(ctx, vec3f::z(position), 3.0f, 1e-5f);
    CHECK_NEAR(ctx, quatf::y(quaternion), quatf::y(character->quaternion), 1e-7f);

    kcc::destroy(character);
}

void test_kcc_falls_and_lands_on_the_floor(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    world::World &world = *scene.world;
    RigidBody *floor = add_floor(scene);
    step_world(world, DT);

    kcc::KCC *character = make_footed_character({0.0f, 1.0f, 0.0f});
    filter::Filter filter = filter::create(world.settings.layers);
    const kcc::UpdateSettings settings;

    for (int i = 0; i < 90; i++) step_walking(world, *character, {0.0f, 0.0f, 0.0f}, settings, filter);

    // resting on the floor at the padding, supported by it with an upward normal
    CHECK(ctx, character->ground.state == kcc::ON_GROUND);
    CHECK(ctx, character->ground.body_id == floor->id);
    CHECK_NEAR(ctx, vec3f::y(character->ground.normal), 1.0f, 1e-3f);
    CHECK_NEAR(ctx, vec3f::y(character->position), 0.0f, 0.03f);
    CHECK(ctx, vec3f::y(character->position) >= -1e-3f);
    CHECK(ctx, kcc::has_collided_with(*character, floor->id));

    kcc::destroy(character);
    filter::release(filter);
    release_world_scene(scene);
}

void test_kcc_walking_into_a_wall_slides_along_it(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    world::World &world = *scene.world;
    add_floor(scene);
    // the wall's face at x = 1
    RigidBody *wall = add_static_box(scene, {0.5f, 2.0f, 20.0f}, {1.5f, 2.0f, 0.0f});
    step_world(world, DT);

    kcc::KCC *character = make_footed_character({0.0f, 0.0f, 0.0f});
    filter::Filter filter = filter::create(world.settings.layers);
    const kcc::UpdateSettings settings;

    // diagonally into the wall: x stops at the wall, z keeps going at the input speed
    for (int i = 0; i < 60; i++) step_walking(world, *character, {2.0f, 0.0f, 2.0f}, settings, filter);

    CHECK(ctx, kcc::has_collided_with(*character, wall->id));
    CHECK(ctx, vec3f::x(character->position) < 1.0f - 0.3f + 1e-3f);
    CHECK(ctx, vec3f::x(character->position) > 1.0f - 0.3f - 0.05f);
    CHECK(ctx, vec3f::z(character->position) > 1.8f);
    CHECK(ctx, vec3f::z(character->position) < 2.05f);
    CHECK(ctx, character->ground.state == kcc::ON_GROUND);
    CHECK_NEAR(ctx, vec3f::y(character->position), 0.0f, 0.03f);

    kcc::destroy(character);
    filter::release(filter);
    release_world_scene(scene);
}

namespace {

/** walks +x for a second toward a step at x = 1 of the given height; the character's final position */
Vec3f walk_into_step(TestCtx &ctx, float step_height, kcc::GroundState &out_state) {
    WorldScene scene = make_world_scene(ctx);
    world::World &world = *scene.world;
    add_floor(scene);
    add_static_box(scene, {2.0f, 0.5f * step_height, 2.0f}, {3.0f, 0.5f * step_height, 0.0f});
    step_world(world, DT);

    kcc::KCC *character = make_footed_character({0.0f, 0.0f, 0.0f});
    filter::Filter filter = filter::create(world.settings.layers);
    const kcc::UpdateSettings settings;

    for (int i = 0; i < 60; i++) step_walking(world, *character, {2.0f, 0.0f, 0.0f}, settings, filter);

    const Vec3f position = character->position;
    out_state = character->ground.state;
    kcc::destroy(character);
    filter::release(filter);
    release_world_scene(scene);
    return position;
}

}  // namespace

void test_kcc_climbs_a_step_up_to_its_height_and_not_taller(TestCtx &ctx) {
    // the default step up is 0.4: a 0.3 step is climbed and walked across
    kcc::GroundState state;
    const Vec3f climbed = walk_into_step(ctx, 0.3f, state);
    CHECK_NEAR(ctx, vec3f::y(climbed), 0.3f, 0.03f);
    CHECK(ctx, vec3f::x(climbed) > 1.5f);
    CHECK(ctx, state == kcc::ON_GROUND);

    // a 0.6 step is a wall
    const Vec3f blocked = walk_into_step(ctx, 0.6f, state);
    CHECK_NEAR(ctx, vec3f::y(blocked), 0.0f, 0.03f);
    CHECK(ctx, vec3f::x(blocked) < 1.0f - 0.3f + 1e-3f);
    CHECK(ctx, vec3f::x(blocked) > 1.0f - 0.3f - 0.05f);
}

namespace {

/** a slope rising toward +x at the angle, its surface through the origin */
void add_slope(WorldScene &scene, float angle) {
    const Float4 rotation = stored_axis_angle(vec3f::axis_z(), angle);
    // the box's top face centre at the origin: its centre half a thickness below along the rotated normal
    const Float3 normal = {-scalar::sin(angle), scalar::cos(angle), 0.0f};
    add_static_box(scene, {10.0f, 0.5f, 10.0f}, {-0.5f * normal.x, -0.5f * normal.y, 0.0f}, rotation);
}

}  // namespace

void test_kcc_walks_up_a_walkable_slope(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    world::World &world = *scene.world;
    const float angle = 30.0f * scalar::DEGREES_TO_RADIANS;
    add_slope(scene, angle);
    step_world(world, DT);

    kcc::KCC *character = make_footed_character({0.0f, 0.05f, 0.0f});
    filter::Filter filter = filter::create(world.settings.layers);
    const kcc::UpdateSettings settings;

    // settle, then walk uphill
    for (int i = 0; i < 20; i++) step_walking(world, *character, {0.0f, 0.0f, 0.0f}, settings, filter);
    CHECK(ctx, character->ground.state == kcc::ON_GROUND);
    CHECK_NEAR(ctx, vec3f::x(character->ground.normal), -scalar::sin(angle), 1e-3f);
    const Vec3f start = character->position;

    for (int i = 0; i < 60; i++) {
        step_walking(world, *character, {2.0f, 0.0f, 0.0f}, settings, filter);
        if (character->ground.state != kcc::ON_GROUND) break;
    }

    // a second's walk gets well up the slope and stays on it
    CHECK(ctx, character->ground.state == kcc::ON_GROUND);
    CHECK(ctx, vec3f::x(character->position) - vec3f::x(start) > 1.0f);
    const float tan_angle = scalar::sin(angle) / scalar::cos(angle);
    CHECK_NEAR(ctx, vec3f::y(character->position) - vec3f::y(start), (vec3f::x(character->position) - vec3f::x(start)) * tan_angle, 0.05f);

    kcc::destroy(character);
    filter::release(filter);
    release_world_scene(scene);
}

void test_kcc_slides_down_a_slope_too_steep(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    world::World &world = *scene.world;
    const float angle = 60.0f * scalar::DEGREES_TO_RADIANS;
    add_slope(scene, angle);
    step_world(world, DT);

    kcc::KCC *character = make_footed_character({2.0f, 2.0f * 1.7320508f + 0.05f, 0.0f});
    filter::Filter filter = filter::create(world.settings.layers);
    const kcc::UpdateSettings settings;

    for (int i = 0; i < 10; i++) step_walking(world, *character, {0.0f, 0.0f, 0.0f}, settings, filter);
    CHECK(ctx, character->ground.state == kcc::ON_STEEP_GROUND);
    const Vec3f start = character->position;

    // trying to walk up it: it slides down regardless, along the slope
    for (int i = 0; i < 30; i++) step_walking(world, *character, {1.0f, 0.0f, 0.0f}, settings, filter);
    CHECK(ctx, character->ground.state == kcc::ON_STEEP_GROUND);
    CHECK(ctx, vec3f::y(character->position) < vec3f::y(start) - 0.2f);
    CHECK(ctx, vec3f::x(character->position) < vec3f::x(start));

    kcc::destroy(character);
    filter::release(filter);
    release_world_scene(scene);
}

void test_kcc_rides_a_moving_kinematic_platform(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    world::World &world = *scene.world;
    add_floor(scene);
    RigidBody *platform = add_body(scene, box::create({.half_extents = {2.0f, 0.25f, 2.0f}}), motion_type::MOTION_KINEMATIC, {0.0f, 1.0f, 0.0f});
    rigid_body::set_linear_velocity(world, platform, vec3f::make(1.0f, 0.0f, 0.0f));
    step_world(world, DT);

    kcc::KCC *character = make_footed_character({0.0f, 1.3f, 0.0f});
    filter::Filter filter = filter::create(world.settings.layers);
    const kcc::UpdateSettings settings;

    for (int i = 0; i < 10; i++) step_walking(world, *character, {0.0f, 0.0f, 0.0f}, settings, filter);
    CHECK(ctx, character->ground.state == kcc::ON_GROUND);
    CHECK(ctx, character->ground.body_id == platform->id);
    CHECK_NEAR(ctx, vec3f::x(character->ground.velocity), 1.0f, 1e-3f);

    const float platform_start = platform->position.x;
    const float character_start = vec3f::x(character->position);
    for (int i = 0; i < 60; i++) step_walking(world, *character, {0.0f, 0.0f, 0.0f}, settings, filter);

    // it moved with the platform, standing on its top
    CHECK_NEAR(ctx, platform->position.x - platform_start, 1.0f, 1e-3f);
    CHECK_NEAR(ctx, vec3f::x(character->position) - character_start, platform->position.x - platform_start, 0.02f);
    CHECK_NEAR(ctx, vec3f::y(character->position), 1.25f, 0.03f);
    CHECK(ctx, character->ground.state == kcc::ON_GROUND);

    kcc::destroy(character);
    filter::release(filter);
    release_world_scene(scene);
}

void test_kcc_reports_a_sensor_and_walks_through_it(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    world::World &world = *scene.world;
    add_floor(scene);
    rigid_body::RigidBodySettings sensor_settings =
        body_settings(box::create({.half_extents = {0.5f, 1.0f, 2.0f}}), scene.not_moving, motion_type::MOTION_STATIC, {1.5f, 1.0f, 0.0f});
    sensor_settings.sensor = true;
    RigidBody *sensor = rigid_body::create(world, sensor_settings);
    step_world(world, DT);

    kcc::KCC *character = make_footed_character({0.0f, 0.0f, 0.0f});
    filter::Filter filter = filter::create(world.settings.layers);
    const kcc::UpdateSettings settings;
    ContactLog log = {};

    // 3 m at 2 m/s: into the sensor at x = 1 and out past its far side at x = 2
    bool saw_sensor_contact = false;
    for (int i = 0; i < 90; i++) {
        step_walking(world, *character, {2.0f, 0.0f, 0.0f}, settings, filter, &log);
        if (contacts_with(*character, sensor->id) > 0) saw_sensor_contact = true;
    }

    // reported in the events and the contacts, but it never blocked the walk
    CHECK(ctx, saw_sensor_contact);
    CHECK(ctx, log_has(log.added, log.added_count, sensor->id));
    CHECK(ctx, log_has(log.removed, log.removed_count, sensor->id));
    CHECK(ctx, vec3f::x(character->position) > 2.9f);
    CHECK_NEAR(ctx, vec3f::y(character->position), 0.0f, 0.03f);

    kcc::destroy(character);
    filter::release(filter);
    release_world_scene(scene);
}

void test_kcc_pushes_a_dynamic_box(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    world::World &world = *scene.world;
    add_floor(scene);
    // 10 kg: the character's 100 N strength moves it against friction
    RigidBody *crate =
        add_body(scene, box::create({.half_extents = {0.5f, 0.5f, 0.5f}, .density = 10.0f}), motion_type::MOTION_DYNAMIC, {1.5f, 0.5f, 0.0f});
    for (int i = 0; i < 30; i++) step_world(world, DT);
    const float crate_start = crate->position.x;

    kcc::KCC *character = make_footed_character({0.0f, 0.0f, 0.0f});
    filter::Filter filter = filter::create(world.settings.layers);
    const kcc::UpdateSettings settings;

    bool collided = false;
    for (int i = 0; i < 60; i++) {
        step_walking(world, *character, {2.0f, 0.0f, 0.0f}, settings, filter);
        if (kcc::has_collided_with(*character, crate->id)) collided = true;
    }

    // the character's impulses moved the box along +x, and the character followed it
    CHECK(ctx, collided);
    CHECK(ctx, crate->position.x > crate_start + 0.1f);
    CHECK(ctx, vec3f::x(character->position) > 0.8f);

    kcc::destroy(character);
    filter::release(filter);
    release_world_scene(scene);
}

void test_kcc_inner_body_follows_and_is_ignored_by_its_own_queries(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    world::World &world = *scene.world;
    add_floor(scene);
    step_world(world, DT);

    kcc::KCCSettings character_settings;
    character_settings.shape = capsule::create({.half_height_of_cylinder = 0.5f, .radius = 0.3f});
    character_settings.shape_offset = {0.0f, 0.8f, 0.0f};
    character_settings.user_data = 42;
    character_settings.inner_rigid_body_shape = character_settings.shape;
    character_settings.inner_rigid_body_object_layer = scene.moving;
    kcc::KCC *character = kcc::create(character_settings, vec3f::make(0.0f, 0.0f, 0.0f), quatf::load(IDENTITY));

    kcc::add(world, *character);
    RigidBody *inner = rigid_body::get(world, character->inner_rigid_body_id);
    CHECK(ctx, inner != nullptr);
    if (inner == nullptr) return;
    CHECK(ctx, inner->motion_type == motion_type::MOTION_KINEMATIC);
    CHECK(ctx, inner->user_data == 42);
    // add again is a no-op
    const body_id::BodyId inner_id = character->inner_rigid_body_id;
    kcc::add(world, *character);
    CHECK(ctx, character->inner_rigid_body_id == inner_id);

    filter::Filter filter = filter::create(world.settings.layers);
    const kcc::UpdateSettings settings;
    for (int i = 0; i < 60; i++) step_walking(world, *character, {1.0f, 0.0f, 0.0f}, settings, filter);

    // walked a metre on the floor, never pushed by its own body, which sits at the shape (offset plus padding)
    CHECK_NEAR(ctx, vec3f::x(character->position), 1.0f, 0.03f);
    CHECK_NEAR(ctx, vec3f::y(character->position), 0.0f, 0.03f);
    CHECK(ctx, !kcc::has_collided_with(*character, inner_id));
    CHECK_NEAR(ctx, inner->position.x, vec3f::x(character->position), 1e-5f);
    CHECK_NEAR(ctx, inner->position.y, vec3f::y(character->position) + 0.8f + character->character_padding, 1e-5f);

    // a ray down onto the character hits the inner body
    cast_ray::CastRayCollector rays = cast_ray::create_collector(collector::COLLECT_CLOSEST);
    const cast_ray::Rayf ray = {vec3f::make(vec3f::x(character->position), 5.0f, 0.0f), vec3f::make(0.0f, -1.0f, 0.0f), 10.0f};
    query::cast_ray(world, rays, cast_ray::default_settings(), ray, filter);
    CHECK(ctx, rays.has_hit && rays.hits[0].body_id_b == inner_id);
    cast_ray::release(rays);

    kcc::remove(world, *character);
    CHECK(ctx, character->inner_rigid_body_id == body_id::INVALID_BODY_ID);
    CHECK(ctx, rigid_body::get(world, inner_id) == nullptr);

    kcc::destroy(character);
    filter::release(filter);
    release_world_scene(scene);
}

void test_kcc_set_shape_refuses_a_shape_that_would_penetrate(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    world::World &world = *scene.world;
    add_floor(scene);
    // a ceiling whose underside is at 1.4: the standing capsule (top at 1.6) does not fit, the crouched one does
    add_static_box(scene, {5.0f, 0.5f, 5.0f}, {0.0f, 1.9f, 0.0f});
    step_world(world, DT);

    kcc::KCCSettings character_settings;
    shapes::Shape *crouched = capsule::create({.half_height_of_cylinder = 0.2f, .radius = 0.3f});
    shapes::Shape *standing = capsule::create({.half_height_of_cylinder = 0.5f, .radius = 0.3f});
    character_settings.shape = crouched;
    character_settings.shape_offset = {0.0f, 0.5f, 0.0f};
    kcc::KCC *character = kcc::create(character_settings, vec3f::make(0.0f, 0.0f, 0.0f), quatf::load(IDENTITY));
    filter::Filter filter = filter::create(world.settings.layers);

    kcc::refresh_contacts(world, *character, filter, nullptr);
    CHECK(ctx, character->ground.state == kcc::ON_GROUND);

    // standing up penetrates the ceiling by 0.3
    CHECK(ctx, !kcc::set_shape(world, *character, standing, filter, nullptr, 0.1f));
    CHECK(ctx, character->shape == crouched);
    // the same shape is always fine, and with no penetration test any shape goes
    CHECK(ctx, kcc::set_shape(world, *character, crouched, filter, nullptr, 0.1f));
    CHECK(ctx, kcc::set_shape(world, *character, standing, filter, nullptr, __FLT_MAX__));
    CHECK(ctx, character->shape == standing);

    kcc::destroy(character);
    filter::release(filter);
    release_world_scene(scene);
}

void test_kcc_is_deterministic(TestCtx &ctx) {
    Float3 positions[2];
    for (int run = 0; run < 2; run++) {
        WorldScene scene = make_world_scene(ctx);
        world::World &world = *scene.world;
        add_floor(scene);
        add_static_box(scene, {0.5f, 2.0f, 20.0f}, {1.5f, 2.0f, 0.0f});
        add_static_box(scene, {2.0f, 0.15f, 2.0f}, {-3.0f, 0.15f, 0.0f});
        step_world(world, DT);

        kcc::KCC *character = make_footed_character({0.0f, 0.5f, 0.0f});
        filter::Filter filter = filter::create(world.settings.layers);
        const kcc::UpdateSettings settings;
        for (int i = 0; i < 120; i++) {
            const Float3 input = i < 60 ? Float3{2.0f, 0.0f, 1.0f} : Float3{-3.0f, 0.0f, -0.5f};
            step_walking(world, *character, input, settings, filter);
        }
        vec3f::store(positions[run], character->position);
        kcc::destroy(character);
        filter::release(filter);
        release_world_scene(scene);
    }
    CHECK(ctx, positions[0].x == positions[1].x && positions[0].y == positions[1].y && positions[0].z == positions[1].z);
}

namespace {

/** a static triangle mesh at the origin from flat positions and indices */
RigidBody *add_static_mesh(WorldScene &scene, const float *positions, uint32_t num_positions, const uint32_t *indices, uint32_t num_indices) {
    triangle_mesh_builder::TriangleMeshBuilderSettings mesh;
    mesh.positions = positions;
    mesh.num_positions = num_positions;
    mesh.indices = indices;
    mesh.num_indices = num_indices;
    return add_body(scene, triangle_mesh::create({.data = triangle_mesh_builder::build(mesh)}), motion_type::MOTION_STATIC, {0.0f, 0.0f, 0.0f});
}

kcc::KCC *make_sphere_character(const Float3 &position, float radius, uint32_t max_num_hits, float character_padding = 0.02f) {
    kcc::KCCSettings settings;
    settings.shape = sphere::create({.radius = radius});
    settings.max_num_hits = max_num_hits;
    settings.character_padding = character_padding;
    return kcc::create(settings, vec3f::load(position), quatf::load(IDENTITY));
}

}  // namespace

// sweep padding correction
void test_kcc_sweep_onto_a_floor_stops_one_padding_from_its_face(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    world::World &world = *scene.world;
    add_static_box(scene, {5.0f, 0.5f, 5.0f}, {0.0f, -0.5f, 0.0f});
    step_world(world, DT);

    kcc::KCC *character = make_sphere_character({0.0f, 1.0f, 0.0f}, 0.5f, 256, 0.1f);
    filter::Filter filter = filter::create(world.settings.layers);
    CHECK(ctx, kcc::stick_to_floor(world, *character, vec3f::make(0.0f, -2.0f, 0.0f), filter, nullptr));
    CHECK_NEAR(ctx, vec3f::y(character->position), 0.5f, 1e-4f);

    kcc::destroy(character);
    filter::release(filter);
    release_world_scene(scene);
}

// the padding correction casts at 1e-4, not the character's 1e-3 tolerance. a sphere coming down 0.3 past the floor's
// edge settles on the edge rounded by the padding, its centre 0.6 from the edge (0.5196 above it), which the cast only
// reaches to within its tolerance
void test_kcc_padding_correction_casts_at_its_own_tolerance(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    world::World &world = *scene.world;
    add_static_box(scene, {5.0f, 0.5f, 5.0f}, {0.0f, -0.5f, 0.0f});
    step_world(world, DT);

    kcc::KCC *character = make_sphere_character({5.3f, 1.0f, 0.0f}, 0.5f, 256, 0.1f);
    filter::Filter filter = filter::create(world.settings.layers);
    CHECK(ctx, kcc::stick_to_floor(world, *character, vec3f::make(0.0f, -2.0f, 0.0f), filter, nullptr));
    // the shape sits one padding above the position
    CHECK_NEAR(ctx, vec3f::y(character->position), scalar::sqrt(0.6f * 0.6f - 0.3f * 0.3f) - 0.1f, 1e-4f);

    kcc::destroy(character);
    filter::release(filter);
    release_world_scene(scene);
}

// hit reduction: two triangles at y = 0 and one at y = 0.05, all under the sphere and all
// facing up. with room for two hits the full collide finds three, and reduction (same body, same normal, different
// triangles) keeps the deepest
void test_kcc_hit_reduction_keeps_the_deepest_contact_on_one_body(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    world::World &world = *scene.world;
    const float positions[] = {-4, 0, -2, 0, 0, 4, 4, 0, -2, -3, 0, -3, 0, 0, 5, 3, 0, -3, -4, 0.05f, -2, 0, 0.05f, 4, 4, 0.05f, -2};
    const uint32_t indices[] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
    RigidBody *mesh = add_static_mesh(scene, positions, 9, indices, 9);
    step_world(world, DT);

    // sphere bottom at 0.03: 0.02 into the upper triangle, 0.03 above the lower two
    kcc::KCC *character = make_sphere_character({0.0f, 0.51f, 0.0f}, 0.5f, 2);
    filter::Filter filter = filter::create(world.settings.layers);
    kcc::refresh_contacts(world, *character, filter, nullptr);

    CHECK(ctx, character->contacts.length == 2);
    float deepest = __FLT_MAX__;
    for (uint32_t i = 0; i < character->contacts.length; i++) {
        CHECK(ctx, character->contacts[i].body_id == mesh->id);
        deepest = scalar::min(deepest, character->contacts[i].distance);
    }
    // penetration 0.02, less the 0.02 padding
    CHECK_NEAR(ctx, deepest, -0.04f, 1e-4f);

    kcc::destroy(character);
    filter::release(filter);
    release_world_scene(scene);
}

// four coplanar triangles with room for three: the fourth hit runs one full reduction pass,
// which merges all three stored contacts into one before the fourth is added
void test_kcc_hit_reduction_makes_a_full_pass(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    world::World &world = *scene.world;
    const float positions[] = {-4, 0, -2, 0, 0, 4, 4, 0, -2, -3, 0, -3, 0, 0, 5, 3, 0, -3, -5, 0, -2, 0, 0, 6, 5, 0, -2, -4, 0, -4, 0, 0, 3, 4, 0, -4};
    const uint32_t indices[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    add_static_mesh(scene, positions, 12, indices, 12);
    step_world(world, DT);

    kcc::KCC *character = make_sphere_character({0.0f, 0.47f, 0.0f}, 0.5f, 3);
    filter::Filter filter = filter::create(world.settings.layers);
    kcc::refresh_contacts(world, *character, filter, nullptr);

    CHECK(ctx, character->contacts.length == 2);

    kcc::destroy(character);
    filter::release(filter);
    release_world_scene(scene);
}

// two triangles of one mesh fill both slots, then a box deeper under the sphere is reached:
// reduction frees a slot by merging the two triangle contacts, so the query must not stop at max hits before it
// gets there
void test_kcc_max_hits_does_not_stop_the_query_while_reduction_can_free_a_slot(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    world::World &world = *scene.world;
    const float positions[] = {-4, 0, -2, 0, 0, 4, 4, 0, -2, -3, 0, -3, 0, 0, 5, 3, 0, -3};
    const uint32_t indices[] = {0, 1, 2, 3, 4, 5};
    RigidBody *mesh = add_static_mesh(scene, positions, 6, indices, 6);
    RigidBody *block = add_static_box(scene, {1.0f, 0.5f, 1.0f}, {0.0f, -0.45f, 0.0f});
    step_world(world, DT);

    // sphere bottom at 0.03: 0.02 into the box, 0.03 above the triangles
    kcc::KCC *character = make_sphere_character({0.0f, 0.51f, 0.0f}, 0.5f, 2);
    filter::Filter filter = filter::create(world.settings.layers);
    kcc::refresh_contacts(world, *character, filter, nullptr);

    CHECK(ctx, character->contacts.length == 2);
    CHECK(ctx, contacts_with(*character, mesh->id) == 1);
    CHECK(ctx, contacts_with(*character, block->id) == 1);
    for (uint32_t i = 0; i < character->contacts.length; i++) {
        if (character->contacts[i].body_id == block->id) CHECK_NEAR(ctx, character->contacts[i].distance, -0.04f, 1e-4f);
    }

    kcc::destroy(character);
    filter::release(filter);
    release_world_scene(scene);
}


namespace {

/** move from the toolkit, the platform's contacts from below passed over: a caller's one-way platform */
void one_way_move(world::World &world, kcc::KCC &character, body_id::BodyId platform, const filter::Filter &filter) {
    Vec3f position = character.position;
    Vec3f velocity = character.linear_velocity;
    const Vec3f direction = vec3f::normalize(velocity);
    float time_remaining = DT;
    for (uint32_t iteration = 0; iteration < character.max_collision_iterations && time_remaining >= character.min_time_remaining; iteration++) {
        kcc::collide(world, character, position, direction, filter, nullptr);
        // the platform from below: its normal toward the character points down
        for (uint32_t i = 0; i < character.contacts.length; i++) {
            kcc::CharacterContact &contact = character.contacts[i];
            if (contact.body_id == platform && vec3f::y(contact.contact_normal) < 0.5f) contact.was_discarded = true;
        }
        Vec3f displacement;
        float time_simulated = kcc::solve(world, character, velocity, DT, time_remaining, nullptr, displacement);
        kcc::CharacterContact hit;
        const bool blocked = kcc::cast(hit, world, character, position, displacement, filter, nullptr) &&
                             !(hit.body_id == platform && vec3f::y(hit.contact_normal) < 0.5f);
        if (blocked) {
            displacement = vec3f::scale(displacement, hit.fraction);
            time_simulated *= hit.fraction;
        }
        position = vec3f::add(position, displacement);
        time_remaining -= time_simulated;
        if (vec3f::squared_length(displacement) < 1e-8f) break;
    }
    kcc::set_position(world, character, position);
    kcc::update_ground(world, character, DT, false, nullptr);
}

}  // namespace

// the toolkit's contract: a contact discarded between collide and solve is passed over by the solve, the sweep and
// the ground. jumped up through a platform, the character lands on it
void test_kcc_toolkit_one_way_platform(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    world::World &world = *scene.world;
    add_floor(scene);
    RigidBody *platform = add_static_box(scene, {2.0f, 0.1f, 2.0f}, {0.0f, 2.0f, 0.0f});
    step_world(world, DT);

    kcc::KCC *character = make_footed_character({0.0f, 0.0f, 0.0f});
    filter::Filter filter = filter::create(world.settings.layers);
    character->linear_velocity = vec3f::make(0.0f, 9.0f, 0.0f);
    float highest = 0.0f;
    for (int i = 0; i < 120; i++) {
        character->linear_velocity = vec3f::scale_and_add(character->linear_velocity, vec3f::load(GRAVITY), DT);
        one_way_move(world, *character, platform->id, filter);
        // landed only coming down: rising out through the platform's top it is under the character, but not yet stood on
        if (character->ground.state == kcc::ON_GROUND && vec3f::y(character->linear_velocity) <= 0.0f) {
            character->linear_velocity = vec3f::make(0.0f, 0.0f, 0.0f);
        }
        highest = scalar::max(highest, vec3f::y(character->position));
        step_world(world, DT);
    }
    // up past the platform's top at 2.1, then down onto it
    CHECK(ctx, highest > 2.5f);
    CHECK(ctx, character->ground.body_id == platform->id);
    CHECK_NEAR(ctx, vec3f::y(character->position), 2.1f, 0.01f);

    kcc::destroy(character);
    filter::release(filter);
    release_world_scene(scene);
}
