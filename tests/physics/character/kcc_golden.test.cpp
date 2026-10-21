// the character's whole behaviour, frame by frame, as hashes: a change to the controller that should not change what
// it does is checked against these. vectors hash their x, y and z, the fourth lane being nothing the results depend on

#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/character/kcc.h"
#include "tumble/physics/shapes/transformed.h"
#include "tumble/physics/shapes/triangle_mesh.h"
#include "tumble/physics/shapes/utils/triangle_mesh_builder.h"
#include "../world_test_scene.h"

using rigid_body::RigidBody;

namespace {

constexpr float DT = 1.0f / 60.0f;
const Float3 GRAVITY = {0.0f, -10.0f, 0.0f};
const Float4 IDENTITY = {0.0f, 0.0f, 0.0f, 1.0f};

struct Hash {
    uint32_t value = 2166136261u;

    void word(uint32_t w) {
        for (int i = 0; i < 4; i++) {
            value ^= (w >> (i * 8)) & 0xffu;
            value *= 16777619u;
        }
    }
    void real(float f) { word(f != f ? 0x7fc00000u : __builtin_bit_cast(uint32_t, f)); }
    void vec3(Vec3f v) {
        real(vec3f::x(v));
        real(vec3f::y(v));
        real(vec3f::z(v));
    }
    void float3(const Float3 &f) {
        real(f.x);
        real(f.y);
        real(f.z);
    }
    void float4(const Float4 &f) {
        real(f.x);
        real(f.y);
        real(f.z);
        real(f.w);
    }
};

/** an event of a frame: its kind, body and sub shape */
struct LogEntry {
    uint32_t kind;
    body_id::BodyId body_id;
    sub_shape::SubShapeId sub_shape_id;
};

constexpr uint32_t MAX_LOG = 64;

/** the bodies the policy answers for, INVALID_BODY_ID for none, and the events the character reported */
struct Rules {
    body_id::BodyId conveyor = body_id::INVALID_BODY_ID;
    body_id::BodyId unpushable = body_id::INVALID_BODY_ID;
    body_id::BodyId ignored = body_id::INVALID_BODY_ID;
    body_id::BodyId cannot_push_character = body_id::INVALID_BODY_ID;
    /** jolt's sample: no sliding down a still, walkable surface */
    bool slide_prevention = false;
    /** whether this body was ever reported added, and removed */
    body_id::BodyId watched = body_id::INVALID_BODY_ID;
    bool watched_added = false;
    bool watched_removed = false;
    LogEntry log[MAX_LOG];
    uint32_t log_count = 0;
    kcc_policy::Policy policy = kcc_policy::create();

    Rules() = default;
    Rules(const Rules &) = delete;
    ~Rules() { kcc_policy::release(policy); }
};

void log_entry(Rules &rules, uint32_t kind, body_id::BodyId id, sub_shape::SubShapeId sub_shape_id) {
    if (rules.log_count < MAX_LOG) rules.log[rules.log_count++] = {kind, id, sub_shape_id};
    if (id == rules.watched && kind == kcc::CONTACT_ADDED) rules.watched_added = true;
    if (id == rules.watched && kind == kcc::CONTACT_REMOVED) rules.watched_removed = true;
}

/** the events of the character's last tracked call (update, refresh_contacts, set_shape) into the log */
void log_events(Rules &rules, const kcc::KCC &character) {
    for (uint32_t i = 0; i < character.contact_events.length; i++) {
        const kcc::ContactEvent &event = character.contact_events[i];
        log_entry(rules, event.kind, event.body_id, event.sub_shape_id);
    }
}

void set_body(kcc_policy::Policy &policy, body_id::BodyId body_id, const kcc_policy::BodyPolicy &body_policy) {
    if (body_id != body_id::INVALID_BODY_ID) kcc_policy::set(policy, body_id, body_policy);
}

/** the policy the rules make, as of this frame */
const kcc_policy::Policy *policy_of(Rules &rules) {
    kcc_policy::Policy &policy = rules.policy;
    policy.defaults.slide_on_walkable = !rules.slide_prevention;
    set_body(policy, rules.conveyor, {.surface_linear_velocity = {2.0f, 0.0f, 0.0f}});
    set_body(policy, rules.unpushable, {.can_receive_impulses = false});
    set_body(policy, rules.ignored, {.ignore = true});
    set_body(policy, rules.cannot_push_character, {.can_push_character = false});
    return &policy;
}

/** a frame's log in one order, whichever order it was reported in */
void hash_log(Hash &hash, Rules &rules) {
    LogEntry *log = rules.log;
    for (uint32_t i = 1; i < rules.log_count; i++) {
        const LogEntry entry = log[i];
        uint32_t j = i;
        while (j > 0 && (log[j - 1].kind > entry.kind || (log[j - 1].kind == entry.kind && (log[j - 1].body_id > entry.body_id ||
                                                                                            (log[j - 1].body_id == entry.body_id &&
                                                                                             log[j - 1].sub_shape_id > entry.sub_shape_id))))) {
            log[j] = log[j - 1];
            j--;
        }
        log[j] = entry;
    }
    hash.word(rules.log_count);
    for (uint32_t i = 0; i < rules.log_count; i++) {
        hash.word(log[i].kind);
        hash.word(log[i].body_id);
        hash.word(log[i].sub_shape_id);
    }
    rules.log_count = 0;
}

void hash_frame(Hash &hash, const world::World &world, const kcc::KCC &character) {
    hash.vec3(character.position);
    hash.vec3(vec3f::make(quatf::x(character.quaternion), quatf::y(character.quaternion), quatf::z(character.quaternion)));
    hash.real(quatf::w(character.quaternion));
    hash.vec3(character.linear_velocity);
    const kcc::GroundInfo &ground = character.ground;
    hash.word(ground.state);
    hash.vec3(ground.normal);
    hash.vec3(ground.velocity);
    hash.word(ground.body_id);
    hash.word(ground.sub_shape_id);
    hash.vec3(ground.position);
    hash.word(character.contacts.length);
    for (uint32_t i = 0; i < character.contacts.length; i++) {
        const kcc::CharacterContact &contact = character.contacts[i];
        hash.vec3(contact.position);
        hash.vec3(contact.linear_velocity);
        hash.vec3(contact.contact_normal);
        hash.vec3(contact.surface_normal);
        hash.real(contact.distance);
        hash.real(contact.fraction);
        hash.word(contact.body_id);
        hash.word(contact.sub_shape_id);
        hash.word((uint32_t)contact.material_id);
        hash.word(contact.motion_type);
        hash.word((contact.is_sensor ? 1u : 0u) | (contact.had_collision ? 2u : 0u) | (contact.was_discarded ? 4u : 0u) |
                  (contact.can_push_character ? 8u : 0u) | (contact.can_receive_impulses ? 16u : 0u));
    }
    const bodies::Bodies &bodies = world.bodies;
    for (uint32_t i = 0; i < bodies.pool.length; i++) {
        const RigidBody *body = bodies.pool[i];
        if (body == nullptr || bodies.sequences[i] == bodies::SLOT_FREE) continue;
        hash.word(body->id);
        hash.float3(body->position);
        hash.float4(body->quaternion);
        hash.float3(body->motion_properties.linear_velocity);
        hash.float3(body->motion_properties.angular_velocity);
    }
}

/** the hashes of a run: the physics, and the events' log */
struct Golden {
    Hash physics;
    Hash log;
};

/**
 * a frame of a game: the ground velocity brought up to date, the velocity from the ground's when standing (else the
 * vertical part of the last), gravity and the input; the character's update, then the world's step
 */
void frame(Golden &golden, world::World &world, kcc::KCC &character, Vec3f input, const kcc::UpdateSettings &settings,
           const filter::Filter &filter, Rules *rules) {
    const kcc_policy::Policy *policy = rules != nullptr ? policy_of(*rules) : nullptr;
    kcc::update_ground_velocity(world, character, policy);
    Vec3f velocity = character.ground.state == kcc::ON_GROUND ? character.ground.velocity
                                                              : vec3f::scale(character.up, vec3f::dot(character.linear_velocity, character.up));
    velocity = vec3f::scale_and_add(velocity, vec3f::load(GRAVITY), DT);
    character.linear_velocity = vec3f::add(velocity, input);
    kcc::update(world, character, DT, vec3f::load(GRAVITY), settings, policy, filter);
    if (rules != nullptr) log_events(*rules, character);
    step_world(world, DT);
    hash_frame(golden.physics, world, character);
    if (rules != nullptr) hash_log(golden.log, *rules);
}

RigidBody *add_static_box(WorldScene &scene, const Float3 &half_extents, const Float3 &position, const Float4 &quaternion = IDENTITY) {
    rigid_body::RigidBodySettings settings =
        body_settings(box::create({.half_extents = half_extents}), scene.not_moving, motion_type::MOTION_STATIC, position);
    settings.quaternion = quaternion;
    return rigid_body::create(*scene.world, settings);
}

Float4 about_z(float angle) { return {0.0f, 0.0f, scalar::sin(0.5f * angle), scalar::cos(0.5f * angle)}; }

/** a capsule whose position is its feet, the shape 0.8 above them; with an inner body on the moving layer when given one */
kcc::KCC *make_character(WorldScene &scene, const Float3 &position, bool inner_body = false) {
    kcc::KCCSettings settings;
    settings.shape = capsule::create({.half_height_of_cylinder = 0.5f, .radius = 0.3f});
    settings.shape_offset = {0.0f, 0.8f, 0.0f};
    settings.max_slope_angle = 45.0f * scalar::DEGREES_TO_RADIANS;
    if (inner_body) {
        settings.inner_rigid_body_shape = settings.shape;
        settings.inner_rigid_body_object_layer = scene.moving;
    }
    kcc::KCC *character = kcc::create(settings, vec3f::load(position), quatf::load(IDENTITY));
    if (inner_body) kcc::add(*scene.world, *character);
    return character;
}

/**
 * a failure names the golden and the hash it got, to paste in when the change was meant: the harness keeps the text's
 * pointer and there is no libc in the wasm build, so each failure is written into a buffer of its own
 */
void check_golden(TestCtx &ctx, const char *name, uint32_t hash, uint32_t expected) {
    static char messages[32][96];
    static uint32_t used = 0;
    if (hash == expected || used == 32) {
        test_check(ctx, hash == expected, __FILE__, __LINE__, "golden");
        return;
    }
    char *message = messages[used++];
    uint32_t length = 0;
    for (const char *c = name; *c != '\0' && length < 72; c++) message[length++] = *c;
    const char *got = ": got 0x";
    for (const char *c = got; *c != '\0'; c++) message[length++] = *c;
    for (int shift = 28; shift >= 0; shift -= 4) message[length++] = "0123456789abcdef"[(hash >> shift) & 0xfu];
    message[length++] = 'u';
    message[length] = '\0';
    test_check(ctx, false, __FILE__, __LINE__, message);
}

void release(WorldScene &scene, kcc::KCC *character, filter::Filter &filter) {
    if (character->inner_rigid_body_id != body_id::INVALID_BODY_ID) kcc::remove(*scene.world, *character);
    kcc::destroy(character);
    filter::release(filter);
    release_world_scene(scene);
}

float x_of(const kcc::KCC &character) { return vec3f::x(character.position); }
float y_of(const kcc::KCC &character) { return vec3f::y(character.position); }
float z_of(const kcc::KCC &character) { return vec3f::z(character.position); }

}  // namespace

// up three steps, stopped by a fourth too tall to climb, then back down them to the floor: walk_stairs and
// stick_to_floor
void test_kcc_golden_steps_up_and_down(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    add_static_box(scene, {50.0f, 0.5f, 50.0f}, {0.0f, -0.5f, 0.0f});
    const float heights[4] = {0.1f, 0.25f, 0.4f, 0.9f};
    for (int i = 0; i < 4; i++) add_static_box(scene, {0.5f, 0.5f * heights[i], 3.0f}, {2.0f + 1.0f * (float)i, 0.5f * heights[i], 0.0f});
    step_world(*scene.world);

    kcc::KCC *character = make_character(scene, {0.0f, 0.0f, 0.0f});
    filter::Filter filter = filter::create(scene.world->settings.layers);
    const kcc::UpdateSettings settings;
    Golden golden;
    for (int i = 0; i < 300; i++) {
        const Vec3f input = i < 150 ? vec3f::make(2.5f, 0.0f, 0.3f) : i < 200 ? vec3f::make(-2.0f, 0.0f, 2.0f) : vec3f::make(-1.0f, 0.0f, 0.0f);
        frame(golden, *scene.world, *character, input, settings, filter, nullptr);
        if (i == 149) {
            // on the 0.4 step, against the face of the 0.9 one at x 4.5
            CHECK_NEAR(ctx, y_of(*character), 0.4f, 0.01f);
            CHECK(ctx, x_of(*character) < 4.21f);
        }
    }
    CHECK_NEAR(ctx, y_of(*character), 0.0f, 0.01f);
    check_golden(ctx, "steps_up_and_down", golden.physics.value, 0x875d209au);
    release(scene, character, filter);
}

// up a walkable ramp and creeping back down it while still, then held at the foot of a slope too steep
void test_kcc_golden_slopes(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    add_static_box(scene, {50.0f, 0.5f, 50.0f}, {0.0f, -0.5f, 0.0f});
    add_static_box(scene, {3.0f, 0.2f, 3.0f}, {4.0f, 0.5f, 0.0f}, about_z(25.0f * scalar::DEGREES_TO_RADIANS));
    add_static_box(scene, {3.0f, 0.2f, 3.0f}, {-4.0f, 1.5f, 0.0f}, about_z(-60.0f * scalar::DEGREES_TO_RADIANS));
    step_world(*scene.world);

    kcc::KCC *character = make_character(scene, {0.0f, 0.0f, 0.0f});
    filter::Filter filter = filter::create(scene.world->settings.layers);
    const kcc::UpdateSettings settings;
    Golden golden;
    float up_the_ramp = 0.0f;
    for (int i = 0; i < 360; i++) {
        const Vec3f input = i < 120 ? vec3f::make(2.0f, 0.0f, 0.0f) : i < 180 ? vec3f::make(0.0f, 0.0f, 0.0f) : vec3f::make(-3.0f, 0.0f, 0.2f);
        frame(golden, *scene.world, *character, input, settings, filter, nullptr);
        if (i == 119) {
            CHECK(ctx, y_of(*character) > 0.5f);
            up_the_ramp = x_of(*character);
        }
        if (i == 179) CHECK(ctx, x_of(*character) < up_the_ramp);
    }
    // not up the steep slope, whose foot is near x -2.7
    CHECK(ctx, x_of(*character) > -3.0f);
    CHECK_NEAR(ctx, y_of(*character), 0.0f, 0.01f);
    check_golden(ctx, "slopes", golden.physics.value, 0xf578bb56u);
    release(scene, character, filter);
}

// along a wall and into a corner it makes with another at 70 degrees
void test_kcc_golden_walls_and_a_wedge(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    add_static_box(scene, {50.0f, 0.5f, 50.0f}, {0.0f, -0.5f, 0.0f});
    add_static_box(scene, {6.0f, 1.5f, 0.2f}, {0.0f, 1.5f, 2.0f});
    const float angle = 70.0f * scalar::DEGREES_TO_RADIANS;
    add_static_box(scene, {3.0f, 1.5f, 0.2f}, {6.0f, 1.5f, 0.0f}, {0.0f, scalar::sin(0.5f * angle), 0.0f, scalar::cos(0.5f * angle)});
    step_world(*scene.world);

    kcc::KCC *character = make_character(scene, {-4.0f, 0.0f, 0.0f});
    filter::Filter filter = filter::create(scene.world->settings.layers);
    const kcc::UpdateSettings settings;
    Golden golden;
    for (int i = 0; i < 300; i++) {
        frame(golden, *scene.world, *character, vec3f::make(3.0f, 0.0f, 1.5f), settings, filter, nullptr);
        // along the wall's face at z 1.8
        if (i == 60) CHECK_NEAR(ctx, z_of(*character), 1.5f, 0.03f);
    }
    // in the corner, touching both walls and the floor
    CHECK(ctx, character->contacts.length >= 3);
    CHECK(ctx, x_of(*character) < 5.2f);
    check_golden(ctx, "walls_and_a_wedge", golden.physics.value, 0x763b4d1bu);
    release(scene, character, filter);
}

// standing on a kinematic platform that moves, then on one that turns
void test_kcc_golden_platforms(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    world::World &world = *scene.world;
    add_static_box(scene, {50.0f, 0.5f, 50.0f}, {0.0f, -0.5f, 0.0f});
    RigidBody *mover = add_body(scene, box::create({.half_extents = {2.0f, 0.25f, 2.0f}}), motion_type::MOTION_KINEMATIC, {0.0f, 1.0f, 0.0f});
    RigidBody *turner = add_body(scene, box::create({.half_extents = {3.0f, 0.25f, 3.0f}}), motion_type::MOTION_KINEMATIC, {8.0f, 1.0f, 0.0f});
    rigid_body::set_linear_velocity(world, mover, vec3f::make(1.0f, 0.0f, 0.5f));
    rigid_body::set_angular_velocity(world, turner, vec3f::make(0.0f, 1.0f, 0.0f));
    step_world(world);

    kcc::KCC *character = make_character(scene, {0.0f, 1.3f, 0.0f});
    filter::Filter filter = filter::create(world.settings.layers);
    const kcc::UpdateSettings settings;
    Golden golden;
    float turning_from = 0.0f;
    for (int i = 0; i < 360; i++) {
        if (i == 180) rigid_body::set_linear_velocity(world, mover, vec3f::make(0.0f, 0.0f, 0.0f));
        const Vec3f input = i < 120 ? vec3f::make(0.0f, 0.0f, 0.0f) : i < 200 ? vec3f::make(3.0f, 0.0f, -0.5f) : vec3f::make(0.0f, 0.0f, 0.0f);
        frame(golden, world, *character, input, settings, filter, nullptr);
        if (i == 119) {
            // carried two seconds at 1 m/s along x
            CHECK(ctx, character->ground.body_id == mover->id);
            CHECK_NEAR(ctx, x_of(*character), 1.9f, 0.05f);
        }
        if (i == 199) {
            CHECK(ctx, character->ground.body_id == turner->id);
            turning_from = z_of(*character);
        }
    }
    // standing still, carried round by the turning platform
    CHECK(ctx, character->ground.body_id == turner->id);
    CHECK(ctx, z_of(*character) < turning_from - 1.0f);
    check_golden(ctx, "platforms", golden.physics.value, 0xef12e144u);
    release(scene, character, filter);
}

// a character with an inner body pushing two crates, then standing on a dynamic slab (its weight pressing it)
void test_kcc_golden_pushing_and_standing_on_dynamics(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    add_static_box(scene, {50.0f, 0.5f, 50.0f}, {0.0f, -0.5f, 0.0f});
    RigidBody *crate =
        add_body(scene, box::create({.half_extents = {0.4f, 0.4f, 0.4f}, .density = 20.0f}), motion_type::MOTION_DYNAMIC, {1.5f, 0.4f, 0.1f});
    RigidBody *small_crate =
        add_body(scene, box::create({.half_extents = {0.3f, 0.3f, 0.3f}, .density = 20.0f}), motion_type::MOTION_DYNAMIC, {2.5f, 0.3f, -0.2f});
    RigidBody *slab =
        add_body(scene, box::create({.half_extents = {1.0f, 0.15f, 1.0f}, .density = 50.0f}), motion_type::MOTION_DYNAMIC, {-3.0f, 0.15f, 0.0f});
    for (int i = 0; i < 30; i++) step_world(*scene.world);

    kcc::KCC *character = make_character(scene, {0.0f, 0.0f, 0.0f}, true);
    filter::Filter filter = filter::create(scene.world->settings.layers);
    const kcc::UpdateSettings settings;
    Golden golden;
    for (int i = 0; i < 360; i++) {
        const Vec3f input = i < 150 ? vec3f::make(2.0f, 0.0f, 0.0f) : i < 300 ? vec3f::make(-3.0f, 0.0f, 0.0f) : vec3f::make(0.0f, 0.0f, 0.0f);
        frame(golden, *scene.world, *character, input, settings, filter, nullptr);
    }
    CHECK(ctx, crate->position.x > 4.5f);
    CHECK(ctx, small_crate->position.x > 5.5f);
    CHECK(ctx, character->ground.body_id == slab->id);
    CHECK_NEAR(ctx, y_of(*character), 0.3f, 0.01f);
    check_golden(ctx, "pushing_and_standing_on_dynamics", golden.physics.value, 0x48e9daaau);
    release(scene, character, filter);
}

// across bumpy triangle mesh terrain, with enhanced internal edge removal, then a sphere character against it with few
// hits allowed, so hit reduction runs
void test_kcc_golden_mesh_terrain(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    constexpr int CELLS = 12;
    float positions[(CELLS + 1) * (CELLS + 1) * 3];
    uint32_t indices[CELLS * CELLS * 6];
    for (int z = 0; z <= CELLS; z++) {
        for (int x = 0; x <= CELLS; x++) {
            float *p = positions + ((z * (CELLS + 1) + x) * 3);
            p[0] = (float)(x - CELLS / 2);
            p[1] = 0.15f * scalar::sin(1.3f * (float)x) * scalar::cos(0.9f * (float)z);
            p[2] = (float)(z - CELLS / 2);
        }
    }
    uint32_t *index = indices;
    for (int z = 0; z < CELLS; z++) {
        for (int x = 0; x < CELLS; x++) {
            const uint32_t a = (uint32_t)(z * (CELLS + 1) + x), b = a + 1, c = a + CELLS + 1, d = c + 1;
            *index++ = a;
            *index++ = c;
            *index++ = b;
            *index++ = b;
            *index++ = c;
            *index++ = d;
        }
    }
    triangle_mesh_builder::TriangleMeshBuilderSettings mesh_settings;
    mesh_settings.positions = positions;
    mesh_settings.num_positions = (CELLS + 1) * (CELLS + 1);
    mesh_settings.indices = indices;
    mesh_settings.num_indices = CELLS * CELLS * 6;
    triangle_mesh_data::TriangleMeshData *data = triangle_mesh_builder::build(mesh_settings);
    CHECK(ctx, data != nullptr);
    add_body(scene, triangle_mesh::create({.data = data}), motion_type::MOTION_STATIC, {0.0f, 0.0f, 0.0f});
    step_world(*scene.world);

    filter::Filter filter = filter::create(scene.world->settings.layers);
    const kcc::UpdateSettings settings;
    Golden golden;

    kcc::KCC *walker = make_character(scene, {-4.0f, 0.3f, -4.0f});
    walker->enhanced_internal_edge_removal = true;
    for (int i = 0; i < 240; i++) frame(golden, *scene.world, *walker, vec3f::make(2.0f, 0.0f, 1.7f), settings, filter, nullptr);
    // across the bumps, on the mesh throughout
    CHECK(ctx, x_of(*walker) > 3.5f && z_of(*walker) > 2.5f);
    CHECK(ctx, walker->ground.state == kcc::ON_GROUND);
    kcc::destroy(walker);

    kcc::KCCSettings ball_settings;
    ball_settings.shape = sphere::create({.radius = 0.6f});
    ball_settings.max_num_hits = 3;
    kcc::KCC *ball = kcc::create(ball_settings, vec3f::make(3.0f, 0.4f, 3.0f), quatf::load(IDENTITY));
    bool at_the_hit_limit = false;
    for (int i = 0; i < 180; i++) {
        frame(golden, *scene.world, *ball, vec3f::make(-1.5f, 0.0f, -2.0f), settings, filter, nullptr);
        if (ball->contacts.length == 3) at_the_hit_limit = true;
    }
    CHECK(ctx, at_the_hit_limit);
    CHECK(ctx, x_of(*ball) < -1.0f);
    check_golden(ctx, "mesh_terrain", golden.physics.value, 0x3d03ebdcu);

    release(scene, ball, filter);
    triangle_mesh_data::destroy(data);
}

// crouching (a shorter shape) mid walk, refused standing under a low ceiling and standing again past it, a teleport
// with its contacts refreshed, and a sensor walked through, its events logged
void test_kcc_golden_crouch_teleport_and_a_sensor(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    world::World &world = *scene.world;
    add_static_box(scene, {50.0f, 0.5f, 50.0f}, {0.0f, -0.5f, 0.0f});
    // a ceiling from y 1.25, under the standing shape's top at 1.6
    add_static_box(scene, {1.0f, 0.2f, 2.0f}, {4.0f, 1.45f, 0.0f});
    rigid_body::RigidBodySettings sensor_settings =
        body_settings(box::create({.half_extents = {0.5f, 1.0f, 2.0f}}), scene.not_moving, motion_type::MOTION_STATIC, {-3.0f, 1.0f, 0.0f});
    sensor_settings.sensor = true;
    Rules rules;
    rules.watched = rigid_body::create(world, sensor_settings)->id;
    step_world(world);

    // each shape's origin at the feet, as the docs give them, so they swap freely
    shapes::Shape *standing = transformed::create({.shape = capsule::create({.half_height_of_cylinder = 0.5f, .radius = 0.3f}),
                                                   .position = {0.0f, 0.8f, 0.0f}});
    shapes::Shape *crouching = transformed::create({.shape = capsule::create({.half_height_of_cylinder = 0.2f, .radius = 0.3f}),
                                                    .position = {0.0f, 0.5f, 0.0f}});
    kcc::KCCSettings character_settings;
    character_settings.shape = standing;
    character_settings.max_slope_angle = 45.0f * scalar::DEGREES_TO_RADIANS;
    kcc::KCC *character = kcc::create(character_settings, vec3f::make(0.0f, 0.0f, 0.0f), quatf::load(IDENTITY));
    filter::Filter filter = filter::create(world.settings.layers);
    const kcc::UpdateSettings settings;
    Golden golden;
    for (int i = 0; i < 360; i++) {
        if (i == 40) {
            CHECK(ctx, kcc::set_shape(world, *character, crouching, filter, policy_of(rules), 0.01f));
            log_events(rules, *character);
        }
        if (i == 120 || i == 200) {
            // refused, it begins no frame: the events are still the last update's, already logged
            const bool stood = kcc::set_shape(world, *character, standing, filter, policy_of(rules), 0.01f);
            if (stood) log_events(rules, *character);
            // under the ceiling at x 4, then past it at x 6.7
            CHECK(ctx, stood == (i == 200));
            golden.physics.word(stood ? 1u : 0u);
        }
        if (i == 240) {
            kcc::set_position(world, *character, vec3f::make(-1.0f, 0.0f, 0.0f));
            kcc::refresh_contacts(world, *character, filter, policy_of(rules));
            log_events(rules, *character);
        }
        const Vec3f input = i < 220 ? vec3f::make(2.0f, 0.0f, 0.0f) : vec3f::make(-2.5f, 0.0f, 0.0f);
        frame(golden, world, *character, input, settings, filter, &rules);
    }
    // through the sensor at x -3 and out past it, told of it coming and going
    CHECK(ctx, x_of(*character) < -5.5f);
    CHECK(ctx, rules.watched_added && rules.watched_removed);
    check_golden(ctx, "crouch_teleport_and_a_sensor", golden.physics.value, 0x95689fb3u);
    check_golden(ctx, "crouch_teleport_and_a_sensor log", golden.log.value, 0x675f03a8u);
    release(scene, character, filter);
}

// the behaviour hooks, each answering by body so that data can say the same: a conveyor under the character, a crate
// it cannot push, a wall it walks through, a kinematic pusher that cannot push it, no sliding while there is no input

void test_kcc_golden_conveyor(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    add_static_box(scene, {50.0f, 0.5f, 50.0f}, {0.0f, -0.5f, 0.0f});
    Rules rules;
    rules.conveyor = add_static_box(scene, {3.0f, 0.1f, 1.0f}, {2.0f, 0.1f, 0.0f})->id;
    step_world(*scene.world);

    kcc::KCC *character = make_character(scene, {0.0f, 0.2f, 0.0f});
    filter::Filter filter = filter::create(scene.world->settings.layers);
    const kcc::UpdateSettings settings;
    Golden golden;
    float boarded = 0.0f;
    for (int i = 0; i < 240; i++) {
        const Vec3f input = i < 30 ? vec3f::make(0.5f, 0.0f, 0.0f) : i < 150 ? vec3f::make(0.0f, 0.0f, 0.0f) : vec3f::make(-1.0f, 0.0f, 0.5f);
        frame(golden, *scene.world, *character, input, settings, filter, &rules);
        if (i == 29) boarded = x_of(*character);
        // two seconds standing still, carried at the belt's 2 m/s
        if (i == 149) CHECK_NEAR(ctx, x_of(*character) - boarded, 4.0f, 0.05f);
    }
    check_golden(ctx, "conveyor", golden.physics.value, 0x7293f453u);
    check_golden(ctx, "conveyor log", golden.log.value, 0xf97c6c7fu);
    release(scene, character, filter);
}

void test_kcc_golden_unpushable_crate(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    add_static_box(scene, {50.0f, 0.5f, 50.0f}, {0.0f, -0.5f, 0.0f});
    Rules rules;
    RigidBody *unpushable =
        add_body(scene, box::create({.half_extents = {0.4f, 0.4f, 0.4f}, .density = 10.0f}), motion_type::MOTION_DYNAMIC, {1.5f, 0.4f, 0.0f});
    rules.unpushable = unpushable->id;
    RigidBody *pushable =
        add_body(scene, box::create({.half_extents = {0.4f, 0.4f, 0.4f}, .density = 10.0f}), motion_type::MOTION_DYNAMIC, {1.5f, 0.4f, 2.0f});
    for (int i = 0; i < 30; i++) step_world(*scene.world);

    kcc::KCC *character = make_character(scene, {0.0f, 0.0f, 0.0f});
    filter::Filter filter = filter::create(scene.world->settings.layers);
    const kcc::UpdateSettings settings;
    Golden golden;
    for (int i = 0; i < 240; i++) {
        const Vec3f input = i < 90 ? vec3f::make(2.0f, 0.0f, 0.0f) : i < 120 ? vec3f::make(-2.0f, 0.0f, 4.0f) : vec3f::make(2.0f, 0.0f, 0.0f);
        frame(golden, *scene.world, *character, input, settings, filter, &rules);
        // held at the unpushable crate's face
        if (i == 89) CHECK_NEAR(ctx, x_of(*character), 0.78f, 0.01f);
    }
    CHECK_NEAR(ctx, unpushable->position.x, 1.5f, 1e-3f);
    CHECK(ctx, pushable->position.x > 3.5f);
    check_golden(ctx, "unpushable_crate", golden.physics.value, 0x43fcd8ecu);
    check_golden(ctx, "unpushable_crate log", golden.log.value, 0x203281ebu);
    release(scene, character, filter);
}

void test_kcc_golden_ignored_wall(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    add_static_box(scene, {50.0f, 0.5f, 50.0f}, {0.0f, -0.5f, 0.0f});
    Rules rules;
    rules.ignored = add_static_box(scene, {0.2f, 1.5f, 3.0f}, {2.0f, 1.5f, 0.0f})->id;
    add_static_box(scene, {0.2f, 1.5f, 3.0f}, {5.0f, 1.5f, 0.0f});
    step_world(*scene.world);

    kcc::KCC *character = make_character(scene, {0.0f, 0.0f, 0.0f});
    filter::Filter filter = filter::create(scene.world->settings.layers);
    const kcc::UpdateSettings settings;
    Golden golden;
    for (int i = 0; i < 180; i++) frame(golden, *scene.world, *character, vec3f::make(3.0f, 0.0f, 0.4f), settings, filter, &rules);
    // through the wall at x 2, stopped by the one at x 5
    CHECK_NEAR(ctx, x_of(*character), 4.48f, 0.01f);
    check_golden(ctx, "ignored_wall", golden.physics.value, 0x33b1aa53u);
    check_golden(ctx, "ignored_wall log", golden.log.value, 0x544f0659u);
    release(scene, character, filter);
}

void test_kcc_golden_pusher_that_cannot_push(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    world::World &world = *scene.world;
    add_static_box(scene, {50.0f, 0.5f, 50.0f}, {0.0f, -0.5f, 0.0f});
    Rules rules;
    RigidBody *pusher = add_body(scene, box::create({.half_extents = {0.3f, 0.8f, 1.0f}}), motion_type::MOTION_KINEMATIC, {3.0f, 0.8f, 0.0f});
    rules.cannot_push_character = pusher->id;
    RigidBody *other = add_body(scene, box::create({.half_extents = {0.3f, 0.8f, 1.0f}}), motion_type::MOTION_KINEMATIC, {-3.0f, 0.8f, 3.0f});
    rigid_body::set_linear_velocity(world, pusher, vec3f::make(-1.0f, 0.0f, 0.0f));
    rigid_body::set_linear_velocity(world, other, vec3f::make(1.0f, 0.0f, 0.0f));
    step_world(world);

    kcc::KCC *character = make_character(scene, {0.0f, 0.0f, 0.0f});
    filter::Filter filter = filter::create(world.settings.layers);
    const kcc::UpdateSettings settings;
    Golden golden;
    for (int i = 0; i < 300; i++) {
        if (i == 150) kcc::set_position(world, *character, vec3f::make(0.0f, 0.0f, 3.0f));
        frame(golden, world, *character, vec3f::make(0.0f, 0.0f, 0.0f), settings, filter, &rules);
        // the pusher has passed over it without moving it
        if (i == 149) CHECK_NEAR(ctx, x_of(*character), 0.0f, 1e-3f);
    }
    // the other pusher shoved it along
    CHECK(ctx, x_of(*character) > 1.0f);
    check_golden(ctx, "pusher_that_cannot_push", golden.physics.value, 0xdf87185cu);
    check_golden(ctx, "pusher_that_cannot_push log", golden.log.value, 0xdc226d9au);
    release(scene, character, filter);
}

// on a walkable slope: with jolt's sample rule (no sliding while there is no input) the character holds still and
// walks when told; without it, it creeps down
void test_kcc_golden_slide_prevention(TestCtx &ctx) {
    for (int prevent = 0; prevent < 2; prevent++) {
        WorldScene scene = make_world_scene(ctx);
        add_static_box(scene, {50.0f, 0.5f, 50.0f}, {0.0f, -0.5f, 0.0f});
        add_static_box(scene, {4.0f, 0.2f, 3.0f}, {0.0f, 1.0f, 0.0f}, about_z(20.0f * scalar::DEGREES_TO_RADIANS));
        step_world(*scene.world);

        kcc::KCC *character = make_character(scene, {0.0f, 1.6f, 0.0f});
        filter::Filter filter = filter::create(scene.world->settings.layers);
        const kcc::UpdateSettings settings;
        Rules rules;
        Golden golden;
        for (int i = 0; i < 240; i++) {
            const Vec3f input = i < 120 ? vec3f::make(0.0f, 0.0f, 0.0f) : i < 150 ? vec3f::make(1.0f, 0.0f, 0.0f) : vec3f::make(0.0f, 0.0f, 0.0f);
            rules.slide_prevention = prevent == 1 && vec3f::squared_length(input) == 0.0f;
            frame(golden, *scene.world, *character, input, settings, filter, &rules);
            if (i == 119) {
                if (prevent == 1) CHECK_NEAR(ctx, x_of(*character), 0.0f, 1e-4f);
                else CHECK(ctx, x_of(*character) < -0.05f);
            }
        }
        // walked up the slope when told, either way
        CHECK(ctx, x_of(*character) > 0.2f);
        check_golden(ctx, prevent == 1 ? "slide_prevention" : "slide_prevention off", golden.physics.value, prevent == 1 ? 0xcc013157u : 0x1abb9757u);
        check_golden(ctx, prevent == 1 ? "slide_prevention log" : "slide_prevention off log", golden.log.value, prevent == 1 ? 0x07825431u : 0x07825431u);
        release(scene, character, filter);
    }
}
