// the character's whole behaviour, frame by frame, as hashes: a change to the controller that should not change what
// it does is checked against these. vectors hash their x, y and z, the fourth lane being nothing the results depend on

#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/character/kcc.h"
#include "tumble/physics/character/kcc_policy.h"
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
    kcc::CharacterListener listener;

    Rules() = default;
    Rules(const Rules &) = delete;
    ~Rules() { kcc_policy::release(policy); }
};

void log_entry(Rules &rules, uint32_t kind, body_id::BodyId id, sub_shape::SubShapeId sub_shape_id) {
    if (rules.log_count < MAX_LOG) rules.log[rules.log_count++] = {kind, id, sub_shape_id};
    if (id == rules.watched && kind == kcc::CONTACT_ADDED) rules.watched_added = true;
    if (id == rules.watched && kind == kcc::CONTACT_REMOVED) rules.watched_removed = true;
}

/** the frames run with the hand-written listener below in place of the policy's, for test_kcc_golden_hand_written_listener */
bool use_hand_written = false;

/** the events of the character's last tracked call (update, refresh_contacts, set_shape) into the log; the hand-written listener logs its own */
void log_events(Rules &rules, const kcc::KCC &character) {
    if (use_hand_written) return;
    for (uint32_t i = 0; i < character.contact_events.length; i++) {
        const kcc::ContactEvent &event = character.contact_events[i];
        log_entry(rules, event.kind, event.body_id, event.sub_shape_id);
    }
}

void set_body(kcc_policy::Policy &policy, body_id::BodyId body_id, const kcc_policy::BodyPolicy &body_policy) {
    if (body_id != body_id::INVALID_BODY_ID) kcc_policy::set(policy, body_id, body_policy);
}

void settings_for(const Rules &rules, const RigidBody &body, kcc::CharacterContactSettings &io_settings) {
    if (body.id == rules.unpushable) io_settings.can_receive_impulses = false;
    if (body.id == rules.cannot_push_character) io_settings.can_push_character = false;
}

// the rules as a listener's callbacks, written as crashcat's and Jolt's samples write them, logging what they are told

void on_adjust(const kcc::KCC &, const RigidBody &body, Vec3f &io_linear_velocity, Vec3f &, void *context) {
    const Rules &rules = *static_cast<const Rules *>(context);
    if (body.id == rules.conveyor) io_linear_velocity = vec3f::add(io_linear_velocity, vec3f::make(2.0f, 0.0f, 0.0f));
}

bool on_validate(const kcc::KCC &, const RigidBody &body, sub_shape::SubShapeId, Vec3f, Vec3f, void *context) {
    return body.id != static_cast<const Rules *>(context)->ignored;
}

void on_added(const kcc::KCC &, const RigidBody &body, sub_shape::SubShapeId sub_shape_id, Vec3f, Vec3f,
              kcc::CharacterContactSettings &io_settings, void *context) {
    Rules &rules = *static_cast<Rules *>(context);
    settings_for(rules, body, io_settings);
    log_entry(rules, kcc::CONTACT_ADDED, body.id, sub_shape_id);
}

void on_persisted(const kcc::KCC &, const RigidBody &body, sub_shape::SubShapeId sub_shape_id, Vec3f, Vec3f,
                  kcc::CharacterContactSettings &io_settings, void *context) {
    Rules &rules = *static_cast<Rules *>(context);
    settings_for(rules, body, io_settings);
    log_entry(rules, kcc::CONTACT_PERSISTED, body.id, sub_shape_id);
}

/** jolt's sample: no sliding down a still, walkable surface */
void on_solve(const kcc::KCC &character, const RigidBody &, sub_shape::SubShapeId, Vec3f, Vec3f contact_normal, Vec3f contact_velocity,
              Vec3f, Vec3f &io_new_character_velocity, void *context) {
    const Rules &rules = *static_cast<const Rules *>(context);
    if (rules.slide_prevention && vec3f::squared_length(contact_velocity) <= 1e-12f && !kcc::is_slope_too_steep(character, contact_normal)) {
        io_new_character_velocity = vec3f::make(0.0f, 0.0f, 0.0f);
    }
}

void on_removed(const kcc::KCC &, body_id::BodyId id, sub_shape::SubShapeId sub_shape_id, void *context) {
    log_entry(*static_cast<Rules *>(context), kcc::CONTACT_REMOVED, id, sub_shape_id);
}

/** the listener the rules make, as of this frame: the policy's, or the hand-written one */
const kcc::CharacterListener *listener_of(Rules &rules) {
    if (use_hand_written) {
        rules.listener = {on_adjust, on_validate, on_added, on_persisted, on_solve, on_removed, &rules};
        return &rules.listener;
    }
    kcc_policy::Policy &policy = rules.policy;
    policy.defaults.slide_on_walkable = !rules.slide_prevention;
    set_body(policy, rules.conveyor, {.surface_linear_velocity = {2.0f, 0.0f, 0.0f}});
    set_body(policy, rules.unpushable, {.can_receive_impulses = false});
    set_body(policy, rules.ignored, {.ignore = true});
    set_body(policy, rules.cannot_push_character, {.can_push_character = false});
    rules.listener = kcc_policy::listener(policy);
    return &rules.listener;
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

/** update's frames run through toolkit_update instead, for test_kcc_golden_toolkit_composes_update */
bool use_toolkit = false;

/** move, from the toolkit's phases alone, as a caller's own loop would run them */
void toolkit_move(world::World &world, kcc::KCC &character, float delta_time, Vec3f gravity, const kcc::CharacterListener *listener,
                  const filter::Filter &filter) {
    if (delta_time <= 0.0f) return;
    Vec3f position = character.position;
    Vec3f velocity = character.linear_velocity;
    Vec3f direction = vec3f::make(0.0f, 0.0f, 0.0f);
    const float speed_sq = vec3f::squared_length(velocity);
    if (speed_sq > 1e-12f) direction = vec3f::scale(velocity, 1.0f / scalar::sqrt(speed_sq));
    float time_remaining = delta_time;
    for (uint32_t iteration = 0; iteration < character.max_collision_iterations; iteration++) {
        if (time_remaining < character.min_time_remaining) break;
        kcc::collide(world, character, position, direction, filter, listener);
        Vec3f displacement;
        float time_simulated = kcc::solve(world, character, velocity, delta_time, time_remaining, listener, displacement);
        kcc::CharacterContact hit;
        if (kcc::cast(hit, world, character, position, displacement, filter, listener)) {
            displacement = vec3f::scale(displacement, hit.fraction);
            time_simulated *= hit.fraction;
        }
        position = vec3f::add(position, displacement);
        time_remaining -= time_simulated;
        if (vec3f::squared_length(displacement) < 1e-8f) break;
    }
    kcc::set_position(world, character, position);
    kcc::update_ground(world, character, delta_time, false, listener);
    kcc::apply_weight(world, character, gravity, delta_time);
}

/** update, from the toolkit's phases and the public calls alone: proof a caller's loop can do all update does */
void toolkit_update(world::World &world, kcc::KCC &character, float delta_time, Vec3f gravity, const kcc::UpdateSettings &settings,
                    const kcc::CharacterListener *listener, const filter::Filter &filter) {
    kcc::begin_contact_tracking(character);
    const Vec3f desired_velocity = character.linear_velocity;
    character.linear_velocity = kcc::cancel_velocity_towards_steep_slopes(character, desired_velocity);
    const Vec3f old_position = character.position;
    bool ground_to_air = kcc::is_supported(character);

    toolkit_move(world, character, delta_time, gravity, listener, filter);

    if (kcc::is_supported(character)) ground_to_air = false;
    if (ground_to_air && vec3f::squared_length(vec3f::load(settings.stick_to_floor_step_down)) > 1e-12f) {
        const Vec3f step = vec3f::subtract(character.position, old_position);
        if (vec3f::dot(step, character.up) / delta_time <= 1e-6f) {
            kcc::stick_to_floor(world, character, vec3f::load(settings.stick_to_floor_step_down), filter, listener);
        }
    }
    if (vec3f::squared_length(vec3f::load(settings.walk_stairs_step_up)) > 1e-12f) {
        Vec3f desired_step = vec3f::scale(desired_velocity, delta_time);
        desired_step = vec3f::scale_and_add(desired_step, character.up, -vec3f::dot(desired_step, character.up));
        const float desired_length = vec3f::length(desired_step);
        if (desired_length > 0.0f) {
            Vec3f achieved = vec3f::subtract(character.position, old_position);
            achieved = vec3f::scale_and_add(achieved, character.up, -vec3f::dot(achieved, character.up));
            const Vec3f forward = vec3f::scale(desired_step, 1.0f / desired_length);
            const float achieved_forward = scalar::max(0.0f, vec3f::dot(achieved, forward));
            achieved = vec3f::scale(forward, achieved_forward);
            const float achieved_length = vec3f::length(achieved);
            if (achieved_length + 1e-4f < desired_length && kcc::can_walk_stairs(character, desired_velocity)) {
                // towards the steep contact most opposing the movement, within the forward contact angle
                Vec3f direction = forward;
                float max_dot = settings.walk_stairs_cos_angle_forward_contact;
                for (uint32_t i = 0; i < character.contacts.length; i++) {
                    const kcc::CharacterContact &c = character.contacts[i];
                    if (!c.had_collision || c.was_discarded || !kcc::is_slope_too_steep(character, c.surface_normal)) continue;
                    if (vec3f::dot(c.surface_normal, vec3f::subtract(desired_velocity, c.linear_velocity)) >= 0.0f) continue;
                    Vec3f test = vec3f::subtract(vec3f::scale(character.up, vec3f::dot(c.surface_normal, character.up)), c.surface_normal);
                    const float test_length = vec3f::length(test);
                    if (test_length <= 1e-6f) continue;
                    test = vec3f::scale(test, 1.0f / test_length);
                    if (vec3f::dot(test, forward) > max_dot) {
                        direction = test;
                        max_dot = vec3f::dot(test, forward);
                    }
                }
                const Vec3f step_forward = vec3f::scale(direction, scalar::max(settings.walk_stairs_min_step_forward, desired_length - achieved_length));
                const Vec3f step_forward_test = vec3f::scale(direction, settings.walk_stairs_step_forward_test);
                kcc::walk_stairs(world, character, delta_time, vec3f::load(settings.walk_stairs_step_up), step_forward, step_forward_test,
                                 vec3f::load(settings.walk_stairs_step_down_extra), filter, listener);
            }
        }
    }
    kcc::end_contact_tracking(character, listener);
}

/**
 * a frame of a game: the ground velocity brought up to date, the velocity from the ground's when standing (else the
 * vertical part of the last), gravity and the input; the character's update, then the world's step
 */
void frame(Golden &golden, world::World &world, kcc::KCC &character, Vec3f input, const kcc::UpdateSettings &settings,
           const filter::Filter &filter, Rules *rules) {
    const kcc::CharacterListener *listener = rules != nullptr ? listener_of(*rules) : nullptr;
    kcc::update_ground_velocity(world, character, listener);
    Vec3f velocity = character.ground.state == kcc::ON_GROUND ? character.ground.velocity
                                                              : vec3f::scale(character.up, vec3f::dot(character.linear_velocity, character.up));
    velocity = vec3f::scale_and_add(velocity, vec3f::load(GRAVITY), DT);
    character.linear_velocity = vec3f::add(velocity, input);
    if (use_toolkit) toolkit_update(world, character, DT, vec3f::load(GRAVITY), settings, listener, filter);
    else kcc::update(world, character, DT, vec3f::load(GRAVITY), settings, listener, filter);
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
// stick_to_floor. the fourth rises 0.6, as a rise of 0.5 is climbed: the capsule's foot steps onto its rounded edge
void test_kcc_golden_steps_up_and_down(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    add_static_box(scene, {50.0f, 0.5f, 50.0f}, {0.0f, -0.5f, 0.0f});
    const float heights[4] = {0.1f, 0.25f, 0.4f, 1.0f};
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
    check_golden(ctx, "steps_up_and_down", golden.physics.value, 0x4e60d085u);
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
    check_golden(ctx, "slopes", golden.physics.value, 0xc84f9cefu);
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
    check_golden(ctx, "walls_and_a_wedge", golden.physics.value, 0xc7d2201bu);
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
    check_golden(ctx, "platforms", golden.physics.value, 0xe0813654u);
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
    check_golden(ctx, "pushing_and_standing_on_dynamics", golden.physics.value, 0x971acf3bu);
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
    check_golden(ctx, "mesh_terrain", golden.physics.value, 0xa2ea5476u);

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
            CHECK(ctx, kcc::set_shape(world, *character, crouching, filter, listener_of(rules), 0.01f));
            log_events(rules, *character);
        }
        if (i == 120 || i == 200) {
            // refused, it begins no frame: the events are still the last update's, already logged
            const bool stood = kcc::set_shape(world, *character, standing, filter, listener_of(rules), 0.01f);
            if (stood) log_events(rules, *character);
            // under the ceiling at x 4, then past it at x 6.7
            CHECK(ctx, stood == (i == 200));
            golden.physics.word(stood ? 1u : 0u);
        }
        if (i == 240) {
            kcc::set_position(world, *character, vec3f::make(-1.0f, 0.0f, 0.0f));
            kcc::refresh_contacts(world, *character, filter, listener_of(rules));
            log_events(rules, *character);
        }
        const Vec3f input = i < 220 ? vec3f::make(2.0f, 0.0f, 0.0f) : vec3f::make(-2.5f, 0.0f, 0.0f);
        frame(golden, world, *character, input, settings, filter, &rules);
    }
    // through the sensor at x -3 and out past it, told of it coming and going
    CHECK(ctx, x_of(*character) < -5.5f);
    CHECK(ctx, rules.watched_added && rules.watched_removed);
    check_golden(ctx, "crouch_teleport_and_a_sensor", golden.physics.value, 0x80c96ab3u);
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
    check_golden(ctx, "conveyor", golden.physics.value, 0xa25e9579u);
    check_golden(ctx, "conveyor log", golden.log.value, 0xc499a247u);
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
    check_golden(ctx, "unpushable_crate", golden.physics.value, 0x38806201u);
    check_golden(ctx, "unpushable_crate log", golden.log.value, 0x5da92f11u);
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
    check_golden(ctx, "pusher_that_cannot_push", golden.physics.value, 0x73bac05cu);
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

// every scene again, each update run from the toolkit's phases by a caller's own loop: the same, bit for bit
void test_kcc_golden_toolkit_composes_update(TestCtx &ctx) {
    use_toolkit = true;
    test_kcc_golden_steps_up_and_down(ctx);
    test_kcc_golden_slopes(ctx);
    test_kcc_golden_walls_and_a_wedge(ctx);
    test_kcc_golden_platforms(ctx);
    test_kcc_golden_pushing_and_standing_on_dynamics(ctx);
    test_kcc_golden_mesh_terrain(ctx);
    test_kcc_golden_crouch_teleport_and_a_sensor(ctx);
    test_kcc_golden_conveyor(ctx);
    test_kcc_golden_unpushable_crate(ctx);
    test_kcc_golden_ignored_wall(ctx);
    test_kcc_golden_pusher_that_cannot_push(ctx);
    test_kcc_golden_slide_prevention(ctx);
    use_toolkit = false;
}

// every scene again, its rules as a hand-written listener in place of the policy's: the same, bit for bit, events and all
void test_kcc_golden_hand_written_listener(TestCtx &ctx) {
    use_hand_written = true;
    test_kcc_golden_steps_up_and_down(ctx);
    test_kcc_golden_slopes(ctx);
    test_kcc_golden_walls_and_a_wedge(ctx);
    test_kcc_golden_platforms(ctx);
    test_kcc_golden_pushing_and_standing_on_dynamics(ctx);
    test_kcc_golden_mesh_terrain(ctx);
    test_kcc_golden_crouch_teleport_and_a_sensor(ctx);
    test_kcc_golden_conveyor(ctx);
    test_kcc_golden_unpushable_crate(ctx);
    test_kcc_golden_ignored_wall(ctx);
    test_kcc_golden_pusher_that_cannot_push(ctx);
    test_kcc_golden_slide_prevention(ctx);
    use_hand_written = false;
}
