#include "tumble/physics/body/body_activation.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/constraints/point_constraint.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/dispatch.h"
#include "tumble/physics/world.h"
#include "test.h"


namespace {

struct Scene {
    World *world;
    uint32_t layer;
    Ref<Shape> box;
    BodyActivationRecorder recorder;
};

void make_scene(Scene &s) {
    dispatch::init();
    WorldSettings settings = world_settings::create();
    s.layer = world_settings::add_object_layer(settings, world_settings::add_broadphase_layer(settings));
    world_settings::enable_collision(settings, s.layer, s.layer);
    s.world = world::create(settings);
    s.box = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});
    body_activation::init_recorder(s.recorder);
    world::set_body_activation_listener(*s.world, &s.recorder.listener);
}

void release_scene(Scene &s) {
    world::destroy(s.world);
    s.box = nullptr;
}

RigidBody *add(Scene &s, MotionType motion_type, Float3 position, uint64_t user_data = 0) {
    return rigid_body::create(*s.world, {.shape = s.box, .object_layer = s.layer, .motion_type = motion_type, .user_data = user_data, .position = position});
}

void steps(Scene &s, uint32_t count) {
    for (uint32_t i = 0; i < count; i++) world::step(*s.world, 1.0f / 60.0f);
}

/** the recorder holds exactly this one event, which it then forgets */
bool heard_once(Scene &s, const RigidBody *body, bool activated) {
    const bool once = s.recorder.events.length == 1 && s.recorder.events[0].body_id == body->id &&
                      s.recorder.events[0].user_data == body->user_data && s.recorder.events[0].activated == activated;
    s.recorder.events.length = 0;
    return once;
}

bool heard_nothing(Scene &s) { return s.recorder.events.length == 0; }

/** how many events the recorder holds for the body, of the kind */
uint32_t count_heard(const Scene &s, const RigidBody *body, bool activated) {
    uint32_t count = 0;
    for (uint32_t i = 0; i < s.recorder.events.length; i++) {
        if (s.recorder.events[i].body_id == body->id && s.recorder.events[i].activated == activated) count++;
    }
    return count;
}

}  // namespace

void test_body_activation_calls_between_steps(TestCtx &ctx) {
    Scene s;
    make_scene(s);

    // a dynamic or kinematic body starts active; a static one never is
    RigidBody *dynamic = add(s, MotionType::DYNAMIC, {0.0f, 5.0f, 0.0f}, 7);
    CHECK(ctx, heard_once(s, dynamic, true));
    RigidBody *kinematic = add(s, MotionType::KINEMATIC, {3.0f, 5.0f, 0.0f});
    CHECK(ctx, heard_once(s, kinematic, true));
    RigidBody *still = add(s, MotionType::STATIC, {6.0f, 0.0f, 0.0f});
    CHECK(ctx, heard_nothing(s));

    // put to sleep and woken by hand, each once: a second call changes nothing
    rigid_body::sleep(*s.world, dynamic);
    CHECK(ctx, heard_once(s, dynamic, false) && rigid_body::is_sleeping(*dynamic));
    rigid_body::sleep(*s.world, dynamic);
    CHECK(ctx, heard_nothing(s));
    rigid_body::wake(*s.world, dynamic);
    CHECK(ctx, heard_once(s, dynamic, true) && rigid_body::is_active(*dynamic));
    rigid_body::wake(*s.world, dynamic);
    CHECK(ctx, heard_nothing(s));
    rigid_body::wake(*s.world, still);
    CHECK(ctx, heard_nothing(s) && !rigid_body::is_sleeping(*still));

    // a setter that wakes wakes through the same door
    rigid_body::sleep(*s.world, dynamic);
    s.recorder.events.length = 0;
    rigid_body::set_linear_velocity(*s.world, dynamic, vec3f::make(1.0f, 0.0f, 0.0f));
    CHECK(ctx, heard_once(s, dynamic, true));

    // made static: off the list; made dynamic again: back on it
    rigid_body::set_motion_type(*s.world, kinematic, MotionType::STATIC, false);
    CHECK(ctx, heard_once(s, kinematic, false));
    rigid_body::set_motion_type(*s.world, kinematic, MotionType::DYNAMIC, false);
    CHECK(ctx, heard_once(s, kinematic, true));

    // removed while active: it leaves the list first, heard with its user data; removed asleep, nothing more
    const RigidBody before = *dynamic;
    rigid_body::remove(*s.world, dynamic);
    CHECK(ctx, s.recorder.events.length == 1 && s.recorder.events[0].body_id == before.id && s.recorder.events[0].user_data == 7 &&
                   !s.recorder.events[0].activated);
    s.recorder.events.length = 0;
    rigid_body::sleep(*s.world, kinematic);
    s.recorder.events.length = 0;
    rigid_body::remove(*s.world, kinematic);
    CHECK(ctx, heard_nothing(s));

    release_scene(s);
}

void test_body_activation_steps(TestCtx &ctx) {
    Scene s;
    make_scene(s);
    RigidBody *ground = rigid_body::create(*s.world, {.shape = box::create({.half_extents = {20.0f, 0.5f, 20.0f}}),
                                                      .object_layer = s.layer,
                                                      .motion_type = MotionType::STATIC,
                                                      .position = {0.0f, -0.5f, 0.0f}});
    RigidBody *resting = add(s, MotionType::DYNAMIC, {0.0f, 0.5f, 0.0f});
    s.recorder.events.length = 0;

    // an island at rest goes to sleep in a step: heard once
    for (uint32_t i = 0; i < 240 && rigid_body::is_active(*resting); i++) steps(s, 1);
    CHECK(ctx, rigid_body::is_sleeping(*resting) && heard_once(s, resting, false));
    steps(s, 30);
    CHECK(ctx, heard_nothing(s));

    // a body falling onto it wakes it in the step their contact is made
    RigidBody *falling = add(s, MotionType::DYNAMIC, {0.0f, 3.0f, 0.0f});
    CHECK(ctx, heard_once(s, falling, true));
    for (uint32_t i = 0; i < 120 && rigid_body::is_sleeping(*resting); i++) steps(s, 1);
    CHECK(ctx, rigid_body::is_active(*resting) && count_heard(s, resting, true) == 1 && count_heard(s, resting, false) == 0);
    s.recorder.events.length = 0;

    // a sleeping body jointed to an active one is woken by the step
    RigidBody *anchor = add(s, MotionType::KINEMATIC, {10.0f, 5.0f, 0.0f});
    RigidBody *hanging = add(s, MotionType::DYNAMIC, {10.0f, 4.0f, 0.0f});
    point_constraint::create(*s.world, anchor, hanging, {.point_a = {10.0f, 4.5f, 0.0f}, .point_b = {10.0f, 4.5f, 0.0f}});
    rigid_body::sleep(*s.world, hanging);
    s.recorder.events.length = 0;
    steps(s, 1);
    CHECK(ctx, rigid_body::is_active(*hanging) && count_heard(s, hanging, true) == 1);

    Ref<Shape> ground_shape = ground->shape;
    release_scene(s);
    ground_shape = nullptr;
}

namespace {

struct Heard {
    BodyActivationEvent events[8];
    uint32_t count;
    /** woken from the first activation heard, to show a flush hands on what its listener records */
    World *world;
    RigidBody *wake_on_first;
};

void hear(BodyActivationListener &listener, BodyId body_id, uint64_t user_data, bool activated) {
    Heard &heard = *static_cast<Heard *>(listener.context);
    if (heard.count < 8) heard.events[heard.count] = {body_id, user_data, activated};
    heard.count++;
    if (activated && heard.wake_on_first != nullptr) {
        RigidBody *body = heard.wake_on_first;
        heard.wake_on_first = nullptr;
        rigid_body::wake(*heard.world, body);
    }
}

void hear_activated(BodyActivationListener &listener, BodyId body_id, uint64_t user_data) { hear(listener, body_id, user_data, true); }
void hear_deactivated(BodyActivationListener &listener, BodyId body_id, uint64_t user_data) { hear(listener, body_id, user_data, false); }

}  // namespace

void test_body_activation_recorder_flush(TestCtx &ctx) {
    Scene s;
    make_scene(s);
    RigidBody *a = add(s, MotionType::DYNAMIC, {0.0f, 5.0f, 0.0f}, 1);
    RigidBody *b = add(s, MotionType::DYNAMIC, {3.0f, 5.0f, 0.0f}, 2);
    rigid_body::sleep(*s.world, a);
    rigid_body::sleep(*s.world, b);

    // replayed in the order they happened, then gone
    Heard heard = {};
    heard.world = s.world;
    BodyActivationListener listener = {hear_activated, hear_deactivated, &heard};
    body_activation::flush(s.recorder, &listener);
    CHECK(ctx, heard.count == 4 && s.recorder.events.length == 0);
    CHECK(ctx, heard.events[0].body_id == a->id && heard.events[0].activated && heard.events[0].user_data == 1);
    CHECK(ctx, heard.events[1].body_id == b->id && heard.events[1].activated && heard.events[1].user_data == 2);
    CHECK(ctx, heard.events[2].body_id == a->id && !heard.events[2].activated);
    CHECK(ctx, heard.events[3].body_id == b->id && !heard.events[3].activated);
    heard.count = 0;
    body_activation::flush(s.recorder, &listener);
    CHECK(ctx, heard.count == 0);

    // the listener wakes b on hearing a wake: the flush hands that on too
    rigid_body::wake(*s.world, a);
    heard.wake_on_first = b;
    body_activation::flush(s.recorder, &listener);
    CHECK(ctx, heard.count == 2 && heard.events[0].body_id == a->id && heard.events[1].body_id == b->id && heard.events[1].activated);
    CHECK(ctx, s.recorder.events.length == 0);

    // no listener only empties it; a listener missing a callback skips those events
    rigid_body::sleep(*s.world, a);
    body_activation::flush(s.recorder, nullptr);
    CHECK(ctx, s.recorder.events.length == 0);
    heard.count = 0;
    BodyActivationListener deactivations_only = {nullptr, hear_deactivated, &heard};
    rigid_body::wake(*s.world, a);
    rigid_body::sleep(*s.world, a);
    body_activation::flush(s.recorder, &deactivations_only);
    CHECK(ctx, heard.count == 1 && !heard.events[0].activated);

    release_scene(s);
}
