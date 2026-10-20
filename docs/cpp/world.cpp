#include <stdio.h>

#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/events.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/dispatch.h"
#include "tumble/physics/shapes/sphere.h"
#include "tumble/physics/update.h"
#include "tumble/physics/world.h"

using namespace tumble;

/* SNIPPET_START: layers */
// object layers say what collides with what; each maps to a broadphase layer, a tree of its own, so a query or the
// pair search walks only the trees whose layers can collide
struct Layers {
    uint32_t moving;
    uint32_t not_moving;
    uint32_t debris;
};

world_settings::WorldSettings make_settings(Layers &out) {
    world_settings::WorldSettings settings = world_settings::create();
    const uint32_t broadphase_moving = world_settings::add_broadphase_layer(settings);
    const uint32_t broadphase_not_moving = world_settings::add_broadphase_layer(settings);
    out.moving = world_settings::add_object_layer(settings, broadphase_moving);
    out.not_moving = world_settings::add_object_layer(settings, broadphase_not_moving);
    out.debris = world_settings::add_object_layer(settings, broadphase_moving);
    world_settings::enable_collision(settings, out.moving, out.moving);
    world_settings::enable_collision(settings, out.moving, out.not_moving);
    // debris lands on the ground and nothing else
    world_settings::enable_collision(settings, out.debris, out.not_moving);
    return settings;
}
/* SNIPPET_END: layers */

/* SNIPPET_START: phases */
// world::step is these phases, answering nothing. between them the world's events say what a phase found, and its
// answers say what to do about it: here pairs with a body whose user data is 7 are rejected, so it passes through
// everything, and no contact bounces
void step_with_answers(world::World &world, float delta_time) {
    update::step_begin(world, delta_time);
    do {
        update::find_pairs(world);
        for (uint32_t i = 0; i < world.events.pairs.length; i++) {
            const events::PairEvent &pair = world.events.pairs[i];
            if (pair.user_data_a == 7 || pair.user_data_b == 7) events::reject_pair(world.answers, i);
        }
        update::narrowphase(world);
    } while (world.events.woken > 0);
    // after the last narrowphase: one answer per contacts_added entry, then per contacts_persisted entry
    for (uint32_t i = 0; i < world.answers.contact_override.length; i++) {
        events::override_contact(world.answers, world.events, i).settings.combined_restitution = 0.0f;
    }
    update::solve(world);
    update::step_end(world);
}
/* SNIPPET_END: phases */

/* SNIPPET_START: events */
// after a step: the contacts it found and kept, the ones it lost, and the bodies it moved
void print_events(const world::World &world) {
    for (uint32_t i = 0; i < world.events.contacts_added.length; i++) {
        const events::ContactEvent &contact = world.events.contacts_added[i];
        printf("contact %u - %u, %u points, %.3f deep\n", body_id::index(contact.body_id_a), body_id::index(contact.body_id_b),
               contact.num_points, contact.penetration);
    }
    printf("%u contacts kept, %u removed, %u bodies moved\n", world.events.contacts_persisted.length, world.events.contacts_removed.length,
           world.events.moved.length);
}
/* SNIPPET_END: events */

int main() {
    dispatch::init();

    /* SNIPPET_START: settings */
    Layers layers;
    world_settings::WorldSettings settings = make_settings(layers);
    settings.gravity = {0.0f, -20.0f, 0.0f};
    settings.solver.velocity_iterations = 8;
    settings.sleeping.time_before_sleep = 1.0f;
    world::World *world = world::create(settings);
    // a second world from the same settings is a world of its own
    world::World *preview = world::create(settings);
    world_settings::release(settings);
    /* SNIPPET_END: settings */

    shapes::Shape *ground_shape = box::create({.half_extents = {20.0f, 0.5f, 20.0f}});
    shapes::Shape *ball_shape = sphere::create({.radius = 0.5f});
    rigid_body::create(*world, {.shape = ground_shape, .object_layer = layers.not_moving, .motion_type = motion_type::MOTION_STATIC,
                                .position = {0.0f, -0.5f, 0.0f}});
    rigid_body::RigidBody *ball = rigid_body::create(*world, {.shape = ball_shape, .object_layer = layers.moving,
                                                               .motion_type = motion_type::MOTION_DYNAMIC, .position = {0.0f, 3.0f, 0.0f}});
    rigid_body::RigidBody *ghost = rigid_body::create(*world, {.shape = ball_shape, .object_layer = layers.moving,
                                                                .motion_type = motion_type::MOTION_DYNAMIC, .user_data = 7,
                                                                .position = {2.0f, 0.5f, 0.0f}});

    /* SNIPPET_START: fixed-step */
    // whatever the frame time, the world advances in steps of 1/60 s; a long frame catches up a few steps at most
    const float step_time = 1.0f / 60.0f;
    float accumulator = 0.0f;
    const float frame_times[] = {0.016f, 0.017f, 0.033f, 0.2f};
    for (float frame_time : frame_times) {
        accumulator += frame_time;
        for (int steps = 0; accumulator >= step_time && steps < 4; steps++) {
            world::step(*world, step_time);
            accumulator -= step_time;
        }
        if (accumulator > step_time) accumulator = 0.0f;
    }
    /* SNIPPET_END: fixed-step */

    // the ghost passes through everything, and nothing bounces
    for (int i = 0; i < 120; i++) {
        step_with_answers(*world, step_time);
        if (i == 60) print_events(*world);
    }


    /* HIDE_START */
    // the ball came to rest on the ground; the ghost, whose pairs were rejected, fell through it
    const bool ball_rests = ball->position.y > 0.45f && ball->position.y < 0.55f;
    if (!ball_rests || ghost->position.y > 0.0f || preview == world) return 1;
    /* HIDE_END */

    world::destroy(preview);
    world::destroy(world);
    sphere::destroy(ball_shape);
    box::destroy(ground_shape);
    return 0;
}
