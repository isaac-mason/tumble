#include <stdio.h>

#include "tumble/tumble.h"

using namespace tumble;

/* SNIPPET_START: frame */
// a contact pairs frame. begin it, record each world step's contact events and each character move, then end it. a
// pair is added in the frame it starts, persisted while it lasts, and removed in the frame after it was last recorded
void frame(World &world, float frame_time, int steps) {
    contact_pairs::begin(world);
    for (int i = 0; i < steps; i++) {
        world::step(world, frame_time / (float)steps);
        contact_pairs::record_world_contacts(world, 0, 0);
    }
    contact_pairs::end(world);
}
/* SNIPPET_END: frame */

/* SNIPPET_START: read */
void print_pairs(const World &world) {
    const ContactPairs &pairs = world.contact_pairs;
    for (uint32_t i = 0; i < pairs.active.length; i++) {
        const ContactPair &pair = pairs.pairs[pairs.active[i]];
        printf("%s: body %u touches body %u at %.2f %.2f %.2f, %.2f m/s apart\n", pair.added ? "added" : "persisted",
               body_id::index(pair.a.body_id), body_id::index(pair.b.body_id), pair.point.x, pair.point.y, pair.point.z, pair.relative_velocity.y);
    }
    for (uint32_t i = 0; i < pairs.removed.length; i++) {
        const ContactPair &pair = pairs.pairs[pairs.removed[i]];
        printf("removed: body %u and body %u\n", body_id::index(pair.a.body_id), body_id::index(pair.b.body_id));
    }
}
/* SNIPPET_END: read */

int main() {
    dispatch::init();
    WorldSettings settings = world_settings::create();
    const uint32_t moving = world_settings::add_object_layer(settings, world_settings::add_broadphase_layer(settings));
    const uint32_t not_moving = world_settings::add_object_layer(settings, world_settings::add_broadphase_layer(settings));
    world_settings::enable_collision(settings, moving, moving);
    world_settings::enable_collision(settings, moving, not_moving);
    World *world = world::create(settings);
    Ref<Shape> ground_shape = box::create({.half_extents = {20.0f, 0.5f, 20.0f}});
    Ref<Shape> ball_shape = sphere::create({.radius = 0.5f});
    rigid_body::create(*world, {.shape = ground_shape, .object_layer = not_moving, .motion_type = MotionType::STATIC,
                                .position = {0.0f, -0.5f, 0.0f}});

    /* SNIPPET_START: observe */
    // only pairs that include a body observing contacts are kept. these are the few a game reacts to
    RigidBody *ball = rigid_body::create(*world, {.shape = ball_shape, .object_layer = moving,
                                                               .motion_type = MotionType::DYNAMIC, .position = {0.0f, 2.0f, 0.0f},
                                                               .restitution = 0.6f, .observes_contacts = true});
    /* SNIPPET_END: observe */

    /* HIDE_START */
    uint32_t added = 0;
    uint32_t removed = 0;
    bool printed = false;
    for (int i = 0; i < 120; i++) {
        frame(*world, 1.0f / 30.0f, 2);
        const ContactPairs &pairs = world->contact_pairs;
        for (uint32_t p = 0; p < pairs.active.length; p++) added += pairs.pairs[pairs.active[p]].added ? 1 : 0;
        removed += pairs.removed.length;
        if (!printed && pairs.active.length > 0) {
            print_pairs(*world);
            printed = true;
        }
    }
    // the ball bounced: its pair with the ground was added, removed in the air, and added again
    if (added < 2 || removed < 1 || ball->position.y > 0.6f) return 1;
    /* HIDE_END */

    world::destroy(world);
    ball_shape = nullptr;
    ground_shape = nullptr;
    return 0;
}
