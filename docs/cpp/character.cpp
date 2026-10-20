#include <stdio.h>

#include "tumble/math/quatf.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/character/kcc.h"
#include "tumble/physics/contact_pairs.h"
#include "tumble/physics/filter.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/capsule.h"
#include "tumble/physics/shapes/dispatch.h"
#include "tumble/physics/shapes/transformed.h"
#include "tumble/physics/world.h"

using namespace tumble;

/* SNIPPET_START: listener */
// the listener is the C++ way into a move, called as the character meets each body: here crates may be pushed, but
// only by a character that is not crouching
struct Rules {
    uint64_t crate_user_data;
    bool crouching;
    uint32_t touched;
};

void on_contact_added(const kcc::KCC &, const rigid_body::RigidBody &body, sub_shape::SubShapeId, Vec3f, Vec3f,
                      kcc::CharacterContactSettings &io_settings, void *context) {
    Rules &rules = *static_cast<Rules *>(context);
    rules.touched++;
    if (body.user_data == rules.crate_user_data) io_settings.can_receive_impulses = !rules.crouching;
}
/* SNIPPET_END: listener */

int main() {
    dispatch::init();
    world_settings::WorldSettings settings = world_settings::create();
    const uint32_t moving = world_settings::add_object_layer(settings, world_settings::add_broadphase_layer(settings));
    const uint32_t not_moving = world_settings::add_object_layer(settings, world_settings::add_broadphase_layer(settings));
    world_settings::enable_collision(settings, moving, moving);
    world_settings::enable_collision(settings, moving, not_moving);
    world::World *world = world::create(settings);
    world_settings::release(settings);
    shapes::Shape *ground_shape = box::create({.half_extents = {50.0f, 0.5f, 50.0f}});
    shapes::Shape *step_shape = box::create({.half_extents = {1.0f, 0.15f, 2.0f}});
    rigid_body::create(*world, {.shape = ground_shape, .object_layer = not_moving, .motion_type = motion_type::MOTION_STATIC,
                                .position = {0.0f, -0.5f, 0.0f}});
    // a 30 cm step 4 m ahead
    rigid_body::create(*world, {.shape = step_shape, .object_layer = not_moving, .motion_type = motion_type::MOTION_STATIC,
                                .position = {5.0f, 0.15f, 0.0f}});

    /* SNIPPET_START: create */
    // a capsule standing on its feet: the character's position is its feet, the capsule raised 0.8 m to stand on them.
    // the inner body is its presence in the world, which rigid bodies and queries see and contact pairs name it by
    shapes::Shape *standing_capsule = capsule::create({.half_height_of_cylinder = 0.5f, .radius = 0.3f});
    shapes::Shape *standing = transformed::create({.shape = standing_capsule, .position = {0.0f, 0.8f, 0.0f}});
    kcc::KCCSettings character_settings;
    character_settings.shape = standing;
    character_settings.max_slope_angle = 45.0f * scalar::DEGREES_TO_RADIANS;
    character_settings.inner_rigid_body_shape = standing;
    character_settings.inner_rigid_body_object_layer = moving;
    character_settings.observes_contacts = true;
    kcc::KCC *character = kcc::create(character_settings, vec3f::make(0.0f, 0.0f, 0.0f), quatf::identity());
    kcc::add(*world, *character);

    // what the character collides with: everything its layer does
    filter::Filter filter = filter::create(world->settings.layers);
    filter::set_from_layer(filter, world->settings.layers, moving, 0xffffffffu, 0xffffffffu);
    /* SNIPPET_END: create */

    Rules rules = {.crate_user_data = 9, .crouching = false, .touched = 0};
    kcc::CharacterListener listener = {};
    listener.on_contact_added = on_contact_added;
    listener.context = &rules;

    const float delta_time = 1.0f / 60.0f;
    const Vec3f gravity = vec3f::load(world->settings.gravity);
    /* SNIPPET_START: update */
    // a frame of a game: the velocity from the ground's when standing, else falling on, plus the input; then the
    // character moves (sliding along walls, up stairs, sticking to floors going down), then the world steps
    kcc::UpdateSettings update_settings;
    for (int frame = 0; frame < 180; frame++) {
        const Vec3f input = vec3f::make(3.0f, 0.0f, 0.0f);
        Vec3f velocity = character->ground.state == kcc::ON_GROUND
                             ? character->ground.velocity
                             : vec3f::scale(character->up, vec3f::dot(character->linear_velocity, character->up));
        velocity = vec3f::scale_and_add(velocity, gravity, delta_time);
        character->linear_velocity = vec3f::add(velocity, input);
        kcc::update(*world, *character, delta_time, gravity, update_settings, &listener, filter);
        world::step(*world, delta_time);
    }
    printf("at x %.2f y %.2f, %s\n", vec3f::x(character->position), vec3f::y(character->position),
           character->ground.state == kcc::ON_GROUND ? "on the ground" : "not on the ground");
    /* SNIPPET_END: update */
    /* HIDE_START */
    // three seconds at 3 m/s: past the step and onto its top, then off it again, on the ground throughout
    if (vec3f::x(character->position) < 8.0f || character->ground.state != kcc::ON_GROUND || rules.touched == 0) return 1;
    /* HIDE_END */

    /* SNIPPET_START: crouch */
    // a shorter shape, on the same feet, where it fits: refused, false, if it would sink deeper than the given depth
    shapes::Shape *crouching_capsule = capsule::create({.half_height_of_cylinder = 0.2f, .radius = 0.3f});
    shapes::Shape *crouching = transformed::create({.shape = crouching_capsule, .position = {0.0f, 0.5f, 0.0f}});
    if (kcc::set_shape(*world, *character, crouching, filter, &listener, 0.01f)) rules.crouching = true;
    /* SNIPPET_END: crouch */
    /* HIDE_START */
    if (!rules.crouching || character->shape != crouching) return 1;
    /* HIDE_END */

    /* SNIPPET_START: contacts */
    // its contacts as data, through its inner body, in a contact pairs frame
    contact_pairs::begin(*world);
    kcc::refresh_contacts(*world, *character, filter, &listener);
    contact_pairs::record_character_contacts(*world, *character);
    contact_pairs::end(*world);
    /* SNIPPET_END: contacts */
    /* HIDE_START */
    if (world->contact_pairs.active.length == 0) return 1;
    const contact_pairs::ContactPair &pair = world->contact_pairs.pairs[world->contact_pairs.active[0]];
    if (pair.a.body_id != character->inner_rigid_body_id || pair.normal.y > -0.99f) return 1;
    /* HIDE_END */

    /* SNIPPET_START: remove */
    kcc::remove(*world, *character);
    kcc::destroy(character);
    /* SNIPPET_END: remove */

    filter::release(filter);
    world::destroy(world);
    transformed::destroy(crouching);
    transformed::destroy(standing);
    capsule::destroy(crouching_capsule);
    capsule::destroy(standing_capsule);
    box::destroy(step_shape);
    box::destroy(ground_shape);
    return 0;
}
