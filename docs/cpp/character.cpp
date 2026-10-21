#include <stdio.h>

#include "tumble/math/quatf.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/character/kcc.h"
#include "tumble/physics/character/kcc_policy.h"
#include "tumble/physics/contact_pairs.h"
#include "tumble/physics/filter.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/capsule.h"
#include "tumble/physics/shapes/dispatch.h"
#include "tumble/physics/shapes/transformed.h"
#include "tumble/physics/world.h"

using namespace tumble;


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
    // a crate beside the way
    shapes::Shape *crate_shape = box::create({.half_extents = {0.4f, 0.4f, 0.4f}, .density = 20.0f});
    rigid_body::RigidBody *crate = rigid_body::create(*world, {.shape = crate_shape, .object_layer = moving, .motion_type = motion_type::MOTION_DYNAMIC,
                                                               .position = {2.0f, 0.4f, 3.0f}});

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

    /* SNIPPET_START: policy */
    // how characters treat the bodies they meet, by body, the rest taking the policy's defaults; one policy serves any
    // number of characters. here the crate may be pushed (until the character crouches, below)
    kcc_policy::Policy policy = kcc_policy::create();
    kcc_policy::set(policy, crate->id, {.can_receive_impulses = true});
    /* SNIPPET_END: policy */

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
        kcc::update(*world, *character, delta_time, gravity, update_settings, &policy, filter);
        world::step(*world, delta_time);
    }
    printf("at x %.2f y %.2f, %s\n", vec3f::x(character->position), vec3f::y(character->position),
           character->ground.state == kcc::ON_GROUND ? "on the ground" : "not on the ground");
    /* SNIPPET_END: update */
    /* HIDE_START */
    // three seconds at 3 m/s: past the step and onto its top, then off it again, on the ground throughout
    if (vec3f::x(character->position) < 8.0f || character->ground.state != kcc::ON_GROUND) return 1;
    /* HIDE_END */

    /* SNIPPET_START: crouch */
    // a shorter shape, on the same feet, where it fits: refused, false, if it would sink deeper than the given depth
    shapes::Shape *crouching_capsule = capsule::create({.half_height_of_cylinder = 0.2f, .radius = 0.3f});
    shapes::Shape *crouching = transformed::create({.shape = crouching_capsule, .position = {0.0f, 0.5f, 0.0f}});
    const bool crouched = kcc::set_shape(*world, *character, crouching, filter, &policy, 0.01f);
    if (crouched) kcc_policy::set(policy, crate->id, {.can_receive_impulses = false});
    /* SNIPPET_END: crouch */
    /* HIDE_START */
    if (!crouched || character->shape != crouching || kcc_policy::get(&policy, crate->id).can_receive_impulses) return 1;
    /* HIDE_END */

    /* SNIPPET_START: events */
    // what the character met in its last update (or refresh_contacts, or set_shape): each contact added or persisted as
    // it was first touched, and each gone by the end
    uint32_t touching = 0;
    for (uint32_t i = 0; i < character->contact_events.length; i++) {
        const kcc::ContactEvent &event = character->contact_events[i];
        if (event.kind != kcc::CONTACT_REMOVED) touching++;
    }
    printf("touching %u bodies\n", touching);
    /* SNIPPET_END: events */
    /* HIDE_START */
    if (touching == 0) return 1;
    /* HIDE_END */

    /* SNIPPET_START: contacts */
    // its contacts as data, through its inner body, in a contact pairs frame
    contact_pairs::begin(*world);
    kcc::refresh_contacts(*world, *character, filter, &policy);
    contact_pairs::record_character_contacts(*world, *character);
    contact_pairs::end(*world);
    /* SNIPPET_END: contacts */
    /* HIDE_START */
    if (world->contact_pairs.active.length == 0) return 1;
    const contact_pairs::ContactPair &pair = world->contact_pairs.pairs[world->contact_pairs.active[0]];
    if (pair.a.body_id != character->inner_rigid_body_id || pair.normal.y > -0.99f) return 1;
    /* HIDE_END */

    /* HIDE_START */
    const float under_x = vec3f::x(character->position);
    /* HIDE_END */
    /* SNIPPET_START: toolkit */
    // update is the whole move in one call; the toolkit is its phases, for a loop of your own that edits the contacts
    // between collide and solve. here a one-way platform overhead, jumped up through: from below, its contacts and the
    // sweep's hit on it are passed over
    shapes::Shape *platform_shape = box::create({.half_extents = {1.5f, 0.1f, 1.5f}});
    rigid_body::RigidBody *platform = rigid_body::create(*world, {.shape = platform_shape, .object_layer = not_moving,
                                                                  .motion_type = motion_type::MOTION_STATIC,
                                                                  .position = {vec3f::x(character->position), 1.8f, 0.0f}});
    const auto from_below = [&](const kcc::CharacterContact &contact) {
        return contact.body_id == platform->id && vec3f::y(contact.contact_normal) < 0.5f;
    };
    character->linear_velocity = vec3f::make(0.0f, 7.0f, 0.0f);
    for (int frame = 0; frame < 90; frame++) {
        character->linear_velocity = vec3f::scale_and_add(character->linear_velocity, gravity, delta_time);
        Vec3f position = character->position;
        Vec3f velocity = character->linear_velocity;
        float time_remaining = delta_time;
        for (uint32_t iteration = 0; iteration < character->max_collision_iterations && time_remaining >= character->min_time_remaining; iteration++) {
            kcc::collide(*world, *character, position, vec3f::normalize(velocity), filter, &policy);
            for (uint32_t i = 0; i < character->contacts.length; i++) {
                if (from_below(character->contacts[i])) character->contacts[i].was_discarded = true;
            }
            Vec3f displacement;
            float time_simulated = kcc::solve(*world, *character, velocity, delta_time, time_remaining, &policy, displacement);
            kcc::CharacterContact hit;
            if (kcc::cast(hit, *world, *character, position, displacement, filter, &policy) && !from_below(hit)) {
                displacement = vec3f::scale(displacement, hit.fraction);
                time_simulated *= hit.fraction;
            }
            position = vec3f::add(position, displacement);
            time_remaining -= time_simulated;
        }
        kcc::set_position(*world, *character, position);
        kcc::update_ground(*world, *character, delta_time, false, &policy);
        // landed, coming down
        if (character->ground.state == kcc::ON_GROUND && vec3f::y(character->linear_velocity) <= 0.0f) {
            character->linear_velocity = vec3f::make(0.0f, 0.0f, 0.0f);
        }
        world::step(*world, delta_time);
    }
    /* SNIPPET_END: toolkit */
    /* HIDE_START */
    // up through the platform and stood on its top at 1.9, where it jumped
    if (character->ground.body_id != platform->id || vec3f::y(character->position) < 1.89f || vec3f::y(character->position) > 1.91f) return 1;
    if (vec3f::x(character->position) != under_x) return 1;
    /* HIDE_END */

    /* SNIPPET_START: remove */
    kcc::remove(*world, *character);
    kcc::destroy(character);
    /* SNIPPET_END: remove */

    kcc_policy::release(policy);
    filter::release(filter);
    world::destroy(world);
    transformed::destroy(crouching);
    transformed::destroy(standing);
    capsule::destroy(crouching_capsule);
    capsule::destroy(standing_capsule);
    box::destroy(crate_shape);
    box::destroy(platform_shape);
    box::destroy(step_shape);
    box::destroy(ground_shape);
    return 0;
}
