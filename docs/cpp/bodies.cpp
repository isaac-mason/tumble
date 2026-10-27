#include <stdio.h>

#include "tumble/tumble.h"

using namespace tumble;

static uint32_t moving;
static uint32_t not_moving;

static World *make_world() {
    WorldSettings settings = world_settings::create();
    moving = world_settings::add_object_layer(settings, world_settings::add_broadphase_layer(settings));
    not_moving = world_settings::add_object_layer(settings, world_settings::add_broadphase_layer(settings));
    world_settings::enable_collision(settings, moving, moving);
    world_settings::enable_collision(settings, moving, not_moving);
    World *world = world::create(settings);
    return world;
}

static void steps(World &world, int count) {
    for (int i = 0; i < count; i++) world::step(world, 1.0f / 60.0f);
}

static bool near(float a, float b, float tolerance) { return a - b <= tolerance && b - a <= tolerance; }

int main() {
    dispatch::init();
    World *world = make_world();
    Ref<Shape> box_shape = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});
    Ref<Shape> ground_shape = box::create({.half_extents = {50.0f, 0.5f, 50.0f}});
    rigid_body::create(*world, {
        .shape = ground_shape,
        .object_layer = not_moving,
        .motion_type = MotionType::STATIC,
        .position = {0.0f, -0.5f, 0.0f},
    });

    {
        /* SNIPPET_START: create */
        // a body joins the world at create, and the next step finds it. the body holds its shape
        RigidBody *crate = rigid_body::create(*world, {
            .shape = box_shape,
            .object_layer = moving,
            .motion_type = MotionType::DYNAMIC,
            .position = {0.0f, 2.0f, 0.0f},
            .friction = 0.6f,
        });

        // its id names it while it exists. get returns the body for an id, or null once it is gone
        const BodyId id = crate->id;
        rigid_body::get(*world, id);

        // remove takes the body out of the world, with its constraints. its contacts' removal
        // events come with the next step
        rigid_body::remove(*world, crate);
        /* SNIPPET_END: create */
        /* HIDE_START */
        if (rigid_body::get(*world, id) != nullptr) return 1;
        /* HIDE_END */
    }

    {
        /* SNIPPET_START: motion-types */
        // a static body never moves. a kinematic body moves by its velocity and pushes dynamic bodies
        // aside. a dynamic body is moved by gravity, forces and contacts
        RigidBody *platform = rigid_body::create(*world, {
            .shape = box_shape,
            .object_layer = moving,
            .motion_type = MotionType::KINEMATIC,
            .position = {4.0f, 1.0f, 0.0f},
        });

        // a body can change its motion type. `wake` wakes a body that keeps moving
        rigid_body::set_motion_type(*world, platform, MotionType::DYNAMIC, true);
        /* SNIPPET_END: motion-types */
        /* HIDE_START */
        steps(*world, 60);
        if (!near(platform->position.y, 0.5f, 0.05f)) return 1;
        rigid_body::remove(*world, platform);
        /* HIDE_END */
    }

    RigidBody *body = rigid_body::create(*world, {
        .shape = box_shape,
        .object_layer = moving,
        .motion_type = MotionType::DYNAMIC,
        .position = {0.0f, 0.5f, 0.0f},
    });
    {
        /* SNIPPET_START: transform */
        // these teleport the body. the broadphase and the pairs update, and `wake` wakes a sleeping body
        const Quatf turned = quatf::set_axis_angle(vec3f::axis_y(), 0.5f);
        rigid_body::set_position(*world, body, vec3f::make(0.0f, 3.0f, 0.0f), true);
        rigid_body::set_quaternion(*world, body, turned, true);
        rigid_body::set_transform(*world, body, vec3f::make(0.0f, 0.5f, 0.0f), quatf::identity(), true);

        // read the pose from the body's fields. position is the shape's origin, and
        // center_of_mass_position is its centre of mass
        const Float3 &position = body->position;
        printf("at %.2f %.2f %.2f\n", position.x, position.y, position.z);
        /* SNIPPET_END: transform */
        /* HIDE_START */
        if (body->position.y != 0.5f || body->quaternion.w != 1.0f) return 1;
        /* HIDE_END */
    }

    {
        /* SNIPPET_START: velocity */
        // in metres and radians per second, in world space. setting either wakes the body
        rigid_body::set_linear_velocity(*world, body, vec3f::make(2.0f, 0.0f, 0.0f));
        rigid_body::set_angular_velocity(*world, body, vec3f::make(0.0f, 1.0f, 0.0f));
        rigid_body::add_linear_velocity(*world, body, vec3f::make(0.0f, 1.0f, 0.0f));

        // the velocity of any point on the body, from its linear and angular velocity
        const Vec3f corner = vec3f::make(0.5f, 1.0f, 0.5f);
        const Vec3f corner_velocity = rigid_body::get_velocity_at_point(body, corner);
        /* SNIPPET_END: velocity */
        /* HIDE_START */
        if (!near(vec3f::x(corner_velocity), 2.5f, 1e-5f) || body->motion_properties.linear_velocity.y != 1.0f) return 1;
        rigid_body::set_linear_velocity(*world, body, vec3f::zero());
        rigid_body::set_angular_velocity(*world, body, vec3f::zero());
        /* HIDE_END */
    }

    {
        /* SNIPPET_START: forces */
        // a force acts over the next step, then is cleared
        rigid_body::add_force(*world, body, vec3f::make(0.0f, 2000.0f, 0.0f), true);
        rigid_body::add_torque(*world, body, vec3f::make(0.0f, 10.0f, 0.0f), true);

        // a force away from the centre of mass turns the body too
        const Vec3f push = vec3f::make(100.0f, 0.0f, 0.0f);
        const Vec3f at = vec3f::make(0.0f, 1.0f, 0.0f);
        rigid_body::add_force_at_position(*world, body, push, at, true);

        // an impulse changes the velocity at once
        rigid_body::add_impulse(*world, body, vec3f::make(0.0f, 500.0f, 0.0f));
        rigid_body::add_angular_impulse(*world, body, vec3f::make(0.0f, 50.0f, 0.0f));
        /* SNIPPET_END: forces */
        /* HIDE_START */
        // a 1000 kg box: the impulse alone is 0.5 m/s up
        if (!near(body->motion_properties.linear_velocity.y, 0.5f, 1e-4f)) return 1;
        world::step(*world, 1.0f / 60.0f);
        if (body->motion_properties.force.y != 0.0f) return 1;
        steps(*world, 120);
        rigid_body::set_transform(*world, body, vec3f::make(0.0f, 0.5f, 0.0f), quatf::identity(), true);
        rigid_body::set_linear_velocity(*world, body, vec3f::zero());
        rigid_body::set_angular_velocity(*world, body, vec3f::zero());
        /* HIDE_END */
    }

    {
        /* SNIPPET_START: mass */
        // the mass comes from the shape's density by default, which is 1000 kg/m3, so the unit box
        // weighs 1000 kg. `mass` scales the shape's inertia to a mass you choose
        RigidBody *light = rigid_body::create(*world, {
            .shape = box_shape,
            .object_layer = moving,
            .motion_type = MotionType::DYNAMIC,
            .position = {3.0f, 0.5f, 3.0f},
            .mass = 10.0f,
        });

        // mass_properties_override sets the mass and inertia directly
        MassProperties heavy = mass_properties::create();
        mass_properties::set_mass_and_inertia_of_solid_box(heavy, vec3f::make(1.0f, 1.0f, 1.0f), 200.0f);
        RigidBody *given = rigid_body::create(*world, {
            .shape = box_shape,
            .object_layer = moving,
            .motion_type = MotionType::DYNAMIC,
            .position = {-3.0f, 0.5f, 3.0f},
            .mass_properties_override = &heavy,
        });
        /* SNIPPET_END: mass */
        /* HIDE_START */
        if (!near(1.0f / light->motion_properties.inv_mass, 10.0f, 1e-3f) || !near(1.0f / given->motion_properties.inv_mass, 200.0f, 1e-2f)) {
            return 1;
        }
        rigid_body::remove(*world, light);
        rigid_body::remove(*world, given);
        /* HIDE_END */
    }

    {
        /* SNIPPET_START: tuning */
        // damping slows a body a little every step, and the limits clamp its velocities.
        // gravity_factor scales gravity for this body alone, and 0 makes it float.
        // allowed_degrees_of_freedom locks axes: this body only slides in x and y
        RigidBody *puck = rigid_body::create(*world, {
            .shape = box_shape,
            .object_layer = moving,
            .motion_type = MotionType::DYNAMIC,
            .position = {6.0f, 3.0f, 0.0f},
            .allowed_degrees_of_freedom = dof::TRANSLATION_X | dof::TRANSLATION_Y,
            .gravity_factor = 0.0f,
            .linear_damping = 0.0f,
            .angular_damping = 0.0f,
            .max_linear_velocity = 20.0f,
        });
        rigid_body::set_linear_velocity(*world, puck, vec3f::make(30.0f, 0.0f, 5.0f));
        /* SNIPPET_END: tuning */
        /* HIDE_START */
        world::step(*world, 1.0f / 60.0f);
        // the limit clamped the velocity to 20 m/s, then the lock zeroed z; nothing fell
        const Float3 &velocity = puck->motion_properties.linear_velocity;
        if (velocity.z != 0.0f || velocity.x > 20.0f || velocity.x < 19.0f || velocity.y != 0.0f) {
            return 1;
        }
        rigid_body::remove(*world, puck);
        /* HIDE_END */
    }

    {
        /* SNIPPET_START: sleeping */
        // a body that has barely moved for a while sleeps. it costs nothing until something touches
        // or wakes it. set allow_sleeping to false to keep it awake. sleep and wake do it by hand
        rigid_body::sleep(*world, body);
        rigid_body::wake(*world, body);
        /* SNIPPET_END: sleeping */
        /* HIDE_START */
        if (rigid_body::is_sleeping(*body)) return 1;
        steps(*world, 120);
        if (!rigid_body::is_sleeping(*body)) return 1;
        /* HIDE_END */
    }

    {
        /* SNIPPET_START: kinematic */
        // a kinematic body follows a path by its velocities. move_kinematic sets the velocities that
        // reach the target in one step
        RigidBody *lift = rigid_body::create(*world, {
            .shape = box_shape,
            .object_layer = moving,
            .motion_type = MotionType::KINEMATIC,
            .position = {-6.0f, 0.5f, 0.0f},
        });

        const float step_time = 1.0f / 60.0f;
        for (int i = 1; i <= 60; i++) {
            const Vec3f target = vec3f::make(-6.0f, 0.5f + 2.0f * (float)i / 60.0f, 0.0f);
            rigid_body::move_kinematic(*world, lift, target, quatf::identity(), step_time);
            world::step(*world, step_time);
        }
        /* SNIPPET_END: kinematic */
        /* HIDE_START */
        if (!near(lift->position.y, 2.5f, 1e-3f)) return 1;
        rigid_body::remove(*world, lift);
        /* HIDE_END */
    }

    {
        /* SNIPPET_START: ccd */
        // a small fast body can pass through a thin one between two steps. linear casting sweeps
        // it, so it stops at what it would hit. this costs more
        Ref<Shape> pellet_shape = sphere::create({.radius = 0.05f});
        RigidBody *pellet = rigid_body::create(*world, {
            .shape = pellet_shape,
            .object_layer = moving,
            .motion_type = MotionType::DYNAMIC,
            .position = {10.0f, 5.0f, 0.0f},
            .motion_quality = MotionQuality::LINEAR_CAST,
        });
        rigid_body::set_linear_velocity(*world, pellet, vec3f::make(0.0f, -200.0f, 0.0f));
        /* SNIPPET_END: ccd */
        /* HIDE_START */
        steps(*world, 30);
        if (pellet->position.y < 0.0f) return 1;
        rigid_body::remove(*world, pellet);
        pellet_shape = nullptr;
        /* HIDE_END */
    }

    {
        /* SNIPPET_START: user-data */
        // your own value for the body, such as an entity id, an index or a pointer. every event with
        // the body carries it
        RigidBody *tagged = rigid_body::create(*world, {
            .shape = box_shape,
            .object_layer = moving,
            .motion_type = MotionType::DYNAMIC,
            .user_data = 42,
            .position = {0.0f, 0.5f, 6.0f},
        });
        tagged->user_data = 43;
        /* SNIPPET_END: user-data */
        /* HIDE_START */
        steps(*world, 2);
        bool seen = false;
        for (uint32_t i = 0; i < world->events.contacts_persisted.length; i++) {
            const ContactEvent &event = world->events.contacts_persisted[i];
            seen = seen || event.user_data_a == 43 || event.user_data_b == 43;
        }
        if (!seen) return 1;
        rigid_body::remove(*world, tagged);
        /* HIDE_END */
    }

    {
        /* SNIPPET_START: groups */
        // groups and masks filter within one object layer. two bodies collide when each one's mask
        // has a bit of the other's groups
        constexpr uint32_t PLAYERS = 1u << 0;
        constexpr uint32_t PROJECTILES = 1u << 1;

        RigidBody *player = rigid_body::create(*world, {
            .shape = box_shape,
            .object_layer = moving,
            .motion_type = MotionType::DYNAMIC,
            .position = {-10.0f, 0.5f, 0.0f},
            .collision_groups = PLAYERS,
            .collision_mask = ~PROJECTILES,
        });

        // the player's own shots pass through the player
        RigidBody *shot = rigid_body::create(*world, {
            .shape = box_shape,
            .object_layer = moving,
            .motion_type = MotionType::DYNAMIC,
            .position = {-10.0f, 3.0f, 0.0f},
            .collision_groups = PROJECTILES,
            .collision_mask = ~PLAYERS,
        });
        /* SNIPPET_END: groups */
        /* HIDE_START */
        steps(*world, 90);
        // the shot fell through the player to the ground
        if (!near(shot->position.y, 0.5f, 0.05f) || !near(player->position.y, 0.5f, 0.05f)) return 1;
        rigid_body::remove(*world, player);
        rigid_body::remove(*world, shot);
        /* HIDE_END */
    }

    {
        /* SNIPPET_START: materials */
        // each body's combine mode decides how friction and restitution combine in a contact. the
        // more restrictive mode wins
        RigidBody *bouncy = rigid_body::create(*world, {
            .shape = box_shape,
            .object_layer = moving,
            .motion_type = MotionType::DYNAMIC,
            .position = {12.0f, 4.0f, 0.0f},
            .friction = 0.9f,
            .restitution = 0.8f,
            .restitution_combine_mode = MaterialCombineMode::MAX,
        });
        /* SNIPPET_END: materials */
        /* HIDE_START */
        float highest_after_bounce = 0.0f;
        bool fell = false;
        for (int i = 0; i < 120; i++) {
            world::step(*world, 1.0f / 60.0f);
            if (bouncy->motion_properties.linear_velocity.y < -1.0f) fell = true;
            if (fell && bouncy->motion_properties.linear_velocity.y > 0.0f && bouncy->position.y > highest_after_bounce) highest_after_bounce = bouncy->position.y;
        }
        if (highest_after_bounce < 1.5f) return 1;
        rigid_body::remove(*world, bouncy);
        /* HIDE_END */
    }

    {
        /* SNIPPET_START: sensors */
        // a sensor reports its contacts but pushes nothing. use it as a trigger volume
        RigidBody *trigger = rigid_body::create(*world, {
            .shape = box_shape,
            .object_layer = moving,
            .motion_type = MotionType::KINEMATIC,
            .position = {14.0f, 0.5f, 0.0f},
            .sensor = true,
        });
        /* SNIPPET_END: sensors */
        /* HIDE_START */
        RigidBody *visitor = rigid_body::create(*world, {
            .shape = box_shape,
            .object_layer = moving,
            .motion_type = MotionType::DYNAMIC,
            .position = {14.0f, 3.0f, 0.0f},
        });
        bool reported = false;
        for (int i = 0; i < 90; i++) {
            world::step(*world, 1.0f / 60.0f);
            for (uint32_t c = 0; c < world->events.contacts_added.length; c++) {
                const ContactEvent &event = world->events.contacts_added[c];
                reported = reported || event.body_id_a == trigger->id || event.body_id_b == trigger->id;
            }
        }
        // it fell through the sensor to the ground, and the sensor saw it
        if (!reported || !near(visitor->position.y, 0.5f, 0.05f)) return 1;
        rigid_body::remove(*world, visitor);
        rigid_body::remove(*world, trigger);
        /* HIDE_END */
    }

    {
        /* SNIPPET_START: shape */
        // a new shape updates the mass and bounds. constraints keep their anchors where they were on
        // the body
        Ref<Shape> bigger = box::create({.half_extents = {1.0f, 1.0f, 1.0f}});
        rigid_body::set_shape(*world, body, bigger, true);

        // after changing a shape in place, pass it to set_shape for each body that uses it
        shapes::as<BoxShape>(bigger).half_extents = {1.0f, 2.0f, 1.0f};
        box::update(bigger);
        rigid_body::set_shape(*world, body, bigger, true);
        /* SNIPPET_END: shape */
        /* HIDE_START */
        if (body->shape != bigger || !near(body->aabb.max.y - body->aabb.min.y, 4.0f, 1e-3f)) return 1;
        rigid_body::set_shape(*world, body, box_shape, false);
        bigger = nullptr;
        /* HIDE_END */
    }

    {
        /* SNIPPET_START: layer */
        // moving a body to another object layer moves it to that layer's broadphase tree. then it
        // finds its pairs again
        rigid_body::set_object_layer(*world, body, not_moving);
        /* SNIPPET_END: layer */
        /* HIDE_START */
        if (body->object_layer != not_moving) return 1;
        /* HIDE_END */
    }

    world::destroy(world);
    ground_shape = nullptr;
    box_shape = nullptr;
    return 0;
}
