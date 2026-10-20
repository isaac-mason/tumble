#include <stdio.h>

#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/constraints/cone_constraint.h"
#include "tumble/physics/constraints/distance_constraint.h"
#include "tumble/physics/constraints/fixed_constraint.h"
#include "tumble/physics/constraints/hinge_constraint.h"
#include "tumble/physics/constraints/point_constraint.h"
#include "tumble/physics/constraints/six_dof_constraint.h"
#include "tumble/physics/constraints/slider_constraint.h"
#include "tumble/physics/constraints/swing_twist_constraint.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/dispatch.h"
#include "tumble/physics/shapes/empty.h"
#include "tumble/physics/shapes/sphere.h"
#include "tumble/physics/world.h"

using namespace tumble;

static bool near(float a, float b, float tolerance) { return a - b <= tolerance && b - a <= tolerance; }

static void steps(world::World &world, int count) {
    for (int i = 0; i < count; i++) world::step(world, 1.0f / 60.0f);
}

int main() {
    dispatch::init();
    world_settings::WorldSettings settings = world_settings::create();
    const uint32_t moving = world_settings::add_object_layer(settings, world_settings::add_broadphase_layer(settings));
    const uint32_t not_moving = world_settings::add_object_layer(settings, world_settings::add_broadphase_layer(settings));
    world_settings::enable_collision(settings, moving, not_moving);
    world::World *world = world::create(settings);
    world_settings::release(settings);
    shapes::Shape *nothing = empty::create();
    shapes::Shape *door_shape = box::create({.half_extents = {0.5f, 1.0f, 0.05f}});
    shapes::Shape *ball_shape = sphere::create({.radius = 0.25f});
    shapes::Shape *block_shape = box::create({.half_extents = {0.5f, 0.1f, 0.5f}});

    {
        /* SNIPPET_START: hinge */
        // a door on a hinge along y at its left edge, opening a quarter turn at most, driven open by its motor. a motor's
        // torque is unbounded unless limited, and an unbounded motor driving into a limit pushes past it: give it a limit
        rigid_body::RigidBody *frame = rigid_body::create(*world, {.shape = nothing, .object_layer = not_moving,
                                                                   .motion_type = motion_type::MOTION_STATIC, .position = {0.0f, 1.0f, 0.0f}});
        rigid_body::RigidBody *door = rigid_body::create(*world, {.shape = door_shape, .object_layer = moving,
                                                                  .motion_type = motion_type::MOTION_DYNAMIC, .position = {0.5f, 1.0f, 0.0f}});
        const constraint_id::ConstraintId hinge_id = hinge_constraint::create(*world, frame, door,
                                                                              {.point_a = {0.0f, 1.0f, 0.0f},
                                                                               .point_b = {0.0f, 1.0f, 0.0f},
                                                                               .hinge_axis_a = {0.0f, 1.0f, 0.0f},
                                                                               .hinge_axis_b = {0.0f, 1.0f, 0.0f},
                                                                               .normal_axis_a = {1.0f, 0.0f, 0.0f},
                                                                               .normal_axis_b = {1.0f, 0.0f, 0.0f},
                                                                               .limits_min = 0.0f,
                                                                               .limits_max = 0.5f * scalar::PI,
                                                                               .motor_settings = {.min_torque_limit = -100.0f, .max_torque_limit = 100.0f}});

        // the id names it; get gives its record, which changes it, until the next create of a hinge
        hinge_constraint::HingeConstraint *hinge = hinge_constraint::get(*world, hinge_id);
        hinge_constraint::set_motor_state(*hinge, motor_settings::VELOCITY);
        hinge_constraint::set_target_angular_velocity(*hinge, 2.0f);
        /* SNIPPET_END: hinge */
        /* HIDE_START */
        steps(*world, 90);
        if (!near(hinge_constraint::get_current_angle(*hinge_constraint::get(*world, hinge_id)), 0.5f * scalar::PI, 0.05f)) return 1;
        /* HIDE_END */

        /* SNIPPET_START: remove */
        // gone, or with either body: a stale id resolves to nothing
        hinge_constraint::remove(*world, hinge_id);
        /* SNIPPET_END: remove */
        /* HIDE_START */
        if (hinge_constraint::get(*world, hinge_id) != nullptr || hinge_constraint::remove(*world, hinge_id)) return 1;
        rigid_body::remove(*world, door);
        rigid_body::remove(*world, frame);
        /* HIDE_END */
    }

    {
        /* SNIPPET_START: point */
        // a pendulum: a ball held 2 m below a point it swings about. a static body with an empty shape is a fixed anchor
        rigid_body::RigidBody *anchor = rigid_body::create(*world, {.shape = nothing, .object_layer = not_moving,
                                                                    .motion_type = motion_type::MOTION_STATIC, .position = {10.0f, 5.0f, 0.0f}});
        rigid_body::RigidBody *bob = rigid_body::create(*world, {.shape = ball_shape, .object_layer = moving,
                                                                 .motion_type = motion_type::MOTION_DYNAMIC, .position = {12.0f, 5.0f, 0.0f}});
        point_constraint::create(*world, anchor, bob, {.point_a = {10.0f, 5.0f, 0.0f}, .point_b = {10.0f, 5.0f, 0.0f}});
        /* SNIPPET_END: point */
        /* HIDE_START */
        steps(*world, 90);
        const float dx = bob->position.x - 10.0f, dy = bob->position.y - 5.0f;
        if (!near(scalar::sqrt(dx * dx + dy * dy), 2.0f, 0.02f) || bob->position.y > 4.9f) return 1;
        /* HIDE_END */

        /* SNIPPET_START: distance */
        // a bungee: no further than 3 m, pulled back by a spring; min and max of -1 keep the distance it starts at
        rigid_body::RigidBody *jumper = rigid_body::create(*world, {.shape = ball_shape, .object_layer = moving,
                                                                    .motion_type = motion_type::MOTION_DYNAMIC, .position = {14.0f, 5.0f, 0.0f}});
        distance_constraint::create(*world, anchor, jumper,
                                    {.point_a = {10.0f, 5.0f, 0.0f},
                                     .point_b = {14.0f, 5.0f, 0.0f},
                                     .min_distance = 0.0f,
                                     .max_distance = 3.0f,
                                     .spring_settings = {.mode = spring_settings::FREQUENCY_AND_DAMPING, .frequency_or_stiffness = 2.0f, .damping = 0.5f}});
        /* SNIPPET_END: distance */
        /* HIDE_START */
        steps(*world, 240);
        const float jx = jumper->position.x - 10.0f, jy = jumper->position.y - 5.0f;
        if (scalar::sqrt(jx * jx + jy * jy) > 3.3f) return 1;
        /* HIDE_END */
    }

    {
        /* SNIPPET_START: slider */
        // a lift: a platform that slides along y only, its position motor holding it 2 m up the rail. a position motor
        // drives through a spring, which a load sags: this one is stiff enough (10 Hz) to sag by millimetres
        rigid_body::RigidBody *rail = rigid_body::create(*world, {.shape = nothing, .object_layer = not_moving,
                                                                  .motion_type = motion_type::MOTION_STATIC, .position = {20.0f, 0.0f, 0.0f}});
        rigid_body::RigidBody *platform = rigid_body::create(*world, {.shape = block_shape, .object_layer = moving,
                                                                      .motion_type = motion_type::MOTION_DYNAMIC, .position = {20.0f, 0.0f, 0.0f}});
        const constraint_id::ConstraintId lift_id = slider_constraint::create(*world, rail, platform,
                                                                              {.slider_axis_a = {0.0f, 1.0f, 0.0f},
                                                                               .slider_axis_b = {0.0f, 1.0f, 0.0f},
                                                                               .normal_axis_a = {1.0f, 0.0f, 0.0f},
                                                                               .normal_axis_b = {1.0f, 0.0f, 0.0f},
                                                                               .auto_detect_point = true,
                                                                               .limits_min = 0.0f,
                                                                               .limits_max = 5.0f,
                                                                               .motor_settings = {.spring = {.frequency_or_stiffness = 10.0f, .damping = 1.0f}}});
        slider_constraint::SliderConstraint *lift = slider_constraint::get(*world, lift_id);
        slider_constraint::set_motor_state(*lift, motor_settings::POSITION);
        slider_constraint::set_target_position(*lift, 2.0f);
        /* SNIPPET_END: slider */
        /* HIDE_START */
        steps(*world, 240);
        if (!near(slider_constraint::get_current_position(*world, *slider_constraint::get(*world, lift_id)), 2.0f, 0.005f)) return 1;
        if (!near(platform->position.y, 2.0f, 0.005f)) return 1;
        /* HIDE_END */
    }

    {
        /* SNIPPET_START: fixed */
        // a weld: two bodies held as they are, moving as one
        rigid_body::RigidBody *left = rigid_body::create(*world, {.shape = block_shape, .object_layer = moving,
                                                                  .motion_type = motion_type::MOTION_DYNAMIC, .position = {30.0f, 3.0f, 0.0f}});
        rigid_body::RigidBody *right = rigid_body::create(*world, {.shape = block_shape, .object_layer = moving,
                                                                   .motion_type = motion_type::MOTION_DYNAMIC, .position = {31.0f, 3.0f, 0.0f}});
        fixed_constraint::create(*world, left, right, {.auto_detect_point = true});
        /* SNIPPET_END: fixed */
        /* HIDE_START */
        steps(*world, 60);
        const float apart_x = right->position.x - left->position.x, apart_y = right->position.y - left->position.y;
        if (!near(apart_x, 1.0f, 0.01f) || !near(apart_y, 0.0f, 0.01f) || left->position.y > 2.5f) return 1;
        /* HIDE_END */
    }

    {
        /* SNIPPET_START: ragdoll */
        // a shoulder: the arm swings within a 45 degree cone about the shoulder's twist axis (x here), twisting up to 30
        // degrees either way. a cone constraint is the same without the twist limits
        rigid_body::RigidBody *torso = rigid_body::create(*world, {.shape = nothing, .object_layer = not_moving,
                                                                   .motion_type = motion_type::MOTION_STATIC, .position = {40.0f, 3.0f, 0.0f}});
        rigid_body::RigidBody *arm = rigid_body::create(*world, {.shape = ball_shape, .object_layer = moving,
                                                                 .motion_type = motion_type::MOTION_DYNAMIC, .position = {40.5f, 3.0f, 0.0f}});
        swing_twist_constraint::create(*world, torso, arm,
                                       {.point_a = {40.0f, 3.0f, 0.0f},
                                        .point_b = {40.0f, 3.0f, 0.0f},
                                        .twist_axis_a = {1.0f, 0.0f, 0.0f},
                                        .plane_axis_a = {0.0f, 0.0f, 1.0f},
                                        .twist_axis_b = {1.0f, 0.0f, 0.0f},
                                        .plane_axis_b = {0.0f, 0.0f, 1.0f},
                                        .normal_half_cone_angle = 0.25f * scalar::PI,
                                        .plane_half_cone_angle = 0.25f * scalar::PI,
                                        .twist_min_angle = -scalar::PI / 6.0f,
                                        .twist_max_angle = scalar::PI / 6.0f});
        /* SNIPPET_END: ragdoll */
        /* HIDE_START */
        steps(*world, 120);
        // gravity pulls the arm down until the cone stops it 45 degrees below the twist axis
        const float ax = arm->position.x - 40.0f, ay = arm->position.y - 3.0f;
        if (!near(scalar::atan2(-ay, ax), 0.25f * scalar::PI, 0.05f)) return 1;
        rigid_body::RigidBody *cone_arm = rigid_body::create(*world, {.shape = ball_shape, .object_layer = moving,
                                                                      .motion_type = motion_type::MOTION_DYNAMIC, .position = {40.5f, 6.0f, 0.0f}});
        rigid_body::RigidBody *cone_anchor = rigid_body::create(*world, {.shape = nothing, .object_layer = not_moving,
                                                                         .motion_type = motion_type::MOTION_STATIC, .position = {40.0f, 6.0f, 0.0f}});
        /* HIDE_END */

        /* SNIPPET_START: cone */
        cone_constraint::create(*world, cone_anchor, cone_arm,
                                {.point_a = {40.0f, 6.0f, 0.0f},
                                 .twist_axis_a = {1.0f, 0.0f, 0.0f},
                                 .point_b = {40.0f, 6.0f, 0.0f},
                                 .twist_axis_b = {1.0f, 0.0f, 0.0f},
                                 .half_cone_angle = 0.25f * scalar::PI});
        /* SNIPPET_END: cone */
        /* HIDE_START */
        steps(*world, 120);
        const float cx = cone_arm->position.x - 40.0f, cy = cone_arm->position.y - 6.0f;
        if (!near(scalar::atan2(-cy, cx), 0.25f * scalar::PI, 0.05f)) return 1;
        /* HIDE_END */
    }

    {
        /* SNIPPET_START: six-dof */
        // every axis its own: here free to slide along x within a metre either way and to turn about y, the rest locked
        rigid_body::RigidBody *base = rigid_body::create(*world, {.shape = nothing, .object_layer = not_moving,
                                                                  .motion_type = motion_type::MOTION_STATIC, .position = {50.0f, 3.0f, 0.0f}});
        rigid_body::RigidBody *cart = rigid_body::create(*world, {.shape = block_shape, .object_layer = moving,
                                                                  .motion_type = motion_type::MOTION_DYNAMIC, .position = {50.0f, 3.0f, 0.0f}});
        six_dof_constraint::SixDOFConstraintSettings rails;
        rails.point_a = {50.0f, 3.0f, 0.0f};
        rails.point_b = {50.0f, 3.0f, 0.0f};
        for (uint32_t axis = 0; axis < six_dof_constraint::NUM; axis++) rails.limit_min[axis] = rails.limit_max[axis] = 0.0f;
        rails.limit_min[six_dof_constraint::TRANSLATION_X] = -1.0f;
        rails.limit_max[six_dof_constraint::TRANSLATION_X] = 1.0f;
        rails.limit_min[six_dof_constraint::ROTATION_Y] = -scalar::PI;
        rails.limit_max[six_dof_constraint::ROTATION_Y] = scalar::PI;
        six_dof_constraint::create(*world, base, cart, rails);
        rigid_body::set_linear_velocity(*world, cart, vec3f::make(5.0f, 0.0f, 0.0f));
        /* SNIPPET_END: six-dof */
        /* HIDE_START */
        steps(*world, 60);
        // held up against gravity, stopped at the end of its rail
        if (!near(cart->position.y, 3.0f, 0.02f) || cart->position.x > 51.05f) return 1;
        /* HIDE_END */
    }

    world::destroy(world);
    box::destroy(block_shape);
    sphere::destroy(ball_shape);
    box::destroy(door_shape);
    empty::destroy(nothing);
    return 0;
}
