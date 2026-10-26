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

/** drops a body of the shape from 3 m onto the ground and returns where it settles */
static float rest_height(World &world, Shape *shape, float x) {
    RigidBody *body = rigid_body::create(world, {.shape = shape, .object_layer = moving, .motion_type = MotionType::DYNAMIC,
                                                             .position = {x, 3.0f, 0.0f}});
    for (int i = 0; i < 180; i++) world::step(world, 1.0f / 60.0f);
    const float height = body->position.y;
    rigid_body::remove(world, body);
    return height;
}

static bool near(float a, float b, float tolerance) { return a - b <= tolerance && b - a <= tolerance; }

int main() {
    dispatch::init();
    World *world = make_world();

    /* SNIPPET_START: plane */
    // an infinite ground: point . normal + constant = 0, the normal its front
    Ref<Shape> ground = plane::create({.plane = {{0.0f, 1.0f, 0.0f}, 0.0f}});
    rigid_body::create(*world, {.shape = ground, .object_layer = not_moving, .motion_type = MotionType::STATIC});
    /* SNIPPET_END: plane */

    /* SNIPPET_START: convex */
    // the convex primitives, centred on their origin; a body takes its mass from their volume and density
    Ref<Shape> ball = sphere::create({.radius = 0.5f});
    Ref<Shape> crate = box::create({.half_extents = {0.5f, 0.25f, 0.5f}, .density = 500.0f});
    // y is the long axis of a capsule and a cylinder; a capsule's radius is its convex radius
    Ref<Shape> pill = capsule::create({.half_height_of_cylinder = 0.5f, .radius = 0.25f});
    Ref<Shape> can = cylinder::create({.half_height = 0.4f, .radius = 0.3f});
    /* SNIPPET_END: convex */
    /* HIDE_START */
    if (!near(rest_height(*world, ball, 0.0f), 0.5f, 0.03f) || !near(rest_height(*world, crate, 2.0f), 0.25f, 0.03f) ||
        !near(rest_height(*world, can, 6.0f), 0.4f, 0.03f)) {
        return 1;
    }
    /* HIDE_END */

    /* SNIPPET_START: convex-hull */
    // any point cloud: the shape builds its hull, and any number of bodies can share the shape
    const float points[] = {
        -0.5f, 0.0f, -0.5f, 0.5f, 0.0f, -0.5f, 0.5f, 0.0f, 0.5f, -0.5f, 0.0f, 0.5f, 0.0f, 0.8f, 0.0f, 0.1f, 0.2f, 0.1f,
    };
    ConvexHullShape::BuildResult result;
    Ref<Shape> pyramid = convex_hull::create({.positions = points, .num_positions = 6}, &result);
    if (pyramid == nullptr) printf("no hull: build result %u\n", (uint32_t)result);
    Ref<Shape> heavy_pyramid = convex_hull::create({.positions = points, .num_positions = 6, .density = 4000.0f});
    /* SNIPPET_END: convex-hull */
    /* HIDE_START */
    // the interior point (0.1, 0.2, 0.1) is dropped: the hull is the five corners
    if (pyramid == nullptr || heavy_pyramid == nullptr || shapes::as<ConvexHullShape>(pyramid).data->num_points != 5) return 1;
    if (rest_height(*world, pyramid, 8.0f) > 0.05f) return 1;
    /* HIDE_END */

    /* SNIPPET_START: triangle-mesh */
    // static geometry from triangles, counter clockwise seen from the front; the shape builds its bvh and finds the
    // edges a sliding body must not catch on
    const float ramp_positions[] = {-2.0f, 0.0f, -2.0f, 2.0f, 0.0f, -2.0f, 2.0f, 1.0f, 2.0f, -2.0f, 1.0f, 2.0f};
    const uint32_t ramp_indices[] = {0, 2, 1, 0, 3, 2};
    Ref<Shape> ramp =
        triangle_mesh::create({.positions = ramp_positions, .num_positions = 4, .indices = ramp_indices, .num_indices = 6});
    rigid_body::create(*world, {.shape = ramp, .object_layer = not_moving, .motion_type = MotionType::STATIC,
                                .position = {20.0f, 0.0f, 0.0f}});
    /* SNIPPET_END: triangle-mesh */
    /* HIDE_START */
    // a ball dropped on the ramp's high end stays above the ground until it has rolled off
    {
        RigidBody *roller = rigid_body::create(*world, {.shape = ball, .object_layer = moving, .motion_type = MotionType::DYNAMIC,
                                                                    .position = {20.0f, 3.0f, 1.5f}});
        float lowest_on_ramp = 10.0f;
        for (int i = 0; i < 30; i++) {
            world::step(*world, 1.0f / 60.0f);
            if (roller->position.z > 0.0f && roller->position.y < lowest_on_ramp) lowest_on_ramp = roller->position.y;
        }
        if (lowest_on_ramp < 0.9f) return 1;
        rigid_body::remove(*world, roller);
    }
    /* HIDE_END */

    /* SNIPPET_START: compound */
    // several shapes as one body, each at its own pose; the shapes are borrowed and outlive the compound
    const CompoundShapeSettings::Child dumbbell_parts[] = {
        {.position = {-0.6f, 0.0f, 0.0f}, .quaternion = {0.0f, 0.0f, 0.0f, 1.0f}, .shape = ball},
        {.position = {0.6f, 0.0f, 0.0f}, .quaternion = {0.0f, 0.0f, 0.0f, 1.0f}, .shape = ball},
        {.position = {0.0f, 0.0f, 0.0f}, .quaternion = {0.0f, 0.0f, 0.70710678f, 0.70710678f}, .shape = pill},
    };
    Ref<Shape> dumbbell = compound::create({.children = dumbbell_parts, .num_children = 3});

    // a static compound is built once into a bvh: for many children that never change
    CompoundShapeSettings::Child fence_posts[16];
    for (uint32_t i = 0; i < 16; i++) fence_posts[i] = {.position = {(float)i, 0.4f, 0.0f}, .quaternion = {0.0f, 0.0f, 0.0f, 1.0f}, .shape = can};
    Ref<Shape> fence = static_compound::create({.children = fence_posts, .num_children = 16});
    rigid_body::create(*world, {.shape = fence, .object_layer = not_moving, .motion_type = MotionType::STATIC,
                                .position = {-20.0f, 0.0f, 0.0f}});
    /* SNIPPET_END: compound */
    /* HIDE_START */
    if (!near(rest_height(*world, dumbbell, 30.0f), 0.5f, 0.03f)) return 1;
    // a ball dropped on a post stops on its top
    if (!near(rest_height(*world, ball, -20.0f + 3.0f), 1.3f, 0.05f)) return 1;
    /* HIDE_END */

    /* SNIPPET_START: decorators */
    // wrappers over a shape they borrow: moved and turned within the body, scaled, or with the centre of mass moved
    Ref<Shape> raised = transformed::create({.shape = crate, .position = {0.0f, 1.0f, 0.0f}, .quaternion = {0.0f, 0.0f, 0.0f, 1.0f}});
    Ref<Shape> big_ball = scaled::create({.shape = ball, .scale = {2.0f, 2.0f, 2.0f}});
    // a low centre of mass keeps a tall box upright
    Ref<Shape> ballasted = offset_center_of_mass::create({.shape = crate, .offset = {0.0f, -0.2f, 0.0f}});
    /* SNIPPET_END: decorators */
    /* HIDE_START */
    if (!near(rest_height(*world, raised, 34.0f), -0.75f, 0.03f) || !near(rest_height(*world, big_ball, 38.0f), 1.0f, 0.03f) ||
        !near(rest_height(*world, ballasted, 42.0f), 0.25f, 0.03f)) {
        return 1;
    }
    /* HIDE_END */

    /* SNIPPET_START: empty */
    // a body with no geometry: a point for constraints to hold, or one waiting for its shape
    Ref<Shape> nothing = empty::create();
    rigid_body::create(*world, {.shape = nothing, .object_layer = moving, .motion_type = MotionType::STATIC});
    /* SNIPPET_END: empty */

    /* SNIPPET_START: freeing */
    // a shape is freed when the last Ref to it lets go: the world lets go of its bodies' shapes, a wrapper of the shape
    // it wraps, and the Refs here as they leave scope
    world::destroy(world);
    /* SNIPPET_END: freeing */
    /* HIDE_START */
    nothing = nullptr;
    raised = nullptr;
    big_ball = nullptr;
    ballasted = nullptr;
    dumbbell = nullptr;
    fence = nullptr;
    ramp = nullptr;
    pyramid = nullptr;
    heavy_pyramid = nullptr;
    ball = nullptr;
    crate = nullptr;
    pill = nullptr;
    can = nullptr;
    ground = nullptr;
    if (util::allocated_blocks() != 0) return 1;
    /* HIDE_END */
    return 0;
}
