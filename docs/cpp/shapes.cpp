#include <stdio.h>

#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/capsule.h"
#include "tumble/physics/shapes/compound.h"
#include "tumble/physics/shapes/convex_hull.h"
#include "tumble/physics/shapes/cylinder.h"
#include "tumble/physics/shapes/dispatch.h"
#include "tumble/physics/shapes/empty.h"
#include "tumble/physics/shapes/offset_center_of_mass.h"
#include "tumble/physics/shapes/plane.h"
#include "tumble/physics/shapes/scaled.h"
#include "tumble/physics/shapes/sphere.h"
#include "tumble/physics/shapes/static_compound.h"
#include "tumble/physics/shapes/transformed.h"
#include "tumble/physics/shapes/triangle_mesh.h"
#include "tumble/physics/shapes/utils/triangle_mesh_builder.h"
#include "tumble/physics/world.h"
#include "tumble/util/alloc.h"

using namespace tumble;

static uint32_t moving;
static uint32_t not_moving;

static world::World *make_world() {
    world_settings::WorldSettings settings = world_settings::create();
    moving = world_settings::add_object_layer(settings, world_settings::add_broadphase_layer(settings));
    not_moving = world_settings::add_object_layer(settings, world_settings::add_broadphase_layer(settings));
    world_settings::enable_collision(settings, moving, moving);
    world_settings::enable_collision(settings, moving, not_moving);
    world::World *world = world::create(settings);
    world_settings::release(settings);
    return world;
}

/** drops a body of the shape from 3 m onto the ground and returns where it settles */
static float rest_height(world::World &world, shapes::Shape *shape, float x) {
    rigid_body::RigidBody *body = rigid_body::create(world, {.shape = shape, .object_layer = moving, .motion_type = motion_type::MOTION_DYNAMIC,
                                                             .position = {x, 3.0f, 0.0f}});
    for (int i = 0; i < 180; i++) world::step(world, 1.0f / 60.0f);
    const float height = body->position.y;
    rigid_body::remove(world, body);
    return height;
}

static bool near(float a, float b, float tolerance) { return a - b <= tolerance && b - a <= tolerance; }

int main() {
    dispatch::init();
    world::World *world = make_world();

    /* SNIPPET_START: plane */
    // an infinite ground: point . normal + constant = 0, the normal its front
    shapes::Shape *ground = plane::create({.plane = {{0.0f, 1.0f, 0.0f}, 0.0f}});
    rigid_body::create(*world, {.shape = ground, .object_layer = not_moving, .motion_type = motion_type::MOTION_STATIC});
    /* SNIPPET_END: plane */

    /* SNIPPET_START: convex */
    // the convex primitives, centred on their origin; a body takes its mass from their volume and density
    shapes::Shape *ball = sphere::create({.radius = 0.5f});
    shapes::Shape *crate = box::create({.half_extents = {0.5f, 0.25f, 0.5f}, .density = 500.0f});
    // y is the long axis of a capsule and a cylinder; a capsule's radius is its convex radius
    shapes::Shape *pill = capsule::create({.half_height_of_cylinder = 0.5f, .radius = 0.25f});
    shapes::Shape *can = cylinder::create({.half_height = 0.4f, .radius = 0.3f});
    /* SNIPPET_END: convex */
    /* HIDE_START */
    if (!near(rest_height(*world, ball, 0.0f), 0.5f, 0.03f) || !near(rest_height(*world, crate, 2.0f), 0.25f, 0.03f) ||
        !near(rest_height(*world, can, 6.0f), 0.4f, 0.03f)) {
        return 1;
    }
    /* HIDE_END */

    /* SNIPPET_START: convex-hull */
    // any point cloud: build finds its hull, which any number of shapes can share
    const float points[] = {
        -0.5f, 0.0f, -0.5f, 0.5f, 0.0f, -0.5f, 0.5f, 0.0f, 0.5f, -0.5f, 0.0f, 0.5f, 0.0f, 0.8f, 0.0f, 0.1f, 0.2f, 0.1f,
    };
    convex_hull::BuildResult result;
    convex_hull::ConvexHullData *pyramid_hull = convex_hull::build({.positions = points, .num_positions = 6}, &result);
    if (pyramid_hull == nullptr) printf("no hull: build result %u\n", (uint32_t)result);
    shapes::Shape *pyramid = convex_hull::create({.data = pyramid_hull});
    shapes::Shape *heavy_pyramid = convex_hull::create({.data = pyramid_hull, .density = 4000.0f});
    /* SNIPPET_END: convex-hull */
    /* HIDE_START */
    // the interior point (0.1, 0.2, 0.1) is dropped: the hull is the five corners
    if (pyramid_hull == nullptr || pyramid_hull->num_points != 5) return 1;
    if (rest_height(*world, pyramid, 8.0f) > 0.05f) return 1;
    /* HIDE_END */

    /* SNIPPET_START: triangle-mesh */
    // static geometry from triangles, counter clockwise seen from the front; the builder makes the bvh and finds the
    // edges a sliding body must not catch on
    const float ramp_positions[] = {-2.0f, 0.0f, -2.0f, 2.0f, 0.0f, -2.0f, 2.0f, 1.0f, 2.0f, -2.0f, 1.0f, 2.0f};
    const uint32_t ramp_indices[] = {0, 2, 1, 0, 3, 2};
    triangle_mesh_data::TriangleMeshData *ramp_data =
        triangle_mesh_builder::build({.positions = ramp_positions, .num_positions = 4, .indices = ramp_indices, .num_indices = 6});
    shapes::Shape *ramp = triangle_mesh::create({.data = ramp_data});
    rigid_body::create(*world, {.shape = ramp, .object_layer = not_moving, .motion_type = motion_type::MOTION_STATIC,
                                .position = {20.0f, 0.0f, 0.0f}});
    /* SNIPPET_END: triangle-mesh */
    /* HIDE_START */
    // a ball dropped on the ramp's high end stays above the ground until it has rolled off
    {
        rigid_body::RigidBody *roller = rigid_body::create(*world, {.shape = ball, .object_layer = moving, .motion_type = motion_type::MOTION_DYNAMIC,
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
    const compound::CompoundShapeChild dumbbell_parts[] = {
        {.position = {-0.6f, 0.0f, 0.0f}, .quaternion = {0.0f, 0.0f, 0.0f, 1.0f}, .shape = ball},
        {.position = {0.6f, 0.0f, 0.0f}, .quaternion = {0.0f, 0.0f, 0.0f, 1.0f}, .shape = ball},
        {.position = {0.0f, 0.0f, 0.0f}, .quaternion = {0.0f, 0.0f, 0.70710678f, 0.70710678f}, .shape = pill},
    };
    shapes::Shape *dumbbell = compound::create({.children = dumbbell_parts, .num_children = 3});

    // a static compound is built once into a bvh: for many children that never change
    compound::CompoundShapeChild fence_posts[16];
    for (uint32_t i = 0; i < 16; i++) fence_posts[i] = {.position = {(float)i, 0.4f, 0.0f}, .quaternion = {0.0f, 0.0f, 0.0f, 1.0f}, .shape = can};
    shapes::Shape *fence = static_compound::create({.children = fence_posts, .num_children = 16});
    rigid_body::create(*world, {.shape = fence, .object_layer = not_moving, .motion_type = motion_type::MOTION_STATIC,
                                .position = {-20.0f, 0.0f, 0.0f}});
    /* SNIPPET_END: compound */
    /* HIDE_START */
    if (!near(rest_height(*world, dumbbell, 30.0f), 0.5f, 0.03f)) return 1;
    // a ball dropped on a post stops on its top
    if (!near(rest_height(*world, ball, -20.0f + 3.0f), 1.3f, 0.05f)) return 1;
    /* HIDE_END */

    /* SNIPPET_START: decorators */
    // wrappers over a shape they borrow: moved and turned within the body, scaled, or with the centre of mass moved
    shapes::Shape *raised = transformed::create({.shape = crate, .position = {0.0f, 1.0f, 0.0f}, .quaternion = {0.0f, 0.0f, 0.0f, 1.0f}});
    shapes::Shape *big_ball = scaled::create({.shape = ball, .scale = {2.0f, 2.0f, 2.0f}});
    // a low centre of mass keeps a tall box upright
    shapes::Shape *ballasted = offset_center_of_mass::create({.shape = crate, .offset = {0.0f, -0.2f, 0.0f}});
    /* SNIPPET_END: decorators */
    /* HIDE_START */
    if (!near(rest_height(*world, raised, 34.0f), -0.75f, 0.03f) || !near(rest_height(*world, big_ball, 38.0f), 1.0f, 0.03f) ||
        !near(rest_height(*world, ballasted, 42.0f), 0.25f, 0.03f)) {
        return 1;
    }
    /* HIDE_END */

    /* SNIPPET_START: empty */
    // a body with no geometry: a point for constraints to hold, or one waiting for its shape
    shapes::Shape *nothing = empty::create();
    rigid_body::create(*world, {.shape = nothing, .object_layer = moving, .motion_type = motion_type::MOTION_STATIC});
    /* SNIPPET_END: empty */

    /* SNIPPET_START: freeing */
    // shapes, hulls and meshes are yours: free them after the bodies that use them, wrappers before what they wrap
    world::destroy(world);
    empty::destroy(nothing);
    transformed::destroy(raised);
    scaled::destroy(big_ball);
    offset_center_of_mass::destroy(ballasted);
    compound::destroy(dumbbell);
    static_compound::destroy(fence);
    triangle_mesh::destroy(ramp);
    triangle_mesh_data::destroy(ramp_data);
    convex_hull::destroy(pyramid);
    convex_hull::destroy(heavy_pyramid);
    convex_hull::release(pyramid_hull);
    sphere::destroy(ball);
    box::destroy(crate);
    capsule::destroy(pill);
    cylinder::destroy(can);
    plane::destroy(ground);
    /* SNIPPET_END: freeing */
    /* HIDE_START */
    if (util::allocated_blocks() != 0) return 1;
    /* HIDE_END */
    return 0;
}
