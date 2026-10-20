#include "tumble/physics/debug.h"

#include "tumble/physics/character/kcc.h"
#include "tumble/physics/constraints/distance_constraint.h"
#include "tumble/physics/constraints/hinge_constraint.h"
#include "tumble/physics/constraints/point_constraint.h"
#include "tumble/physics/shapes/compound.h"
#include "tumble/physics/shapes/convex_hull.h"
#include "tumble/physics/shapes/cylinder.h"
#include "tumble/physics/shapes/empty.h"
#include "tumble/physics/shapes/offset_center_of_mass.h"
#include "tumble/physics/shapes/plane.h"
#include "tumble/physics/shapes/scaled.h"
#include "tumble/physics/shapes/static_compound.h"
#include "tumble/physics/shapes/transformed.h"
#include "tumble/physics/shapes/triangle_mesh.h"
#include "tumble/physics/shapes/utils/triangle_mesh_builder.h"
#include "world_test_scene.h"
#include "test_math.h"

using rigid_body::RigidBody;

namespace {

constexpr Float4 IDENTITY = {0.0f, 0.0f, 0.0f, 1.0f};

// the ts's tessellation: a sphere is 5 circles of 32 segments; a capsule 2 circles, 8 sides and 12 arcs of 16; a
// cylinder 2 circles and 4 sides; a plane 11 lines each way
constexpr uint32_t SPHERE_LINES = 5 * 32;
constexpr uint32_t BOX_LINES = 12;
constexpr uint32_t CAPSULE_LINES = 2 * 32 + 8 + 12 * 16;
constexpr uint32_t CYLINDER_LINES = 2 * 32 + 4;
constexpr uint32_t PLANE_LINES = 2 * 11;
// a cube hull's 6 quads
constexpr uint32_t CUBE_HULL_LINES = 6 * 4;
// a two-triangle quad mesh
constexpr uint32_t QUAD_MESH_LINES = 2 * 3;
// a cross is 3 lines
constexpr uint32_t CROSS_LINES = 3;

const float CUBE_POINTS[] = {-1, -1, -1, 1, -1, -1, 1, 1, -1, -1, 1, -1, -1, -1, 1, 1, -1, 1, 1, 1, 1, -1, 1, 1};
const float QUAD_POSITIONS[] = {-1, 0, -1, 1, 0, -1, 1, 0, 1, -1, 0, 1};
const uint32_t QUAD_INDICES[] = {0, 2, 1, 0, 3, 2};

shapes::Shape *make_quad_mesh() {
    triangle_mesh_builder::TriangleMeshBuilderSettings settings;
    settings.positions = QUAD_POSITIONS;
    settings.num_positions = 4;
    settings.indices = QUAD_INDICES;
    settings.num_indices = 6;
    return triangle_mesh::create({.data = triangle_mesh_builder::build(settings)});
}

shapes::Shape *make_cube_hull() { return convex_hull::create({.data = convex_hull::build({.positions = CUBE_POINTS, .num_positions = 8})}); }

Float3 vertex_position(const debug::DebugBuffers &buffers, uint32_t vertex) {
    const float *v = buffers.lines.data + vertex * debug::VERTEX_STRIDE;
    return {v[0], v[1], v[2]};
}

Float3 vertex_color(const debug::DebugBuffers &buffers, uint32_t vertex) {
    const float *v = buffers.lines.data + vertex * debug::VERTEX_STRIDE + 3;
    return {v[0], v[1], v[2]};
}

/** the shape's line count drawn alone at the origin, the buffers cleared first */
uint32_t shape_lines(debug::DebugBuffers &buffers, const shapes::Shape *shape) {
    debug::clear(buffers);
    debug::shape(buffers, shape, vec3f::make(0.0f, 0.0f, 0.0f), quatf::load(IDENTITY), {});
    return buffers.line_count;
}

/** the largest |component| over every line vertex */
Float3 max_abs_extent(const debug::DebugBuffers &buffers) {
    Float3 extent = {0.0f, 0.0f, 0.0f};
    for (uint32_t i = 0; i < buffers.line_count * 2; i++) {
        const Float3 p = vertex_position(buffers, i);
        extent.x = scalar::max(extent.x, scalar::abs(p.x));
        extent.y = scalar::max(extent.y, scalar::abs(p.y));
        extent.z = scalar::max(extent.z, scalar::abs(p.z));
    }
    return extent;
}

}  // namespace

void test_debug_buffers_start_empty_and_clear(TestCtx &ctx) {
    dispatch::init();
    debug::DebugBuffers *buffers = debug::create();
    CHECK(ctx, buffers != nullptr);
    CHECK(ctx, buffers->line_count == 0 && buffers->lines.length == 0);
    CHECK(ctx, buffers->triangle_count == 0 && buffers->triangles.length == 0);

    shapes::Shape *box_shape = box::create({.half_extents = {1.0f, 1.0f, 1.0f}});
    debug::shape(*buffers, box_shape, vec3f::make(0.0f, 0.0f, 0.0f), quatf::load(IDENTITY), {});
    CHECK(ctx, buffers->line_count == BOX_LINES);
    CHECK(ctx, buffers->lines.length == BOX_LINES * debug::LINE_STRIDE);
    debug::clear(*buffers);
    CHECK(ctx, buffers->line_count == 0 && buffers->lines.length == 0);
    CHECK(ctx, buffers->lines.capacity >= BOX_LINES * debug::LINE_STRIDE);
    debug::release(buffers);
}

void test_debug_shape_line_counts_per_type(TestCtx &ctx) {
    dispatch::init();
    debug::DebugBuffers *buffers = debug::create();

    shapes::Shape *sphere_shape = sphere::create({.radius = 0.5f});
    shapes::Shape *box_shape = box::create({.half_extents = {1.0f, 2.0f, 3.0f}});
    shapes::Shape *capsule_shape = capsule::create({.half_height_of_cylinder = 1.0f, .radius = 0.5f});
    shapes::Shape *cylinder_shape = cylinder::create({.half_height = 1.0f, .radius = 0.5f});
    shapes::Shape *plane_shape = plane::create({.plane = {{0.0f, 1.0f, 0.0f}, 0.0f}, .half_extent = 10.0f});
    shapes::Shape *hull_shape = make_cube_hull();
    shapes::Shape *mesh_shape = make_quad_mesh();
    const compound::CompoundShapeChild children[2] = {
        {{2.0f, 0.0f, 0.0f}, IDENTITY, box_shape},
        {{-2.0f, 0.0f, 0.0f}, IDENTITY, sphere_shape},
    };
    shapes::Shape *compound_shape = compound::create({.children = children, .num_children = 2});
    shapes::Shape *static_compound_shape = static_compound::create({.children = children, .num_children = 2});
    shapes::Shape *transformed_shape = transformed::create({.shape = box_shape, .position = {0.0f, 1.0f, 0.0f}});
    shapes::Shape *scaled_shape = scaled::create({.shape = capsule_shape, .scale = {2.0f, 2.0f, 2.0f}});
    shapes::Shape *offset_shape = offset_center_of_mass::create({.shape = cylinder_shape, .offset = {0.0f, 0.5f, 0.0f}});
    shapes::Shape *empty_shape = empty::create();

    CHECK(ctx, hull_shape->convex_hull.data->faces.length == 6);
    CHECK(ctx, shape_lines(*buffers, sphere_shape) == SPHERE_LINES);
    CHECK(ctx, shape_lines(*buffers, box_shape) == BOX_LINES);
    CHECK(ctx, shape_lines(*buffers, capsule_shape) == CAPSULE_LINES);
    CHECK(ctx, shape_lines(*buffers, cylinder_shape) == CYLINDER_LINES);
    CHECK(ctx, shape_lines(*buffers, plane_shape) == PLANE_LINES);
    CHECK(ctx, shape_lines(*buffers, hull_shape) == CUBE_HULL_LINES);
    CHECK(ctx, shape_lines(*buffers, mesh_shape) == QUAD_MESH_LINES);
    CHECK(ctx, shape_lines(*buffers, compound_shape) == BOX_LINES + SPHERE_LINES);
    CHECK(ctx, shape_lines(*buffers, static_compound_shape) == BOX_LINES + SPHERE_LINES);
    CHECK(ctx, shape_lines(*buffers, transformed_shape) == BOX_LINES);
    CHECK(ctx, shape_lines(*buffers, scaled_shape) == CAPSULE_LINES);
    CHECK(ctx, shape_lines(*buffers, offset_shape) == CYLINDER_LINES);
    CHECK(ctx, shape_lines(*buffers, empty_shape) == 0);
    // nothing is drawn filled
    CHECK(ctx, buffers->triangle_count == 0);
    debug::release(buffers);
}

void test_debug_box_corners_under_pose(TestCtx &ctx) {
    dispatch::init();
    debug::DebugBuffers *buffers = debug::create();
    shapes::Shape *box_shape = box::create({.half_extents = {1.0f, 2.0f, 3.0f}});

    // a quarter turn about y maps local (x, y, z) to (z, y, -x)
    Float4 rotation = stored_axis_angle(vec3f::make(0.0f, 1.0f, 0.0f), scalar::PI * 0.5f);
    const Float3 position = {10.0f, 0.0f, 0.0f};
    debug::shape(*buffers, box_shape, vec3f::load(position), quatf::load(rotation), {.color = {0.25f, 0.5f, 0.75f}});
    CHECK(ctx, buffers->line_count == BOX_LINES);
    // the first edge runs from corner (-1, -2, -3) to (1, -2, -3)
    CHECK(ctx, near3(vertex_position(*buffers, 0), 10.0f - 3.0f, -2.0f, 1.0f, 1e-5f));
    CHECK(ctx, near3(vertex_position(*buffers, 1), 10.0f - 3.0f, -2.0f, -1.0f, 1e-5f));
    CHECK(ctx, near3(vertex_color(*buffers, 0), 0.25f, 0.5f, 0.75f, 0.0f));
    CHECK(ctx, near3(vertex_color(*buffers, 1), 0.25f, 0.5f, 0.75f, 0.0f));
    // every edge has the length of one of the box's sides
    for (uint32_t i = 0; i < BOX_LINES; i++) {
        const float length = vec3f::distance(vec3f::load(vertex_position(*buffers, 2 * i)), vec3f::load(vertex_position(*buffers, 2 * i + 1)));
        CHECK(ctx, scalar::abs(length - 2.0f) < 1e-5f || scalar::abs(length - 4.0f) < 1e-5f || scalar::abs(length - 6.0f) < 1e-5f);
    }
    debug::release(buffers);
}

void test_debug_round_shapes_lie_on_their_surface(TestCtx &ctx) {
    dispatch::init();
    debug::DebugBuffers *buffers = debug::create();
    const Float3 center = {1.0f, 2.0f, 3.0f};

    // every sphere vertex at the radius from the centre
    debug::shape(*buffers, sphere::create({.radius = 0.75f}), vec3f::load(center), quatf::load(IDENTITY), {});
    bool on_sphere = true;
    for (uint32_t i = 0; i < buffers->line_count * 2; i++) {
        on_sphere = on_sphere && scalar::abs(vec3f::distance(vec3f::load(vertex_position(*buffers, i)), vec3f::load(center)) - 0.75f) < 1e-5f;
    }
    CHECK(ctx, on_sphere);

    // every capsule vertex at the radius from the core segment, the poles included
    debug::clear(*buffers);
    debug::shape(*buffers, capsule::create({.half_height_of_cylinder = 1.0f, .radius = 0.5f}), vec3f::load(center), quatf::load(IDENTITY), {});
    bool on_capsule = true;
    float top = -1e30f;
    for (uint32_t i = 0; i < buffers->line_count * 2; i++) {
        const Float3 p = vertex_position(*buffers, i);
        const Float3 closest = {center.x, scalar::clamp(p.y, center.y - 1.0f, center.y + 1.0f), center.z};
        on_capsule = on_capsule && scalar::abs(vec3f::distance(vec3f::load(p), vec3f::load(closest)) - 0.5f) < 1e-5f;
        top = scalar::max(top, p.y);
    }
    CHECK(ctx, on_capsule);
    CHECK_NEAR(ctx, top, center.y + 1.5f, 1e-5f);
    debug::release(buffers);
}

void test_debug_wrappers_carry_scale_and_offset(TestCtx &ctx) {
    dispatch::init();
    debug::DebugBuffers *buffers = debug::create();
    shapes::Shape *unit_box = box::create({.half_extents = {1.0f, 1.0f, 1.0f}});

    // scaled: the box's corners at the scale
    debug::shape(*buffers, scaled::create({.shape = unit_box, .scale = {2.0f, 3.0f, 4.0f}}), vec3f::make(0.0f, 0.0f, 0.0f), quatf::load(IDENTITY), {});
    CHECK(ctx, near3(max_abs_extent(*buffers), 2.0f, 3.0f, 4.0f, 1e-5f));

    // transformed: moved by its offset
    debug::clear(*buffers);
    debug::shape(*buffers, transformed::create({.shape = unit_box, .position = {0.0f, 5.0f, 0.0f}}), vec3f::make(0.0f, 0.0f, 0.0f), quatf::load(IDENTITY), {});
    CHECK(ctx, near3(vertex_position(*buffers, 0), -1.0f, 4.0f, -1.0f, 1e-5f));

    // a child turned a quarter about z under a compound scaled (2, 1, 1): the child sees the scale in its own space
    // (1, 2, 1), so its (1, 0.5, 0.5) half extents become (1, 1, 0.5) turned, a 2 x 2 x 1 box in the compound
    Float4 quarter_z = stored_axis_angle(vec3f::make(0.0f, 0.0f, 1.0f), scalar::PI * 0.5f);
    const compound::CompoundShapeChild child = {{0.0f, 0.0f, 0.0f}, quarter_z, box::create({.half_extents = {1.0f, 0.5f, 0.5f}})};
    shapes::Shape *compound_shape = compound::create({.children = &child, .num_children = 1});
    debug::clear(*buffers);
    debug::shape(*buffers, scaled::create({.shape = compound_shape, .scale = {2.0f, 1.0f, 1.0f}}), vec3f::make(0.0f, 0.0f, 0.0f), quatf::load(IDENTITY), {});
    CHECK(ctx, near3(max_abs_extent(*buffers), 1.0f, 1.0f, 0.5f, 1e-5f));

    // offset centre of mass: the geometry stays put
    debug::clear(*buffers);
    debug::shape(*buffers, offset_center_of_mass::create({.shape = unit_box, .offset = {0.0f, 3.0f, 0.0f}}), vec3f::make(0.0f, 0.0f, 0.0f), quatf::load(IDENTITY), {});
    CHECK(ctx, near3(max_abs_extent(*buffers), 1.0f, 1.0f, 1.0f, 1e-5f));
    debug::release(buffers);
}

void test_debug_bodies_one_of_each_shape(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    debug::DebugBuffers *buffers = debug::create();
    shapes::Shape *box_shape = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});
    shapes::Shape *sphere_shape = sphere::create({.radius = 0.5f});
    const compound::CompoundShapeChild children[2] = {
        {{1.0f, 0.0f, 0.0f}, IDENTITY, box_shape},
        {{-1.0f, 0.0f, 0.0f}, IDENTITY, sphere_shape},
    };

    add_body(scene, plane::create({.half_extent = 50.0f}), motion_type::MOTION_STATIC, {0.0f, 0.0f, 0.0f});
    add_body(scene, make_quad_mesh(), motion_type::MOTION_STATIC, {0.0f, 0.0f, 20.0f});
    add_body(scene, box_shape, motion_type::MOTION_DYNAMIC, {0.0f, 5.0f, 0.0f});
    add_body(scene, sphere_shape, motion_type::MOTION_DYNAMIC, {3.0f, 5.0f, 0.0f});
    add_body(scene, capsule::create({.half_height_of_cylinder = 0.5f, .radius = 0.25f}), motion_type::MOTION_DYNAMIC, {6.0f, 5.0f, 0.0f});
    add_body(scene, cylinder::create({.half_height = 0.5f, .radius = 0.5f}), motion_type::MOTION_DYNAMIC, {9.0f, 5.0f, 0.0f});
    add_body(scene, make_cube_hull(), motion_type::MOTION_DYNAMIC, {12.0f, 5.0f, 0.0f});
    add_body(scene, compound::create({.children = children, .num_children = 2}), motion_type::MOTION_DYNAMIC, {15.0f, 5.0f, 0.0f});
    RigidBody *kinematic =
        rigid_body::create(*scene.world, body_settings(box_shape, scene.moving, motion_type::MOTION_KINEMATIC, {18.0f, 5.0f, 0.0f}));
    RigidBody *removed = add_body(scene, sphere_shape, motion_type::MOTION_DYNAMIC, {21.0f, 5.0f, 0.0f});
    rigid_body::remove(*scene.world, removed);

    const uint32_t expected = PLANE_LINES + QUAD_MESH_LINES + BOX_LINES + SPHERE_LINES + CAPSULE_LINES + CYLINDER_LINES +
                              CUBE_HULL_LINES + (BOX_LINES + SPHERE_LINES) + BOX_LINES;
    debug::bodies(*buffers, *scene.world, {});
    CHECK(ctx, buffers->line_count == expected);
    CHECK(ctx, buffers->triangle_count == 0);
    // motion type colours: the plane drawn first is static blue, the kinematic box last is yellow
    CHECK(ctx, near3(vertex_color(*buffers, 0), 0.2f, 0.2f, 1.0f, 0.0f));
    CHECK(ctx, near3(vertex_color(*buffers, expected * 2 - 1), 1.0f, 1.0f, 0.0f, 0.0f));
    CHECK(ctx, near3(vertex_position(*buffers, (expected - BOX_LINES) * 2), 17.5f, 4.5f, -0.5f, 1e-5f));

    // the not-moving layer excluded: the plane and the mesh go, the moving bodies stay
    debug::clear(*buffers);
    debug::BodiesOptions moving_only;
    moving_only.excluded_object_layers = 1u << scene.not_moving;
    debug::bodies(*buffers, *scene.world, moving_only);
    CHECK(ctx, buffers->line_count == expected - PLANE_LINES - QUAD_MESH_LINES);
    // both layers excluded: nothing
    debug::clear(*buffers);
    moving_only.excluded_object_layers |= 1u << scene.moving;
    debug::bodies(*buffers, *scene.world, moving_only);
    CHECK(ctx, buffers->line_count == 0);

    // body() with an explicit colour
    debug::clear(*buffers);
    debug::body(*buffers, *kinematic, {.has_color = true, .color = {0.1f, 0.2f, 0.3f}});
    CHECK(ctx, buffers->line_count == BOX_LINES);
    CHECK(ctx, near3(vertex_color(*buffers, 0), 0.1f, 0.2f, 0.3f, 0.0f));

    debug::release(buffers);
    release_world_scene(scene);
}

void test_debug_body_colour_modes(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    debug::DebugBuffers *buffers = debug::create();
    shapes::Shape *box_shape = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});
    RigidBody *ground = add_body(scene, box_shape, motion_type::MOTION_STATIC, {0.0f, -5.0f, 0.0f});
    RigidBody *dynamic = add_body(scene, box_shape, motion_type::MOTION_DYNAMIC, {0.0f, 5.0f, 0.0f});

    // instance: a static body is a grey, a moving one saturated
    debug::body(*buffers, *ground, {.color_mode = debug::COLOR_INSTANCE});
    const Float3 grey = vertex_color(*buffers, 0);
    CHECK(ctx, grey.x == grey.y && grey.y == grey.z && grey.x >= 0.22f && grey.x <= 0.42f);
    debug::clear(*buffers);
    debug::body(*buffers, *dynamic, {.color_mode = debug::COLOR_INSTANCE});
    const Float3 hued = vertex_color(*buffers, 0);
    // s 0.7, l 0.6: the brightest channel 0.88, the dimmest 0.32
    CHECK_NEAR(ctx, scalar::max(hued.x, scalar::max(hued.y, hued.z)), 0.88f, 1e-5f);
    CHECK_NEAR(ctx, scalar::min(hued.x, scalar::min(hued.y, hued.z)), 0.32f, 1e-5f);

    // sleeping: awake red, asleep blue
    debug::clear(*buffers);
    debug::body(*buffers, *dynamic, {.color_mode = debug::COLOR_SLEEPING});
    CHECK(ctx, near3(vertex_color(*buffers, 0), 1.0f, 0.2f, 0.2f, 0.0f));
    rigid_body::sleep(*scene.world, dynamic);
    debug::clear(*buffers);
    debug::body(*buffers, *dynamic, {.color_mode = debug::COLOR_SLEEPING});
    CHECK(ctx, near3(vertex_color(*buffers, 0), 0.2f, 0.2f, 1.0f, 0.0f));

    // island: a body in no island is white
    debug::clear(*buffers);
    debug::body(*buffers, *ground, {.color_mode = debug::COLOR_ISLAND});
    CHECK(ctx, ground->island_index == -1);
    CHECK(ctx, near3(vertex_color(*buffers, 0), 1.0f, 1.0f, 1.0f, 0.0f));

    debug::release(buffers);
    release_world_scene(scene);
}

void test_debug_body_velocities(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    debug::DebugBuffers *buffers = debug::create();
    RigidBody *body = add_body(scene, box::create({.half_extents = {0.5f, 0.5f, 0.5f}}), motion_type::MOTION_DYNAMIC, {0.0f, 5.0f, 0.0f});
    rigid_body::set_linear_velocity(*scene.world, body, vec3f::make(3.0f, 0.0f, 0.0f));
    rigid_body::set_angular_velocity(*scene.world, body, vec3f::make(0.0f, 8.0f, 0.0f));

    debug::body(*buffers, *body, {.show_linear_velocity = true, .show_angular_velocity = true});
    CHECK(ctx, buffers->line_count == BOX_LINES + 2);
    // linear: from the centre of mass along the velocity, its magnitude long (under the 5 cap), yellow
    CHECK(ctx, near3(vertex_position(*buffers, BOX_LINES * 2), 0.0f, 5.0f, 0.0f, 1e-5f));
    CHECK(ctx, near3(vertex_position(*buffers, BOX_LINES * 2 + 1), 3.0f, 5.0f, 0.0f, 1e-5f));
    CHECK(ctx, near3(vertex_color(*buffers, BOX_LINES * 2), 1.0f, 1.0f, 0.0f, 0.0f));
    // angular: half its magnitude, capped at 3, orange
    CHECK(ctx, near3(vertex_position(*buffers, BOX_LINES * 2 + 3), 0.0f, 8.0f, 0.0f, 1e-5f));
    CHECK(ctx, near3(vertex_color(*buffers, BOX_LINES * 2 + 2), 1.0f, 0.5f, 0.0f, 0.0f));

    // a still body draws no arrows
    rigid_body::set_linear_velocity(*scene.world, body, vec3f::make(0.0f, 0.0f, 0.0f));
    rigid_body::set_angular_velocity(*scene.world, body, vec3f::make(0.0f, 0.0f, 0.0f));
    debug::clear(*buffers);
    debug::body(*buffers, *body, {.show_linear_velocity = true, .show_angular_velocity = true});
    CHECK(ctx, buffers->line_count == BOX_LINES);

    debug::release(buffers);
    release_world_scene(scene);
}

void test_debug_contacts_of_a_resting_box(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    debug::DebugBuffers *buffers = debug::create();
    add_body(scene, box::create({.half_extents = {5.0f, 0.5f, 5.0f}}), motion_type::MOTION_STATIC, {0.0f, -0.5f, 0.0f});
    add_body(scene, box::create({.half_extents = {0.5f, 0.5f, 0.5f}}), motion_type::MOTION_DYNAMIC, {0.0f, 0.5f, 0.0f});
    for (uint32_t i = 0; i < 10; i++) step_world(*scene.world);

    CHECK(ctx, scene.world->contact_constraints.count == 1);
    if (scene.world->contact_constraints.count != 1) {
        debug::release(buffers);
        release_world_scene(scene);
        return;
    }
    const contact_constraints::ContactConstraint &constraint = scene.world->contact_constraints.pool[0];
    CHECK(ctx, constraint.num_contact_points == 4);
    CHECK_NEAR(ctx, scalar::abs(constraint.normal.y), 1.0f, 1e-4f);

    // a cross and a normal per point
    debug::contacts(*buffers, *scene.world);
    CHECK(ctx, buffers->line_count == 4 * (CROSS_LINES + 1));
    for (uint32_t j = 0; j < 4; j++) {
        const uint32_t normal_line = j * (CROSS_LINES + 1) + CROSS_LINES;
        const Float3 from = vertex_position(*buffers, normal_line * 2);
        const Float3 to = vertex_position(*buffers, normal_line * 2 + 1);
        // from point a on the box's bottom face corner, 0.15 along the normal, magenta
        CHECK(ctx, near3(from, constraint.contact_points[j].position_a, 0.0f));
        CHECK_NEAR(ctx, scalar::abs(from.x), 0.5f, 1e-2f);
        CHECK_NEAR(ctx, scalar::abs(from.z), 0.5f, 1e-2f);
        CHECK_NEAR(ctx, to.y - from.y, 0.15f * constraint.normal.y, 1e-5f);
        CHECK(ctx, near3(vertex_color(*buffers, normal_line * 2), 1.0f, 0.0f, 1.0f, 0.0f));
    }

    // the manifold: two crosses per point, the closed outline of 4 edges, the normal and both tangents
    debug::clear(*buffers);
    debug::contact_constraints(*buffers, *scene.world);
    CHECK(ctx, buffers->line_count == 4 * 2 * CROSS_LINES + 4 + 3);
    // a resting box carries impulse, so its points draw bright
    CHECK(ctx, near3(vertex_color(*buffers, 0), 0.0f, 1.0f, 1.0f, 0.0f));

    debug::release(buffers);
    release_world_scene(scene);
}

void test_debug_joints_hinge(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    debug::DebugBuffers *buffers = debug::create();
    shapes::Shape *box_shape = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});
    RigidBody *a = add_body(scene, box_shape, motion_type::MOTION_STATIC, {0.0f, 0.0f, 0.0f});
    RigidBody *b = add_body(scene, box_shape, motion_type::MOTION_DYNAMIC, {1.0f, 0.0f, 0.0f});
    hinge_constraint::HingeConstraintSettings settings;
    settings.point_a = settings.point_b = {0.5f, 0.0f, 0.0f};
    hinge_constraint::create(*scene.world, a, b, settings);

    // a's cross and hinge axis, b's cross and hinge axis, b's normal axis
    const uint32_t hinge_lines = 2 * (CROSS_LINES + 1) + 1;
    debug::joints(*buffers, *scene.world, {});
    CHECK(ctx, buffers->line_count == hinge_lines);
    // a's cross centred on the pivot, its arms size * 0.1 long; the hinge axis size long along +y, red
    CHECK(ctx, near3(vertex_position(*buffers, 0), 0.45f, 0.0f, 0.0f, 1e-5f));
    CHECK(ctx, near3(vertex_position(*buffers, 1), 0.55f, 0.0f, 0.0f, 1e-5f));
    CHECK(ctx, near3(vertex_position(*buffers, CROSS_LINES * 2), 0.5f, 0.0f, 0.0f, 1e-5f));
    CHECK(ctx, near3(vertex_position(*buffers, CROSS_LINES * 2 + 1), 0.5f, 0.5f, 0.0f, 1e-5f));
    CHECK(ctx, near3(vertex_color(*buffers, CROSS_LINES * 2), 1.0f, 0.0f, 0.0f, 0.0f));
    // b's normal axis along +x, white
    CHECK(ctx, near3(vertex_position(*buffers, (hinge_lines - 1) * 2 + 1), 1.0f, 0.0f, 0.0f, 1e-5f));
    CHECK(ctx, near3(vertex_color(*buffers, (hinge_lines - 1) * 2), 1.0f, 1.0f, 1.0f, 0.0f));

    // b turned a quarter about the hinge: b's normal axis turns with it
    Float4 quarter_y = stored_axis_angle(vec3f::make(0.0f, 1.0f, 0.0f), scalar::PI * 0.5f);
    rigid_body::set_transform(*scene.world, b, vec3f::make(1.0f, 0.0f, 0.0f), quatf::load(quarter_y), false);
    debug::clear(*buffers);
    debug::joints(*buffers, *scene.world, {});
    // b's pivot (-0.5, 0, 0) from its centre turns to (0, 0, 0.5), its normal axis to -z
    CHECK(ctx, near3(vertex_position(*buffers, (hinge_lines - 1) * 2), 1.0f, 0.0f, 0.5f, 1e-5f));
    CHECK(ctx, near3(vertex_position(*buffers, (hinge_lines - 1) * 2 + 1), 1.0f, 0.0f, 0.0f, 1e-5f));

    // limits add a 32 segment pie: 33 spokes and 32 rim lines
    settings.limits_min = -1.0f;
    settings.limits_max = 1.0f;
    hinge_constraint::create(*scene.world, a, b, settings);
    debug::clear(*buffers);
    debug::joints(*buffers, *scene.world, {});
    CHECK(ctx, buffers->line_count == 2 * hinge_lines + 33 + 32);
    debug::clear(*buffers);
    debug::joints(*buffers, *scene.world, {.draw_limits = false});
    CHECK(ctx, buffers->line_count == 2 * hinge_lines);

    debug::release(buffers);
    release_world_scene(scene);
}

void test_debug_joints_point_and_distance(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    debug::DebugBuffers *buffers = debug::create();
    shapes::Shape *box_shape = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});
    RigidBody *a = add_body(scene, box_shape, motion_type::MOTION_STATIC, {0.0f, 0.0f, 0.0f});
    RigidBody *b = add_body(scene, box_shape, motion_type::MOTION_DYNAMIC, {2.0f, 0.0f, 0.0f});

    // point: two crosses and the line between
    point_constraint::create(*scene.world, a, b, {.point_a = {0.5f, 0.0f, 0.0f}, .point_b = {1.5f, 0.0f, 0.0f}});
    debug::joints(*buffers, *scene.world, {});
    CHECK(ctx, buffers->line_count == 2 * CROSS_LINES + 1);

    // distance with both limits: the line and two 16 segment circles
    distance_constraint::create(*scene.world, a, b,
                                {.point_a = {0.5f, 0.0f, 0.0f}, .point_b = {1.5f, 0.0f, 0.0f}, .min_distance = 0.5f, .max_distance = 2.0f});
    debug::clear(*buffers);
    debug::joints(*buffers, *scene.world, {});
    // the joints draw distance before point
    CHECK(ctx, buffers->line_count == 1 + 2 * 16 + 2 * CROSS_LINES + 1);
    // the max circle's first vertex 2 from point a, in the plane across the line
    const Float3 rim = vertex_position(*buffers, (1 + 16) * 2);
    CHECK_NEAR(ctx, vec3f::distance(vec3f::load(rim), vec3f::make(0.5f, 0.0f, 0.0f)), 2.0f, 1e-5f);
    CHECK_NEAR(ctx, rim.x, 0.5f, 1e-5f);

    // a removed body takes its constraints with it
    rigid_body::remove(*scene.world, b);
    debug::clear(*buffers);
    debug::joints(*buffers, *scene.world, {});
    CHECK(ctx, buffers->line_count == 0);

    debug::release(buffers);
    release_world_scene(scene);
}

void test_debug_buffers_reused_without_growing(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    debug::DebugBuffers *buffers = debug::create();
    add_body(scene, box::create({.half_extents = {5.0f, 0.5f, 5.0f}}), motion_type::MOTION_STATIC, {0.0f, -0.5f, 0.0f});
    for (uint32_t i = 0; i < 8; i++) {
        add_body(scene, sphere::create({.radius = 0.5f}), motion_type::MOTION_DYNAMIC, {(float)i * 1.5f - 6.0f, 0.5f, 0.0f});
    }
    step_world(*scene.world);

    debug::bodies(*buffers, *scene.world, {.show_linear_velocity = true});
    debug::contacts(*buffers, *scene.world);
    const uint32_t first_count = buffers->line_count;
    const float *first_data = buffers->lines.data;
    const uint32_t first_capacity = buffers->lines.capacity;
    CHECK(ctx, first_count >= BOX_LINES + 8 * SPHERE_LINES);

    debug::clear(*buffers);
    debug::bodies(*buffers, *scene.world, {.show_linear_velocity = true});
    debug::contacts(*buffers, *scene.world);
    CHECK(ctx, buffers->line_count == first_count);
    CHECK(ctx, buffers->lines.data == first_data);
    CHECK(ctx, buffers->lines.capacity == first_capacity);

    debug::release(buffers);
    release_world_scene(scene);
}

void test_debug_contacts_draw_each_manifold_point_with_its_normal(TestCtx &ctx) {
    // a box resting on a static box: a cross and a normal per manifold point, every point on the top face
    WorldScene scene = make_world_scene(ctx);
    add_body(scene, box::create({.half_extents = {4.0f, 0.5f, 4.0f}}), motion_type::MOTION_STATIC, {0.0f, 0.0f, 0.0f});
    add_body(scene, box::create({.half_extents = {0.5f, 0.5f, 0.5f}}), motion_type::MOTION_DYNAMIC, {0.0f, 0.99f, 0.0f});
    step_world(*scene.world);
    uint32_t points = 0;
    for (uint32_t i = 0; i < scene.world->contact_constraints.count; i++) points += scene.world->contact_constraints.pool[i].num_contact_points;
    CHECK(ctx, points == 4);
    debug::DebugBuffers *buffers = debug::create();
    debug::contacts(*buffers, *scene.world);
    CHECK(ctx, buffers->line_count == points * 4);
    for (uint32_t line = 0; line < buffers->line_count; line++) {
        const float *v = &buffers->lines.data[line * debug::LINE_STRIDE];
        CHECK(ctx, scalar::abs(v[1] - 0.5f) < 0.2f);
    }
    debug::release(buffers);
    release_world_scene(scene);
}


void test_debug_character_contacts_cross_and_normal_per_contact(TestCtx &ctx) {
    WorldScene scene = make_world_scene(ctx);
    add_body(scene, box::create({.half_extents = {10.0f, 0.5f, 10.0f}}), motion_type::MOTION_STATIC, {0.0f, -0.5f, 0.0f});
    kcc::KCCSettings settings;
    settings.shape = capsule::create({.half_height_of_cylinder = 0.5f, .radius = 0.3f});
    kcc::KCC *character = kcc::create(settings, vec3f::make(0.0f, 0.8f, 0.0f), quatf::identity());
    filter::Filter filter = filter::create(scene.world->settings.layers);
    step_world(*scene.world);
    kcc::refresh_contacts(*scene.world, *character, filter, nullptr);
    CHECK(ctx, character->contacts.length > 0);

    debug::DebugBuffers *buffers = debug::create();
    debug::character_contacts(*buffers, *character);
    CHECK(ctx, buffers->line_count == 4 * character->contacts.length);
    // the cross is at the first contact, and its normal line leaves it toward the character, up off the floor
    const Vec3f at = character->contacts[0].position;
    CHECK_NEAR(ctx, (buffers->lines[0] + buffers->lines[6]) * 0.5f, vec3f::x(at), 1e-5f);
    const float *normal_line = buffers->lines.data + 3 * debug::LINE_STRIDE;
    CHECK(ctx, normal_line[7] > normal_line[1]);

    debug::release(buffers);
    filter::release(filter);
    kcc::destroy(character);
    release_world_scene(scene);
}
