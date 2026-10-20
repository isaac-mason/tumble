#include "instance.h"
#include "math/mat4f.h"
#include "math/vec3f.h"
#include "physics/shapes/box.h"
#include "physics/shapes/convex.h"
#include "physics/shapes/dispatch.h"
#include "test.h"
#include "test_math.h"

static void box_face(shapes::SupportingFaceResult &io, const shapes::Shape *shape, const Float3 &direction, const Float3 &scale) {
    io.transform = mat4f::identity();
    io.scale = vec3f::load(scale);
    box::get_supporting_face(io, vec3f::load(direction), shape, sub_shape::EMPTY_SUB_SHAPE_ID);
}

/** the normal of the face's winding: (v1 - v0) x (v2 - v1), unnormalised */
static Vec3f face_winding(const face::Face &face) {
    const Vec3f *v = face.vertices;
    return vec3f::cross(vec3f::subtract(v[1], v[0]), vec3f::subtract(v[2], v[1]));
}

static bool vertex_is(const face::Face &face, uint32_t i, float x, float y, float z) {
    return vec3f::exact_equals(face.vertices[i], vec3f::make(x, y, z));
}

void test_box_create(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));

    shapes::Shape *b = box::create({.half_extents = {1.0f, 2.0f, 3.0f}});
    CHECK(ctx, b != nullptr);
    CHECK(ctx, b->type == shapes::SHAPE_BOX);
    CHECK(ctx, b->box.half_extents.x == 1.0f && b->box.half_extents.y == 2.0f && b->box.half_extents.z == 3.0f);
    CHECK(ctx, b->box.density == 1000.0f && b->material_id == -1);
    CHECK(ctx, b->box.convex_radius == support::DEFAULT_CONVEX_RADIUS);

    // bounds are +- the half extents, the centre of mass the origin
    CHECK(ctx, b->aabb.min.x == -1.0f && b->aabb.min.y == -2.0f && b->aabb.min.z == -3.0f);
    CHECK(ctx, b->aabb.max.x == 1.0f && b->aabb.max.y == 2.0f && b->aabb.max.z == 3.0f);
    CHECK(ctx, vec3f::exact_equals(vec3f::load(b->center_of_mass), vec3f::zero()));

    // volume 8abc
    CHECK(ctx, b->volume == 48.0f);

    // update after a change recomputes bounds and volume
    b->box.half_extents = {0.5f, 0.5f, 2.0f};
    box::update(b);
    CHECK(ctx, b->volume == 4.0f && b->aabb.max.z == 2.0f && b->aabb.min.x == -0.5f);
    box::destroy(b);

    shapes::Shape *dense = box::create({.half_extents = {1.0f, 1.0f, 1.0f}, .density = 250.0f, .material_id = 7});
    CHECK(ctx, dense->box.density == 250.0f && dense->material_id == 7);
    box::destroy(dense);
}

void test_box_convex_radius_clamp(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));

    // fits: kept
    shapes::Shape *b = box::create({.half_extents = {0.5f, 1.0f, 2.0f}, .convex_radius = 0.1f});
    CHECK(ctx, b->box.convex_radius == 0.1f);
    box::destroy(b);
    // oversized: clamped to the smallest half extent
    b = box::create({.half_extents = {0.02f, 1.0f, 2.0f}, .convex_radius = 1.0f});
    CHECK(ctx, b->box.convex_radius == 0.02f);
    box::destroy(b);
    // the default clamps too
    b = box::create({.half_extents = {0.01f, 1.0f, 2.0f}});
    CHECK(ctx, b->box.convex_radius == 0.01f);
    box::destroy(b);
    // zero sized
    b = box::create({.half_extents = {0.0f, 0.0f, 0.0f}, .convex_radius = 1.0f});
    CHECK(ctx, b->box.convex_radius == 0.0f && b->volume == 0.0f);
    box::destroy(b);
    // exactly the smallest half extent
    b = box::create({.half_extents = {0.3f, 1.0f, 2.0f}, .convex_radius = 0.3f});
    CHECK(ctx, b->box.convex_radius == 0.3f);
    box::destroy(b);
}

void test_box_mass_properties(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));

    // unit cube of half extent 1: mass 8 * density
    shapes::Shape *cube = box::create({.half_extents = {1.0f, 1.0f, 1.0f}});
    mass_properties::MassProperties props = mass_properties::create();
    box::compute_mass_properties(props, cube);
    CHECK_NEAR(ctx, props.mass, 8000.0f, 1e-2f);
    box::destroy(cube);

    // full extents (2, 4, 6) at density 10: m = 480, I = m/12 (b^2 + c^2) and so on
    shapes::Shape *b = box::create({.half_extents = {1.0f, 2.0f, 3.0f}, .density = 10.0f});
    box::compute_mass_properties(props, b);
    const float m = 480.0f;
    CHECK_NEAR(ctx, props.mass, m, 1e-3f);
    const float *inertia = props.inertia.m;
    CHECK_NEAR(ctx, inertia[0], m / 12.0f * (16.0f + 36.0f), 1e-2f);
    CHECK_NEAR(ctx, inertia[5], m / 12.0f * (4.0f + 36.0f), 1e-2f);
    CHECK_NEAR(ctx, inertia[10], m / 12.0f * (4.0f + 16.0f), 1e-2f);
    CHECK(ctx, inertia[1] == 0.0f && inertia[2] == 0.0f && inertia[4] == 0.0f && inertia[6] == 0.0f && inertia[8] == 0.0f &&
                   inertia[9] == 0.0f);
    box::destroy(b);
}

void test_box_inner_radius(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    shapes::Shape *b = box::create({.half_extents = {2.0f, 0.75f, 3.0f}});
    CHECK(ctx, box::get_inner_radius(b) == 0.75f);
    box::destroy(b);
}

void test_box_surface_normal(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));

    shapes::SurfaceNormalResult io;
    shapes::Shape *wide = box::create({.half_extents = {2.0f, 1.0f, 1.0f}});
    // just outside +x
    io.position = vec3f::make(2.1f, 0.0f, 0.0f);
    box::get_surface_normal(io, wide, sub_shape::EMPTY_SUB_SHAPE_ID);
    CHECK(ctx, vec3f::exact_equals(io.normal, vec3f::make(1.0f, 0.0f, 0.0f)));
    // on the top face
    io.position = vec3f::make(0.3f, 1.0f, -0.2f);
    box::get_surface_normal(io, wide, sub_shape::EMPTY_SUB_SHAPE_ID);
    CHECK(ctx, vec3f::exact_equals(io.normal, vec3f::make(0.0f, 1.0f, 0.0f)));
    // on the -z face
    io.position = vec3f::make(-1.0f, 0.5f, -1.0f + 1e-3f);
    box::get_surface_normal(io, wide, sub_shape::EMPTY_SUB_SHAPE_ID);
    CHECK(ctx, vec3f::exact_equals(io.normal, vec3f::make(0.0f, 0.0f, -1.0f)));
    box::destroy(wide);

    shapes::Shape *tall = box::create({.half_extents = {1.0f, 2.0f, 1.0f}});
    io.position = vec3f::make(0.0f, -2.1f, 0.0f);
    box::get_surface_normal(io, tall, sub_shape::EMPTY_SUB_SHAPE_ID);
    CHECK(ctx, vec3f::exact_equals(io.normal, vec3f::make(0.0f, -1.0f, 0.0f)));
    box::destroy(tall);

    // near a corner, the axis with the smallest distance to its face wins
    shapes::Shape *cube = box::create({.half_extents = {1.0f, 1.0f, 1.0f}});
    io.position = vec3f::make(1.1f, 0.5f, 0.5f);
    box::get_surface_normal(io, cube, sub_shape::EMPTY_SUB_SHAPE_ID);
    CHECK(ctx, vec3f::exact_equals(io.normal, vec3f::make(1.0f, 0.0f, 0.0f)));
    box::destroy(cube);
}

void test_box_supporting_face(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));

    shapes::SupportingFaceResult io;
    const Float3 unit = {1.0f, 1.0f, 1.0f};
    shapes::Shape *b = box::create({.half_extents = {1.0f, 2.0f, 3.0f}});

    // the face opposing -y is the top: four vertices at y = 2, ccw seen from outside
    box_face(io, b, {0.0f, -1.0f, 0.0f}, unit);
    CHECK(ctx, io.face.num_vertices == 4);
    CHECK(ctx, vertex_is(io.face, 0, -1.0f, 2.0f, -3.0f));
    CHECK(ctx, vertex_is(io.face, 1, -1.0f, 2.0f, 3.0f));
    CHECK(ctx, vertex_is(io.face, 2, 1.0f, 2.0f, 3.0f));
    CHECK(ctx, vertex_is(io.face, 3, 1.0f, 2.0f, -3.0f));
    Vec3f winding = face_winding(io.face);
    CHECK(ctx, vec3f::x(winding) == 0.0f && vec3f::y(winding) > 0.0f && vec3f::z(winding) == 0.0f);

    // +y gives the bottom, wound outward (-y)
    box_face(io, b, {0.0f, 1.0f, 0.0f}, unit);
    for (uint32_t i = 0; i < 4; i++) CHECK(ctx, vec3f::y(io.face.vertices[i]) == -2.0f);
    winding = face_winding(io.face);
    CHECK(ctx, vec3f::y(winding) < 0.0f);

    // each axis, opposite the direction, wound outward
    box_face(io, b, {1.0f, 0.0f, 0.0f}, unit);
    for (uint32_t i = 0; i < 4; i++) CHECK(ctx, vec3f::x(io.face.vertices[i]) == -1.0f);
    CHECK(ctx, vec3f::x(face_winding(io.face)) < 0.0f);
    box_face(io, b, {-1.0f, 0.0f, 0.0f}, unit);
    for (uint32_t i = 0; i < 4; i++) CHECK(ctx, vec3f::x(io.face.vertices[i]) == 1.0f);
    CHECK(ctx, vec3f::x(face_winding(io.face)) > 0.0f);
    box_face(io, b, {0.0f, 0.0f, 1.0f}, unit);
    for (uint32_t i = 0; i < 4; i++) CHECK(ctx, vec3f::z(io.face.vertices[i]) == -3.0f);
    CHECK(ctx, vec3f::z(face_winding(io.face)) < 0.0f);
    box_face(io, b, {0.0f, 0.0f, -1.0f}, unit);
    for (uint32_t i = 0; i < 4; i++) CHECK(ctx, vec3f::z(io.face.vertices[i]) == 3.0f);
    CHECK(ctx, vec3f::z(face_winding(io.face)) > 0.0f);

    // the dominant axis picks the face
    box_face(io, b, {10.0f, 1.0f, 1.0f}, unit);
    for (uint32_t i = 0; i < 4; i++) CHECK(ctx, vec3f::x(io.face.vertices[i]) == -1.0f);

    // a mirrored scale changes nothing: the box is symmetric, so the face is the same one, still wound outward
    box_face(io, b, {0.0f, -1.0f, 0.0f}, {-1.0f, 1.0f, 1.0f});
    CHECK(ctx, vertex_is(io.face, 0, -1.0f, 2.0f, -3.0f));
    CHECK(ctx, vertex_is(io.face, 2, 1.0f, 2.0f, 3.0f));
    CHECK(ctx, vec3f::y(face_winding(io.face)) > 0.0f);

    // the transform places the face, scale applied first
    io.transform = mat4f::from_translation(vec3f::make(10.0f, 0.0f, -5.0f));
    io.scale = vec3f::make(2.0f, 0.5f, 1.0f);
    box::get_supporting_face(io, vec3f::make(0.0f, -1.0f, 0.0f), b, sub_shape::EMPTY_SUB_SHAPE_ID);
    CHECK(ctx, vertex_is(io.face, 0, 8.0f, 1.0f, -8.0f));
    CHECK(ctx, vertex_is(io.face, 2, 12.0f, 1.0f, -2.0f));
    box::destroy(b);
}

void test_box_set_support(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));

    shapes::Shape *b = box::create({.half_extents = {1.0f, 2.0f, 3.0f}, .convex_radius = 0.04f});
    support::Support s = support::create();

    // include: the full box, nothing reported
    box::set_support(s, b, support::INCLUDE_CONVEX_RADIUS, vec3f::make(1.0f, 1.0f, 1.0f));
    CHECK(ctx, s.get_support == support::box_support && !s.has_transform && s.add_radius == 0.0f);
    CHECK(ctx, vec3f::exact_equals(s.box.half_extents, vec3f::make(1.0f, 2.0f, 3.0f)) && s.convex_radius == 0.0f);

    // default is include for a box
    box::set_support(s, b, support::DEFAULT, vec3f::make(1.0f, 1.0f, 1.0f));
    CHECK(ctx, vec3f::exact_equals(s.box.half_extents, vec3f::make(1.0f, 2.0f, 3.0f)) && s.convex_radius == 0.0f);

    // exclude: the core shrunk by the convex radius, which is reported
    box::set_support(s, b, support::EXCLUDE_CONVEX_RADIUS, vec3f::make(1.0f, 1.0f, 1.0f));
    CHECK(ctx, s.convex_radius == 0.04f);
    CHECK(ctx, vec3f::x(s.box.half_extents) == 1.0f - 0.04f && vec3f::y(s.box.half_extents) == 2.0f - 0.04f && vec3f::z(s.box.half_extents) == 3.0f - 0.04f);

    // scale: per-axis absolute, the convex radius by the smallest scale
    box::set_support(s, b, support::INCLUDE_CONVEX_RADIUS, vec3f::make(-2.0f, 0.5f, 1.0f));
    CHECK(ctx, vec3f::exact_equals(s.box.half_extents, vec3f::make(2.0f, 1.0f, 3.0f)));
    box::set_support(s, b, support::EXCLUDE_CONVEX_RADIUS, vec3f::make(-2.0f, 0.5f, 1.0f));
    CHECK(ctx, s.convex_radius == 0.02f);
    CHECK(ctx, vec3f::x(s.box.half_extents) == 2.0f - 0.02f && vec3f::y(s.box.half_extents) == 1.0f - 0.02f && vec3f::z(s.box.half_extents) == 3.0f - 0.02f);

    // a scaled-up radius is capped at the default convex radius
    box::set_support(s, b, support::EXCLUDE_CONVEX_RADIUS, vec3f::make(4.0f, 4.0f, 4.0f));
    CHECK(ctx, s.convex_radius == support::DEFAULT_CONVEX_RADIUS);
    CHECK(ctx, vec3f::x(s.box.half_extents) == 4.0f - support::DEFAULT_CONVEX_RADIUS);
    box::destroy(b);

    // the core never goes negative
    shapes::Shape *flat = box::create({.half_extents = {0.0f, 1.0f, 1.0f}});
    box::set_support(s, flat, support::EXCLUDE_CONVEX_RADIUS, vec3f::make(1.0f, 1.0f, 1.0f));
    CHECK(ctx, s.convex_radius == 0.0f && vec3f::x(s.box.half_extents) == 0.0f && vec3f::y(s.box.half_extents) == 1.0f);
    box::destroy(flat);
}

/* material by sub shape id */

void test_box_material(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    // a convex shape is one leaf: the empty sub shape id names its own material
    shapes::Shape *painted = box::create({.half_extents = {1, 2, 3}, .material_id = 6});
    shapes::Shape *plain = box::create({.half_extents = {1, 2, 3}});
    CHECK(ctx, shapes::get_shape_material(painted, sub_shape::EMPTY_SUB_SHAPE_ID) == 6);
    CHECK(ctx, shapes::get_shape_material(plain, sub_shape::EMPTY_SUB_SHAPE_ID) == -1);
    CHECK(ctx, shapes::def_of(painted).get_material == convex::get_convex_material);
    box::destroy(painted);
    box::destroy(plain);
}
