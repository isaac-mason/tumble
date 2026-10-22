#include "tumble/math/mat4f.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/shapes/convex.h"
#include "tumble/physics/shapes/cylinder.h"
#include "tumble/physics/shapes/dispatch.h"
#include "test.h"
#include "test_math.h"

static void cylinder_face(shapes::SupportingFaceResult &io, const shapes::Shape *shape, const Float3 &direction, const Float3 &scale) {
    io.transform = mat4f::identity();
    io.scale = vec3f::load(scale);
    cylinder::get_supporting_face(io, vec3f::load(direction), shape, sub_shape::EMPTY_SUB_SHAPE_ID);
}

void test_cylinder_create(TestCtx &ctx) {

    shapes::Shape *c = cylinder::create({.half_height = 1.0f, .radius = 0.5f});
    CHECK(ctx, c != nullptr);
    CHECK(ctx, c->type == shapes::SHAPE_CYLINDER);
    CHECK(ctx, shapes::as<cylinder::CylinderShape>(c).half_height == 1.0f && shapes::as<cylinder::CylinderShape>(c).radius == 0.5f);
    CHECK(ctx, shapes::as<cylinder::CylinderShape>(c).convex_radius == support::DEFAULT_CONVEX_RADIUS);
    CHECK(ctx, shapes::as<cylinder::CylinderShape>(c).density == 1000.0f && c->material_id == -1);
    cylinder::destroy(c);

    // V = pi r^2 2h
    c = cylinder::create({.half_height = 2.0f, .radius = 1.0f});
    CHECK_NEAR(ctx, c->volume, scalar::PI * 4.0f, 1e-5f);
    cylinder::destroy(c);

    // bounds: radius in x and z, half height in y; centre of mass at the origin
    c = cylinder::create({.half_height = 2.0f, .radius = 1.5f});
    CHECK(ctx, c->aabb.min.x == -1.5f && c->aabb.min.y == -2.0f && c->aabb.min.z == -1.5f);
    CHECK(ctx, c->aabb.max.x == 1.5f && c->aabb.max.y == 2.0f && c->aabb.max.z == 1.5f);
    CHECK(ctx, vec3f::exact_equals(vec3f::load(c->center_of_mass), vec3f::make(0.0f, 0.0f, 0.0f)));

    // update after a change
    shapes::as<cylinder::CylinderShape>(c).radius = 0.5f;
    cylinder::update(c);
    CHECK(ctx, c->aabb.max.x == 0.5f);
    CHECK_NEAR(ctx, c->volume, scalar::PI * 0.25f * 4.0f, 1e-5f);
    cylinder::destroy(c);
}

void test_cylinder_convex_radius_clamp(TestCtx &ctx) {

    shapes::Shape *c = cylinder::create({.half_height = 1.0f, .radius = 0.5f, .convex_radius = 0.1f});
    CHECK(ctx, shapes::as<cylinder::CylinderShape>(c).convex_radius == 0.1f);
    cylinder::destroy(c);
    // clamped to the half height when it is smallest
    c = cylinder::create({.half_height = 0.03f, .radius = 0.5f, .convex_radius = 1.0f});
    CHECK(ctx, shapes::as<cylinder::CylinderShape>(c).convex_radius == 0.03f);
    cylinder::destroy(c);
    // to the radius when it is smallest
    c = cylinder::create({.half_height = 2.0f, .radius = 0.02f, .convex_radius = 1.0f});
    CHECK(ctx, shapes::as<cylinder::CylinderShape>(c).convex_radius == 0.02f);
    cylinder::destroy(c);
    // the default clamps too
    c = cylinder::create({.half_height = 0.01f, .radius = 0.01f});
    CHECK(ctx, shapes::as<cylinder::CylinderShape>(c).convex_radius == 0.01f);
    cylinder::destroy(c);
    c = cylinder::create({.half_height = 0.0f, .radius = 0.0f, .convex_radius = 1.0f});
    CHECK(ctx, shapes::as<cylinder::CylinderShape>(c).convex_radius == 0.0f);
    cylinder::destroy(c);
    c = cylinder::create({.half_height = 0.3f, .radius = 0.5f, .convex_radius = 0.3f});
    CHECK(ctx, shapes::as<cylinder::CylinderShape>(c).convex_radius == 0.3f);
    cylinder::destroy(c);
}

void test_cylinder_mass_properties(TestCtx &ctx) {

    shapes::Shape *c = cylinder::create({.half_height = 1.0f, .radius = 0.5f, .density = 100.0f});
    mass_properties::MassProperties props = mass_properties::create();
    cylinder::compute_mass_properties(props, c);

    // m = rho pi r^2 2h; I_yy = m r^2 / 2; I_xx = I_zz = m r^2 / 4 + m (2h)^2 / 12
    const float m = 100.0f * scalar::PI * 0.25f * 2.0f;
    CHECK_NEAR(ctx, props.mass, m, 1e-3f);
    const float *inertia = props.inertia.m;
    CHECK_NEAR(ctx, inertia[5], m * 0.25f / 2.0f, 1e-3f);
    CHECK_NEAR(ctx, inertia[0], m * 0.25f / 4.0f + m * 4.0f / 12.0f, 1e-3f);
    CHECK(ctx, inertia[10] == inertia[0]);
    CHECK(ctx, inertia[1] == 0.0f && inertia[2] == 0.0f && inertia[4] == 0.0f && inertia[6] == 0.0f && inertia[8] == 0.0f &&
                   inertia[9] == 0.0f && inertia[15] == 1.0f);
    cylinder::destroy(c);
}

void test_cylinder_inner_radius(TestCtx &ctx) {
    shapes::Shape *c = cylinder::create({.half_height = 1.0f, .radius = 0.5f});
    CHECK(ctx, cylinder::get_inner_radius(c) == 0.5f);
    cylinder::destroy(c);
    c = cylinder::create({.half_height = 0.25f, .radius = 3.0f});
    CHECK(ctx, cylinder::get_inner_radius(c) == 0.25f);
    cylinder::destroy(c);
}

void test_cylinder_surface_normal(TestCtx &ctx) {

    shapes::Shape *c = cylinder::create({.half_height = 2.0f, .radius = 1.0f});
    shapes::SurfaceNormalResult io;

    // on the side: radial
    io.position = vec3f::make(0.6f, 0.5f, 0.8f);
    cylinder::get_surface_normal(io, c, sub_shape::EMPTY_SUB_SHAPE_ID);
    CHECK_NEAR(ctx, vec3f::x(io.normal), 0.6f, 1e-6f);
    CHECK(ctx, vec3f::y(io.normal) == 0.0f);
    CHECK_NEAR(ctx, vec3f::z(io.normal), 0.8f, 1e-6f);

    // on a cap: +-y
    io.position = vec3f::make(0.2f, 2.0f, 0.1f);
    cylinder::get_surface_normal(io, c, sub_shape::EMPTY_SUB_SHAPE_ID);
    CHECK(ctx, vec3f::exact_equals(io.normal, vec3f::make(0.0f, 1.0f, 0.0f)));
    io.position = vec3f::make(0.2f, -1.99f, 0.1f);
    cylinder::get_surface_normal(io, c, sub_shape::EMPTY_SUB_SHAPE_ID);
    CHECK(ctx, vec3f::exact_equals(io.normal, vec3f::make(0.0f, -1.0f, 0.0f)));

    // on the axis, nearer the side than a cap: x
    shapes::Shape *thin = cylinder::create({.half_height = 5.0f, .radius = 0.1f});
    io.position = vec3f::make(0.0f, 0.0f, 0.0f);
    cylinder::get_surface_normal(io, thin, sub_shape::EMPTY_SUB_SHAPE_ID);
    CHECK(ctx, vec3f::exact_equals(io.normal, vec3f::make(1.0f, 0.0f, 0.0f)));
    cylinder::destroy(thin);
    cylinder::destroy(c);
}

void test_cylinder_supporting_face(TestCtx &ctx) {

    shapes::SupportingFaceResult io;
    const Float3 unit = {1.0f, 1.0f, 1.0f};
    shapes::Shape *c = cylinder::create({.half_height = 1.0f, .radius = 0.5f});

    // the face opposing -y is the top cap: an octagon on the rim at y = h, ccw seen from above
    float v[3 * face::FACE_MAX_VERTICES];
    cylinder_face(io, c, {0.0f, -1.0f, 0.0f}, unit);
    flatten3(v, io.face.vertices, io.face.num_vertices);
    CHECK(ctx, io.face.num_vertices == 8);
    CHECK(ctx, v[0] == 0.0f && v[1] == 1.0f && v[2] == 0.5f);
    CHECK(ctx, v[6] == 0.5f && v[7] == 1.0f && v[8] == 0.0f);
    CHECK(ctx, v[12] == 0.0f && v[13] == 1.0f && v[14] == -0.5f);
    for (uint32_t i = 0; i < 8; i++) {
        CHECK(ctx, v[i * 3 + 1] == 1.0f);
        CHECK_NEAR(ctx, scalar::sqrt(v[i * 3] * v[i * 3] + v[i * 3 + 2] * v[i * 3 + 2]), 0.5f, 1e-6f);
    }
    Float3 e0 = {v[3] - v[0], v[4] - v[1], v[5] - v[2]};
    Float3 e1 = {v[6] - v[3], v[7] - v[4], v[8] - v[5]};
    Vec3f winding = vec3f::cross(vec3f::load(e0), vec3f::load(e1));
    CHECK(ctx, vec3f::y(winding) > 0.0f);

    // +y: the bottom cap, wound outward
    cylinder_face(io, c, {0.0f, 1.0f, 0.0f}, unit);
    flatten3(v, io.face.vertices, io.face.num_vertices);
    CHECK(ctx, io.face.num_vertices == 8);
    for (uint32_t i = 0; i < 8; i++) CHECK(ctx, v[i * 3 + 1] == -1.0f);
    e0 = {v[3] - v[0], v[4] - v[1], v[5] - v[2]};
    e1 = {v[6] - v[3], v[7] - v[4], v[8] - v[5]};
    winding = vec3f::cross(vec3f::load(e0), vec3f::load(e1));
    CHECK(ctx, vec3f::y(winding) < 0.0f);

    // tilted more than 5 degrees from vertical: the octagon turns so a vertex lies along the direction's xz
    const float tilt = 0.6f;
    cylinder_face(io, c, {tilt * 0.6f, -1.0f, tilt * 0.8f}, unit);
    flatten3(v, io.face.vertices, io.face.num_vertices);
    CHECK(ctx, io.face.num_vertices == 8);
    CHECK_NEAR(ctx, v[6], 0.5f * 0.6f, 1e-6f);
    CHECK_NEAR(ctx, v[8], 0.5f * 0.8f, 1e-6f);

    // mostly horizontal: the side, a vertical segment on the rim opposite the direction
    cylinder_face(io, c, {1.0f, 0.0f, 0.0f}, unit);
    flatten3(v, io.face.vertices, io.face.num_vertices);
    CHECK(ctx, io.face.num_vertices == 2);
    CHECK(ctx, v[0] == -0.5f && v[1] == 1.0f && v[2] == 0.0f);
    CHECK(ctx, v[3] == -0.5f && v[4] == -1.0f && v[5] == 0.0f);
    cylinder_face(io, c, {-0.6f, 0.1f, -0.8f}, unit);
    flatten3(v, io.face.vertices, io.face.num_vertices);
    CHECK(ctx, io.face.num_vertices == 2);
    CHECK_NEAR(ctx, v[0], 0.3f, 1e-6f);
    CHECK_NEAR(ctx, v[2], 0.4f, 1e-6f);
    CHECK(ctx, v[1] == 1.0f && v[4] == -1.0f);

    // a mirrored scale changes nothing: the cylinder is symmetric, so the cap still winds outward
    cylinder_face(io, c, {0.0f, -1.0f, 0.0f}, {-1.0f, 1.0f, 1.0f});
    flatten3(v, io.face.vertices, io.face.num_vertices);
    e0 = {v[3] - v[0], v[4] - v[1], v[5] - v[2]};
    e1 = {v[6] - v[3], v[7] - v[4], v[8] - v[5]};
    winding = vec3f::cross(vec3f::load(e0), vec3f::load(e1));
    CHECK(ctx, vec3f::y(winding) > 0.0f);

    // a tilted cap under a mirrored scale is the same cap as under the positive one: top at y = 3, the
    // rotated vertex at max x = 1
    cylinder_face(io, c, {0.3f, -1.0f, 0.0f}, {-2.0f, 3.0f, -2.0f});
    flatten3(v, io.face.vertices, io.face.num_vertices);
    CHECK(ctx, io.face.num_vertices == 8);
    float max_x = -1e9f;
    for (uint32_t i = 0; i < 8; i++) {
        CHECK(ctx, v[i * 3 + 1] == 3.0f);
        max_x = scalar::max(max_x, v[i * 3]);
    }
    CHECK_NEAR(ctx, max_x, 1.0f, 1e-6f);

    // the transform places the face
    io.transform = translation(vec3f::make(0.0f, 3.0f, 0.0f));
    io.scale = vec3f::make(2.0f, 2.0f, 2.0f);
    cylinder::get_supporting_face(io, vec3f::make(1.0f, 0.0f, 0.0f), c, sub_shape::EMPTY_SUB_SHAPE_ID);
    flatten3(v, io.face.vertices, io.face.num_vertices);
    CHECK(ctx, v[0] == -1.0f && v[1] == 5.0f && v[4] == 1.0f);
    io.scale = vec3f::make(-2.0f, -2.0f, -2.0f);
    cylinder::get_supporting_face(io, vec3f::make(1.0f, 0.0f, 0.0f), c, sub_shape::EMPTY_SUB_SHAPE_ID);
    flatten3(v, io.face.vertices, io.face.num_vertices);
    CHECK(ctx, v[0] == -1.0f && v[1] == 5.0f && v[4] == 1.0f);
    cylinder::destroy(c);
}

void test_cylinder_set_support(TestCtx &ctx) {

    shapes::Shape *c = cylinder::create({.half_height = 1.0f, .radius = 0.5f, .convex_radius = 0.04f});
    support::Support s = support::create();

    cylinder::set_support(s, c, support::INCLUDE_CONVEX_RADIUS, vec3f::make(1.0f, 1.0f, 1.0f));
    CHECK(ctx, s.get_support == support::cylinder_support && !s.has_transform && s.add_radius == 0.0f);
    CHECK(ctx, s.cylinder.half_height == 1.0f && s.cylinder.radius == 0.5f && s.convex_radius == 0.0f);

    cylinder::set_support(s, c, support::DEFAULT, vec3f::make(1.0f, 1.0f, 1.0f));
    CHECK(ctx, s.cylinder.half_height == 1.0f && s.cylinder.radius == 0.5f && s.convex_radius == 0.0f);

    // exclude: shrunk by the convex radius, which is reported
    cylinder::set_support(s, c, support::EXCLUDE_CONVEX_RADIUS, vec3f::make(1.0f, 1.0f, 1.0f));
    CHECK(ctx, s.convex_radius == 0.04f);
    CHECK(ctx, s.cylinder.half_height == 1.0f - 0.04f && s.cylinder.radius == 0.5f - 0.04f);

    // absolute scale; the convex radius scales with it but is capped at the default, as the box's
    cylinder::set_support(s, c, support::INCLUDE_CONVEX_RADIUS, vec3f::make(-3.0f, 3.0f, 3.0f));
    CHECK(ctx, s.cylinder.half_height == 3.0f && s.cylinder.radius == 1.5f);
    cylinder::set_support(s, c, support::EXCLUDE_CONVEX_RADIUS, vec3f::make(-3.0f, 3.0f, 3.0f));
    CHECK(ctx, s.convex_radius == support::DEFAULT_CONVEX_RADIUS);
    CHECK(ctx, s.cylinder.half_height == 3.0f - support::DEFAULT_CONVEX_RADIUS && s.cylinder.radius == 1.5f - support::DEFAULT_CONVEX_RADIUS);

    // x (and z) scale the radius, y the half height; the convex radius by the smallest axis
    cylinder::set_support(s, c, support::INCLUDE_CONVEX_RADIUS, vec3f::make(2.0f, 3.0f, 2.0f));
    CHECK(ctx, s.cylinder.half_height == 3.0f && s.cylinder.radius == 1.0f);
    cylinder::set_support(s, c, support::EXCLUDE_CONVEX_RADIUS, vec3f::make(0.5f, 3.0f, 0.5f));
    const float scaled_radius = 0.5f * 0.04f;
    CHECK(ctx, s.convex_radius == scaled_radius);
    CHECK(ctx, s.cylinder.half_height == 3.0f - scaled_radius && s.cylinder.radius == 0.25f - scaled_radius);
    cylinder::destroy(c);
}

/* material by sub shape id */

void test_cylinder_material(TestCtx &ctx) {
    dispatch::init();
    // a convex shape is one leaf: the empty sub shape id names its own material
    shapes::Shape *painted = cylinder::create({.half_height = 1.0f, .radius = 0.5f, .material_id = 6});
    shapes::Shape *plain = cylinder::create({.half_height = 1.0f, .radius = 0.5f});
    CHECK(ctx, shapes::get_shape_material(painted, sub_shape::EMPTY_SUB_SHAPE_ID) == 6);
    CHECK(ctx, shapes::get_shape_material(plain, sub_shape::EMPTY_SUB_SHAPE_ID) == -1);
    CHECK(ctx, shapes::def_of(painted).get_material == convex::get_convex_material);
    cylinder::destroy(painted);
    cylinder::destroy(plain);
}
