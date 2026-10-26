#include "tumble/physics/shapes/capsule.h"

#include "tumble/math/mat4f.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/shapes/convex.h"
#include "tumble/physics/shapes/dispatch.h"
#include "test.h"
#include "test_math.h"

void test_capsule_create(TestCtx &ctx) {
    dispatch::init();

    Ref<Shape> c = capsule::create({.half_height_of_cylinder = 2.0f, .radius = 0.5f});
    CHECK(ctx, c != nullptr);
    CHECK(ctx, c->type == ShapeType::CAPSULE);
    CHECK(ctx, shapes::as<CapsuleShape>(c).half_height_of_cylinder == 2.0f && shapes::as<CapsuleShape>(c).radius == 0.5f);
    CHECK(ctx, shapes::as<CapsuleShape>(c).density == 1000.0f);
    CHECK(ctx, c->material_id == -1);

    // bounds reach half height plus radius along y, the radius across
    CHECK(ctx, c->aabb.min.x == -0.5f && c->aabb.min.y == -2.5f && c->aabb.min.z == -0.5f);
    CHECK(ctx, c->aabb.max.x == 0.5f && c->aabb.max.y == 2.5f && c->aabb.max.z == 0.5f);
    CHECK(ctx, c->center_of_mass.x == 0.0f && c->center_of_mass.y == 0.0f && c->center_of_mass.z == 0.0f);

    // cylinder of height 2h plus a whole sphere
    CHECK_NEAR(ctx, c->volume, scalar::PI * 0.25f * 4.0f + 4.0f / 3.0f * scalar::PI * 0.125f, 1e-5f);

    Ref<Shape> unit = capsule::create({.half_height_of_cylinder = 1.0f, .radius = 1.0f});
    CHECK_NEAR(ctx, unit->volume, 2.0f * scalar::PI + 4.0f / 3.0f * scalar::PI, 1e-5f);
    CHECK(ctx, capsule::get_inner_radius(unit) == 1.0f);
    CHECK(ctx, capsule::get_inner_radius(c) == 0.5f);

    CapsuleShapeSettings custom = {.half_height_of_cylinder = 2.0f, .radius = 1.0f};
    custom.density = 500.0f;
    custom.material_id = 7;
    Ref<Shape> d = capsule::create(custom);
    CHECK(ctx, shapes::as<CapsuleShape>(d).density == 500.0f && d->material_id == 7);

    c = nullptr;
    unit = nullptr;
    d = nullptr;
}

/** a solid cylinder plus two hemispheres, each 2/5 m r^2 about its own centre, moved out by h + 3r/8. */
static void expected_inertia(float h, float r, float density, float &i_xx_zz, float &i_yy) {
    const float cylinder_mass = scalar::PI * 2.0f * h * r * r * density;
    const float hemisphere_mass = 2.0f / 3.0f * scalar::PI * r * r * r * density;
    const float height = 2.0f * h;
    i_yy = 0.5f * cylinder_mass * r * r + 2.0f * (2.0f / 5.0f) * hemisphere_mass * r * r;
    i_xx_zz = cylinder_mass * (height * height / 12.0f + r * r / 4.0f) + 2.0f * (2.0f / 5.0f) * hemisphere_mass * r * r +
              hemisphere_mass * (0.5f * height * height + 0.75f * height * r);
}

void test_capsule_mass_properties(TestCtx &ctx) {
    dispatch::init();

    Ref<Shape> c = capsule::create({.half_height_of_cylinder = 1.0f, .radius = 1.0f});
    MassProperties props = mass_properties::create();
    capsule::compute_mass_properties(props, c);
    CHECK_NEAR(ctx, props.mass, 1000.0f * c->volume, 1e-1f);

    float i_xx_zz, i_yy;
    expected_inertia(1.0f, 1.0f, 1000.0f, i_xx_zz, i_yy);
    CHECK_NEAR(ctx, props.inertia.m[0], i_xx_zz, 1.0f);
    CHECK_NEAR(ctx, props.inertia.m[5], i_yy, 1.0f);
    CHECK_NEAR(ctx, props.inertia.m[10], i_xx_zz, 1.0f);
    CHECK(ctx, props.inertia.m[0] == props.inertia.m[10]);
    CHECK(ctx, props.inertia.m[5] < props.inertia.m[0]);
    CHECK(ctx, props.inertia.m[15] == 1.0f);
    CHECK(ctx, props.inertia.m[1] == 0.0f && props.inertia.m[2] == 0.0f && props.inertia.m[4] == 0.0f);
    CHECK(ctx, props.inertia.m[6] == 0.0f && props.inertia.m[8] == 0.0f && props.inertia.m[9] == 0.0f);

    // twice the density, twice the mass and inertia
    CapsuleShapeSettings dense = {.half_height_of_cylinder = 1.0f, .radius = 1.0f};
    dense.density = 2000.0f;
    Ref<Shape> c2 = capsule::create(dense);
    MassProperties props2 = mass_properties::create();
    capsule::compute_mass_properties(props2, c2);
    CHECK_NEAR(ctx, props2.mass, props.mass * 2.0f, 1e-1f);
    CHECK_NEAR(ctx, props2.inertia.m[0], props.inertia.m[0] * 2.0f, 1.0f);
    CHECK_NEAR(ctx, props2.inertia.m[5], props.inertia.m[5] * 2.0f, 1.0f);

    // long and thin: spinning about the axis is far easier than tumbling
    Ref<Shape> rod = capsule::create({.half_height_of_cylinder = 10.0f, .radius = 0.5f});
    capsule::compute_mass_properties(props, rod);
    CHECK(ctx, props.inertia.m[5] < props.inertia.m[0] * 0.1f);

    // short and wide: close to a sphere
    Ref<Shape> ball = capsule::create({.half_height_of_cylinder = 0.1f, .radius = 2.0f});
    capsule::compute_mass_properties(props, ball);
    CHECK(ctx, props.inertia.m[5] / props.inertia.m[0] > 0.5f);

    // almost no cylinder (create refuses none): a sphere, 2/5 m r^2 on every diagonal entry, to the sliver's share
    Ref<Shape> sphere_like = capsule::create({.half_height_of_cylinder = 1e-3f, .radius = 2.0f});
    capsule::compute_mass_properties(props, sphere_like);
    const float sphere_inertia = 0.4f * props.mass * 4.0f;
    CHECK_NEAR(ctx, props.mass, 1000.0f * 4.0f / 3.0f * scalar::PI * 8.0f, 30.0f);
    CHECK_NEAR(ctx, props.inertia.m[0] / sphere_inertia, 1.0f, 2e-3f);
    CHECK_NEAR(ctx, props.inertia.m[5] / sphere_inertia, 1.0f, 2e-3f);
    CHECK_NEAR(ctx, props.inertia.m[10] / props.inertia.m[5], 1.0f, 2e-3f);
    sphere_like = nullptr;

    c = nullptr;
    c2 = nullptr;
    rod = nullptr;
    ball = nullptr;
}

void test_capsule_surface_normal(TestCtx &ctx) {
    dispatch::init();

    Ref<Shape> c = capsule::create({.half_height_of_cylinder = 1.0f, .radius = 0.5f});
    SurfaceNormalResult io;

    // on the cylinder wall: radial
    io.position = vec3f::make(0.0f, 0.3f, -0.5f);
    capsule::get_surface_normal(io, c, sub_shape::EMPTY_SUB_SHAPE_ID);
    CHECK_NEAR(ctx, vec3f::x(io.normal), 0.0f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::y(io.normal), 0.0f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::z(io.normal), -1.0f, 1e-6f);

    // on the top cap: away from the top sphere's center
    const float k = 0.5f / scalar::sqrt(2.0f);
    io.position = vec3f::make(k, 1.0f + k, 0.0f);
    capsule::get_surface_normal(io, c, sub_shape::EMPTY_SUB_SHAPE_ID);
    const float n = 1.0f / scalar::sqrt(2.0f);
    CHECK_NEAR(ctx, vec3f::x(io.normal), n, 1e-6f);
    CHECK_NEAR(ctx, vec3f::y(io.normal), n, 1e-6f);
    CHECK_NEAR(ctx, vec3f::z(io.normal), 0.0f, 1e-6f);

    // the bottom pole
    io.position = vec3f::make(0.0f, -1.5f, 0.0f);
    capsule::get_surface_normal(io, c, sub_shape::EMPTY_SUB_SHAPE_ID);
    CHECK(ctx, vec3f::x(io.normal) == 0.0f && vec3f::y(io.normal) == -1.0f && vec3f::z(io.normal) == 0.0f);

    // on the axis inside the cylinder: the arbitrary radial x
    io.position = vec3f::make(0.0f, 0.5f, 0.0f);
    capsule::get_surface_normal(io, c, sub_shape::EMPTY_SUB_SHAPE_ID);
    CHECK(ctx, vec3f::x(io.normal) == 1.0f && vec3f::y(io.normal) == 0.0f && vec3f::z(io.normal) == 0.0f);

    c = nullptr;
}

void test_capsule_supporting_face(TestCtx &ctx) {
    dispatch::init();

    Ref<Shape> c = capsule::create({.half_height_of_cylinder = 1.0f, .radius = 0.5f});
    SupportingFaceResult io;
    io.transform = mat4f::identity();
    io.scale = vec3f::make(1.0f, 1.0f, 1.0f);

    // side on: the line down the wall opposite the direction
    capsule::get_supporting_face(io, vec3f::make(2.0f, 0.0f, 0.0f), c, sub_shape::EMPTY_SUB_SHAPE_ID);
    CHECK(ctx, io.face.num_vertices == 2);
    CHECK_NEAR(ctx, vec3f::x(io.face.vertices[0]), -0.5f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::y(io.face.vertices[0]), 1.0f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::z(io.face.vertices[0]), 0.0f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::x(io.face.vertices[1]), -0.5f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::y(io.face.vertices[1]), -1.0f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::z(io.face.vertices[1]), 0.0f, 1e-6f);

    // through the transform (translation) and scale
    io.transform = translation(vec3f::make(10.0f, 20.0f, 30.0f));
    io.scale = vec3f::make(2.0f, 2.0f, 2.0f);
    capsule::get_supporting_face(io, vec3f::make(0.0f, 0.0f, 1.0f), c, sub_shape::EMPTY_SUB_SHAPE_ID);
    CHECK(ctx, io.face.num_vertices == 2);
    CHECK_NEAR(ctx, vec3f::x(io.face.vertices[0]), 10.0f, 1e-5f);
    CHECK_NEAR(ctx, vec3f::y(io.face.vertices[0]), 22.0f, 1e-5f);
    CHECK_NEAR(ctx, vec3f::z(io.face.vertices[0]), 29.0f, 1e-5f);
    CHECK_NEAR(ctx, vec3f::y(io.face.vertices[1]), 18.0f, 1e-5f);
    CHECK_NEAR(ctx, vec3f::z(io.face.vertices[1]), 29.0f, 1e-5f);

    // a mirrored scale is the same line: the capsule is symmetric, and sized by |scale|
    io.scale = vec3f::make(-2.0f, -2.0f, -2.0f);
    capsule::get_supporting_face(io, vec3f::make(0.0f, 0.0f, 1.0f), c, sub_shape::EMPTY_SUB_SHAPE_ID);
    CHECK(ctx, io.face.num_vertices == 2);
    CHECK_NEAR(ctx, vec3f::y(io.face.vertices[0]), 22.0f, 1e-5f);
    CHECK_NEAR(ctx, vec3f::z(io.face.vertices[0]), 29.0f, 1e-5f);
    CHECK_NEAR(ctx, vec3f::y(io.face.vertices[1]), 18.0f, 1e-5f);
    io.scale = vec3f::make(2.0f, 2.0f, 2.0f);

    // tilted well off the side: only one point is relevant
    capsule::get_supporting_face(io, vec3f::make(1.0f, 1.0f, 0.0f), c, sub_shape::EMPTY_SUB_SHAPE_ID);
    CHECK(ctx, io.face.num_vertices == 0);

    // end on: no face
    capsule::get_supporting_face(io, vec3f::make(0.0f, -1.0f, 0.0f), c, sub_shape::EMPTY_SUB_SHAPE_ID);
    CHECK(ctx, io.face.num_vertices == 0);

    c = nullptr;
}

void test_capsule_set_support(TestCtx &ctx) {
    dispatch::init();

    Ref<Shape> c = capsule::create({.half_height_of_cylinder = 1.0f, .radius = 0.5f});
    Support sup;
    sup.add_radius = 9.0f;
    sup.has_transform = true;

    // include: a rounded segment, scaled by |scale.x|
    capsule::set_support(sup, c, SupportFunctionMode::INCLUDE_CONVEX_RADIUS, vec3f::make(-3.0f, 3.0f, 3.0f));
    CHECK(ctx, sup.get_support == support::capsule_support);
    CHECK(ctx, sup.capsule.half_height == 3.0f);
    CHECK(ctx, sup.capsule.radius == 1.5f);
    CHECK(ctx, sup.convex_radius == 0.0f);
    CHECK(ctx, sup.add_radius == 0.0f);
    CHECK(ctx, !sup.has_transform);

    // exclude: the bare segment, the radius reported
    capsule::set_support(sup, c, SupportFunctionMode::EXCLUDE_CONVEX_RADIUS, vec3f::make(1.0f, 1.0f, 1.0f));
    CHECK(ctx, sup.capsule.half_height == 1.0f);
    CHECK(ctx, sup.capsule.radius == 0.0f);
    CHECK(ctx, sup.convex_radius == 0.5f);

    // evaluated: the cap's pole up the axis, the wall's radius across it, the segment's end without the radius
    Support evaluated = support::create();
    capsule::set_support(evaluated, c, SupportFunctionMode::INCLUDE_CONVEX_RADIUS, vec3f::make(1.0f, 1.0f, 1.0f));
    Vec3f point = evaluated.get_support(evaluated, vec3f::make(0.0f, 2.0f, 0.0f));
    CHECK_NEAR(ctx, vec3f::x(point), 0.0f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::y(point), 1.5f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::z(point), 0.0f, 1e-6f);
    point = evaluated.get_support(evaluated, vec3f::make(0.0f, -1.0f, -1.0f));
    const float k = 0.5f / scalar::sqrt(2.0f);
    CHECK_NEAR(ctx, vec3f::x(point), 0.0f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::y(point), -1.0f - k, 1e-6f);
    CHECK_NEAR(ctx, vec3f::z(point), -k, 1e-6f);
    capsule::set_support(evaluated, c, SupportFunctionMode::EXCLUDE_CONVEX_RADIUS, vec3f::make(1.0f, 1.0f, 1.0f));
    point = evaluated.get_support(evaluated, vec3f::make(1.0f, 1.0f, 0.0f));
    CHECK_NEAR(ctx, vec3f::x(point), 0.0f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::y(point), 1.0f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::z(point), 0.0f, 1e-6f);

    // default reads as exclude for a capsule
    capsule::set_support(sup, c, SupportFunctionMode::DEFAULT, vec3f::make(1.0f, 1.0f, 1.0f));
    CHECK(ctx, sup.capsule.radius == 0.0f);
    CHECK(ctx, sup.convex_radius == 0.5f);

    c = nullptr;
}

/* material by sub shape id */

void test_capsule_material(TestCtx &ctx) {
    dispatch::init();
    // a convex shape is one leaf: the empty sub shape id names its own material
    Ref<Shape> painted = capsule::create({.half_height_of_cylinder = 1.0f, .radius = 0.5f, .material_id = 6});
    Ref<Shape> plain = capsule::create({.half_height_of_cylinder = 1.0f, .radius = 0.5f});
    CHECK(ctx, shapes::get_shape_material(painted, sub_shape::EMPTY_SUB_SHAPE_ID) == 6);
    CHECK(ctx, shapes::get_shape_material(plain, sub_shape::EMPTY_SUB_SHAPE_ID) == -1);
    CHECK(ctx, shapes::def_of(painted).get_material == convex::get_convex_material);
    painted = nullptr;
    plain = nullptr;
}
