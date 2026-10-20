#include "physics/shapes/sphere.h"

#include "instance.h"
#include "math/scalar.h"
#include "math/vec3f.h"
#include "physics/shapes/convex.h"
#include "physics/shapes/dispatch.h"
#include "test.h"
#include "test_math.h"

void test_sphere_create(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));

    shapes::Shape *s = sphere::create({.radius = 2.0f});
    CHECK(ctx, s != nullptr);
    CHECK(ctx, s->type == shapes::SHAPE_SPHERE);
    CHECK(ctx, s->sphere.radius == 2.0f);
    CHECK(ctx, s->sphere.density == 1000.0f);
    CHECK(ctx, s->material_id == -1);

    // bounds +-r
    CHECK(ctx, s->aabb.min.x == -2.0f && s->aabb.min.y == -2.0f && s->aabb.min.z == -2.0f);
    CHECK(ctx, s->aabb.max.x == 2.0f && s->aabb.max.y == 2.0f && s->aabb.max.z == 2.0f);
    CHECK(ctx, s->center_of_mass.x == 0.0f && s->center_of_mass.y == 0.0f && s->center_of_mass.z == 0.0f);

    // V = 4/3 pi r^3
    CHECK_NEAR(ctx, s->volume, 4.0f / 3.0f * scalar::PI * 8.0f, 1e-4f);

    // update follows a radius change
    s->sphere.radius = 0.5f;
    sphere::update(s);
    CHECK(ctx, s->aabb.max.y == 0.5f && s->aabb.min.x == -0.5f);
    CHECK_NEAR(ctx, s->volume, 4.0f / 3.0f * scalar::PI * 0.125f, 1e-6f);

    sphere::SphereShapeSettings custom = {.radius = 1.0f};
    custom.density = 500.0f;
    custom.material_id = 3;
    shapes::Shape *c = sphere::create(custom);
    CHECK(ctx, c->sphere.density == 500.0f && c->material_id == 3);

    sphere::destroy(s);
    sphere::destroy(c);
}

void test_sphere_mass_properties(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));

    shapes::Shape *s = sphere::create({.radius = 1.0f});
    mass_properties::MassProperties props = mass_properties::create();
    sphere::compute_mass_properties(props, s);
    const float mass = 4.0f / 3.0f * scalar::PI * 1000.0f;
    CHECK_NEAR(ctx, props.mass, mass, 1e-2f);
    // solid sphere: 2/5 m r^2 on the diagonal, nothing off it
    const float inertia = 0.4f * mass;
    CHECK_NEAR(ctx, props.inertia.m[0], inertia, 1e-2f);
    CHECK_NEAR(ctx, props.inertia.m[5], inertia, 1e-2f);
    CHECK_NEAR(ctx, props.inertia.m[10], inertia, 1e-2f);
    CHECK(ctx, props.inertia.m[15] == 1.0f);
    CHECK(ctx, props.inertia.m[1] == 0.0f && props.inertia.m[2] == 0.0f && props.inertia.m[4] == 0.0f);
    CHECK(ctx, props.inertia.m[6] == 0.0f && props.inertia.m[8] == 0.0f && props.inertia.m[9] == 0.0f);

    // radius 2: mass scales by r^3, inertia by r^5
    shapes::Shape *big = sphere::create({.radius = 2.0f});
    sphere::compute_mass_properties(props, big);
    CHECK_NEAR(ctx, props.mass, mass * 8.0f, 1e-1f);
    CHECK_NEAR(ctx, props.inertia.m[0], inertia * 32.0f, 1.0f);

    sphere::destroy(s);
    sphere::destroy(big);
}

void test_sphere_surface_normal_and_radius(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));

    shapes::Shape *s = sphere::create({.radius = 2.0f});
    CHECK(ctx, sphere::get_inner_radius(s) == 2.0f);

    shapes::SurfaceNormalResult io;
    // a point on the surface: the outward normal is the point over r
    io.position = vec3f::make(0.0f, 0.0f, 2.0f);
    sphere::get_surface_normal(io, s, sub_shape::EMPTY_SUB_SHAPE_ID);
    CHECK_NEAR(ctx, vec3f::x(io.normal), 0.0f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::y(io.normal), 0.0f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::z(io.normal), 1.0f, 1e-6f);

    const float k = 2.0f / scalar::sqrt(3.0f);
    io.position = vec3f::make(-k, k, -k);
    sphere::get_surface_normal(io, s, sub_shape::EMPTY_SUB_SHAPE_ID);
    const float n = 1.0f / scalar::sqrt(3.0f);
    CHECK_NEAR(ctx, vec3f::x(io.normal), -n, 1e-6f);
    CHECK_NEAR(ctx, vec3f::y(io.normal), n, 1e-6f);
    CHECK_NEAR(ctx, vec3f::z(io.normal), -n, 1e-6f);
    CHECK_NEAR(ctx, vec3f::length(io.normal), 1.0f, 1e-6f);

    // the origin falls back to +y
    io.position = vec3f::zero();
    sphere::get_surface_normal(io, s, sub_shape::EMPTY_SUB_SHAPE_ID);
    CHECK(ctx, vec3f::x(io.normal) == 0.0f && vec3f::y(io.normal) == 1.0f && vec3f::z(io.normal) == 0.0f);

    // a sphere has no supporting face
    shapes::SupportingFaceResult face_io;
    face_io.face.num_vertices = 7;
    sphere::get_supporting_face(face_io, vec3f::make(1.0f, 0.0f, 0.0f), s, sub_shape::EMPTY_SUB_SHAPE_ID);
    CHECK(ctx, face_io.face.num_vertices == 0);

    sphere::destroy(s);
}

void test_sphere_set_support(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));

    shapes::Shape *s = sphere::create({.radius = 1.5f});
    support::Support sup;
    sup.add_radius = 9.0f;
    sup.has_transform = true;

    // include: a rounded point, the whole radius in the core; scale's x magnitude applies
    sphere::set_support(sup, s, support::INCLUDE_CONVEX_RADIUS, vec3f::make(-2.0f, 2.0f, 2.0f));
    CHECK(ctx, sup.get_support == support::sphere_support);
    CHECK(ctx, sup.sphere.radius == 3.0f);
    CHECK(ctx, sup.convex_radius == 0.0f);
    CHECK(ctx, sup.add_radius == 0.0f);
    CHECK(ctx, !sup.has_transform);

    // exclude: the core is the origin, the radius is reported
    sphere::set_support(sup, s, support::EXCLUDE_CONVEX_RADIUS, vec3f::make(1.0f, 1.0f, 1.0f));
    CHECK(ctx, sup.sphere.radius == 0.0f);
    CHECK(ctx, sup.convex_radius == 1.5f);

    // evaluated: r along any direction in include mode, the origin in exclude mode
    support::Support evaluated = support::create();
    sphere::set_support(evaluated, s, support::INCLUDE_CONVEX_RADIUS, vec3f::make(1.0f, 1.0f, 1.0f));
    Vec3f point = evaluated.get_support(evaluated, vec3f::make(0.0f, -3.0f, 0.0f));
    CHECK_NEAR(ctx, vec3f::x(point), 0.0f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::y(point), -1.5f, 1e-6f);
    CHECK_NEAR(ctx, vec3f::z(point), 0.0f, 1e-6f);
    point = evaluated.get_support(evaluated, vec3f::make(1.0f, 1.0f, 1.0f));
    const float d = 1.5f / scalar::sqrt(3.0f);
    CHECK_NEAR(ctx, vec3f::x(point), d, 1e-6f);
    CHECK_NEAR(ctx, vec3f::y(point), d, 1e-6f);
    CHECK_NEAR(ctx, vec3f::z(point), d, 1e-6f);
    sphere::set_support(evaluated, s, support::EXCLUDE_CONVEX_RADIUS, vec3f::make(1.0f, 1.0f, 1.0f));
    point = evaluated.get_support(evaluated, vec3f::make(1.0f, 0.0f, 0.0f));
    CHECK(ctx, vec3f::length(point) == 0.0f);

    // default reads as exclude for a sphere
    sphere::set_support(sup, s, support::DEFAULT, vec3f::make(2.0f, 2.0f, 2.0f));
    CHECK(ctx, sup.sphere.radius == 0.0f);
    CHECK(ctx, sup.convex_radius == 3.0f);

    sphere::destroy(s);
}

/* material by sub shape id */

void test_sphere_material(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    // a convex shape is one leaf: the empty sub shape id names its own material
    shapes::Shape *painted = sphere::create({.radius = 1.0f, .material_id = 6});
    shapes::Shape *plain = sphere::create({.radius = 1.0f});
    CHECK(ctx, shapes::get_shape_material(painted, sub_shape::EMPTY_SUB_SHAPE_ID) == 6);
    CHECK(ctx, shapes::get_shape_material(plain, sub_shape::EMPTY_SUB_SHAPE_ID) == -1);
    CHECK(ctx, shapes::def_of(painted).get_material == convex::get_convex_material);
    sphere::destroy(painted);
    sphere::destroy(plain);
}
