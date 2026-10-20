#include "physics/shapes/empty.h"

#include "instance.h"
#include "math/mat4f.h"
#include "math/vec3f.h"
#include "physics/shapes/dispatch.h"
#include "test.h"
#include "test_math.h"

void test_empty_shape(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));

    shapes::Shape *e = empty::create();
    CHECK(ctx, e != nullptr);
    CHECK(ctx, e->type == shapes::SHAPE_EMPTY);
    CHECK(ctx, e->volume == 0.0f);
    // a point at the origin
    CHECK(ctx, e->aabb.min.x == 0.0f && e->aabb.min.y == 0.0f && e->aabb.min.z == 0.0f);
    CHECK(ctx, e->aabb.max.x == 0.0f && e->aabb.max.y == 0.0f && e->aabb.max.z == 0.0f);
    CHECK(ctx, e->center_of_mass.x == 0.0f && e->center_of_mass.y == 0.0f && e->center_of_mass.z == 0.0f);

    // unit mass and identity inertia, whatever the caller left there
    mass_properties::MassProperties props = mass_properties::create();
    props.mass = 5.0f;
    props.inertia = {{7, 0, 0, 0, 0, 7, 0, 0, 0, 0, 7, 0, 0, 0, 0, 1}};
    empty::compute_mass_properties(props, e);
    CHECK(ctx, props.mass == 1.0f);
    CHECK(ctx, props.inertia.m[0] == 1.0f && props.inertia.m[5] == 1.0f && props.inertia.m[10] == 1.0f && props.inertia.m[15] == 1.0f);

    // no surface: zero normal, and the accumulated pose is reset
    shapes::SurfaceNormalResult io;
    io.position = vec3f::make(1.0f, 2.0f, 3.0f);
    io.normal = vec3f::make(1.0f, 0.0f, 0.0f);
    io.scale = vec3f::make(2.0f, 2.0f, 2.0f);
    empty::get_surface_normal(io, e, sub_shape::EMPTY_SUB_SHAPE_ID);
    CHECK(ctx, vec3f::x(io.normal) == 0.0f && vec3f::y(io.normal) == 0.0f && vec3f::z(io.normal) == 0.0f);
    CHECK(ctx, vec3f::x(io.position) == 0.0f && vec3f::y(io.position) == 0.0f && vec3f::z(io.position) == 0.0f);
    CHECK(ctx, vec3f::x(io.scale) == 1.0f && vec3f::y(io.scale) == 1.0f && vec3f::z(io.scale) == 1.0f);

    shapes::SupportingFaceResult face_io;
    face_io.face.num_vertices = 3;
    empty::get_supporting_face(face_io, vec3f::make(0.0f, 1.0f, 0.0f), e, sub_shape::EMPTY_SUB_SHAPE_ID);
    CHECK(ctx, face_io.face.num_vertices == 0);

    empty::destroy(e);
}

/* material by sub shape id */

void test_empty_material(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    dispatch::init();
    // no surface, no material
    shapes::Shape *s = empty::create();
    CHECK(ctx, shapes::get_shape_material(s, sub_shape::EMPTY_SUB_SHAPE_ID) == -1);
    empty::destroy(s);
}
