#include "tumble/physics/shapes/empty.h"

#include "tumble/math/mat4f.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/shapes/dispatch.h"
#include "test.h"
#include "test_math.h"

void test_empty_shape(TestCtx &ctx) {
    dispatch::init();

    Ref<Shape> e = empty::create();
    CHECK(ctx, e != nullptr);
    CHECK(ctx, e->type == ShapeType::EMPTY);
    CHECK(ctx, e->volume == 0.0f);
    // a point at the origin
    CHECK(ctx, vec3f::exact_equals(e->aabb.min, vec3f::zero()) && vec3f::exact_equals(e->aabb.max, vec3f::zero()));
    CHECK(ctx, vec3f::exact_equals(e->center_of_mass, vec3f::zero()));

    // unit mass and identity inertia, whatever the caller left there
    MassProperties props = mass_properties::create();
    props.mass = 5.0f;
    props.inertia = mat4f::from_scaling(vec3f::splat(7.0f));
    empty::compute_mass_properties(props, e);
    CHECK(ctx, props.mass == 1.0f);
    CHECK(ctx, same_bits(props.inertia, mat4f::identity()));

    // no surface: zero normal, and the accumulated pose is reset
    SurfaceNormalResult io;
    io.position = vec3f::make(1.0f, 2.0f, 3.0f);
    io.normal = vec3f::make(1.0f, 0.0f, 0.0f);
    io.scale = vec3f::make(2.0f, 2.0f, 2.0f);
    empty::get_surface_normal(io, e, sub_shape::EMPTY_SUB_SHAPE_ID);
    CHECK(ctx, vec3f::x(io.normal) == 0.0f && vec3f::y(io.normal) == 0.0f && vec3f::z(io.normal) == 0.0f);
    CHECK(ctx, vec3f::x(io.position) == 0.0f && vec3f::y(io.position) == 0.0f && vec3f::z(io.position) == 0.0f);
    CHECK(ctx, vec3f::x(io.scale) == 1.0f && vec3f::y(io.scale) == 1.0f && vec3f::z(io.scale) == 1.0f);

    SupportingFaceResult face_io;
    face_io.face.num_vertices = 3;
    empty::get_supporting_face(face_io, vec3f::make(0.0f, 1.0f, 0.0f), e, sub_shape::EMPTY_SUB_SHAPE_ID);
    CHECK(ctx, face_io.face.num_vertices == 0);

    e = nullptr;
}

/* material by sub shape id */

void test_empty_material(TestCtx &ctx) {
    dispatch::init();
    // no surface, no material
    Ref<Shape> s = empty::create();
    CHECK(ctx, shapes::get_shape_material(s, sub_shape::EMPTY_SUB_SHAPE_ID) == -1);
    s = nullptr;
}
