#include "tumble/physics/shapes/shapes.h"

#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "test.h"
#include "test_math.h"

/* the scale a rotated child sees */

void test_shapes_transform_scale(TestCtx &ctx) {
    // uniform scale and the identity rotation pass through
    Vec3f out = shapes::transform_scale(axis_angle(vec3f::make(0, 0, 1), 0.3f), vec3f::make(2, 2, 2));
    CHECK(ctx, vec3f::x(out) == 2 && vec3f::y(out) == 2 && vec3f::z(out) == 2);
    out = shapes::transform_scale(quatf::identity(), vec3f::make(2, 3, 4));
    CHECK(ctx, vec3f::x(out) == 2 && vec3f::y(out) == 3 && vec3f::z(out) == 4);

    // a quarter turn about z swaps x and y, about x swaps y and z
    out = shapes::transform_scale(axis_angle(vec3f::make(0, 0, 1), scalar::PI * 0.5f), vec3f::make(2, 3, 4));
    CHECK(ctx, near3(out, 3, 2, 4, 1e-5f));
    out = shapes::transform_scale(axis_angle(vec3f::make(1, 0, 0), scalar::PI * 0.5f), vec3f::make(2, 3, 4));
    CHECK(ctx, near3(out, 2, 4, 3, 1e-5f));

    // a half turn keeps every axis on itself
    out = shapes::transform_scale(axis_angle(vec3f::make(1, 0, 0), scalar::PI), vec3f::make(2, 3, 4));
    CHECK(ctx, near3(out, 2, 3, 4, 1e-5f));
}

void test_shapes_transform_scale_negated_quaternion(TestCtx &ctx) {
    // -identity is the identity rotation and passes the scale through untouched
    Vec3f out = shapes::transform_scale(quatf::make(0, 0, 0, -1), vec3f::make(2, 3, 4));
    CHECK(ctx, vec3f::x(out) == 2 && vec3f::y(out) == 3 && vec3f::z(out) == 4);

    // -q is the same rotation as q, so a negated quarter turn swaps the same axes
    const Float4 quarter_z = stored_axis_angle(vec3f::make(0, 0, 1), scalar::PI * 0.5f);
    const Float4 negated_quarter_z = {-quarter_z.x, -quarter_z.y, -quarter_z.z, -quarter_z.w};
    const Vec3f from_positive = shapes::transform_scale(quatf::load(quarter_z), vec3f::make(2, 3, 4));
    out = shapes::transform_scale(quatf::load(negated_quarter_z), vec3f::make(2, 3, 4));
    CHECK(ctx, near3(out, from_positive, 1e-6f) && near3(out, 3, 2, 4, 1e-5f));
}
