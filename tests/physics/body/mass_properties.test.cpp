#include "tumble/math/mat4f.h"
#include "tumble/math/quatf.h"
#include "tumble/physics/body/mass_properties.h"
#include "test.h"
#include "test_math.h"


static float at(const MassProperties &mp, int row, int col) { return mp.inertia.m[col * 4 + row]; }

void test_mass_properties_solid_box(TestCtx &ctx) {
    // a unit cube of water: 1000 kg, m (b^2 + c^2) / 12 = 1000 / 6 on the diagonal
    MassProperties cube = mass_properties::create();
    mass_properties::set_mass_and_inertia_of_solid_box(cube, vec3f::make(1.0f, 1.0f, 1.0f), 1000.0f);
    CHECK_NEAR(ctx, cube.mass, 1000.0f, 1e-3f);
    for (int i = 0; i < 3; i++) CHECK_NEAR(ctx, at(cube, i, i), 1000.0f / 6.0f, 1e-3f);
    CHECK(ctx, at(cube, 0, 1) == 0.0f && at(cube, 0, 2) == 0.0f && at(cube, 1, 2) == 0.0f);
    CHECK(ctx, cube.inertia.m[15] == 1.0f);

    // a 2 x 1 x 3 box: m = 6 kg at density 1; ixx = m (1 + 9) / 12, iyy = m (4 + 9) / 12, izz = m (4 + 1) / 12
    MassProperties slab = mass_properties::create();
    mass_properties::set_mass_and_inertia_of_solid_box(slab, vec3f::make(2.0f, 1.0f, 3.0f), 1.0f);
    CHECK_NEAR(ctx, slab.mass, 6.0f, 1e-5f);
    CHECK_NEAR(ctx, at(slab, 0, 0), 5.0f, 1e-5f);
    CHECK_NEAR(ctx, at(slab, 1, 1), 6.5f, 1e-5f);
    CHECK_NEAR(ctx, at(slab, 2, 2), 2.5f, 1e-5f);
}

void test_mass_properties_scale(TestCtx &ctx) {
    // scaling a unit cube by (2, 1, 1) gives the 2 x 1 x 1 box at the same density
    MassProperties cube = mass_properties::create();
    mass_properties::set_mass_and_inertia_of_solid_box(cube, vec3f::make(1.0f, 1.0f, 1.0f), 1000.0f);
    MassProperties scaled = mass_properties::create();
    mass_properties::scale(scaled, cube, vec3f::make(2.0f, 1.0f, 1.0f));
    MassProperties expected = mass_properties::create();
    mass_properties::set_mass_and_inertia_of_solid_box(expected, vec3f::make(2.0f, 1.0f, 1.0f), 1000.0f);
    CHECK_NEAR(ctx, scaled.mass, 2000.0f, 1e-2f);
    for (int i = 0; i < 16; i++) CHECK_NEAR(ctx, scaled.inertia.m[i], expected.inertia.m[i], 1e-2f);

    // a mirror scale keeps a positive mass, and flips the sign of the products of inertia it crosses
    MassProperties skewed = mass_properties::create();
    skewed.mass = 2.0f;
    skewed.inertia.m[0] = 3.0f;
    skewed.inertia.m[5] = 4.0f;
    skewed.inertia.m[10] = 5.0f;
    skewed.inertia.m[4] = skewed.inertia.m[1] = 0.5f;
    MassProperties mirrored = mass_properties::create();
    mass_properties::scale(mirrored, skewed, vec3f::make(-1.0f, 1.0f, 1.0f));
    CHECK_NEAR(ctx, mirrored.mass, 2.0f, 1e-6f);
    CHECK_NEAR(ctx, at(mirrored, 0, 0), 3.0f, 1e-5f);
    CHECK_NEAR(ctx, at(mirrored, 1, 1), 4.0f, 1e-5f);
    CHECK_NEAR(ctx, at(mirrored, 2, 2), 5.0f, 1e-5f);
    CHECK_NEAR(ctx, at(mirrored, 0, 1), -0.5f, 1e-6f);
    CHECK_NEAR(ctx, at(mirrored, 1, 0), -0.5f, 1e-6f);
}

void test_mass_properties_scale_to_mass(TestCtx &ctx) {
    MassProperties slab = mass_properties::create();
    mass_properties::set_mass_and_inertia_of_solid_box(slab, vec3f::make(2.0f, 1.0f, 3.0f), 1.0f);
    mass_properties::scale_to_mass(slab, 18.0f);
    // three times the mass, three times every moment; the ratios stay
    CHECK_NEAR(ctx, slab.mass, 18.0f, 1e-5f);
    CHECK_NEAR(ctx, at(slab, 0, 0), 15.0f, 1e-4f);
    CHECK_NEAR(ctx, at(slab, 1, 1), 19.5f, 1e-4f);
    CHECK_NEAR(ctx, at(slab, 2, 2), 7.5f, 1e-4f);
    CHECK(ctx, slab.inertia.m[15] == 1.0f);

    // a massless input only takes the mass
    MassProperties massless = mass_properties::create();
    mass_properties::scale_to_mass(massless, 5.0f);
    CHECK(ctx, massless.mass == 5.0f);
    CHECK(ctx, at(massless, 0, 0) == 1.0f);
}

void test_mass_properties_rotate(TestCtx &ctx) {
    MassProperties slab = mass_properties::create();
    mass_properties::set_mass_and_inertia_of_solid_box(slab, vec3f::make(2.0f, 1.0f, 3.0f), 1.0f);

    // a quarter turn about z swaps the x and y moments
    Float4 q = stored_axis_angle(vec3f::make(0.0f, 0.0f, 1.0f), scalar::PI * 0.5f);
    Float4x4 rotation;
    mat4f::store(rotation, mat4f::from_quat(quatf::load(q)));
    MassProperties rotated = mass_properties::create();
    mass_properties::rotate(rotated, slab, mat4f::load(rotation));
    CHECK_NEAR(ctx, rotated.mass, 6.0f, 1e-6f);
    CHECK_NEAR(ctx, at(rotated, 0, 0), 6.5f, 1e-4f);
    CHECK_NEAR(ctx, at(rotated, 1, 1), 5.0f, 1e-4f);
    CHECK_NEAR(ctx, at(rotated, 2, 2), 2.5f, 1e-4f);
    CHECK_NEAR(ctx, at(rotated, 0, 1), 0.0f, 1e-4f);
    CHECK(ctx, rotated.inertia.m[15] == 1.0f);

    // an eighth turn mixes x and y: the trace is kept and the product is (iyy - ixx) / 2 in magnitude
    q = stored_axis_angle(vec3f::make(0.0f, 0.0f, 1.0f), scalar::PI * 0.25f);
    mat4f::store(rotation, mat4f::from_quat(quatf::load(q)));
    mass_properties::rotate(rotated, slab, mat4f::load(rotation));
    CHECK_NEAR(ctx, at(rotated, 0, 0), 5.75f, 1e-4f);
    CHECK_NEAR(ctx, at(rotated, 1, 1), 5.75f, 1e-4f);
    CHECK_NEAR(ctx, scalar::abs(at(rotated, 0, 1)), 0.75f, 1e-4f);
    CHECK_NEAR(ctx, at(rotated, 0, 1), at(rotated, 1, 0), 1e-6f);

    // in place
    mass_properties::rotate(slab, slab, mat4f::load(rotation));
    for (int i = 0; i < 16; i++) CHECK_NEAR(ctx, slab.inertia.m[i], rotated.inertia.m[i], 1e-6f);
}

void test_mass_properties_translate(TestCtx &ctx) {
    MassProperties cube = mass_properties::create();
    mass_properties::set_mass_and_inertia_of_solid_box(cube, vec3f::make(1.0f, 1.0f, 1.0f), 1000.0f);

    // parallel axis: I + m (d.d E - d d^T)
    const Float3 d = {1.0f, 2.0f, 3.0f};
    MassProperties moved = mass_properties::create();
    mass_properties::translate(moved, cube, vec3f::load(d));
    const float m = 1000.0f;
    const float base = 1000.0f / 6.0f;
    CHECK_NEAR(ctx, moved.mass, m, 1e-3f);
    CHECK_NEAR(ctx, at(moved, 0, 0), base + m * (14.0f - 1.0f), 1e-1f);
    CHECK_NEAR(ctx, at(moved, 1, 1), base + m * (14.0f - 4.0f), 1e-1f);
    CHECK_NEAR(ctx, at(moved, 2, 2), base + m * (14.0f - 9.0f), 1e-1f);
    CHECK_NEAR(ctx, at(moved, 0, 1), -m * 2.0f, 1e-2f);
    CHECK_NEAR(ctx, at(moved, 0, 2), -m * 3.0f, 1e-2f);
    CHECK_NEAR(ctx, at(moved, 1, 2), -m * 6.0f, 1e-2f);
    CHECK_NEAR(ctx, at(moved, 2, 1), -m * 6.0f, 1e-2f);
    CHECK(ctx, moved.inertia.m[15] == 1.0f && moved.inertia.m[3] == 0.0f);

    // a point mass moved one metre along x: the textbook m d^2 about y and z, nothing about x
    MassProperties point = mass_properties::create();
    point.mass = 2.0f;
    for (int i = 0; i < 3; i++) point.inertia.m[i * 5] = 0.0f;
    mass_properties::translate(point, point, vec3f::make(1.0f, 0.0f, 0.0f));
    CHECK_NEAR(ctx, at(point, 0, 0), 0.0f, 1e-6f);
    CHECK_NEAR(ctx, at(point, 1, 1), 2.0f, 1e-6f);
    CHECK_NEAR(ctx, at(point, 2, 2), 2.0f, 1e-6f);
}
