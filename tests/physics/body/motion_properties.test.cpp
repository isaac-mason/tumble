#include "tumble/math/mat4f.h"
#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/dof.h"
#include "tumble/physics/body/mass_properties.h"
#include "tumble/physics/body/motion_properties.h"
#include "test.h"
#include "test_math.h"

using motion_properties::MotionProperties;

static float at(const Float4x4 &m, int row, int col) { return m.m[col * 4 + row]; }

/** a rotation of rad about (x, y, z), stored */
static Float4 turn(float x, float y, float z, float rad) {
    Float4 out;
    return quatf::store(out, axis_angle(vec3f::make(x, y, z), rad));
}

// the inertia set by hand: inverse diagonal (1, 2, 3), turned 0.7 rad about (1, 2, 3)
static MotionProperties make_motion_properties() {
    MotionProperties mp = motion_properties::create();
    mp.inv_mass = 1.0f;
    mp.inv_inertia_diagonal = {1.0f, 2.0f, 3.0f};
    mp.inertia_rotation = turn(1.0f, 2.0f, 3.0f, 0.7f);
    mp.allowed_degrees_of_freedom = dof::DOF_ALL;
    return mp;
}

static Float4x4 inverse_inertia_for_rotation(const MotionProperties &mp, const Float4x4 &rotation) {
    Float4x4 out;
    return mat4f::store(out, motion_properties::get_inverse_inertia_for_rotation(mp, mat4f::load(rotation)));
}

static Float4x4 fresh_world_inverse_inertia(const MotionProperties &mp, const Float4 &q) {
    Float4x4 rotation;
    return inverse_inertia_for_rotation(mp, mat4f::store(rotation, mat4f::from_quat(quatf::load(q))));
}

static Float4x4 world_inverse_inertia(MotionProperties &mp, const Float4 &q, uint32_t step_stamp) {
    Float4x4 out;
    return mat4f::store(out, motion_properties::get_world_inverse_inertia(mp, quatf::load(q), step_stamp));
}

static Vec3f multiply_by_vector(const MotionProperties &mp, const Float4 &q, Vec3f v) {
    return motion_properties::multiply_world_space_inverse_inertia_by_vector(mp, quatf::load(q), v);
}

static const Float4 NO_ROTATION = {0.0f, 0.0f, 0.0f, 1.0f};

static bool mat_equal(const Float4x4 &a, const Float4x4 &b) {
    for (int i = 0; i < 16; i++)
        if (a.m[i] != b.m[i]) return false;
    return true;
}

void test_motion_properties_create(TestCtx &ctx) {
    const MotionProperties mp = motion_properties::create();
    CHECK(ctx, mp.inv_mass == 0.0f);
    CHECK(ctx, mp.linear_damping == 0.0f && mp.angular_damping == 0.0f);
    CHECK(ctx, mp.gravity_factor == 1.0f);
    CHECK(ctx, mp.max_linear_velocity == __builtin_inff() && mp.max_angular_velocity == __builtin_inff());
    CHECK(ctx, mp.motion_quality == motion_properties::MOTION_QUALITY_DISCRETE);
    CHECK(ctx, mp.allowed_degrees_of_freedom == dof::DOF_ALL);
    CHECK(ctx, mp.num_velocity_steps_override == 0 && mp.num_position_steps_override == 0);
    CHECK(ctx, mp.allow_sleeping && mp.sleep_test_timer == 0.0f);
    for (int i = 0; i < 3; i++) CHECK(ctx, mp.sleep_test_spheres[i].radius == 0.0f);
    CHECK(ctx, same_bits(quatf::load(mp.inertia_rotation), quatf::identity()));
    CHECK(ctx, vec3f::exact_equals(vec3f::load(mp.linear_velocity), vec3f::zero()) &&
                   vec3f::exact_equals(vec3f::load(mp.angular_velocity), vec3f::zero()));
    CHECK(ctx, mp.world_inverse_inertia_stamp == motion_properties::STEP_STAMP_NONE);
    bool zero = true;
    for (int i = 0; i < 16; i++) zero = zero && mp.world_inverse_inertia.m[i] == 0.0f;
    CHECK(ctx, zero);
}

void test_motion_properties_damping_and_gravity_factor(TestCtx &ctx) {
    // plain fields: what is written is what the integrator reads
    MotionProperties mp = motion_properties::create();
    mp.linear_damping = 0.05f;
    mp.angular_damping = 0.3f;
    mp.gravity_factor = -0.5f;
    CHECK(ctx, mp.linear_damping == 0.05f);
    CHECK(ctx, mp.angular_damping == 0.3f);
    CHECK(ctx, mp.gravity_factor == -0.5f);
}

void test_motion_properties_multiply_by_vector_matches_matrix(TestCtx &ctx) {
    const MotionProperties mp = make_motion_properties();
    const Float4 body_quat = turn(0.5f, 0.8f, 0.2f, 1.1f);
    const Vec3f input = vec3f::make(10.0f, -5.0f, 7.0f);

    const Float4x4 inv_inertia = fresh_world_inverse_inertia(mp, body_quat);
    const Vec3f by_matrix = mat4f::multiply3x3_vec(mat4f::load(inv_inertia), input);
    const Vec3f direct = multiply_by_vector(mp, body_quat, input);

    CHECK(ctx, near3(by_matrix, direct, 1e-4f));
}

void test_motion_properties_multiply_by_vector_masks_locked_dofs(TestCtx &ctx) {
    MotionProperties mp = make_motion_properties();
    mp.inertia_rotation = NO_ROTATION;
    mp.allowed_degrees_of_freedom = 0b101111;  // all except rotation y

    const Vec3f result = multiply_by_vector(mp, NO_ROTATION, vec3f::make(1.0f, 1.0f, 1.0f));
    CHECK_NEAR(ctx, vec3f::x(result), 1.0f, 1e-6f);
    CHECK(ctx, vec3f::y(result) == 0.0f);
    CHECK_NEAR(ctx, vec3f::z(result), 3.0f, 1e-6f);
}

void test_motion_properties_inverse_inertia_for_rotation(TestCtx &ctx) {
    // a uniform sphere at identity: the world inverse inertia is the local one
    MotionProperties sphere = motion_properties::create();
    sphere.inv_mass = 1.0f;
    sphere.inv_inertia_diagonal = {2.5f, 2.5f, 2.5f};
    Float4x4 identity;
    mat4f::store(identity, mat4f::identity());
    Float4x4 out;
    out = inverse_inertia_for_rotation(sphere, identity);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) CHECK_NEAR(ctx, at(out, i, j), i == j ? 2.5f : 0.0f, 1e-6f);
    CHECK(ctx, out.m[3] == 0.0f && out.m[7] == 0.0f && out.m[11] == 0.0f && out.m[12] == 0.0f && out.m[15] == 1.0f);

    // diagonal (1, 2, 3) at identity, then a quarter turn about z: R D R^T with R = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    // is diag(2, 1, 3), every off-diagonal zero
    MotionProperties box = motion_properties::create();
    box.inv_mass = 1.0f;
    box.inv_inertia_diagonal = {1.0f, 2.0f, 3.0f};
    out = inverse_inertia_for_rotation(box, identity);
    CHECK_NEAR(ctx, at(out, 0, 0), 1.0f, 1e-6f);
    CHECK_NEAR(ctx, at(out, 1, 1), 2.0f, 1e-6f);
    CHECK_NEAR(ctx, at(out, 2, 2), 3.0f, 1e-6f);

    Float4x4 quarter_z;
    mat4f::store(quarter_z, mat4f::from_quat(axis_angle(vec3f::axis_z(), scalar::PI / 2.0f)));
    out = inverse_inertia_for_rotation(box, quarter_z);
    const float expected_quarter[3][3] = {{2, 0, 0}, {0, 1, 0}, {0, 0, 3}};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) CHECK_NEAR(ctx, at(out, i, j), expected_quarter[i][j], 1e-6f);

    // the same quarter turn held in the inertia rotation instead, at identity body rotation
    MotionProperties turned = box;
    turned.inertia_rotation = turn(0.0f, 0.0f, 1.0f, scalar::PI / 2.0f);
    out = inverse_inertia_for_rotation(turned, identity);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) CHECK_NEAR(ctx, at(out, i, j), expected_quarter[i][j], 1e-6f);

    // 45 degrees about y, by hand: R = [[c, 0, s], [0, 1, 0], [-s, 0, c]]
    Float4x4 eighth_y;
    mat4f::store(eighth_y, mat4f::from_quat(axis_angle(vec3f::axis_y(), scalar::PI / 4.0f)));
    out = inverse_inertia_for_rotation(box, eighth_y);
    const float c = scalar::cos(scalar::PI / 4.0f);
    const float s = scalar::sin(scalar::PI / 4.0f);
    CHECK_NEAR(ctx, at(out, 0, 0), c * c * 1.0f + s * s * 3.0f, 1e-5f);
    CHECK_NEAR(ctx, at(out, 1, 1), 2.0f, 1e-5f);
    CHECK_NEAR(ctx, at(out, 2, 2), s * s * 1.0f + c * c * 3.0f, 1e-5f);
    CHECK_NEAR(ctx, at(out, 0, 2), 2.0f * c * s, 1e-5f);
    CHECK_NEAR(ctx, at(out, 2, 0), 2.0f * c * s, 1e-5f);
    CHECK_NEAR(ctx, at(out, 0, 1), 0.0f, 1e-6f);
    CHECK_NEAR(ctx, at(out, 1, 2), 0.0f, 1e-6f);
}

void test_motion_properties_inverse_inertia_symmetric(TestCtx &ctx) {
    MotionProperties mp = make_motion_properties();
    mp.inertia_rotation = turn(1.0f, 1.0f, 1.0f, 0.5f);
    const Float4x4 out = fresh_world_inverse_inertia(mp, turn(0.5f, 0.7f, 0.3f, 1.2f));
    CHECK_NEAR(ctx, at(out, 0, 1), at(out, 1, 0), 1e-6f);
    CHECK_NEAR(ctx, at(out, 0, 2), at(out, 2, 0), 1e-6f);
    CHECK_NEAR(ctx, at(out, 1, 2), at(out, 2, 1), 1e-6f);
    // a rotation keeps the trace: 1 + 2 + 3
    CHECK_NEAR(ctx, at(out, 0, 0) + at(out, 1, 1) + at(out, 2, 2), 6.0f, 1e-5f);
}

void test_motion_properties_inverse_inertia_masks_locked_rotation(TestCtx &ctx) {
    MotionProperties mp = motion_properties::create();
    mp.inv_mass = 1.0f;
    mp.inv_inertia_diagonal = {1.0f, 2.0f, 3.0f};
    mp.allowed_degrees_of_freedom = 0b110111;  // all except rotation x
    Float4x4 identity;
    mat4f::store(identity, mat4f::identity());
    Float4x4 out;
    out = inverse_inertia_for_rotation(mp, identity);
    for (int i = 0; i < 3; i++) {
        CHECK(ctx, at(out, 0, i) == 0.0f);
        CHECK(ctx, at(out, i, 0) == 0.0f);
    }
    CHECK_NEAR(ctx, at(out, 1, 1), 2.0f, 1e-6f);
    CHECK_NEAR(ctx, at(out, 2, 2), 3.0f, 1e-6f);

    // rotated, the locked row and column stay zero while the rest is the turned tensor
    MotionProperties turned = make_motion_properties();
    turned.allowed_degrees_of_freedom = dof::DOF_ALL & ~dof::ROTATION_Z;
    const Float4 q = turn(0.3f, 0.8f, -0.5f, 0.9f);
    const Float4x4 masked = fresh_world_inverse_inertia(turned, q);
    turned.allowed_degrees_of_freedom = dof::DOF_ALL;
    const Float4x4 full = fresh_world_inverse_inertia(turned, q);
    for (int i = 0; i < 3; i++) {
        CHECK(ctx, at(masked, 2, i) == 0.0f);
        CHECK(ctx, at(masked, i, 2) == 0.0f);
    }
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 2; j++) CHECK(ctx, at(masked, i, j) == at(full, i, j));
}

void test_motion_properties_world_inverse_inertia_memo(TestCtx &ctx) {
    // computes once per stamp and returns the memo for repeated calls
    {
        MotionProperties mp = make_motion_properties();
        const Float4 q = turn(0.3f, 0.8f, -0.5f, 1.1f);
        const Float4x4 first = world_inverse_inertia(mp, q, 1);
        CHECK(ctx, mat_equal(first, mp.world_inverse_inertia));
        CHECK(ctx, mp.world_inverse_inertia_stamp == 1);
        CHECK(ctx, mat_equal(first, fresh_world_inverse_inertia(mp, q)));

        // same stamp: the rotation is ignored, the memo is returned untouched
        const Float4 rotated = turn(0.3f, 0.8f, -0.5f, 2.0f);
        const Float4x4 again = world_inverse_inertia(mp, rotated, 1);
        CHECK(ctx, mat_equal(again, mp.world_inverse_inertia));
        CHECK(ctx, mat_equal(again, fresh_world_inverse_inertia(mp, q)));
        CHECK(ctx, !mat_equal(again, fresh_world_inverse_inertia(mp, rotated)));

        // new stamp: recomputed for the rotation passed now
        const Float4x4 next = world_inverse_inertia(mp, rotated, 2);
        CHECK(ctx, mp.world_inverse_inertia_stamp == 2);
        CHECK(ctx, mat_equal(next, fresh_world_inverse_inertia(mp, rotated)));

    }

    // computes fresh for STEP_STAMP_NONE without touching the memo
    {
        MotionProperties mp = make_motion_properties();
        world_inverse_inertia(mp, turn(0.3f, 0.8f, -0.5f, 1.1f), 5);
        const Float4x4 memo = mp.world_inverse_inertia;

        const Float4 rotated = turn(0.3f, 0.8f, -0.5f, 2.0f);
        const Float4x4 result = world_inverse_inertia(mp, rotated, motion_properties::STEP_STAMP_NONE);
        CHECK(ctx, mat_equal(result, fresh_world_inverse_inertia(mp, rotated)));
        CHECK(ctx, mp.world_inverse_inertia_stamp == 5);
        CHECK(ctx, mat_equal(mp.world_inverse_inertia, memo));

        // the stamp after a STEP_STAMP_NONE call still hits the memo
        const Float4x4 hit = world_inverse_inertia(mp, rotated, 5);
        CHECK(ctx, mat_equal(hit, memo));
    }

    // invalidated when the local inertia changes
    {
        MotionProperties mp = make_motion_properties();
        world_inverse_inertia(mp, NO_ROTATION, 3);
        CHECK(ctx, mp.world_inverse_inertia_stamp == 3);

        mass_properties::MassProperties mass = mass_properties::create();
        mass.mass = 2.0f;
        motion_properties::set_mass_properties(mp, dof::DOF_ALL, mass);
        CHECK(ctx, mp.world_inverse_inertia_stamp == motion_properties::STEP_STAMP_NONE);

        // the next call for the same stamp recomputes from the new inertia: unit inertia, so identity
        const Float4x4 recomputed = world_inverse_inertia(mp, turn(0.3f, 0.8f, -0.5f, 0.4f), 3);
        CHECK(ctx, mp.world_inverse_inertia_stamp == 3);
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) CHECK_NEAR(ctx, at(recomputed, i, j), i == j ? 1.0f : 0.0f, 1e-5f);
    }
}

void test_motion_properties_decompose_principal_moments(TestCtx &ctx) {
    // an axis-aligned 2 x 1 x 3 box: the moments (5, 6.5, 2.5) come back sorted, the axes as a right handed basis
    mass_properties::MassProperties slab = mass_properties::create();
    mass_properties::set_mass_and_inertia_of_solid_box(slab, vec3f::make(2.0f, 1.0f, 3.0f), 1.0f);
    Mat4f rotation;
    Vec3f diagonal;
    CHECK(ctx, motion_properties::decompose_principal_moments_of_inertia(mat4f::load(slab.inertia), rotation, diagonal));
    CHECK_NEAR(ctx, vec3f::x(diagonal), 6.5f, 1e-5f);
    CHECK_NEAR(ctx, vec3f::y(diagonal), 5.0f, 1e-5f);
    CHECK_NEAR(ctx, vec3f::z(diagonal), 2.5f, 1e-5f);
    CHECK_NEAR(ctx, mat4f::determinant(rotation), 1.0f, 1e-5f);

    // the same box turned: the moments are unchanged and R diag R^T rebuilds the tensor
    mass_properties::MassProperties turned = mass_properties::create();
    mass_properties::rotate(turned, slab, mat4f::from_quat(axis_angle(vec3f::make(1.0f, 2.0f, 3.0f), 0.7f)));
    CHECK(ctx, motion_properties::decompose_principal_moments_of_inertia(mat4f::load(turned.inertia), rotation, diagonal));
    CHECK_NEAR(ctx, vec3f::x(diagonal), 6.5f, 1e-4f);
    CHECK_NEAR(ctx, vec3f::y(diagonal), 5.0f, 1e-4f);
    CHECK_NEAR(ctx, vec3f::z(diagonal), 2.5f, 1e-4f);
    CHECK_NEAR(ctx, mat4f::determinant(rotation), 1.0f, 1e-5f);
    const float d[3] = {vec3f::x(diagonal), vec3f::y(diagonal), vec3f::z(diagonal)};
    Float4x4 r;
    mat4f::store(r, rotation);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 3; k++) sum += r.m[k * 4 + i] * d[k] * r.m[k * 4 + j];
            CHECK_NEAR(ctx, sum, at(turned.inertia, i, j), 1e-4f);
        }
    }
}

void test_motion_properties_set_mass_properties_box(TestCtx &ctx) {
    // a 2 x 1 x 3 box of density 1: 6 kg, inertia diag (5, 6.5, 2.5)
    mass_properties::MassProperties slab = mass_properties::create();
    mass_properties::set_mass_and_inertia_of_solid_box(slab, vec3f::make(2.0f, 1.0f, 3.0f), 1.0f);

    MotionProperties mp = motion_properties::create();
    motion_properties::set_mass_properties(mp, dof::DOF_ALL, slab);
    CHECK(ctx, mp.allowed_degrees_of_freedom == dof::DOF_ALL);
    CHECK_NEAR(ctx, mp.inv_mass, 1.0f / 6.0f, 1e-7f);
    // the principal moments sorted descending, inverted
    CHECK_NEAR(ctx, mp.inv_inertia_diagonal.x, 1.0f / 6.5f, 1e-6f);
    CHECK_NEAR(ctx, mp.inv_inertia_diagonal.y, 1.0f / 5.0f, 1e-6f);
    CHECK_NEAR(ctx, mp.inv_inertia_diagonal.z, 1.0f / 2.5f, 1e-6f);
    CHECK_NEAR(ctx, quat_length(quatf::load(mp.inertia_rotation)), 1.0f, 1e-5f);

    // whatever the inertia rotation, the body-space inverse inertia is the box's diagonal inverted
    const Float4x4 local = fresh_world_inverse_inertia(mp, NO_ROTATION);
    const float expected[3] = {1.0f / 5.0f, 1.0f / 6.5f, 1.0f / 2.5f};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) CHECK_NEAR(ctx, at(local, i, j), i == j ? expected[i] : 0.0f, 1e-5f);

    // a quarter turn about z swaps the x and y moments in world space: R I^-1 R^T
    const Float4x4 world = fresh_world_inverse_inertia(mp, turn(0.0f, 0.0f, 1.0f, scalar::PI / 2.0f));
    const float swapped[3] = {1.0f / 6.5f, 1.0f / 5.0f, 1.0f / 2.5f};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) CHECK_NEAR(ctx, at(world, i, j), i == j ? swapped[i] : 0.0f, 1e-5f);

    // a unit cube's inertia is isotropic: 1/6 m on each axis
    mass_properties::MassProperties cube = mass_properties::create();
    mass_properties::set_mass_and_inertia_of_solid_box(cube, vec3f::make(1.0f, 1.0f, 1.0f), 1000.0f);
    motion_properties::set_mass_properties(mp, dof::DOF_ALL, cube);
    CHECK_NEAR(ctx, mp.inv_mass, 1e-3f, 1e-9f);
    CHECK_NEAR(ctx, mp.inv_inertia_diagonal.x, 6e-3f, 1e-8f);
    CHECK_NEAR(ctx, mp.inv_inertia_diagonal.y, 6e-3f, 1e-8f);
    CHECK_NEAR(ctx, mp.inv_inertia_diagonal.z, 6e-3f, 1e-8f);

    // a zero inertia tensor falls back to the unit sphere's: 2.5 / m
    mass_properties::MassProperties point = mass_properties::create();
    point.mass = 2.0f;
    mat4f::store(point.inertia, mat4f::zero());
    motion_properties::set_mass_properties(mp, dof::DOF_ALL, point);
    CHECK_NEAR(ctx, mp.inv_inertia_diagonal.x, 1.25f, 1e-6f);
    CHECK_NEAR(ctx, mp.inv_inertia_diagonal.y, 1.25f, 1e-6f);
    CHECK_NEAR(ctx, mp.inv_inertia_diagonal.z, 1.25f, 1e-6f);
    CHECK(ctx, same_bits(quatf::load(mp.inertia_rotation), quatf::identity()));
}

void test_motion_properties_dof_locking(TestCtx &ctx) {
    mass_properties::MassProperties slab = mass_properties::create();
    mass_properties::set_mass_and_inertia_of_solid_box(slab, vec3f::make(2.0f, 1.0f, 3.0f), 1.0f);

    // translation locked entirely: the mass term is zero, rotation untouched
    MotionProperties rotating = motion_properties::create();
    motion_properties::set_mass_properties(rotating, dof::DOF_ROTATION_ONLY, slab);
    CHECK(ctx, rotating.inv_mass == 0.0f);
    CHECK_NEAR(ctx, rotating.inv_inertia_diagonal.x, 1.0f / 6.5f, 1e-6f);

    // rotation locked entirely: no inverse inertia, identity inertia rotation
    MotionProperties sliding = motion_properties::create();
    motion_properties::set_mass_properties(sliding, dof::DOF_TRANSLATION_ONLY, slab);
    CHECK_NEAR(ctx, sliding.inv_mass, 1.0f / 6.0f, 1e-7f);
    CHECK(ctx, vec3f::exact_equals(vec3f::load(sliding.inv_inertia_diagonal), vec3f::zero()));
    CHECK(ctx, same_bits(quatf::load(sliding.inertia_rotation), quatf::identity()));
    sliding.angular_velocity = {0.1f, 0.2f, 0.3f};
    motion_properties::add_angular_impulse(sliding, vec3f::make(5.0f, 5.0f, 5.0f), quatf::identity());
    CHECK(ctx, vec3f::exact_equals(vec3f::load(sliding.angular_velocity), vec3f::make(0.1f, 0.2f, 0.3f)));

    // one translation axis locked: the mass stays, that velocity component is zeroed by the solver steps
    MotionProperties planar = motion_properties::create();
    motion_properties::set_mass_properties(planar, dof::DOF_ALL & ~dof::TRANSLATION_Y, slab);
    CHECK_NEAR(ctx, planar.inv_mass, 1.0f / 6.0f, 1e-7f);
    motion_properties::add_linear_velocity_step(planar, vec3f::make(1.0f, 2.0f, 3.0f));
    CHECK(ctx, vec3f::exact_equals(vec3f::load(planar.linear_velocity), vec3f::make(1.0f, 0.0f, 3.0f)));
    motion_properties::sub_linear_velocity_step(planar, vec3f::make(0.5f, 7.0f, 1.0f));
    CHECK(ctx, vec3f::exact_equals(vec3f::load(planar.linear_velocity), vec3f::make(0.5f, 0.0f, 2.0f)));
    const Vec3f velocity = motion_properties::apply_translation_dof_constraint(vec3f::make(4.0f, 5.0f, 6.0f), planar.allowed_degrees_of_freedom);
    CHECK(ctx, vec3f::exact_equals(velocity, vec3f::make(4.0f, 0.0f, 6.0f)));
    // angular steps do not mask
    motion_properties::add_angular_velocity_step(planar, vec3f::make(1.0f, 2.0f, 3.0f));
    motion_properties::sub_angular_velocity_step(planar, vec3f::make(0.0f, 0.5f, 0.0f));
    CHECK(ctx, vec3f::exact_equals(vec3f::load(planar.angular_velocity), vec3f::make(1.0f, 1.5f, 3.0f)));

    // one rotation axis locked: that row and column of the world inverse inertia are zero, at any rotation
    MotionProperties hinge = motion_properties::create();
    motion_properties::set_mass_properties(hinge, dof::DOF_ALL & ~dof::ROTATION_X, slab);
    const Float4x4 world = fresh_world_inverse_inertia(hinge, turn(0.3f, 0.8f, -0.5f, 0.9f));
    for (int i = 0; i < 3; i++) {
        CHECK(ctx, at(world, 0, i) == 0.0f);
        CHECK(ctx, at(world, i, 0) == 0.0f);
    }
    CHECK(ctx, at(world, 1, 1) > 0.0f && at(world, 2, 2) > 0.0f);
    const Vec3f angular_delta = motion_properties::apply_rotation_dof_constraint(vec3f::make(9.0f, 9.0f, 9.0f), hinge.allowed_degrees_of_freedom);
    CHECK(ctx, vec3f::exact_equals(angular_delta, vec3f::make(0.0f, 9.0f, 9.0f)));
}

void test_motion_properties_velocity_clamps(TestCtx &ctx) {
    MotionProperties mp = motion_properties::create();
    mp.max_linear_velocity = 2.0f;
    mp.max_angular_velocity = 1.0f;

    // over the limit: the direction is kept, the length is the limit
    motion_properties::set_linear_velocity(mp, vec3f::make(3.0f, 4.0f, 0.0f));
    CHECK_NEAR(ctx, mp.linear_velocity.x, 1.2f, 1e-6f);
    CHECK_NEAR(ctx, mp.linear_velocity.y, 1.6f, 1e-6f);
    CHECK(ctx, mp.linear_velocity.z == 0.0f);

    // under the limit: untouched
    motion_properties::set_linear_velocity(mp, vec3f::make(1.0f, -1.0f, 0.5f));
    CHECK(ctx, vec3f::exact_equals(vec3f::load(mp.linear_velocity), vec3f::make(1.0f, -1.0f, 0.5f)));

    // adding clamps the sum
    motion_properties::add_linear_velocity(mp, vec3f::make(0.0f, 0.0f, 10.0f));
    CHECK_NEAR(ctx, vec3f::length(vec3f::load(mp.linear_velocity)), 2.0f, 1e-5f);

    motion_properties::set_angular_velocity(mp, vec3f::make(0.0f, 0.0f, -5.0f));
    CHECK(ctx, vec3f::exact_equals(vec3f::load(mp.angular_velocity), vec3f::make(0.0f, 0.0f, -1.0f)));
    motion_properties::set_angular_velocity(mp, vec3f::make(0.5f, 0.0f, 0.0f));
    motion_properties::add_angular_velocity(mp, vec3f::make(0.0f, 2.0f, 0.0f));
    CHECK_NEAR(ctx, vec3f::length(vec3f::load(mp.angular_velocity)), 1.0f, 1e-6f);
    CHECK_NEAR(ctx, mp.angular_velocity.y / mp.angular_velocity.x, 4.0f, 1e-4f);

    // the solver steps do not clamp
    motion_properties::add_angular_velocity_step(mp, vec3f::make(0.0f, 10.0f, 0.0f));
    CHECK(ctx, vec3f::length(vec3f::load(mp.angular_velocity)) > 10.0f);

    // the default limits are infinite: nothing clamps
    MotionProperties unbounded = motion_properties::create();
    motion_properties::set_linear_velocity(unbounded, vec3f::make(1e20f, 0.0f, 0.0f));
    CHECK(ctx, unbounded.linear_velocity.x == 1e20f);
}

void test_motion_properties_forces_and_impulses(TestCtx &ctx) {
    MotionProperties mp = motion_properties::create();
    mp.inv_mass = 0.5f;
    mp.inv_inertia_diagonal = {1.0f, 2.0f, 3.0f};

    motion_properties::add_force(mp, vec3f::make(1.0f, 2.0f, 3.0f));
    motion_properties::add_torque(mp, vec3f::make(0.5f, 0.0f, 0.0f));
    // a force along z one metre along x from the centre of mass: torque r x F = (0, -1, 0)
    motion_properties::add_force_at_position(mp, vec3f::make(0.0f, 0.0f, 1.0f), vec3f::make(2.0f, 1.0f, 1.0f), vec3f::make(1.0f, 1.0f, 1.0f));
    CHECK(ctx, vec3f::exact_equals(vec3f::load(mp.force), vec3f::make(1.0f, 2.0f, 4.0f)));
    CHECK(ctx, vec3f::exact_equals(vec3f::load(mp.torque), vec3f::make(0.5f, -1.0f, 0.0f)));

    // an impulse changes velocity by J / m
    motion_properties::add_impulse(mp, vec3f::make(2.0f, 0.0f, -4.0f));
    CHECK(ctx, vec3f::exact_equals(vec3f::load(mp.linear_velocity), vec3f::make(1.0f, 0.0f, -2.0f)));

    // an angular impulse changes angular velocity by I^-1 J, here at identity
    motion_properties::add_angular_impulse(mp, vec3f::make(1.0f, 1.0f, 1.0f), quatf::identity());
    CHECK_NEAR(ctx, mp.angular_velocity.x, 1.0f, 1e-6f);
    CHECK_NEAR(ctx, mp.angular_velocity.y, 2.0f, 1e-6f);
    CHECK_NEAR(ctx, mp.angular_velocity.z, 3.0f, 1e-6f);

    // an impulse along y one metre along x: dv = J / m, dw = I^-1 (r x J) = I^-1 (0, 0, 1)
    MotionProperties off_centre = motion_properties::create();
    off_centre.inv_mass = 0.5f;
    off_centre.inv_inertia_diagonal = {1.0f, 2.0f, 3.0f};
    motion_properties::add_impulse_at_position(off_centre, vec3f::make(0.0f, 1.0f, 0.0f), vec3f::make(1.0f, 0.0f, 0.0f), vec3f::make(0.0f, 0.0f, 0.0f),
                                               quatf::identity());
    CHECK(ctx, vec3f::exact_equals(vec3f::load(off_centre.linear_velocity), vec3f::make(0.0f, 0.5f, 0.0f)));
    CHECK_NEAR(ctx, off_centre.angular_velocity.x, 0.0f, 1e-6f);
    CHECK_NEAR(ctx, off_centre.angular_velocity.y, 0.0f, 1e-6f);
    CHECK_NEAR(ctx, off_centre.angular_velocity.z, 3.0f, 1e-6f);

    // an infinite mass takes no linear impulse
    MotionProperties immovable = motion_properties::create();
    motion_properties::add_impulse(immovable, vec3f::make(5.0f, 5.0f, 5.0f));
    CHECK(ctx, vec3f::exact_equals(vec3f::load(immovable.linear_velocity), vec3f::zero()));

    // impulses clamp
    MotionProperties capped = motion_properties::create();
    capped.inv_mass = 1.0f;
    capped.max_linear_velocity = 1.0f;
    motion_properties::add_impulse(capped, vec3f::make(0.0f, 3.0f, 0.0f));
    CHECK(ctx, vec3f::exact_equals(vec3f::load(capped.linear_velocity), vec3f::make(0.0f, 1.0f, 0.0f)));
}

void test_motion_properties_scale_to_mass(TestCtx &ctx) {
    mass_properties::MassProperties slab = mass_properties::create();
    mass_properties::set_mass_and_inertia_of_solid_box(slab, vec3f::make(2.0f, 1.0f, 3.0f), 1.0f);
    MotionProperties mp = motion_properties::create();
    motion_properties::set_mass_properties(mp, dof::DOF_ALL, slab);

    // twice the mass halves the inverse inertia
    motion_properties::scale_to_mass(mp, 12.0f);
    CHECK_NEAR(ctx, mp.inv_mass, 1.0f / 12.0f, 1e-7f);
    CHECK_NEAR(ctx, mp.inv_inertia_diagonal.x, 1.0f / 13.0f, 1e-6f);
    CHECK_NEAR(ctx, mp.inv_inertia_diagonal.y, 1.0f / 10.0f, 1e-6f);
    CHECK_NEAR(ctx, mp.inv_inertia_diagonal.z, 1.0f / 5.0f, 1e-6f);

    // the same as setting the box at the new mass
    mass_properties::scale_to_mass(slab, 12.0f);
    MotionProperties direct = motion_properties::create();
    motion_properties::set_mass_properties(direct, dof::DOF_ALL, slab);
    CHECK_NEAR(ctx, direct.inv_inertia_diagonal.x, mp.inv_inertia_diagonal.x, 1e-6f);
    CHECK_NEAR(ctx, direct.inv_inertia_diagonal.y, mp.inv_inertia_diagonal.y, 1e-6f);
    CHECK_NEAR(ctx, direct.inv_inertia_diagonal.z, mp.inv_inertia_diagonal.z, 1e-6f);
}

void test_motion_properties_move_kinematic(TestCtx &ctx) {
    MotionProperties mp = motion_properties::create();

    // (1, 2, 3) and a quarter turn about y in half a second: v = (2, 4, 6), w = (0, pi, 0)
    motion_properties::move_kinematic(mp, vec3f::make(1.0f, 2.0f, 3.0f), axis_angle(vec3f::axis_y(), scalar::PI / 2.0f), 0.5f);
    CHECK_NEAR(ctx, mp.linear_velocity.x, 2.0f, 1e-6f);
    CHECK_NEAR(ctx, mp.linear_velocity.y, 4.0f, 1e-6f);
    CHECK_NEAR(ctx, mp.linear_velocity.z, 6.0f, 1e-6f);
    CHECK_NEAR(ctx, mp.angular_velocity.x, 0.0f, 1e-6f);
    CHECK_NEAR(ctx, mp.angular_velocity.y, scalar::PI, 1e-5f);
    CHECK_NEAR(ctx, mp.angular_velocity.z, 0.0f, 1e-6f);

    // no rotation: no angular velocity
    motion_properties::move_kinematic(mp, vec3f::make(0.0f, 0.0f, 0.0f), quatf::identity(), 0.5f);
    CHECK(ctx, vec3f::length(vec3f::load(mp.angular_velocity)) == 0.0f);

    // 170 degrees about y takes the short way round whichever twin of the quaternion arrives: never 190 the other way
    const float big = 170.0f * scalar::DEGREES_TO_RADIANS;
    const Float4 half_turn = turn(0.0f, 1.0f, 0.0f, big);
    motion_properties::move_kinematic(mp, vec3f::make(0.0f, 0.0f, 0.0f), quatf::load(half_turn), 1.0f);
    CHECK_NEAR(ctx, mp.angular_velocity.y, big, 1e-4f);
    CHECK_NEAR(ctx, mp.angular_velocity.x, 0.0f, 1e-6f);
    const Float4 twin = {-half_turn.x, -half_turn.y, -half_turn.z, -half_turn.w};
    motion_properties::move_kinematic(mp, vec3f::make(0.0f, 0.0f, 0.0f), quatf::load(twin), 1.0f);
    CHECK_NEAR(ctx, mp.angular_velocity.y, big, 1e-4f);
    CHECK_NEAR(ctx, mp.angular_velocity.z, 0.0f, 1e-6f);

    // locked axes are zeroed
    mp.allowed_degrees_of_freedom = dof::DOF_ALL & ~dof::TRANSLATION_X & ~dof::ROTATION_Y;
    motion_properties::move_kinematic(mp, vec3f::make(1.0f, 2.0f, 3.0f), axis_angle(vec3f::make(0.0f, 1.0f, 1.0f), 1.0f), 1.0f);
    CHECK(ctx, mp.linear_velocity.x == 0.0f && mp.linear_velocity.y == 2.0f);
    CHECK(ctx, mp.angular_velocity.y == 0.0f);
    CHECK_NEAR(ctx, mp.angular_velocity.z, scalar::sqrt(0.5f), 1e-5f);

    // a non-positive delta time is a no-op
    const Vec3f before = vec3f::load(mp.linear_velocity);
    motion_properties::move_kinematic(mp, vec3f::make(9.0f, 9.0f, 9.0f), quatf::identity(), 0.0f);
    CHECK(ctx, vec3f::exact_equals(vec3f::load(mp.linear_velocity), before));
}

void test_motion_properties_gyroscopic_force(TestCtx &ctx) {
    MotionProperties mp = make_motion_properties();
    const Quatf q = axis_angle(vec3f::make(0.5f, 0.8f, 0.2f), 1.1f);

    // a spin about a principal axis is steady: w x (I w) is zero
    const Quatf principal = quatf::multiply(q, quatf::load(mp.inertia_rotation));
    const Vec3f steady = vec3f::scale(vec3f::transform_quat(vec3f::axis_y(), principal), 3.0f);
    vec3f::store(mp.angular_velocity, steady);
    motion_properties::apply_gyroscopic_force(mp, q, 1.0f / 60.0f);
    CHECK(ctx, near3(mp.angular_velocity, steady, 1e-5f));

    // an off-axis spin precesses with its angular momentum's magnitude kept
    mp.angular_velocity = {1.0f, 2.0f, -0.5f};
    const Quatf to_local = quatf::conjugate(principal);
    const Vec3f inertia = vec3f::make(1.0f, 2.0f, 3.0f);
    const Vec3f local_before = vec3f::transform_quat(vec3f::load(mp.angular_velocity), to_local);
    const Vec3f momentum_before = vec3f::divide(local_before, inertia);
    motion_properties::apply_gyroscopic_force(mp, q, 1.0f / 60.0f);
    const Vec3f local_after = vec3f::transform_quat(vec3f::load(mp.angular_velocity), to_local);
    const Vec3f momentum_after = vec3f::divide(local_after, inertia);
    CHECK_NEAR(ctx, vec3f::length(momentum_after), vec3f::length(momentum_before), 1e-5f);
    CHECK(ctx, !near3(local_after, local_before, 1e-6f));
}
