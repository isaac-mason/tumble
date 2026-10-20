#include "tumble/math/scalar.h"
#include "tumble/math/aabox.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/collision/cast_utils.h"
#include "test.h"
#include "test_math.h"
#include "test_rng.h"

static constexpr float MISS = __FLT_MAX__;

static const Box3f UNIT_BOX = {{-1, -1, -1}, {1, 1, 1}};

static float fraction(const Float3 &origin, const Float3 &direction, float length, const Box3f &box) {
    const Float3 inv = {cast_utils::safe_reciprocal(direction.x * length), cast_utils::safe_reciprocal(direction.y * length),
                       cast_utils::safe_reciprocal(direction.z * length)};
    return cast_utils::ray_fraction_to_box3(vec3f::load(origin), vec3f::load(inv), aabox::load(box));
}

void test_cast_utils_ray_fraction_to_box3(TestCtx &ctx) {
    // entry fraction along the segment
    CHECK_NEAR(ctx, fraction({-5, 0, 0}, {1, 0, 0}, 10, UNIT_BOX), 0.4f, 1e-7f);
    CHECK_NEAR(ctx, fraction({0, 5, 0}, {0, -1, 0}, 20, UNIT_BOX), 0.2f, 1e-7f);
    // origin inside the box gives zero
    CHECK(ctx, fraction({0.2f, -0.3f, 0.1f}, {0, 0, 1}, 5, UNIT_BOX) == 0.0f);
    // segment ending before the box misses
    CHECK(ctx, fraction({-5, 0, 0}, {1, 0, 0}, 3, UNIT_BOX) == MISS);
    // box behind the origin misses
    CHECK(ctx, fraction({5, 0, 0}, {1, 0, 0}, 10, UNIT_BOX) == MISS);
    // axis-parallel rays: inside the slab hit, outside the slab miss
    CHECK_NEAR(ctx, fraction({-5, 0.5f, -0.5f}, {1, 0, 0}, 10, UNIT_BOX), 0.4f, 1e-7f);
    CHECK(ctx, fraction({-5, 1.5f, 0}, {1, 0, 0}, 10, UNIT_BOX) == MISS);
    CHECK(ctx, fraction({-5, 0, -1.5f}, {1, 0, 0}, 10, UNIT_BOX) == MISS);
    // negative zero and tiny components behave the same
    CHECK_NEAR(ctx, fraction({-5, 0.5f, 0.5f}, {1, -0.0f, 1e-40f}, 10, UNIT_BOX), 0.4f, 1e-7f);
    // origin exactly on a slab face counts as inside
    CHECK_NEAR(ctx, fraction({-5, 1, 0}, {1, 0, 0}, 10, UNIT_BOX), 0.4f, 1e-7f);
}

void test_cast_utils_ray_fraction_agrees_with_distance(TestCtx &ctx) {
    uint32_t seed = 12345;
    const auto rand = [&seed]() { return (float)((double)lcg32(seed) / 4294967296.0); };
    uint32_t disagreements = 0;
    uint32_t hits = 0;
    // the ts's sample (i < 2000) hits rarely; the second half aims each ray at its box's centre, give or take
    for (int i = 0; i < 4000; i++) {
        const bool aimed = i >= 2000;
        Box3f box;
        float *mins = &box.min.x;
        float *maxs = &box.max.x;
        for (int k = 0; k < 3; k++) {
            const float a = rand() * 10 - 5;
            const float b = a + rand() * 4;
            mins[k] = a;
            maxs[k] = b;
        }
        const Float3 origin = {rand() * 20 - 10, rand() * 20 - 10, rand() * 20 - 10};
        Float3 direction = {rand() * 2 - 1, rand() * 2 - 1, rand() * 2 - 1};
        if (aimed) {
            direction.x = (mins[0] + maxs[0]) * 0.5f - origin.x + direction.x * 0.5f;
            direction.y = (mins[1] + maxs[1]) * 0.5f - origin.y + direction.y * 0.5f;
            direction.z = (mins[2] + maxs[2]) * 0.5f - origin.z + direction.z * 0.5f;
        }
        if (i % 7 == 0) direction.x = 0;
        if (i % 11 == 0) direction.y = 0;
        const float len = vec3f::length(vec3f::load(direction));
        if (len > 0.0f) vec3f::store(direction, vec3f::scale(vec3f::load(direction), 1.0f / len));
        const float length = rand() * 30;

        const float expected = cast_utils::ray_distance_to_box3({vec3f::load(origin), vec3f::load(direction), length}, aabox::load(box));
        const float actual = fraction(origin, direction, length, box);
        if (expected == MISS) {
            if (actual != MISS) disagreements++;
        } else {
            hits++;
            if (scalar::abs(actual - expected) > 1e-5f) disagreements++;
        }
        // a finite entry means the segment touches the box
        CHECK(ctx, cast_utils::ray_hits_box3({vec3f::load(origin), vec3f::load(direction), length}, aabox::load(box)) == (expected != MISS));
    }
    CHECK(ctx, disagreements == 0);
    CHECK(ctx, hits > 900);
}

void test_cast_utils_ray_distance_to_box3(TestCtx &ctx) {
    // normalised: 4 of a length 10 ray to the near face
    CHECK_NEAR(ctx, cast_utils::ray_distance_to_box3({vec3f::make(-5, 0, 0), vec3f::make(1, 0, 0), 10}, aabox::load(UNIT_BOX)), 0.4f, 1e-7f);
    // from the far side along -z
    CHECK_NEAR(ctx, cast_utils::ray_distance_to_box3({vec3f::make(0, 0, 9), vec3f::make(0, 0, -1), 16}, aabox::load(UNIT_BOX)), 0.5f, 1e-7f);
    // inside: 0; a zero-length ray inside: 0 (the overlap of a still cast); outside: a miss
    CHECK(ctx, cast_utils::ray_distance_to_box3({vec3f::make(0.5f, 0, 0), vec3f::make(0, 1, 0), 3}, aabox::load(UNIT_BOX)) == 0.0f);
    CHECK(ctx, cast_utils::ray_distance_to_box3({vec3f::make(0.5f, 0, 0), vec3f::make(0, 0, 0), 0}, aabox::load(UNIT_BOX)) == 0.0f);
    CHECK(ctx, cast_utils::ray_distance_to_box3({vec3f::make(3, 0, 0), vec3f::make(0, 0, 0), 0}, aabox::load(UNIT_BOX)) == MISS);
    // too short, pointing away, parallel outside the slab
    CHECK(ctx, cast_utils::ray_distance_to_box3({vec3f::make(-5, 0, 0), vec3f::make(1, 0, 0), 3.9f}, aabox::load(UNIT_BOX)) == MISS);
    CHECK(ctx, cast_utils::ray_distance_to_box3({vec3f::make(-5, 0, 0), vec3f::make(-1, 0, 0), 100}, aabox::load(UNIT_BOX)) == MISS);
    CHECK(ctx, cast_utils::ray_distance_to_box3({vec3f::make(-5, 2, 0), vec3f::make(1, 0, 0), 100}, aabox::load(UNIT_BOX)) == MISS);
    // diagonal into the corner region: enters through x = -1 at t = 4 sqrt 2 of the 10 sqrt 2 ray
    const float r = 1.0f / scalar::sqrt(2.0f);
    CHECK_NEAR(ctx, cast_utils::ray_distance_to_box3({vec3f::make(-5, -4.5f, 0), vec3f::make(r, r, 0), 10 * scalar::sqrt(2.0f)}, aabox::load(UNIT_BOX)), 0.4f, 1e-6f);

    // ray_hits_box3 on the same cases
    CHECK(ctx, cast_utils::ray_hits_box3({vec3f::make(-5, 0, 0), vec3f::make(1, 0, 0), 10}, aabox::load(UNIT_BOX)));
    CHECK(ctx, cast_utils::ray_hits_box3({vec3f::make(0.5f, 0, 0), vec3f::make(0, 0, 0), 0}, aabox::load(UNIT_BOX)));
    CHECK(ctx, !cast_utils::ray_hits_box3({vec3f::make(-5, 0, 0), vec3f::make(1, 0, 0), 3.9f}, aabox::load(UNIT_BOX)));
    CHECK(ctx, !cast_utils::ray_hits_box3({vec3f::make(-5, 0, 0), vec3f::make(-1, 0, 0), 100}, aabox::load(UNIT_BOX)));
    CHECK(ctx, !cast_utils::ray_hits_box3({vec3f::make(-5, 2, 0), vec3f::make(1, 0, 0), 100}, aabox::load(UNIT_BOX)));
    // grazing the face plane counts
    CHECK(ctx, cast_utils::ray_hits_box3({vec3f::make(-5, 1, 0), vec3f::make(1, 0, 0), 100}, aabox::load(UNIT_BOX)));
}

void test_cast_utils_safe_reciprocal(TestCtx &ctx) {
    CHECK(ctx, cast_utils::safe_reciprocal(4.0f) == 0.25f);
    CHECK(ctx, cast_utils::safe_reciprocal(-0.5f) == -2.0f);
    CHECK(ctx, cast_utils::safe_reciprocal(0.0f) == 0.0f);
    CHECK(ctx, cast_utils::safe_reciprocal(-0.0f) == 0.0f);
    CHECK(ctx, cast_utils::safe_reciprocal(1e-31f) == 0.0f);
    CHECK(ctx, cast_utils::safe_reciprocal(1e-29f) != 0.0f);
}

void test_cast_utils_ray_intersects_triangle(TestCtx &ctx) {
    // a triangle in z = 0, counter-clockwise seen from +z: its front faces +z
    const Float3 a = {0, 0, 0}, b = {2, 0, 0}, c = {0, 2, 0};
    cast_utils::RayIntersectsTriangleResult result = cast_utils::create_result();

    // from above, down through (0.5, 0.5): 3 along a length 4 ray
    const cast_ray::Rayf down = {vec3f::make(0.5f, 0.5f, 3), vec3f::make(0, 0, -1), 4};
    cast_utils::ray_intersects_triangle(result, down, down.length, vec3f::load(a), vec3f::load(b), vec3f::load(c), true);
    CHECK(ctx, result.hit && result.front_facing);
    CHECK_NEAR(ctx, result.fraction, 0.75f, 1e-7f);

    // the bound rejects it before the divide when it lies beyond max_t
    cast_utils::ray_intersects_triangle(result, down, 2.9f, vec3f::load(a), vec3f::load(b), vec3f::load(c), true);
    CHECK(ctx, !result.hit && result.fraction == 0.0f);
    cast_utils::ray_intersects_triangle(result, down, 3.0f, vec3f::load(a), vec3f::load(b), vec3f::load(c), true);
    CHECK(ctx, result.hit);

    // from below: a back face, culled or hit
    const cast_ray::Rayf up = {vec3f::make(0.5f, 0.5f, -1), vec3f::make(0, 0, 1), 2};
    cast_utils::ray_intersects_triangle(result, up, up.length, vec3f::load(a), vec3f::load(b), vec3f::load(c), true);
    CHECK(ctx, !result.hit);
    cast_utils::ray_intersects_triangle(result, up, up.length, vec3f::load(a), vec3f::load(b), vec3f::load(c), false);
    CHECK(ctx, result.hit && !result.front_facing);
    CHECK_NEAR(ctx, result.fraction, 0.5f, 1e-7f);

    // outside the triangle past the hypotenuse (b1 + b2 > 1), and outside each edge
    cast_utils::ray_intersects_triangle(result, {vec3f::make(1.5f, 1.5f, 3), vec3f::make(0, 0, -1), 4}, 4, vec3f::load(a), vec3f::load(b), vec3f::load(c), false);
    CHECK(ctx, !result.hit);
    cast_utils::ray_intersects_triangle(result, {vec3f::make(-0.1f, 0.5f, 3), vec3f::make(0, 0, -1), 4}, 4, vec3f::load(a), vec3f::load(b), vec3f::load(c), false);
    CHECK(ctx, !result.hit);
    cast_utils::ray_intersects_triangle(result, {vec3f::make(0.5f, -0.1f, 3), vec3f::make(0, 0, -1), 4}, 4, vec3f::load(a), vec3f::load(b), vec3f::load(c), false);
    CHECK(ctx, !result.hit);

    // the plane behind the origin, and a ray parallel to it
    cast_utils::ray_intersects_triangle(result, {vec3f::make(0.5f, 0.5f, -1), vec3f::make(0, 0, -1), 4}, 4, vec3f::load(a), vec3f::load(b), vec3f::load(c), false);
    CHECK(ctx, !result.hit);
    cast_utils::ray_intersects_triangle(result, {vec3f::make(-1, 0.5f, 0), vec3f::make(1, 0, 0), 4}, 4, vec3f::load(a), vec3f::load(b), vec3f::load(c), false);
    CHECK(ctx, !result.hit);

    // an oblique ray: from (0.5, 0.5, 2) towards (1, 0.25, 0); the hit lies on that point
    const Vec3f toward = vec3f::make(0.5f, -0.25f, -2);
    const float distance = vec3f::length(toward);
    const Vec3f direction = vec3f::scale(toward, 1.0f / distance);
    cast_utils::ray_intersects_triangle(result, {vec3f::make(0.5f, 0.5f, 2), direction, 10}, 10, vec3f::load(a), vec3f::load(b), vec3f::load(c), true);
    CHECK(ctx, result.hit && result.front_facing);
    CHECK_NEAR(ctx, result.fraction, distance / 10.0f, 1e-6f);
}
