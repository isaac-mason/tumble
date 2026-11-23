#include "tumble/math/simd.h"
#include "test.h"

namespace simd_reference {

struct V128 {
    uint32_t lanes[4];
};

namespace v4f {

float get(V128 v, int i) { return __builtin_bit_cast(float, v.lanes[i]); }
uint32_t bits_of(float s) { return __builtin_bit_cast(uint32_t, s); }
uint32_t mask_of(bool set) { return set ? 0xffffffffu : 0u; }

V128 make(float x, float y, float z, float w) { return {{bits_of(x), bits_of(y), bits_of(z), bits_of(w)}}; }
V128 splat(float s) { return make(s, s, s, s); }
V128 zero() { return {{0, 0, 0, 0}}; }

float nan_propagating_min(float a, float b) {
    if (a != a || b != b) return a + b;
    if (a == b) return bits_of(a) >> 31 ? a : b;
    return a < b ? a : b;
}

float nan_propagating_max(float a, float b) {
    if (a != a || b != b) return a + b;
    if (a == b) return bits_of(a) >> 31 ? b : a;
    return a < b ? b : a;
}

V128 add(V128 a, V128 b) { return make(get(a, 0) + get(b, 0), get(a, 1) + get(b, 1), get(a, 2) + get(b, 2), get(a, 3) + get(b, 3)); }
V128 sub(V128 a, V128 b) { return make(get(a, 0) - get(b, 0), get(a, 1) - get(b, 1), get(a, 2) - get(b, 2), get(a, 3) - get(b, 3)); }
V128 mul(V128 a, V128 b) { return make(get(a, 0) * get(b, 0), get(a, 1) * get(b, 1), get(a, 2) * get(b, 2), get(a, 3) * get(b, 3)); }
V128 div(V128 a, V128 b) { return make(get(a, 0) / get(b, 0), get(a, 1) / get(b, 1), get(a, 2) / get(b, 2), get(a, 3) / get(b, 3)); }
V128 mul_add(V128 a, V128 b, V128 c) { return add(mul(a, b), c); }

V128 min(V128 a, V128 b) {
    V128 r;
    for (int i = 0; i < 4; i++) r.lanes[i] = get(b, i) < get(a, i) ? b.lanes[i] : a.lanes[i];
    return r;
}

V128 max(V128 a, V128 b) {
    V128 r;
    for (int i = 0; i < 4; i++) r.lanes[i] = get(a, i) < get(b, i) ? b.lanes[i] : a.lanes[i];
    return r;
}

V128 fmin(V128 a, V128 b) {
    return make(nan_propagating_min(get(a, 0), get(b, 0)), nan_propagating_min(get(a, 1), get(b, 1)), nan_propagating_min(get(a, 2), get(b, 2)),
                nan_propagating_min(get(a, 3), get(b, 3)));
}

V128 fmax(V128 a, V128 b) {
    return make(nan_propagating_max(get(a, 0), get(b, 0)), nan_propagating_max(get(a, 1), get(b, 1)), nan_propagating_max(get(a, 2), get(b, 2)),
                nan_propagating_max(get(a, 3), get(b, 3)));
}

V128 neg(V128 a) { return {{a.lanes[0] ^ 0x80000000u, a.lanes[1] ^ 0x80000000u, a.lanes[2] ^ 0x80000000u, a.lanes[3] ^ 0x80000000u}}; }
V128 neg_xyz(V128 a) { return {{a.lanes[0] ^ 0x80000000u, a.lanes[1] ^ 0x80000000u, a.lanes[2] ^ 0x80000000u, a.lanes[3]}}; }
V128 abs(V128 a) { return {{a.lanes[0] & 0x7fffffffu, a.lanes[1] & 0x7fffffffu, a.lanes[2] & 0x7fffffffu, a.lanes[3] & 0x7fffffffu}}; }
V128 sqrt(V128 a) { return make(__builtin_sqrtf(get(a, 0)), __builtin_sqrtf(get(a, 1)), __builtin_sqrtf(get(a, 2)), __builtin_sqrtf(get(a, 3))); }

template <int I>
float lane(V128 v) {
    return get(v, I);
}

template <int I>
V128 set_lane(V128 v, float s) {
    v.lanes[I] = bits_of(s);
    return v;
}

template <int X, int Y, int Z, int W>
V128 shuffle(V128 a) {
    return {{a.lanes[X], a.lanes[Y], a.lanes[Z], a.lanes[W]}};
}

template <int X, int Y, int Z, int W>
V128 shuffle2(V128 a, V128 b) {
    const uint32_t both[8] = {a.lanes[0], a.lanes[1], a.lanes[2], a.lanes[3], b.lanes[0], b.lanes[1], b.lanes[2], b.lanes[3]};
    return {{both[X], both[Y], both[Z], both[W]}};
}

V128 lt(V128 a, V128 b) { return {{mask_of(get(a, 0) < get(b, 0)), mask_of(get(a, 1) < get(b, 1)), mask_of(get(a, 2) < get(b, 2)), mask_of(get(a, 3) < get(b, 3))}}; }
V128 le(V128 a, V128 b) { return {{mask_of(get(a, 0) <= get(b, 0)), mask_of(get(a, 1) <= get(b, 1)), mask_of(get(a, 2) <= get(b, 2)), mask_of(get(a, 3) <= get(b, 3))}}; }
V128 gt(V128 a, V128 b) { return {{mask_of(get(a, 0) > get(b, 0)), mask_of(get(a, 1) > get(b, 1)), mask_of(get(a, 2) > get(b, 2)), mask_of(get(a, 3) > get(b, 3))}}; }
V128 ge(V128 a, V128 b) { return {{mask_of(get(a, 0) >= get(b, 0)), mask_of(get(a, 1) >= get(b, 1)), mask_of(get(a, 2) >= get(b, 2)), mask_of(get(a, 3) >= get(b, 3))}}; }
V128 eq(V128 a, V128 b) { return {{mask_of(get(a, 0) == get(b, 0)), mask_of(get(a, 1) == get(b, 1)), mask_of(get(a, 2) == get(b, 2)), mask_of(get(a, 3) == get(b, 3))}}; }

V128 bit_and(V128 a, V128 b) { return {{a.lanes[0] & b.lanes[0], a.lanes[1] & b.lanes[1], a.lanes[2] & b.lanes[2], a.lanes[3] & b.lanes[3]}}; }
V128 bit_or(V128 a, V128 b) { return {{a.lanes[0] | b.lanes[0], a.lanes[1] | b.lanes[1], a.lanes[2] | b.lanes[2], a.lanes[3] | b.lanes[3]}}; }
V128 bit_xor(V128 a, V128 b) { return {{a.lanes[0] ^ b.lanes[0], a.lanes[1] ^ b.lanes[1], a.lanes[2] ^ b.lanes[2], a.lanes[3] ^ b.lanes[3]}}; }

V128 select(V128 a, V128 b, V128 mask) {
    V128 r;
    for (int i = 0; i < 4; i++) r.lanes[i] = (a.lanes[i] & mask.lanes[i]) | (b.lanes[i] & ~mask.lanes[i]);
    return r;
}

bool all_true(V128 mask) { return mask.lanes[0] != 0 && mask.lanes[1] != 0 && mask.lanes[2] != 0 && mask.lanes[3] != 0; }
bool any_true(V128 mask) { return (mask.lanes[0] | mask.lanes[1] | mask.lanes[2] | mask.lanes[3]) != 0; }

uint32_t bitmask(V128 mask) {
    return (mask.lanes[0] >> 31) | (mask.lanes[1] >> 31) << 1 | (mask.lanes[2] >> 31) << 2 | (mask.lanes[3] >> 31) << 3;
}

float hsum3(V128 v) { return get(v, 0) + get(v, 1) + get(v, 2); }
float hsum4(V128 v) { return (get(v, 0) + get(v, 2)) + (get(v, 1) + get(v, 3)); }
float dot3(V128 a, V128 b) { return hsum3(mul(a, b)); }
float dot4(V128 a, V128 b) { return hsum4(mul(a, b)); }

void load3x4(const float *p, V128 &x, V128 &y, V128 &z) {
    x = make(p[0], p[3], p[6], p[9]);
    y = make(p[1], p[4], p[7], p[10]);
    z = make(p[2], p[5], p[8], p[11]);
}

void transpose3x4(V128 r0, V128 r1, V128 r2, V128 r3, V128 &x, V128 &y, V128 &z) {
    x = {{r0.lanes[0], r1.lanes[0], r2.lanes[0], r3.lanes[0]}};
    y = {{r0.lanes[1], r1.lanes[1], r2.lanes[1], r3.lanes[1]}};
    z = {{r0.lanes[2], r1.lanes[2], r2.lanes[2], r3.lanes[2]}};
}

}  // namespace v4f

namespace v4i {

V128 make(int32_t x, int32_t y, int32_t z, int32_t w) { return {{(uint32_t)x, (uint32_t)y, (uint32_t)z, (uint32_t)w}}; }
V128 splat(int32_t s) { return make(s, s, s, s); }
V128 add(V128 a, V128 b) { return {{a.lanes[0] + b.lanes[0], a.lanes[1] + b.lanes[1], a.lanes[2] + b.lanes[2], a.lanes[3] + b.lanes[3]}}; }
template <int I>
int32_t lane(V128 v) {
    return (int32_t)v.lanes[I];
}

}  // namespace v4i

}  // namespace simd_reference

namespace {

constexpr uint32_t ITERATIONS = 20000;

constexpr uint32_t SPECIAL_BITS[] = {
    0x00000000u, 0x80000000u, 0x3f800000u, 0xbf800000u, 0x7f800000u, 0xff800000u, 0x7fc00000u, 0xffc00000u, 0x7fa00000u,
    0xffa00000u, 0x7fc12345u, 0x00000001u, 0x80000001u, 0x007fffffu, 0x807fffffu, 0x00800000u, 0x7f7fffffu, 0xff7fffffu,
    0x3f000000u, 0x40000000u, 0x40490fdbu, 0xc0490fdbu, 0x358637bdu, 0x4b800000u,
};
constexpr uint32_t SPECIAL_COUNT = sizeof(SPECIAL_BITS) / sizeof(SPECIAL_BITS[0]);

constexpr uint32_t MASK_BITS[] = {0x00000000u, 0xffffffffu, 0x80000000u, 0x00000001u, 0x00010000u, 0x7fffffffu, 0x3f800000u, 0xff000000u};
constexpr uint32_t MASK_COUNT = sizeof(MASK_BITS) / sizeof(MASK_BITS[0]);

struct Rng {
    uint32_t state;
    uint32_t next() {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        return state;
    }
};

struct Lanes {
    uint32_t bits[4];
};

float float_of(uint32_t bits) { return __builtin_bit_cast(float, bits); }
uint32_t bits_of(float value) { return __builtin_bit_cast(uint32_t, value); }

uint32_t random_float_bits(Rng &rng) {
    switch (rng.next() % 4) {
        case 0:
        case 1: return SPECIAL_BITS[rng.next() % SPECIAL_COUNT];
        case 2: return rng.next();
        default: return bits_of(((float)(int)(rng.next() % 2001) - 1000.0f) * 0.37f);
    }
}

Lanes random_floats(Rng &rng) { return {{random_float_bits(rng), random_float_bits(rng), random_float_bits(rng), random_float_bits(rng)}}; }

Lanes random_raw(Rng &rng) { return {{rng.next(), rng.next(), rng.next(), rng.next()}}; }

Lanes random_masks(Rng &rng) {
    Lanes lanes;
    for (int i = 0; i < 4; i++) lanes.bits[i] = rng.next() % 4 == 0 ? rng.next() : MASK_BITS[rng.next() % MASK_COUNT];
    return lanes;
}

V128 active_of(const Lanes &lanes) {
    const float values[4] = {float_of(lanes.bits[0]), float_of(lanes.bits[1]), float_of(lanes.bits[2]), float_of(lanes.bits[3])};
    return v4f::load4(values);
}

simd_reference::V128 reference_of(const Lanes &lanes) { return {{lanes.bits[0], lanes.bits[1], lanes.bits[2], lanes.bits[3]}}; }

Lanes bits_of_active(V128 v) {
    float values[4];
    v4f::store4(values, v);
    return {{bits_of(values[0]), bits_of(values[1]), bits_of(values[2]), bits_of(values[3])}};
}

bool is_nan_bits(uint32_t bits) { return (bits & 0x7f800000u) == 0x7f800000u && (bits & 0x007fffffu) != 0; }

bool same_lane(uint32_t a, uint32_t b, bool nan_alike) { return a == b || (nan_alike && is_nan_bits(a) && is_nan_bits(b)); }

bool matches(V128 active, simd_reference::V128 reference, bool nan_alike) {
    const Lanes a = bits_of_active(active);
    for (int i = 0; i < 4; i++)
        if (!same_lane(a.bits[i], reference.lanes[i], nan_alike)) return false;
    return true;
}

bool matches(float active, float reference, bool nan_alike) { return same_lane(bits_of(active), bits_of(reference), nan_alike); }

#define CHECK_UNARY(ctx, op, nan_alike)                                                                  \
    do {                                                                                                 \
        Rng rng{0x9e3779b9u};                                                                            \
        uint32_t mismatches = 0;                                                                         \
        for (uint32_t i = 0; i < ITERATIONS; i++) {                                                      \
            const Lanes a = random_floats(rng);                                                          \
            if (!matches(v4f::op(active_of(a)), simd_reference::v4f::op(reference_of(a)), nan_alike)) mismatches++; \
        }                                                                                                \
        test_check(ctx, mismatches == 0, __FILE__, __LINE__, #op);                                       \
    } while (0)

#define CHECK_BINARY(ctx, op, nan_alike, generator)                                                      \
    do {                                                                                                 \
        Rng rng{0x2545f491u};                                                                            \
        uint32_t mismatches = 0;                                                                         \
        for (uint32_t i = 0; i < ITERATIONS; i++) {                                                      \
            const Lanes a = generator(rng);                                                              \
            const Lanes b = generator(rng);                                                              \
            if (!matches(v4f::op(active_of(a), active_of(b)), simd_reference::v4f::op(reference_of(a), reference_of(b)), nan_alike)) \
                mismatches++;                                                                            \
        }                                                                                                \
        test_check(ctx, mismatches == 0, __FILE__, __LINE__, #op);                                       \
    } while (0)

#define CHECK_LANE(ctx, index)                                                                           \
    do {                                                                                                 \
        Rng rng{0x1b873593u};                                                                            \
        uint32_t mismatches = 0;                                                                         \
        for (uint32_t i = 0; i < ITERATIONS; i++) {                                                      \
            const Lanes a = random_floats(rng);                                                          \
            const float s = float_of(random_float_bits(rng));                                            \
            if (!matches(v4f::lane<index>(active_of(a)), simd_reference::v4f::lane<index>(reference_of(a)), false)) mismatches++; \
            if (!matches(v4f::set_lane<index>(active_of(a), s), simd_reference::v4f::set_lane<index>(reference_of(a), s), false)) \
                mismatches++;                                                                            \
        }                                                                                                \
        test_check(ctx, mismatches == 0, __FILE__, __LINE__, "lane " #index);                            \
    } while (0)

template <int Code>
__attribute__((noinline)) void check_shuffle(const Lanes &a, uint32_t &mismatches) {
    constexpr int X = Code & 3, Y = (Code >> 2) & 3, Z = (Code >> 4) & 3, W = (Code >> 6) & 3;
    if (!matches(v4f::shuffle<X, Y, Z, W>(active_of(a)), simd_reference::v4f::shuffle<X, Y, Z, W>(reference_of(a)), false)) mismatches++;
}

template <int Code>
__attribute__((noinline)) void check_shuffle2(const Lanes &a, const Lanes &b, uint32_t &mismatches) {
    constexpr int X = Code & 7, Y = (Code >> 3) & 7, Z = (Code >> 6) & 7, W = (Code >> 9) & 7;
    if (!matches(v4f::shuffle2<X, Y, Z, W>(active_of(a), active_of(b)), simd_reference::v4f::shuffle2<X, Y, Z, W>(reference_of(a), reference_of(b)), false))
        mismatches++;
}

template <int Lo, int Hi>
void check_shuffle_range(const Lanes &a, uint32_t &mismatches) {
    if constexpr (Hi - Lo == 1) {
        check_shuffle<Lo>(a, mismatches);
    } else {
        check_shuffle_range<Lo, (Lo + Hi) / 2>(a, mismatches);
        check_shuffle_range<(Lo + Hi) / 2, Hi>(a, mismatches);
    }
}

template <int Lo, int Hi>
void check_shuffle2_range(const Lanes &a, const Lanes &b, uint32_t &mismatches) {
    if constexpr (Hi - Lo == 1) {
        check_shuffle2<Lo>(a, b, mismatches);
    } else {
        check_shuffle2_range<Lo, (Lo + Hi) / 2>(a, b, mismatches);
        check_shuffle2_range<(Lo + Hi) / 2, Hi>(a, b, mismatches);
    }
}

}  // namespace

void test_simd_backend_arithmetic_matches_scalar(TestCtx &ctx) {
    CHECK_BINARY(ctx, add, true, random_floats);
    CHECK_BINARY(ctx, sub, true, random_floats);
    CHECK_BINARY(ctx, mul, true, random_floats);
    CHECK_BINARY(ctx, div, true, random_floats);
    CHECK_BINARY(ctx, min, false, random_floats);
    CHECK_BINARY(ctx, max, false, random_floats);
    CHECK_BINARY(ctx, fmin, true, random_floats);
    CHECK_BINARY(ctx, fmax, true, random_floats);
    CHECK_BINARY(ctx, min, false, random_raw);
    CHECK_BINARY(ctx, max, false, random_raw);
    CHECK_UNARY(ctx, sqrt, true);
    CHECK_UNARY(ctx, neg, false);
    CHECK_UNARY(ctx, abs, false);
    CHECK_UNARY(ctx, neg_xyz, false);

    Rng rng{0x68e31da4u};
    uint32_t mismatches = 0;
    for (uint32_t i = 0; i < ITERATIONS; i++) {
        const Lanes a = random_floats(rng), b = random_floats(rng), c = random_floats(rng);
        if (!matches(v4f::mul_add(active_of(a), active_of(b), active_of(c)),
                     simd_reference::v4f::mul_add(reference_of(a), reference_of(b), reference_of(c)), true))
            mismatches++;
    }
    test_check(ctx, mismatches == 0, __FILE__, __LINE__, "mul_add");
}

void test_simd_backend_comparisons_and_masks_match_scalar(TestCtx &ctx) {
    CHECK_BINARY(ctx, lt, false, random_floats);
    CHECK_BINARY(ctx, le, false, random_floats);
    CHECK_BINARY(ctx, gt, false, random_floats);
    CHECK_BINARY(ctx, ge, false, random_floats);
    CHECK_BINARY(ctx, eq, false, random_floats);
    CHECK_BINARY(ctx, bit_and, false, random_raw);
    CHECK_BINARY(ctx, bit_or, false, random_raw);
    CHECK_BINARY(ctx, bit_xor, false, random_raw);

    Rng rng{0x85ebca6bu};
    uint32_t select_mismatches = 0, all_true_mismatches = 0, any_true_mismatches = 0, bitmask_mismatches = 0;
    for (uint32_t i = 0; i < ITERATIONS * 4; i++) {
        const Lanes a = random_raw(rng), b = random_raw(rng), mask = random_masks(rng);
        if (!matches(v4f::select(active_of(a), active_of(b), active_of(mask)),
                     simd_reference::v4f::select(reference_of(a), reference_of(b), reference_of(mask)), false))
            select_mismatches++;
        if (v4f::all_true(active_of(mask)) != simd_reference::v4f::all_true(reference_of(mask))) all_true_mismatches++;
        if (v4f::any_true(active_of(mask)) != simd_reference::v4f::any_true(reference_of(mask))) any_true_mismatches++;
        if (v4f::bitmask(active_of(mask)) != simd_reference::v4f::bitmask(reference_of(mask))) bitmask_mismatches++;
    }
    test_check(ctx, select_mismatches == 0, __FILE__, __LINE__, "select");
    test_check(ctx, all_true_mismatches == 0, __FILE__, __LINE__, "all_true");
    test_check(ctx, any_true_mismatches == 0, __FILE__, __LINE__, "any_true");
    test_check(ctx, bitmask_mismatches == 0, __FILE__, __LINE__, "bitmask");
}

void test_simd_backend_shuffles_match_scalar(TestCtx &ctx) {
    Rng rng{0xc2b2ae35u};
    uint32_t shuffle_mismatches = 0, shuffle2_mismatches = 0;
    for (uint32_t i = 0; i < 4; i++) {
        const Lanes a = random_raw(rng), b = random_raw(rng);
        check_shuffle_range<0, 256>(a, shuffle_mismatches);
        check_shuffle2_range<0, 4096>(a, b, shuffle2_mismatches);
    }
    test_check(ctx, shuffle_mismatches == 0, __FILE__, __LINE__, "shuffle");
    test_check(ctx, shuffle2_mismatches == 0, __FILE__, __LINE__, "shuffle2");
}

void test_simd_backend_lanes_and_constructors_match_scalar(TestCtx &ctx) {
    CHECK_LANE(ctx, 0);
    CHECK_LANE(ctx, 1);
    CHECK_LANE(ctx, 2);
    CHECK_LANE(ctx, 3);

    Rng rng{0x27d4eb2fu};
    uint32_t make_mismatches = 0, splat_mismatches = 0, integer_mismatches = 0;
    for (uint32_t i = 0; i < ITERATIONS; i++) {
        const Lanes a = random_floats(rng), b = random_floats(rng);
        if (!matches(v4f::make(float_of(a.bits[0]), float_of(a.bits[1]), float_of(a.bits[2]), float_of(a.bits[3])), reference_of(a), false)) make_mismatches++;
        if (!matches(v4f::splat(float_of(a.bits[0])), simd_reference::v4f::splat(float_of(a.bits[0])), false)) splat_mismatches++;
        const int32_t x = (int32_t)b.bits[0], y = (int32_t)b.bits[1], z = (int32_t)b.bits[2], w = (int32_t)b.bits[3];
        const V128 sum = v4i::add(v4i::make(x, y, z, w), v4i::splat(x));
        const simd_reference::V128 expected = simd_reference::v4i::add(simd_reference::v4i::make(x, y, z, w), simd_reference::v4i::splat(x));
        if (!matches(sum, expected, false)) integer_mismatches++;
        if (v4i::lane<0>(sum) != simd_reference::v4i::lane<0>(expected) || v4i::lane<1>(sum) != simd_reference::v4i::lane<1>(expected) ||
            v4i::lane<2>(sum) != simd_reference::v4i::lane<2>(expected) || v4i::lane<3>(sum) != simd_reference::v4i::lane<3>(expected))
            integer_mismatches++;
    }
    CHECK(ctx, matches(v4f::zero(), simd_reference::v4f::zero(), false));
    test_check(ctx, make_mismatches == 0, __FILE__, __LINE__, "make");
    test_check(ctx, splat_mismatches == 0, __FILE__, __LINE__, "splat");
    test_check(ctx, integer_mismatches == 0, __FILE__, __LINE__, "v4i");
}

void test_simd_backend_loads_and_stores_match_scalar(TestCtx &ctx) {
    constexpr uint32_t GUARD = 0xdeadbeefu;
    Rng rng{0x165667b1u};
    uint32_t load_mismatches = 0, store_mismatches = 0, guard_mismatches = 0;
    for (uint32_t i = 0; i < ITERATIONS; i++) {
        alignas(16) float source[17];
        for (int k = 0; k < 17; k++) source[k] = float_of(random_float_bits(rng));
        const float *p = source + 1;
        const Lanes four = {{bits_of(p[0]), bits_of(p[1]), bits_of(p[2]), bits_of(p[3])}};
        const Lanes three_zero = {{four.bits[0], four.bits[1], four.bits[2], 0}};
        const Lanes three_repeat = {{four.bits[0], four.bits[1], four.bits[2], four.bits[2]}};
        if (!matches(v4f::load4(p), reference_of(four), false)) load_mismatches++;
        if (!matches(v4f::load3(p), reference_of(three_zero), false)) load_mismatches++;
        if (!matches(v4f::load3_xyzz(p), reference_of(three_repeat), false)) load_mismatches++;

        const Lanes value = random_raw(rng);
        alignas(16) float destination[8];
        for (int k = 0; k < 8; k++) destination[k] = float_of(GUARD);
        v4f::store3(destination + 1, active_of(value));
        for (int k = 0; k < 8; k++) {
            const uint32_t expected = k >= 1 && k <= 3 ? value.bits[k - 1] : GUARD;
            if (bits_of(destination[k]) != expected) store_mismatches++;
        }
        for (int k = 0; k < 8; k++) destination[k] = float_of(GUARD);
        v4f::store4(destination + 1, active_of(value));
        for (int k = 0; k < 8; k++) {
            const uint32_t expected = k >= 1 && k <= 4 ? value.bits[k - 1] : GUARD;
            if (bits_of(destination[k]) != expected) guard_mismatches++;
        }
    }
    test_check(ctx, load_mismatches == 0, __FILE__, __LINE__, "loads");
    test_check(ctx, store_mismatches == 0, __FILE__, __LINE__, "store3");
    test_check(ctx, guard_mismatches == 0, __FILE__, __LINE__, "store4");
}

void test_simd_backend_horizontal_helpers_match_scalar(TestCtx &ctx) {
    Rng rng{0xd3a2646cu};
    uint32_t hsum_mismatches = 0, dot_mismatches = 0, load3x4_mismatches = 0, transpose_mismatches = 0;
    for (uint32_t i = 0; i < ITERATIONS; i++) {
        const Lanes a = random_floats(rng), b = random_floats(rng), c = random_floats(rng), d = random_floats(rng);
        if (!matches(v4f::hsum3(active_of(a)), simd_reference::v4f::hsum3(reference_of(a)), true)) hsum_mismatches++;
        if (!matches(v4f::hsum4(active_of(a)), simd_reference::v4f::hsum4(reference_of(a)), true)) hsum_mismatches++;
        if (!matches(v4f::dot3(active_of(a), active_of(b)), simd_reference::v4f::dot3(reference_of(a), reference_of(b)), true)) dot_mismatches++;
        if (!matches(v4f::dot4(active_of(a), active_of(b)), simd_reference::v4f::dot4(reference_of(a), reference_of(b)), true)) dot_mismatches++;

        float packed[12];
        for (int k = 0; k < 4; k++) {
            packed[k] = float_of(a.bits[k]);
            packed[4 + k] = float_of(b.bits[k]);
            packed[8 + k] = float_of(c.bits[k]);
        }
        V128 x, y, z;
        v4f::load3x4(packed, x, y, z);
        simd_reference::V128 reference_x, reference_y, reference_z;
        simd_reference::v4f::load3x4(packed, reference_x, reference_y, reference_z);
        if (!matches(x, reference_x, false) || !matches(y, reference_y, false) || !matches(z, reference_z, false)) load3x4_mismatches++;

        v4f::transpose3x4(active_of(a), active_of(b), active_of(c), active_of(d), x, y, z);
        simd_reference::v4f::transpose3x4(reference_of(a), reference_of(b), reference_of(c), reference_of(d), reference_x, reference_y, reference_z);
        if (!matches(x, reference_x, false) || !matches(y, reference_y, false) || !matches(z, reference_z, false)) transpose_mismatches++;
    }
    test_check(ctx, hsum_mismatches == 0, __FILE__, __LINE__, "hsum");
    test_check(ctx, dot_mismatches == 0, __FILE__, __LINE__, "dot");
    test_check(ctx, load3x4_mismatches == 0, __FILE__, __LINE__, "load3x4");
    test_check(ctx, transpose_mismatches == 0, __FILE__, __LINE__, "transpose3x4");
}
