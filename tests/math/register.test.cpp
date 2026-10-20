#include "tumble/math/aabox.h"
#include "math/storage/box3f.h"
#include "math/storage/mat4f.h"
#include "math/storage/plane3f.h"
#include "math/storage/quatf.h"
#include "math/storage/triangle3f.h"
#include "math/storage/vec3f.h"
#include "math/storage/vec4f.h"
#include "test.h"
#include "test_rng.h"

namespace {

constexpr int ROUNDS = 3000;

float bits_to_float(uint32_t bits) { return __builtin_bit_cast(float, bits); }
uint32_t float_to_bits(float f) { return __builtin_bit_cast(uint32_t, f); }

/** uniform in [-1, 1) */
float unit(uint32_t &rng) { return xorshift32_unit(rng) * 2.0f - 1.0f; }

/** an exact power of two in [2^-20, 2^20] */
float magnitude(uint32_t &rng) { return bits_to_float((uint32_t)(127 + (int32_t)(xorshift32(rng) % 41) - 20) << 23); }

/** a float from every class: specials, raw bit patterns, small integers and halves, and ordinary magnitudes */
float any(uint32_t &rng) {
    static constexpr float SPECIALS[] = {0.0f, -0.0f, 1.0f, -1.0f, 0.5f, 2.0f, __builtin_inff(), -__builtin_inff(),
                                         __builtin_nanf(""), 1e-40f, -1e-40f, 3.4e38f, -3.4e38f, 1.1754944e-38f, 1e-20f, 1e20f};
    switch (xorshift32(rng) % 6) {
        case 0:
            return SPECIALS[xorshift32(rng) % 16];
        case 1:
            return bits_to_float(xorshift32(rng));
        case 2:
            return (float)((int32_t)(xorshift32(rng) % 2001) - 1000) * 0.5f;
        default:
            return unit(rng) * magnitude(rng);
    }
}

bool same(float a, float b) { return float_to_bits(a) == float_to_bits(b) || (a != a && b != b); }

Float3 any3(uint32_t &rng) { return {any(rng), any(rng), any(rng)}; }
Float4 any4(uint32_t &rng) { return {any(rng), any(rng), any(rng), any(rng)}; }
Float3 nice3(uint32_t &rng) { return {unit(rng) * 10.0f, unit(rng) * 10.0f, unit(rng) * 10.0f}; }
Float3 unit3(uint32_t &rng) {
    Float3 v = nice3(rng);
    return vec3f::normalize(v, v);
}
Float4 unit_quat(uint32_t &rng) {
    Float4 q = {unit(rng), unit(rng), unit(rng), unit(rng)};
    return quatf::normalize(q, q);
}
Float4x4 any16(uint32_t &rng) {
    Float4x4 m;
    for (float &f : m.m) f = any(rng);
    return m;
}
Float4x4 nice16(uint32_t &rng) {
    Float4x4 m;
    for (float &f : m.m) f = unit(rng) * 4.0f;
    return m;
}
/** a rotation, translation and positive scale */
Float4x4 trs(uint32_t &rng) {
    Float4x4 m;
    const Float3 s = {1.0f + unit(rng) * 0.9f, 1.0f + unit(rng) * 0.9f, 1.0f + unit(rng) * 0.9f};
    return mat4f::from_rotation_translation_scale(m, unit_quat(rng), nice3(rng), s);
}

/** lanes 0..2 equal the storage value and lane 3 is a copy of z */
bool same3(Vec3f r, const Float3 &s) {
    return same(vec3f::x(r), s.x) && same(vec3f::y(r), s.y) && same(vec3f::z(r), s.z) &&
           same(v4f::lane<3>(r.v), v4f::lane<2>(r.v));
}
bool same4(v128_t r, const Float4 &s) {
    return same(v4f::lane<0>(r), s.x) && same(v4f::lane<1>(r), s.y) && same(v4f::lane<2>(r), s.z) && same(v4f::lane<3>(r), s.w);
}
bool same16(const Mat4f &r, const Float4x4 &s) {
    Float4x4 out;
    mat4f::store(out, r);
    for (int i = 0; i < 16; i++) {
        if (!same(out.m[i], s.m[i])) return false;
    }
    return true;
}

Vec3f L(const Float3 &a) { return vec3f::load(a); }
Vec4f L(const Float4 &a) { return {v4f::load4(&a.x)}; }
Quatf Q(const Float4 &a) { return quatf::load(a); }
Mat4f L(const Float4x4 &a) { return mat4f::load(a); }
Mat3f mat3(uint32_t &rng, bool nice) {
    Mat3f m;
    for (float &f : m.m) f = nice ? unit(rng) : any(rng);
    return m;
}

}  // namespace

void test_register_vec3f_matches_storage(TestCtx &ctx) {
    uint32_t rng = 0x9e3779b9u;
    uint32_t bad = 0;
    for (int round = 0; round < ROUNDS * 2; round++) {
        const bool nice = round >= ROUNDS;
        const Float3 a = nice ? nice3(rng) : any3(rng);
        const Float3 b = nice ? nice3(rng) : any3(rng);
        const float s = nice ? unit(rng) * 3.0f : any(rng);
        const Float4 q = nice ? unit_quat(rng) : any4(rng);
        const Float4x4 m = nice ? trs(rng) : any16(rng);
        Float3 o;

        bad += !same3(L(a), a);
        bad += !same3(vec3f::make(a.x, a.y, a.z), a);
        bad += !same3(vec3f::add(L(a), L(b)), vec3f::add(o, a, b));
        bad += !same3(vec3f::subtract(L(a), L(b)), vec3f::subtract(o, a, b));
        bad += !same3(vec3f::multiply(L(a), L(b)), vec3f::multiply(o, a, b));
        bad += !same3(vec3f::divide(L(a), L(b)), vec3f::divide(o, a, b));
        bad += !same3(vec3f::min(L(a), L(b)), vec3f::min(o, a, b));
        bad += !same3(vec3f::max(L(a), L(b)), vec3f::max(o, a, b));
        bad += !same3(vec3f::fmax(L(a), L(b)), vec3f::from_values(scalar::max(a.x, b.x), scalar::max(a.y, b.y), scalar::max(a.z, b.z)));
        bad += !same3(vec3f::negate(L(a)), vec3f::negate(o, a));
        bad += !same3(vec3f::abs(L(a)), vec3f::from_values(scalar::abs(a.x), scalar::abs(a.y), scalar::abs(a.z)));
        bad += !same3(vec3f::scale(L(a), s), vec3f::scale(o, a, s));
        bad += !same3(vec3f::scale_and_add(L(a), L(b), s), vec3f::scale_and_add(o, a, b, s));
        bad += !same3(vec3f::inverse(L(a)), vec3f::inverse(o, a));
        bad += !same(vec3f::dot(L(a), L(b)), vec3f::dot(a, b));
        bad += !same(vec3f::squared_length(L(a)), vec3f::squared_length(a));
        bad += !same(vec3f::reduce_sum(L(a)), a.x + a.y + a.z);
        bad += !same(vec3f::length(L(a)), vec3f::length(a));
        bad += !same(vec3f::squared_distance(L(a), L(b)), vec3f::squared_distance(a, b));
        bad += !same(vec3f::distance(L(a), L(b)), vec3f::distance(a, b));
        bad += !same3(vec3f::normalize(L(a)), vec3f::normalize(o, a));
        bad += !same3(vec3f::cross(L(a), L(b)), vec3f::cross(o, a, b));
        bad += !same3(vec3f::perpendicular(L(a)), vec3f::perpendicular(o, a));
        bad += !same3(vec3f::lerp(L(a), L(b), s), vec3f::lerp(o, a, b, s));
        bad += !same3(vec3f::transform_mat4(L(a), L(m)), vec3f::transform_mat4(o, a, m));
        bad += !same3(vec3f::transform_quat(L(a), Q(q)), vec3f::transform_quat(o, a, q));
        bad += vec3f::exact_equals(L(a), L(b)) != vec3f::exact_equals(a, b);
        bad += vec3f::exact_equals(L(a), L(a)) != vec3f::exact_equals(a, a);
        bad += vec3f::is_scale_inside_out(L(a)) != vec3f::is_scale_inside_out(a);
        vec3f::store(o, L(a));
        bad += !(same(o.x, a.x) && same(o.y, a.y) && same(o.z, a.z));
    }
    CHECK(ctx, bad == 0);
}

void test_register_vec4f_matches_storage(TestCtx &ctx) {
    uint32_t rng = 0x85ebca6bu;
    uint32_t bad = 0;
    for (int round = 0; round < ROUNDS * 2; round++) {
        const bool nice = round >= ROUNDS;
        const Float4 a = nice ? unit_quat(rng) : any4(rng);
        Float4 o;
        bad += !same4(vec4f::normalize(L(a)).v, vec4f::normalize(o, a));
    }
    CHECK(ctx, bad == 0);
}

void test_register_quatf_matches_storage(TestCtx &ctx) {
    uint32_t rng = 0xc2b2ae35u;
    uint32_t bad = 0;
    for (int round = 0; round < ROUNDS * 2; round++) {
        const bool nice = round >= ROUNDS;
        const Float4 a = nice ? unit_quat(rng) : any4(rng);
        const Float4 b = nice ? unit_quat(rng) : any4(rng);
        const Float3 u = nice ? unit3(rng) : any3(rng);
        const Float3 w = nice ? nice3(rng) : any3(rng);
        const float s = nice ? unit(rng) * 3.0f : any(rng);
        const Mat3f m3 = mat3(rng, nice);
        const Float4x4 m = nice ? trs(rng) : any16(rng);
        Float4 o;
        bad += !same4(quatf::scale(Q(a), s).v, quatf::scale(o, a, s));
        bad += !same(quatf::dot(Q(a), Q(b)), quatf::dot(a, b));
        bad += !same4(quatf::normalize(Q(a)).v, quatf::normalize(o, a));
        bad += !same4(quatf::set_axis_angle(L(u), s).v, quatf::set_axis_angle(o, u, s));
        bad += !same4(quatf::multiply(Q(a), Q(b)).v, quatf::multiply(o, a, b));
        bad += !same4(quatf::invert(Q(a)).v, quatf::invert(o, a));
        bad += !same4(quatf::conjugate(Q(a)).v, quatf::conjugate(o, a));
        bad += !same4(quatf::from_mat3(m3).v, quatf::from_mat3(o, m3));
        bad += !same4(quatf::from_mat4(L(m)).v, quatf::from_mat4(o, m));
        for (int order = 0; order < 6; order++) {
            const Eulerf e = {w.x, w.y, w.z, (EulerOrder)order};
            bad += !same4(quatf::from_euler(e).v, quatf::from_euler(o, e));
        }
        bad += !same3(quatf::xyz(Q(a)), {a.x, a.y, a.z});
    }
    CHECK(ctx, bad == 0);
}

void test_register_mat4f_matches_storage(TestCtx &ctx) {
    uint32_t rng = 0x27d4eb2fu;
    uint32_t bad = 0;
    for (int round = 0; round < ROUNDS * 2; round++) {
        const bool nice = round >= ROUNDS;
        const Float4x4 a = nice ? (round & 1 ? trs(rng) : nice16(rng)) : any16(rng);
        const Float4x4 b = nice ? (round & 1 ? trs(rng) : nice16(rng)) : any16(rng);
        const Float3 u = nice ? nice3(rng) : any3(rng);
        const Float3 v = nice ? nice3(rng) : any3(rng);
        const Float4 q = nice ? unit_quat(rng) : any4(rng);
        const float s = nice ? unit(rng) * 3.0f : any(rng);
        Float4x4 o;
        Float3 o3;

        bad += !same16(L(a), a);
        bad += !same16(mat4f::transpose(L(a)), mat4f::transpose(o, a));
        {
            Float4x4 so = {};
            Mat4f ro = mat4f::zero();
            const bool sok = mat4f::invert(so, a);
            const bool rok = mat4f::invert(ro, L(a));
            bad += sok != rok || !same16(ro, so);
        }
        {
            Float4x4 so = {};
            Mat4f ro = mat4f::zero();
            const bool sok = mat4f::invert3x3(so, a);
            const bool rok = mat4f::invert3x3(ro, L(a));
            bad += sok != rok || !same16(ro, so);
        }
        bad += !same16(mat4f::adjoint(L(a)), mat4f::adjoint(o, a));
        bad += !same(mat4f::determinant(L(a)), mat4f::determinant(a));
        bad += !same16(mat4f::multiply(L(a), L(b)), mat4f::multiply(o, a, b));
        bad += !same16(mat4f::multiply3x3(L(a), L(b)), mat4f::multiply3x3(o, a, b));
        bad += !same16(mat4f::multiply3x3_right_transposed(L(a), L(b)), mat4f::multiply3x3_right_transposed(o, a, b));
        bad += !same3(mat4f::multiply3x3_transposed_vec(L(a), L(u)), mat4f::multiply3x3_transposed_vec(o3, a, u));
        bad += !same3(mat4f::multiply3x3_vec(L(a), L(u)), mat4f::multiply3x3_vec(o3, a, u));
        bad += !same16(mat4f::cross_product_matrix(L(u)), mat4f::cross_product_matrix(o, u));
        bad += !same16(mat4f::translate(L(a), L(u)), mat4f::translate(o, a, u));
        bad += !same16(mat4f::scale(L(a), L(u)), mat4f::scale(o, a, u));
        bad += !same16(mat4f::from_scaling(L(u)), mat4f::from_scaling(o, u));
        bad += !same16(mat4f::from_rotation_translation(Q(q), L(u)), mat4f::from_rotation_translation(o, q, u));
        bad += !same3(mat4f::get_translation(L(a)), mat4f::get_translation(o3, a));
        bad += !same16(mat4f::from_rotation_translation_scale(Q(q), L(u), L(v)), mat4f::from_rotation_translation_scale(o, q, u, v));
        bad += !same16(mat4f::from_quat(Q(q)), mat4f::from_quat(o, q));
        bad += !same16(mat4f::add(L(a), L(b)), mat4f::add(o, a, b));
        bad += !same16(mat4f::subtract(L(a), L(b)), mat4f::subtract(o, a, b));
        bad += !same16(mat4f::multiply_scalar(L(a), s), mat4f::multiply_scalar(o, a, s));
        bad += !same3(mat4f::axis(L(a), 1), {a.m[4], a.m[5], a.m[6]});
    }
    CHECK(ctx, bad == 0);
}

void test_register_aabox_matches_storage(TestCtx &ctx) {
    uint32_t rng = 0x51ed2705u;
    uint32_t bad = 0;
    const auto same_box = [](const AABox &r, const Box3f &s) { return same3(r.min, s.min) && same3(r.max, s.max); };
    for (int round = 0; round < ROUNDS * 2; round++) {
        const bool nice = round >= ROUNDS;
        const Box3f a = {nice ? nice3(rng) : any3(rng), nice ? nice3(rng) : any3(rng)};
        const Box3f b = {nice ? nice3(rng) : any3(rng), nice ? nice3(rng) : any3(rng)};
        const Float3 p = nice ? nice3(rng) : any3(rng);
        const Float3 q = nice ? nice3(rng) : any3(rng);
        const Float3 r = nice ? nice3(rng) : any3(rng);
        const float margin = nice ? unit(rng) : any(rng);
        const Float4x4 m = nice ? trs(rng) : any16(rng);
        const AABox la = aabox::load(a), lb = aabox::load(b);
        Box3f o;
        Box3f stored;

        bad += !same_box(la, a);
        aabox::store(stored, la);
        bad += !same(stored.min.x, a.min.x) || !same(stored.min.y, a.min.y) || !same(stored.min.z, a.min.z);
        bad += !same(stored.max.x, a.max.x) || !same(stored.max.y, a.max.y) || !same(stored.max.z, a.max.z);
        bad += !same_box(aabox::empty(), box3f::create());
        bad += aabox::is_empty(la) != box3f::is_empty(a);
        bad += !same_box(aabox::from_triangle(L(p), L(q), L(r)), triangle3f::bounds(o, p, q, r));
        bad += !same_box(aabox::expand_by_point(la, L(p)), box3f::expand_by_point(o, a, p));
        bad += !same_box(aabox::expand_by_extents(la, L(p)), box3f::expand_by_extents(o, a, p));
        bad += !same_box(aabox::expand_by_margin(la, margin), box3f::expand_by_margin(o, a, margin));
        bad += !same_box(aabox::union_(la, lb), box3f::union_(o, a, b));
        bad += !same_box(aabox::scale(la, L(p)), box3f::scale(o, a, p));
        bad += !same_box(aabox::transform_mat4(la, L(m)), box3f::transform_mat4(o, a, m));
        bad += !same(aabox::surface_area(la), box3f::surface_area(a));
        bad += aabox::contains_point(la, L(p)) != box3f::contains_point(a, p);
        bad += aabox::contains(la, lb) != box3f::contains_box3(a, b);
        bad += aabox::intersects(la, lb) != box3f::intersects_box3(a, b);
        Float3 sum, half;
        vec3f::add(sum, a.min, a.max);
        bad += !same3(aabox::center(la), vec3f::scale(half, sum, 0.5f));
        Float3 difference;
        vec3f::subtract(difference, a.max, a.min);
        bad += !same3(aabox::size(la), difference);
        bad += !same3(aabox::extents(la), vec3f::scale(half, difference, 0.5f));
    }
    CHECK(ctx, bad == 0);
}

void test_register_plane_matches_storage(TestCtx &ctx) {
    uint32_t rng = 0x2f6b1a3du;
    uint32_t bad = 0;
    for (int round = 0; round < ROUNDS * 2; round++) {
        const bool nice = round >= ROUNDS;
        const Plane3f plane = {nice ? unit3(rng) : any3(rng), nice ? unit(rng) * 10.0f : any(rng)};
        const Float3 p = nice ? nice3(rng) : any3(rng);
        const Plane loaded = plane3f::load(plane);
        Float3 o;

        bad += !same3(loaded.normal, plane.normal) || !same(loaded.constant, plane.constant);
        bad += !same(plane3f::distance_to_point(loaded, L(p)), plane3f::distance_to_point(plane, p));
        bad += !same3(plane3f::project_point(loaded, L(p)), plane3f::project_point(o, plane, p));
    }
    CHECK(ctx, bad == 0);
}
