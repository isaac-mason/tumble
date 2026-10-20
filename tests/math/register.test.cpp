// the register functions against the storage functions they replace, bit for bit, over random inputs drawn from
// every magnitude and the special values (signed zeros, infinities, nan, denormals), and over well-behaved ones
// (unit vectors and quaternions, rotation matrices) where the specials alone would leave a branch unexercised.

#include "math/aabox.h"
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

/** uniform in [-1, 1). */
float unit(uint32_t &rng) { return xorshift32_unit(rng) * 2.0f - 1.0f; }

/** a float from every class: specials, raw bit patterns, small integers and halves, and ordinary magnitudes. */
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
            return unit(rng) * scalar::pow(10.0f, unit(rng) * 6.0f);
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
/** a rotation, translation and positive scale. */
Float4x4 trs(uint32_t &rng) {
    Float4x4 m;
    const Float3 s = {1.0f + unit(rng) * 0.9f, 1.0f + unit(rng) * 0.9f, 1.0f + unit(rng) * 0.9f};
    return mat4f::from_rotation_translation_scale(m, unit_quat(rng), nice3(rng), s);
}

/** lanes 0..2 equal the storage value and lane 3 is a copy of z. */
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
Vec4f L(const Float4 &a) { return vec4f::load(a); }
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
        const Float3 c = nice ? nice3(rng) : any3(rng);
        const Float3 d = nice ? nice3(rng) : any3(rng);
        const Float3 ua = nice ? unit3(rng) : a;
        const Float3 ub = nice ? unit3(rng) : b;
        const float s = nice ? unit(rng) * 3.0f : any(rng);
        const Float4 q = nice ? unit_quat(rng) : any4(rng);
        const Float4x4 m = nice ? trs(rng) : any16(rng);
        const Mat3f m3 = mat3(rng, nice);
        Float3 o;

        bad += !same3(L(a), a);
        bad += !same3(vec3f::make(a.x, a.y, a.z), a);
        bad += !same3(vec3f::add(L(a), L(b)), vec3f::add(o, a, b));
        bad += !same3(vec3f::add_scalar(L(a), s), vec3f::add_scalar(o, a, s));
        bad += !same3(vec3f::subtract(L(a), L(b)), vec3f::subtract(o, a, b));
        bad += !same3(vec3f::subtract_scalar(L(a), s), vec3f::subtract_scalar(o, a, s));
        bad += !same3(vec3f::multiply(L(a), L(b)), vec3f::multiply(o, a, b));
        bad += !same3(vec3f::divide(L(a), L(b)), vec3f::divide(o, a, b));
        bad += !same3(vec3f::min(L(a), L(b)), vec3f::min(o, a, b));
        bad += !same3(vec3f::max(L(a), L(b)), vec3f::max(o, a, b));
        bad += !same3(vec3f::negate(L(a)), vec3f::negate(o, a));
        bad += !same3(vec3f::abs(L(a)), vec3f::from_values(scalar::abs(a.x), scalar::abs(a.y), scalar::abs(a.z)));
        bad += !same3(vec3f::scale(L(a), s), vec3f::scale(o, a, s));
        bad += !same3(vec3f::scale_and_add(L(a), L(b), s), vec3f::scale_and_add(o, a, b, s));
        bad += !same3(vec3f::ceil(L(a)), vec3f::ceil(o, a));
        bad += !same3(vec3f::floor(L(a)), vec3f::floor(o, a));
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
        bad += !same3(vec3f::lagrange(L(a), L(b), L(c), s), vec3f::lagrange(o, a, b, c, s));
        bad += !same3(vec3f::slerp(L(ua), L(ub), s), vec3f::slerp(o, ua, ub, s));
        bad += !same3(vec3f::hermite(L(a), L(b), L(c), L(d), s), vec3f::hermite(o, a, b, c, d, s));
        bad += !same3(vec3f::bezier(L(a), L(b), L(c), L(d), s), vec3f::bezier(o, a, b, c, d, s));
        bad += !same3(vec3f::transform_mat4(L(a), L(m)), vec3f::transform_mat4(o, a, m));
        bad += !same3(vec3f::transform_mat3(L(a), m3), vec3f::transform_mat3(o, a, m3));
        bad += !same3(vec3f::transform_quat(L(a), Q(q)), vec3f::transform_quat(o, a, q));
        bad += !same3(vec3f::rotate_x(L(a), L(b), s), vec3f::rotate_x(o, a, b, s));
        bad += !same3(vec3f::rotate_y(L(a), L(b), s), vec3f::rotate_y(o, a, b, s));
        bad += !same3(vec3f::rotate_z(L(a), L(b), s), vec3f::rotate_z(o, a, b, s));
        bad += !same(vec3f::angle(L(a), L(b)), vec3f::angle(a, b));
        bad += !same(vec3f::signed_angle(L(a), L(b), L(ua)), vec3f::signed_angle(a, b, ua));
        bad += !same3(vec3f::rotate_towards(L(ua), L(ub), s), vec3f::rotate_towards(o, ua, ub, s));
        bad += vec3f::exact_equals(L(a), L(b)) != vec3f::exact_equals(a, b);
        bad += vec3f::exact_equals(L(a), L(a)) != vec3f::exact_equals(a, a);
        bad += vec3f::equals(L(a), L(b)) != vec3f::equals(a, b);
        bad += vec3f::finite(L(a)) != vec3f::finite(a);
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
        const Float4 b = nice ? unit_quat(rng) : any4(rng);
        const float s = nice ? unit(rng) * 3.0f : any(rng);
        Float4 o;
        bad += !same4(L(a).v, a);
        bad += !same4(vec4f::add(L(a), L(b)).v, vec4f::add(o, a, b));
        bad += !same4(vec4f::subtract(L(a), L(b)).v, vec4f::subtract(o, a, b));
        bad += !same4(vec4f::multiply(L(a), L(b)).v, vec4f::multiply(o, a, b));
        bad += !same4(vec4f::divide(L(a), L(b)).v, vec4f::divide(o, a, b));
        bad += !same4(vec4f::min(L(a), L(b)).v, vec4f::min(o, a, b));
        bad += !same4(vec4f::max(L(a), L(b)).v, vec4f::max(o, a, b));
        bad += !same4(vec4f::negate(L(a)).v, vec4f::negate(o, a));
        bad += !same4(vec4f::scale(L(a), s).v, vec4f::scale(o, a, s));
        bad += !same4(vec4f::scale_and_add(L(a), L(b), s).v, vec4f::scale_and_add(o, a, b, s));
        bad += !same(vec4f::dot(L(a), L(b)), vec4f::dot(a, b));
        bad += !same(vec4f::squared_length(L(a)), vec4f::squared_length(a));
        bad += !same(vec4f::length(L(a)), vec4f::length(a));
        bad += !same(vec4f::squared_distance(L(a), L(b)), vec4f::squared_distance(a, b));
        bad += !same(vec4f::distance(L(a), L(b)), vec4f::distance(a, b));
        bad += !same4(vec4f::normalize(L(a)).v, vec4f::normalize(o, a));
        bad += !same4(vec4f::lerp(L(a), L(b), s).v, vec4f::lerp(o, a, b, s));
        bad += vec4f::exact_equals(L(a), L(b)) != vec4f::exact_equals(a, b);
        bad += vec4f::exact_equals(L(a), L(a)) != vec4f::exact_equals(a, a);
        bad += vec4f::equals(L(a), L(b)) != vec4f::equals(a, b);
        vec4f::store(o, L(a));
        bad += !same4(L(o).v, a);
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
        const Float4 c = nice ? unit_quat(rng) : any4(rng);
        const Float4 d = nice ? unit_quat(rng) : any4(rng);
        const Float3 u = nice ? unit3(rng) : any3(rng);
        const Float3 v = nice ? unit3(rng) : any3(rng);
        const Float3 w = nice ? nice3(rng) : any3(rng);
        const float s = nice ? unit(rng) * 3.0f : any(rng);
        const Mat3f m3 = mat3(rng, nice);
        const Float4x4 m = nice ? trs(rng) : any16(rng);
        Float4 o;
        bad += !same4(quatf::add(Q(a), Q(b)).v, quatf::add(o, a, b));
        bad += !same4(quatf::scale(Q(a), s).v, quatf::scale(o, a, s));
        bad += !same(quatf::dot(Q(a), Q(b)), quatf::dot(a, b));
        bad += !same4(quatf::lerp(Q(a), Q(b), s).v, quatf::lerp(o, a, b, s));
        bad += !same(quatf::length(Q(a)), quatf::length(a));
        bad += !same(quatf::squared_length(Q(a)), quatf::squared_length(a));
        bad += !same4(quatf::normalize(Q(a)).v, quatf::normalize(o, a));
        bad += quatf::exact_equals(Q(a), Q(b)) != quatf::exact_equals(a, b);
        bad += quatf::equals(Q(a), Q(b)) != quatf::equals(a, b);
        bad += !same4(quatf::set_axis_angle(L(u), s).v, quatf::set_axis_angle(o, u, s));
        Float3 axis_s;
        Vec3f axis_r;
        bad += !same(quatf::get_axis_angle(axis_r, Q(a)), quatf::get_axis_angle(axis_s, a));
        bad += !same3(axis_r, axis_s);
        bad += !same(quatf::get_angle(Q(a), Q(b)), quatf::get_angle(a, b));
        bad += !same4(quatf::multiply(Q(a), Q(b)).v, quatf::multiply(o, a, b));
        bad += !same4(quatf::rotate_x(Q(a), s).v, quatf::rotate_x(o, a, s));
        bad += !same4(quatf::rotate_y(Q(a), s).v, quatf::rotate_y(o, a, s));
        bad += !same4(quatf::rotate_z(Q(a), s).v, quatf::rotate_z(o, a, s));
        bad += !same4(quatf::calculate_w(Q(a)).v, quatf::calculate_w(o, a));
        bad += !same4(quatf::exp(Q(a)).v, quatf::exp(o, a));
        bad += !same4(quatf::ln(Q(a)).v, quatf::ln(o, a));
        bad += !same4(quatf::pow(Q(a), s).v, quatf::pow(o, a, s));
        bad += !same4(quatf::slerp(Q(a), Q(b), s).v, quatf::slerp(o, a, b, s));
        bad += !same4(quatf::invert(Q(a)).v, quatf::invert(o, a));
        bad += !same4(quatf::conjugate(Q(a)).v, quatf::conjugate(o, a));
        bad += !same4(quatf::from_mat3(m3).v, quatf::from_mat3(o, m3));
        bad += !same4(quatf::from_mat4(L(m)).v, quatf::from_mat4(o, m));
        for (int order = 0; order < 6; order++) {
            const Eulerf e = {w.x, w.y, w.z, (EulerOrder)order};
            bad += !same4(quatf::from_euler(e).v, quatf::from_euler(o, e));
            bad += !same4(quatf::from_degrees(w.x, w.y, w.z, (EulerOrder)order).v, quatf::from_degrees(o, w.x, w.y, w.z, (EulerOrder)order));
        }
        bad += !same4(quatf::rotation_to(L(u), L(v)).v, quatf::rotation_to(o, u, v));
        bad += !same4(quatf::rotation_to(L(u), L(u)).v, quatf::rotation_to(o, u, u));
        Float3 nu;
        vec3f::negate(nu, u);
        bad += !same4(quatf::rotation_to(L(u), L(nu)).v, quatf::rotation_to(o, u, nu));
        bad += !same4(quatf::sqlerp(Q(a), Q(b), Q(c), Q(d), s).v, quatf::sqlerp(o, a, b, c, d, s));
        bad += !same4(quatf::set_axes(L(u), L(v), L(w)).v, quatf::set_axes(o, u, v, w));
        bad += !same4(quatf::target_to(L(w), L(u), L(v)).v, quatf::target_to(o, w, u, v));
        bad += !same4(quatf::target_to(L(w), L(w), L(v)).v, quatf::target_to(o, w, w, v));
        const Float3 along = {0.0f, 0.0f, 1.0f};
        bad += !same4(quatf::target_to(L(w), L(u), L(along)).v, quatf::target_to(o, w, u, along));
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
        const Float3 w = nice ? unit3(rng) : any3(rng);
        const Float4 q = nice ? unit_quat(rng) : any4(rng);
        const float s = nice ? unit(rng) * 3.0f : any(rng);
        Float4x4 o;
        Float3 o3;
        Float4 o4;
        Mat4f r;

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
        {
            Float4x4 so = {};
            r = mat4f::zero();
            const bool sok = mat4f::rotate(so, a, s, u);
            const bool rok = mat4f::rotate(r, L(a), s, L(u));
            bad += sok != rok || !same16(r, so);
        }
        bad += !same16(mat4f::rotate_x(L(a), s), mat4f::rotate_x(o, a, s));
        bad += !same16(mat4f::rotate_y(L(a), s), mat4f::rotate_y(o, a, s));
        bad += !same16(mat4f::rotate_z(L(a), s), mat4f::rotate_z(o, a, s));
        bad += !same16(mat4f::from_translation(L(u)), mat4f::from_translation(o, u));
        bad += !same16(mat4f::from_scaling(L(u)), mat4f::from_scaling(o, u));
        {
            Float4x4 so = {};
            r = mat4f::zero();
            const bool sok = mat4f::from_rotation(so, s, u);
            const bool rok = mat4f::from_rotation(r, s, L(u));
            bad += sok != rok || !same16(r, so);
        }
        bad += !same16(mat4f::from_x_rotation(s), mat4f::from_x_rotation(o, s));
        bad += !same16(mat4f::from_y_rotation(s), mat4f::from_y_rotation(o, s));
        bad += !same16(mat4f::from_z_rotation(s), mat4f::from_z_rotation(o, s));
        bad += !same16(mat4f::from_rotation_translation(Q(q), L(u)), mat4f::from_rotation_translation(o, q, u));
        bad += !same3(mat4f::get_translation(L(a)), mat4f::get_translation(o3, a));
        bad += !same3(mat4f::get_scaling(L(a)), mat4f::get_scaling(o3, a));
        bad += !same4(mat4f::rotation_with_scaling(L(a), L(v)).v, mat4f::rotation_with_scaling(o4, a, v));
        bad += !same4(mat4f::get_rotation(L(a)).v, mat4f::get_rotation(o4, a));
        {
            Float3 st, ss;
            Vec3f rt, rs;
            bad += !same4(mat4f::decompose(rt, rs, L(a)).v, mat4f::decompose(o4, st, ss, a));
            bad += !same3(rt, st) || !same3(rs, ss);
        }
        bad += !same16(mat4f::from_rotation_translation_scale(Q(q), L(u), L(v)), mat4f::from_rotation_translation_scale(o, q, u, v));
        bad += !same16(mat4f::from_rotation_translation_scale_origin(Q(q), L(u), L(v), L(w)),
                       mat4f::from_rotation_translation_scale_origin(o, q, u, v, w));
        bad += !same16(mat4f::from_quat(Q(q)), mat4f::from_quat(o, q));
        bad += !same16(mat4f::look_at(L(u), L(v), L(w)), mat4f::look_at(o, u, v, w));
        bad += !same16(mat4f::look_at(L(u), L(u), L(w)), mat4f::look_at(o, u, u, w));
        bad += !same16(mat4f::target_to(L(u), L(v), L(w)), mat4f::target_to(o, u, v, w));
        bad += !same(mat4f::frob(L(a)), mat4f::frob(a));
        bad += !same16(mat4f::add(L(a), L(b)), mat4f::add(o, a, b));
        bad += !same16(mat4f::subtract(L(a), L(b)), mat4f::subtract(o, a, b));
        bad += !same16(mat4f::multiply_scalar(L(a), s), mat4f::multiply_scalar(o, a, s));
        bad += !same16(mat4f::multiply_scalar_and_add(L(a), L(b), s), mat4f::multiply_scalar_and_add(o, a, b, s));
        bad += mat4f::exact_equals(L(a), L(b)) != mat4f::exact_equals(a, b);
        bad += mat4f::exact_equals(L(a), L(a)) != mat4f::exact_equals(a, a);
        bad += mat4f::equals(L(a), L(b)) != mat4f::equals(a, b);
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
        bad += !same_box(aabox::from_center_and_size(L(p), L(q)), box3f::set_from_center_and_size(o, p, q));
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
