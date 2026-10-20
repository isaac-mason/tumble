#pragma once

// every math struct, in one place so any module can take any of them. plain floats, no padding
// beyond what the compiler needs, no methods: the functions live in the namespaces named after them.
// matrices are column-major, as the math package's are. Float3, Float4 and Float4x4 are the storage
// types, plain and unaligned; a quaternion is stored as a Float4. Vec3f, Vec4f, Quatf and Mat4f are
// the register types, one v128 each (a Mat4f four columns), which code computes in and never stores
// where js reads; vec3f::load / vec3f::store and their kin cross between the two.

#include <stdint.h>
#include <wasm_simd128.h>
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

struct Vec2f {
    float x, y;
};

struct Float3 {
    float x, y, z;
};

struct Float4 {
    float x, y, z, w;
};

/** lane 3 holds a copy of z, so a whole-register divide or sqrt makes nothing in it that z does not. */
struct Vec3f {
    v128_t v;
    // the constructor makes `{0, 1, 0}` an error: as an aggregate over the i32x4 v128_t it would compile to integer lanes
    Vec3f() = default;
    constexpr Vec3f(v128_t value) : v(value) {}
};

struct Vec4f {
    v128_t v;
    Vec4f() = default;
    constexpr Vec4f(v128_t value) : v(value) {}
};

/** x, y, z, w in lanes 0..3. */
struct Quatf {
    v128_t v;
    Quatf() = default;
    constexpr Quatf(v128_t value) : v(value) {}
};

/** four columns. */
struct Mat4f {
    v128_t c[4];
};

/** an axis aligned box, as jolt's AABox; empty is +inf min, -inf max. a record stores a box as a Box3f. */
struct AABox {
    Vec3f min, max;
};

/** a plane normal . x + constant = 0. a record stores a plane as a Plane3f. */
struct Plane {
    Vec3f normal;
    float constant;
};

/** the axis order of an euler rotation; the js names these 'xyz' and so on. */
enum EulerOrder : uint8_t {
    EULER_XYZ = 0,
    EULER_XZY = 1,
    EULER_YXZ = 2,
    EULER_YZX = 3,
    EULER_ZXY = 4,
    EULER_ZYX = 5,
};

/** euler angles in radians, applied in `order`. */
struct Eulerf {
    float x, y, z;
    EulerOrder order;
};

struct Mat2f {
    float m[4];
};

struct Mat3f {
    float m[9];
};

/** column-major. */
struct Float4x4 {
    float m[16];
};

struct Box3f {
    Float3 min, max;
};

struct Plane3f {
    Float3 normal;
    float constant;
};

struct Triangle3f {
    Float3 a, b, c;
};

struct Spheref {
    Float3 center;
    float radius;
};

struct Frustumf {
    Plane3f planes[6];
};

/** a pose with scale, the one argument that replaces crashcat's twelve scalar pose parameters. */
struct Transformf {
    Float3 position;
    Float4 quaternion;
    Float3 scale;
};

static_assert(sizeof(Float3) == 12, "Float3 packs like a js tuple of three");
static_assert(sizeof(Float4) == 16, "Float4 loads as one lane");
static_assert(sizeof(Float4x4) == 64, "Float4x4 is four lanes");
static_assert(alignof(Vec3f) == 16 && alignof(Mat4f) == 16, "register types are whole lanes");

TUMBLE_NAMESPACE_END
