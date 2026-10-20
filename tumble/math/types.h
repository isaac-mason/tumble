#pragma once

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
    // not an aggregate, so `{0, 1, 0}` is an error rather than three integer lanes of the i32x4 v128_t
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

/** columns */
struct Mat4f {
    v128_t c[4];
};

/** empty is +inf min, -inf max */
struct AABox {
    Vec3f min, max;
};

/** normal . x + constant = 0 */
struct Plane {
    Vec3f normal;
    float constant;
};

enum EulerOrder : uint8_t {
    EULER_XYZ = 0,
    EULER_XZY = 1,
    EULER_YXZ = 2,
    EULER_YZX = 3,
    EULER_ZXY = 4,
    EULER_ZYX = 5,
};

/** radians */
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

/** column-major */
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

struct Spheref {
    Float3 center;
    float radius;
};

struct Transformf {
    Float3 position;
    Float4 quaternion;
    Float3 scale;
};

static_assert(sizeof(Float3) == 12, "Float3 is three packed floats");
static_assert(sizeof(Float4) == 16, "Float4 loads as one lane");
static_assert(sizeof(Float4x4) == 64, "Float4x4 is four lanes");
static_assert(alignof(Vec3f) == 16 && alignof(Mat4f) == 16, "register types are whole lanes");

TUMBLE_NAMESPACE_END
