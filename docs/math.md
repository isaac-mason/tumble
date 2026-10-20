# math

Two kinds of type: ones that are stored and ones that compute.

**Storage types** (`types.h`): `Float3`, `Float4` (also a quaternion), `Float4x4` (column-major), `Box3f`, `Plane3f`,
`Transformf`. Plain and unaligned: the only math types in records, arrays, events and hits.

**Register types**: `Vec3f` (lane 3 is a copy of z), `Vec4f`, `Quatf`, `Mat4f` (four column lanes), and `AABox` /
`Plane` built from them. Never stored in records. Their functions live in the namespace named after the type
(`vec3f::`, `mat4f::`, ...), take and return values, and build on the raw lanes in `simd.h` (`v4f::`).

**Crossing** is explicit: `vec3f::load(const Float3 &)`, `vec3f::store(Float3 &, Vec3f)`, and the same for the rest.

**Exactness**: each register function matches its scalar reference in `tests/math/storage/` bit for bit, held by
`tests/math/register.test.cpp`. Component-wise operations are lane operations; horizontal ones keep scalar order (the
three-wide dot is `(x * x + y * y) + z * z`, the four-wide `(x + z) + (y + w)`). Matrix multiply is four column
passes; the scalar-shaped functions (invert, adjoint, the decompositions, the view builders) read their entries out
and keep the scalar expressions. `min` / `max` are `pmin` / `pmax` (`b < a ? b : a`); `fmin` / `fmax` are IEEE min /
max (nan propagates, -0 below +0).

**Failure**: functions that can fail on a singular matrix or a zero axis (`invert`, `invert3x3`, `rotate`,
`from_rotation`, `normal_from_mat4`) return false and leave `out` untouched.

`Mat2f`, `Mat3f`, `Vec2f` and `Eulerf` are small scalar types used by the constraint solver.
