#pragma once

#include <stdint.h>

#include "tumble/math/mat4f.h"
#include "tumble/math/types.h"
#include "tumble/math/vec3f.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace face {

constexpr uint32_t FACE_MAX_VERTICES = 64;

struct Face {
    Vec3f vertices[FACE_MAX_VERTICES];
    uint32_t num_vertices;
};

inline Face &clear(Face &out) {
    out.num_vertices = 0;
    return out;
}

inline Face &copy(Face &out, const Face &source) {
    out.num_vertices = source.num_vertices;
    for (uint32_t i = 0; i < source.num_vertices; i++) out.vertices[i] = source.vertices[i];
    return out;
}

/** transforms the vertices by a rotation + translation matrix (no scale); the vertices are already scaled. */
inline void transform_with_mat4_rotation_translation(Face &face, const Mat4f &matrix) {
    for (uint32_t i = 0; i < face.num_vertices; i++) face.vertices[i] = mat4f::multiply_vec3(matrix, face.vertices[i]);
}

/** transforms the vertices by a rotation + translation matrix with a separate scale applied first. */
inline void transform_with_mat4_scale(Face &face, const Mat4f &matrix, Vec3f scale) {
    Mat4f scaled = matrix;
    scaled.c[0] = v4f::mul(matrix.c[0], v4f::splat(vec3f::x(scale)));
    scaled.c[1] = v4f::mul(matrix.c[1], v4f::splat(vec3f::y(scale)));
    scaled.c[2] = v4f::mul(matrix.c[2], v4f::splat(vec3f::z(scale)));
    for (uint32_t i = 0; i < face.num_vertices; i++) face.vertices[i] = mat4f::multiply_vec3(scaled, face.vertices[i]);
}

/** whether a scale turns geometry inside out (negative determinant). */
inline bool is_scale_inside_out(Vec3f scale) { return vec3f::x(scale) * vec3f::y(scale) * vec3f::z(scale) < 0.0f; }

}  // namespace face

TUMBLE_NAMESPACE_END
