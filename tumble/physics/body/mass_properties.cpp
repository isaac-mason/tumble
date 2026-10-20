#include "tumble/physics/body/mass_properties.h"

#include "tumble/math/scalar.h"
#include "tumble/math/mat4f.h"
#include "tumble/math/vec3f.h"

TUMBLE_NAMESPACE_BEGIN

namespace mass_properties {

MassProperties &set_mass_and_inertia_of_solid_box(MassProperties &out, Vec3f box_size, float density) {
    // calculate mass
    out.mass = vec3f::x(box_size) * vec3f::y(box_size) * vec3f::z(box_size) * density;

    // calculate inertia
    const Vec3f size_sq = vec3f::multiply(box_size, box_size);
    const float sx = vec3f::x(size_sq), sy = vec3f::y(size_sq), sz = vec3f::z(size_sq);
    const Vec3f scale = vec3f::make((sy + sz) * (out.mass / 12.0f), (sx + sz) * (out.mass / 12.0f), (sx + sy) * (out.mass / 12.0f));

    mat4f::store(out.inertia, mat4f::from_scaling(scale));

    return out;
}

MassProperties &scale(MassProperties &out, const MassProperties &source, Vec3f scale) {
    const float *in = source.inertia.m;

    // diagonal of the 3x3 inertia tensor (elements 0, 5, 10)
    const Vec3f diagonal = vec3f::make(in[0], in[5], in[10]);

    // xyz_sq = 0.5 * dot(diagonal) - diagonal
    const float half_dot = 0.5f * (vec3f::x(diagonal) + vec3f::y(diagonal) + vec3f::z(diagonal));
    const Vec3f xyz_sq = vec3f::subtract(vec3f::splat(half_dot), diagonal);

    // xyz_scaled_sq = scale * scale * xyz_sq (component-wise)
    const Vec3f xyz_scaled_sq = vec3f::multiply(vec3f::multiply(scale, scale), xyz_sq);

    // new diagonal elements
    const float i_xx = vec3f::y(xyz_scaled_sq) + vec3f::z(xyz_scaled_sq);
    const float i_yy = vec3f::x(xyz_scaled_sq) + vec3f::z(xyz_scaled_sq);
    const float i_zz = vec3f::x(xyz_scaled_sq) + vec3f::y(xyz_scaled_sq);

    // new off-diagonal elements (column-major: 4 is (0,1), 8 is (0,2), 9 is (1,2))
    const float scale_x = vec3f::x(scale), scale_y = vec3f::y(scale), scale_z = vec3f::z(scale);
    const float i_xy = scale_x * scale_y * in[4];
    const float i_xz = scale_x * scale_z * in[8];
    const float i_yz = scale_y * scale_z * in[9];

    // mass scales by volume (absolute value of scale factors product)
    const float mass_scale = scalar::abs(scale_x * scale_y * scale_z);

    // copy and update inertia tensor
    out.inertia = source.inertia;
    float *o = out.inertia.m;
    o[0] = i_xx;
    o[1] = i_xy;
    o[4] = i_xy;
    o[5] = i_yy;
    o[2] = i_xz;
    o[8] = i_xz;
    o[6] = i_yz;
    o[9] = i_yz;
    o[10] = i_zz;

    out.mass = source.mass * mass_scale;

    // inertia scales linearly with mass
    for (int i = 0; i < 15; i++) o[i] *= mass_scale;

    o[15] = 1.0f;

    return out;
}

MassProperties &rotate(MassProperties &out, const MassProperties &source, const Mat4f &rotation) {
    // rotation doesn't affect mass
    out.mass = source.mass;

    // I' = R * I * R^T over the upper 3x3, the rest identity
    const Mat4f rotated = mat4f::multiply3x3(rotation, mat4f::load(source.inertia));
    mat4f::store(out.inertia, mat4f::multiply3x3_right_transposed(rotated, rotation));

    return out;
}

MassProperties &translate(MassProperties &out, const MassProperties &source, Vec3f translation) {
    out.inertia = source.inertia;
    out.mass = source.mass;

    // parallel axis theorem: I' = I + m * (translation^2 * E - translation * translation^T)
    // where E is identity and translation^2 is the dot product of translation with itself
    const float translation_dot_self = vec3f::dot(translation, translation);
    const float t[3] = {vec3f::x(translation), vec3f::y(translation), vec3f::z(translation)};

    for (int col = 0; col < 3; col++) {
        for (int row = 0; row < 3; row++) {
            const float diagonal = row == col ? translation_dot_self : 0.0f;
            out.inertia.m[col * 4 + row] += source.mass * (diagonal - t[row] * t[col]);
        }
    }

    out.inertia.m[13] = 0.0f;
    out.inertia.m[14] = 0.0f;
    out.inertia.m[15] = 1.0f;

    return out;
}

MassProperties &scale_to_mass(MassProperties &out, float new_mass) {
    if (out.mass > 0.0f) {
        // calculate how much we have to scale the inertia tensor
        const float mass_scale = new_mass / out.mass;

        out.mass = new_mass;

        // scale the 3x3 inertia tensor (upper-left of the 4x4 matrix)
        for (int col = 0; col < 3; col++) {
            for (int row = 0; row < 3; row++) out.inertia.m[col * 4 + row] *= mass_scale;
        }
    } else {
        // just set the mass if the current mass is zero
        out.mass = new_mass;
    }

    return out;
}

}  // namespace mass_properties

TUMBLE_NAMESPACE_END
