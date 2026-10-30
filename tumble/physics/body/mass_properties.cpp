#include "tumble/physics/body/mass_properties.h"

#include "tumble/math/scalar.h"
#include "tumble/math/mat4f.h"
#include "tumble/math/vec3f.h"

TUMBLE_NAMESPACE_BEGIN

namespace mass_properties {

MassProperties &set_mass_and_inertia_of_solid_box(MassProperties &out, Vec3f box_size, float density) {
    out.mass = vec3f::x(box_size) * vec3f::y(box_size) * vec3f::z(box_size) * density;

    const Vec3f size_sq = vec3f::multiply(box_size, box_size);
    const float sx = vec3f::x(size_sq), sy = vec3f::y(size_sq), sz = vec3f::z(size_sq);
    const Vec3f scale = vec3f::make((sy + sz) * (out.mass / 12.0f), (sx + sz) * (out.mass / 12.0f), (sx + sy) * (out.mass / 12.0f));

    out.inertia = mat4f::from_scaling(scale);

    return out;
}

MassProperties &scale(MassProperties &out, const MassProperties &source, Vec3f scale) {
    Mat4f inertia = source.inertia;

    // diagonal of the 3x3 inertia tensor
    const Vec3f diagonal = vec3f::make(mat4f::element(inertia, 0, 0), mat4f::element(inertia, 1, 1), mat4f::element(inertia, 2, 2));

    // xyz_sq = 0.5 * dot(diagonal) - diagonal
    const float half_dot = 0.5f * (vec3f::x(diagonal) + vec3f::y(diagonal) + vec3f::z(diagonal));
    const Vec3f xyz_sq = vec3f::subtract(vec3f::splat(half_dot), diagonal);

    // xyz_scaled_sq = scale * scale * xyz_sq (component-wise)
    const Vec3f xyz_scaled_sq = vec3f::multiply(vec3f::multiply(scale, scale), xyz_sq);

    // new diagonal elements
    const float i_xx = vec3f::y(xyz_scaled_sq) + vec3f::z(xyz_scaled_sq);
    const float i_yy = vec3f::x(xyz_scaled_sq) + vec3f::z(xyz_scaled_sq);
    const float i_zz = vec3f::x(xyz_scaled_sq) + vec3f::y(xyz_scaled_sq);

    // new off-diagonal elements
    const float scale_x = vec3f::x(scale), scale_y = vec3f::y(scale), scale_z = vec3f::z(scale);
    const float i_xy = scale_x * scale_y * mat4f::element(inertia, 0, 1);
    const float i_xz = scale_x * scale_z * mat4f::element(inertia, 0, 2);
    const float i_yz = scale_y * scale_z * mat4f::element(inertia, 1, 2);

    // mass scales by volume (absolute value of scale factors product)
    const float mass_scale = scalar::abs(scale_x * scale_y * scale_z);

    mat4f::set_element(inertia, 0, 0, i_xx);
    mat4f::set_element(inertia, 1, 1, i_yy);
    mat4f::set_element(inertia, 2, 2, i_zz);
    mat4f::set_element(inertia, 0, 1, i_xy);
    mat4f::set_element(inertia, 1, 0, i_xy);
    mat4f::set_element(inertia, 0, 2, i_xz);
    mat4f::set_element(inertia, 2, 0, i_xz);
    mat4f::set_element(inertia, 1, 2, i_yz);
    mat4f::set_element(inertia, 2, 1, i_yz);

    out.mass = source.mass * mass_scale;

    // inertia scales linearly with mass
    out.inertia = mat4f::multiply_scalar(inertia, mass_scale);
    mat4f::set_element(out.inertia, 3, 3, 1.0f);

    return out;
}

MassProperties &rotate(MassProperties &out, const MassProperties &source, const Mat4f &rotation) {
    // rotation doesn't affect mass
    out.mass = source.mass;

    // I' = R * I * R^T over the upper 3x3, the rest identity
    const Mat4f rotated = mat4f::multiply3x3(rotation, source.inertia);
    out.inertia = mat4f::multiply3x3_right_transposed(rotated, rotation);

    return out;
}

MassProperties &translate(MassProperties &out, const MassProperties &source, Vec3f translation) {
    Mat4f inertia = source.inertia;
    out.mass = source.mass;

    // parallel axis theorem: I' = I + m * (translation^2 * E - translation * translation^T)
    // where E is identity and translation^2 is the dot product of translation with itself
    const float translation_dot_self = vec3f::dot(translation, translation);
    const float t[3] = {vec3f::x(translation), vec3f::y(translation), vec3f::z(translation)};

    for (int col = 0; col < 3; col++) {
        for (int row = 0; row < 3; row++) {
            const float diagonal = row == col ? translation_dot_self : 0.0f;
            mat4f::set_element(inertia, row, col, mat4f::element(inertia, row, col) + out.mass * (diagonal - t[row] * t[col]));
        }
    }

    mat4f::set_element(inertia, 1, 3, 0.0f);
    mat4f::set_element(inertia, 2, 3, 0.0f);
    mat4f::set_element(inertia, 3, 3, 1.0f);
    out.inertia = inertia;

    return out;
}

MassProperties &scale_to_mass(MassProperties &out, float new_mass) {
    if (out.mass > 0.0f) {
        // calculate how much we have to scale the inertia tensor
        const float mass_scale = new_mass / out.mass;

        out.mass = new_mass;

        // scale the 3x3 inertia tensor (upper-left of the 4x4 matrix)
        for (int col = 0; col < 3; col++) {
            for (int row = 0; row < 3; row++) mat4f::set_element(out.inertia, row, col, mat4f::element(out.inertia, row, col) * mass_scale);
        }
    } else {
        // just set the mass if the current mass is zero
        out.mass = new_mass;
    }

    return out;
}

}  // namespace mass_properties

TUMBLE_NAMESPACE_END
