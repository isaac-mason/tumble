#include "tumble/physics/shapes/capsule.h"

#include "tumble/math/aabox.h"
#include "tumble/math/mat4f.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/shapes/convex.h"
#include "tumble/util/assert.h"

TUMBLE_NAMESPACE_BEGIN

namespace capsule {

using shapes::CapsuleShape;

Shape *create(const CapsuleShapeSettings &settings) {
    TUMBLE_ASSERT(settings.radius > 0.0f);
    TUMBLE_ASSERT(settings.half_height_of_cylinder > 0.0f);

    Shape *shape = shapes::allocate(shapes::SHAPE_CAPSULE);
    if (shape == nullptr) return nullptr;
    shape->capsule.half_height_of_cylinder = settings.half_height_of_cylinder;
    shape->capsule.radius = settings.radius;
    shape->capsule.density = settings.density;
    shape->material_id = settings.material_id;
    update(shape);
    return shape;
}

void destroy(Shape *shape) { shapes::release(shape); }

float compute_capsule_volume(float half_height_of_cylinder, float radius) {
    // volume = cylinder volume + sphere volume
    // v_cylinder = pi * r^2 * h (where h = 2 * half_height)
    // v_sphere = (4/3) * pi * r^3
    const float cylinder_volume = scalar::PI * radius * radius * (2.0f * half_height_of_cylinder);
    const float sphere_volume = (4.0f / 3.0f) * scalar::PI * radius * radius * radius;
    return cylinder_volume + sphere_volume;
}

void compute_capsule_local_bounds(Box3f &out, float half_height_of_cylinder, float radius) {
    // -half_height-radius to +half_height+radius along y, +-radius in x and z
    const float total_half_height = half_height_of_cylinder + radius;
    out = {{-radius, -total_half_height, -radius}, {radius, total_half_height, radius}};
}

Vec3f compute_capsule_center_of_mass() {
    // center of mass is at origin
    return vec3f::zero();
}

void update(Shape *shape) {
    const CapsuleShape &capsule = shape->capsule;
    compute_capsule_local_bounds(shape->aabb, capsule.half_height_of_cylinder, capsule.radius);
    vec3f::store(shape->center_of_mass, compute_capsule_center_of_mass());
    shape->volume = compute_capsule_volume(capsule.half_height_of_cylinder, capsule.radius);
}

void set_support(support::Support &out, const Shape *shape, support::SupportFunctionMode mode, Vec3f scale) {
    const CapsuleShape &capsule = shape->capsule;
    const float abs_scale = scalar::abs(vec3f::x(scale));  // uniform scale only
    const float scaled_half_height = abs_scale * capsule.half_height_of_cylinder;
    const float scaled_radius = abs_scale * capsule.radius;

    out.get_support = support::capsule_support;
    out.has_transform = false;
    out.add_radius = 0.0f;
    out.capsule.half_height = scaled_half_height;

    if (mode == support::INCLUDE_CONVEX_RADIUS) {
        out.capsule.radius = scaled_radius;  // a capsule is a rounded segment
        out.convex_radius = 0.0f;
    } else {
        out.capsule.radius = 0.0f;  // segment only, the radius is reported instead
        out.convex_radius = scaled_radius;
    }
}

void compute_mass_properties(mass_properties::MassProperties &out, const Shape *shape) {
    const CapsuleShape &capsule = shape->capsule;
    const float r = capsule.radius;
    const float h = capsule.half_height_of_cylinder;

    // mass = density * volume
    out.mass = shape->volume * capsule.density;

    const float radius_sq = r * r;
    const float height = 2.0f * h;
    const float cylinder_mass = scalar::PI * height * radius_sq * capsule.density;
    const float hemisphere_mass = ((2.0f * scalar::PI) / 3.0f) * radius_sq * r * capsule.density;

    // from cylinder
    const float height_sq = height * height;
    const float inertia_y = radius_sq * cylinder_mass * 0.5f;
    const float inertia_xz = cylinder_mass * ((1.0f / 12.0f) * height_sq + (1.0f / 4.0f) * radius_sq);

    // from both hemispheres: their own inertia about the capsule's axis, then the
    // parallel axis terms for their centres sitting h + 3r/8 from the middle
    const float hemispheres_own = hemisphere_mass * 4.0f * radius_sq / 5.0f;
    const float i_yy = inertia_y + hemispheres_own;
    const float i_xx_zz = inertia_xz + hemispheres_own + hemisphere_mass * (0.5f * height_sq + 0.75f * height * r);

    out.inertia = {{i_xx_zz, 0, 0, 0, 0, i_yy, 0, 0, 0, 0, i_xx_zz, 0, 0, 0, 0, 1.0f}};
}

void get_surface_normal(shapes::SurfaceNormalResult &io, const Shape *shape, sub_shape::SubShapeId sub_shape_id) {
    (void)sub_shape_id;
    TUMBLE_ASSERT(sub_shape::is_empty(sub_shape_id));
    const CapsuleShape &capsule = shape->capsule;

    // clamp y to the cylinder range, then the normal is from the clamped point
    const float clamped_y = scalar::max(-capsule.half_height_of_cylinder, scalar::min(capsule.half_height_of_cylinder, vec3f::y(io.position)));

    const Vec3f to_point = vec3f::with_y(io.position, vec3f::y(io.position) - clamped_y);

    const float len = vec3f::length(to_point);
    if (len != 0.0f) {
        io.normal = vec3f::divide(to_point, vec3f::splat(len));
        return;
    }

    // on the central axis: radial direction from the xz plane; exactly at the center: x
    const float radial_len = scalar::sqrt(vec3f::x(io.position) * vec3f::x(io.position) + vec3f::z(io.position) * vec3f::z(io.position));
    if (radial_len > 0.0f) {
        io.normal = vec3f::make(vec3f::x(io.position) / radial_len, 0.0f, vec3f::z(io.position) / radial_len);
    } else {
        io.normal = vec3f::make(1.0f, 0.0f, 0.0f);  // arbitrary radial direction
    }
}

void get_supporting_face(shapes::SupportingFaceResult &io, Vec3f direction, const Shape *shape, sub_shape::SubShapeId) {
    // the scaled capsule: |vec3f::x(scale)| sizes it (the scale is uniform), so a mirrored scale changes
    // nothing; the transform then carries rotation and translation only
    face::Face &face = io.face;
    const float abs_scale = scalar::abs(vec3f::x(io.scale));
    const float half_height_of_cylinder = abs_scale * shape->capsule.half_height_of_cylinder;
    const float radius = abs_scale * shape->capsule.radius;

    // direction in the horizontal plane
    const float horizontal_dir_x = vec3f::x(direction);
    const float horizontal_dir_z = vec3f::z(direction);

    // zero: hitting from top or bottom, so there is no supporting face
    const float len = scalar::sqrt(horizontal_dir_x * horizontal_dir_x + horizontal_dir_z * horizontal_dir_z);
    if (len == 0.0f) {
        face.num_vertices = 0;
        return;
    }

    // support point for the top and bottom sphere opposite `direction` (including convex radius)
    const float support_x = (radius / len) * horizontal_dir_x;
    const float support_z = (radius / len) * horizontal_dir_z;

    // support_top = (0, half_height, 0) - support
    const float support_top_x = -support_x;
    const float support_top_y = half_height_of_cylinder;
    const float support_top_z = -support_z;

    // support_bottom = (0, -half_height, 0) - support
    const float support_bottom_x = -support_x;
    const float support_bottom_y = -half_height_of_cylinder;
    const float support_bottom_z = -support_z;

    // projection on direction. direction is not normalized, so both sides of the test below are
    // multiplied by its length.
    const float proj_top = support_top_x * vec3f::x(direction) + support_top_y * vec3f::y(direction) + support_top_z * vec3f::z(direction);
    const float proj_bottom = support_bottom_x * vec3f::x(direction) + support_bottom_y * vec3f::y(direction) + support_bottom_z * vec3f::z(direction);

    const float capsule_projection_slop = 0.02f;
    const float direction_length = scalar::sqrt(vec3f::x(direction) * vec3f::x(direction) + vec3f::y(direction) * vec3f::y(direction) + vec3f::z(direction) * vec3f::z(direction));

    // roughly equal: the line; otherwise there is only one point
    if (scalar::abs(proj_top - proj_bottom) < capsule_projection_slop * direction_length) {
        face.num_vertices = 2;
        face.vertices[0] = vec3f::make(support_top_x, support_top_y, support_top_z);
        face.vertices[1] = vec3f::make(support_bottom_x, support_bottom_y, support_bottom_z);
        face::transform_with_mat4_rotation_translation(face, io.transform);
    } else {
        face.num_vertices = 0;
    }
}

float get_inner_radius(const Shape *shape) { return shape->capsule.radius; }

shapes::ShapeDef def() {
    shapes::ShapeDef shape_def = {};
    shape_def.type = shapes::SHAPE_CAPSULE;
    shape_def.set_support = set_support;
    shape_def.compute_mass_properties = compute_mass_properties;
    shape_def.get_surface_normal = get_surface_normal;
    shape_def.get_supporting_face = get_supporting_face;
    shape_def.get_material = convex::get_convex_material;
    shape_def.get_inner_radius = get_inner_radius;
    shape_def.cast_ray = convex::cast_ray_vs_convex;
    shape_def.collide_point = convex::collide_point_vs_convex;
    return shapes::define_shape(shape_def);
}

}  // namespace capsule

TUMBLE_NAMESPACE_END
