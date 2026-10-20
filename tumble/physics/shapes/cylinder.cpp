#include "tumble/physics/shapes/cylinder.h"

#include "tumble/math/aabox.h"
#include "tumble/math/mat4f.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/shapes/convex.h"
#include "tumble/util/assert.h"

TUMBLE_NAMESPACE_BEGIN

namespace cylinder {

using shapes::CylinderShape;

// octagon template for cap faces
// y component is always 1 (unit circle at y = 1), scaled to +-half height
static const float CYLINDER_CAP_OCTAGON[8][3] = {
    {0.0f, 1.0f, 1.0f},
    {0.707106769f, 1.0f, 0.707106769f},  // sqrt(2)/2
    {1.0f, 1.0f, 0.0f},
    {0.707106769f, 1.0f, -0.707106769f},
    {0.0f, 1.0f, -1.0f},
    {-0.707106769f, 1.0f, -0.707106769f},
    {-1.0f, 1.0f, 0.0f},
    {-0.707106769f, 1.0f, 0.707106769f},
};

Shape *create(const CylinderShapeSettings &settings) {
    // auto-clamp convex radius to fit the smallest dimension
    const float convex_radius = scalar::min(settings.convex_radius, scalar::min(settings.half_height, settings.radius));

    Shape *shape = shapes::allocate(shapes::SHAPE_CYLINDER);
    if (shape == nullptr) return nullptr;
    shape->cylinder.half_height = settings.half_height;
    shape->cylinder.radius = settings.radius;
    shape->cylinder.convex_radius = convex_radius;
    shape->cylinder.density = settings.density;
    shape->material_id = settings.material_id;
    update(shape);
    return shape;
}

void destroy(Shape *shape) { shapes::release(shape); }

/** V = pi * r^2 * (2h) */
static float calculate_volume(float half_height, float radius) { return scalar::PI * radius * radius * 2.0f * half_height; }

static void calculate_aabb(Box3f &out, float half_height, float radius) {
    out = {{-radius, -half_height, -radius}, {radius, half_height, radius}};
}

void update(Shape *shape) {
    const CylinderShape &cylinder = shape->cylinder;
    // dimensions must be non-negative
    TUMBLE_ASSERT(cylinder.half_height >= 0.0f);
    TUMBLE_ASSERT(cylinder.radius >= 0.0f);
    TUMBLE_ASSERT(cylinder.convex_radius >= 0.0f);

    calculate_aabb(shape->aabb, cylinder.half_height, cylinder.radius);
    vec3f::store(shape->center_of_mass, vec3f::zero());
    shape->volume = calculate_volume(cylinder.half_height, cylinder.radius);
}

void compute_mass_properties(mass_properties::MassProperties &out, const Shape *shape) {
    const CylinderShape &cylinder = shape->cylinder;

    // mass = density * volume
    out.mass = shape->volume * cylinder.density;

    // inertia tensor for solid cylinder aligned with y axis:
    // I_yy = 0.5 * mass * r^2
    // I_xx = I_zz = 0.5 * I_yy + mass * (2h)^2 / 12
    const float r = cylinder.radius;
    const float h = cylinder.half_height;
    const float mass = out.mass;

    const float iyy = 0.5f * mass * r * r;
    const float ixx = 0.5f * iyy + (mass * 4.0f * h * h) / 12.0f;
    const float izz = ixx;

    out.inertia = {{ixx, 0, 0, 0, 0, iyy, 0, 0, 0, 0, izz, 0, 0, 0, 0, 1.0f}};
}

void get_surface_normal(shapes::SurfaceNormalResult &io, const Shape *shape, sub_shape::SubShapeId) {
    const CylinderShape &cylinder = shape->cylinder;
    const float x = vec3f::x(io.position);
    const float y = vec3f::y(io.position);
    const float z = vec3f::z(io.position);

    // calculate distance to curved surface and cap surfaces
    const float radial_dist = scalar::sqrt(x * x + z * z);
    const float dist_curved = scalar::abs(radial_dist - cylinder.radius);
    const float dist_caps = scalar::abs(scalar::abs(y) - cylinder.half_height);

    if (dist_caps > dist_curved) {
        // closer to curved surface - return radial normal
        if (radial_dist > 0.0f) {
            io.normal = vec3f::make(x / radial_dist, 0.0f, z / radial_dist);
        } else {
            // point on central axis - default to x axis
            io.normal = vec3f::make(1.0f, 0.0f, 0.0f);
        }
    } else {
        // closer to cap - return +-y normal
        io.normal = vec3f::make(0.0f, y >= 0.0f ? 1.0f : -1.0f, 0.0f);
    }
}

void get_supporting_face(shapes::SupportingFaceResult &io, Vec3f direction, const Shape *shape, sub_shape::SubShapeId) {
    // the scaled cylinder: |scale| sizes it, so a mirrored scale changes nothing and the cap keeps
    // its winding; the transform then carries rotation and translation only
    face::Face &face = io.face;
    const float half_height = scalar::abs(vec3f::y(io.scale)) * shape->cylinder.half_height;
    const float radius = scalar::abs(vec3f::x(io.scale)) * shape->cylinder.radius;

    // calculate squared components to determine if hitting side or cap
    const float xz_sq = vec3f::x(direction) * vec3f::x(direction) + vec3f::z(direction) * vec3f::z(direction);
    const float y_sq = vec3f::y(direction) * vec3f::y(direction);

    if (xz_sq > y_sq) {
        // case 1: side face - direction is more horizontal than vertical
        // returns 2 vertices forming a vertical edge on the curved surface

        face.num_vertices = 2;

        const float horizontal_len = scalar::sqrt(xz_sq);

        // point on curved surface in penetration direction (negative)
        const float f = -radius / horizontal_len;
        const float vx = vec3f::x(direction) * f;
        const float vz = vec3f::z(direction) * f;

        face.vertices[0] = vec3f::make(vx, half_height, vz);
        face.vertices[1] = vec3f::make(vx, -half_height, vz);
    } else {
        // case 2: cap face - direction is more vertical than horizontal
        // returns 8 vertices (octagon approximation of circular cap)

        face.num_vertices = 8;

        // the cap facing against the direction: the top cap for a downward direction, the
        // bottom cap (mirrored) for an upward one
        const float scale_x = vec3f::y(direction) < 0.0f ? radius : -radius;
        const float scale_y = vec3f::y(direction) < 0.0f ? half_height : -half_height;
        const float scale_z = radius;

        // when direction is more than 5 degrees from vertical, rotate octagon so one vertex
        // points toward max penetration
        // 0.00765427 ~= tan^2(5 degrees)
        float rotation_cos = 1.0f;
        float rotation_sin = 0.0f;
        if (xz_sq > 0.00765427f * y_sq) {
            // normalize xz direction to get rotation angle
            const float horizontal_len = scalar::sqrt(xz_sq);
            rotation_cos = vec3f::x(direction) / horizontal_len;
            rotation_sin = vec3f::z(direction) / horizontal_len;
        }

        for (uint32_t i = 0; i < 8; i++) {
            const float *octagon_vertex = CYLINDER_CAP_OCTAGON[i];

            // first apply scale
            const float scaled_x = scale_x * octagon_vertex[0];
            const float scaled_y = scale_y * octagon_vertex[1];
            const float scaled_z = scale_z * octagon_vertex[2];

            // then apply 2d rotation in xz plane
            const float rotated_x = scaled_x * rotation_cos - scaled_z * rotation_sin;
            const float rotated_z = scaled_x * rotation_sin + scaled_z * rotation_cos;

            face.vertices[i] = vec3f::make(rotated_x, scaled_y, rotated_z);
        }
    }

    face::transform_with_mat4_rotation_translation(face, io.transform);
}

float get_inner_radius(const Shape *shape) { return scalar::min(shape->cylinder.half_height, shape->cylinder.radius); }

void set_support(support::Support &out, const Shape *shape, support::SupportFunctionMode mode, Vec3f scale) {
    const CylinderShape &cylinder = shape->cylinder;
    // x and z scale the radius (they must match), y the half height
    const float scale_xz = scalar::abs(vec3f::x(scale));
    const float scale_y = scalar::abs(vec3f::y(scale));
    const float scaled_half_height = scale_y * cylinder.half_height;
    const float scaled_radius = scale_xz * cylinder.radius;
    out.get_support = support::cylinder_support;
    out.has_transform = false;
    out.add_radius = 0.0f;

    if (mode == support::INCLUDE_CONVEX_RADIUS || mode == support::DEFAULT) {
        out.cylinder.half_height = scaled_half_height;
        out.cylinder.radius = scaled_radius;
        out.convex_radius = 0.0f;
    } else {
        const float min_scale = scalar::min(scalar::min(scale_xz, scale_y), scalar::abs(vec3f::z(scale)));
        const float scaled_convex_radius = scalar::min(cylinder.convex_radius * min_scale, support::DEFAULT_CONVEX_RADIUS);
        out.cylinder.half_height = scaled_half_height - scaled_convex_radius;
        out.cylinder.radius = scaled_radius - scaled_convex_radius;
        out.convex_radius = scaled_convex_radius;
    }
}

shapes::ShapeDef def() {
    shapes::ShapeDef shape_def = {};
    shape_def.type = shapes::SHAPE_CYLINDER;
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

}  // namespace cylinder

TUMBLE_NAMESPACE_END
