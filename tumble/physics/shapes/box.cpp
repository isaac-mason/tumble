#include "tumble/physics/shapes/box.h"

#include "tumble/math/aabox.h"
#include "tumble/math/mat4f.h"
#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec4f.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/shapes/convex.h"
#include "tumble/physics/shapes/sphere.h"
#include "tumble/util/assert.h"

TUMBLE_NAMESPACE_BEGIN

namespace box {

Shape *create(const BoxShapeSettings &settings) {
    BoxShape *shape = shapes::allocate<BoxShape>();
    // always at origin
    vec3f::store(shape->center_of_mass, vec3f::zero());
    shape->half_extents = settings.half_extents;
    shape->convex_radius = settings.convex_radius;
    shape->density = settings.density;
    shape->material_id = settings.material_id;
    update(shape);
    return shape;
}

static void destroy(Shape *shape) {
    shapes::release<BoxShape>(shape);
}

static float compute_box_volume(Vec3f half_extents) {
    // V = width * height * depth
    // half extents are already half the full dimensions, so multiply by 8
    return 8.0f * vec3f::x(half_extents) * vec3f::y(half_extents) * vec3f::z(half_extents);
}

static void compute_box_local_bounds(Box3f &out, Vec3f half_extents) {
    aabox::store(out, aabox::make(vec3f::negate(half_extents), half_extents));
}

void update(Shape *shape) {
    BoxShape &box = shapes::as<BoxShape>(shape);
    // dimensions must be non-negative
    TUMBLE_ASSERT(box.half_extents.x >= 0.0f && box.half_extents.y >= 0.0f && box.half_extents.z >= 0.0f);
    TUMBLE_ASSERT(box.convex_radius >= 0.0f);

    const Vec3f half_extents = vec3f::load(box.half_extents);
    // the convex radius fits the smallest half extent
    box.convex_radius = scalar::min(box.convex_radius, scalar::min(box.half_extents.x, scalar::min(box.half_extents.y, box.half_extents.z)));
    compute_box_local_bounds(shape->aabb, half_extents);
    shape->volume = compute_box_volume(half_extents);
}

void compute_mass_properties(MassProperties &out, const Shape *shape) {
    const Vec3f full_extents = vec3f::scale(vec3f::load(shapes::as<BoxShape>(shape).half_extents), 2.0f);
    mass_properties::set_mass_and_inertia_of_solid_box(out, full_extents, shapes::as<BoxShape>(shape).density);
}

void get_surface_normal(SurfaceNormalResult &io, const Shape *shape, SubShapeId sub_shape_id) {
    TUMBLE_ASSERT(sub_shape::is_empty(sub_shape_id));
    (void)sub_shape_id;
    const Vec3f half_extents = vec3f::load(shapes::as<BoxShape>(shape).half_extents);

    // get absolute distance from center to each face
    const Vec3f diff = vec3f::abs(vec3f::subtract(vec3f::abs(io.position), half_extents));
    const float diff_x = vec3f::x(diff);
    const float diff_y = vec3f::y(diff);
    const float diff_z = vec3f::z(diff);

    // find axis closest to box surface
    int dominant_axis = 0;
    float min_dist = diff_x;
    if (diff_y < min_dist) {
        dominant_axis = 1;
        min_dist = diff_y;
    }
    if (diff_z < min_dist) {
        dominant_axis = 2;
    }

    // return axis normal with sign based on position
    float normal[3] = {0.0f, 0.0f, 0.0f};
    const float position[3] = {vec3f::x(io.position), vec3f::y(io.position), vec3f::z(io.position)};
    normal[dominant_axis] = position[dominant_axis] > 0.0f ? 1.0f : -1.0f;
    io.normal = vec3f::make(normal[0], normal[1], normal[2]);
}

static void set_face(SupportingFace &face, float x0, float y0, float z0, float x1, float y1, float z1, float x2, float y2, float z2,
                     float x3, float y3, float z3) {
    face.vertices[0] = vec3f::make(x0, y0, z0);
    face.vertices[1] = vec3f::make(x1, y1, z1);
    face.vertices[2] = vec3f::make(x2, y2, z2);
    face.vertices[3] = vec3f::make(x3, y3, z3);
}

void get_supporting_face(SupportingFaceResult &io, Vec3f direction, const Shape *shape, SubShapeId) {
    // the face of the scaled box: |scale| sizes it, so a mirrored scale changes nothing (the box
    // is symmetric) and the winding stays outward; the transform then carries rotation and translation only
    const Vec3f half_extents = vec3f::multiply(vec3f::abs(io.scale), vec3f::load(shapes::as<BoxShape>(shape).half_extents));
    const float hx = vec3f::x(half_extents);
    const float hy = vec3f::y(half_extents);
    const float hz = vec3f::z(half_extents);
    SupportingFace &face = io.face;

    // find dominant axis
    const float abs_x = scalar::abs(vec3f::x(direction));
    const float abs_y = scalar::abs(vec3f::y(direction));
    const float abs_z = scalar::abs(vec3f::z(direction));

    face.num_vertices = 4;

    if (abs_x >= abs_y && abs_x >= abs_z) {
        // face perpendicular to x axis
        if (vec3f::x(direction) < 0.0f) {
            // positive x face (direction points toward -x, so face is on +x side)
            set_face(face, hx, -hy, -hz, hx, hy, -hz, hx, hy, hz, hx, -hy, hz);
        } else {
            set_face(face, -hx, -hy, -hz, -hx, -hy, hz, -hx, hy, hz, -hx, hy, -hz);
        }
    } else if (abs_y >= abs_x && abs_y >= abs_z) {
        // face perpendicular to y axis
        if (vec3f::y(direction) < 0.0f) {
            set_face(face, -hx, hy, -hz, -hx, hy, hz, hx, hy, hz, hx, hy, -hz);
        } else {
            set_face(face, -hx, -hy, -hz, hx, -hy, -hz, hx, -hy, hz, -hx, -hy, hz);
        }
    } else {
        // face perpendicular to z axis
        if (vec3f::z(direction) < 0.0f) {
            set_face(face, -hx, -hy, hz, hx, -hy, hz, hx, hy, hz, -hx, hy, hz);
        } else {
            set_face(face, -hx, -hy, -hz, -hx, hy, -hz, hx, hy, -hz, hx, -hy, -hz);
        }
    }

    face::transform_with_mat4_rotation_translation(face, io.transform);
}

float get_inner_radius(const Shape *shape) {
    const Vec3f half_extents = vec3f::load(shapes::as<BoxShape>(shape).half_extents);
    return scalar::min(vec3f::x(half_extents), scalar::min(vec3f::y(half_extents), vec3f::z(half_extents)));
}

void set_support(Support &out, const Shape *shape, SupportFunctionMode mode, Vec3f scale) {
    const BoxShape &box = shapes::as<BoxShape>(shape);
    const Vec3f scaled = vec3f::multiply(vec3f::abs(scale), vec3f::load(box.half_extents));

    out.get_support = support::box_support;
    out.has_transform = false;
    out.add_radius = 0.0f;

    if (mode == SupportFunctionMode::EXCLUDE_CONVEX_RADIUS) {
        const float min_scale = scalar::min(scalar::abs(vec3f::x(scale)), scalar::min(scalar::abs(vec3f::y(scale)), scalar::abs(vec3f::z(scale))));
        const float scaled_convex_radius = scalar::min(box.convex_radius * min_scale, support::DEFAULT_CONVEX_RADIUS);
        out.box.half_extents = vec3f::make(scalar::max(0.0f, vec3f::x(scaled) - scaled_convex_radius), scalar::max(0.0f, vec3f::y(scaled) - scaled_convex_radius),
                                           scalar::max(0.0f, vec3f::z(scaled) - scaled_convex_radius));
        out.convex_radius = scaled_convex_radius;
    } else {
        out.box.half_extents = scaled;
        out.convex_radius = 0.0f;
    }
}

ShapeDef def() {
    ShapeDef shape_def = {};
    shape_def.type = ShapeType::BOX;
    shape_def.destroy = destroy;
    shape_def.set_support = set_support;
    shape_def.compute_mass_properties = compute_mass_properties;
    shape_def.get_surface_normal = get_surface_normal;
    shape_def.get_supporting_face = get_supporting_face;
    shape_def.get_material = convex::get_convex_material;
    shape_def.get_inner_radius = get_inner_radius;
    shape_def.cast_ray = cast_ray_vs_box;
    shape_def.collide_point = collide_point_vs_box;
    return shapes::define_shape(shape_def);
}

/* collide sphere vs box (analytical, a = sphere, b = box) */

void collide_sphere_vs_box(CollideShapeCollector &collector, const CollideShapeSettings &settings,
                           const Shape *shape_a, const SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                           const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b) {
    const BoxShape &box = shapes::as<BoxShape>(shape_b);
    const Vec3f scale_b = vec3f::load(transform_b.scale);
    const Vec3f pos_a = vec3f::load(transform_a.position);
    const Vec3f pos_b = vec3f::load(transform_b.position);

    const float sphere_radius = shapes::as<SphereShape>(shape_a).radius * scalar::abs(transform_a.scale.x);

    // scaled half extents and shrunk core (mirror set_support EXCLUDE_CONVEX_RADIUS)
    const Vec3f scaled_half = vec3f::multiply(vec3f::abs(scale_b), vec3f::load(box.half_extents));
    const float scaled_half_x = vec3f::x(scaled_half);
    const float scaled_half_y = vec3f::y(scaled_half);
    const float scaled_half_z = vec3f::z(scaled_half);
    const float min_box_scale = scalar::min(scalar::abs(vec3f::x(scale_b)), scalar::min(scalar::abs(vec3f::y(scale_b)), scalar::abs(vec3f::z(scale_b))));
    const float scaled_convex_radius = scalar::min(box.convex_radius * min_box_scale, support::DEFAULT_CONVEX_RADIUS);
    const float core_half_x = scalar::max(0.0f, scaled_half_x - scaled_convex_radius);
    const float core_half_y = scalar::max(0.0f, scaled_half_y - scaled_convex_radius);
    const float core_half_z = scalar::max(0.0f, scaled_half_z - scaled_convex_radius);
    const float combined_radius = sphere_radius + scaled_convex_radius;

    // box rotation as a basis; isometric, so the inverse is the transpose
    const Quatf q = quatf::load(transform_b.quaternion);
    const float x2 = quatf::x(q) + quatf::x(q);
    const float y2 = quatf::y(q) + quatf::y(q);
    const float z2 = quatf::z(q) + quatf::z(q);
    const float xx = quatf::x(q) * x2;
    const float yx = quatf::y(q) * x2;
    const float yy = quatf::y(q) * y2;
    const float zx = quatf::z(q) * x2;
    const float zy = quatf::z(q) * y2;
    const float zz = quatf::z(q) * z2;
    const float wx = quatf::w(q) * x2;
    const float wy = quatf::w(q) * y2;
    const float wz = quatf::w(q) * z2;
    const float m0 = 1.0f - yy - zz;
    const float m1 = yx + wz;
    const float m2 = zx - wy;
    const float m4 = yx - wz;
    const float m5 = 1.0f - xx - zz;
    const float m6 = zy + wx;
    const float m8 = zx + wy;
    const float m9 = zy - wx;
    const float m10 = 1.0f - xx - yy;

    // sphere centre into box-local: (pos_a - pos_b) through the transposed basis
    const float offset_x = vec3f::x(pos_a) - vec3f::x(pos_b);
    const float offset_y = vec3f::y(pos_a) - vec3f::y(pos_b);
    const float offset_z = vec3f::z(pos_a) - vec3f::z(pos_b);
    const float local_x = m0 * offset_x + m1 * offset_y + m2 * offset_z;
    const float local_y = m4 * offset_x + m5 * offset_y + m6 * offset_z;
    const float local_z = m8 * offset_x + m9 * offset_y + m10 * offset_z;

    // closest point on the core box: clamp the centre to [-core_half, +core_half]
    const float closest_x = scalar::max(scalar::min(local_x, core_half_x), -core_half_x);
    const float closest_y = scalar::max(scalar::min(local_y, core_half_y), -core_half_y);
    const float closest_z = scalar::max(scalar::min(local_z, core_half_z), -core_half_z);

    // delta = centre - closest; distance squared is its squared length
    const float delta_x = local_x - closest_x;
    const float delta_y = local_y - closest_y;
    const float delta_z = local_z - closest_z;
    const float distance_sq = delta_x * delta_x + delta_y * delta_y + delta_z * delta_z;
    const float contact_distance = combined_radius + settings.max_separation_distance;
    if (distance_sq > contact_distance * contact_distance) {
        return;  // separated beyond radius (+ speculative margin)
    }

    // outward normal (box surface -> sphere centre) and the core-face contact point, in box-local
    float normal_x, normal_y, normal_z;
    float face_x, face_y, face_z;
    float penetration;

    if (distance_sq > 1e-12f) {
        // shallow: sphere centre outside the core box, normal is the normalised delta
        const float distance = scalar::sqrt(distance_sq);
        const float inv = 1.0f / distance;
        normal_x = delta_x * inv;
        normal_y = delta_y * inv;
        normal_z = delta_z * inv;
        face_x = closest_x;
        face_y = closest_y;
        face_z = closest_z;
        penetration = combined_radius - distance;
    } else {
        // deep: sphere centre inside the core box -> nearest face per axis (sat, no epa). per axis the nearest-face
        // depth is core_half - |centre|, its outward normal is sign(centre); pick the smallest-depth axis (ties x > y > z,
        // +face over -face, a straight six-face scan).
        const float depth_x = core_half_x - scalar::abs(local_x);
        const float depth_y = core_half_y - scalar::abs(local_y);
        const float depth_z = core_half_z - scalar::abs(local_z);
        normal_x = 0.0f;
        normal_y = 0.0f;
        normal_z = 0.0f;
        face_x = local_x;
        face_y = local_y;
        face_z = local_z;
        float depth;
        if (depth_x <= depth_y && depth_x <= depth_z) {
            depth = depth_x;
            normal_x = local_x < 0.0f ? -1.0f : 1.0f;
            face_x = normal_x * core_half_x;
        } else if (depth_y <= depth_z) {
            depth = depth_y;
            normal_y = local_y < 0.0f ? -1.0f : 1.0f;
            face_y = normal_y * core_half_y;
        } else {
            depth = depth_z;
            normal_z = local_z < 0.0f ? -1.0f : 1.0f;
            face_z = normal_z * core_half_z;
        }
        penetration = combined_radius + depth;
    }

    // early-out on penetration vs the collector budget (matches the convex path)
    if (-penetration >= collector.early_out_fraction) {
        return;
    }

    CollideShapeHit hit = collide_shape::create_hit();

    // contact points, box-local -> world through the same basis:
    //   box surface  = core face + convex radius along the normal
    //   sphere point = centre - sphere radius along the (box -> sphere) normal
    const float surface_x = face_x + normal_x * scaled_convex_radius;
    const float surface_y = face_y + normal_y * scaled_convex_radius;
    const float surface_z = face_z + normal_z * scaled_convex_radius;
    hit.point_b = vec3f::make(m0 * surface_x + m4 * surface_y + m8 * surface_z + vec3f::x(pos_b), m1 * surface_x + m5 * surface_y + m9 * surface_z + vec3f::y(pos_b),
                              m2 * surface_x + m6 * surface_y + m10 * surface_z + vec3f::z(pos_b));

    const float sphere_point_x = local_x - normal_x * sphere_radius;
    const float sphere_point_y = local_y - normal_y * sphere_radius;
    const float sphere_point_z = local_z - normal_z * sphere_radius;
    hit.point_a = vec3f::make(m0 * sphere_point_x + m4 * sphere_point_y + m8 * sphere_point_z + vec3f::x(pos_b),
                              m1 * sphere_point_x + m5 * sphere_point_y + m9 * sphere_point_z + vec3f::y(pos_b),
                              m2 * sphere_point_x + m6 * sphere_point_y + m10 * sphere_point_z + vec3f::z(pos_b));

    // penetration axis: a -> b = -(box-local normal, which points box -> sphere = b -> a) in world
    hit.penetration_axis = vec3f::make(-(m0 * normal_x + m4 * normal_y + m8 * normal_z), -(m1 * normal_x + m5 * normal_y + m9 * normal_z),
                                       -(m2 * normal_x + m6 * normal_y + m10 * normal_z));

    hit.penetration = penetration;
    hit.sub_shape_id_a = sub_shape_id_a.value;
    hit.sub_shape_id_b = sub_shape_id_b.value;
    hit.body_id_b = collector.body_id_b;

    if (settings.collect_faces) {
        // box supporting face: get_supporting_face picks the face by dominant axis + sign of the passed direction;
        // -normal selects the face whose outward normal is +normal
        const Vec3f face_direction = vec3f::make(-normal_x, -normal_y, -normal_z);
        const Mat4f box_to_world = mat4f::make_columns(vec4f::make(m0, m1, m2, 0.0f), vec4f::make(m4, m5, m6, 0.0f), vec4f::make(m8, m9, m10, 0.0f),
                                                       vec4f::make(vec3f::x(pos_b), vec3f::y(pos_b), vec3f::z(pos_b), 1.0f));
        shapes::get_shape_supporting_face(hit.face_b, shape_b, sub_shape_id_b.value, face_direction, box_to_world, scale_b);

        // sphere has no supporting face (single point contact)
        hit.face_a.num_vertices = 0;
    }

    collide_shape::add_hit(collector, hit);
}

/* cast ray */

void cast_ray_vs_box(CastRayCollector &collector, const CastRaySettings &settings, const Rayf &ray,
                     const Shape *shape, const SubShapeIdBuilder &sub_shape_id, const Transformf &transform) {
    // transform the ray into the box's rotation-local frame (translation + rotation removed). scale is folded into the
    // half extents below rather than inverted out of the ray, matching the gjk path's convention, so the geometry is
    // identical.
    const Quatf inv_quat = quatf::conjugate(quatf::load(transform.quaternion));

    Vec3f origin = vec3f::subtract(ray.origin, vec3f::load(transform.position));
    origin = vec3f::transform_quat(origin, inv_quat);
    Vec3f direction = vec3f::transform_quat(ray.direction, inv_quat);

    // scale the direction by ray length so the slab fractions are along [0, 1] of the ray (matching the gjk lambda
    // convention in cast_ray_vs_convex)
    direction = vec3f::scale(direction, ray.length);

    const Vec3f half_extents = vec3f::multiply(vec3f::abs(vec3f::load(transform.scale)), vec3f::load(shapes::as<BoxShape>(shape).half_extents));

    // slab test against the centered box [-h, h]; tmin/tmax bound the ray-fraction hit interval
    float tmin = -scalar::INF;
    float tmax = scalar::INF;

    const float o[3] = {vec3f::x(origin), vec3f::y(origin), vec3f::z(origin)};
    const float d[3] = {vec3f::x(direction), vec3f::y(direction), vec3f::z(direction)};
    const float h[3] = {vec3f::x(half_extents), vec3f::y(half_extents), vec3f::z(half_extents)};
    for (int axis = 0; axis < 3; axis++) {
        if (scalar::abs(d[axis]) < 1e-10f) {
            if (o[axis] < -h[axis] || o[axis] > h[axis]) {
                cast_ray::add_miss(collector);
                return;
            }
        } else {
            const float inv = 1.0f / d[axis];
            float t0 = (-h[axis] - o[axis]) * inv;
            float t1 = (h[axis] - o[axis]) * inv;
            if (t0 > t1) {
                const float t = t0;
                t0 = t1;
                t1 = t;
            }
            if (t0 > tmin) tmin = t0;
            if (t1 < tmax) tmax = t1;
            if (tmin > tmax) {
                cast_ray::add_miss(collector);
                return;
            }
        }
    }

    // the box is hit over ray fractions [tmin, tmax]; reject if it lies entirely behind the origin or starts at or
    // past the early out fraction
    if (tmax < 0.0f || tmin >= collector.early_out_fraction) {
        cast_ray::add_miss(collector);
        return;
    }

    CastRayHit hit = cast_ray::create_hit();
    hit.status = CastRayStatus::COLLIDING;
    hit.sub_shape_id = sub_shape_id.value;
    hit.body_id_b = collector.body_id_b;
    bool hit_added = false;

    // front side: tmin < 0 means the origin is inside the box
    if (settings.treat_convex_as_solid || tmin > 0.0f) {
        hit.fraction = scalar::max(0.0f, tmin);
        cast_ray::add_hit(collector, hit);
        hit_added = true;
    }

    // back side: where the ray leaves the box
    if (settings.collide_with_backfaces_convex && tmax < collector.early_out_fraction) {
        hit.fraction = tmax;
        cast_ray::add_hit(collector, hit);
        hit_added = true;
    }

    if (!hit_added) cast_ray::add_miss(collector);
}

/* collide point */

void collide_point_vs_box(CollidePointCollector &collector, const CollidePointSettings &,
                          Vec3f point, const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b,
                          const Transformf &transform_b) {
    // transform point to box's local space: rotate by the inverse of the box rotation
    const Quatf inv_quat_b = quatf::conjugate(quatf::load(transform_b.quaternion));
    Vec3f local = vec3f::subtract(point, vec3f::load(transform_b.position));
    local = vec3f::transform_quat(local, inv_quat_b);

    // apply accumulated scale to half extents
    const Vec3f scaled_half = vec3f::multiply(vec3f::load(shapes::as<BoxShape>(shape_b).half_extents), vec3f::abs(vec3f::load(transform_b.scale)));
    const float scaled_half_x = vec3f::x(scaled_half);
    const float scaled_half_y = vec3f::y(scaled_half);
    const float scaled_half_z = vec3f::z(scaled_half);

    // test if absolute value of each component is within scaled half extents
    if (scalar::abs(vec3f::x(local)) <= scaled_half_x && scalar::abs(vec3f::y(local)) <= scaled_half_y && scalar::abs(vec3f::z(local)) <= scaled_half_z) {
        CollidePointHit hit = collide_point::create_hit();
        hit.sub_shape_id_b = sub_shape_id_b.value;
        hit.body_id_b = collector.body_id_b;
        collide_point::add_hit(collector, hit);
    }
}

}  // namespace box

TUMBLE_NAMESPACE_END
