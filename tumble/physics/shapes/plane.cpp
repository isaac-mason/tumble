#include "tumble/physics/shapes/plane.h"

#include "tumble/math/aabox.h"
#include "tumble/math/mat4f.h"
#include "tumble/math/plane3f.h"
#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/util/assert.h"

TUMBLE_NAMESPACE_BEGIN

namespace plane {

Shape *create(const PlaneShapeSettings &settings) {
    Shape *shape = shapes::allocate(shapes::SHAPE_PLANE);
    if (shape == nullptr) return nullptr;
    // normalise the whole equation so the plane stays where it was described
    const float normal_length = vec3f::length(vec3f::load(settings.plane.normal));
    vec3f::store(shape->plane.plane.normal, vec3f::scale(vec3f::load(settings.plane.normal), 1.0f / normal_length));
    shape->plane.plane.constant = settings.plane.constant / normal_length;
    shape->plane.half_extent = settings.half_extent;
    shape->material_id = settings.material_id;
    // always at the origin
    vec3f::store(shape->center_of_mass, vec3f::zero());

    update(shape);

    return shape;
}

void destroy(Shape *shape) { shapes::release(shape); }

/* geometry helpers */

/** scales a plane by a per-axis scale, non-uniform and negative included */
static Plane scale_plane(const Plane &plane, Vec3f scale) {
    // scaled normal = normal / scale (component-wise division)
    const Vec3f scaled_normal = vec3f::divide(plane.normal, scale);
    const float scaled_normal_length = vec3f::length(scaled_normal);
    const float inverse_length = 1.0f / scaled_normal_length;
    return {vec3f::scale(scaled_normal, inverse_length), plane.constant / scaled_normal_length};
}

/** transforms a plane by a rotation + translation; no renormalisation, the transform is orthogonal */
static Plane transform_plane(const Plane &plane, const Mat4f &transform) {
    // transform normal by rotation part (3x3 multiply)
    const Vec3f normal = mat4f::multiply3x3_vec(transform, plane.normal);

    // update constant: constant - translation . transformed normal
    return {normal, plane.constant - vec3f::dot(mat4f::get_translation(transform), normal)};
}

/** an orthogonal basis for a plane normal, for the quad representation */
static void get_orthogonal_basis(Vec3f normal, Vec3f &out_perp1, Vec3f &out_perp2) {
    const Vec3f axis_y = vec3f::make(0.0f, 1.0f, 0.0f);
    out_perp1 = vec3f::cross(normal, axis_y);

    const float len_sq = vec3f::squared_length(out_perp1);
    if (len_sq < 1e-6f) {
        // parallel to y, use x instead
        out_perp1 = vec3f::make(1.0f, 0.0f, 0.0f);
    } else {
        out_perp1 = vec3f::scale(out_perp1, 1.0f / scalar::sqrt(len_sq));
    }

    out_perp2 = vec3f::cross(out_perp1, normal);
    out_perp2 = vec3f::normalize(out_perp2);

    // recompute perp1 for stability
    out_perp1 = vec3f::cross(normal, out_perp2);
    out_perp1 = vec3f::normalize(out_perp1);
}

/** the four corners of a quad centred on `center` spanned by the two scaled perpendiculars */
static void get_quad_vertices(Vec3f out[4], Vec3f center, Vec3f perp1, Vec3f perp2) {
    Vec3f temp = vec3f::add(perp1, perp2);
    out[0] = vec3f::add(center, temp);

    temp = vec3f::subtract(perp1, perp2);
    out[1] = vec3f::add(center, temp);

    temp = vec3f::add(perp1, perp2);
    out[2] = vec3f::subtract(center, temp);

    // vertex 3 = center + (-perp1 + perp2)
    temp = vec3f::negate(perp1);
    temp = vec3f::add(temp, perp2);
    out[3] = vec3f::add(center, temp);
}

static void get_plane_vertices(Vec3f out[4], const Shape *shape) {
    const Vec3f normal = vec3f::load(shape->plane.plane.normal);

    // create orthogonal basis
    Vec3f perp1, perp2;
    get_orthogonal_basis(normal, perp1, perp2);

    // scale by half extent
    perp1 = vec3f::scale(perp1, shape->plane.half_extent);
    perp2 = vec3f::scale(perp2, shape->plane.half_extent);

    // point on plane: -normal * constant
    const Vec3f point = vec3f::scale(normal, -shape->plane.plane.constant);

    get_quad_vertices(out, point, perp1, perp2);
}

static Box3f &compute_plane_local_bounds(Box3f &out, const Shape *shape) {
    Vec3f vertices[4];
    get_plane_vertices(vertices, shape);

    const Vec3f offset = vec3f::scale(vec3f::load(shape->plane.plane.normal), shape->plane.half_extent);

    // encapsulate all vertices and points behind the plane
    AABox bounds = aabox::empty();
    for (uint32_t i = 0; i < 4; i++) {
        bounds = aabox::expand_by_point(bounds, vertices[i]);

        // also include the point half extent behind the plane
        bounds = aabox::expand_by_point(bounds, vec3f::subtract(vertices[i], offset));
    }
    return aabox::store(out, bounds);
}

void update(Shape *shape) {
    compute_plane_local_bounds(shape->aabb, shape);
    shape->volume = 0.0f;
}

/* shape def */

void compute_mass_properties(mass_properties::MassProperties &out, const Shape *) {
    // planes are static only: no mass, no inertia
    out.mass = 0.0f;
    mat4f::store(out.inertia, mat4f::zero());
}

void get_surface_normal(shapes::SurfaceNormalResult &io, const Shape *shape, sub_shape::SubShapeId) {
    // the plane normal is always the surface normal
    io.normal = vec3f::load(shape->plane.plane.normal);
}

void get_supporting_face(shapes::SupportingFaceResult &io, Vec3f, const Shape *shape, sub_shape::SubShapeId) {
    face::Face &face = io.face;

    // the 4 vertices of the plane quad in local space
    Vec3f vertices[4];
    get_plane_vertices(vertices, shape);

    // a scale that turns the plane inside out reverses the winding: 0, 1, 2, 3 -> 3, 2, 1, 0
    const bool inside_out = face::is_scale_inside_out(io.scale);

    face.num_vertices = 4;
    for (uint32_t i = 0; i < 4; i++) face.vertices[i] = vertices[inside_out ? 3 - i : i];

    face::transform_with_mat4_scale(face, io.transform, io.scale);
}

int32_t get_material(const Shape *shape, sub_shape::SubShapeId sub_shape_id) {
    TUMBLE_ASSERT(sub_shape::is_empty(sub_shape_id));
    (void)sub_shape_id;
    return shape->material_id;
}

float get_inner_radius(const Shape *) {
    // planes are infinite, no meaningful inner radius
    return 0.0f;
}

/** the plane in world space: scaled, then moved by the transform's rotation and translation */
static Plane world_plane(const Shape *shape, const Transformf &transform) {
    const Plane scaled_plane = scale_plane(plane3f::load(shape->plane.plane), vec3f::load(transform.scale));
    const Mat4f rotation_translation = mat4f::from_rotation_translation(quatf::load(transform.quaternion), vec3f::load(transform.position));
    return transform_plane(scaled_plane, rotation_translation);
}

/* cast ray */

void cast_ray_vs_plane(cast_ray::CastRayCollector &collector, const cast_ray::CastRaySettings &settings, const cast_ray::Rayf &ray,
                       const Shape *shape, const sub_shape::SubShapeIdBuilder &sub_shape_id, const Transformf &transform) {
    // transform plane to world space
    const Plane plane = world_plane(shape, transform);

    // signed distance from the ray origin to the plane
    const float distance = plane3f::distance_to_point(plane, ray.origin);

    // inside the solid half space (distance <= 0)?
    if (settings.treat_convex_as_solid && distance <= 0.0f && collector.early_out_fraction > 0.0f) {
        cast_ray::CastRayHit hit = cast_ray::create_hit();
        hit.status = cast_ray::COLLIDING;
        hit.fraction = 0.0f;
        hit.sub_shape_id = sub_shape_id.value;
        hit.material_id = shape->material_id;
        hit.body_id_b = collector.body_id_b;
        cast_ray::add_hit(collector, hit);
    }

    // ray direction against the plane normal
    const float dot = vec3f::dot(ray.direction, plane.normal);

    // parallel to plane?
    if (dot == 0.0f) return;

    // back face culling
    const bool is_back_facing = dot > 0.0f;
    if (!settings.collide_with_backfaces_convex && is_back_facing) return;

    // hit fraction, normalised by the ray length
    const float fraction = -distance / dot / ray.length;

    // valid hit?
    if (fraction >= 0.0f && fraction < collector.early_out_fraction) {
        cast_ray::CastRayHit hit = cast_ray::create_hit();
        hit.status = cast_ray::COLLIDING;
        hit.fraction = fraction;
        hit.sub_shape_id = sub_shape_id.value;
        hit.material_id = shape->material_id;
        hit.body_id_b = collector.body_id_b;
        cast_ray::add_hit(collector, hit);
    } else {
        cast_ray::add_miss(collector);
    }
}

/* collide point */

void collide_point_vs_plane(collide_point::CollidePointCollector &collector, const collide_point::CollidePointSettings &,
                            Vec3f point, const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b,
                            const Transformf &transform_b) {
    // transform plane to world space
    const Plane plane = world_plane(shape_b, transform_b);

    // a point in the negative half space is inside the solid
    if (plane3f::distance_to_point(plane, point) < 0.0f) {
        collide_point::CollidePointHit hit = collide_point::create_hit();
        hit.sub_shape_id_b = sub_shape_id_b.value;
        hit.material_id = shape_b->material_id;
        hit.body_id_b = collector.body_id_b;
        collide_point::add_hit(collector, hit);
    }
}

/* convex vs plane collision */

/**
 * an adaptive-sized supporting face for the plane, sized by the convex shape's bounding box so the clipped face
 * stays numerically sound
 */
static void get_adaptive_plane_supporting_face(face::Face &out_face, const Shape *convex_shape, Vec3f convex_world_pos,
                                               const Plane &plane, const Mat4f &plane_to_world) {
    // transform plane to world space
    const Plane world = transform_plane(plane, plane_to_world);

    // project the convex position onto the plane
    const Vec3f center = plane3f::project_point(world, convex_world_pos);

    // create orthogonal basis
    Vec3f perp1, perp2;
    get_orthogonal_basis(world.normal, perp1, perp2);

    // size based on the convex shape's bounding box diagonal
    const float size = vec3f::length(aabox::size(aabox::load(convex_shape->aabb)));

    perp1 = vec3f::scale(perp1, size);
    perp2 = vec3f::scale(perp2, size);

    Vec3f vertices[4];
    get_quad_vertices(vertices, center, perp1, perp2);

    out_face.num_vertices = 4;
    for (uint32_t i = 0; i < 4; i++) out_face.vertices[i] = vertices[i];
}

static support::Support collide_convex_vs_plane_support;

void collide_convex_vs_plane(collide_shape::CollideShapeCollector &collector, const collide_shape::CollideShapeSettings &settings,
                             const Shape *shape_a, const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                             const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b) {
    // scale plane b
    const Plane scaled_plane = scale_plane(plane3f::load(shape_b->plane.plane), vec3f::load(transform_b.scale));

    // transform b: the plane's world transform, no scale (already applied to the plane)
    Mat4f plane_to_world = mat4f::from_rotation_translation(quatf::load(transform_b.quaternion), vec3f::load(transform_b.position));

    // transform a, built once and reused
    const Mat4f convex_to_world = mat4f::from_rotation_translation(quatf::load(transform_a.quaternion), vec3f::load(transform_a.position));

    Mat4f inv_convex_to_world;
    mat4f::invert(inv_convex_to_world, convex_to_world);

    // transform the plane to the convex shape's local space (inverse a * transform b)
    const Mat4f combined_transform = mat4f::multiply(inv_convex_to_world, plane_to_world);
    const Plane local_plane = transform_plane(scaled_plane, combined_transform);

    const Vec3f normal = local_plane.normal;

    // support function for the convex shape with its scale applied
    support::Support &support = collide_convex_vs_plane_support;
    shapes::set_shape_support(support, shape_a, support::DEFAULT, vec3f::load(transform_a.scale));

    // support point in the direction opposite the plane normal
    const Vec3f negated_normal = vec3f::negate(normal);
    const Vec3f support_point = support.get_support(support, negated_normal);

    // penetration
    const float signed_distance = plane3f::distance_to_point(local_plane, support_point);
    const float convex_radius = support.convex_radius;
    const float penetration = -signed_distance + convex_radius;

    // within tolerance?
    if (penetration > -settings.max_separation_distance) {
        // contact points in convex local space
        Vec3f point1 = vec3f::scale_and_add(support_point, normal, -convex_radius);
        Vec3f point2 = vec3f::scale_and_add(support_point, normal, -signed_distance);

        collide_shape::CollideShapeHit hit = collide_shape::create_hit();

        // contact points to world space
        hit.point_a = vec3f::transform_mat4(point1, convex_to_world);
        hit.point_b = vec3f::transform_mat4(point2, convex_to_world);

        // penetration axis to world space (rotation only)
        hit.penetration_axis = mat4f::multiply3x3_vec(convex_to_world, negated_normal);

        hit.penetration = penetration;
        hit.sub_shape_id_a = sub_shape_id_a.value;
        hit.sub_shape_id_b = sub_shape_id_b.value;
        hit.material_id_a = shape_a->material_id;
        hit.material_id_b = shape_b->material_id;
        hit.body_id_b = collector.body_id_b;

        // gather supporting faces if requested
        if (settings.collect_faces) {
            // supporting face of the convex shape in world space; the direction is local
            shapes::get_shape_supporting_face(hit.face_a, shape_a, sub_shape_id_a.value, normal, convex_to_world, vec3f::load(transform_a.scale));

            // adaptive supporting face for the plane
            if (hit.face_a.num_vertices > 0) {
                get_adaptive_plane_supporting_face(hit.face_b, shape_a, vec3f::load(transform_a.position), scaled_plane, plane_to_world);
            }
        }

        collide_shape::add_hit(collector, hit);
    } else {
        collide_shape::add_miss(collector);
    }
}

/* cast convex vs plane */

static support::Support cast_convex_vs_plane_support;

void cast_convex_vs_plane(cast_shape::CastShapeCollector &collector, const cast_shape::CastShapeSettings &settings, const Shape *shape_a,
                          const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a, Vec3f displacement_a,
                          const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b) {
    // move the cast into the plane's space: a's start transform and the displacement relative to b
    const Mat4f a_to_world = mat4f::from_rotation_translation(quatf::load(transform_a.quaternion), vec3f::load(transform_a.position));
    const Mat4f b_to_world = mat4f::from_rotation_translation(quatf::load(transform_b.quaternion), vec3f::load(transform_b.position));
    Mat4f inv_b_to_world, a_to_b;
    mat4f::invert(inv_b_to_world, b_to_world);
    a_to_b = mat4f::multiply(inv_b_to_world, a_to_world);

    const Quatf inv_quat_b = quatf::conjugate(quatf::load(transform_b.quaternion));
    const Vec3f direction_in_b = vec3f::transform_quat(displacement_a, inv_quat_b);

    // in the plane's space only the scale remains to be applied to the plane
    const Plane plane = scale_plane(plane3f::load(shape_b->plane.plane), vec3f::load(transform_b.scale));
    const Vec3f normal = plane.normal;

    // support function
    support::Support &support = cast_convex_vs_plane_support;
    shapes::set_shape_support(support, shape_a, support::DEFAULT, vec3f::load(transform_a.scale));

    // support point of the convex shape opposite the plane normal, in the plane's space
    const Vec3f normal_in_convex_space = mat4f::multiply3x3_transposed_vec(a_to_b, normal);
    const Vec3f negated_normal_in_convex_space = vec3f::negate(normal_in_convex_space);
    const Vec3f support_point_local = support.get_support(support, negated_normal_in_convex_space);
    const Vec3f support_point = vec3f::transform_mat4(support_point_local, a_to_b);

    const float signed_distance = plane3f::distance_to_point(plane, support_point);
    const float convex_radius = support.convex_radius + settings.extra_convex_radius;
    const float penetration_depth = -signed_distance + convex_radius;
    const float dot = vec3f::dot(direction_in_b, normal);

    Mat4f com_hit;
    Vec3f point1, point2;
    float fraction;

    // starting in collision?
    if (penetration_depth > 0.0f) {
        // back face culling?
        if (!settings.collide_with_backfaces_convex && dot > 0.0f) return;

        // shallower hit?
        if (penetration_depth <= -collector.early_out_fraction) return;

        // hit at fraction 0
        fraction = 0.0f;

        com_hit = b_to_world;

        Vec3f contact_local = vec3f::scale_and_add(support_point, normal, -convex_radius);
        point1 = vec3f::transform_mat4(contact_local, com_hit);

        contact_local = vec3f::scale_and_add(support_point, normal, -signed_distance);
        point2 = vec3f::transform_mat4(contact_local, com_hit);
    } else if (dot < 0.0f) {
        // moving towards the plane, calculate time of impact
        fraction = penetration_depth / dot;

        // further than early out?
        if (fraction >= collector.early_out_fraction) return;

        // the plane's transform moved along the displacement in its own space, so contact points land at the time of impact
        Vec3f offset = vec3f::scale(direction_in_b, fraction);
        com_hit = mat4f::translate(b_to_world, offset);

        Vec3f contact_local = vec3f::scale_and_add(support_point, normal, -convex_radius);
        point1 = vec3f::transform_mat4(contact_local, com_hit);
        point2 = point1;
    } else {
        // moving away from plane or parallel
        return;
    }

    // penetration axis in world space
    Vec3f negated_normal = vec3f::negate(normal);
    const Vec3f penetration_axis_world = mat4f::multiply3x3_vec(com_hit, negated_normal);

    // create hit
    cast_shape::CastShapeHit hit = cast_shape::create_hit();
    hit.status = cast_shape::COLLIDING;
    hit.fraction = fraction;
    hit.point_a = point1;
    hit.point_b = point2;
    hit.penetration_depth = vec3f::distance(point1, point2);
    hit.penetration_axis = penetration_axis_world;
    hit.normal = vec3f::negate(penetration_axis_world);
    hit.normal = vec3f::normalize(hit.normal);
    hit.sub_shape_id_a = sub_shape_id_a.value;
    hit.sub_shape_id_b = sub_shape_id_b.value;
    hit.material_id_a = shape_a->material_id;
    hit.material_id_b = shape_b->material_id;
    hit.body_id_b = collector.body_id_b;

    // gather faces if requested
    if (settings.collect_faces) {
        // the convex shape's transform at the hit
        const Mat4f shape_to_world = mat4f::multiply(com_hit, a_to_b);

        shapes::get_shape_supporting_face(hit.face_a, shape_a, sub_shape_id_a.value, normal_in_convex_space, shape_to_world, vec3f::load(transform_a.scale));

        if (hit.face_a.num_vertices > 0) {
            const Vec3f shape_to_world_position = mat4f::get_translation(shape_to_world);
            get_adaptive_plane_supporting_face(hit.face_b, shape_a, shape_to_world_position, plane, b_to_world);
        }
    }

    cast_shape::add_hit(collector, hit);
}

shapes::ShapeDef def() {
    shapes::ShapeDef shape_def = {};
    shape_def.type = shapes::SHAPE_PLANE;
    shape_def.compute_mass_properties = compute_mass_properties;
    shape_def.get_surface_normal = get_surface_normal;
    shape_def.get_supporting_face = get_supporting_face;
    shape_def.get_material = get_material;
    shape_def.get_inner_radius = get_inner_radius;
    shape_def.cast_ray = cast_ray_vs_plane;
    shape_def.collide_point = collide_point_vs_plane;
    return shapes::define_shape(shape_def);
}

}  // namespace plane

TUMBLE_NAMESPACE_END
