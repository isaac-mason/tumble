#include "tumble/physics/shapes/sphere.h"

#include "tumble/math/aabox.h"
#include "tumble/math/mat4f.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/shapes/convex.h"

TUMBLE_NAMESPACE_BEGIN

namespace sphere {

Shape *create(const SphereShapeSettings &settings) {
    SphereShape *shape = shapes::allocate<SphereShape>();
    if (shape == nullptr) return nullptr;
    shape->radius = settings.radius;
    shape->density = settings.density;
    shape->material_id = settings.material_id;
    // always at origin
    vec3f::store(shape->center_of_mass, vec3f::zero());
    update(shape);
    return shape;
}

void destroy(Shape *shape) { shapes::release<SphereShape>(shape); }

float compute_sphere_volume(float radius) {
    // V = (4/3) * pi * r^3
    return (4.0f / 3.0f) * scalar::PI * radius * radius * radius;
}

void compute_sphere_local_bounds(Box3f &out, float radius) { out = {{-radius, -radius, -radius}, {radius, radius, radius}}; }

void update(Shape *shape) {
    compute_sphere_local_bounds(shape->aabb, shapes::as<SphereShape>(shape).radius);
    shape->volume = compute_sphere_volume(shapes::as<SphereShape>(shape).radius);
}

void set_support(support::Support &out, const Shape *shape, support::SupportFunctionMode mode, Vec3f scale) {
    const float abs_scale = scalar::abs(vec3f::x(scale));  // uniform scale only
    out.get_support = support::sphere_support;
    out.has_transform = false;
    out.add_radius = 0.0f;

    if (mode == support::INCLUDE_CONVEX_RADIUS) {
        out.sphere.radius = shapes::as<SphereShape>(shape).radius * abs_scale;  // a sphere is a rounded point
        out.convex_radius = 0.0f;
    } else {
        out.sphere.radius = 0.0f;  // core = origin, the radius is reported instead
        out.convex_radius = shapes::as<SphereShape>(shape).radius * abs_scale;
    }
}

void compute_mass_properties(mass_properties::MassProperties &out, const Shape *shape) {
    const float radius = shapes::as<SphereShape>(shape).radius;
    const float r2 = radius * radius;
    out.mass = (4.0f / 3.0f) * scalar::PI * radius * r2 * shapes::as<SphereShape>(shape).density;

    // I = (2/5) * m * r^2
    const float inertia = (2.0f / 5.0f) * out.mass * r2;
    out.inertia = {{inertia, 0, 0, 0, 0, inertia, 0, 0, 0, 0, inertia, 0, 0, 0, 0, 1.0f}};
}

void get_surface_normal(shapes::SurfaceNormalResult &io, const Shape *, sub_shape::SubShapeId) {
    const float len = vec3f::length(io.position);

    if (len != 0.0f) {
        io.normal = vec3f::divide(io.position, vec3f::splat(len));
        return;
    }

    // fallback to y axis if position is at origin
    io.normal = vec3f::make(0.0f, 1.0f, 0.0f);
}

void get_supporting_face(shapes::SupportingFaceResult &io, Vec3f, const Shape *, sub_shape::SubShapeId) {
    // sphere has no supporting face (single point)
    io.face.num_vertices = 0;
}

float get_inner_radius(const Shape *shape) { return shapes::as<SphereShape>(shape).radius; }

shapes::ShapeDef def() {
    shapes::ShapeDef shape_def = {};
    shape_def.type = shapes::SHAPE_SPHERE;
    shape_def.set_support = set_support;
    shape_def.compute_mass_properties = compute_mass_properties;
    shape_def.get_surface_normal = get_surface_normal;
    shape_def.get_supporting_face = get_supporting_face;
    shape_def.get_material = convex::get_convex_material;
    shape_def.get_inner_radius = get_inner_radius;
    shape_def.cast_ray = convex::cast_ray_vs_convex;
    shape_def.collide_point = collide_point_vs_sphere;
    return shapes::define_shape(shape_def);
}

/* collide point */

void collide_point_vs_sphere(collide_point::CollidePointCollector &collector, const collide_point::CollidePointSettings &,
                             Vec3f point, const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b,
                             const Transformf &transform_b) {
    // transform point to sphere's local space
    const Vec3f local = vec3f::subtract(point, vec3f::load(transform_b.position));

    // apply accumulated scale to radius (uniform scale, using x component)
    const float radius = shapes::as<SphereShape>(shape_b).radius * scalar::abs(transform_b.scale.x);

    // sphere is centered at origin in local space, test distance squared
    if (vec3f::squared_length(local) <= radius * radius) {
        collide_point::CollidePointHit hit = collide_point::create_hit();
        hit.sub_shape_id_b = sub_shape_id_b.value;
        hit.body_id_b = collector.body_id_b;
        collide_point::add_hit(collector, hit);
    }
}

/* collide shape */

void collide_sphere_vs_sphere(collide_shape::CollideShapeCollector &collector, const collide_shape::CollideShapeSettings &settings,
                              const Shape *shape_a, const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                              const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b) {
    // optimized case for sphere vs sphere: simpler than gjk, just check distance between centers

    // apply accumulated scale to radii (uniform scale, using x component)
    const float radius_a = shapes::as<SphereShape>(shape_a).radius * scalar::abs(transform_a.scale.x);
    const float radius_b = shapes::as<SphereShape>(shape_b).radius * scalar::abs(transform_b.scale.x);

    // distance between centers
    const Vec3f delta = vec3f::subtract(vec3f::load(transform_b.position), vec3f::load(transform_a.position));
    const float dist_sq = vec3f::squared_length(delta);
    const float radius_sum = radius_a + radius_b;

    // no collision if the gap is wider than the speculative distance
    const float reach = radius_sum + settings.max_separation_distance;
    if (dist_sq >= reach * reach) {
        return;
    }

    const float dist = scalar::sqrt(dist_sq);
    const float penetration = radius_sum - dist;

    if (-penetration >= collector.early_out_fraction) {
        return;
    }

    // normal from a to b
    Vec3f normal;
    if (dist > 0.0f) {
        normal = vec3f::scale(delta, 1.0f / dist);
    } else {
        // spheres at same position, use arbitrary normal
        normal = vec3f::make(0.0f, 1.0f, 0.0f);
    }

    // report hit; a sphere has no supporting face
    collide_shape::CollideShapeHit hit = collide_shape::create_hit();
    hit.point_a = vec3f::scale_and_add(vec3f::load(transform_a.position), normal, radius_a);
    hit.point_b = vec3f::scale_and_add(vec3f::load(transform_b.position), normal, -radius_b);
    hit.penetration_axis = normal;
    hit.penetration = penetration;
    hit.sub_shape_id_a = sub_shape_id_a.value;
    hit.sub_shape_id_b = sub_shape_id_b.value;
    hit.body_id_b = collector.body_id_b;
    collide_shape::add_hit(collector, hit);
}

}  // namespace sphere

TUMBLE_NAMESPACE_END
