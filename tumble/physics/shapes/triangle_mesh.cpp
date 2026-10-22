#include "tumble/physics/shapes/triangle_mesh.h"

#include "tumble/math/aabox.h"
#include "tumble/math/mat4f.h"
#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/collision/active_edges.h"
#include "tumble/physics/collision/cast_utils.h"
#include "tumble/physics/collision/closest_points.h"
#include "tumble/physics/collision/gjk.h"
#include "tumble/physics/collision/penetration.h"
#include "tumble/physics/collision/simplex.h"
#include "tumble/physics/collision/sphere_triangle.h"
#include "tumble/physics/collision/support.h"
#include "tumble/physics/collision/triangle.h"
#include "tumble/physics/shapes/sphere.h"
#include "tumble/physics/shapes/utils/bvh.h"
#include "tumble/physics/shapes/utils/triangle_mesh_bvh.h"
#include "tumble/util/assert.h"

TUMBLE_NAMESPACE_BEGIN

namespace triangle_mesh {

using triangle_mesh_data::TriangleMeshData;

/** the walks' fixed stacks: each pop pushes at most two, so this holds any tree up to this deep */
static constexpr int32_t STACK_SIZE = 128;

static inline void push_node(int32_t *stack_nodes, int32_t &stack_size, int32_t node) {
    TUMBLE_ASSERT(stack_size < STACK_SIZE);
    if (stack_size < STACK_SIZE) stack_nodes[stack_size++] = node;
}

static inline void push_node(int32_t *stack_nodes, float *stack_dist, int32_t &stack_size, int32_t node, float distance) {
    TUMBLE_ASSERT(stack_size < STACK_SIZE);
    if (stack_size >= STACK_SIZE) return;
    stack_nodes[stack_size] = node;
    stack_dist[stack_size] = distance;
    stack_size++;
}

Shape *create(const TriangleMeshShapeSettings &settings) {
    TriangleMeshData *data = settings.data;
    TUMBLE_ASSERT(data != nullptr);
    TriangleMeshShape *shape = shapes::allocate<TriangleMeshShape>();
    if (shape == nullptr) return nullptr;
    shape->data = data;
    shape->owns_data = settings.owns_data;
    // a mesh's materials are its triangles'
    shape->material_id = -1;
    update(shape);
    return shape;
}

void destroy(Shape *shape) {
    const TriangleMeshShape &mesh = shapes::as<TriangleMeshShape>(shape);
    if (mesh.owns_data) triangle_mesh_data::destroy(mesh.data);
    shapes::release<TriangleMeshShape>(shape);
}

void update(Shape *shape) {
    const TriangleMeshData &data = *shapes::as<TriangleMeshShape>(shape).data;
    if (data.node_count > 0) {
        bvh::node_get_bounds(shape->aabb, data.nodes, 0);
    } else {
        shape->aabb = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
    }
    shape->center_of_mass = {0.0f, 0.0f, 0.0f};
    shape->volume = 0.0f;
}

void compute_mass_properties(mass_properties::MassProperties &out, const Shape *) {
    // triangle meshes are static collision geometry by default.
    // mass properties should be overridden at the body level if needed.
    out.mass = 0.0f;
    mat4f::store(out.inertia, mat4f::identity());
}

void get_surface_normal(shapes::SurfaceNormalResult &io, const Shape *shape, sub_shape::SubShapeId sub_shape_id) {
    const TriangleMeshData &data = *shapes::as<TriangleMeshShape>(shape).data;
    sub_shape::PopResult pop;
    sub_shape::pop_index(pop, sub_shape_id, data.triangle_count);
    const uint32_t triangle_index = pop.value;

    TUMBLE_ASSERT(triangle_index < data.triangle_count);
    if (triangle_index < data.triangle_count) io.normal = triangle_mesh_data::get_triangle_normal(data, triangle_index);
}

int32_t get_material(const Shape *shape, sub_shape::SubShapeId sub_shape_id) {
    const TriangleMeshData &data = *shapes::as<TriangleMeshShape>(shape).data;
    sub_shape::PopResult pop;
    sub_shape::pop_index(pop, sub_shape_id, data.triangle_count);
    TUMBLE_ASSERT(pop.value < data.triangle_count);
    return triangle_mesh_data::get_material_id(data, pop.value);
}

void get_supporting_face(shapes::SupportingFaceResult &io, Vec3f, const Shape *shape, sub_shape::SubShapeId sub_shape_id) {
    const TriangleMeshData &data = *shapes::as<TriangleMeshShape>(shape).data;
    face::Face &face = io.face;

    // extract triangle index from sub shape id
    sub_shape::PopResult pop;
    sub_shape::pop_index(pop, sub_shape_id, data.triangle_count);

    Vec3f a, b, c;
    triangle_mesh_data::get_triangle_vertices(a, b, c, data, pop.value);

    // check if scale inverts winding (negative determinant)
    const bool inside_out = face::is_scale_inside_out(io.scale);

    // return the 3 vertices of the triangle, a, b, c reversed to c, b, a when inside out
    const Vec3f first = inside_out ? c : a;
    const Vec3f last = inside_out ? a : c;
    face.num_vertices = 3;
    face.vertices[0] = first;
    face.vertices[1] = b;
    face.vertices[2] = last;

    face::transform_with_mat4_scale(face, io.transform, io.scale);
}

/* cast ray */

void cast_ray_vs_triangle_mesh(cast_ray::CastRayCollector &collector, const cast_ray::CastRaySettings &settings, const cast_ray::Rayf &ray,
                               const Shape *shape, const sub_shape::SubShapeIdBuilder &sub_shape_id, const Transformf &transform) {
    const TriangleMeshData &data = *shapes::as<TriangleMeshShape>(shape).data;
    const bvh::Node *nodes = data.nodes;

    if (data.node_count == 0) {
        cast_ray::add_miss(collector);
        return;
    }

    // transform the ray into the mesh's unscaled local space: rigid inverse, then divide origin and
    // direction by the scale. the bvh and the vertices are unscaled, so nothing is scaled per
    // triangle; the hit parameter t is unchanged by rescaling origin and direction together, so the
    // fraction needs no correction either
    const Quatf inv_quat = quatf::conjugate(quatf::load(transform.quaternion));

    Vec3f ray_origin_local = vec3f::subtract(ray.origin, vec3f::load(transform.position));
    Vec3f ray_direction_local;
    ray_origin_local = vec3f::transform_quat(ray_origin_local, inv_quat);
    ray_direction_local = vec3f::transform_quat(ray.direction, inv_quat);

    const Vec3f scale = vec3f::load(transform.scale);
    cast_ray::Rayf local_ray;
    local_ray.origin = vec3f::divide(ray_origin_local, scale);
    local_ray.direction = vec3f::divide(ray_direction_local, scale);
    local_ray.length = ray.length;

    // reciprocals of the local displacement, once per query, for the fraction-space slab tests
    const Vec3f inv_displacement = vec3f::make(cast_utils::safe_reciprocal(vec3f::x(local_ray.direction) * ray.length),
                                    cast_utils::safe_reciprocal(vec3f::y(local_ray.direction) * ray.length),
                                    cast_utils::safe_reciprocal(vec3f::z(local_ray.direction) * ray.length));

    // a mirrored scale needs no winding fix-up: the ray was mirrored into local space with the
    // mesh, so the local front-face test already answers for the world-space surface side
    const bool backface_culling = !settings.collide_with_backfaces_triangles;

    // the collector's early-out fraction, refreshed after each hit. the triangle test gets the same
    // bound in ray units, clamped to the ray length: the initial early-out sits a hair above 1 so
    // that a hit at the very end still counts, but nothing beyond the segment may
    float early_out = collector.early_out_fraction;
    float max_t = early_out < 1.0f ? early_out * ray.length : ray.length;

    bool found_hit = false;
    cast_utils::RayIntersectsTriangleResult hit_result = cast_utils::create_result();

    int32_t stack_nodes[STACK_SIZE];
    float stack_dist[STACK_SIZE];
    int32_t stack_size = 0;
    push_node(stack_nodes, stack_dist, stack_size, 0, -scalar::INF);  // root always visited

    while (stack_size > 0) {
        stack_size--;
        const int32_t node = stack_nodes[stack_size];
        const float node_distance = stack_dist[stack_size];

        // early out: if fraction to this node >= closest hit, skip it. a node is only ever pushed
        // with a finite entry fraction, so no re-test of its bounds is needed here
        if (node_distance >= early_out) continue;

        if (bvh::node_is_leaf(nodes, node)) {
            // leaf: check triangles
            const uint32_t tri_start = triangle_mesh_bvh::node_tri_start(nodes, node);
            const uint32_t tri_count = triangle_mesh_bvh::node_tri_count(nodes, node);
            for (uint32_t i = 0; i < tri_count; i++) {
                // an any-hit collector drops its early-out to zero: nothing further can qualify
                if (early_out <= 0.0f) break;

                const uint32_t triangle_index = tri_start + i;
                Vec3f a, b, c;
                triangle_mesh_data::get_triangle_vertices(a, b, c, data, triangle_index);

                // note: no per-triangle aabb test, the bvh node aabb already provides tight culling
                // and the ray x triangle test is cheap next to gjk/epa or a shape cast
                cast_utils::ray_intersects_triangle(hit_result, local_ray, max_t, a, b, c, backface_culling);

                // the triangle test already rejected anything beyond max_t
                if (hit_result.hit) {
                    found_hit = true;

                    sub_shape::SubShapeIdBuilder triangle_sub_shape_id;
                    sub_shape::push_index(triangle_sub_shape_id, sub_shape_id, triangle_index, data.triangle_count);

                    cast_ray::CastRayHit hit = cast_ray::create_hit();
                    hit.status = cast_ray::COLLIDING;
                    hit.fraction = hit_result.fraction;
                    hit.sub_shape_id = triangle_sub_shape_id.value;
                    hit.body_id_b = collector.body_id_b;
                    cast_ray::add_hit(collector, hit);

                    early_out = collector.early_out_fraction;
                    max_t = early_out < 1.0f ? early_out * ray.length : ray.length;
                }
            }
        } else {
            // internal node: entry fractions of both children; push the farther child first so the
            // closer one is popped first
            const int32_t left = bvh::node_left(node);
            const int32_t right = bvh::node_right(nodes, node);

            const float left_dist = cast_utils::ray_fraction_to_box3(local_ray.origin, inv_displacement, aabox::load(nodes[left].bounds));
            const float right_dist = cast_utils::ray_fraction_to_box3(local_ray.origin, inv_displacement, aabox::load(nodes[right].bounds));

            if (left_dist <= right_dist) {
                if (right_dist < early_out) push_node(stack_nodes, stack_dist, stack_size, right, right_dist);
                if (left_dist < early_out) push_node(stack_nodes, stack_dist, stack_size, left, left_dist);
            } else {
                if (left_dist < early_out) push_node(stack_nodes, stack_dist, stack_size, left, left_dist);
                if (right_dist < early_out) push_node(stack_nodes, stack_dist, stack_size, right, right_dist);
            }
        }
    }

    if (!found_hit) cast_ray::add_miss(collector);
}

/* collide point */

/** counts every crossing of the parity ray, never early-outs */
struct HitCountContext {
    uint32_t hit_count;
    sub_shape::SubShapeId last_sub_shape_id;
};

static void hit_count_add_hit(cast_ray::CastRayCollector &collector, const cast_ray::CastRayHit &hit) {
    HitCountContext &context = *static_cast<HitCountContext *>(collector.context);
    context.hit_count++;
    context.last_sub_shape_id = hit.sub_shape_id;
}

static bool hit_count_should_early_out(const cast_ray::CastRayCollector &) {
    return false;  // never early out, count all hits
}

void collide_point_vs_triangle_mesh(collide_point::CollidePointCollector &collector, const collide_point::CollidePointSettings &,
                                    Vec3f point, const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b,
                                    const Transformf &transform_b) {
    // transform point to mesh's local space: inverse rotation, then inverse scale
    const Quatf inv_quat_b = quatf::conjugate(quatf::load(transform_b.quaternion));
    Vec3f local_point = vec3f::subtract(point, vec3f::load(transform_b.position));
    local_point = vec3f::transform_quat(local_point, inv_quat_b);
    const Vec3f inv_scale = vec3f::inverse(vec3f::load(transform_b.scale));
    local_point = vec3f::multiply(local_point, inv_scale);

    // early exit if point is outside aabb (point is already in unscaled local space)
    const AABox aabb = aabox::load(shape_b->aabb);
    if (!aabox::contains_point(aabb, local_point)) return;

    // construct ray from point in +y direction
    // ray length = 10% longer than aabb height to ensure we exit the mesh
    const cast_ray::Rayf ray = {local_point, vec3f::axis_y(), vec3f::y(aabox::size(aabb)) * 1.1f};

    // a mesh with no height gives a ray of no length, which crosses nothing: the point is outside
    if (ray.length == 0.0f) return;

    // the odd-even rule counts every crossing, front or back
    cast_ray::CastRaySettings cast_ray_settings = cast_ray::default_settings();
    cast_ray_settings.collide_with_backfaces_triangles = true;
    cast_ray_settings.collide_with_backfaces_convex = true;

    HitCountContext context = {0, sub_shape::EMPTY_SUB_SHAPE_ID};
    cast_ray::CastRayCollector hit_counter = cast_ray::create_collector(collector::COLLECT_CUSTOM);
    hit_counter.body_id_b = collector.body_id_b;
    hit_counter.custom_add_hit = hit_count_add_hit;
    hit_counter.custom_should_early_out = hit_count_should_early_out;
    hit_counter.context = &context;

    // cast ray through mesh, counting all triangle intersections; the mesh is already in local space
    const Transformf local_space = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}};
    cast_ray_vs_triangle_mesh(hit_counter, cast_ray_settings, ray, shape_b, sub_shape_id_b, local_space);

    // apply odd-even rule (jordan curve theorem)
    // odd number of hits = point is inside
    // even number of hits = point is outside
    if ((context.hit_count & 1) != 0) {
        collide_point::CollidePointHit hit = collide_point::create_hit();
        hit.sub_shape_id_b = context.last_sub_shape_id;
        hit.body_id_b = collector.body_id_b;
        collide_point::add_hit(collector, hit);
    }
}

/**
 * bring a box from the mesh's scaled local space into the unscaled space its bvh is built in.
 * a negative scale component swaps that axis' min and max, so they are re-ordered.
 */
static AABox divide_box_by_scale(const AABox &box, Vec3f scale) {
    if (vec3f::x(scale) == 1.0f && vec3f::y(scale) == 1.0f && vec3f::z(scale) == 1.0f) return box;

    const V128 lo = v4f::div(box.min.v, scale.v);
    const V128 hi = v4f::div(box.max.v, scale.v);
    // lo < hi ? lo : hi, and lo < hi ? hi : lo, per lane
    const V128 lo_first = v4f::lt(lo, hi);
    return {{v4f::select(lo, hi, lo_first)}, {v4f::select(hi, lo, lo_first)}};
}

static void set_face_triangle(face::Face &face, Vec3f a, Vec3f b, Vec3f c) {
    face.num_vertices = 3;
    face.vertices[0] = a;
    face.vertices[1] = b;
    face.vertices[2] = c;
}

static void set_triangle_support(support::Support &out, Vec3f a, Vec3f b, Vec3f c) { support::set_triangle_support(out, a, b, c, 0.0f); }

/* cast shape */

static support::Support cast_support_a;
static support::Support cast_triangle_support;

void cast_convex_vs_triangle_mesh(cast_shape::CastShapeCollector &collector, const cast_shape::CastShapeSettings &settings,
                                  const Shape *shape_a, const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                                  Vec3f displacement_a, const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b,
                                  const Transformf &transform_b) {
    const TriangleMeshData &data = *shapes::as<TriangleMeshShape>(shape_b).data;
    const bvh::Node *nodes = data.nodes;

    // early out if there are no triangles
    if (data.node_count == 0) return;

    const Vec3f scale_a = vec3f::load(transform_a.scale);
    const Vec3f scale_b = vec3f::load(transform_b.scale);

    // transform a into b's local space (as cast_convex_vs_convex): rotation and translation only, a's
    // scale goes to its support
    const Mat4f transform_a_mat = mat4f::from_rotation_translation(quatf::load(transform_a.quaternion), vec3f::load(transform_a.position));
    const Mat4f target_transform = mat4f::from_rotation_translation(quatf::load(transform_b.quaternion), vec3f::load(transform_b.position));
    Mat4f inv_target_transform, cast_transform;
    mat4f::invert(inv_target_transform, target_transform);

    // a's start transform in b's space
    cast_transform = mat4f::multiply(inv_target_transform, transform_a_mat);

    // transform displacement to b's space
    const Vec3f displacement_in_b = mat4f::multiply3x3_vec(inv_target_transform, displacement_a);

    // compute base aabb of shape a at t=0 in mesh local space, then into the unscaled space the bvh
    // is built in. the cast itself stays in scaled mesh space; the walk ray, half-extents and
    // displacement below are the unscaled-space copies
    const AABox swept_aabb = divide_box_by_scale(aabox::transform_mat4(aabox::scale(aabox::load(shape_a->aabb), scale_a), cast_transform), scale_b);
    const Vec3f walk_displacement = vec3f::divide(displacement_in_b, scale_b);

    // determine if we want to use the actual shape or a shrunken shape with convex radius
    const support::SupportFunctionMode support_mode =
        settings.use_shrunken_shape_and_convex_radius ? support::EXCLUDE_CONVEX_RADIUS : support::DEFAULT;
    support::Support &support_a = cast_support_a;
    shapes::set_shape_support(support_a, shape_a, support_mode, scale_a);

    // determine if shape is inside out or not
    const float scale_sign = vec3f::is_scale_inside_out(scale_b) ? -1.0f : 1.0f;

    // cast ray: swept aabb center along the normalized displacement
    cast_ray::Rayf walk_ray;
    walk_ray.origin = aabox::center(swept_aabb);
    walk_ray.length = vec3f::length(walk_displacement);
    const float inv_ray_length = walk_ray.length > 1e-10f ? 1.0f / walk_ray.length : 0.0f;
    walk_ray.direction = vec3f::scale(walk_displacement, inv_ray_length);

    // half-extents of the base aabb
    const Vec3f half_extents = vec3f::add(aabox::extents(swept_aabb), vec3f::divide(vec3f::splat(settings.extra_convex_radius), vec3f::abs(scale_b)));

    // the active edge movement hint in the mesh's local space
    Vec3f active_edge_movement_direction = mat4f::multiply3x3_transposed_vec(target_transform, settings.active_edge_movement_direction);

    int32_t stack_nodes[STACK_SIZE];
    float stack_dist[STACK_SIZE];
    int32_t stack_size = 0;
    push_node(stack_nodes, stack_dist, stack_size, 0, 0.0f);  // root always visited

    while (stack_size > 0) {
        stack_size--;
        const int32_t node = stack_nodes[stack_size];
        const float node_distance = stack_dist[stack_size];

        // early out: if fraction to this node >= closest hit, skip it
        if (node_distance >= scalar::max(scalar::FLOAT_MIN_NORMAL, collector.early_out_fraction)) continue;

        // note: no need to test ray intersection here - we already proved the ray
        // intersects this node's expanded bounds when we computed the distance during push.

        if (bvh::node_is_leaf(nodes, node)) {
            // leaf: test triangles
            const uint32_t tri_start = triangle_mesh_bvh::node_tri_start(nodes, node);
            const uint32_t tri_count = triangle_mesh_bvh::node_tri_count(nodes, node);

            for (uint32_t i = 0; i < tri_count; i++) {
                const uint32_t triangle_index = tri_start + i;

                // triangle aabb (precomputed at build) expanded by the cast shape's half-extents
                const float *triangle_aabb = data.triangle_aabbs + triangle_index * 6;
                const AABox tri_bounds = aabox::expand_by_extents({vec3f::load(triangle_aabb, 0), vec3f::load(triangle_aabb, 3)}, half_extents);

                // early out: ray x triangle expanded bounds
                if (!cast_utils::ray_hits_box3(walk_ray, tri_bounds)) continue;

                // stop once the collector wants no more hits
                if (collector.early_out_fraction <= -collector::UNBOUNDED) return;

                // scaled triangle
                Vec3f a, b, c;
                triangle_mesh_data::get_triangle_vertices(a, b, c, data, triangle_index);
                a = vec3f::multiply(a, scale_b);
                b = vec3f::multiply(b, scale_b);
                c = vec3f::multiply(c, scale_b);

                // scaled triangle normal
                Vec3f edge_a = vec3f::subtract(b, a);
                Vec3f edge_b = vec3f::subtract(c, a);
                Vec3f normal = vec3f::cross(edge_a, edge_b);
                normal = vec3f::scale(normal, scale_sign);

                // backface check
                const bool back_facing = vec3f::dot(normal, displacement_in_b) > 0.0f;
                if (!settings.collide_with_backfaces_triangles && back_facing) continue;

                set_triangle_support(cast_triangle_support, a, b, c);

                // gjk shapecast with epa fallback for deep penetration, a folded into b's space by cast_transform
                gjk::GjkCastShapeResult gjk_result = gjk::create_gjk_cast_shape_result();
                penetration::cast_shape(gjk_result, cast_transform, support_a, cast_triangle_support, displacement_in_b,
                                        settings.collision_tolerance, settings.penetration_tolerance,
                                        support_a.convex_radius + settings.extra_convex_radius,
                                        0.0f,  // triangle has no convex radius
                                        collector.early_out_fraction, settings.return_deepest_point);

                if (!gjk_result.hit) continue;

                const float penetration_depth = vec3f::distance(gjk_result.point_a, gjk_result.point_b);

                // early out: if this hit is deeper than the collector's early out value
                if (gjk_result.lambda == 0.0f && -penetration_depth >= collector.early_out_fraction) continue;

                // active edge detection - correct normal if hitting inactive edge
                const uint32_t triangle_active_edges = triangle_mesh_data::get_active_edges(data, triangle_index);
                if (settings.collide_only_with_active_edges && triangle_active_edges != 0b111) {
                    // the triangle normal is flipped as the axis points towards the triangle instead of away
                    const Vec3f triangle_normal_for_fix = back_facing ? normal : vec3f::negate(normal);

                    // apply active edge correction (all parameters in mesh's local space)
                    const Vec3f corrected_axis =
                        active_edges::fix_normal(a, b, c, triangle_normal_for_fix, triangle_active_edges, gjk_result.point_b,
                                                 gjk_result.separating_axis, active_edge_movement_direction);
                    gjk_result.separating_axis = corrected_axis;
                }

                cast_shape::CastShapeHit hit = cast_shape::create_hit();

                // convert to world space
                hit.point_a = vec3f::transform_mat4(gjk_result.point_a, target_transform);
                hit.point_b = vec3f::transform_mat4(gjk_result.point_b, target_transform);
                hit.penetration_axis = mat4f::multiply3x3_vec(target_transform, gjk_result.separating_axis);

                hit.status = cast_shape::COLLIDING;
                hit.fraction = gjk_result.lambda;
                hit.normal = vec3f::normalize(hit.penetration_axis);
                hit.normal = vec3f::negate(hit.normal);
                hit.penetration_depth = penetration_depth;
                hit.sub_shape_id_a = sub_shape_id_a.value;

                sub_shape::SubShapeIdBuilder triangle_sub_shape_id;
                sub_shape::push_index(triangle_sub_shape_id, sub_shape_id_b, triangle_index, data.triangle_count);
                hit.sub_shape_id_b = triangle_sub_shape_id.value;
                hit.body_id_b = collector.body_id_b;

                // gather faces if requested
                if (settings.collect_faces) {
                    // face a: the contact normal from b's local space into a's local space through the
                    // transposed 3x3 of cast_transform (inverse rotation)
                    const Vec3f normal_in_b = vec3f::negate(gjk_result.separating_axis);
                    const Vec3f normal_in_a = mat4f::multiply3x3_transposed_vec(cast_transform, normal_in_b);

                    // shape a at the contact: cast_transform with translation += fraction * displacement_in_b, into world space
                    Mat4f a_at_contact = cast_transform;
                    a_at_contact.c[3] = v4f::add(a_at_contact.c[3], v4f::set_lane<3>(vec3f::scale(displacement_in_b, gjk_result.lambda).v, 0.0f));
                    const Mat4f a_to_world_at_contact = mat4f::multiply(target_transform, a_at_contact);

                    shapes::get_shape_supporting_face(hit.face_a, shape_a, sub_shape_id_a.value, normal_in_a, a_to_world_at_contact, scale_a);

                    // face b: the triangle vertices, already scaled and in b's local space, winding swapped
                    // when the scale turns the mesh inside out; to world by rotation + translation only
                    const bool inside_out = scale_sign < 0.0f;
                    set_face_triangle(hit.face_b, a, inside_out ? c : b, inside_out ? b : c);
                    face::transform_with_mat4_rotation_translation(hit.face_b, target_transform);
                } else {
                    face::clear(hit.face_a);
                    face::clear(hit.face_b);
                }

                cast_shape::add_hit(collector, hit);
            }
        } else {
            // internal node: entry fractions of both children's bounds expanded by the cast shape's
            // half-extents, along the ray from the swept aabb centre
            const int32_t left = bvh::node_left(node);
            const int32_t right = bvh::node_right(nodes, node);

            const float left_dist = cast_utils::ray_distance_to_box3(walk_ray, aabox::expand_by_extents(aabox::load(nodes[left].bounds), half_extents));
            const float right_dist = cast_utils::ray_distance_to_box3(walk_ray, aabox::expand_by_extents(aabox::load(nodes[right].bounds), half_extents));

            // floored at FLT_MIN so nodes the cast starts inside survive a hit at fraction 0
            const float positive_early_out = scalar::max(scalar::FLOAT_MIN_NORMAL, collector.early_out_fraction);

            // push farther child first (so closer child is on top of stack), lifo traversal
            if (left_dist <= right_dist) {
                if (right_dist < positive_early_out) push_node(stack_nodes, stack_dist, stack_size, right, right_dist);
                if (left_dist < positive_early_out) push_node(stack_nodes, stack_dist, stack_size, left, left_dist);
            } else {
                if (left_dist < positive_early_out) push_node(stack_nodes, stack_dist, stack_size, left, left_dist);
                if (right_dist < positive_early_out) push_node(stack_nodes, stack_dist, stack_size, right, right_dist);
            }
        }
    }
}

/* collide shape */

static support::Support collide_support_a;
// a separate support for the epa-inflated a, so `collide_support_a` stays EXCLUDE across the triangle loop
static support::Support collide_support_a_with_radius;
static support::Support collide_triangle_support;
// zeroed is an empty depth; epa's hull initialisation resets the builder before use
static penetration::PenetrationDepth collide_penetration_depth;

void collide_convex_vs_triangle_mesh(collide_shape::CollideShapeCollector &collector, const collide_shape::CollideShapeSettings &settings,
                                     const Shape *shape_a, const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                                     const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b) {
    const TriangleMeshData &data = *shapes::as<TriangleMeshShape>(shape_b).data;
    const bvh::Node *nodes = data.nodes;

    if (data.node_count == 0) return;

    const Vec3f scale_a = vec3f::load(transform_a.scale);
    const Vec3f scale_b = vec3f::load(transform_b.scale);

    // transform b (mesh) into a's (convex) local space for triangle tests
    const Quatf inverse_quat_a = quatf::conjugate(quatf::load(transform_a.quaternion));
    const Quatf transform_2_to_1_quat = quatf::multiply(inverse_quat_a, quatf::load(transform_b.quaternion));
    const Vec3f vector_ab = vec3f::subtract(vec3f::load(transform_b.position), vec3f::load(transform_a.position));
    const Vec3f transform_2_to_1_pos = vec3f::transform_quat(vector_ab, inverse_quat_a);

    // transform a into b's space for the bvh query
    const Quatf inverse_quat_b = quatf::conjugate(quatf::load(transform_b.quaternion));
    const Quatf quat_a_in_b = quatf::multiply(inverse_quat_b, quatf::load(transform_a.quaternion));
    const Vec3f position_difference = vec3f::subtract(vec3f::load(transform_a.position), vec3f::load(transform_b.position));
    const Vec3f pos_a_in_b = vec3f::transform_quat(position_difference, inverse_quat_b);

    const Vec3f max_separation_extents = vec3f::splat(settings.max_separation_distance);

    // shape a's bounds in its own local space
    const AABox bounds_of_1 = aabox::expand_by_extents(aabox::scale(aabox::load(shape_a->aabb), scale_a), max_separation_extents);

    // shape a's bounds in shape b's space for bvh culling
    const Mat4f aabb_matrix = mat4f::from_rotation_translation_scale(quat_a_in_b, pos_a_in_b, scale_a);
    AABox bounds_of_1_in_space_of_2 = aabox::expand_by_extents(aabox::transform_mat4(aabox::load(shape_a->aabb), aabb_matrix), max_separation_extents);
    // the bvh is built in the mesh's unscaled local space
    bounds_of_1_in_space_of_2 = divide_box_by_scale(bounds_of_1_in_space_of_2, scale_b);

    // a to world (rotation + translation only, no scale)
    const Mat4f a_to_world = mat4f::from_rotation_translation(quatf::load(transform_a.quaternion), vec3f::load(transform_a.position));

    // b to a with the mesh scale folded in (scale first, then rotation and translation), so each
    // vertex is one affine transform
    const Mat4f b_to_a = mat4f::from_rotation_translation_scale(transform_2_to_1_quat, transform_2_to_1_pos, scale_b);

    // determine if mesh is inside-out
    const float scale_sign = vec3f::is_scale_inside_out(scale_b) ? -1.0f : 1.0f;

    // both supports for shape a are filled once per pair, the first time a triangle needs one: the
    // shrunk core on the first triangle to reach gjk, the inflated one on the first to reach epa.
    // a pair whose bvh walk reaches no leaf fills neither.
    support::Support &support_a = collide_support_a;
    bool support_a_filled = false;
    support::Support &support_a_with_radius = collide_support_a_with_radius;
    bool support_a_with_radius_filled = false;
    penetration::PenetrationDepth &penetration_depth = collide_penetration_depth;

    // query shape centre in b's space for the centre-to-centre distance ordering
    const Vec3f query_center = aabox::center(bounds_of_1_in_space_of_2);

    int32_t stack_nodes[STACK_SIZE];
    int32_t stack_size = 0;
    push_node(stack_nodes, stack_size, 0);  // root always visited

    while (stack_size > 0) {
        const int32_t node = stack_nodes[--stack_size];

        // skip if node bounds don't intersect query
        if (!bvh::node_intersects_box(nodes, node, bounds_of_1_in_space_of_2)) continue;

        if (bvh::node_is_leaf(nodes, node)) {
            // leaf node: check triangles
            const uint32_t tri_start = triangle_mesh_bvh::node_tri_start(nodes, node);
            const uint32_t tri_count = triangle_mesh_bvh::node_tri_count(nodes, node);
            for (uint32_t i = 0; i < tri_count; i++) {
                const uint32_t triangle_index = tri_start + i;

                // scaled and transformed into shape a's local space in one affine multiply each
                Vec3f a, b, c;
                triangle_mesh_data::get_triangle_vertices(a, b, c, data, triangle_index);
                a = vec3f::transform_mat4(a, b_to_a);
                b = vec3f::transform_mat4(b, b_to_a);
                c = vec3f::transform_mat4(c, b_to_a);

                // early out: if triangle aabb doesn't overlap shape aabb, skip this triangle
                if (!aabox::intersects(aabox::from_triangle(a, b, c), bounds_of_1)) continue;

                // triangle normal in a's local space
                Vec3f edge_a = vec3f::subtract(b, a);
                Vec3f edge_b = vec3f::subtract(c, a);
                Vec3f normal = vec3f::cross(edge_a, edge_b);
                normal = vec3f::scale(normal, scale_sign);

                // back-face check: the triangle normal and its first vertex point the same way from the
                // origin (shape a is at the origin in its local space)
                const bool back_facing = vec3f::dot(normal, a) > 0.0f;
                if (!settings.collide_with_backfaces && back_facing) continue;

                set_triangle_support(collide_triangle_support, a, b, c);

                // run gjk with negative triangle normal as initial penetration axis
                // (likely that shape a is in front of triangle b)
                Vec3f penetration_axis = vec3f::negate(normal);

                // ensure non-zero penetration axis
                if (vec3f::squared_length(penetration_axis) < 1e-10f) {
                    penetration_axis = vec3f::make(1.0f, 0.0f, 0.0f);
                } else {
                    penetration_axis = vec3f::normalize(penetration_axis);
                }

                if (!support_a_filled) {
                    shapes::set_shape_support(support_a, shape_a, support::EXCLUDE_CONVEX_RADIUS, scale_a);
                    support_a_filled = true;
                }

                // perform gjk step with inflated shape (convex radius + max separation distance)
                float max_separation_distance = settings.max_separation_distance;
                simplex::Simplex simplex;
                simplex.size = 0;
                penetration::penetration_depth_step_gjk(penetration_depth, simplex, support_a, collide_triangle_support,
                                                        support_a.convex_radius + max_separation_distance,
                                                        0.0f,  // triangle has no convex radius
                                                        penetration_axis, settings.collision_tolerance);

                // check result of collision detection
                if (penetration_depth.status == penetration::NOT_COLLIDING) continue;

                if (penetration_depth.status == penetration::INDETERMINATE) {
                    // need to run expensive epa algorithm
                    // clamp max separation distance to avoid excessive inflation
                    max_separation_distance = scalar::min(max_separation_distance, 1.0f);

                    if (!support_a_with_radius_filled) {
                        shapes::set_shape_support(support_a_with_radius, shape_a, support::INCLUDE_CONVEX_RADIUS, scale_a);
                        support_a_with_radius_filled = true;
                    }
                    support_a_with_radius.add_radius = max_separation_distance;

                    // perform epa step
                    if (!penetration::penetration_depth_step_epa(penetration_depth, support_a_with_radius, collide_triangle_support,
                                                                 settings.penetration_tolerance, simplex)) {
                        continue;
                    }
                }

                // calculate penetration depth (subtract the inflation from the distance)
                const float penetration = vec3f::distance(penetration_depth.point_a, penetration_depth.point_b) - max_separation_distance;

                // check if penetration exceeds early-out threshold
                if (-penetration >= collector.early_out_fraction) continue;

                // correct point a by moving it back along penetration axis to account for the max separation distance
                const float penetration_axis_len = vec3f::length(penetration_depth.penetration_axis);
                if (penetration_axis_len > 0.0f) {
                    const float correction = max_separation_distance / penetration_axis_len;
                    penetration_depth.point_a = vec3f::scale_and_add(penetration_depth.point_a, penetration_depth.penetration_axis, -correction);
                }

                // active edge detection - correct normal if hitting inactive edge
                const uint32_t triangle_active_edges = triangle_mesh_data::get_active_edges(data, triangle_index);
                if (settings.collide_only_with_active_edges && triangle_active_edges != 0b111) {
                    // transform movement direction from world space to shape a's local space
                    Vec3f active_edge_movement_direction = vec3f::transform_quat(settings.active_edge_movement_direction, inverse_quat_a);

                    // the triangle normal is flipped as the axis points towards the triangle instead of away
                    const Vec3f triangle_normal_for_fix = back_facing ? normal : vec3f::negate(normal);

                    // apply active edge correction (all parameters in shape a's local space)
                    const Vec3f corrected_axis =
                        active_edges::fix_normal(a, b, c, triangle_normal_for_fix, triangle_active_edges, penetration_depth.point_b,
                                                 penetration_depth.penetration_axis, active_edge_movement_direction);
                    penetration_depth.penetration_axis = corrected_axis;
                }

                collide_shape::CollideShapeHit hit = collide_shape::create_hit();

                // transform results to world space
                hit.point_a = vec3f::transform_mat4(penetration_depth.point_a, a_to_world);
                hit.point_b = vec3f::transform_mat4(penetration_depth.point_b, a_to_world);
                hit.penetration_axis = mat4f::multiply3x3_vec(a_to_world, penetration_depth.penetration_axis);

                sub_shape::SubShapeIdBuilder triangle_sub_shape_id;
                sub_shape::push_index(triangle_sub_shape_id, sub_shape_id_b, triangle_index, data.triangle_count);

                hit.penetration = penetration;
                hit.sub_shape_id_a = sub_shape_id_a.value;
                hit.sub_shape_id_b = triangle_sub_shape_id.value;
                hit.body_id_b = collector.body_id_b;

                // collect faces if requested
                if (settings.collect_faces) {
                    // direction for shape a: opposite of penetration axis (local space)
                    const Vec3f face_direction_a = vec3f::negate(penetration_depth.penetration_axis);
                    shapes::get_shape_supporting_face(hit.face_a, shape_a, sub_shape_id_a.value, face_direction_a, a_to_world, scale_a);

                    // shape b face: the triangle in shape a's local space, already scaled by scale_b, winding
                    // swapped when the scale turns the mesh inside out; to world by rotation + translation only
                    const bool inside_out = scale_sign < 0.0f;
                    set_face_triangle(hit.face_b, a, inside_out ? c : b, inside_out ? b : c);
                    face::transform_with_mat4_rotation_translation(hit.face_b, a_to_world);
                }

                collide_shape::add_hit(collector, hit);
            }
        } else {
            // internal node: push the child whose centre is farther from the query centre first
            const int32_t left = bvh::node_left(node);
            const int32_t right = bvh::node_right(nodes, node);

            const float left_dist = vec3f::squared_distance(query_center, bvh::node_get_center(nodes, left));
            const float right_dist = vec3f::squared_distance(query_center, bvh::node_get_center(nodes, right));

            if (left_dist <= right_dist) {
                push_node(stack_nodes, stack_size, right);
                push_node(stack_nodes, stack_size, left);
            } else {
                push_node(stack_nodes, stack_size, left);
                push_node(stack_nodes, stack_size, right);
            }
        }
    }
}

/* collide sphere vs triangle mesh */

void collide_sphere_vs_triangle_mesh(collide_shape::CollideShapeCollector &collector, const collide_shape::CollideShapeSettings &settings,
                                     const Shape *shape_a, const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                                     const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b) {
    const TriangleMeshData &data = *shapes::as<TriangleMeshShape>(shape_b).data;
    const bvh::Node *nodes = data.nodes;

    if (data.node_count == 0) return;

    // scaled sphere radius (assuming uniform scale)
    const float sphere_radius = scalar::abs(transform_a.scale.x) * shapes::as<sphere::SphereShape>(shape_a).radius;
    const float expanded_radius = sphere_radius + settings.max_separation_distance;
    const float max_separation_sq = expanded_radius * expanded_radius;

    const Vec3f scale_b = vec3f::load(transform_b.scale);

    // the mesh's world transform, held for the whole triangle walk; its inverse rotation is the transpose
    const Mat4f mesh_to_world = mat4f::from_rotation_translation(quatf::load(transform_b.quaternion), vec3f::load(transform_b.position));

    // sphere centre into mesh-local
    const Vec3f center_offset = vec3f::subtract(vec3f::load(transform_a.position), vec3f::load(transform_b.position));
    const Vec3f sphere_center_in_mesh = mat4f::multiply3x3_transposed_vec(mesh_to_world, center_offset);

    // detect inside-out scaling
    const float scale_sign = vec3f::is_scale_inside_out(scale_b) ? -1.0f : 1.0f;

    // sphere aabb for bvh culling, in the unscaled space the bvh is built in
    const Vec3f unscaled_center = vec3f::divide(sphere_center_in_mesh, scale_b);
    const Vec3f unscaled_radius = vec3f::divide(vec3f::splat(expanded_radius), vec3f::abs(scale_b));
    const AABox sphere_bounds = {vec3f::subtract(unscaled_center, unscaled_radius), vec3f::add(unscaled_center, unscaled_radius)};

    triangle::ClosestPointOnTriangleResult closest_point = triangle::create_closest_point_on_triangle_result();

    int32_t stack_nodes[STACK_SIZE];
    int32_t stack_size = 0;
    push_node(stack_nodes, stack_size, 0);  // root

    while (stack_size > 0) {
        const int32_t node = stack_nodes[--stack_size];

        // skip if node bounds don't intersect sphere
        if (!bvh::node_intersects_box(nodes, node, sphere_bounds)) continue;

        if (bvh::node_is_leaf(nodes, node)) {
            // leaf node: test triangles
            const uint32_t tri_start = triangle_mesh_bvh::node_tri_start(nodes, node);
            const uint32_t tri_count = triangle_mesh_bvh::node_tri_count(nodes, node);

            for (uint32_t i = 0; i < tri_count; i++) {
                const uint32_t triangle_index = tri_start + i;

                // scaled triangle vertices
                Vec3f sv0, sv1, sv2;
                triangle_mesh_data::get_triangle_vertices(sv0, sv1, sv2, data, triangle_index);
                sv0 = vec3f::multiply(sv0, scale_b);
                sv1 = vec3f::multiply(sv1, scale_b);
                sv2 = vec3f::multiply(sv2, scale_b);

                // relative to sphere center (sphere at origin)
                Vec3f rv0 = vec3f::subtract(sv0, sphere_center_in_mesh);
                Vec3f rv1 = vec3f::subtract(sv1, sphere_center_in_mesh);
                Vec3f rv2 = vec3f::subtract(sv2, sphere_center_in_mesh);

                // triangle normal
                Vec3f edge1 = vec3f::subtract(rv1, rv0);
                Vec3f edge2 = vec3f::subtract(rv2, rv0);
                Vec3f triangle_normal = vec3f::cross(edge1, edge2);
                triangle_normal = vec3f::scale(triangle_normal, scale_sign);

                // backface check
                const bool back_facing = vec3f::dot(triangle_normal, rv0) > 0.0f;
                if (!settings.collide_with_backfaces && back_facing) continue;

                // closest point on triangle to sphere center (origin)
                triangle::get_closest_point_on_triangle(closest_point, rv0, rv1, rv2);
                const Vec3f point2 = closest_point.point;
                const float point2_len_sq = closest_point.distance_sq;

                // early out if too far
                if (point2_len_sq > max_separation_sq) continue;

                const float penetration_depth = sphere_radius - scalar::sqrt(point2_len_sq);

                // early out if worse than current best
                if (-penetration_depth >= collector.early_out_fraction) continue;

                // penetration axis (away from sphere center)
                Vec3f penetration_axis;
                if (point2_len_sq > 0.0f) {
                    penetration_axis = vec3f::normalize(point2);
                } else {
                    // sphere center exactly on triangle - use arbitrary axis
                    penetration_axis = vec3f::make(0.0f, 1.0f, 0.0f);
                }

                // point on sphere surface
                const Vec3f point1 = vec3f::scale(penetration_axis, sphere_radius);

                // active edge handling
                const uint32_t feature = closest_point.feature;
                const uint32_t triangle_active_edges = triangle_mesh_data::get_active_edges(data, triangle_index);
                if (settings.collide_only_with_active_edges && feature != 0b111 && triangle_active_edges != 0b111) {
                    // check if the feature we hit requires an active edge that isn't active
                    if ((triangle_active_edges & sphere_triangle::FEATURE_TO_ACTIVE_EDGES[feature]) == 0) {
                        // movement direction into mesh space
                        Vec3f movement = mat4f::multiply3x3_transposed_vec(mesh_to_world, settings.active_edge_movement_direction);

                        const Vec3f new_penetration_axis = back_facing ? triangle_normal : vec3f::negate(triangle_normal);
                        const float new_penetration_axis_len = vec3f::length(new_penetration_axis);

                        // if penetration axis affects movement less than the triangle normal, use the normal
                        if (vec3f::dot(movement, penetration_axis) * new_penetration_axis_len >= vec3f::dot(movement, new_penetration_axis)) {
                            penetration_axis = new_penetration_axis;
                        }
                    }
                }

                // results into world space
                collide_shape::CollideShapeHit hit = collide_shape::create_hit();
                Vec3f local_point = vec3f::add(sphere_center_in_mesh, point1);
                hit.point_a = vec3f::transform_mat4(local_point, mesh_to_world);
                local_point = vec3f::add(sphere_center_in_mesh, point2);
                hit.point_b = vec3f::transform_mat4(local_point, mesh_to_world);
                hit.penetration_axis = mat4f::multiply3x3_vec(mesh_to_world, penetration_axis);

                sub_shape::SubShapeIdBuilder triangle_sub_shape_id;
                sub_shape::push_index(triangle_sub_shape_id, sub_shape_id_b, triangle_index, data.triangle_count);

                hit.penetration = penetration_depth;
                hit.sub_shape_id_a = sub_shape_id_a.value;
                hit.sub_shape_id_b = triangle_sub_shape_id.value;
                hit.body_id_b = collector.body_id_b;

                // collect faces if requested
                if (settings.collect_faces) {
                    // sphere has no supporting face (point contact)
                    face::clear(hit.face_a);

                    // the scaled triangle in world space, winding swapped when the scale turns the mesh inside out
                    const bool inside_out = scale_sign < 0.0f;
                    set_face_triangle(hit.face_b, sv0, inside_out ? sv2 : sv1, inside_out ? sv1 : sv2);
                    face::transform_with_mat4_rotation_translation(hit.face_b, mesh_to_world);
                }

                collide_shape::add_hit(collector, hit);
            }
        } else {
            // internal node: push the child whose centre is farther from the sphere centre first
            const int32_t left = bvh::node_left(node);
            const int32_t right = bvh::node_right(nodes, node);

            const float left_dist = vec3f::squared_distance(sphere_center_in_mesh, bvh::node_get_center(nodes, left));
            const float right_dist = vec3f::squared_distance(sphere_center_in_mesh, bvh::node_get_center(nodes, right));

            if (left_dist <= right_dist) {
                push_node(stack_nodes, stack_size, right);
                push_node(stack_nodes, stack_size, left);
            } else {
                push_node(stack_nodes, stack_size, left);
                push_node(stack_nodes, stack_size, right);
            }
        }
    }
}

/* cast sphere vs triangle mesh */

/** what a sphere cast's hits share across one triangle walk */
struct CastSphereContext {
    cast_shape::CastShapeCollector *collector;
    const cast_shape::CastShapeSettings *settings;
    const Shape *sphere_shape;
    const TriangleMeshData *data;
    sub_shape::SubShapeId sub_shape_id_a;
    const sub_shape::SubShapeIdBuilder *sub_shape_id_b;
    /** the sphere's start in mesh-local space */
    Vec3f start;
    Mat4f mesh_to_world;
};

/**
 * adds a cast hit with active edge detection for sphere vs triangle mesh; the triangle, points and normal are relative to
 * the start, mesh-local
 */
static void cast_sphere_vs_triangle_mesh_add_hit(const CastSphereContext &context, Vec3f v0, Vec3f v1, Vec3f v2,
                                                 bool back_facing, Vec3f triangle_normal, uint32_t triangle_active_edges,
                                                 uint32_t triangle_index, float fraction, Vec3f contact_point_a,
                                                 Vec3f contact_point_b, Vec3f contact_normal) {
    const cast_shape::CastShapeSettings &settings = *context.settings;
    Vec3f final_contact_normal = contact_normal;

    if (settings.collide_only_with_active_edges && triangle_active_edges != 0b111) {
        // movement direction into mesh space
        const Vec3f movement = mat4f::multiply3x3_transposed_vec(context.mesh_to_world, settings.active_edge_movement_direction);

        // the triangle normal is flipped as the contact normal points towards the triangle instead of away
        const Vec3f triangle_normal_for_fix = back_facing ? triangle_normal : vec3f::negate(triangle_normal);

        final_contact_normal = active_edges::fix_normal(v0, v1, v2, triangle_normal_for_fix, triangle_active_edges, contact_point_b,
                                                        contact_normal, movement);
    }

    // transform to world space
    cast_shape::CastShapeHit hit = cast_shape::create_hit();
    Vec3f local_point = vec3f::add(context.start, contact_point_a);
    hit.point_a = vec3f::transform_mat4(local_point, context.mesh_to_world);
    local_point = vec3f::add(context.start, contact_point_b);
    hit.point_b = vec3f::transform_mat4(local_point, context.mesh_to_world);

    // the contact normal points from the sphere towards the triangle, a to b: the penetration axis, and the normal its
    // negated unit as every other cast reports
    hit.penetration_axis = mat4f::multiply3x3_vec(context.mesh_to_world, final_contact_normal);
    hit.normal = vec3f::normalize(hit.penetration_axis);
    hit.normal = vec3f::negate(hit.normal);

    sub_shape::SubShapeIdBuilder triangle_sub_shape_id;
    sub_shape::push_index(triangle_sub_shape_id, *context.sub_shape_id_b, triangle_index, context.data->triangle_count);

    hit.status = cast_shape::COLLIDING;
    hit.fraction = fraction;
    hit.penetration_depth = vec3f::distance(hit.point_a, hit.point_b);
    hit.sub_shape_id_a = context.sub_shape_id_a;
    hit.sub_shape_id_b = triangle_sub_shape_id.value;
    hit.body_id_b = context.collector->body_id_b;
    face::clear(hit.face_a);  // sphere has no face
    face::clear(hit.face_b);  // don't collect faces for swept tests

    cast_shape::add_hit(*context.collector, hit);
}

void cast_sphere_vs_triangle_mesh(cast_shape::CastShapeCollector &collector, const cast_shape::CastShapeSettings &settings,
                                  const Shape *shape_a, const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                                  Vec3f displacement_a, const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b,
                                  const Transformf &transform_b) {
    const TriangleMeshData &data = *shapes::as<TriangleMeshShape>(shape_b).data;
    const bvh::Node *nodes = data.nodes;

    if (data.node_count == 0) return;

    // scaled sphere radius (assuming uniform scale)
    const float sphere_radius = scalar::abs(transform_a.scale.x) * shapes::as<sphere::SphereShape>(shape_a).radius + settings.extra_convex_radius;

    const Vec3f scale_b = vec3f::load(transform_b.scale);

    CastSphereContext context;
    context.collector = &collector;
    context.settings = &settings;
    context.sphere_shape = shape_a;
    context.data = &data;
    context.sub_shape_id_a = sub_shape_id_a.value;
    context.sub_shape_id_b = &sub_shape_id_b;

    // the mesh's world transform, held for the whole cast; its inverse rotation is the transpose
    context.mesh_to_world = mat4f::from_rotation_translation(quatf::load(transform_b.quaternion), vec3f::load(transform_b.position));

    // sphere start and displacement into mesh-local
    const Vec3f start_offset = vec3f::subtract(vec3f::load(transform_a.position), vec3f::load(transform_b.position));
    context.start = mat4f::multiply3x3_transposed_vec(context.mesh_to_world, start_offset);
    const Vec3f start = context.start;

    const Vec3f direction = mat4f::multiply3x3_transposed_vec(context.mesh_to_world, displacement_a);

    // detect inside-out scaling
    const float scale_sign = vec3f::is_scale_inside_out(scale_b) ? -1.0f : 1.0f;

    // ordered front-to-back traversal (matches cast_convex_vs_triangle_mesh): ray from the sphere
    // start along the normalized displacement, in the unscaled space the bvh is built in, node
    // bounds expanded by the sphere radius; the sweep itself stays in scaled mesh space below
    cast_ray::Rayf walk_ray;
    walk_ray.origin = vec3f::divide(start, scale_b);
    const Vec3f walk_displacement = vec3f::divide(direction, scale_b);
    walk_ray.length = vec3f::length(walk_displacement);
    const float inv_ray_length = walk_ray.length > 1e-10f ? 1.0f / walk_ray.length : 0.0f;
    walk_ray.direction = vec3f::scale(walk_displacement, inv_ray_length);
    const Vec3f walk_radius = vec3f::divide(vec3f::splat(sphere_radius), vec3f::abs(scale_b));

    triangle::ClosestPointOnTriangleResult closest_point = triangle::create_closest_point_on_triangle_result();

    int32_t stack_nodes[STACK_SIZE];
    float stack_dist[STACK_SIZE];
    int32_t stack_size = 0;
    push_node(stack_nodes, stack_dist, stack_size, 0, 0.0f);  // root always visited

    while (stack_size > 0) {
        stack_size--;
        const int32_t node = stack_nodes[stack_size];
        const float node_distance = stack_dist[stack_size];

        // early out: if fraction to this node >= closest hit, skip it
        if (node_distance >= scalar::max(scalar::FLOAT_MIN_NORMAL, collector.early_out_fraction)) continue;

        if (bvh::node_is_leaf(nodes, node)) {
            // leaf node: test triangles
            const uint32_t tri_start = triangle_mesh_bvh::node_tri_start(nodes, node);
            const uint32_t tri_count = triangle_mesh_bvh::node_tri_count(nodes, node);

            for (uint32_t i = 0; i < tri_count; i++) {
                if (collector.early_out_fraction <= -collector::UNBOUNDED) return;

                const uint32_t triangle_index = tri_start + i;

                // scaled triangle vertices, relative to the sweep start
                Vec3f rv0, rv1, rv2;
                triangle_mesh_data::get_triangle_vertices(rv0, rv1, rv2, data, triangle_index);
                rv0 = vec3f::multiply(rv0, scale_b);
                rv1 = vec3f::multiply(rv1, scale_b);
                rv2 = vec3f::multiply(rv2, scale_b);
                rv0 = vec3f::subtract(rv0, start);
                rv1 = vec3f::subtract(rv1, start);
                rv2 = vec3f::subtract(rv2, start);

                // triangle normal
                Vec3f edge1 = vec3f::subtract(rv1, rv0);
                Vec3f edge2 = vec3f::subtract(rv2, rv0);
                Vec3f triangle_normal = vec3f::cross(edge1, edge2);
                const float triangle_normal_len = vec3f::length(triangle_normal);

                if (triangle_normal_len == 0.0f) continue;  // degenerate triangle

                triangle_normal = vec3f::scale(triangle_normal, scale_sign / triangle_normal_len);

                // backface check
                const float normal_dot_direction = vec3f::dot(triangle_normal, direction);
                const bool back_facing = normal_dot_direction > 0.0f;
                if (!settings.collide_with_backfaces_triangles && back_facing) continue;

                const uint32_t triangle_active_edges = triangle_mesh_data::get_active_edges(data, triangle_index);

                // case 1: already penetrating at t=0
                if (scalar::abs(vec3f::dot(rv0, triangle_normal)) <= sphere_radius) {
                    triangle::get_closest_point_on_triangle(closest_point, rv0, rv1, rv2);
                    const Vec3f q = closest_point.point;
                    const float q_len_sq = closest_point.distance_sq;

                    if (q_len_sq <= sphere_radius * sphere_radius) {
                        const float q_len = scalar::sqrt(q_len_sq);
                        const float penetration_depth = sphere_radius - q_len;

                        if (-penetration_depth >= collector.early_out_fraction) continue;

                        Vec3f contact_normal = vec3f::make(0.0f, 1.0f, 0.0f);
                        if (q_len > 0.0f) (contact_normal = vec3f::scale(q, 1.0f / q_len));
                        const Vec3f contact_point_a = vec3f::scale_and_add(q, contact_normal, penetration_depth);
                        const Vec3f contact_point_b = q;

                        cast_sphere_vs_triangle_mesh_add_hit(context, rv0, rv1, rv2, back_facing, triangle_normal, triangle_active_edges, triangle_index, 0.0f,
                                                             contact_point_a, contact_point_b, contact_normal);
                        continue;
                    }
                }

                // case 2: plane intersection test
                const float abs_normal_dot_direction = scalar::abs(normal_dot_direction);
                if (abs_normal_dot_direction > 1e-6f) {
                    // the point on the sphere that hits the plane first
                    const Vec3f d = vec3f::scale(triangle_normal, scalar::sign(normal_dot_direction) * sphere_radius);

                    const Vec3f plane_intersection_temp = vec3f::subtract(rv0, d);
                    const float plane_intersection = vec3f::dot(plane_intersection_temp, triangle_normal) / normal_dot_direction;

                    // check if hit is within sweep range
                    if (plane_intersection * abs_normal_dot_direction >= -sphere_radius && plane_intersection < collector.early_out_fraction &&
                        plane_intersection >= 0.0f) {
                        // contact point on plane
                        Vec3f p = vec3f::scale_and_add(d, direction, plane_intersection);

                        // check if this is an interior point
                        const Vec3f v0_rel = vec3f::subtract(rv0, p);
                        const Vec3f v1_rel = vec3f::subtract(rv1, p);
                        const Vec3f v2_rel = vec3f::subtract(rv2, p);
                        closest_points::BarycentricCoordinatesResult barycentric = closest_points::create_barycentric_coordinates_result();
                        closest_points::compute_barycentric_coordinates_3d(barycentric, v0_rel, v1_rel, v2_rel, 1e-10f);
                        if (barycentric.is_valid && barycentric.u >= 0.0f && barycentric.v >= 0.0f && barycentric.w >= 0.0f) {
                            // interior hit, all edges active
                            const Vec3f contact_normal = back_facing ? triangle_normal : vec3f::negate(triangle_normal);
                            cast_sphere_vs_triangle_mesh_add_hit(context, rv0, rv1, rv2, back_facing, triangle_normal, 0b111, triangle_index,
                                                                 plane_intersection, p, p, contact_normal);
                            continue;
                        }
                    }
                }

                // case 3: edge and vertex tests
                float fraction = scalar::INF;

                // test 3 edges (swept sphere vs cylinder)
                fraction = scalar::min(fraction, sphere_triangle::ray_cylinder(direction, rv0, rv1, sphere_radius));
                fraction = scalar::min(fraction, sphere_triangle::ray_cylinder(direction, rv1, rv2, sphere_radius));
                fraction = scalar::min(fraction, sphere_triangle::ray_cylinder(direction, rv2, rv0, sphere_radius));

                // test 3 vertices (swept sphere vs sphere)
                fraction = scalar::min(fraction, sphere_triangle::ray_sphere_from_origin(direction, rv0, sphere_radius));
                fraction = scalar::min(fraction, sphere_triangle::ray_sphere_from_origin(direction, rv1, sphere_radius));
                fraction = scalar::min(fraction, sphere_triangle::ray_sphere_from_origin(direction, rv2, sphere_radius));

                // check if we have a valid hit
                if (fraction >= 0.0f && fraction < collector.early_out_fraction) {
                    // sphere center at collision point
                    Vec3f p = vec3f::scale(direction, fraction);

                    // closest point on triangle
                    const Vec3f v0_relative_to_hit = vec3f::subtract(rv0, p);
                    const Vec3f v1_relative_to_hit = vec3f::subtract(rv1, p);
                    const Vec3f v2_relative_to_hit = vec3f::subtract(rv2, p);
                    triangle::get_closest_point_on_triangle(closest_point, v0_relative_to_hit, v1_relative_to_hit, v2_relative_to_hit);

                    // the distance between p and the triangle should be the radius, but floating point accuracy can put p on the
                    // plane with q zero, so fall back to the normal the triangle is hit along
                    const Vec3f q = closest_point.point;
                    Vec3f contact_normal, contact_point_ab;
                    const float q_len_sq = vec3f::squared_length(q);
                    if (q_len_sq > scalar::FLOAT_MIN_NORMAL) (contact_normal = vec3f::scale(q, 1.0f / scalar::sqrt(q_len_sq)));
                    else if (back_facing) contact_normal = triangle_normal;
                    else (contact_normal = vec3f::negate(triangle_normal));
                    contact_point_ab = vec3f::add(p, q);

                    cast_sphere_vs_triangle_mesh_add_hit(context, rv0, rv1, rv2, back_facing, triangle_normal, triangle_active_edges, triangle_index, fraction,
                                                         contact_point_ab, contact_point_ab, contact_normal);
                }
            }
        } else {
            // internal node: distance-order both children along the cast ray, node bounds expanded by the sphere radius
            const int32_t left = bvh::node_left(node);
            const int32_t right = bvh::node_right(nodes, node);

            const float left_dist = cast_utils::ray_distance_to_box3(walk_ray, aabox::expand_by_extents(aabox::load(nodes[left].bounds), walk_radius));
            const float right_dist = cast_utils::ray_distance_to_box3(walk_ray, aabox::expand_by_extents(aabox::load(nodes[right].bounds), walk_radius));

            // floored at FLT_MIN so nodes the cast starts inside survive a hit at fraction 0
            const float positive_early_out = scalar::max(scalar::FLOAT_MIN_NORMAL, collector.early_out_fraction);

            // push farther child first (so closer child is on top of stack), lifo traversal
            if (left_dist <= right_dist) {
                if (right_dist < positive_early_out) push_node(stack_nodes, stack_dist, stack_size, right, right_dist);
                if (left_dist < positive_early_out) push_node(stack_nodes, stack_dist, stack_size, left, left_dist);
            } else {
                if (left_dist < positive_early_out) push_node(stack_nodes, stack_dist, stack_size, left, left_dist);
                if (right_dist < positive_early_out) push_node(stack_nodes, stack_dist, stack_size, right, right_dist);
            }
        }
    }
}

shapes::ShapeDef def() {
    shapes::ShapeDef shape_def = {};
    shape_def.type = shapes::SHAPE_TRIANGLE_MESH;
    shape_def.compute_mass_properties = compute_mass_properties;
    shape_def.get_surface_normal = get_surface_normal;
    shape_def.get_supporting_face = get_supporting_face;
    shape_def.get_material = get_material;
    shape_def.cast_ray = cast_ray_vs_triangle_mesh;
    shape_def.collide_point = collide_point_vs_triangle_mesh;
    return shapes::define_shape(shape_def);
}

}  // namespace triangle_mesh

TUMBLE_NAMESPACE_END
