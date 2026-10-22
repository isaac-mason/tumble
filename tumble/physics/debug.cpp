#include "tumble/physics/debug.h"

#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/bodies.h"
#include "tumble/physics/body/motion_type.h"
#include "tumble/physics/character/kcc.h"
#include "tumble/physics/constraints/cone_constraint.h"
#include "tumble/physics/constraints/constraints.h"
#include "tumble/physics/constraints/distance_constraint.h"
#include "tumble/physics/constraints/fixed_constraint.h"
#include "tumble/physics/constraints/hinge_constraint.h"
#include "tumble/physics/constraints/point_constraint.h"
#include "tumble/physics/constraints/six_dof_constraint.h"
#include "tumble/physics/constraints/slider_constraint.h"
#include "tumble/physics/constraints/swing_twist_constraint.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/capsule.h"
#include "tumble/physics/shapes/compound.h"
#include "tumble/physics/shapes/convex_hull.h"
#include "tumble/physics/shapes/cylinder.h"
#include "tumble/physics/shapes/offset_center_of_mass.h"
#include "tumble/physics/shapes/plane.h"
#include "tumble/physics/shapes/scaled.h"
#include "tumble/physics/shapes/sphere.h"
#include "tumble/physics/shapes/transformed.h"
#include "tumble/physics/shapes/triangle_mesh.h"
#include "tumble/physics/shapes/utils/triangle_mesh_data.h"
#include "tumble/util/object.h"

TUMBLE_NAMESPACE_BEGIN

namespace debug {

using box::BoxShape;
using capsule::CapsuleShape;
using convex_hull::ConvexHullShape;
using cylinder::CylinderShape;
using offset_center_of_mass::OffsetCenterOfMassShape;
using plane::PlaneShape;
using rigid_body::RigidBody;
using scaled::ScaledShape;
using shapes::Shape;
using sphere::SphereShape;
using transformed::TransformedShape;
using triangle_mesh::TriangleMeshShape;
using tumble::bodies::Bodies;
using tumble::contact_constraints::ContactConstraint;
using tumble::contact_constraints::ContactConstraints;
using tumble::contact_constraints::WorldContactPoint;

DebugBuffers *create() { return util::create<DebugBuffers>(); }

void release(DebugBuffers *buffers) {
    buffers->lines.release();
    buffers->triangles.release();
    util::destroy(buffers);
}

void clear(DebugBuffers &buffers) {
    buffers.lines.length = 0;
    buffers.line_count = 0;
    buffers.triangles.length = 0;
    buffers.triangle_count = 0;
}


static void push_line(DebugBuffers &out, Vec3f a, Vec3f b, Vec3f color) {
    collections::Vec<float> &lines = out.lines;
    if (lines.length + LINE_STRIDE > lines.capacity) {
        const uint32_t doubled = lines.capacity * 2;
        const uint32_t wanted = lines.length + LINE_STRIDE * 64;
        if (!lines.reserve(doubled > wanted ? doubled : wanted)) return;
    }
    float *vertex = lines.data + lines.length;
    vertex[0] = vec3f::x(a);
    vertex[1] = vec3f::y(a);
    vertex[2] = vec3f::z(a);
    vertex[3] = vec3f::x(color);
    vertex[4] = vec3f::y(color);
    vertex[5] = vec3f::z(color);
    vertex[6] = vec3f::x(b);
    vertex[7] = vec3f::y(b);
    vertex[8] = vec3f::z(b);
    vertex[9] = vec3f::x(color);
    vertex[10] = vec3f::y(color);
    vertex[11] = vec3f::z(color);
    lines.length += LINE_STRIDE;
    out.line_count++;
}

void line(DebugBuffers &out, Vec3f a, Vec3f b, Vec3f color) { push_line(out, a, b, color); }

/** circle in the plane of two perpendicular unit axes u and v, centred at `center` */
static void push_circle(DebugBuffers &out, Vec3f center, Vec3f u, Vec3f v, float radius, uint32_t segments,
                        Vec3f color) {
    const float step = (2.0f * scalar::PI) / (float)segments;
    Vec3f previous = vec3f::scale_and_add(center, u, radius);
    for (uint32_t i = 1; i <= segments; i++) {
        const float angle = (float)i * step;
        const float cos = scalar::cos(angle);
        const float sin = scalar::sin(angle);
        Vec3f next = vec3f::scale_and_add(center, u, cos * radius);
        next = vec3f::scale_and_add(next, v, sin * radius);
        push_line(out, previous, next, color);
        previous = next;
    }
}

/** 3-axis cross/marker at a point */
static void push_cross(DebugBuffers &out, Vec3f position, float size, Vec3f color) {
    push_line(out, vec3f::make(vec3f::x(position) - size, vec3f::y(position), vec3f::z(position)), vec3f::make(vec3f::x(position) + size, vec3f::y(position), vec3f::z(position)), color);
    push_line(out, vec3f::make(vec3f::x(position), vec3f::y(position) - size, vec3f::z(position)), vec3f::make(vec3f::x(position), vec3f::y(position) + size, vec3f::z(position)), color);
    push_line(out, vec3f::make(vec3f::x(position), vec3f::y(position), vec3f::z(position) - size), vec3f::make(vec3f::x(position), vec3f::y(position), vec3f::z(position) + size), color);
}

static float hue_to_rgb(float p, float q, float t) {
    if (t < 0.0f) t += 1.0f;
    if (t > 1.0f) t -= 1.0f;
    if (t < 1.0f / 6.0f) return p + (q - p) * 6.0f * t;
    if (t < 1.0f / 2.0f) return q;
    if (t < 2.0f / 3.0f) return p + (q - p) * (2.0f / 3.0f - t) * 6.0f;
    return p;
}

static Vec3f hsl_to_rgb(float h, float s, float l) {
    if (s == 0.0f) return vec3f::make(l, l, l);
    const float q = l < 0.5f ? l * (1.0f + s) : l + s - l * s;
    const float p = 2.0f * l - q;
    return vec3f::make(hue_to_rgb(p, q, h + 1.0f / 3.0f), hue_to_rgb(p, q, h), hue_to_rgb(p, q, h - 1.0f / 3.0f));
}

/** (n * 137.5) % modulus for a non-negative integer n, exact: n * 137.5 is (n * 275) / 2 */
static float golden_hash_mod(uint64_t n, uint32_t modulus) { return (float)((n * 275u) % (2u * (uint64_t)modulus)) * 0.5f; }

static Vec3f body_color(const RigidBody &body, BodyColorMode mode) {
    switch (mode) {
        case COLOR_MOTION_TYPE:
            switch (body.motion_type) {
                case motion_type::MOTION_DYNAMIC:
                    return vec3f::make(0.2f, 1.0f, 0.2f);  // green
                case motion_type::MOTION_KINEMATIC:
                    return vec3f::make(1.0f, 1.0f, 0.0f);  // yellow
                case motion_type::MOTION_STATIC:
                    return vec3f::make(0.2f, 0.2f, 1.0f);  // blue
                default:
                    return vec3f::make(1.0f, 1.0f, 1.0f);
            }

        case COLOR_INSTANCE: {
            if (body.motion_type == motion_type::MOTION_STATIC) {
                const float l = 0.22f + (golden_hash_mod(body.id, 100) / 100.0f) * 0.2f;
                return hsl_to_rgb(0.0f, 0.0f, l);
            }
            const float hue = golden_hash_mod(body.id, 360) / 360.0f;
            return hsl_to_rgb(hue, 0.7f, 0.6f);
        }

        case COLOR_SLEEPING:
            return body.sleeping ? vec3f::make(0.2f, 0.2f, 1.0f) : vec3f::make(1.0f, 0.2f, 0.2f);

        case COLOR_ISLAND: {
            if (body.island_index < 0) return vec3f::make(1.0f, 1.0f, 1.0f);
            return hsl_to_rgb(golden_hash_mod((uint64_t)body.island_index, 360) / 360.0f, 0.8f, 0.6f);
        }
    }
    return vec3f::make(1.0f, 1.0f, 1.0f);
}

/** a pair of unit axes perpendicular to `n` */
static void perpendicular_axes(Vec3f &out_u, Vec3f &out_v, Vec3f n) {
    // pick an axis not parallel to n
    if (scalar::abs(vec3f::x(n)) < 0.9f) {
        out_u = vec3f::make(0.0f, -vec3f::z(n), vec3f::y(n));
    } else {
        out_u = vec3f::make(-vec3f::y(n), vec3f::x(n), 0.0f);
    }
    const float length = vec3f::length(out_u);
    out_u = vec3f::scale(out_u, 1.0f / length);
    // v = n x u
    out_v = vec3f::cross(n, out_u);
}

/** a local point into world space: position + q * (scale * local); the scale is the shape's local space, as a scaled shape's */
static Vec3f &transform_point(Vec3f &out, Vec3f local, const Transformf &transform) {
    out = vec3f::multiply(local, vec3f::load(transform.scale));
    out = vec3f::transform_quat(out, quatf::load(transform.quaternion));
    return (out = vec3f::add(out, vec3f::load(transform.position)));
}

static void draw_shape(DebugBuffers &out, const Shape *shape, const Transformf &transform, Vec3f color);

static void draw_sphere(DebugBuffers &out, const Shape *shape, const Transformf &transform, Vec3f color) {
    // sphere only supports uniform scaling
    const float radius = shapes::as<SphereShape>(shape).radius * transform.scale.x;
    const float a =1.0f / scalar::sqrt(2.0f);
    const Vec3f normals[5] = {vec3f::make(1.0f, 0.0f, 0.0f), vec3f::make(0.0f, 1.0f, 0.0f), vec3f::make(0.0f, 0.0f, 1.0f), vec3f::make(a, 0.0f, a), vec3f::make(a, 0.0f, -a)};
    for (uint32_t i = 0; i < 5; i++) {
        Vec3f u, v;
        perpendicular_axes(u, v, normals[i]);
        push_circle(out, vec3f::load(transform.position), u, v, radius, 32, color);
    }
}

/** the 12 edges of the local box from lo to hi */
static void draw_local_box(DebugBuffers &out, Vec3f lo, Vec3f hi, const Transformf &transform, Vec3f color) {
    const Vec3f corners[8] = {
        vec3f::make(vec3f::x(lo), vec3f::y(lo), vec3f::z(lo)), vec3f::make(vec3f::x(hi), vec3f::y(lo), vec3f::z(lo)), vec3f::make(vec3f::x(hi), vec3f::y(lo), vec3f::z(hi)), vec3f::make(vec3f::x(lo), vec3f::y(lo), vec3f::z(hi)),
        vec3f::make(vec3f::x(lo), vec3f::y(hi), vec3f::z(lo)), vec3f::make(vec3f::x(hi), vec3f::y(hi), vec3f::z(lo)), vec3f::make(vec3f::x(hi), vec3f::y(hi), vec3f::z(hi)), vec3f::make(vec3f::x(lo), vec3f::y(hi), vec3f::z(hi)),
    };
    Vec3f world_corners[8];
    for (uint32_t i = 0; i < 8; i++) transform_point(world_corners[i], corners[i], transform);
    static constexpr uint8_t EDGES[12][2] = {
        {0, 1}, {1, 2}, {2, 3}, {3, 0},  // bottom
        {4, 5}, {5, 6}, {6, 7}, {7, 4},  // top
        {0, 4}, {1, 5}, {2, 6}, {3, 7},  // verticals
    };
    for (uint32_t i = 0; i < 12; i++) push_line(out, world_corners[EDGES[i][0]], world_corners[EDGES[i][1]], color);
}

static void draw_box(DebugBuffers &out, const Shape *shape, const Transformf &transform, Vec3f color) {
    const Vec3f h = vec3f::load(shapes::as<BoxShape>(shape).half_extents);
    draw_local_box(out, vec3f::negate(h), h, transform, color);
}

static void draw_capsule(DebugBuffers &out, const Shape *shape, const Transformf &transform, Vec3f color) {
    // capsule only supports uniform scaling
    const CapsuleShape &capsule = shapes::as<CapsuleShape>(shape);
    const float radius = capsule.radius * transform.scale.x;
    const float half_height = capsule.half_height_of_cylinder * transform.scale.x;
    const Vec3f position = vec3f::load(transform.position);
    Vec3f axis = vec3f::transform_quat(vec3f::make(0.0f, 1.0f, 0.0f), quatf::load(transform.quaternion));
    Vec3f u, v;
    perpendicular_axes(u, v, axis);
    const float total_half_height = half_height + radius;
    Vec3f top = vec3f::scale_and_add(position, axis, half_height);
    Vec3f bottom = vec3f::scale_and_add(position, axis, -half_height);
    push_circle(out, top, u, v, radius, 32, color);
    push_circle(out, bottom, u, v, radius, 32, color);
    for (uint32_t i = 0; i < 8; i++) {
        const float angle = ((float)i / 8.0f) * scalar::PI * 2.0f;
        Vec3f direction = vec3f::scale(u, scalar::cos(angle));
        direction = vec3f::scale_and_add(direction, v, scalar::sin(angle));
        Vec3f from = vec3f::scale_and_add(top, direction, radius);
        Vec3f to = vec3f::scale_and_add(bottom, direction, radius);
        push_line(out, from, to, color);
    }
    const uint32_t num_longitudes = 12;
    const uint32_t segments = 16;
    for (uint32_t i = 0; i < num_longitudes; i++) {
        const float angle = ((float)i / (float)num_longitudes) * scalar::PI * 2.0f;
        Vec3f arc_direction = vec3f::scale(u, scalar::cos(angle));
        arc_direction = vec3f::scale_and_add(arc_direction, v, scalar::sin(angle));
        Vec3f previous = vec3f::scale_and_add(position, axis, total_half_height);
        for (uint32_t j = 1; j <= segments; j++) {
            const float t = (float)j / (float)segments;
            const float y_local = (1.0f - t) * total_half_height - t * total_half_height;
            float radial_offset = radius;
            if (scalar::abs(y_local) > half_height) {
                const float y_offset = (scalar::abs(y_local) - half_height) / radius;
                // clamped: at the pole y_offset can round a hair past 1
                radial_offset = radius * scalar::sqrt(scalar::max(0.0f, 1.0f - y_offset * y_offset));
            }
            Vec3f next = vec3f::scale_and_add(position, axis, y_local);
            next = vec3f::scale_and_add(next, arc_direction, radial_offset);
            push_line(out, previous, next, color);
            previous = next;
        }
    }
}

static void draw_cylinder(DebugBuffers &out, const Shape *shape, const Transformf &transform, Vec3f color) {
    // cylinder scales radius by x/z (uniform) and half-height by y
    const CylinderShape &cylinder = shapes::as<CylinderShape>(shape);
    const float radius = cylinder.radius * transform.scale.x;
    const float half_height = cylinder.half_height * transform.scale.y;
    Vec3f axis = vec3f::transform_quat(vec3f::make(0.0f, 1.0f, 0.0f), quatf::load(transform.quaternion));
    Vec3f u, v;
    perpendicular_axes(u, v, axis);
    Vec3f top = vec3f::scale_and_add(vec3f::load(transform.position), axis, half_height);
    Vec3f bottom = vec3f::scale_and_add(vec3f::load(transform.position), axis, -half_height);
    push_circle(out, top, u, v, radius, 32, color);
    push_circle(out, bottom, u, v, radius, 32, color);
    Vec3f sides[4];
    sides[0] = u;
    sides[1] = vec3f::negate(u);
    sides[2] = v;
    sides[3] = vec3f::negate(v);
    for (uint32_t i = 0; i < 4; i++) {
        Vec3f from = vec3f::scale_and_add(top, sides[i], radius);
        Vec3f to = vec3f::scale_and_add(bottom, sides[i], radius);
        push_line(out, from, to, color);
    }
}

static void draw_plane(DebugBuffers &out, const Shape *shape, const Transformf &transform, Vec3f color) {
    // infinite plane: draw a grid (sized by half_extent) in the plane's local frame, then transform every point into
    // world space by the node transform
    const PlaneShape &plane = shapes::as<PlaneShape>(shape);
    const Vec3f n = vec3f::load(plane.plane.normal);
    const float size = plane.half_extent;
    Vec3f u, v;
    perpendicular_axes(u, v, n);
    const Vec3f origin = vec3f::scale(n, -plane.plane.constant);
    const int32_t steps = 5;
    for (int32_t i = -steps; i <= steps; i++) {
        const float t = ((float)i / (float)steps) * size;
        Vec3f local_a, local_b, world_a, world_b;
        local_a = vec3f::scale_and_add(origin, u, t);
        local_b = vec3f::scale_and_add(local_a, v, size);
        local_a = vec3f::scale_and_add(local_a, v, -size);
        push_line(out, transform_point(world_a, local_a, transform), transform_point(world_b, local_b, transform), color);
        local_a = vec3f::scale_and_add(origin, v, t);
        local_b = vec3f::scale_and_add(local_a, u, size);
        local_a = vec3f::scale_and_add(local_a, u, -size);
        push_line(out, transform_point(world_a, local_a, transform), transform_point(world_b, local_b, transform), color);
    }
}

static void draw_convex_hull(DebugBuffers &out, const Shape *shape, const Transformf &transform, Vec3f color) {
    // emit all face edges (may duplicate, fine for debug use)
    const convex_hull::ConvexHullData &hull = *shapes::as<ConvexHullShape>(shape).data;
    const float *points = hull.point_positions.data;
    for (uint32_t f = 0; f < hull.faces.length; f++) {
        const convex_hull::ConvexHullFace &face = hull.faces[f];
        const uint32_t n = face.num_vertices;
        for (uint32_t i = 0; i < n; i++) {
            const uint32_t index_a = hull.vertex_indices[face.first_vertex + i];
            const uint32_t index_b = hull.vertex_indices[face.first_vertex + (i + 1) % n];
            Vec3f world_a, world_b;
            transform_point(world_a, vec3f::make(points[index_a * 3], points[index_a * 3 + 1], points[index_a * 3 + 2]), transform);
            transform_point(world_b, vec3f::make(points[index_b * 3], points[index_b * 3 + 1], points[index_b * 3 + 2]), transform);
            push_line(out, world_a, world_b, color);
        }
    }
}

static void draw_triangle_mesh(DebugBuffers &out, const Shape *shape, const Transformf &transform, Vec3f color) {
    const triangle_mesh_data::TriangleMeshData &mesh = *shapes::as<TriangleMeshShape>(shape).data;
    const float *positions = mesh.positions;
    for (uint32_t i = 0; i < mesh.triangle_count; i++) {
        const triangle_mesh_data::Triangle &triangle = mesh.triangles[i];
        const uint32_t ia = triangle.index_a, ib = triangle.index_b, ic = triangle.index_c;
        Vec3f world_a, world_b, world_c;
        transform_point(world_a, vec3f::make(positions[ia * 3], positions[ia * 3 + 1], positions[ia * 3 + 2]), transform);
        transform_point(world_b, vec3f::make(positions[ib * 3], positions[ib * 3 + 1], positions[ib * 3 + 2]), transform);
        transform_point(world_c, vec3f::make(positions[ic * 3], positions[ic * 3 + 1], positions[ic * 3 + 2]), transform);
        push_line(out, world_a, world_b, color);
        push_line(out, world_b, world_c, color);
        push_line(out, world_c, world_a, color);
    }
}

/** recursively emits a shape and its children, accumulating the local to world transform as it descends */
static void draw_shape(DebugBuffers &out, const Shape *shape, const Transformf &transform, Vec3f color) {
    switch (shape->type) {
        case shapes::SHAPE_SPHERE:
            draw_sphere(out, shape, transform, color);
            break;
        case shapes::SHAPE_BOX:
            draw_box(out, shape, transform, color);
            break;
        case shapes::SHAPE_CAPSULE:
            draw_capsule(out, shape, transform, color);
            break;
        case shapes::SHAPE_CYLINDER:
            draw_cylinder(out, shape, transform, color);
            break;
        case shapes::SHAPE_PLANE:
            draw_plane(out, shape, transform, color);
            break;
        case shapes::SHAPE_CONVEX_HULL:
            draw_convex_hull(out, shape, transform, color);
            break;
        case shapes::SHAPE_TRIANGLE_MESH:
            draw_triangle_mesh(out, shape, transform, color);
            break;
        case shapes::SHAPE_COMPOUND:
        case shapes::SHAPE_STATIC_COMPOUND: {
            const collections::Vec<compound::CompoundShapeChild> &children = compound::children_of(shape);
            for (uint32_t i = 0; i < children.length; i++) {
                Transformf child;
                compound::child_transform(child, transform, children[i]);
                draw_shape(out, children[i].shape, child, color);
            }
            break;
        }
        case shapes::SHAPE_TRANSFORMED: {
            // the parent's scale applies to the offset; the inner shape sees it in its own space
            const TransformedShape &transformed = shapes::as<TransformedShape>(shape);
            Transformf inner;
            Vec3f translation = vec3f::multiply(vec3f::load(transformed.position), vec3f::load(transform.scale));
            translation = vec3f::transform_quat(translation, quatf::load(transform.quaternion));
            vec3f::store(inner.position, vec3f::add(vec3f::load(transform.position), translation));
            quatf::store(inner.quaternion, quatf::multiply(quatf::load(transform.quaternion), quatf::load(transformed.quaternion)));
            vec3f::store(inner.scale, shapes::transform_scale(quatf::load(transformed.quaternion), vec3f::load(transform.scale)));
            draw_shape(out, transformed.shape, inner, color);
            break;
        }
        case shapes::SHAPE_SCALED: {
            // accumulate this node's scale into the local-space scale, then recurse. leaf shapes apply the scale to their
            // geometry; the inner shape's own data is never pre-scaled, so the scale must be threaded through here
            const ScaledShape &scaled = shapes::as<ScaledShape>(shape);
            Transformf inner = transform;
            vec3f::store(inner.scale, vec3f::multiply(vec3f::load(transform.scale), vec3f::load(scaled.scale)));
            draw_shape(out, scaled.shape, inner, color);
            break;
        }
        case shapes::SHAPE_OFFSET_CENTER_OF_MASS:
            // offset only affects com, not geometry position
            draw_shape(out, shapes::as<OffsetCenterOfMassShape>(shape).shape, transform, color);
            break;
        case shapes::SHAPE_EMPTY:
            break;
        default: {
            const shapes::ShapeDef &def = shapes::def_of(shape);
            if (def.draw != nullptr) def.draw(out, shape, transform, color);
            else draw_local_box(out, vec3f::load(shape->aabb.min), vec3f::load(shape->aabb.max), transform, color);
            break;
        }
    }
}

static const Float3 VELOCITY_LINEAR_COLOR = {1.0f, 1.0f, 0.0f};
static const Float3 VELOCITY_ANGULAR_COLOR = {1.0f, 0.5f, 0.0f};

static void push_velocity(DebugBuffers &out, Vec3f origin, Vec3f velocity, float length_scale, float max_length,
                          Vec3f color) {
    const float magnitude = vec3f::length(velocity);
    if (magnitude <= 0.01f) return;
    const float length = scalar::min(magnitude * length_scale, max_length);
    Vec3f tip = vec3f::scale_and_add(origin, velocity, length / magnitude);
    push_line(out, origin, tip, color);
}

/** shape and velocity arrows for one body */
static void draw_body_lines(DebugBuffers &out, const RigidBody &body, Vec3f color, bool show_linear_velocity,
                            bool show_angular_velocity) {
    draw_shape(out, body.shape, {body.position, body.quaternion, {1.0f, 1.0f, 1.0f}}, color);
    if (body.motion_type == motion_type::MOTION_STATIC) return;
    if (show_linear_velocity) {
        push_velocity(out, vec3f::load(body.center_of_mass_position), vec3f::load(body.motion_properties.linear_velocity), 1.0f, 5.0f, vec3f::load(VELOCITY_LINEAR_COLOR));
    }
    if (show_angular_velocity) {
        push_velocity(out, vec3f::load(body.center_of_mass_position), vec3f::load(body.motion_properties.angular_velocity), 0.5f, 3.0f, vec3f::load(VELOCITY_ANGULAR_COLOR));
    }
}

/** the body in a live slot, or null */
static const RigidBody *live_body(const Bodies &pool, uint32_t index) {
    if (index >= pool.pool.length || pool.sequences[index] == tumble::bodies::SLOT_FREE) return nullptr;
    return pool.pool[index];
}

void bodies(DebugBuffers &out, const world::World &world, const BodiesOptions &options) {
    for (uint32_t i = 0; i < world.bodies.pool.length; i++) {
        const RigidBody *body = live_body(world.bodies, i);
        if (body == nullptr) continue;
        if (body->object_layer < 32 && (options.excluded_object_layers & (1u << body->object_layer)) != 0) continue;
        draw_body_lines(out, *body, body_color(*body, options.color_mode), options.show_linear_velocity, options.show_angular_velocity);
    }
}

void body(DebugBuffers &out, const RigidBody &body, const BodyOptions &options) {
    const Vec3f color = options.has_color ? vec3f::load(options.color) : body_color(body, options.color_mode);
    draw_body_lines(out, body, color, options.show_linear_velocity, options.show_angular_velocity);
}

void shape(DebugBuffers &out, const Shape *shape, Vec3f position, Quatf quaternion, const ShapeOptions &options) {
    Transformf transform = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}};
    vec3f::store(transform.position, position);
    quatf::store(transform.quaternion, quaternion);
    draw_shape(out, shape, transform, vec3f::load(options.color));
}

void contacts(DebugBuffers &out, const world::World &world) {
    const ContactConstraints &manifolds = world.contact_constraints;
    for (uint32_t i = 0; i < manifolds.count; i++) {
        const ContactConstraint &constraint = manifolds.pool[i];
        for (uint32_t j = 0; j < constraint.num_contact_points; j++) {
            const Vec3f a = vec3f::load(constraint.contact_points[j].position_a);
            // cross at contact point a, red
            push_cross(out, a, 0.05f, vec3f::make(1.0f, 0.0f, 0.0f));
            // normal line from contact point a, magenta
            Vec3f tip = vec3f::scale_and_add(a, vec3f::load(constraint.normal), 0.15f);
            push_line(out, a, tip, vec3f::make(1.0f, 0.0f, 1.0f));
        }
    }
}

void character_contacts(DebugBuffers &out, const kcc::KCC &character) {
    for (uint32_t i = 0; i < character.contacts.length; i++) {
        const kcc::CharacterContact &contact = character.contacts[i];
        // a collision red, a predictive contact yellow, a discarded one grey; the normal toward the character
        const Vec3f color = contact.was_discarded ? vec3f::make(0.5f, 0.5f, 0.5f)
                            : contact.had_collision ? vec3f::make(1.0f, 0.0f, 0.0f)
                                                    : vec3f::make(1.0f, 1.0f, 0.0f);
        push_cross(out, contact.position, 0.05f, color);
        push_line(out, contact.position, vec3f::scale_and_add(contact.position, contact.contact_normal, 0.15f), color);
    }
}

void contact_constraints(DebugBuffers &out, const world::World &world) {
    const ContactConstraints &manifolds = world.contact_constraints;
    const Vec3f edge_color = vec3f::make(0.8f, 0.8f, 0.0f);
    for (uint32_t i = 0; i < manifolds.count; i++) {
        const ContactConstraint &constraint = manifolds.pool[i];
        if (constraint.num_contact_points == 0) continue;

        const bool manifold_impulse = constraint.friction_constraint_1.total_lambda != 0.0f ||
                                      constraint.friction_constraint_2.total_lambda != 0.0f ||
                                      constraint.angular_friction_constraint.total_lambda != 0.0f;
        for (uint32_t j = 0; j < constraint.num_contact_points; j++) {
            const WorldContactPoint &point = constraint.contact_points[j];
            const bool has_impulse = manifold_impulse || point.normal_constraint.total_lambda != 0.0f;
            const float size = has_impulse ? 0.04f : 0.02f;
            // point a cyan, point b magenta, bright if active
            push_cross(out, vec3f::load(point.position_a), size, has_impulse ? vec3f::make(0.0f, 1.0f, 1.0f) : vec3f::make(0.0f, 0.4f, 0.6f));
            push_cross(out, vec3f::load(point.position_b), size, has_impulse ? vec3f::make(1.0f, 0.0f, 1.0f) : vec3f::make(0.5f, 0.0f, 0.5f));
            // edge between consecutive a points, yellow
            if (j > 0) push_line(out, vec3f::load(constraint.contact_points[j - 1].position_a), vec3f::load(point.position_a), edge_color);
        }

        // close loop if 3+ points
        if (constraint.num_contact_points > 2) {
            push_line(out, vec3f::load(constraint.contact_points[constraint.num_contact_points - 1].position_a), vec3f::load(constraint.contact_points[0].position_a),
                      edge_color);
        }

        const Vec3f first = vec3f::load(constraint.contact_points[0].position_a);
        Vec3f tip;
        // normal arrow from first point, magenta
        push_line(out, first, (tip = vec3f::scale_and_add(first, vec3f::load(constraint.normal), 0.3f)), vec3f::make(1.0f, 0.0f, 1.0f));
        // tangent 1 green, tangent 2 blue
        push_line(out, first, (tip = vec3f::scale_and_add(first, vec3f::load(constraint.tangent_1), 0.2f)), vec3f::make(0.0f, 1.0f, 0.0f));
        push_line(out, first, (tip = vec3f::scale_and_add(first, vec3f::load(constraint.tangent_2), 0.2f)), vec3f::make(0.0f, 0.4f, 1.0f));
    }
}


static void draw_pie(DebugBuffers &out, Vec3f center, float radius, Vec3f normal, Vec3f axis, float min_angle,
                     float max_angle, Vec3f color) {
    if (min_angle >= max_angle) return;
    const uint32_t segments = 32;
    const float step = (max_angle - min_angle) / (float)segments;
    Vec3f previous;
    for (uint32_t i = 0; i <= segments; i++) {
        Quatf rotation = quatf::set_axis_angle(normal, min_angle + (float)i * step);
        const Vec3f rotated_axis = vec3f::transform_quat(axis, rotation);
        Vec3f point = vec3f::scale_and_add(center, rotated_axis, radius);
        // radial spoke from centre
        push_line(out, center, point, color);
        if (i > 0) push_line(out, previous, point, color);
        previous = point;
    }
}

static void draw_swing_cone_limits(DebugBuffers &out, Vec3f center, Vec3f twist_axis, Vec3f plane_axis,
                                   Vec3f normal_axis, float plane_half_angle, float normal_half_angle, float radius,
                                   Vec3f color) {
    const float max_angle = scalar::max(plane_half_angle, normal_half_angle);
    if (max_angle <= 0.0f) return;
    const uint32_t segments = 32;
    const float step = (2.0f * scalar::PI) / (float)segments;
    const float cos_max = scalar::cos(max_angle);
    const float sin_max = scalar::sin(max_angle);
    Vec3f previous;
    for (uint32_t i = 0; i <= segments; i++) {
        const float theta = (float)i * step;
        Vec3f direction = vec3f::scale(plane_axis, scalar::cos(theta) * sin_max);
        Vec3f point;
        direction = vec3f::scale_and_add(direction, normal_axis, scalar::sin(theta) * sin_max);
        direction = vec3f::scale_and_add(direction, twist_axis, cos_max);
        point = vec3f::scale_and_add(center, direction, radius);
        if (i % 4 == 0) push_line(out, center, point, color);
        if (i > 0) push_line(out, previous, point, color);
        previous = point;
    }
}

/** a body's centre of mass frame, which constraint local positions are given in */
struct BodyTransform {
    Float3 center_of_mass_position;
    Float4 quaternion;
};

static Vec3f transform_point_to_world(Vec3f local, const BodyTransform &transform) {
    return vec3f::add(vec3f::transform_quat(local, quatf::load(transform.quaternion)), vec3f::load(transform.center_of_mass_position));
}

static Vec3f transform_direction_to_world(Vec3f local, const BodyTransform &transform) {
    return vec3f::transform_quat(local, quatf::load(transform.quaternion));
}

/** the frames of an enabled constraint's bodies; false when the constraint is free or disabled or a body is gone */
static bool constraint_transforms(BodyTransform &out_a, BodyTransform &out_b, const Bodies &pool,
                                  const constraints::ConstraintBase &base) {
    if (base.pooled || !base.enabled) return false;
    const RigidBody *body_a = live_body(pool, base.body_index_a);
    const RigidBody *body_b = live_body(pool, base.body_index_b);
    if (body_a == nullptr || body_b == nullptr) return false;
    out_a = {body_a->center_of_mass_position, body_a->quaternion};
    out_b = {body_b->center_of_mass_position, body_b->quaternion};
    return true;
}

static void push_axis(DebugBuffers &out, Vec3f from, Vec3f direction, float length, Vec3f color) {
    Vec3f tip;
    push_line(out, from, (tip = vec3f::scale_and_add(from, direction, length)), color);
}

static const Float3 RED = {1.0f, 0.0f, 0.0f};
static const Float3 GREEN = {0.0f, 1.0f, 0.0f};
static const Float3 BLUE = {0.0f, 0.0f, 1.0f};
static const Float3 WHITE = {1.0f, 1.0f, 1.0f};
static const Float3 PURPLE = {0.5f, 0.0f, 0.5f};

static void draw_hinge(DebugBuffers &out, const hinge_constraint::HingeConstraint &constraint, const BodyTransform &a,
                       const BodyTransform &b, const JointsOptions &options) {
    const float size = options.size;
    Vec3f point_a, point_b, hinge_axis_a, hinge_axis_b, normal_axis_a, normal_axis_b;
    point_a = transform_point_to_world(constraint.local_space_position1, a);
    point_b = transform_point_to_world(constraint.local_space_position2, b);
    hinge_axis_a = transform_direction_to_world(constraint.local_space_hinge_axis1, a);
    hinge_axis_b = transform_direction_to_world(constraint.local_space_hinge_axis2, b);
    normal_axis_a = transform_direction_to_world(constraint.local_space_normal_axis1, a);
    normal_axis_b = transform_direction_to_world(constraint.local_space_normal_axis2, b);

    push_cross(out, point_a, size * 0.1f, vec3f::load(RED));
    push_axis(out, point_a, hinge_axis_a, size, vec3f::load(RED));
    push_cross(out, point_b, size * 0.1f, vec3f::load(GREEN));
    push_axis(out, point_b, hinge_axis_b, size, vec3f::load(GREEN));
    // b's normal axis, which turns against a's limit pie as the hinge swings
    push_axis(out, point_b, normal_axis_b, size, vec3f::load(WHITE));

    if (options.draw_limits && constraint.has_limits && constraint.limits_max > constraint.limits_min) {
        draw_pie(out, point_a, size, hinge_axis_a, normal_axis_a, constraint.limits_min, constraint.limits_max, vec3f::load(PURPLE));
    }
}

/** the constraint space's axes in world space: the body's rotation times constraint_to_body */
static void constraint_space_axes(Vec3f out_axes[3], const BodyTransform &body, Quatf constraint_to_body) {
    const Quatf rotation = quatf::multiply(quatf::load(body.quaternion), constraint_to_body);
    out_axes[0] = vec3f::transform_quat(vec3f::axis_x(), rotation);
    out_axes[1] = vec3f::transform_quat(vec3f::axis_y(), rotation);
    out_axes[2] = vec3f::transform_quat(vec3f::axis_z(), rotation);
}

static void draw_swing_twist(DebugBuffers &out, const swing_twist_constraint::SwingTwistConstraint &constraint, const BodyTransform &a,
                             const BodyTransform &b, const JointsOptions &options) {
    const float size = options.size;
    Vec3f point_a, point_b;
    point_a = transform_point_to_world(constraint.local_space_position1, a);
    point_b = transform_point_to_world(constraint.local_space_position2, b);
    push_cross(out, point_a, size * 0.1f, vec3f::load(RED));
    push_cross(out, point_b, size * 0.1f, vec3f::load(GREEN));
    push_line(out, point_a, point_b, vec3f::load(WHITE));

    if (!options.draw_limits) return;
    // twist, plane, normal
    Vec3f axes[3];
    constraint_space_axes(axes, a, constraint.constraint_to_body1);
    if (constraint.plane_half_cone_angle > 0.0f || constraint.normal_half_cone_angle > 0.0f) {
        draw_swing_cone_limits(out, point_a, axes[0], axes[1], axes[2], constraint.plane_half_cone_angle, constraint.normal_half_cone_angle,
                               size, vec3f::load(GREEN));
    }
    if (constraint.twist_max_angle > constraint.twist_min_angle) {
        draw_pie(out, point_a, size, axes[0], axes[1], constraint.twist_min_angle, constraint.twist_max_angle, vec3f::load(PURPLE));
    }
}

static void draw_distance(DebugBuffers &out, const distance_constraint::DistanceConstraint &constraint, const BodyTransform &a,
                          const BodyTransform &b, const JointsOptions &options) {
    Vec3f point_a, point_b;
    point_a = transform_point_to_world(constraint.local_space_position1, a);
    point_b = transform_point_to_world(constraint.local_space_position2, b);
    push_line(out, point_a, point_b, vec3f::load(GREEN));

    if (!options.draw_limits) return;
    const Vec3f delta = vec3f::subtract(point_b, point_a);
    const float distance = vec3f::length(delta);
    if (distance <= 0.001f) return;
    Vec3f direction = vec3f::scale(delta, 1.0f / distance);
    Vec3f u, v;
    perpendicular_axes(u, v, direction);
    if (constraint.min_distance > 0.0f) push_circle(out, point_a, u, v, constraint.min_distance, 16, vec3f::make(0.0f, 0.4f, 1.0f));
    if (constraint.max_distance < scalar::INF) push_circle(out, point_a, u, v, constraint.max_distance, 16, vec3f::make(1.0f, 0.2f, 0.2f));
}

static void draw_cone(DebugBuffers &out, const cone_constraint::ConeConstraint &constraint, const BodyTransform &a, const BodyTransform &b,
                      const JointsOptions &options) {
    const float size = options.size;
    Vec3f point_a, point_b, twist_axis_a, twist_axis_b;
    point_a = transform_point_to_world(constraint.local_space_position1, a);
    point_b = transform_point_to_world(constraint.local_space_position2, b);
    twist_axis_a = transform_direction_to_world(constraint.local_space_twist_axis1, a);
    twist_axis_b = transform_direction_to_world(constraint.local_space_twist_axis2, b);

    push_cross(out, point_a, size * 0.1f, vec3f::load(RED));
    push_axis(out, point_a, twist_axis_a, size, vec3f::load(RED));
    push_cross(out, point_b, size * 0.1f, vec3f::load(GREEN));
    push_axis(out, point_b, twist_axis_b, size, vec3f::load(GREEN));

    if (options.draw_limits && constraint.cos_half_cone_angle < 1.0f) {
        const float half_angle = scalar::acos(constraint.cos_half_cone_angle);
        Vec3f u, v;
        perpendicular_axes(u, v, twist_axis_a);
        draw_swing_cone_limits(out, point_a, twist_axis_a, u, v, half_angle, half_angle, size, vec3f::make(1.0f, 1.0f, 0.0f));
    }
}

static void draw_fixed(DebugBuffers &out, const fixed_constraint::FixedConstraint &constraint, const BodyTransform &a, const BodyTransform &b,
                       const JointsOptions &options) {
    Vec3f point_a, point_b;
    push_cross(out, point_a = transform_point_to_world(constraint.local_space_position1, a), options.size * 0.1f, vec3f::load(BLUE));
    push_cross(out, point_b = transform_point_to_world(constraint.local_space_position2, b), options.size * 0.1f, vec3f::load(BLUE));
}

static void draw_point(DebugBuffers &out, const point_constraint::PointConstraint &constraint, const BodyTransform &a, const BodyTransform &b,
                       const JointsOptions &options) {
    Vec3f point_a, point_b;
    point_a = transform_point_to_world(constraint.local_space_position1, a);
    point_b = transform_point_to_world(constraint.local_space_position2, b);
    push_cross(out, point_a, options.size * 0.1f, vec3f::load(WHITE));
    push_cross(out, point_b, options.size * 0.1f, vec3f::load(WHITE));
    push_line(out, point_a, point_b, vec3f::load(WHITE));
}

static void draw_slider(DebugBuffers &out, const slider_constraint::SliderConstraint &constraint, const BodyTransform &a,
                        const BodyTransform &b, const JointsOptions &options) {
    const float size = options.size;
    Vec3f point_a, point_b, slider_axis;
    point_a = transform_point_to_world(constraint.local_space_position_a, a);
    point_b = transform_point_to_world(constraint.local_space_position_b, b);
    slider_axis = transform_direction_to_world(constraint.local_space_slider_axis_a, a);

    push_cross(out, point_a, size * 0.1f, vec3f::load(RED));
    push_axis(out, point_a, slider_axis, size, vec3f::load(RED));
    push_cross(out, point_b, size * 0.1f, vec3f::load(GREEN));

    if (!options.draw_limits || !constraint.has_limits) return;
    Vec3f min_point = vec3f::scale_and_add(point_a, slider_axis, constraint.limits_min);
    Vec3f max_point = vec3f::scale_and_add(point_a, slider_axis, constraint.limits_max);
    const bool has_min = constraint.limits_min > -scalar::INF;
    const bool has_max = constraint.limits_max < scalar::INF;
    if (has_min) push_cross(out, min_point, size * 0.15f, vec3f::load(BLUE));
    if (has_max) push_cross(out, max_point, size * 0.15f, vec3f::load(RED));
    if (has_min && has_max) push_line(out, min_point, max_point, vec3f::load(PURPLE));
}

static void draw_six_dof(DebugBuffers &out, const six_dof_constraint::SixDOFConstraint &constraint, const BodyTransform &a,
                         const BodyTransform &b, const JointsOptions &options) {
    const float size = options.size;
    Vec3f point_a, point_b;
    point_a = transform_point_to_world(constraint.local_space_position1, a);
    point_b = transform_point_to_world(constraint.local_space_position2, b);
    push_cross(out, point_a, size * 0.1f, vec3f::load(PURPLE));
    push_cross(out, point_b, size * 0.1f, vec3f::load(PURPLE));

    if (!options.draw_limits) return;
    Vec3f axes[3];
    constraint_space_axes(axes, a, constraint.constraint_to_body1);
    const Vec3f axis_colors[3] = {vec3f::load(RED), vec3f::load(GREEN), vec3f::load(BLUE)};
    const float big = 3.4e38f;

    for (uint32_t i = 0; i < 3; i++) {
        const float min_limit = constraint.limit_min[six_dof_constraint::TRANSLATION_X + i];
        const float max_limit = constraint.limit_max[six_dof_constraint::TRANSLATION_X + i];
        const bool is_free = min_limit <= -big && max_limit >= big;
        const bool is_fixed = min_limit >= big && max_limit <= -big;
        if (is_free || is_fixed || min_limit >= max_limit) continue;
        Vec3f min_point = vec3f::scale_and_add(point_a, axes[i], min_limit);
        Vec3f max_point = vec3f::scale_and_add(point_a, axes[i], max_limit);
        if (min_limit > -big) push_cross(out, min_point, size * 0.08f, axis_colors[i]);
        if (max_limit < big) push_cross(out, max_point, size * 0.08f, axis_colors[i]);
        if (min_limit > -big && max_limit < big) push_line(out, min_point, max_point, axis_colors[i]);
    }

    for (uint32_t i = 0; i < 3; i++) {
        const float min_limit = constraint.limit_min[six_dof_constraint::ROTATION_X + i];
        const float max_limit = constraint.limit_max[six_dof_constraint::ROTATION_X + i];
        const bool is_free = min_limit <= -big && max_limit >= big;
        const bool is_fixed = min_limit >= big && max_limit <= -big;
        if (!is_free && !is_fixed && min_limit < max_limit && scalar::abs(max_limit - min_limit) > 0.001f) {
            draw_pie(out, point_a, size * 0.5f, axes[i], axes[(i + 1) % 3], min_limit, max_limit, axis_colors[i]);
        }
    }
}

/** each record of the type's pool whose constraint draws, passed with its bodies' frames */
template <typename T>
static void draw_pool(DebugBuffers &out, const world::World &world, constraint_id::ConstraintType type, const JointsOptions &options,
                      void (*draw)(DebugBuffers &, const T &, const BodyTransform &, const BodyTransform &, const JointsOptions &)) {
    const constraints::ConstraintPool &pool = world.constraints.pools[type];
    if (pool.stride == 0) return;
    for (uint32_t i = 0; i < pool.count; i++) {
        const constraints::ConstraintBase *base = constraints::base_at(pool, i);
        BodyTransform a, b;
        if (!constraint_transforms(a, b, world.bodies, *base)) continue;
        draw(out, *reinterpret_cast<const T *>(base), a, b, options);
    }
}

void joints(DebugBuffers &out, const world::World &world, const JointsOptions &options) {
    draw_pool<hinge_constraint::HingeConstraint>(out, world, constraint_id::HINGE, options, draw_hinge);
    draw_pool<swing_twist_constraint::SwingTwistConstraint>(out, world, constraint_id::SWING_TWIST, options, draw_swing_twist);
    draw_pool<distance_constraint::DistanceConstraint>(out, world, constraint_id::DISTANCE, options, draw_distance);
    draw_pool<cone_constraint::ConeConstraint>(out, world, constraint_id::CONE, options, draw_cone);
    draw_pool<fixed_constraint::FixedConstraint>(out, world, constraint_id::FIXED, options, draw_fixed);
    draw_pool<point_constraint::PointConstraint>(out, world, constraint_id::POINT, options, draw_point);
    draw_pool<slider_constraint::SliderConstraint>(out, world, constraint_id::SLIDER, options, draw_slider);
    draw_pool<six_dof_constraint::SixDOFConstraint>(out, world, constraint_id::SIX_DOF, options, draw_six_dof);
}

}  // namespace debug

TUMBLE_NAMESPACE_END
