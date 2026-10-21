#include "app/shape_mesh.h"

#include <math.h>

#include "app/vec.h"
#include "tumble/physics/shapes/compound.h"
#include "tumble/physics/shapes/convex_hull.h"
#include "tumble/physics/shapes/utils/triangle_mesh_data.h"

using namespace tumble;
using namespace vec;

namespace shape_mesh {

static constexpr float PI = 3.14159265f;
static constexpr int SEGMENTS = 24;
static constexpr int RINGS = 12;

/** where a leaf sits in the outermost shape's space: scaled, then rotated, then moved, as the library places children */
struct Placement {
    Float3 position;
    Float4 quaternion;
    Float3 scale;
};

struct Builder {
    collections::Vec<MeshVertex> &out;
    Placement placement;
};

static void vertex(Builder &builder, Float3 point, Float3 normal) {
    const Placement &p = builder.placement;
    const Float3 placed = add(p.position, rotate(p.quaternion, multiply(point, p.scale)));
    const Float3 turned = normalize(rotate(p.quaternion, {normal.x / p.scale.x, normal.y / p.scale.y, normal.z / p.scale.z}));
    builder.out.push({placed.x, placed.y, placed.z, turned.x, turned.y, turned.z});
}

static void triangle(Builder &builder, Float3 a, Float3 b, Float3 c, Float3 normal) {
    vertex(builder, a, normal);
    vertex(builder, b, normal);
    vertex(builder, c, normal);
}

static void quad(Builder &builder, Float3 a, Float3 b, Float3 c, Float3 d, Float3 na, Float3 nb, Float3 nc, Float3 nd) {
    vertex(builder, a, na);
    vertex(builder, b, nb);
    vertex(builder, c, nc);
    vertex(builder, a, na);
    vertex(builder, c, nc);
    vertex(builder, d, nd);
}

static void box(Builder &builder, Float3 half) {
    for (int axis = 0; axis < 3; axis++) {
        for (int sign = -1; sign <= 1; sign += 2) {
            const int u = (axis + 1) % 3;
            const int v = (axis + 2) % 3;
            Float3 corners[4];
            for (int k = 0; k < 4; k++) {
                float c[3];
                c[axis] = (float)sign;
                c[u] = (k == 1 || k == 2) ? 1.0f : -1.0f;
                c[v] = (k >= 2) ? 1.0f : -1.0f;
                if (sign < 0) c[v] = -c[v];
                corners[k] = {c[0] * half.x, c[1] * half.y, c[2] * half.z};
            }
            float n[3] = {0, 0, 0};
            n[axis] = (float)sign;
            const Float3 normal = {n[0], n[1], n[2]};
            quad(builder, corners[0], corners[1], corners[2], corners[3], normal, normal, normal, normal);
        }
    }
}

/** a sphere split at its equator, its halves moved half_height up and down: a capsule, or at 0 a sphere */
static void rounded(Builder &builder, float radius, float half_height) {
    struct Ring {
        float theta;
        float lift;
    };
    // pole to pole, the equator twice when the halves part: once for each half, the cylinder between
    Ring rings[RINGS + 2];
    int count = 0;
    for (int k = 0; k <= RINGS / 2; k++) rings[count++] = {PI * (float)k / (float)RINGS, half_height};
    for (int k = half_height > 0.0f ? RINGS / 2 : RINGS / 2 + 1; k <= RINGS; k++) rings[count++] = {PI * (float)k / (float)RINGS, -half_height};
    const auto direction = [](float theta, float phi) { return Float3{sinf(theta) * cosf(phi), cosf(theta), sinf(theta) * sinf(phi)}; };
    for (int r = 0; r + 1 < count; r++) {
        const Ring upper = rings[r], lower = rings[r + 1];
        for (int segment = 0; segment < SEGMENTS; segment++) {
            const float phi0 = 2.0f * PI * (float)segment / (float)SEGMENTS;
            const float phi1 = 2.0f * PI * (float)(segment + 1) / (float)SEGMENTS;
            const Float3 n00 = direction(upper.theta, phi0), n01 = direction(upper.theta, phi1);
            const Float3 n10 = direction(lower.theta, phi0), n11 = direction(lower.theta, phi1);
            const Float3 up = {0, upper.lift, 0}, down = {0, lower.lift, 0};
            quad(builder, add(scale(n00, radius), up), add(scale(n01, radius), up), add(scale(n11, radius), down),
                 add(scale(n10, radius), down), n00, n01, n11, n10);
        }
    }
}

static void cylinder(Builder &builder, float radius, float half_height) {
    const Float3 up = {0, 1, 0}, down = {0, -1, 0};
    for (int segment = 0; segment < SEGMENTS; segment++) {
        const float phi0 = 2.0f * PI * (float)segment / (float)SEGMENTS;
        const float phi1 = 2.0f * PI * (float)(segment + 1) / (float)SEGMENTS;
        const Float3 n0 = {cosf(phi0), 0, sinf(phi0)}, n1 = {cosf(phi1), 0, sinf(phi1)};
        const Float3 b0 = {n0.x * radius, -half_height, n0.z * radius}, b1 = {n1.x * radius, -half_height, n1.z * radius};
        const Float3 t0 = {b0.x, half_height, b0.z}, t1 = {b1.x, half_height, b1.z};
        quad(builder, b0, t0, t1, b1, n0, n0, n1, n1);
        triangle(builder, {0, half_height, 0}, t1, t0, up);
        triangle(builder, {0, -half_height, 0}, b0, b1, down);
    }
}

static void convex_hull(Builder &builder, const convex_hull::ConvexHullData &hull) {
    const auto point = [&](uint32_t index) {
        const float *p = hull.point_positions.data + hull.vertex_indices[index] * 3;
        return Float3{p[0], p[1], p[2]};
    };
    for (uint32_t f = 0; f < hull.faces.length; f++) {
        const convex_hull::ConvexHullFace &face = hull.faces[f];
        const Float3 normal = hull.planes[f].normal;
        for (uint32_t k = 1; k + 1 < face.num_vertices; k++) {
            triangle(builder, point(face.first_vertex), point(face.first_vertex + k), point(face.first_vertex + k + 1), normal);
        }
    }
}

static void triangle_mesh(Builder &builder, const triangle_mesh_data::TriangleMeshData &mesh) {
    const auto point = [&](uint32_t index) { return Float3{mesh.positions[index * 3], mesh.positions[index * 3 + 1], mesh.positions[index * 3 + 2]}; };
    for (uint32_t i = 0; i < mesh.triangle_count; i++) {
        const triangle_mesh_data::Triangle &t = mesh.triangles[i];
        triangle(builder, point(t.index_a), point(t.index_b), point(t.index_c), t.normal);
    }
}

static void plane(Builder &builder, const shapes::PlaneShape &shape) {
    const Float3 normal = shape.plane.normal;
    const Float3 center = scale(normal, -shape.plane.constant);
    const Float4 turn = quat_from_to({0, 1, 0}, normal);
    const float e = shape.half_extent;
    const Float3 u = rotate(turn, {e, 0, 0}), v = rotate(turn, {0, 0, e});
    const Float3 a = subtract(subtract(center, u), v), b = add(subtract(center, u), v);
    const Float3 c = add(add(center, u), v), d = subtract(add(center, u), v);
    quad(builder, a, b, c, d, normal, normal, normal, normal);
}

static void shape(Builder &builder, const shapes::Shape *shape);

/** the inner shape placed by a local position and rotation, the scale passed on as it is */
static void child(Builder &builder, const shapes::Shape *inner, Float3 position, Float4 quaternion) {
    const Placement outer = builder.placement;
    builder.placement.position = add(outer.position, rotate(outer.quaternion, multiply(position, outer.scale)));
    builder.placement.quaternion = quat_multiply(outer.quaternion, quaternion);
    shape(builder, inner);
    builder.placement = outer;
}

static void shape(Builder &builder, const shapes::Shape *shape) {
    switch (shape->type) {
        case shapes::SHAPE_SPHERE:
            rounded(builder, shape->sphere.radius, 0.0f);
            break;
        case shapes::SHAPE_BOX:
            box(builder, shape->box.half_extents);
            break;
        case shapes::SHAPE_CAPSULE:
            rounded(builder, shape->capsule.radius, shape->capsule.half_height_of_cylinder);
            break;
        case shapes::SHAPE_CYLINDER:
            cylinder(builder, shape->cylinder.radius, shape->cylinder.half_height);
            break;
        case shapes::SHAPE_CONVEX_HULL:
            convex_hull(builder, *shape->convex_hull.data);
            break;
        case shapes::SHAPE_TRIANGLE_MESH:
            triangle_mesh(builder, *shape->triangle_mesh.data);
            break;
        case shapes::SHAPE_PLANE:
            plane(builder, shape->plane);
            break;
        case shapes::SHAPE_COMPOUND:
        case shapes::SHAPE_STATIC_COMPOUND: {
            const collections::Vec<compound::CompoundShapeChild> &children = compound::children_of(shape);
            for (uint32_t i = 0; i < children.length; i++) child(builder, children[i].shape, children[i].position, children[i].quaternion);
            break;
        }
        case shapes::SHAPE_TRANSFORMED:
            child(builder, shape->transformed.shape, shape->transformed.position, shape->transformed.quaternion);
            break;
        case shapes::SHAPE_SCALED: {
            const Float3 outer = builder.placement.scale;
            builder.placement.scale = multiply(outer, shape->scaled.scale);
            shape_mesh::shape(builder, shape->scaled.shape);
            builder.placement.scale = outer;
            break;
        }
        case shapes::SHAPE_OFFSET_CENTER_OF_MASS:
            shape_mesh::shape(builder, shape->offset_center_of_mass.shape);
            break;
        default:
            // empty, and user shapes: the debug wireframes draw those
            break;
    }
}

void build(collections::Vec<MeshVertex> &out, const shapes::Shape *root) {
    Builder builder = {out, {{0, 0, 0}, {0, 0, 0, 1}, {1, 1, 1}}};
    shape(builder, root);
}

}
