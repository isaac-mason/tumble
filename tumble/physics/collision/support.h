#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/math/mat4f.h"
#include "tumble/math/scalar.h"
#include "tumble/math/types.h"
#include "tumble/math/vec3f.h"
#include "tumble/util/collections.h"

TUMBLE_NAMESPACE_BEGIN

// what gjk and epa evaluate: a shape's fill installs its evaluator beside its parameters. convex_radius is reported and
// never added by an evaluator (the driver passes it on); add_radius is added along the direction by the evaluator
// itself (epa's speculative separation, a cast's convex radius) and is 0 on the gjk path. whether the convex radius is
// included is baked into the parameters by the fill

namespace support {

constexpr float DEFAULT_CONVEX_RADIUS = 0.05f;

/** convex_hull::MAX_POINTS_IN_HULL, which convex_hull.h asserts; the hull builder never produces more */
constexpr uint32_t MAX_HULL_POINTS = 256;

}  // namespace support

enum class SupportFunctionMode : uint32_t {
    INCLUDE_CONVEX_RADIUS = 0,
    EXCLUDE_CONVEX_RADIUS = 1,
    DEFAULT = 2,
};

struct Support;

/** the support point of `support` in `direction`. */
typedef Vec3f (*SupportFunction)(Support &support, Vec3f direction);

struct Support {
    SupportFunction get_support;
    /** reported convex radius; 0 in include mode */
    float convex_radius;
    /** extra radius the evaluator adds along the local direction */
    float add_radius;
    bool has_transform;
    /** b-in-a transform, applied when has_transform */
    Mat4f transform;

    /** axis-aligned box: the corner picked by the direction's signs */
    struct {
        Vec3f half_extents;
    } box;
    /** sphere core: the origin rounded by `radius` (0 in exclude mode) */
    struct {
        float radius;
    } sphere;
    /** capsule core: a segment of half-length along local y, rounded by `radius` */
    struct {
        float half_height;
        float radius;
    } capsule;
    /** cylinder: radial extent in the local xz plane, axial extent along y */
    struct {
        float radius;
        float half_height;
    } cylinder;
    /** triangle operand: three vertices, copied in by the fill */
    struct {
        Vec3f vertices[3];
    } triangle;
    /** single point operand, copied in by the fill */
    Vec3f point;
    /** convex vertex set (a hull, or a borrowed polygon face): the vertex with the greatest dot against the direction */
    struct {
        /** flat xyz vertices scanned by the evaluator; a borrow valid for the current pair */
        const float *vertices;
        uint32_t vertex_count;
        /** per-vertex output scale: the winning vertex is multiplied by it (uniform-scale fast path); 1 when `vertices` are baked */
        float output_scale;
        /** inline flat xyz buffer `vertices` points at on the non-uniform or mirrored scale slow path */
        float scratch[support::MAX_HULL_POINTS * 3];
        /** borrowed csr neighbour prefix offsets (shape-owned, vertex_count + 1 long); null means brute scan */
        const uint32_t *neighbors_start;
        /** borrowed csr flat neighbour indices, indexed through neighbors_start */
        const uint32_t *neighbors;
        /** warm start: the last winning vertex, carried across calls within one pair; -1 when cold */
        int32_t last_vertex;
    } hull;
    /** a user shape type's operand: what its set_support leaves for its own evaluator */
    struct {
        const void *data;
        Vec3f scale;
    } user;
};

namespace support {

// the steps of every evaluator, a user type's too: the direction into local space, the core's support point there,
// rounded by add_radius along the local direction, and back into the caller's space

/** the direction into local space (inverse rotation = transposed 3x3) */
inline Vec3f to_local_direction(const Support &support, Vec3f direction) {
    return support.has_transform ? mat4f::multiply3x3_transposed_vec(support.transform, direction) : direction;
}

/** rounds `point` by `radius` along the local direction */
inline Vec3f add_radius_along(Vec3f point, Vec3f direction, float radius) {
    if (radius > 0.0f) {
        const float length_sq = vec3f::squared_length(direction);
        if (length_sq > 0.0f) return vec3f::scale_and_add(point, direction, radius / scalar::sqrt(length_sq));
    }
    return point;
}

/** the local point back to the caller's space */
inline Vec3f to_caller_space(const Support &support, Vec3f point) {
    if (!support.has_transform) return point;
    return {vec3f::fix_w(v4f::add(mat4f::combine3(support.transform, point.v), support.transform.c[3]))};
}

/** a support as a zero-radius sphere at the origin. */
Support create();

// the evaluators, installed by the shape fills (support.cpp)
Vec3f sphere_support(Support &support, Vec3f direction);
Vec3f box_support(Support &support, Vec3f direction);
Vec3f capsule_support(Support &support, Vec3f direction);
Vec3f cylinder_support(Support &support, Vec3f direction);
Vec3f triangle_support(Support &support, Vec3f direction);
Vec3f point_support(Support &support, Vec3f direction);
Vec3f hull_support(Support &support, Vec3f direction);

// the fills for operands that are not shapes (support.cpp); shape fills live with their shapes
void set_triangle_support(Support &out, Vec3f a, Vec3f b, Vec3f c, float convex_radius);
void set_point_support(Support &out, Vec3f point);
void set_polygon_support(Support &out, const Vec3f *vertices, uint32_t vertex_count, float convex_radius);

// the hull geometry convex_hull's create and fill read (support.cpp). per point, `point_num_faces`
// (1..3) neighbouring faces listed 3 per point in `point_faces` (-1 unused), indexing `planes`.

/** the convex-radius-shrunk vertices, unscaled, into `dst` as flat xyz (grown to fit). */
void compute_shrunk_hull_points(collections::Array<float> &dst, uint32_t num_points, const float *point_positions, const uint32_t *point_num_faces,
                                const int32_t *point_faces, const Plane3f *planes, float convex_radius);
/** the shrunk vertices at a non-uniform or mirrored `scale` into `dst` (num_points * 3 long): the hull fill's slow path into `hull.scratch`. */
void compute_scaled_shrunk_hull_points(float *dst, uint32_t num_points, const float *point_positions,
                                       const uint32_t *point_num_faces, const int32_t *point_faces, const Plane3f *planes,
                                       float convex_radius, Vec3f scale);
/** a convex radius at `scale`: scaled by the smallest absolute component. */
float scale_convex_radius(float radius, Vec3f scale);

}  // namespace support

TUMBLE_NAMESPACE_END
