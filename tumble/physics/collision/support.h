#pragma once

// crashcat's support.ts, the struct: what gjk and epa evaluate. a shape's fill installs its
// evaluator on the struct next to its parameters, and the driver calls
// `support.get_support(support, direction)` many times per pair. the evaluators and the
// hull search are support.cpp.
//
// radius contract, as the ts: `convex_radius` is reported and never added by an evaluator (the
// driver passes it to gjk/epa); `add_radius` is added along the local direction by the evaluator
// itself (epa speculative separation, cast convex radius), 0 on the gjk path. the mode (include or
// exclude the convex radius) is baked into the params by the fill.

#include <stdint.h>

#include "tumble/math/types.h"
#include "tumble/tumble.h"
#include "tumble/util/collections.h"

TUMBLE_NAMESPACE_BEGIN

namespace support {

constexpr float DEFAULT_CONVEX_RADIUS = 0.05f;

/** convex_hull::MAX_POINTS_IN_HULL, which convex_hull.h asserts; the hull builder never produces more */
constexpr uint32_t MAX_HULL_POINTS = 256;

enum SupportFunctionMode : uint32_t {
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
        float scratch[MAX_HULL_POINTS * 3];
        /** borrowed csr neighbour prefix offsets (shape-owned, vertex_count + 1 long); null means brute scan */
        const uint32_t *neighbors_start;
        /** borrowed csr flat neighbour indices, indexed through neighbors_start */
        const uint32_t *neighbors;
        /** warm start: the last winning vertex, carried across calls within one pair; -1 when cold */
        int32_t last_vertex;
    } hull;
};

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
void compute_shrunk_hull_points(collections::Vec<float> &dst, uint32_t num_points, const float *point_positions, const uint32_t *point_num_faces,
                                const int32_t *point_faces, const Plane3f *planes, float convex_radius);
/** the shrunk vertices at a non-uniform or mirrored `scale` into `dst` (num_points * 3 long): the hull fill's slow path into `hull.scratch`. */
void compute_scaled_shrunk_hull_points(float *dst, uint32_t num_points, const float *point_positions,
                                       const uint32_t *point_num_faces, const int32_t *point_faces, const Plane3f *planes,
                                       float convex_radius, Vec3f scale);
/** a convex radius at `scale`: scaled by the smallest absolute component. */
float scale_convex_radius(float radius, Vec3f scale);

}  // namespace support

TUMBLE_NAMESPACE_END
