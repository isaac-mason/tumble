#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/math/types.h"
#include "tumble/physics/body/mass_properties.h"
#include "tumble/physics/body/sub_shape.h"
#include "tumble/physics/collision/cast_ray_vs_shape.h"
#include "tumble/physics/collision/cast_shape_vs_shape.h"
#include "tumble/physics/collision/collide_point_vs_shape.h"
#include "tumble/physics/collision/collide_shape_vs_shape.h"
#include "tumble/physics/collision/support.h"
#include "tumble/physics/utils/face.h"
#include "tumble/util/assert.h"
#include "tumble/util/object.h"
#include "tumble/util/ref.h"

TUMBLE_NAMESPACE_BEGIN

struct World;
struct DebugDraw;
// a shape is the fields every shape has, and each type's own struct derives from it: small shapes keep their payload
// inline, the heavy ones (hull, mesh, compound) point at data many shapes can share, the wrappers hold the shape they
// wrap. shapes::as<T> reads a shape as its type's struct once `type` says which. each type's struct and functions live in
// the namespace named after its file, and dispatch.cpp gathers every def() into shape_defs and fills the collide and cast
// tables. a type of the caller's own is a struct deriving from Shape in a user slot, its def going to dispatch::init
// with the built-ins

enum class ShapeType : uint32_t {
    SPHERE = 0,
    BOX = 1,
    CAPSULE = 2,
    CONVEX_HULL = 3,
    TRIANGLE_MESH = 4,
    COMPOUND = 5,
    TRANSFORMED = 6,
    SCALED = 7,
    EMPTY = 8,
    CYLINDER = 9,
    OFFSET_CENTER_OF_MASS = 10,
    PLANE = 11,
    STATIC_COMPOUND = 12,
    USER_1 = 13,
    USER_2 = 14,
    USER_3 = 15,
    USER_4 = 16,
    USER_5 = 17,
    USER_6 = 18,
    USER_7 = 19,
    USER_8 = 20,
};

namespace shapes {

constexpr uint32_t USER_SHAPE_TYPE_COUNT = 8;
/** the built-ins and the user slots, as the tables are sized. */
constexpr uint32_t SHAPE_TYPE_COUNT = 21;

inline bool is_user_shape_type(ShapeType type) { return (uint32_t)type >= (uint32_t)ShapeType::USER_1 && (uint32_t)type < (uint32_t)ShapeType::USER_1 + USER_SHAPE_TYPE_COUNT; }

/** a type's row in the tables. */
inline uint32_t shape_type_index(ShapeType type) { return (uint32_t)type; }

constexpr float DEFAULT_SHAPE_DENSITY = 1000.0f;

}  // namespace shapes

  // kg/m^3

/**
 * the fields every shape has. a type's struct derives from it with a `static constexpr ShapeType TYPE`, and is made by
 * allocate<T>, read by as<T> and freed by release<T>. never copied: a copy would lose the type's fields
 */
struct Shape {
    ShapeType type;
    /** local bounds */
    Box3f aabb;
    Float3 center_of_mass;
    float volume;
    /** the surface material a hit on this shape reports; -1 for none. a mesh carries its own per triangle */
    int32_t material_id;
    /**
     * read only: how many Refs hold the shape, bodies', characters', the shapes built over it and the caller's. a new
     * shape starts at 0; the last Ref to let go frees it
     */
    mutable uint32_t ref_count;

    Shape() = default;
    Shape(const Shape &) = delete;
    Shape &operator=(const Shape &) = delete;
};

inline void add_ref(const Shape *shape) { __atomic_fetch_add(&shape->ref_count, 1u, __ATOMIC_RELAXED); }
void release_ref(const Shape *shape);

namespace shapes {

/** a shape of type T, zeroed but for its type; the type's create fills it */
template <typename T>
T *allocate() {
    T *shape = util::create<T>();
    shape->type = T::TYPE;
    return shape;
}

/** the shape as its type's struct, which `type` must be */
template <typename T>
T &as(Shape &shape) {
    TUMBLE_ASSERT(shape.type == T::TYPE);
    return static_cast<T &>(shape);
}
template <typename T>
const T &as(const Shape &shape) {
    TUMBLE_ASSERT(shape.type == T::TYPE);
    return static_cast<const T &>(shape);
}
template <typename T>
T &as(Shape *shape) {
    return as<T>(*shape);
}
template <typename T>
const T &as(const Shape *shape) {
    return as<T>(*shape);
}

/** another Ref holds the shape; Ref calls it */
/** a Ref lets go of the shape, the last one freeing it with its type's destroy; Ref calls it */

/** added to the count of an embedded shape, so letting go of it never reaches 0: jolt's value */
constexpr uint32_t EMBEDDED_REF_COUNT = 0x0ebedded;

/**
 * a shape on the stack or inside another object, which its owner frees itself: Refs to it count, but the last one never
 * frees it
 */
inline void set_embedded(const Shape *shape) { __atomic_fetch_add(&shape->ref_count, EMBEDDED_REF_COUNT, __ATOMIC_RELAXED); }

/** frees the shape's own memory, sized as T, which `type` must be; a type's destroy frees what it points at first */
template <typename T>
void release(Shape *shape) {
    util::destroy(&as<T>(shape));
}

}  // namespace shapes

struct SurfaceNormalResult {
    Vec3f normal;
    /** accumulated through the sub-shape hierarchy */
    Vec3f position;
    Vec3f scale;
};

struct SupportingFaceResult {
    SupportingFace face;
    /** world transform of the shape: rotation and translation, no scale */
    Mat4f transform;
    Vec3f scale;
};

struct GetLeafShapeResult {
    /** null when the path did not resolve */
    const Shape *shape;
    SubShapeId remainder;
};

struct GetSubShapeTransformedShapeResult {
    const Shape *shape;
    /** accumulated through the hierarchy */
    Vec3f position;
    Quatf rotation;
    Vec3f scale;
    SubShapeId remainder;
};

// the per-type functions, as `def()` of each shape file provides them

typedef void (*ComputeMassPropertiesFn)(MassProperties &out, const Shape *shape);
typedef void (*GetSurfaceNormalFn)(SurfaceNormalResult &io, const Shape *shape, SubShapeId sub_shape_id);
typedef void (*GetSupportingFaceFn)(SupportingFaceResult &io, Vec3f direction, const Shape *shape, SubShapeId sub_shape_id);
typedef int32_t (*GetMaterialFn)(const Shape *shape, SubShapeId sub_shape_id);
typedef float (*GetInnerRadiusFn)(const Shape *shape);
typedef void (*GetLeafShapeFn)(GetLeafShapeResult &out, const Shape *shape, SubShapeId sub_shape_id);
typedef void (*GetSubShapeTransformedShapeFn)(GetSubShapeTransformedShapeResult &out, const Shape *shape, SubShapeId sub_shape_id);
/** fills a support for a convex shape in `mode` at `scale`; the convex narrowphase's one view of a shape */
typedef void (*SetSupportFn)(Support &out, const Shape *shape, SupportFunctionMode mode, Vec3f scale);
typedef void (*CastRayFn)(CastRayCollector &collector, const CastRaySettings &settings, const Rayf &ray,
                          const Shape *shape, const SubShapeIdBuilder &sub_shape_id, const Transformf &transform);
typedef void (*CollidePointFn)(CollidePointCollector &collector, const CollidePointSettings &settings,
                               Vec3f point, const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b,
                               const Transformf &transform_b);

/** collides a against b; the pair's entry in the collide table */
typedef void (*CollideShapeVsShapeFn)(CollideShapeCollector &collector, const CollideShapeSettings &settings,
                                      const Shape *shape_a, const SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                                      const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);
/** sweeps a by `displacement_a` against b; the pair's entry in the cast table */
typedef void (*CastShapeVsShapeFn)(CastShapeCollector &collector, const CastShapeSettings &settings,
                                   const Shape *shape_a, const SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                                   Vec3f displacement_a, const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b,
                                   const Transformf &transform_b);

/** appends the shape's wireframe at `transform` (scale included) to `out`, through debug::line */
typedef void (*ShapeDrawFn)(DebugDraw &out, const Shape *shape, const Transformf &transform, Vec3f color);

/** frees a shape of the def's type, and lets go of what it holds; called by release_ref when no Ref holds it */
typedef void (*ShapeDestroyFn)(Shape *shape);

struct ShapeDef {
    ShapeType type;
    ShapeDestroyFn destroy;
    /** present for convex types only; null otherwise */
    SetSupportFn set_support;
    ComputeMassPropertiesFn compute_mass_properties;
    GetSurfaceNormalFn get_surface_normal;
    GetSupportingFaceFn get_supporting_face;
    /** the material id of the leaf a sub shape id names, -1 for none */
    GetMaterialFn get_material;
    /** the biggest sphere that fits entirely inside; the smallest part's for a compound */
    GetInnerRadiusFn get_inner_radius;
    GetLeafShapeFn get_leaf_shape;
    GetSubShapeTransformedShapeFn get_sub_shape_transformed_shape;
    CastRayFn cast_ray;
    CollidePointFn collide_point;
    /** null draws the shape's bounds */
    ShapeDrawFn draw;
    /**
     * the body pair cache rebuilds a pair's contacts from last step's when the bodies barely moved; a type whose sub
     * shape ids name state its owner renews between steps (as a pool of hits it clears) sets this so its pairs always
     * collide anew
     */
    bool skips_pair_cache;
};

namespace shapes {

/** what a def leaves null filled in: unit mass, zero inner radius, a leaf that is itself */
void default_compute_mass_properties(MassProperties &out, const Shape *shape);
float default_get_inner_radius(const Shape *shape);
void default_get_leaf_shape(GetLeafShapeResult &out, const Shape *shape, SubShapeId sub_shape_id);
void default_get_sub_shape_transformed_shape(GetSubShapeTransformedShapeResult &out, const Shape *shape, SubShapeId sub_shape_id);

/** a def with the defaults in place of what a shape file leaves null; the rest is the file's. */
ShapeDef define_shape(const ShapeDef &def);

/** the tables, filled once by dispatch.cpp */
extern ShapeDef shape_defs[SHAPE_TYPE_COUNT];
extern CollideShapeVsShapeFn collide_table[SHAPE_TYPE_COUNT][SHAPE_TYPE_COUNT];
extern CastShapeVsShapeFn cast_table[SHAPE_TYPE_COUNT][SHAPE_TYPE_COUNT];

inline const ShapeDef &def_of(const Shape *shape) { return shape_defs[shape_type_index(shape->type)]; }
inline bool is_convex(const Shape *shape) { return def_of(shape).set_support != nullptr; }
/** false when a leaf anywhere in the shape (under wrappers and compound children) skips the pair cache */
bool caches_pairs(const Shape *shape);

void set_shape_def(const ShapeDef &def);
/** a handler for the pair; a later set for the same pair replaces an earlier one */
void set_collide_shape_fn(ShapeType type_a, ShapeType type_b, CollideShapeVsShapeFn fn);
void set_cast_shape_fn(ShapeType type_a, ShapeType type_b, CastShapeVsShapeFn fn);
inline CollideShapeVsShapeFn collide_fn(ShapeType a, ShapeType b) { return collide_table[shape_type_index(a)][shape_type_index(b)]; }
inline CastShapeVsShapeFn cast_fn(ShapeType a, ShapeType b) { return cast_table[shape_type_index(a)][shape_type_index(b)]; }

// the calls every shape answers, through its def

/** fills a support for a convex shape; only reached through a dispatch entry that exists for convex types */
inline void set_shape_support(Support &out, const Shape *shape, SupportFunctionMode mode, Vec3f scale) {
    def_of(shape).set_support(out, shape, mode, scale);
}
/** the surface normal at `position` on the sub-shape, local space */
Vec3f get_shape_surface_normal(const Shape *shape, Vec3f position, SubShapeId sub_shape_id);
MassProperties &compute_mass_properties(MassProperties &out, const Shape *shape);
/** the supporting face in `local_direction`, its vertices in world space through `transform` (rotation + translation) and `scale` */
void get_shape_supporting_face(SupportingFace &out, const Shape *shape, SubShapeId sub_shape_id, Vec3f local_direction,
                               const Mat4f &transform, Vec3f scale);
float get_shape_inner_radius(const Shape *shape);
/**
 * a scale in a parent's space as a child rotated by `rotation` sees it: the diagonal of
 * R^T S R. a uniform scale or an identity rotation (q or -q) passes through unchanged; a non-uniform scale is only
 * exact when the rotation keeps the axes aligned.
 */
Vec3f transform_scale(Quatf rotation, Vec3f scale);
/** the material id of the leaf a sub shape id names, -1 for none */
int32_t get_shape_material(const Shape *shape, SubShapeId sub_shape_id);
void get_leaf_shape(GetLeafShapeResult &out, const Shape *shape, SubShapeId sub_shape_id);
void get_sub_shape_transformed_shape(GetSubShapeTransformedShapeResult &out, const Shape *shape, SubShapeId sub_shape_id);

}  // namespace shapes

TUMBLE_NAMESPACE_END
