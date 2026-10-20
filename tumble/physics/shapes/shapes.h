#pragma once

#include <stdint.h>

#include "tumble/math/types.h"
#include "tumble/physics/body/mass_properties.h"
#include "tumble/physics/body/sub_shape.h"
#include "tumble/physics/collision/cast_ray_vs_shape.h"
#include "tumble/physics/collision/cast_shape_vs_shape.h"
#include "tumble/physics/collision/collide_point_vs_shape.h"
#include "tumble/physics/collision/collide_shape_vs_shape.h"
#include "tumble/physics/collision/support.h"
#include "tumble/physics/utils/face.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

// the heavy data the per-type payloads point at, each defined by its shape's own header.
namespace convex_hull {
struct ConvexHullData;
}
namespace triangle_mesh_data {
struct TriangleMeshData;
}
namespace compound {
struct CompoundData;
}
namespace static_compound {
struct StaticCompoundData;
}
// a shape is the fields every shape has, then a union of the per-type payloads: small shapes inline, the heavy ones
// (hull, mesh, compound) pointing at data many shapes can share, the wrappers holding the shape they wrap. read a
// payload once `type` says which. each type's functions live in the namespace named after its file, and dispatch.cpp
// gathers every def() into shape_defs and fills the collide and cast tables
namespace shapes {

enum ShapeType : uint32_t {
    SHAPE_SPHERE = 0,
    SHAPE_BOX = 1,
    SHAPE_CAPSULE = 2,
    SHAPE_CONVEX_HULL = 3,
    SHAPE_TRIANGLE_MESH = 4,
    SHAPE_COMPOUND = 5,
    SHAPE_TRANSFORMED = 6,
    SHAPE_SCALED = 7,
    SHAPE_EMPTY = 8,
    SHAPE_CYLINDER = 9,
    SHAPE_OFFSET_CENTER_OF_MASS = 10,
    SHAPE_PLANE = 11,
    SHAPE_STATIC_COMPOUND = 12,
};

/** the built-ins, as the tables are sized. */
constexpr uint32_t SHAPE_TYPE_COUNT = 13;

/** a type's row in the tables. */
inline uint32_t shape_type_index(ShapeType type) { return (uint32_t)type; }

constexpr float DEFAULT_SHAPE_DENSITY = 1000.0f;  // kg/m^3

struct Shape;

// the per-type payloads.
struct SphereShape {
    float radius;
    float density;
};

struct BoxShape {
    Float3 half_extents;
    float convex_radius;
    float density;
};

struct CapsuleShape {
    /** half height of the central cylinder, the hemispheres excluded */
    float half_height_of_cylinder;
    /** the radius, which is the convex radius */
    float radius;
    float density;
};

struct CylinderShape {
    float half_height;
    float radius;
    float convex_radius;
    float density;
};

struct PlaneShape {
    Plane3f plane;
    /** the bounded extent the plane is treated as having */
    float half_extent;
};

struct ConvexHullShape {
    /** the caller's, shared by any number of shapes; the shape never frees it */
    convex_hull::ConvexHullData *data;
    float density;
};

struct TriangleMeshShape {
    triangle_mesh_data::TriangleMeshData *data;
};

struct CompoundShape {
    compound::CompoundData *data;
};

struct StaticCompoundShape {
    static_compound::StaticCompoundData *data;
};

struct TransformedShape {
    Shape *shape;
    Float3 position;
    Float4 quaternion;
};

struct ScaledShape {
    Shape *shape;
    Float3 scale;
};

struct OffsetCenterOfMassShape {
    Shape *shape;
    Float3 offset;
};

struct Shape {
    ShapeType type;
    /** local bounds */
    Box3f aabb;
    Float3 center_of_mass;
    float volume;
    /** the surface material a hit on this shape reports; -1 for none. a mesh carries its own per triangle */
    int32_t material_id;
    union {
        SphereShape sphere;
        BoxShape box;
        CapsuleShape capsule;
        CylinderShape cylinder;
        PlaneShape plane;
        ConvexHullShape convex_hull;
        TriangleMeshShape triangle_mesh;
        CompoundShape compound;
        StaticCompoundShape static_compound;
        TransformedShape transformed;
        ScaledShape scaled;
        OffsetCenterOfMassShape offset_center_of_mass;
    };
};

struct SurfaceNormalResult {
    Vec3f normal;
    /** accumulated through the sub-shape hierarchy */
    Vec3f position;
    Vec3f scale;
};

struct SupportingFaceResult {
    face::Face face;
    /** world transform of the shape: rotation and translation, no scale */
    Mat4f transform;
    Vec3f scale;
};

struct GetLeafShapeResult {
    /** null when the path did not resolve */
    const Shape *shape;
    sub_shape::SubShapeId remainder;
};

struct GetSubShapeTransformedShapeResult {
    const Shape *shape;
    /** accumulated through the hierarchy */
    Vec3f position;
    Quatf rotation;
    Vec3f scale;
    sub_shape::SubShapeId remainder;
};

// the per-type functions, as `def()` of each shape file provides them

typedef void (*ComputeMassPropertiesFn)(mass_properties::MassProperties &out, const Shape *shape);
typedef void (*GetSurfaceNormalFn)(SurfaceNormalResult &io, const Shape *shape, sub_shape::SubShapeId sub_shape_id);
typedef void (*GetSupportingFaceFn)(SupportingFaceResult &io, Vec3f direction, const Shape *shape, sub_shape::SubShapeId sub_shape_id);
typedef int32_t (*GetMaterialFn)(const Shape *shape, sub_shape::SubShapeId sub_shape_id);
typedef float (*GetInnerRadiusFn)(const Shape *shape);
typedef void (*GetLeafShapeFn)(GetLeafShapeResult &out, const Shape *shape, sub_shape::SubShapeId sub_shape_id);
typedef void (*GetSubShapeTransformedShapeFn)(GetSubShapeTransformedShapeResult &out, const Shape *shape, sub_shape::SubShapeId sub_shape_id);
/** fills a support for a convex shape in `mode` at `scale`; the convex narrowphase's one view of a shape */
typedef void (*SetSupportFn)(support::Support &out, const Shape *shape, support::SupportFunctionMode mode, Vec3f scale);
typedef void (*CastRayFn)(cast_ray::CastRayCollector &collector, const cast_ray::CastRaySettings &settings, const cast_ray::Rayf &ray,
                          const Shape *shape, const sub_shape::SubShapeIdBuilder &sub_shape_id, const Transformf &transform);
typedef void (*CollidePointFn)(collide_point::CollidePointCollector &collector, const collide_point::CollidePointSettings &settings,
                               Vec3f point, const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b,
                               const Transformf &transform_b);

/** collides a against b; the pair's entry in the collide table */
typedef void (*CollideShapeVsShapeFn)(collide_shape::CollideShapeCollector &collector, const collide_shape::CollideShapeSettings &settings,
                                      const Shape *shape_a, const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                                      const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);
/** sweeps a by `displacement_a` against b; the pair's entry in the cast table */
typedef void (*CastShapeVsShapeFn)(cast_shape::CastShapeCollector &collector, const cast_shape::CastShapeSettings &settings,
                                   const Shape *shape_a, const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                                   Vec3f displacement_a, const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b,
                                   const Transformf &transform_b);

struct ShapeDef {
    ShapeType type;
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
};

/** what a def leaves null filled in: unit mass, zero inner radius, a leaf that is itself */
void default_compute_mass_properties(mass_properties::MassProperties &out, const Shape *shape);
float default_get_inner_radius(const Shape *shape);
void default_get_leaf_shape(GetLeafShapeResult &out, const Shape *shape, sub_shape::SubShapeId sub_shape_id);
void default_get_sub_shape_transformed_shape(GetSubShapeTransformedShapeResult &out, const Shape *shape, sub_shape::SubShapeId sub_shape_id);

/** a def with the defaults in place of what a shape file leaves null; the rest is the file's. */
ShapeDef define_shape(const ShapeDef &def);

/** the tables, filled once by dispatch.cpp */
extern ShapeDef shape_defs[SHAPE_TYPE_COUNT];
extern CollideShapeVsShapeFn collide_table[SHAPE_TYPE_COUNT][SHAPE_TYPE_COUNT];
extern CastShapeVsShapeFn cast_table[SHAPE_TYPE_COUNT][SHAPE_TYPE_COUNT];

inline const ShapeDef &def_of(const Shape *shape) { return shape_defs[shape_type_index(shape->type)]; }
inline bool is_convex(const Shape *shape) { return def_of(shape).set_support != nullptr; }

void set_shape_def(const ShapeDef &def);
/** a handler for the pair; a later set for the same pair replaces an earlier one */
void set_collide_shape_fn(ShapeType type_a, ShapeType type_b, CollideShapeVsShapeFn fn);
void set_cast_shape_fn(ShapeType type_a, ShapeType type_b, CastShapeVsShapeFn fn);
inline CollideShapeVsShapeFn collide_fn(ShapeType a, ShapeType b) { return collide_table[shape_type_index(a)][shape_type_index(b)]; }
inline CastShapeVsShapeFn cast_fn(ShapeType a, ShapeType b) { return cast_table[shape_type_index(a)][shape_type_index(b)]; }

/** a zeroed shape of `type`, or null (error set) when the memory cannot grow; the type's create fills it. */
Shape *allocate(ShapeType type);
/** frees the shape's own memory; a type's destroy frees what it points at first. */
void release(Shape *shape);

// the calls every shape answers, through its def

/** fills a support for a convex shape; only reached through a dispatch entry that exists for convex types */
inline void set_shape_support(support::Support &out, const Shape *shape, support::SupportFunctionMode mode, Vec3f scale) {
    def_of(shape).set_support(out, shape, mode, scale);
}
/** the surface normal at `position` on the sub-shape, local space */
Vec3f get_shape_surface_normal(const Shape *shape, Vec3f position, sub_shape::SubShapeId sub_shape_id);
mass_properties::MassProperties &compute_mass_properties(mass_properties::MassProperties &out, const Shape *shape);
/** the supporting face in `local_direction`, its vertices in world space through `transform` (rotation + translation) and `scale` */
void get_shape_supporting_face(face::Face &out, const Shape *shape, sub_shape::SubShapeId sub_shape_id, Vec3f local_direction,
                               const Mat4f &transform, Vec3f scale);
float get_shape_inner_radius(const Shape *shape);
/**
 * a scale in a parent's space as a child rotated by `rotation` sees it: the diagonal of
 * R^T S R. a uniform scale or an identity rotation (q or -q) passes through unchanged; a non-uniform scale is only
 * exact when the rotation keeps the axes aligned.
 */
Vec3f transform_scale(Quatf rotation, Vec3f scale);
/** the material id of the leaf a sub shape id names, -1 for none */
int32_t get_shape_material(const Shape *shape, sub_shape::SubShapeId sub_shape_id);
void get_leaf_shape(GetLeafShapeResult &out, const Shape *shape, sub_shape::SubShapeId sub_shape_id);
void get_sub_shape_transformed_shape(GetSubShapeTransformedShapeResult &out, const Shape *shape, sub_shape::SubShapeId sub_shape_id);

}  // namespace shapes

TUMBLE_NAMESPACE_END
