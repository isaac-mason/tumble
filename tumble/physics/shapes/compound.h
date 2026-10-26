#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/physics/shapes/shapes.h"
#include "tumble/util/collections.h"

TUMBLE_NAMESPACE_BEGIN

struct CompoundShape : Shape {
    /** a child as the compound keeps it */
    struct Child {
        Float3 position;
        Float4 quaternion;
        Ref<Shape> shape;
    };

      // namespace compound

    /** the child list, owned by the compound shape: copied from the settings by create, freed with it */
    struct Data {
        collections::Array<Child> children;
        /** each child's bounds in the compound's unscaled space, for culling children against a query; filled by update */
        collections::Array<Box3f> child_bounds;
    };

    static constexpr ShapeType TYPE = ShapeType::COMPOUND;
    Data *data;
};

struct CompoundShapeSettings {
    // child shapes, each placed by a position and rotation; a sub shape id's level indexes the children. the static
    // compound shares the functions that only walk the child list

    /** a child as the settings give it; the compound holds the shape */
    struct Child {
        Float3 position;
        Float4 quaternion;
        Shape *shape;
    };

    /** read during create only; the compound holds the shapes they point at */
    const Child *children = nullptr;
    /** at least one */
    uint32_t num_children = 0;
};

namespace compound {

/** the settings' child as the compound keeps it, holding its shape */
inline CompoundShape::Child held_child(const CompoundShapeSettings::Child &child) { return {child.position, child.quaternion, child.shape}; }

}
namespace compound {

Shape *create(const CompoundShapeSettings &settings);
/** bounds, centre of mass and volume from the children; call after changing the child list */
void update(Shape *shape);

/** the child list of a compound or a static compound */
const collections::Array<CompoundShape::Child> &children_of(const Shape *shape);
/** a child's world transform: the parent's scale applies to the child's position and passes on unchanged */
void child_transform(Transformf &out, const Transformf &parent, const CompoundShape::Child &child);

/** the children's mass properties summed about the compound's centre of mass */
void compute_mass_properties(MassProperties &out, const Shape *shape);
void get_surface_normal(SurfaceNormalResult &io, const Shape *shape, SubShapeId sub_shape_id);
void get_supporting_face(SupportingFaceResult &io, Vec3f direction, const Shape *shape, SubShapeId sub_shape_id);
/** the material of the child's leaf */
int32_t get_material(const Shape *shape, SubShapeId sub_shape_id);
/** the smallest child's */
float get_inner_radius(const Shape *shape);
void get_leaf_shape(GetLeafShapeResult &out, const Shape *shape, SubShapeId sub_shape_id);
void get_sub_shape_transformed_shape(GetSubShapeTransformedShapeResult &out, const Shape *shape, SubShapeId sub_shape_id);

void cast_ray_vs_compound(CastRayCollector &collector, const CastRaySettings &settings, const Rayf &ray,
                          const Shape *shape, const SubShapeIdBuilder &sub_shape_id, const Transformf &transform);
void collide_point_vs_compound(CollidePointCollector &collector, const CollidePointSettings &settings,
                               Vec3f point, const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b,
                               const Transformf &transform_b);

/** each of a's children against b, through the collide table */
void collide_compound_vs_shape(CollideShapeCollector &collector, const CollideShapeSettings &settings,
                               const Shape *shape_a, const SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                               const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);
/** a against each of b's children, through the collide table */
void collide_shape_vs_compound(CollideShapeCollector &collector, const CollideShapeSettings &settings,
                               const Shape *shape_a, const SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                               const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);
void cast_compound_vs_shape(CastShapeCollector &collector, const CastShapeSettings &settings, const Shape *shape_a,
                            const SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a, Vec3f displacement_a,
                            const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);
void cast_shape_vs_compound(CastShapeCollector &collector, const CastShapeSettings &settings, const Shape *shape_a,
                            const SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a, Vec3f displacement_a,
                            const Shape *shape_b, const SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b);

ShapeDef def();

}  // namespace compound

TUMBLE_NAMESPACE_END
