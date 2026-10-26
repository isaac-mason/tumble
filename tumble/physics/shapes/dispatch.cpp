#include "tumble/physics/shapes/dispatch.h"

#include "tumble/physics/collision/collide_shape_vs_shape.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/capsule.h"
#include "tumble/physics/shapes/compound.h"
#include "tumble/physics/shapes/convex.h"
#include "tumble/physics/shapes/convex_hull.h"
#include "tumble/physics/shapes/cylinder.h"
#include "tumble/physics/shapes/empty.h"
#include "tumble/physics/shapes/offset_center_of_mass.h"
#include "tumble/physics/shapes/plane.h"
#include "tumble/physics/shapes/scaled.h"
#include "tumble/physics/shapes/shapes.h"
#include "tumble/physics/shapes/sphere.h"
#include "tumble/physics/shapes/static_compound.h"
#include "tumble/physics/shapes/transformed.h"
#include "tumble/physics/shapes/triangle_mesh.h"
#include "tumble/util/assert.h"
#include "tumble/util/mem.h"

TUMBLE_NAMESPACE_BEGIN

namespace dispatch {

void init(const ShapeDef *user_defs, uint32_t user_def_count) {
    util::zero(shapes::shape_defs, sizeof(shapes::shape_defs));
    util::zero(shapes::collide_table, sizeof(shapes::collide_table));
    util::zero(shapes::cast_table, sizeof(shapes::cast_table));

    // every shape's own functions
    shapes::set_shape_def(sphere::def());
    shapes::set_shape_def(box::def());
    shapes::set_shape_def(capsule::def());
    shapes::set_shape_def(compound::def());
    shapes::set_shape_def(convex_hull::def());
    shapes::set_shape_def(cylinder::def());
    shapes::set_shape_def(empty::def());
    shapes::set_shape_def(offset_center_of_mass::def());
    shapes::set_shape_def(plane::def());
    shapes::set_shape_def(scaled::def());
    shapes::set_shape_def(static_compound::def());
    shapes::set_shape_def(transformed::def());
    shapes::set_shape_def(triangle_mesh::def());
    for (uint32_t i = 0; i < user_def_count; i++) {
        TUMBLE_ASSERT(shapes::is_user_shape_type(user_defs[i].type));
        shapes::set_shape_def(user_defs[i]);
    }

    // the pair tables in full: the generic rule first, then every specialised pair over it

    // any two convex shapes: gjk and epa
    for (uint32_t i = 0; i < shapes::SHAPE_TYPE_COUNT; i++) {
        for (uint32_t j = 0; j < shapes::SHAPE_TYPE_COUNT; j++) {
            const ShapeDef &a = shapes::shape_defs[i];
            const ShapeDef &b = shapes::shape_defs[j];
            if (a.set_support == nullptr || b.set_support == nullptr) continue;
            shapes::set_collide_shape_fn(a.type, b.type, convex::collide_convex_vs_convex);
            shapes::set_cast_shape_fn(a.type, b.type, convex::cast_convex_vs_convex);
        }
    }

    // sphere vs sphere: the distance between the centres
    shapes::set_collide_shape_fn(ShapeType::SPHERE, ShapeType::SPHERE, sphere::collide_sphere_vs_sphere);
    // sphere vs box: the closed-form clamp, both ways
    shapes::set_collide_shape_fn(ShapeType::SPHERE, ShapeType::BOX, box::collide_sphere_vs_box);
    shapes::set_collide_shape_fn(ShapeType::BOX, ShapeType::SPHERE, collide_shape::reversed_collide_shape_vs_shape<box::collide_sphere_vs_box>);

    // convex vs triangle mesh, both ways: the mesh's bvh walk, the reverse through the swapping wrapper
    for (uint32_t i = 0; i < shapes::SHAPE_TYPE_COUNT; i++) {
        const ShapeDef &convex_def = shapes::shape_defs[i];
        if (convex_def.set_support == nullptr) continue;
        shapes::set_collide_shape_fn(convex_def.type, ShapeType::TRIANGLE_MESH, triangle_mesh::collide_convex_vs_triangle_mesh);
        shapes::set_collide_shape_fn(ShapeType::TRIANGLE_MESH, convex_def.type,
                                     collide_shape::reversed_collide_shape_vs_shape<triangle_mesh::collide_convex_vs_triangle_mesh>);
        shapes::set_cast_shape_fn(convex_def.type, ShapeType::TRIANGLE_MESH, triangle_mesh::cast_convex_vs_triangle_mesh);
        shapes::set_cast_shape_fn(ShapeType::TRIANGLE_MESH, convex_def.type,
                                  cast_shape::reversed_cast_shape_vs_shape<triangle_mesh::cast_convex_vs_triangle_mesh>);
    }
    // sphere vs triangle mesh: closest points and direct sweeps in place of gjk, both ways
    shapes::set_collide_shape_fn(ShapeType::SPHERE, ShapeType::TRIANGLE_MESH, triangle_mesh::collide_sphere_vs_triangle_mesh);
    shapes::set_collide_shape_fn(ShapeType::TRIANGLE_MESH, ShapeType::SPHERE,
                                 collide_shape::reversed_collide_shape_vs_shape<triangle_mesh::collide_sphere_vs_triangle_mesh>);
    shapes::set_cast_shape_fn(ShapeType::SPHERE, ShapeType::TRIANGLE_MESH, triangle_mesh::cast_sphere_vs_triangle_mesh);
    shapes::set_cast_shape_fn(ShapeType::TRIANGLE_MESH, ShapeType::SPHERE,
                              cast_shape::reversed_cast_shape_vs_shape<triangle_mesh::cast_sphere_vs_triangle_mesh>);

    // convex vs plane, both ways: the convex's support point against the plane, the reverse through the swapping wrapper
    for (uint32_t i = 0; i < shapes::SHAPE_TYPE_COUNT; i++) {
        const ShapeDef &convex_def = shapes::shape_defs[i];
        if (convex_def.set_support == nullptr) continue;
        shapes::set_collide_shape_fn(convex_def.type, ShapeType::PLANE, plane::collide_convex_vs_plane);
        shapes::set_collide_shape_fn(ShapeType::PLANE, convex_def.type,
                                     collide_shape::reversed_collide_shape_vs_shape<plane::collide_convex_vs_plane>);
        shapes::set_cast_shape_fn(convex_def.type, ShapeType::PLANE, plane::cast_convex_vs_plane);
        shapes::set_cast_shape_fn(ShapeType::PLANE, convex_def.type, cast_shape::reversed_cast_shape_vs_shape<plane::cast_convex_vs_plane>);
    }

    // compound vs anything, both ways: walk the compound's children. compound vs compound walks b's
    // children first, and the static compound's rows lie over the compound's where both meet. the empty shape keeps no
    // pairs: every child against it would find none
    for (uint32_t i = 0; i < shapes::SHAPE_TYPE_COUNT; i++) {
        const ShapeDef &other = shapes::shape_defs[i];
        if (other.get_surface_normal == nullptr || other.type == ShapeType::EMPTY) continue;
        shapes::set_collide_shape_fn(ShapeType::COMPOUND, other.type, compound::collide_compound_vs_shape);
        shapes::set_collide_shape_fn(other.type, ShapeType::COMPOUND, compound::collide_shape_vs_compound);
        shapes::set_cast_shape_fn(ShapeType::COMPOUND, other.type, compound::cast_compound_vs_shape);
        shapes::set_cast_shape_fn(other.type, ShapeType::COMPOUND, compound::cast_shape_vs_compound);
    }

    // the wrappers (transformed, scaled, offset centre of mass) vs anything, both ways: peel the wrapper and
    // dispatch its inner shape. these lie over the compound's rows and under the static compound's, and where two
    // wrappers meet the later one here peels first
    for (uint32_t i = 0; i < shapes::SHAPE_TYPE_COUNT; i++) {
        const ShapeDef &other = shapes::shape_defs[i];
        if (other.get_surface_normal == nullptr || other.type == ShapeType::EMPTY) continue;
        shapes::set_collide_shape_fn(ShapeType::TRANSFORMED, other.type, transformed::collide_transformed_vs_shape);
        shapes::set_collide_shape_fn(other.type, ShapeType::TRANSFORMED, transformed::collide_shape_vs_transformed);
        shapes::set_cast_shape_fn(ShapeType::TRANSFORMED, other.type, transformed::cast_transformed_vs_shape);
        shapes::set_cast_shape_fn(other.type, ShapeType::TRANSFORMED, transformed::cast_shape_vs_transformed);
    }
    for (uint32_t i = 0; i < shapes::SHAPE_TYPE_COUNT; i++) {
        const ShapeDef &other = shapes::shape_defs[i];
        if (other.get_surface_normal == nullptr || other.type == ShapeType::EMPTY) continue;
        shapes::set_collide_shape_fn(ShapeType::SCALED, other.type, scaled::collide_scaled_vs_shape);
        shapes::set_collide_shape_fn(other.type, ShapeType::SCALED, scaled::collide_shape_vs_scaled);
        shapes::set_cast_shape_fn(ShapeType::SCALED, other.type, scaled::cast_scaled_vs_shape);
        shapes::set_cast_shape_fn(other.type, ShapeType::SCALED, scaled::cast_shape_vs_scaled);
    }
    for (uint32_t i = 0; i < shapes::SHAPE_TYPE_COUNT; i++) {
        const ShapeDef &other = shapes::shape_defs[i];
        if (other.get_surface_normal == nullptr || other.type == ShapeType::EMPTY) continue;
        shapes::set_collide_shape_fn(ShapeType::OFFSET_CENTER_OF_MASS, other.type,
                                     offset_center_of_mass::collide_offset_center_of_mass_vs_shape);
        shapes::set_collide_shape_fn(other.type, ShapeType::OFFSET_CENTER_OF_MASS,
                                     offset_center_of_mass::collide_shape_vs_offset_center_of_mass);
        shapes::set_cast_shape_fn(ShapeType::OFFSET_CENTER_OF_MASS, other.type, offset_center_of_mass::cast_offset_center_of_mass_vs_shape);
        shapes::set_cast_shape_fn(other.type, ShapeType::OFFSET_CENTER_OF_MASS, offset_center_of_mass::cast_shape_vs_offset_center_of_mass);
    }

    for (uint32_t i = 0; i < shapes::SHAPE_TYPE_COUNT; i++) {
        const ShapeDef &other = shapes::shape_defs[i];
        if (other.get_surface_normal == nullptr || other.type == ShapeType::EMPTY) continue;
        shapes::set_collide_shape_fn(ShapeType::STATIC_COMPOUND, other.type, static_compound::collide_static_compound_vs_shape);
        shapes::set_collide_shape_fn(other.type, ShapeType::STATIC_COMPOUND, static_compound::collide_shape_vs_static_compound);
        shapes::set_cast_shape_fn(ShapeType::STATIC_COMPOUND, other.type, static_compound::cast_static_compound_vs_shape);
        shapes::set_cast_shape_fn(other.type, ShapeType::STATIC_COMPOUND, static_compound::cast_shape_vs_static_compound);
    }
}

}  // namespace dispatch

TUMBLE_NAMESPACE_END
