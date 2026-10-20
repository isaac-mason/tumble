#include "physics/shapes/dispatch.h"

#include "physics/collision/collide_shape_vs_shape.h"
#include "physics/shapes/box.h"
#include "physics/shapes/capsule.h"
#include "physics/shapes/compound.h"
#include "physics/shapes/convex.h"
#include "physics/shapes/convex_hull.h"
#include "physics/shapes/cylinder.h"
#include "physics/shapes/empty.h"
#include "physics/shapes/offset_center_of_mass.h"
#include "physics/shapes/plane.h"
#include "physics/shapes/scaled.h"
#include "physics/shapes/shapes.h"
#include "physics/shapes/sphere.h"
#include "physics/shapes/static_compound.h"
#include "physics/shapes/transformed.h"
#include "physics/shapes/triangle_mesh.h"
#include "util/mem.h"

namespace dispatch {

void init() {
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

    // the pair tables, laid out here in full rather than registered shape by shape (crashcat's register() per shape
    // only gives the right table because js visits integer keys in order). the generic rule first, then every
    // specialised pair over it; a later shape adds its rows below, no order to get right.

    // any two convex shapes: gjk and epa
    for (uint32_t i = 0; i < shapes::SHAPE_TYPE_COUNT; i++) {
        for (uint32_t j = 0; j < shapes::SHAPE_TYPE_COUNT; j++) {
            const shapes::ShapeDef &a = shapes::shape_defs[i];
            const shapes::ShapeDef &b = shapes::shape_defs[j];
            if (a.set_support == nullptr || b.set_support == nullptr) continue;
            shapes::set_collide_shape_fn(a.type, b.type, convex::collide_convex_vs_convex);
            shapes::set_cast_shape_fn(a.type, b.type, convex::cast_convex_vs_convex);
        }
    }

    // sphere vs sphere: the distance between the centres
    shapes::set_collide_shape_fn(shapes::SHAPE_SPHERE, shapes::SHAPE_SPHERE, sphere::collide_sphere_vs_sphere);
    // sphere vs box: the closed-form clamp, both ways
    shapes::set_collide_shape_fn(shapes::SHAPE_SPHERE, shapes::SHAPE_BOX, box::collide_sphere_vs_box);
    shapes::set_collide_shape_fn(shapes::SHAPE_BOX, shapes::SHAPE_SPHERE, collide_shape::reversed_collide_shape_vs_shape<box::collide_sphere_vs_box>);

    // convex vs triangle mesh, both ways: the mesh's bvh walk, the reverse through the swapping wrapper
    for (uint32_t i = 0; i < shapes::SHAPE_TYPE_COUNT; i++) {
        const shapes::ShapeDef &convex_def = shapes::shape_defs[i];
        if (convex_def.set_support == nullptr) continue;
        shapes::set_collide_shape_fn(convex_def.type, shapes::SHAPE_TRIANGLE_MESH, triangle_mesh::collide_convex_vs_triangle_mesh);
        shapes::set_collide_shape_fn(shapes::SHAPE_TRIANGLE_MESH, convex_def.type,
                                     collide_shape::reversed_collide_shape_vs_shape<triangle_mesh::collide_convex_vs_triangle_mesh>);
        shapes::set_cast_shape_fn(convex_def.type, shapes::SHAPE_TRIANGLE_MESH, triangle_mesh::cast_convex_vs_triangle_mesh);
        shapes::set_cast_shape_fn(shapes::SHAPE_TRIANGLE_MESH, convex_def.type,
                                  cast_shape::reversed_cast_shape_vs_shape<triangle_mesh::cast_convex_vs_triangle_mesh>);
    }
    // sphere vs triangle mesh: closest points and direct sweeps in place of gjk, both ways
    shapes::set_collide_shape_fn(shapes::SHAPE_SPHERE, shapes::SHAPE_TRIANGLE_MESH, triangle_mesh::collide_sphere_vs_triangle_mesh);
    shapes::set_collide_shape_fn(shapes::SHAPE_TRIANGLE_MESH, shapes::SHAPE_SPHERE,
                                 collide_shape::reversed_collide_shape_vs_shape<triangle_mesh::collide_sphere_vs_triangle_mesh>);
    shapes::set_cast_shape_fn(shapes::SHAPE_SPHERE, shapes::SHAPE_TRIANGLE_MESH, triangle_mesh::cast_sphere_vs_triangle_mesh);
    shapes::set_cast_shape_fn(shapes::SHAPE_TRIANGLE_MESH, shapes::SHAPE_SPHERE,
                              cast_shape::reversed_cast_shape_vs_shape<triangle_mesh::cast_sphere_vs_triangle_mesh>);

    // convex vs plane, both ways: the convex's support point against the plane, the reverse through the swapping wrapper
    for (uint32_t i = 0; i < shapes::SHAPE_TYPE_COUNT; i++) {
        const shapes::ShapeDef &convex_def = shapes::shape_defs[i];
        if (convex_def.set_support == nullptr) continue;
        shapes::set_collide_shape_fn(convex_def.type, shapes::SHAPE_PLANE, plane::collide_convex_vs_plane);
        shapes::set_collide_shape_fn(shapes::SHAPE_PLANE, convex_def.type,
                                     collide_shape::reversed_collide_shape_vs_shape<plane::collide_convex_vs_plane>);
        shapes::set_cast_shape_fn(convex_def.type, shapes::SHAPE_PLANE, plane::cast_convex_vs_plane);
        shapes::set_cast_shape_fn(shapes::SHAPE_PLANE, convex_def.type, cast_shape::reversed_cast_shape_vs_shape<plane::cast_convex_vs_plane>);
    }

    // compound vs anything, both ways: walk the compound's children. compound vs compound walks b's
    // children first, and the static compound's rows lie over the compound's where both meet, as
    // crashcat's registration order leaves them. the empty shape keeps no pairs: every child against
    // it would find none
    for (uint32_t i = 0; i < shapes::SHAPE_TYPE_COUNT; i++) {
        const shapes::ShapeDef &other = shapes::shape_defs[i];
        if (other.get_surface_normal == nullptr || other.type == shapes::SHAPE_EMPTY) continue;
        shapes::set_collide_shape_fn(shapes::SHAPE_COMPOUND, other.type, compound::collide_compound_vs_shape);
        shapes::set_collide_shape_fn(other.type, shapes::SHAPE_COMPOUND, compound::collide_shape_vs_compound);
        shapes::set_cast_shape_fn(shapes::SHAPE_COMPOUND, other.type, compound::cast_compound_vs_shape);
        shapes::set_cast_shape_fn(other.type, shapes::SHAPE_COMPOUND, compound::cast_shape_vs_compound);
    }

    // the wrappers (transformed, scaled, offset centre of mass) vs anything, both ways: peel the wrapper and
    // dispatch its inner shape. as crashcat's registration order leaves them, these lie over the compound's
    // rows and under the static compound's, and where two wrappers meet the later one here peels first
    for (uint32_t i = 0; i < shapes::SHAPE_TYPE_COUNT; i++) {
        const shapes::ShapeDef &other = shapes::shape_defs[i];
        if (other.get_surface_normal == nullptr || other.type == shapes::SHAPE_EMPTY) continue;
        shapes::set_collide_shape_fn(shapes::SHAPE_TRANSFORMED, other.type, transformed::collide_transformed_vs_shape);
        shapes::set_collide_shape_fn(other.type, shapes::SHAPE_TRANSFORMED, transformed::collide_shape_vs_transformed);
        shapes::set_cast_shape_fn(shapes::SHAPE_TRANSFORMED, other.type, transformed::cast_transformed_vs_shape);
        shapes::set_cast_shape_fn(other.type, shapes::SHAPE_TRANSFORMED, transformed::cast_shape_vs_transformed);
    }
    for (uint32_t i = 0; i < shapes::SHAPE_TYPE_COUNT; i++) {
        const shapes::ShapeDef &other = shapes::shape_defs[i];
        if (other.get_surface_normal == nullptr || other.type == shapes::SHAPE_EMPTY) continue;
        shapes::set_collide_shape_fn(shapes::SHAPE_SCALED, other.type, scaled::collide_scaled_vs_shape);
        shapes::set_collide_shape_fn(other.type, shapes::SHAPE_SCALED, scaled::collide_shape_vs_scaled);
        shapes::set_cast_shape_fn(shapes::SHAPE_SCALED, other.type, scaled::cast_scaled_vs_shape);
        shapes::set_cast_shape_fn(other.type, shapes::SHAPE_SCALED, scaled::cast_shape_vs_scaled);
    }
    for (uint32_t i = 0; i < shapes::SHAPE_TYPE_COUNT; i++) {
        const shapes::ShapeDef &other = shapes::shape_defs[i];
        if (other.get_surface_normal == nullptr || other.type == shapes::SHAPE_EMPTY) continue;
        shapes::set_collide_shape_fn(shapes::SHAPE_OFFSET_CENTER_OF_MASS, other.type,
                                     offset_center_of_mass::collide_offset_center_of_mass_vs_shape);
        shapes::set_collide_shape_fn(other.type, shapes::SHAPE_OFFSET_CENTER_OF_MASS,
                                     offset_center_of_mass::collide_shape_vs_offset_center_of_mass);
        shapes::set_cast_shape_fn(shapes::SHAPE_OFFSET_CENTER_OF_MASS, other.type, offset_center_of_mass::cast_offset_center_of_mass_vs_shape);
        shapes::set_cast_shape_fn(other.type, shapes::SHAPE_OFFSET_CENTER_OF_MASS, offset_center_of_mass::cast_shape_vs_offset_center_of_mass);
    }

    for (uint32_t i = 0; i < shapes::SHAPE_TYPE_COUNT; i++) {
        const shapes::ShapeDef &other = shapes::shape_defs[i];
        if (other.get_surface_normal == nullptr || other.type == shapes::SHAPE_EMPTY) continue;
        shapes::set_collide_shape_fn(shapes::SHAPE_STATIC_COMPOUND, other.type, static_compound::collide_static_compound_vs_shape);
        shapes::set_collide_shape_fn(other.type, shapes::SHAPE_STATIC_COMPOUND, static_compound::collide_shape_vs_static_compound);
        shapes::set_cast_shape_fn(shapes::SHAPE_STATIC_COMPOUND, other.type, static_compound::cast_static_compound_vs_shape);
        shapes::set_cast_shape_fn(other.type, shapes::SHAPE_STATIC_COMPOUND, static_compound::cast_shape_vs_static_compound);
    }
}

}  // namespace dispatch
