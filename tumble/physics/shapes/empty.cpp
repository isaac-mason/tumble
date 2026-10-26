#include "tumble/physics/shapes/empty.h"

#include "tumble/math/aabox.h"
#include "tumble/math/mat4f.h"
#include "tumble/math/vec3f.h"

TUMBLE_NAMESPACE_BEGIN

namespace empty {

Shape *create() {
    EmptyShape *shape = shapes::allocate<EmptyShape>();
    // zero-size bounds at the origin
    shape->aabb = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
    vec3f::store(shape->center_of_mass, vec3f::zero());
    shape->volume = 0.0f;
    return shape;
}

static void destroy(Shape *shape) {
    shapes::release<EmptyShape>(shape);
}

void compute_mass_properties(MassProperties &out, const Shape *) {
    // unit mass and inertia, so a dynamic body without a shape still integrates
    out.mass = 1.0f;
    mat4f::store(out.inertia, mat4f::identity());
}

void get_surface_normal(SurfaceNormalResult &io, const Shape *, SubShapeId) {
    // no surface, a zero normal
    io.normal = vec3f::zero();
    io.position = vec3f::zero();
    io.scale = vec3f::make(1.0f, 1.0f, 1.0f);
}

void get_supporting_face(SupportingFaceResult &io, Vec3f, const Shape *, SubShapeId) {
    // no faces
    io.face.num_vertices = 0;
}

int32_t get_material(const Shape *, SubShapeId) { return -1; }

void cast_ray(CastRayCollector &, const CastRaySettings &, const Rayf &, const Shape *,
              const SubShapeIdBuilder &, const Transformf &) {}

void collide_point(CollidePointCollector &, const CollidePointSettings &, Vec3f, const Shape *,
                   const SubShapeIdBuilder &, const Transformf &) {}

ShapeDef def() {
    ShapeDef shape_def = {};
    shape_def.type = ShapeType::EMPTY;
    shape_def.destroy = destroy;
    shape_def.compute_mass_properties = compute_mass_properties;
    shape_def.get_surface_normal = get_surface_normal;
    shape_def.get_supporting_face = get_supporting_face;
    shape_def.get_material = get_material;
    shape_def.cast_ray = cast_ray;
    shape_def.collide_point = collide_point;
    return shapes::define_shape(shape_def);
}

}  // namespace empty

TUMBLE_NAMESPACE_END
