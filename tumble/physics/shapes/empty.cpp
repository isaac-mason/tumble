#include "tumble/physics/shapes/empty.h"

#include "tumble/math/aabox.h"
#include "tumble/math/mat4f.h"
#include "tumble/math/vec3f.h"

TUMBLE_NAMESPACE_BEGIN

namespace empty {

Shape *create() {
    Shape *shape = shapes::allocate(shapes::SHAPE_EMPTY);
    if (shape == nullptr) return nullptr;
    // zero-size bounds at the origin
    shape->aabb = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
    vec3f::store(shape->center_of_mass, vec3f::zero());
    shape->volume = 0.0f;
    return shape;
}

void destroy(Shape *shape) { shapes::release(shape); }

void compute_mass_properties(mass_properties::MassProperties &out, const Shape *) {
    // unit mass and inertia, so a dynamic body without a shape still integrates
    out.mass = 1.0f;
    mat4f::store(out.inertia, mat4f::identity());
}

void get_surface_normal(shapes::SurfaceNormalResult &io, const Shape *, sub_shape::SubShapeId) {
    // no surface, a zero normal
    io.normal = vec3f::zero();
    io.position = vec3f::zero();
    io.scale = vec3f::make(1.0f, 1.0f, 1.0f);
}

void get_supporting_face(shapes::SupportingFaceResult &io, Vec3f, const Shape *, sub_shape::SubShapeId) {
    // no faces
    io.face.num_vertices = 0;
}

int32_t get_material(const Shape *, sub_shape::SubShapeId) { return -1; }

void cast_ray(cast_ray::CastRayCollector &, const cast_ray::CastRaySettings &, const cast_ray::Rayf &, const Shape *,
              const sub_shape::SubShapeIdBuilder &, const Transformf &) {}

void collide_point(collide_point::CollidePointCollector &, const collide_point::CollidePointSettings &, Vec3f, const Shape *,
                   const sub_shape::SubShapeIdBuilder &, const Transformf &) {}

shapes::ShapeDef def() {
    shapes::ShapeDef shape_def = {};
    shape_def.type = shapes::SHAPE_EMPTY;
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
