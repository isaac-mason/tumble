#include "tumble/physics/shapes/shapes.h"

#include "tumble/math/mat4f.h"
#include "tumble/math/quatf.h"
#include "tumble/math/vec3f.h"
#include "tumble/util/assert.h"
#include "tumble/util/object.h"

TUMBLE_NAMESPACE_BEGIN

namespace shapes {

Shape *allocate(ShapeType type) {
    Shape *shape = util::create<Shape>();
    if (shape == nullptr) return nullptr;
    shape->type = type;
    return shape;
}

void release(Shape *shape) { util::destroy(shape); }

ShapeDef shape_defs[SHAPE_TYPE_COUNT];
CollideShapeVsShapeFn collide_table[SHAPE_TYPE_COUNT][SHAPE_TYPE_COUNT];
CastShapeVsShapeFn cast_table[SHAPE_TYPE_COUNT][SHAPE_TYPE_COUNT];

void default_compute_mass_properties(mass_properties::MassProperties &out, const Shape *) { out.mass = 1.0f; }

float default_get_inner_radius(const Shape *) { return 0.0f; }

void default_get_leaf_shape(GetLeafShapeResult &out, const Shape *shape, sub_shape::SubShapeId sub_shape_id) {
    out.shape = shape;
    out.remainder = sub_shape_id;
}

void default_get_sub_shape_transformed_shape(GetSubShapeTransformedShapeResult &out, const Shape *shape, sub_shape::SubShapeId sub_shape_id) {
    out.shape = shape;
    out.remainder = sub_shape_id;
}

ShapeDef define_shape(const ShapeDef &def) {
    ShapeDef out = def;
    if (out.compute_mass_properties == nullptr) out.compute_mass_properties = default_compute_mass_properties;
    if (out.get_inner_radius == nullptr) out.get_inner_radius = default_get_inner_radius;
    if (out.get_leaf_shape == nullptr) out.get_leaf_shape = default_get_leaf_shape;
    if (out.get_sub_shape_transformed_shape == nullptr) out.get_sub_shape_transformed_shape = default_get_sub_shape_transformed_shape;
    TUMBLE_ASSERT(out.get_surface_normal != nullptr && out.get_supporting_face != nullptr && out.get_material != nullptr &&
              out.cast_ray != nullptr && out.collide_point != nullptr);
    return out;
}

void set_shape_def(const ShapeDef &def) { shape_defs[shape_type_index(def.type)] = def; }

void set_collide_shape_fn(ShapeType type_a, ShapeType type_b, CollideShapeVsShapeFn fn) {
    collide_table[shape_type_index(type_a)][shape_type_index(type_b)] = fn;
}

void set_cast_shape_fn(ShapeType type_a, ShapeType type_b, CastShapeVsShapeFn fn) {
    cast_table[shape_type_index(type_a)][shape_type_index(type_b)] = fn;
}

// jolt's ScaleHelpers tolerances
static constexpr float SCALE_TOLERANCE_SQ = 1.0e-8f;
static constexpr float ROTATION_IDENTITY_TOLERANCE_SQ = 1.0e-12f;

static bool is_uniform_scale(Vec3f scale) {
    const float dx = vec3f::y(scale) - vec3f::x(scale), dy = vec3f::z(scale) - vec3f::y(scale), dz = vec3f::x(scale) - vec3f::z(scale);
    return dx * dx + dy * dy + dz * dz <= SCALE_TOLERANCE_SQ;
}

/** close to the identity quaternion or its negation, which is the same rotation */
static bool is_rotation_identity(Quatf rotation) {
    const float xyz_sq = quatf::x(rotation) * quatf::x(rotation) + quatf::y(rotation) * quatf::y(rotation) + quatf::z(rotation) * quatf::z(rotation);
    const float dw_positive = quatf::w(rotation) - 1.0f, dw_negative = quatf::w(rotation) + 1.0f;
    return xyz_sq + dw_positive * dw_positive <= ROTATION_IDENTITY_TOLERANCE_SQ ||
           xyz_sq + dw_negative * dw_negative <= ROTATION_IDENTITY_TOLERANCE_SQ;
}

Vec3f transform_scale(Quatf rotation, Vec3f scale) {
    // we don't need to transform uniform scale or if the rotation is identity
    if (is_rotation_identity(rotation) || is_uniform_scale(scale)) return scale;

    // the diagonal of R^T * diag(scale) * R: column j of R squared, weighted by the scale per row
    const Mat4f r = mat4f::from_quat(rotation);
    const Vec3f c0 = mat4f::axis(r, 0), c1 = mat4f::axis(r, 1), c2 = mat4f::axis(r, 2);
    return vec3f::make(vec3f::dot(vec3f::multiply(c0, c0), scale), vec3f::dot(vec3f::multiply(c1, c1), scale),
                       vec3f::dot(vec3f::multiply(c2, c2), scale));
}

static SurfaceNormalResult surface_normal_result;
static SupportingFaceResult supporting_face_result;

Vec3f get_shape_surface_normal(const Shape *shape, Vec3f position, sub_shape::SubShapeId sub_shape_id) {
    SurfaceNormalResult &io = surface_normal_result;
    io.position = position;
    io.scale = vec3f::splat(1.0f);
    def_of(shape).get_surface_normal(io, shape, sub_shape_id);
    return io.normal;
}

mass_properties::MassProperties &compute_mass_properties(mass_properties::MassProperties &out, const Shape *shape) {
    def_of(shape).compute_mass_properties(out, shape);
    return out;
}

void get_shape_supporting_face(face::Face &out, const Shape *shape, sub_shape::SubShapeId sub_shape_id, Vec3f local_direction,
                               const Mat4f &transform, Vec3f scale) {
    SupportingFaceResult &io = supporting_face_result;
    io.transform = transform;
    io.scale = scale;
    def_of(shape).get_supporting_face(io, local_direction, shape, sub_shape_id);
    face::copy(out, io.face);
}

float get_shape_inner_radius(const Shape *shape) { return def_of(shape).get_inner_radius(shape); }

int32_t get_shape_material(const Shape *shape, sub_shape::SubShapeId sub_shape_id) {
    return def_of(shape).get_material(shape, sub_shape_id);
}

void get_leaf_shape(GetLeafShapeResult &out, const Shape *shape, sub_shape::SubShapeId sub_shape_id) {
    def_of(shape).get_leaf_shape(out, shape, sub_shape_id);
}

void get_sub_shape_transformed_shape(GetSubShapeTransformedShapeResult &out, const Shape *shape, sub_shape::SubShapeId sub_shape_id) {
    def_of(shape).get_sub_shape_transformed_shape(out, shape, sub_shape_id);
}

}  // namespace shapes

TUMBLE_NAMESPACE_END
