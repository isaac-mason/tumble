#include <stdio.h>

#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/debug.h"
#include "tumble/physics/query.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/convex.h"
#include "tumble/physics/shapes/dispatch.h"
#include "tumble/physics/world.h"

using namespace tumble;

/* SNIPPET_START: data */
// an ellipsoid: a convex shape tumble does not have, in user slot 1. its payload is the shape's `user.data`
constexpr shapes::ShapeType SHAPE_ELLIPSOID = shapes::SHAPE_USER_1;

struct Ellipsoid {
    Float3 radii;
    float density;
};

const Ellipsoid &ellipsoid_of(const shapes::Shape *shape) { return *static_cast<const Ellipsoid *>(shape->user.data); }
/* SNIPPET_END: data */

/* SNIPPET_START: support */
// gjk and epa see a convex shape only through its support: the farthest point in a direction. for radii r and a local
// direction d it is r^2 d / |r d|, per component
Vec3f ellipsoid_support(support::Support &support, Vec3f direction) {
    const Vec3f local = support::to_local_direction(support, direction);
    const Ellipsoid &ellipsoid = *static_cast<const Ellipsoid *>(support.user.data);
    const Vec3f radii = vec3f::multiply(vec3f::load(ellipsoid.radii), vec3f::abs(support.user.scale));
    const Vec3f stretched = vec3f::multiply(radii, local);
    const float length = vec3f::length(stretched);
    const Vec3f point = length > 0.0f ? vec3f::scale(vec3f::multiply(radii, stretched), 1.0f / length) : vec3f::zero();
    return support::to_caller_space(support, support::add_radius_along(point, local, support.add_radius));
}

// no rounding, so both modes give the whole shape
void ellipsoid_set_support(support::Support &out, const shapes::Shape *shape, support::SupportFunctionMode, Vec3f scale) {
    out.get_support = ellipsoid_support;
    out.has_transform = false;
    out.add_radius = 0.0f;
    out.convex_radius = 0.0f;
    out.user.data = shape->user.data;
    out.user.scale = scale;
}
/* SNIPPET_END: support */

/* SNIPPET_START: functions */
void ellipsoid_mass_properties(mass_properties::MassProperties &out, const shapes::Shape *shape) {
    const Ellipsoid &ellipsoid = ellipsoid_of(shape);
    const float a = ellipsoid.radii.x, b = ellipsoid.radii.y, c = ellipsoid.radii.z;
    out.mass = 4.0f / 3.0f * scalar::PI * a * b * c * ellipsoid.density;
    const float k = out.mass / 5.0f;
    out.inertia = {{k * (b * b + c * c), 0, 0, 0, 0, k * (a * a + c * c), 0, 0, 0, 0, k * (a * a + b * b), 0, 0, 0, 0, 1.0f}};
}

// the gradient of x^2/a^2 + y^2/b^2 + z^2/c^2 at the point
void ellipsoid_surface_normal(shapes::SurfaceNormalResult &io, const shapes::Shape *shape, sub_shape::SubShapeId) {
    const Vec3f radii = vec3f::load(ellipsoid_of(shape).radii);
    const Vec3f gradient = vec3f::divide(io.position, vec3f::multiply(radii, radii));
    io.normal = vec3f::squared_length(gradient) > 0.0f ? vec3f::normalize(gradient) : vec3f::axis_y();
}

// curved: no face to build a manifold from, so contacts are single points
void ellipsoid_supporting_face(shapes::SupportingFaceResult &io, Vec3f, const shapes::Shape *, sub_shape::SubShapeId) {
    io.face.num_vertices = 0;
}

int32_t ellipsoid_material(const shapes::Shape *shape, sub_shape::SubShapeId) { return shape->material_id; }

float ellipsoid_inner_radius(const shapes::Shape *shape) {
    const Float3 &r = ellipsoid_of(shape).radii;
    return scalar::min(r.x, scalar::min(r.y, r.z));
}

void ellipsoid_collide_point(collide_point::CollidePointCollector &collector, const collide_point::CollidePointSettings &, Vec3f point,
                             const shapes::Shape *shape, const sub_shape::SubShapeIdBuilder &sub_shape_id, const Transformf &transform) {
    // into the shape's space, then inside when the scaled coordinates are within the unit sphere
    const Vec3f local = vec3f::transform_quat(vec3f::subtract(point, vec3f::load(transform.position)), quatf::conjugate(quatf::load(transform.quaternion)));
    const Vec3f radii = vec3f::multiply(vec3f::load(ellipsoid_of(shape).radii), vec3f::abs(vec3f::load(transform.scale)));
    if (vec3f::squared_length(vec3f::divide(local, radii)) > 1.0f) return;
    collide_point::CollidePointHit hit = collide_point::create_hit();
    hit.sub_shape_id_b = sub_shape_id.value;
    hit.body_id_b = collector.body_id_b;
    collide_point::add_hit(collector, hit);
}

// three rings, one in each axis plane
void ellipsoid_draw(debug::DebugBuffers &out, const shapes::Shape *shape, const Transformf &transform, Vec3f color) {
    const Vec3f radii = vec3f::multiply(vec3f::load(ellipsoid_of(shape).radii), vec3f::load(transform.scale));
    const Quatf rotation = quatf::load(transform.quaternion);
    const Vec3f centre = vec3f::load(transform.position);
    for (int plane = 0; plane < 3; plane++) {
        Vec3f previous = vec3f::zero();
        for (int i = 0; i <= 24; i++) {
            const float angle = 2.0f * scalar::PI * (float)i / 24.0f;
            const float u = scalar::cos(angle), v = scalar::sin(angle);
            const Vec3f ring = plane == 0 ? vec3f::make(u, v, 0.0f) : plane == 1 ? vec3f::make(u, 0.0f, v) : vec3f::make(0.0f, u, v);
            const Vec3f point = vec3f::add(centre, vec3f::transform_quat(vec3f::multiply(ring, radii), rotation));
            if (i > 0) debug::line(out, previous, point, color);
            previous = point;
        }
    }
}
/* SNIPPET_END: functions */

/* SNIPPET_START: def */
shapes::ShapeDef ellipsoid_def() {
    shapes::ShapeDef def = {};
    def.type = SHAPE_ELLIPSOID;
    def.set_support = ellipsoid_set_support;
    def.compute_mass_properties = ellipsoid_mass_properties;
    def.get_surface_normal = ellipsoid_surface_normal;
    def.get_supporting_face = ellipsoid_supporting_face;
    def.get_material = ellipsoid_material;
    def.get_inner_radius = ellipsoid_inner_radius;
    // a convex type's rays go through its support
    def.cast_ray = convex::cast_ray_vs_convex;
    def.collide_point = ellipsoid_collide_point;
    def.draw = ellipsoid_draw;
    // define_shape fills what a def leaves out with the defaults
    return shapes::define_shape(def);
}

/** a shape of the type: its payload, and the bounds, centre of mass and volume every shape carries */
shapes::Shape *create_ellipsoid(Ellipsoid *ellipsoid) {
    shapes::Shape *shape = shapes::allocate(SHAPE_ELLIPSOID);
    if (shape == nullptr) return nullptr;
    shape->user.data = ellipsoid;
    const Float3 &r = ellipsoid->radii;
    shape->aabb = {{-r.x, -r.y, -r.z}, {r.x, r.y, r.z}};
    shape->center_of_mass = {0.0f, 0.0f, 0.0f};
    shape->volume = 4.0f / 3.0f * scalar::PI * r.x * r.y * r.z;
    shape->material_id = -1;
    return shape;
}
/* SNIPPET_END: def */

int main() {
    /* SNIPPET_START: register */
    // the type beside the built-ins: it collides with every convex type, mesh and plane, in compounds and under
    // wrappers. a pair of its own (an ellipsoid against a mesh by a faster rule, say) goes in after, over the generic one
    const shapes::ShapeDef def = ellipsoid_def();
    dispatch::init(&def, 1);
    /* SNIPPET_END: register */

    world_settings::WorldSettings settings = world_settings::create();
    const uint32_t moving = world_settings::add_object_layer(settings, world_settings::add_broadphase_layer(settings));
    const uint32_t not_moving = world_settings::add_object_layer(settings, world_settings::add_broadphase_layer(settings));
    world_settings::enable_collision(settings, moving, moving);
    world_settings::enable_collision(settings, moving, not_moving);
    world::World *world = world::create(settings);
    world_settings::release(settings);
    shapes::Shape *ground_shape = box::create({.half_extents = {20.0f, 0.5f, 20.0f}});
    rigid_body::create(*world, {.shape = ground_shape, .object_layer = not_moving, .motion_type = motion_type::MOTION_STATIC,
                                .position = {0.0f, -0.5f, 0.0f}});

    /* SNIPPET_START: use */
    // an egg, 0.3 m wide and 0.5 m tall, dropped on its side: it rolls to lie on its long axis
    static Ellipsoid egg = {.radii = {0.3f, 0.5f, 0.3f}, .density = 1000.0f};
    shapes::Shape *egg_shape = create_ellipsoid(&egg);
    rigid_body::RigidBody *body = rigid_body::create(*world, {.shape = egg_shape, .object_layer = moving,
                                                              .motion_type = motion_type::MOTION_DYNAMIC, .position = {0.0f, 2.0f, 0.0f},
                                                              .quaternion = {0.0f, 0.0f, 0.38268343f, 0.92387953f}});
    for (int i = 0; i < 300; i++) world::step(*world, 1.0f / 60.0f);
    printf("the egg rests at y %.2f\n", body->position.y);
    /* SNIPPET_END: use */
    /* HIDE_START */
    // lying on its side, its centre a short radius up (less the penetration slop); its tip no longer up
    if (body->position.y < 0.26f || body->position.y > 0.32f) return 1;
    // a point inside it is found by a query, and it draws its rings
    filter::Filter filter = filter::create(world->settings.layers);
    collide_point::CollidePointCollector inside = collide_point::create_collector(collector::COLLECT_ALL);
    query::collide_point(*world, inside, collide_point::default_settings(), vec3f::load(body->position), filter);
    const bool found = inside.hits.length == 1 && inside.hits[0].body_id_b == body->id;
    collide_point::release(inside);
    filter::release(filter);
    debug::DebugBuffers *buffers = debug::create();
    debug::body(*buffers, *body, {});
    const bool drawn = buffers->line_count == 72;
    debug::release(buffers);
    if (!found || !drawn) return 1;
    /* HIDE_END */

    world::destroy(world);
    shapes::release(egg_shape);
    box::destroy(ground_shape);
    return 0;
}
