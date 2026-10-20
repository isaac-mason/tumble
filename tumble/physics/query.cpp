#include "tumble/physics/query.h"

#include "tumble/math/aabox.h"
#include "tumble/math/mat4f.h"
#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/broadphase/broadphase.h"
#include "tumble/physics/collision/narrowphase.h"

TUMBLE_NAMESPACE_BEGIN

namespace query {

using rigid_body::RigidBody;

/** the body's pose as the narrowphase takes it: the shape origin, unscaled */
static Transformf body_transform(const RigidBody &body) { return {body.position, body.quaternion, {1.0f, 1.0f, 1.0f}}; }

/** the shape's local bounds under the pose */
static AABox shape_world_bounds(const shapes::Shape *shape, const Transformf &transform) {
    const Mat4f matrix = mat4f::from_rotation_translation_scale(quatf::load(transform.quaternion), vec3f::load(transform.position),
                                                                vec3f::load(transform.scale));
    return aabox::transform_mat4(aabox::load(shape->aabb), matrix);
}

struct CastRayVisit {
    cast_ray::CastRayCollector *collector;
    const cast_ray::CastRaySettings *settings;
    cast_ray::Rayf ray;
};

static void visit_cast_ray(body_visitor::BodyVisitor *visitor, RigidBody *body) {
    if (visitor->should_exit) return;
    CastRayVisit &visit = *static_cast<CastRayVisit *>(visitor->context);
    cast_ray::CastRayCollector &collector = *visit.collector;

    collector.body_id_b = body->id;
    narrowphase::cast_ray_vs_shape(collector, *visit.settings, visit.ray, body->shape, sub_shape::builder(), body_transform(*body));

    visitor->should_exit = cast_ray::should_early_out(collector);
    // surface the collector's shrinking early-out fraction so the broadphase can prune nodes beyond the closest hit so far
    visitor->early_out_fraction = collector.early_out_fraction;
}

void cast_ray(world::World &world, cast_ray::CastRayCollector &collector, const cast_ray::CastRaySettings &settings, const cast_ray::Rayf &ray,
              const filter::Filter &filter) {
    CastRayVisit visit = {&collector, &settings, ray};
    body_visitor::BodyVisitor visitor = body_visitor::create(visit_cast_ray, &visit);
    visitor.early_out_fraction = collector.early_out_fraction;
    broadphase::cast_ray(world.broadphase, world.bodies, ray, filter, visitor);
}

struct CastShapeVisit {
    cast_shape::CastShapeCollector *collector;
    const cast_shape::CastShapeSettings *settings;
    const shapes::Shape *shape;
    Transformf transform;
    Vec3f displacement;
};

static void visit_cast_shape(body_visitor::BodyVisitor *visitor, RigidBody *body) {
    if (visitor->should_exit) return;
    CastShapeVisit &visit = *static_cast<CastShapeVisit *>(visitor->context);
    cast_shape::CastShapeCollector &collector = *visit.collector;

    collector.body_id_b = body->id;
    narrowphase::cast_shape_vs_shape(collector, *visit.settings, visit.shape, sub_shape::builder(), visit.transform, visit.displacement,
                                     body->shape, sub_shape::builder(), body_transform(*body));

    visitor->should_exit = cast_shape::should_early_out(collector);
    // positive early-out fraction so cast_aabb can prune nodes past the closest impact so far; floored at FLT_MIN, not 0,
    // so bodies the cast starts inside are still visited after a hit at fraction 0
    visitor->early_out_fraction = scalar::max(scalar::FLOAT_MIN_NORMAL, collector.early_out_fraction);
}

void cast_shape(world::World &world, cast_shape::CastShapeCollector &collector, const cast_shape::CastShapeSettings &settings,
                const shapes::Shape *shape, const Transformf &transform, Vec3f displacement, const filter::Filter &filter) {
    Box3f bounds;
    aabox::store(bounds, shape_world_bounds(shape, transform));

    CastShapeVisit visit = {&collector, &settings, shape, transform, displacement};
    body_visitor::BodyVisitor visitor = body_visitor::create(visit_cast_shape, &visit);
    visitor.early_out_fraction = scalar::max(scalar::FLOAT_MIN_NORMAL, collector.early_out_fraction);
    broadphase::cast_aabb(world.broadphase, world.bodies, bounds, displacement, filter, visitor);
}

struct CollideShapeVisit {
    collide_shape::CollideShapeCollector *collector;
    const collide_shape::CollideShapeSettings *settings;
    const shapes::Shape *shape;
    Transformf transform;
    bool internal_edge_removal;
};

static void visit_collide_shape(body_visitor::BodyVisitor *visitor, RigidBody *body) {
    if (visitor->should_exit) return;
    CollideShapeVisit &visit = *static_cast<CollideShapeVisit *>(visitor->context);
    collide_shape::CollideShapeCollector &collector = *visit.collector;

    collector.body_id_b = body->id;
    collide_shape::on_body(collector, body->id);
    if (visit.internal_edge_removal) {
        // the wrapper flushes its held back hits into the collector before returning, so per body
        narrowphase::collide_shape_vs_shape_with_internal_edge_removal(collector, *visit.settings, visit.shape, sub_shape::builder(),
                                                                       visit.transform, body->shape, sub_shape::builder(),
                                                                       body_transform(*body));
    } else {
        narrowphase::collide_shape_vs_shape(collector, *visit.settings, visit.shape, sub_shape::builder(), visit.transform, body->shape,
                                            sub_shape::builder(), body_transform(*body));
    }
    collide_shape::on_body_end(collector);

    visitor->should_exit = collide_shape::should_early_out(collector);
}

static void collide_shape_walk(world::World &world, collide_shape::CollideShapeCollector &collector,
                               const collide_shape::CollideShapeSettings &settings, const shapes::Shape *shape, const Transformf &transform,
                               const filter::Filter &filter, bool internal_edge_removal) {
    Box3f bounds;
    aabox::store(bounds, aabox::expand_by_margin(shape_world_bounds(shape, transform), settings.max_separation_distance));

    CollideShapeVisit visit = {&collector, &settings, shape, transform, internal_edge_removal};
    body_visitor::BodyVisitor visitor = body_visitor::create(visit_collide_shape, &visit);
    broadphase::intersect_aabb(world.broadphase, world.bodies, bounds, filter, visitor);
}

void collide_shape(world::World &world, collide_shape::CollideShapeCollector &collector, const collide_shape::CollideShapeSettings &settings,
                   const shapes::Shape *shape, const Transformf &transform, const filter::Filter &filter) {
    collide_shape_walk(world, collector, settings, shape, transform, filter, false);
}

void collide_shape_with_internal_edge_removal(world::World &world, collide_shape::CollideShapeCollector &collector,
                                              const collide_shape::CollideShapeSettings &settings, const shapes::Shape *shape,
                                              const Transformf &transform, const filter::Filter &filter) {
    collide_shape_walk(world, collector, settings, shape, transform, filter, true);
}

struct CollidePointVisit {
    collide_point::CollidePointCollector *collector;
    const collide_point::CollidePointSettings *settings;
    Vec3f point;
};

static void visit_collide_point(body_visitor::BodyVisitor *visitor, RigidBody *body) {
    if (visitor->should_exit) return;
    CollidePointVisit &visit = *static_cast<CollidePointVisit *>(visitor->context);
    collide_point::CollidePointCollector &collector = *visit.collector;

    collector.body_id_b = body->id;
    narrowphase::collide_point_vs_shape(collector, *visit.settings, visit.point, body->shape, sub_shape::builder(), body_transform(*body));

    visitor->should_exit = collide_point::should_early_out(collector);
}

void collide_point(world::World &world, collide_point::CollidePointCollector &collector, const collide_point::CollidePointSettings &settings,
                   Vec3f point, const filter::Filter &filter) {
    CollidePointVisit visit = {&collector, &settings, point};
    body_visitor::BodyVisitor visitor = body_visitor::create(visit_collide_point, &visit);
    broadphase::intersect_point(world.broadphase, world.bodies, point, filter, visitor);
}

}  // namespace query

TUMBLE_NAMESPACE_END
