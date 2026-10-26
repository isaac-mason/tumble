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


/** the body's pose as the narrowphase takes it: the shape origin, unscaled */
static Transformf body_transform(const RigidBody &body) { return {body.position, body.quaternion, {1.0f, 1.0f, 1.0f}}; }

/** the shape's local bounds under the pose */
static AABox shape_world_bounds(const Shape *shape, const Transformf &transform) {
    const Mat4f matrix = mat4f::from_rotation_translation_scale(quatf::load(transform.quaternion), vec3f::load(transform.position),
                                                                vec3f::load(transform.scale));
    return aabox::transform_mat4(aabox::load(shape->aabb), matrix);
}

struct CastRayVisit {
    CastRayCollector *collector;
    const CastRaySettings *settings;
    Rayf ray;
};

static void visit_cast_ray(BodyVisitor *visitor, RigidBody *body) {
    if (visitor->should_exit) return;
    CastRayVisit &visit = *static_cast<CastRayVisit *>(visitor->context);
    CastRayCollector &collector = *visit.collector;

    collector.body_id_b = body->id;
    narrowphase::cast_ray_vs_shape(collector, *visit.settings, visit.ray, body->shape, sub_shape::builder(), body_transform(*body));

    visitor->should_exit = cast_ray::should_early_out(collector);
    // surface the collector's shrinking early-out fraction so the broadphase can prune nodes beyond the closest hit so far
    visitor->early_out_fraction = collector.early_out_fraction;
}

void cast_ray(World &world, CastRayCollector &collector, const CastRaySettings &settings, const Rayf &ray,
              const Filter &filter) {
    CastRayVisit visit = {&collector, &settings, ray};
    BodyVisitor visitor = body_visitor::create(visit_cast_ray, &visit);
    visitor.early_out_fraction = collector.early_out_fraction;
    broadphase::cast_ray(world.broadphase, world.bodies, ray, filter, visitor);
}

struct CastShapeVisit {
    CastShapeCollector *collector;
    const CastShapeSettings *settings;
    const Shape *shape;
    Transformf transform;
    Vec3f displacement;
};

static void visit_cast_shape(BodyVisitor *visitor, RigidBody *body) {
    if (visitor->should_exit) return;
    CastShapeVisit &visit = *static_cast<CastShapeVisit *>(visitor->context);
    CastShapeCollector &collector = *visit.collector;

    collector.body_id_b = body->id;
    narrowphase::cast_shape_vs_shape(collector, *visit.settings, visit.shape, sub_shape::builder(), visit.transform, visit.displacement,
                                     body->shape, sub_shape::builder(), body_transform(*body));

    visitor->should_exit = cast_shape::should_early_out(collector);
    // positive early-out fraction so cast_aabb can prune nodes past the closest impact so far; floored at FLT_MIN, not 0,
    // so bodies the cast starts inside are still visited after a hit at fraction 0
    visitor->early_out_fraction = scalar::max(scalar::FLOAT_MIN_NORMAL, collector.early_out_fraction);
}

void cast_shape(World &world, CastShapeCollector &collector, const CastShapeSettings &settings,
                const Shape *shape, const Transformf &transform, Vec3f displacement, const Filter &filter) {
    // the shape's bounds grown by the extra convex radius it is cast with
    Box3f bounds;
    aabox::store(bounds, aabox::expand_by_margin(shape_world_bounds(shape, transform), settings.extra_convex_radius));

    CastShapeVisit visit = {&collector, &settings, shape, transform, displacement};
    BodyVisitor visitor = body_visitor::create(visit_cast_shape, &visit);
    visitor.early_out_fraction = scalar::max(scalar::FLOAT_MIN_NORMAL, collector.early_out_fraction);
    broadphase::cast_aabb(world.broadphase, world.bodies, bounds, displacement, filter, visitor);
}

struct CollideShapeVisit {
    CollideShapeCollector *collector;
    const CollideShapeSettings *settings;
    const Shape *shape;
    Transformf transform;
    bool internal_edge_removal;
};

static void visit_collide_shape(BodyVisitor *visitor, RigidBody *body) {
    if (visitor->should_exit) return;
    CollideShapeVisit &visit = *static_cast<CollideShapeVisit *>(visitor->context);
    CollideShapeCollector &collector = *visit.collector;

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

static void collide_shape_walk(World &world, CollideShapeCollector &collector,
                               const CollideShapeSettings &settings, const Shape *shape, const Transformf &transform,
                               const Filter &filter, bool internal_edge_removal) {
    Box3f bounds;
    aabox::store(bounds, aabox::expand_by_margin(shape_world_bounds(shape, transform), settings.max_separation_distance));

    CollideShapeVisit visit = {&collector, &settings, shape, transform, internal_edge_removal};
    BodyVisitor visitor = body_visitor::create(visit_collide_shape, &visit);
    broadphase::intersect_aabb(world.broadphase, world.bodies, bounds, filter, visitor);
}

void collide_shape(World &world, CollideShapeCollector &collector, const CollideShapeSettings &settings,
                   const Shape *shape, const Transformf &transform, const Filter &filter) {
    collide_shape_walk(world, collector, settings, shape, transform, filter, false);
}

void collide_shape_with_internal_edge_removal(World &world, CollideShapeCollector &collector,
                                              const CollideShapeSettings &settings, const Shape *shape,
                                              const Transformf &transform, const Filter &filter) {
    collide_shape_walk(world, collector, settings, shape, transform, filter, true);
}

struct CollidePointVisit {
    CollidePointCollector *collector;
    const CollidePointSettings *settings;
    Vec3f point;
};

static void visit_collide_point(BodyVisitor *visitor, RigidBody *body) {
    if (visitor->should_exit) return;
    CollidePointVisit &visit = *static_cast<CollidePointVisit *>(visitor->context);
    CollidePointCollector &collector = *visit.collector;

    collector.body_id_b = body->id;
    narrowphase::collide_point_vs_shape(collector, *visit.settings, visit.point, body->shape, sub_shape::builder(), body_transform(*body));

    visitor->should_exit = collide_point::should_early_out(collector);
}

void collide_point(World &world, CollidePointCollector &collector, const CollidePointSettings &settings,
                   Vec3f point, const Filter &filter) {
    CollidePointVisit visit = {&collector, &settings, point};
    BodyVisitor visitor = body_visitor::create(visit_collide_point, &visit);
    broadphase::intersect_point(world.broadphase, world.bodies, point, filter, visitor);
}

}  // namespace query

TUMBLE_NAMESPACE_END
