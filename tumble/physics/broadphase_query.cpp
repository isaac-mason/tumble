#include "tumble/physics/broadphase_query.h"

#include "tumble/math/aabox.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/broadphase/broadphase.h"

TUMBLE_NAMESPACE_BEGIN

namespace broadphase_query {

static void visit_overlap(body_visitor::BodyVisitor *visitor, rigid_body::RigidBody *body) {
    BodyCollector &collector = *static_cast<BodyCollector *>(visitor->context);
    add_hit(collector, body->id);
    visitor->should_exit = should_early_out(collector);
}

static void visit_cast(body_visitor::BodyVisitor *visitor, rigid_body::RigidBody *body) {
    BodyCastCollector &collector = *static_cast<BodyCastCollector *>(visitor->context);
    // the walk visits a body it could not cull, which may lie at or past the collector's bound
    if (visitor->fraction >= collector.early_out_fraction) return;
    add_hit(collector, {body->id, visitor->fraction});
    visitor->should_exit = should_early_out(collector);
    visitor->early_out_fraction = collector.early_out_fraction;
}

static body_visitor::BodyVisitor overlap_visitor(BodyCollector &collector) { return body_visitor::create(visit_overlap, &collector); }

static body_visitor::BodyVisitor cast_visitor(BodyCastCollector &collector) {
    body_visitor::BodyVisitor visitor = body_visitor::create(visit_cast, &collector);
    visitor.early_out_fraction = collector.early_out_fraction;
    return visitor;
}

void cast_ray(world::World &world, BodyCastCollector &collector, const cast_ray::Rayf &ray, const filter::Filter &filter) {
    body_visitor::BodyVisitor visitor = cast_visitor(collector);
    broadphase::cast_ray(world.broadphase, world.bodies, ray, filter, visitor);
}

void collide_aabb(world::World &world, BodyCollector &collector, const AABox &box, const filter::Filter &filter) {
    Box3f bounds;
    aabox::store(bounds, box);
    body_visitor::BodyVisitor visitor = overlap_visitor(collector);
    broadphase::intersect_aabb(world.broadphase, world.bodies, bounds, filter, visitor);
}

void collide_sphere(world::World &world, BodyCollector &collector, Vec3f center, float radius, const filter::Filter &filter) {
    body_visitor::BodyVisitor visitor = overlap_visitor(collector);
    broadphase::intersect_sphere(world.broadphase, world.bodies, center, radius, filter, visitor);
}

void collide_point(world::World &world, BodyCollector &collector, Vec3f point, const filter::Filter &filter) {
    body_visitor::BodyVisitor visitor = overlap_visitor(collector);
    broadphase::intersect_point(world.broadphase, world.bodies, point, filter, visitor);
}

void collide_oriented_box(world::World &world, BodyCollector &collector, const OrientedBox &box, const filter::Filter &filter) {
    body_visitor::BodyVisitor visitor = overlap_visitor(collector);
    broadphase::intersect_oriented_box(world.broadphase, world.bodies, box, filter, visitor);
}

void cast_aabb(world::World &world, BodyCastCollector &collector, const AABox &box, Vec3f displacement, const filter::Filter &filter) {
    Box3f bounds;
    aabox::store(bounds, box);
    body_visitor::BodyVisitor visitor = cast_visitor(collector);
    broadphase::cast_aabb(world.broadphase, world.bodies, bounds, displacement, filter, visitor);
}

}  // namespace broadphase_query

TUMBLE_NAMESPACE_END
