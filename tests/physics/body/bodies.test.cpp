#include "tumble/physics/body/bodies.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/util/object.h"
#include "test.h"
#include "test_math.h"

void test_bodies_slots(TestCtx &ctx) {
    bodies::Bodies pool = bodies::init();
    CHECK(ctx, pool.pool.length == 0 && pool.active_body_count == 0 && pool.step_stamp == 0);

    // fresh slots come in index order, each with its own first sequence, 1
    const body_id::BodyId a = bodies::add_slot(pool);
    const body_id::BodyId b = bodies::add_slot(pool);
    const body_id::BodyId c = bodies::add_slot(pool);
    CHECK(ctx, body_id::index(a) == 0 && body_id::sequence(a) == 1);
    CHECK(ctx, body_id::index(b) == 1 && body_id::sequence(b) == 1);
    CHECK(ctx, body_id::index(c) == 2 && body_id::sequence(c) == 1);
    CHECK(ctx, pool.pool.length == 3 && pool.active_body_indices.length == 3);
    CHECK(ctx, bodies::is_valid(pool, a) && bodies::is_valid(pool, b) && bodies::is_valid(pool, c));
    // a new slot holds no body until rigid_body puts one there
    CHECK(ctx, bodies::get(pool, b) == nullptr);

    // release frees what the pool holds, so the stand-in is a real allocation
    rigid_body::RigidBody *fake = util::create<rigid_body::RigidBody>();
    pool.pool[body_id::index(b)] = fake;
    CHECK(ctx, bodies::get(pool, b) == fake);

    // freeing b: its id stops resolving at once, before the slot is reused
    CHECK(ctx, bodies::remove_slot(pool, b));
    CHECK(ctx, !bodies::is_valid(pool, b));
    CHECK(ctx, bodies::get(pool, b) == nullptr);
    CHECK(ctx, !bodies::remove_slot(pool, b));

    // the next add reuses b's slot with a bumped sequence, keeping the body there for reuse
    const body_id::BodyId d = bodies::add_slot(pool);
    CHECK(ctx, body_id::index(d) == body_id::index(b));
    CHECK(ctx, body_id::sequence(d) == 2);
    CHECK(ctx, pool.pool.length == 3);
    CHECK(ctx, bodies::get(pool, d) == fake);
    // the old id names the same slot but is still rejected
    CHECK(ctx, !bodies::is_valid(pool, b) && bodies::get(pool, b) == nullptr);
    CHECK(ctx, bodies::is_valid(pool, a) && bodies::is_valid(pool, c));

    // freed slots come back last freed first
    CHECK(ctx, bodies::remove_slot(pool, a));
    CHECK(ctx, bodies::remove_slot(pool, c));
    CHECK(ctx, body_id::index(bodies::add_slot(pool)) == 2);
    CHECK(ctx, body_id::index(bodies::add_slot(pool)) == 0);
    CHECK(ctx, body_id::index(bodies::add_slot(pool)) == 3);

    // ids out of range and the invalid id resolve to nothing
    CHECK(ctx, !bodies::is_valid(pool, body_id::ser(100, 0)));
    CHECK(ctx, !bodies::is_valid(pool, body_id::INVALID_BODY_ID));

    bodies::release(pool);
    CHECK(ctx, pool.pool.length == 0 && pool.free_indices.length == 0);
}

void test_bodies_sequence_wraps(TestCtx &ctx) {
    bodies::Bodies pool = bodies::init();
    // each reuse of a slot bumps its sequence, so an id the slot's last body had stops naming the next; it wraps at 8 bits
    body_id::BodyId previous = bodies::add_slot(pool);
    const uint32_t index = body_id::index(previous);
    CHECK(ctx, body_id::sequence(previous) == 1);
    for (uint32_t reuse = 0; reuse < 300; reuse++) {
        CHECK(ctx, bodies::remove_slot(pool, previous));
        const body_id::BodyId next = bodies::add_slot(pool);
        CHECK(ctx, body_id::index(next) == index);
        CHECK(ctx, body_id::sequence(next) == ((body_id::sequence(previous) + 1) & body_id::MAX_SEQUENCE));
        CHECK(ctx, !bodies::is_valid(pool, previous) && bodies::is_valid(pool, next));
        previous = next;
    }
    // a slot's sequence is its own: a new slot starts at 1 whatever the first has been through
    CHECK(ctx, body_id::sequence(bodies::add_slot(pool)) == 1);
    // the invalid id names nothing
    CHECK(ctx, !bodies::is_valid(pool, body_id::INVALID_BODY_ID));
    CHECK(ctx, bodies::get(pool, body_id::INVALID_BODY_ID) == nullptr);
    bodies::release(pool);
}
