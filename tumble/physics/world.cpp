#include "tumble/physics/world.h"

#include "tumble/physics/update.h"
#include "tumble/util/assert.h"
#include "tumble/util/object.h"

TUMBLE_NAMESPACE_BEGIN

namespace world {

// the bodies a world can hold all have an index the id has bits for, and no id is the invalid one
static_assert(body_id::MAX_BODIES - 1 == body_id::MAX_BODY_INDEX, "every body index fits the id's index bits");
static_assert(((body_id::MAX_SEQUENCE << body_id::SEQUENCE_SHIFT) & body_id::BROADPHASE_BIT) == 0, "no sequence reaches the broadphase bit");

World *create(const world_settings::WorldSettings &settings) {
    World *world = util::create<World>();
    if (world == nullptr) return nullptr;
    world->settings = settings;
    world->settings.layers = {};
    if (!layers::copy(world->settings.layers, settings.layers)) {
        util::destroy(world);
        return nullptr;
    }
    world->broadphase = broadphase::init(world->settings.layers);
    world->pairs = pairs::create();
    world->bodies = bodies::init();
    // the pool grows only as far as body_id::MAX_BODIES (bodies::add_slot refuses past it)
    TUMBLE_ASSERT(world->bodies.pool.length <= body_id::MAX_BODIES);
    world->contact_constraints = contact_constraints::create();
    world->constraints = constraints::create();
    world->contacts = contacts::create();
    world->contact_pairs = contact_pairs::create();
    world->islands = islands::create();
    world->ccd = ccd::create();
    world->events = events::create_events();
    world->answers = events::create_answers();
    world->delta_time = 0.0f;
    world->previous_time_step = 0.0f;
    return world;
}

void destroy(World *world) {
    events::release(world->answers);
    events::release(world->events);
    ccd::release(world->ccd);
    islands::release(world->islands);
    contact_pairs::release(world->contact_pairs);
    contacts::release(world->contacts);
    constraints::release(world->constraints);
    contact_constraints::release(world->contact_constraints);
    bodies::release(world->bodies);
    pairs::release(world->pairs);
    broadphase::release(world->broadphase);
    world_settings::release(world->settings);
    util::destroy(world);
}

void step(World &world, float delta_time) {
    update::step_begin(world, delta_time);
    do {
        update::find_pairs(world);
        update::narrowphase(world);
    } while (world.events.woken > 0);
    update::solve(world);
    update::step_end(world);
}

}  // namespace world

TUMBLE_NAMESPACE_END
