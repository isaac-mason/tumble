#include "tumble/physics/world.h"

#include "tumble/physics/update.h"
#include "tumble/util/assert.h"
#include "tumble/util/object.h"

TUMBLE_NAMESPACE_BEGIN

namespace world {

// the bodies a world can hold all have an index the id has bits for, and no id is the invalid one
static_assert(body_id::MAX_BODIES - 1 == body_id::MAX_BODY_INDEX, "every body index fits the id's index bits");
static_assert(((body_id::MAX_SEQUENCE << body_id::SEQUENCE_SHIFT) & body_id::BROADPHASE_BIT) == 0, "no sequence reaches the broadphase bit");

World *create(const WorldSettings &settings) {
    World *world = util::create<World>();
    world->settings = settings;
    broadphase::init(world->broadphase, world->settings.layers);
    world->pairs = pairs::create();
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

void destroy(World *world) { util::destroy(world); }

void set_materials(World &world, const WorldMaterial *materials, uint32_t count) {
    collections::Array<WorldMaterial> &table = world.contact_constraints.materials;
    table.length = 0;
    table.reserve(count);
    for (uint32_t i = 0; i < count; i++) table.data[i] = materials[i];
    table.length = count;
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
