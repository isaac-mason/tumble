#include <stdio.h>

#include "tumble/physics/shapes/dispatch.h"
#include "tumble/physics/world.h"
#include "tumble/util/alloc.h"
#include "tumble/util/error.h"
#include "tumble/util/mem.h"

using namespace tumble;

/* SNIPPET_START: arena */
/** a bump allocator over a block of its own: frees count what came back, but nothing is reused */
struct Arena {
    alignas(64) unsigned char memory[1 << 20];
    size_t used;
    size_t freed;
};

void *arena_allocate(size_t size, size_t alignment, void *user) {
    Arena &arena = *static_cast<Arena *>(user);
    const size_t at = util::align_up(arena.used, alignment);
    if (at + size > sizeof(arena.memory)) return nullptr;
    arena.used = at + size;
    return arena.memory + at;
}

void arena_free(void *, size_t size, void *user) { static_cast<Arena *>(user)->freed += size; }

static Arena arena;
/* SNIPPET_END: arena */

int main() {
    /* SNIPPET_START: set-allocator */
    // before anything allocates: every block tumble takes from here on comes from the arena
    util::set_allocator({arena_allocate, arena_free, &arena});
    /* SNIPPET_END: set-allocator */

    dispatch::init();

    /* SNIPPET_START: allocated */
    world_settings::WorldSettings settings = world_settings::create();
    world_settings::add_object_layer(settings, world_settings::add_broadphase_layer(settings));
    world::World *world = world::create(settings);
    world_settings::release(settings);
    printf("a world holds %zu bytes in %zu blocks\n", util::allocated_bytes(), util::allocated_blocks());

    world::destroy(world);
    printf("after destroy: %zu bytes\n", util::allocated_bytes());
    /* SNIPPET_END: allocated */
    /* HIDE_START */
    // every byte handed out came back, through the arena's free
    if (util::allocated_bytes() != 0 || util::allocated_blocks() != 0 || arena.freed == 0) return 1;
    /* HIDE_END */

    /* SNIPPET_START: errors */
    // the arena is nearly spent: the next world cannot be made
    arena.used = sizeof(arena.memory) - 64;
    world_settings::WorldSettings more = world_settings::create();
    world::World *refused = world::create(more);
    if (refused == nullptr && util::error() == util::ERROR_OUT_OF_MEMORY) {
        printf("out of memory: the call backed out and nothing leaked\n");
        util::error_clear();
    }
    /* SNIPPET_END: errors */
    /* HIDE_START */
    if (refused != nullptr || util::error() != util::ERROR_NONE || util::allocated_bytes() != 0) return 1;
    /* HIDE_END */
    return 0;
}
