#include "instance.h"
#include "physics/filter.h"
#include "physics/layers.h"
#include "test.h"
#include "test_math.h"

void test_layers(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    layers::Layers l = layers::create();
    // two broadphase layers: moving and static; three object layers: player and debris move, terrain is static
    const uint32_t bp_moving = layers::add_broadphase_layer(l);
    const uint32_t bp_static = layers::add_broadphase_layer(l);
    const uint32_t player = layers::add_object_layer(l, bp_moving);
    const uint32_t debris = layers::add_object_layer(l, bp_moving);
    const uint32_t terrain = layers::add_object_layer(l, bp_static);
    CHECK(ctx, l.broadphase_layers == 2 && l.object_layers == 3);
    // an object layer queries its own broadphase layer and nothing else until told
    CHECK(ctx, layers::object_layer_collides_with_broadphase_layer(l, player, bp_moving));
    CHECK(ctx, !layers::object_layer_collides_with_broadphase_layer(l, player, bp_static));
    CHECK(ctx, !layers::object_layer_collides_with_object_layer(l, player, terrain));
    CHECK(ctx, !layers::broadphase_layer_collides_with_broadphase_layer(l, bp_moving, bp_static));
    // enabling player vs terrain is symmetric and opens the broadphase pair both ways
    layers::enable_collision(l, player, terrain);
    CHECK(ctx, layers::object_layer_collides_with_object_layer(l, player, terrain));
    CHECK(ctx, layers::object_layer_collides_with_object_layer(l, terrain, player));
    CHECK(ctx, layers::broadphase_layer_collides_with_broadphase_layer(l, bp_moving, bp_static));
    CHECK(ctx, layers::broadphase_layer_collides_with_broadphase_layer(l, bp_static, bp_moving));
    CHECK(ctx, layers::object_layer_collides_with_broadphase_layer(l, player, bp_static));
    CHECK(ctx, layers::object_layer_collides_with_broadphase_layer(l, terrain, bp_moving));
    // debris was not part of it
    CHECK(ctx, !layers::object_layer_collides_with_object_layer(l, debris, terrain));
    CHECK(ctx, !layers::object_layer_collides_with_broadphase_layer(l, debris, bp_static));
    // disabling takes the object pair out but leaves the broadphase pair, as the ts does
    layers::disable_collision(l, player, terrain);
    CHECK(ctx, !layers::object_layer_collides_with_object_layer(l, player, terrain));
    CHECK(ctx, !layers::object_layer_collides_with_object_layer(l, terrain, player));
    CHECK(ctx, layers::broadphase_layer_collides_with_broadphase_layer(l, bp_moving, bp_static));
    // a layer added after a pair was enabled keeps the earlier table entries where they were
    layers::enable_collision(l, player, debris);
    const uint32_t bp_sensor = layers::add_broadphase_layer(l);
    const uint32_t sensor = layers::add_object_layer(l, bp_sensor);
    CHECK(ctx, l.broadphase_layers == 3 && l.object_layers == 4);
    CHECK(ctx, layers::object_layer_collides_with_object_layer(l, player, debris));
    CHECK(ctx, layers::broadphase_layer_collides_with_broadphase_layer(l, bp_moving, bp_static));
    CHECK(ctx, layers::object_layer_collides_with_broadphase_layer(l, sensor, bp_sensor));
    CHECK(ctx, !layers::object_layer_collides_with_object_layer(l, sensor, player));

    // a filter from the layers has everything on; from a body's layer, only what collides with it
    filter::Filter f = filter::create(l);
    CHECK(ctx, filter::filter_object_layer(f, terrain) && filter::filter_broadphase_layer(f, bp_sensor));
    CHECK(ctx, f.collision_mask == ~0u && f.collision_groups == ~0u);
    filter::set_from_layer(f, l, player, 0x1, 0x3);
    CHECK(ctx, filter::filter_object_layer(f, debris));
    CHECK(ctx, !filter::filter_object_layer(f, terrain));
    CHECK(ctx, !filter::filter_object_layer(f, sensor));
    CHECK(ctx, filter::filter_broadphase_layer(f, bp_moving));
    CHECK(ctx, filter::filter_broadphase_layer(f, bp_static));
    CHECK(ctx, !filter::filter_broadphase_layer(f, bp_sensor));
    CHECK(ctx, f.collision_groups == 0x1 && f.collision_mask == 0x3);
    // disabling the last object layer in a broadphase layer disables the broadphase layer; not while another uses it
    filter::enable_all_layers(f, l);
    filter::disable_object_layer(f, l, player);
    CHECK(ctx, filter::filter_broadphase_layer(f, bp_moving));
    filter::disable_object_layer(f, l, debris);
    CHECK(ctx, !filter::filter_broadphase_layer(f, bp_moving));
    filter::enable_broadphase_layer(f, l, bp_moving);
    CHECK(ctx, filter::filter_object_layer(f, player) && filter::filter_object_layer(f, debris));
    filter::disable_broadphase_layer(f, l, bp_moving);
    CHECK(ctx, !filter::filter_object_layer(f, player) && !filter::filter_object_layer(f, debris) && filter::filter_object_layer(f, terrain));
    // copy carries the tables and the mask
    filter::Filter g = filter::create_empty();
    filter::copy(g, f);
    CHECK(ctx, g.enabled_object_layers.length == 4 && !filter::filter_object_layer(g, player) && filter::filter_object_layer(g, terrain));
    // groups and masks collide only when each is in the other's mask
    CHECK(ctx, filter::should_pair_collide(1, 2, 2, 1));
    CHECK(ctx, !filter::should_pair_collide(1, 2, 2, 2));
    CHECK(ctx, !filter::should_pair_collide(1, 1, 2, 1));
    filter::release(f);
    filter::release(g);
    layers::release(l);
    CHECK(ctx, util::error() == 0);
}
