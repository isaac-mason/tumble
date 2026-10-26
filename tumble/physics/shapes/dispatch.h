#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/physics/shapes/shapes.h"

TUMBLE_NAMESPACE_BEGIN

// every shape type's def and every pair's collide and cast entries, in one place

namespace dispatch {

/**
 * every built-in def and the caller's `user_defs` (each in a user slot, made by define_shape) into shape_defs, then
 * the pair tables: the generic convex rule, then every specialised pair over it. a user type takes part in every
 * generic rule as a built-in does: convex against convex, mesh and plane when it has a support, and inside compounds
 * and wrappers. its own pairs go in after, through set_collide_shape_fn and set_cast_shape_fn. clears the tables
 * first, so a second call leaves them as the first did
 */
void init(const ShapeDef *user_defs = nullptr, uint32_t user_def_count = 0);

}  // namespace dispatch

TUMBLE_NAMESPACE_END
