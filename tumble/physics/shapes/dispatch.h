#pragma once

#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

// every shape type's def and every pair's collide and cast entries, in one place
namespace dispatch {

/**
 * every built-in def into shape_defs, then the pair tables: the generic convex rule, then every
 * specialised pair over it. clears the tables first, so a second call leaves them as the first did.
 */
void init();

}  // namespace dispatch

TUMBLE_NAMESPACE_END
