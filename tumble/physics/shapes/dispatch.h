#pragma once

// crashcat's register.ts, register-all.ts and every shape's register(): fills shape_defs and lays
// out the collide and cast tables in full, one place for every pair. each further type (plane,
// triangle mesh, the compounds, the wrappers) adds its def and its rows to init.

namespace dispatch {

/**
 * every built-in def into shape_defs, then the pair tables: the generic convex rule, then every
 * specialised pair over it. clears the tables first, so a second call leaves them as the first did.
 */
void init();

}  // namespace dispatch
