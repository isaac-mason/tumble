#pragma once

// a failed assert traps (wasm `unreachable`), which JS sees as a RuntimeError naming the function
// from the name section. compiled out of release builds.
#ifdef CORE_DEBUG
#define CC_ASSERT(cond) ((cond) ? (void)0 : __builtin_trap())
#else
#define CC_ASSERT(cond) ((void)0)
#endif
