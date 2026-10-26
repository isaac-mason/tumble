#pragma once

#include "tumble/config.h"

TUMBLE_NAMESPACE_BEGIN

#ifdef TUMBLE_DEBUG
#define TUMBLE_ASSERT(cond) ((cond) ? (void)0 : __builtin_trap())
#else
#define TUMBLE_ASSERT(cond) ((void)0)
#endif

TUMBLE_NAMESPACE_END
