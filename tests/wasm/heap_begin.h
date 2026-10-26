#pragma once

#include <stdint.h>

/** the first byte above the module's own data and stack, from the linker */
extern "C" unsigned char __heap_base[];
inline uint32_t heap_begin() { return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(__heap_base)); }
