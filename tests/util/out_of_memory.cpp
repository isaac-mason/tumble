#include "out_of_memory.h"

#include "tumble/util/alloc.h"
#include "tumble/util/error.h"

#ifdef __wasm__
#include "wasm/runtime/export.h"
#endif

using namespace tumble;

static void *refuse(size_t, size_t, void *) { return nullptr; }
static void free_nothing(void *, size_t, void *) {}

void out_of_memory_traps() {
    util::set_allocator({refuse, free_nothing, nullptr});
    util::alloc<float>(4);
}

#ifdef __wasm__
WASM_EXPORT void probe_out_of_memory() { out_of_memory_traps(); }
WASM_EXPORT uint32_t probe_error() { return (uint32_t)util::error(); }
#endif
