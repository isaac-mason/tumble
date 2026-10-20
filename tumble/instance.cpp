#include "instance.h"

#include "math/simd.h"
#include "util/alloc.h"

namespace util {

static Instance the_instance = {0, 0, ERROR_NONE};

Instance &instance() { return the_instance; }

bool init(uint32_t heap_begin) {
    Instance &inst = the_instance;
    inst.heap_begin = heap_begin;
    inst.error = ERROR_NONE;
    if (!heap_init(heap_begin)) return false;
    inst.initialized = 1;
    return true;
}

uint32_t abi_version() { return 1; }

float simd_check() {
    const float a[3] = {1.0f, 2.0f, 3.0f};
    const v128_t v = v4f::load3(a);
    return v4f::dot3(v, v);
}

}  // namespace util
