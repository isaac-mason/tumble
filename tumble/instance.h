#pragma once

#include <stdint.h>
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace util {

/** what `error()` reports: the first failure since the last clear. */
enum WasmError : uint32_t {
    ERROR_NONE = 0,
    ERROR_OUT_OF_MEMORY = 1,
    ERROR_BAD_HANDLE = 2,
    ERROR_QUOTA = 3,
};

/** what `init` records, and the error word every failing call writes. */
struct Instance {
    uint32_t heap_begin;
    uint32_t initialized;
    /** set when a call could not do what it was asked */
    uint32_t error;
};

Instance &instance();

inline void set_error(uint32_t error) {
    if (instance().error == ERROR_NONE) instance().error = error;
}

/** starts the heap at heap_begin; false (error set) when the memory cannot hold it */
bool init(uint32_t heap_begin);

uint32_t abi_version();
/** the first error since the last clear, a WasmError; 0 when none. */
inline uint32_t error() { return instance().error; }
inline void error_clear() { instance().error = ERROR_NONE; }
/** dot((1, 2, 3), (1, 2, 3)) through the simd path: 14 when simd runs */
float simd_check();

}  // namespace util

TUMBLE_NAMESPACE_END
