#pragma once

#include <stdint.h>
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace runtime {

/** the heap from heap_begin, installed as tumble's allocator; false (error set) when the memory cannot hold it */
bool init(uint32_t heap_begin);

uint32_t abi_version();

/** dot((1, 2, 3), (1, 2, 3)) through the simd backend: 14 */
float simd_check();

}  // namespace runtime

TUMBLE_NAMESPACE_END
