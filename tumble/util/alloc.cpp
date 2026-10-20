#include "tumble/util/alloc.h"

#include "tumble/util/assert.h"
#include "tumble/util/error.h"
#include "tumble/util/mem.h"

#if __has_include(<stdlib.h>)
#include <stdlib.h>
#define TUMBLE_HOSTED_ALLOCATOR 1
#endif

TUMBLE_NAMESPACE_BEGIN

namespace util {

#ifdef TUMBLE_HOSTED_ALLOCATOR
static void *hosted_allocate(size_t size, size_t alignment, void *) {
#ifdef _MSC_VER
    return _aligned_malloc(size, alignment);
#else
    // aligned_alloc wants a whole number of alignments
    return aligned_alloc(alignment, align_up(size, alignment));
#endif
}

static void hosted_free(void *block, size_t, void *) {
#ifdef _MSC_VER
    _aligned_free(block);
#else
    ::free(block);
#endif
}

constinit static Allocator the_allocator = {hosted_allocate, hosted_free, nullptr};
#else
constinit static Allocator the_allocator = {nullptr, nullptr, nullptr};
#endif

constinit static size_t the_bytes = 0;
constinit static size_t the_blocks = 0;

void set_allocator(const Allocator &allocator) { the_allocator = allocator; }

size_t allocated_bytes() { return the_bytes; }
size_t allocated_blocks() { return the_blocks; }

void *alloc_bytes(size_t bytes, size_t alignment) {
    if (bytes == 0) return nullptr;
    void *block = the_allocator.allocate == nullptr ? nullptr : the_allocator.allocate(bytes, alignment, the_allocator.user);
    if (block == nullptr) {
        set_error(ERROR_OUT_OF_MEMORY);
        return nullptr;
    }
    the_bytes += bytes;
    the_blocks++;
    return block;
}

void free_bytes(void *block, size_t bytes) {
    if (block == nullptr) return;
    TUMBLE_ASSERT(bytes <= the_bytes && the_blocks > 0);
    the_bytes -= bytes;
    the_blocks--;
    the_allocator.free(block, bytes, the_allocator.user);
}

}  // namespace util

TUMBLE_NAMESPACE_END
