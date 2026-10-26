#include "test.h"
#include "tumble/util/alloc.h"
#include "tumble/util/collections.h"
#include "tumble/util/error.h"
#include "tumble/util/object.h"

/** a bump allocator over a static buffer that remembers the last request and refuses once `fail` is set */
struct Recorder {
    alignas(64) uint8_t buffer[1 << 16];
    size_t used;
    size_t last_size;
    size_t last_alignment;
    size_t freed_bytes;
    uint32_t frees;
    bool fail;
};

static Recorder recorder;

static void *record_allocate(size_t size, size_t alignment, void *user) {
    Recorder &r = *static_cast<Recorder *>(user);
    r.last_size = size;
    r.last_alignment = alignment;
    if (r.fail) return nullptr;
    const size_t at = util::align_up(r.used, alignment);
    if (at + size > sizeof(r.buffer)) return nullptr;
    r.used = at + size;
    return r.buffer + at;
}

static void record_free(void *, size_t size, void *user) {
    Recorder &r = *static_cast<Recorder *>(user);
    r.freed_bytes += size;
    r.frees++;
}

struct alignas(64) Wide {
    float lanes[16];
};

void test_alloc_set_allocator(TestCtx &ctx) {
    util::set_allocator({record_allocate, record_free, &recorder});
    CHECK(ctx, util::allocated_bytes() == 0 && util::allocated_blocks() == 0);

    // a request reaches the allocator with its size and at least the minimum alignment
    float *floats = util::alloc<float>(10);
    CHECK(ctx, floats != nullptr && recorder.last_size == 40 && recorder.last_alignment == util::MIN_ALIGNMENT);
    CHECK(ctx, util::allocated_bytes() == 40 && util::allocated_blocks() == 1);

    // a type aligned past the minimum asks for its own alignment
    Wide *wide = util::alloc<Wide>(2);
    CHECK(ctx, wide != nullptr && recorder.last_size == 128 && recorder.last_alignment == 64);
    CHECK(ctx, (reinterpret_cast<uintptr_t>(wide) & 63) == 0);

    // a free hands back the size the block was asked with
    util::free(wide, 2);
    CHECK(ctx, recorder.frees == 1 && recorder.freed_bytes == 128);
    util::free(floats, 10);
    CHECK(ctx, recorder.frees == 2 && recorder.freed_bytes == 168);
    CHECK(ctx, util::allocated_bytes() == 0 && util::allocated_blocks() == 0);

    // nothing to free never reaches the allocator, and zero bytes allocate nothing
    util::free<float>(nullptr, 10);
    CHECK(ctx, util::alloc_bytes(0) == nullptr && recorder.frees == 2 && util::error() == Error::NONE);

    // create and destroy go through the same allocator, a whole object at a time
    collections::Array<uint32_t> *vec = util::create<collections::Array<uint32_t>>();
    CHECK(ctx, vec != nullptr && vec->length == 0 && recorder.last_size == sizeof(collections::Array<uint32_t>));
    util::destroy(vec);
    CHECK(ctx, recorder.freed_bytes == 168 + sizeof(collections::Array<uint32_t>));
}
