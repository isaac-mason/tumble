#include "tumble/util/alloc.h"

#include "tumble/instance.h"
#include "tumble/util/assert.h"
#include "tumble/util/mem.h"
#include "tumble/util/offset_allocator.h"

TUMBLE_NAMESPACE_BEGIN

namespace util {

using OffsetAllocator::Allocation;
using OffsetAllocator::Allocator;
using OffsetAllocator::NodeIndex;

constexpr uint32_t PAGE_BYTES = 65536;
constexpr uint32_t BLOCK_ALIGN = 16;
/** the first node tables hold this many; they double, through the heap itself, as blocks pile up. */
constexpr uint32_t INITIAL_NODES = 4096;
/** the allocator needs two free nodes to split a block; relocating its tables takes one more. */
constexpr uint32_t NODE_RESERVE = 3;

/**
 * small blocks come in power-of-two size classes, 16 bytes to SMALL_MAX, and a freed one waits on its class's list for
 * the next request of that class rather than going back to the allocator: vectors growing and dropping in a hot loop
 * then cost a pop and a push. each list holds at most SMALL_CACHE_BYTES; past that a block goes back.
 */
constexpr uint32_t SMALL_CLASSES = 7;
constexpr uint32_t SMALL_MIN = 16;
constexpr uint32_t SMALL_MAX = SMALL_MIN << (SMALL_CLASSES - 1);
constexpr uint32_t SMALL_CACHE_BYTES = 256 * 1024;

/** before every block: what free needs, padded to keep the block itself aligned. */
struct BlockHeader {
    uint32_t node;
    uint32_t bytes;
    /** the block's size class plus one, or 0 for a block sized to its request. */
    uint32_t small_class;
    /** on a class's list, the next block's header address, or 0. */
    uint32_t next;
};
static_assert(sizeof(BlockHeader) == BLOCK_ALIGN);

inline uint32_t small_class_of(uint32_t bytes) { return bytes <= SMALL_MIN ? 0 : 32 - (uint32_t)__builtin_clz(bytes - 1) - 4; }

struct Heap {
    Allocator allocator = {};
    /** memory address the allocator's offset 0 maps to. */
    uint32_t range_begin = 0;
    /** the current tables when they live on the heap (not the first ones), so a relocation can free them. */
    BlockHeader *tables_on_heap = nullptr;
    uint32_t bytes_in_use = 0;
    uint32_t allocations = 0;
    uint32_t grows = 0;
    /** per size class, the header address of the first freed block waiting, or 0, and how many wait. */
    uint32_t small_free[SMALL_CLASSES] = {};
    uint32_t small_free_count[SMALL_CLASSES] = {};
};

// constant-initialised: a static needing a constructor would make the linker wrap every export in
// a call to the constructor list, re-running it on each entry (the build refuses such statics).
constinit static Heap heap = {};

uint32_t memory_bytes() { return (uint32_t)__builtin_wasm_memory_size(0) * PAGE_BYTES; }

/** grows the memory to cover `end`, at least doubling so growth stays rare; false when the host refuses. */
static bool ensure_memory(uint32_t end) {
    const uint32_t have = memory_bytes();
    if (end <= have) return true;
    uint32_t want = have;
    while (want < end) {
        if (want >= 0x80000000u) {
            want = end;
            break;
        }
        want *= 2;
    }
    const uint32_t pages = (align_up(want, PAGE_BYTES) - have) / PAGE_BYTES;
    if (__builtin_wasm_memory_grow(0, pages) == (unsigned long)-1) {
        // try for exactly what was asked before giving up
        const uint32_t minimum = (align_up(end, PAGE_BYTES) - have) / PAGE_BYTES;
        if (minimum == pages || __builtin_wasm_memory_grow(0, minimum) == (unsigned long)-1) return false;
    }
    heap.grows++;
    return true;
}

static uint32_t tables_bytes(uint32_t nodes) { return nodes * (uint32_t)sizeof(Allocator::Node) + nodes * (uint32_t)sizeof(NodeIndex); }

bool heap_init(uint32_t heap_begin) {
    heap = {};
    const uint32_t tables_at = align_up(heap_begin, BLOCK_ALIGN);
    // the tables and at least a page of range have to fit below the top of the 32-bit space
    const uint64_t range_begin_wide = ((uint64_t)tables_at + tables_bytes(INITIAL_NODES) + BLOCK_ALIGN - 1) & ~(uint64_t)(BLOCK_ALIGN - 1);
    if (tables_at < heap_begin || range_begin_wide + PAGE_BYTES > 0x100000000ull) {
        set_error(ERROR_OUT_OF_MEMORY);
        return false;
    }
    const uint32_t range_begin = (uint32_t)range_begin_wide;
    if (!ensure_memory(range_begin)) {
        set_error(ERROR_OUT_OF_MEMORY);
        return false;
    }
    heap.range_begin = range_begin;
    // everything up to the top of the 32-bit space, less the alignment slack at the end
    const uint32_t range_size = (0xffffffffu - range_begin + 1) & ~(BLOCK_ALIGN - 1);
    heap.allocator.init(range_size, ptr<Allocator::Node>(tables_at), ptr<NodeIndex>(tables_at + INITIAL_NODES * sizeof(Allocator::Node)),
                        INITIAL_NODES);
    return true;
}

/** hands out `size` bytes (header included) from the allocator with the memory grown to cover it. */
static BlockHeader *take(uint32_t size) {
    const Allocation allocation = heap.allocator.allocate(size);
    if (allocation.offset == Allocation::NO_SPACE) return nullptr;
    const uint32_t address = heap.range_begin + allocation.offset;
    if (!ensure_memory(address + size)) {
        heap.allocator.free(allocation);
        return nullptr;
    }
    BlockHeader *header = ptr<BlockHeader>(address);
    header->node = allocation.metadata;
    header->bytes = size;
    return header;
}

/** gives a block from take back to the allocator. */
static void give_back(BlockHeader *header) {
    heap.allocator.free({.offset = address_of(header) - heap.range_begin, .metadata = header->node});
}

/** doubles the node tables, placing the new ones on the heap; false when the memory cannot grow. */
static bool grow_tables() {
    const uint32_t nodes = heap.allocator.maxAllocs() * 2;
    BlockHeader *block = take(align_up(tables_bytes(nodes), BLOCK_ALIGN) + sizeof(BlockHeader));
    if (block == nullptr) return false;
    const uint32_t at = address_of(block) + sizeof(BlockHeader);
    heap.allocator.relocateTables(ptr<Allocator::Node>(at), ptr<NodeIndex>(at + nodes * sizeof(Allocator::Node)), nodes);
    BlockHeader *previous = heap.tables_on_heap;
    heap.tables_on_heap = block;
    if (previous != nullptr) give_back(previous);
    return true;
}

void *alloc_bytes(uint32_t bytes) {
    const bool small = bytes <= SMALL_MAX;
    const uint32_t small_class = small ? small_class_of(bytes) : 0;
    BlockHeader *header = nullptr;
    if (small && heap.small_free[small_class] != 0) {
        header = ptr<BlockHeader>(heap.small_free[small_class]);
        heap.small_free[small_class] = header->next;
        heap.small_free_count[small_class]--;
    } else {
        if (heap.allocator.freeNodeCount() < NODE_RESERVE && !grow_tables()) {
            set_error(ERROR_OUT_OF_MEMORY);
            return nullptr;
        }
        header = take((small ? SMALL_MIN << small_class : align_up(bytes, BLOCK_ALIGN)) + sizeof(BlockHeader));
        if (header == nullptr) {
            set_error(ERROR_OUT_OF_MEMORY);
            return nullptr;
        }
        header->small_class = small ? small_class + 1 : 0;
    }
    heap.bytes_in_use += bytes;
    heap.allocations++;
    header->bytes = bytes;
    return header + 1;
}

void free(void *p) {
    if (p == nullptr) return;
    BlockHeader *header = static_cast<BlockHeader *>(p) - 1;
    CC_ASSERT(address_of(header) >= heap.range_begin);
    heap.bytes_in_use -= header->bytes;
    heap.allocations--;
    if (header->small_class != 0) {
        const uint32_t small_class = header->small_class - 1;
        if (heap.small_free_count[small_class] < SMALL_CACHE_BYTES / (SMALL_MIN << small_class)) {
            header->next = heap.small_free[small_class];
            heap.small_free[small_class] = address_of(header);
            heap.small_free_count[small_class]++;
            return;
        }
    }
    give_back(header);
}

void heap_stats(HeapStats *out) {
    const uint32_t memory = memory_bytes();
    const uint32_t largest_free = heap.allocator.largestFreeRegionBelow(memory - heap.range_begin);
    *out = {heap.bytes_in_use, heap.allocations, memory, heap.grows, largest_free};
}

}  // namespace util

TUMBLE_NAMESPACE_END
