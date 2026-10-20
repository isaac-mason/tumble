// (C) Sebastian Aaltonen 2023
// MIT License (see file: OFFSET_ALLOCATOR_LICENSE)
//
// https://github.com/sebbbi/OffsetAllocator, adapted for the core: no standard library, no
// destructor (a static may not run code at exit here), and the node tables can be handed in and
// relocated so the heap can grow its own through itself. offsets are relative to whatever range
// the caller manages; the core's heap is one user, anything else that suballocates a range is another.

#pragma once

#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace OffsetAllocator {
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned int uint32;

// 16 bit offsets mode will halve the metadata storage cost
// But it only supports up to 65536 maximum allocation count
#ifdef USE_16_BIT_NODE_INDICES
typedef uint16 NodeIndex;
#else
typedef uint32 NodeIndex;
#endif

static constexpr uint32 NUM_TOP_BINS = 32;
static constexpr uint32 BINS_PER_LEAF = 8;
static constexpr uint32 TOP_BINS_INDEX_SHIFT = 3;
static constexpr uint32 LEAF_BINS_INDEX_MASK = 0x7;
static constexpr uint32 NUM_LEAF_BINS = NUM_TOP_BINS * BINS_PER_LEAF;

struct Allocation {
    static constexpr uint32 NO_SPACE = 0xffffffff;

    uint32 offset = NO_SPACE;
    NodeIndex metadata = NO_SPACE;  // internal: node index
};

struct StorageReport {
    uint32 totalFreeSpace;
    uint32 largestFreeRegion;
};

struct StorageReportFull {
    struct Region {
        uint32 size;
        uint32 count;
    };

    Region freeRegions[NUM_LEAF_BINS];
};

class Allocator {
public:
    struct Node {
        static constexpr NodeIndex unused = 0xffffffff;

        uint32 dataOffset = 0;
        uint32 dataSize = 0;
        NodeIndex binListPrev = unused;
        NodeIndex binListNext = unused;
        NodeIndex neighborPrev = unused;
        NodeIndex neighborNext = unused;
        bool used = false;  // TODO: Merge as bit flag
    };

    /** an allocator with no range yet; `init` gives it one. constexpr so a static holding one needs no constructor. */
    constexpr Allocator() = default;
    /** owns tables for `maxAllocs` nodes, taken from the core heap. */
    Allocator(uint32 size, uint32 maxAllocs = 128 * 1024);
    /** manages `size` with caller-placed tables of `maxAllocs` nodes; the core heap bootstraps itself this way. */
    void init(uint32 size, Node *nodes, NodeIndex *freeNodes, uint32 maxAllocs);
    void reset();
    /** gives owned tables back; an allocator over caller-placed tables has nothing to release. */
    void release();

    Allocation allocate(uint32 size);
    void free(Allocation allocation);

    /** moves the bookkeeping into larger caller-placed tables; the old tables are the caller's to free. */
    void relocateTables(Node *nodes, NodeIndex *freeNodes, uint32 maxAllocs);
    /** nodes left for splits and merges; `allocate` needs two. */
    uint32 freeNodeCount() const { return m_freeOffset + 1; }
    uint32 maxAllocs() const { return m_maxAllocs; }

    uint32 allocationSize(Allocation allocation) const;
    StorageReport storageReport() const;
    /** the largest free region's size counting only its part below offset `end`, exact; walks the largest bins' lists. */
    uint32 largestFreeRegionBelow(uint32 end) const;
    StorageReportFull storageReportFull() const;

private:
    uint32 insertNodeIntoBin(uint32 size, uint32 dataOffset);
    void removeNodeFromBin(uint32 nodeIndex);
    void resetTables();

    uint32 m_size = 0;
    uint32 m_maxAllocs = 0;
    uint32 m_freeStorage = 0;

    uint32 m_usedBinsTop = 0;
    uint8 m_usedBins[NUM_TOP_BINS] = {};
    NodeIndex m_binIndices[NUM_LEAF_BINS] = {};

    Node *m_nodes = nullptr;
    NodeIndex *m_freeNodes = nullptr;
    uint32 m_freeOffset = 0;
    bool m_ownsTables = false;
};
}  // namespace OffsetAllocator

TUMBLE_NAMESPACE_END
