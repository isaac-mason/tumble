#pragma once

#include <stdint.h>

#include "tumble/tumble.h"
#include "tumble/util/alloc.h"
#include "tumble/util/mem.h"

TUMBLE_NAMESPACE_BEGIN

namespace sort {

template <typename T>
inline void assign(T &out, const T &value) {
    out = value;
}

/** stable, in place; `before` is a strict weak ordering, `copy(out, value)` moves an element */
template <typename T, typename Before, typename Copy>
inline void insertion_sort(T *begin, T *end, Before before, Copy copy) {
    if (end - begin < 2) return;
    for (T *i = begin + 1; i < end; i++) {
        if (!before(*i, *(i - 1))) continue;
        T moving;
        copy(moving, *i);
        T *j = i;
        do {
            copy(*j, *(j - 1));
            j--;
        } while (j > begin && before(moving, *(j - 1)));
        copy(*j, moving);
    }
}

template <typename T, typename Before>
inline void insertion_sort(T *begin, T *end, Before before) {
    insertion_sort(begin, end, before, assign<T>);
}

/** runs of this many are insertion sorted before the merges start */
constexpr uint32_t MERGE_RUN = 8;

/** stable, bottom-up over insertion-sorted runs, through `scratch` (room for the range); the result is in place */
template <typename T, typename Before>
inline void merge_sort(T *begin, T *end, Before before, T *scratch) {
    const uint32_t count = (uint32_t)(end - begin);
    if (count < 2) return;
    for (uint32_t start = 0; start < count; start += MERGE_RUN)
        insertion_sort(begin + start, begin + (start + MERGE_RUN < count ? start + MERGE_RUN : count), before);
    T *source = begin;
    T *target = scratch;
    for (uint32_t width = MERGE_RUN; width < count; width *= 2) {
        for (uint32_t start = 0; start < count; start += 2 * width) {
            const uint32_t middle = start + width < count ? start + width : count;
            const uint32_t stop = start + 2 * width < count ? start + 2 * width : count;
            uint32_t left = start;
            uint32_t right = middle;
            for (uint32_t out = start; out < stop; out++) {
                // the left run wins ties, which keeps the sort stable
                if (left < middle && (right >= stop || !before(source[right], source[left]))) {
                    target[out] = source[left++];
                } else {
                    target[out] = source[right++];
                }
            }
        }
        T *swap = source;
        source = target;
        target = swap;
    }
    if (source != begin) util::copy(begin, source, count * sizeof(T));
}

/** through a scratch copy of the range; false, the range untouched, when the scratch cannot be allocated */
template <typename T, typename Before>
inline bool merge_sort(T *begin, T *end, Before before) {
    const uint32_t count = (uint32_t)(end - begin);
    if (count <= MERGE_RUN) {
        insertion_sort(begin, end, before);
        return true;
    }
    T *scratch = util::alloc<T>(count);
    if (scratch == nullptr) return false;
    merge_sort(begin, end, before, scratch);
    util::free(scratch);
    return true;
}

/** lsd over the bytes the keys don't all share, through `scratch`; returns whichever of the two holds the result */
inline uint64_t *radix_sort(uint64_t *keys, uint64_t *scratch, uint32_t count) {
    if (count < 2) return keys;
    uint32_t counts[8][256];
    util::zero(counts, sizeof(counts));
    for (uint32_t i = 0; i < count; i++) {
        const uint64_t key = keys[i];
        for (uint32_t digit = 0; digit < 8; digit++) counts[digit][(key >> (digit * 8)) & 0xff]++;
    }
    uint64_t *from = keys;
    uint64_t *to = scratch;
    for (uint32_t digit = 0; digit < 8; digit++) {
        const uint32_t shift = digit * 8;
        uint32_t *digit_counts = counts[digit];
        // a byte every key shares leaves the order as it is
        if (digit_counts[(from[0] >> shift) & 0xff] == count) continue;
        uint32_t sum = 0;
        for (uint32_t b = 0; b < 256; b++) {
            const uint32_t offset = sum;
            sum += digit_counts[b];
            digit_counts[b] = offset;
        }
        for (uint32_t i = 0; i < count; i++) to[digit_counts[(from[i] >> shift) & 0xff]++] = from[i];
        uint64_t *swap = from;
        from = to;
        to = swap;
    }
    return from;
}

}  // namespace sort

TUMBLE_NAMESPACE_END
