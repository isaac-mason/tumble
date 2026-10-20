#pragma once

#include <stdint.h>

#include "tumble/tumble.h"
#include "tumble/util/assert.h"

TUMBLE_NAMESPACE_BEGIN

// a path through a compound shape's children packed into 32 bits, pushed a level at a time by the shape that owns the
// level and popped back out the same way
namespace sub_shape {

typedef uint32_t SubShapeId;

constexpr uint32_t MAX_SUB_SHAPE_ID_BITS = 32;
constexpr SubShapeId EMPTY_SUB_SHAPE_ID = 0xffffffffu;

struct SubShapeIdBuilder {
    uint32_t value;
    uint32_t current_bit;
};

struct PopResult {
    uint32_t value;
    SubShapeId remainder;
};

/** a builder in the empty state. */
inline SubShapeIdBuilder builder() { return {EMPTY_SUB_SHAPE_ID, 0}; }

/**
 * a real path can be all ones too (the last child of a power of two count), so shapes never test this
 * while navigating; they pop their level and range check it.
 */
inline bool is_empty(SubShapeId id) { return id == EMPTY_SUB_SHAPE_ID; }

/** pops a level of `num_bits` off the id. */
inline void pop(PopResult &out, SubShapeId id, uint32_t num_bits) {
    if (num_bits == 0) {
        out.value = 0;
        out.remainder = id;
        return;
    }
    const uint32_t mask = num_bits >= 32 ? 0xffffffffu : (1u << num_bits) - 1;
    const uint32_t fill_bits = EMPTY_SUB_SHAPE_ID << (MAX_SUB_SHAPE_ID_BITS - num_bits);
    out.value = id & mask;
    out.remainder = num_bits >= 32 ? EMPTY_SUB_SHAPE_ID : (id >> num_bits) | fill_bits;
}

/** pushes `id` as a level of `num_bits` onto `source`, into `out`. */
inline void push(SubShapeIdBuilder &out, const SubShapeIdBuilder &source, uint32_t id, uint32_t num_bits) {
    TUMBLE_ASSERT(source.current_bit + num_bits <= MAX_SUB_SHAPE_ID_BITS);
    out.value = source.value;
    out.current_bit = source.current_bit;
    if (num_bits == 0) return;
    const uint32_t level = num_bits >= 32 ? 0xffffffffu : (1u << num_bits) - 1;
    const uint32_t mask = level << out.current_bit;
    out.value = (out.value & ~mask) | ((id & level) << out.current_bit);
    out.current_bit += num_bits;
}

/** bits needed to hold any index below `count`. */
inline uint32_t bits_for_count(uint32_t count) {
    const uint32_t max_index = count - 1;
    return max_index == 0 ? 0 : 32 - __builtin_clz(max_index);
}

/** pushes an index into a list of `count` entries. */
inline void push_index(SubShapeIdBuilder &out, const SubShapeIdBuilder &source, uint32_t index, uint32_t count) {
    push(out, source, index, bits_for_count(count));
}

/** pops an index from a list of `count` entries. */
inline void pop_index(PopResult &out, SubShapeId id, uint32_t count) { pop(out, id, bits_for_count(count)); }

}  // namespace sub_shape

TUMBLE_NAMESPACE_END
