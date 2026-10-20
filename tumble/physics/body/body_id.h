#pragma once

// a body index and a sequence number packed in one u32, as jolt's BodyID:
//   bits 0-22: body index
//   bits 23-30: sequence number, which the slot bumps each time a body takes it
//   bit 31: jolt's broadphase bit, which no body id sets, so the invalid id (all ones) is never a body's
// ids are what the broadphase, pairs, constraints and events hold, and the sequence catches a slot reused after its
// body was destroyed.

#include <stdint.h>

namespace body_id {

typedef uint32_t BodyId;

constexpr uint32_t BODY_INDEX_BITS = 23;
constexpr uint32_t SEQUENCE_BITS = 8;
constexpr uint32_t BODY_INDEX_MASK = (1u << BODY_INDEX_BITS) - 1;
constexpr uint32_t SEQUENCE_MASK = (1u << SEQUENCE_BITS) - 1;
constexpr uint32_t SEQUENCE_SHIFT = BODY_INDEX_BITS;
constexpr uint32_t BROADPHASE_BIT = 0x80000000u;

constexpr uint32_t MAX_BODY_INDEX = BODY_INDEX_MASK;
/** the most bodies a world holds at once: every index the id has room for */
constexpr uint32_t MAX_BODIES = MAX_BODY_INDEX + 1;
constexpr uint32_t MAX_SEQUENCE = SEQUENCE_MASK;

constexpr BodyId INVALID_BODY_ID = 0xffffffffu;

inline BodyId ser(uint32_t index, uint32_t sequence) { return (index & BODY_INDEX_MASK) | ((sequence & SEQUENCE_MASK) << SEQUENCE_SHIFT); }
inline uint32_t index(BodyId id) { return id & BODY_INDEX_MASK; }
inline uint32_t sequence(BodyId id) { return (id >> SEQUENCE_SHIFT) & SEQUENCE_MASK; }

}  // namespace body_id
