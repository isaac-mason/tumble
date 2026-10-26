#pragma once

#include <stdint.h>
#include "tumble/config.h"

TUMBLE_NAMESPACE_BEGIN

// bits 0-19 the index in its type's pool, 20-23 the type, 24-31 a sequence that catches an id kept after its constraint
// was removed. the invalid id is all ones: type 15 is no type, so it is never a constraint's

typedef uint32_t ConstraintId;

namespace constraint_id {

constexpr uint32_t CONSTRAINT_INDEX_BITS = 20;
constexpr uint32_t CONSTRAINT_TYPE_BITS = 4;
constexpr uint32_t SEQUENCE_BITS = 8;

constexpr uint32_t CONSTRAINT_INDEX_MASK = (1u << CONSTRAINT_INDEX_BITS) - 1;
constexpr uint32_t CONSTRAINT_TYPE_MASK = (1u << CONSTRAINT_TYPE_BITS) - 1;
constexpr uint32_t SEQUENCE_MASK = (1u << SEQUENCE_BITS) - 1;

constexpr uint32_t TYPE_SHIFT = CONSTRAINT_INDEX_BITS;
constexpr uint32_t SEQUENCE_SHIFT = CONSTRAINT_INDEX_BITS + CONSTRAINT_TYPE_BITS;

constexpr uint32_t MAX_CONSTRAINT_INDEX = CONSTRAINT_INDEX_MASK;
constexpr uint32_t MAX_CONSTRAINT_TYPE = CONSTRAINT_TYPE_MASK;
constexpr uint32_t MAX_SEQUENCE = SEQUENCE_MASK;

}  // namespace constraint_id

/** encoded in the id */
enum class ConstraintType : uint32_t {
    POINT = 0,
    DISTANCE = 1,
    HINGE = 2,
    SLIDER = 3,
    FIXED = 4,
    CONE = 5,
    SWING_TWIST = 6,
    SIX_DOF = 7,
    USER_1 = 8,
    USER_2 = 9,
    USER_3 = 10,
};

namespace constraint_id {

constexpr uint32_t CONSTRAINT_TYPE_COUNT = 11;

constexpr ConstraintId INVALID_CONSTRAINT_ID = 0xffffffffu;
static_assert(CONSTRAINT_TYPE_COUNT <= CONSTRAINT_TYPE_MASK, "the invalid id's type is never a constraint type");

inline ConstraintId ser(uint32_t index, ConstraintType type, uint32_t sequence) {
    return (index & CONSTRAINT_INDEX_MASK) | (((uint32_t)type & CONSTRAINT_TYPE_MASK) << TYPE_SHIFT) | ((sequence & SEQUENCE_MASK) << SEQUENCE_SHIFT);
}
inline uint32_t index(ConstraintId id) { return id & CONSTRAINT_INDEX_MASK; }
inline ConstraintType type(ConstraintId id) { return (ConstraintType)((id >> TYPE_SHIFT) & CONSTRAINT_TYPE_MASK); }
inline uint32_t sequence(ConstraintId id) { return (id >> SEQUENCE_SHIFT) & SEQUENCE_MASK; }

}  // namespace constraint_id

TUMBLE_NAMESPACE_END
