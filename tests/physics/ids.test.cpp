#include "physics/bitmask.h"
#include "physics/body/body_id.h"
#include "physics/body/dof.h"
#include "physics/body/sub_shape.h"
#include "physics/constraints/constraint_id.h"
#include "test.h"
#include "test_math.h"

void test_body_id(TestCtx &ctx) {
    // jolt's layout: index in bits 0-22, sequence in 23-30, bit 31 never set
    const body_id::BodyId id = body_id::ser(0x345678u, 0xabu);
    CHECK(ctx, body_id::index(id) == 0x345678u);
    CHECK(ctx, body_id::sequence(id) == 0xabu);
    CHECK(ctx, id == (0xabu << 23 | 0x345678u));
    CHECK(ctx, (id & body_id::BROADPHASE_BIT) == 0);
    // round trips at the extremes; each field masks to its width
    const uint32_t indices[4] = {0u, 1u, 0x400000u, body_id::MAX_BODY_INDEX};
    const uint32_t sequences[4] = {0u, 1u, 0x80u, body_id::MAX_SEQUENCE};
    for (uint32_t index : indices) {
        for (uint32_t sequence : sequences) {
            const body_id::BodyId packed = body_id::ser(index, sequence);
            CHECK(ctx, body_id::index(packed) == index && body_id::sequence(packed) == sequence);
            CHECK(ctx, packed != body_id::INVALID_BODY_ID && (packed & body_id::BROADPHASE_BIT) == 0);
        }
    }
    CHECK(ctx, body_id::sequence(body_id::ser(0, body_id::MAX_SEQUENCE + 1)) == 0);
    CHECK(ctx, body_id::index(body_id::ser(body_id::MAX_BODY_INDEX + 1, 0)) == 0);
    CHECK(ctx, body_id::ser(0, 0) == 0);
    // the invalid id is all ones, which names no body: its broadphase bit is set
    CHECK(ctx, body_id::INVALID_BODY_ID == 0xffffffffu);
    CHECK(ctx, (body_id::INVALID_BODY_ID & body_id::BROADPHASE_BIT) != 0);
    CHECK(ctx, body_id::MAX_BODIES == 0x800000u);
}

void test_constraint_id(TestCtx &ctx) {
    // index in bits 0-19, type in 20-23, sequence in 24-31
    const constraint_id::ConstraintId id = constraint_id::ser(0x23456u, constraint_id::HINGE, 0xabu);
    CHECK(ctx, constraint_id::index(id) == 0x23456u);
    CHECK(ctx, constraint_id::type(id) == constraint_id::HINGE);
    CHECK(ctx, constraint_id::sequence(id) == 0xabu);
    CHECK(ctx, id == (0xabu << 24 | 2u << 20 | 0x23456u));
    // round trips at the extremes for every type
    const uint32_t indices[4] = {0u, 1u, 0x80000u, constraint_id::MAX_CONSTRAINT_INDEX};
    const uint32_t sequences[4] = {0u, 1u, 0x80u, constraint_id::MAX_SEQUENCE};
    bool round_trips = true;
    for (uint32_t type = 0; type < constraint_id::CONSTRAINT_TYPE_COUNT; type++) {
        for (uint32_t index : indices) {
            for (uint32_t sequence : sequences) {
                const constraint_id::ConstraintId packed = constraint_id::ser(index, (constraint_id::ConstraintType)type, sequence);
                round_trips = round_trips && constraint_id::index(packed) == index && constraint_id::type(packed) == type &&
                              constraint_id::sequence(packed) == sequence && packed != constraint_id::INVALID_CONSTRAINT_ID;
            }
        }
    }
    CHECK(ctx, round_trips);
    // each field masks to its width
    CHECK(ctx, constraint_id::index(constraint_id::ser(0xffffffffu, constraint_id::POINT, 0)) == constraint_id::MAX_CONSTRAINT_INDEX);
    CHECK(ctx, constraint_id::sequence(constraint_id::ser(0, constraint_id::POINT, 0xffffffffu)) == constraint_id::MAX_SEQUENCE);
    CHECK(ctx, constraint_id::sequence(constraint_id::ser(0, constraint_id::POINT, constraint_id::MAX_SEQUENCE + 1)) == 0);
    CHECK(ctx, constraint_id::type(constraint_id::ser(0, constraint_id::USER_3, 0)) == constraint_id::USER_3);
    // the invalid id is all ones, whose type is no constraint type
    CHECK(ctx, constraint_id::INVALID_CONSTRAINT_ID == 0xffffffffu);
    CHECK(ctx, constraint_id::type(constraint_id::INVALID_CONSTRAINT_ID) >= constraint_id::CONSTRAINT_TYPE_COUNT);
    CHECK(ctx, constraint_id::MAX_CONSTRAINT_INDEX == 0xfffffu);
}

void test_sub_shape(TestCtx &ctx) {
    using namespace sub_shape;
    // an empty id is all ones; a push writes the level's bits at the bottom
    SubShapeIdBuilder root = builder();
    CHECK(ctx, root.value == EMPTY_SUB_SHAPE_ID && root.current_bit == 0);
    CHECK(ctx, is_empty(root.value));
    // a compound of 5 children takes 3 bits; child 2 then a mesh of 1000 triangles (10 bits), triangle 777
    SubShapeIdBuilder level1;
    push_index(level1, root, 2, 5);
    CHECK(ctx, level1.current_bit == 3);
    CHECK(ctx, (level1.value & 7) == 2);
    SubShapeIdBuilder level2;
    push_index(level2, level1, 777, 1000);
    CHECK(ctx, level2.current_bit == 13);
    // the untouched high bits stay set, as crashcat's empty fill
    CHECK(ctx, (level2.value >> 13) == (EMPTY_SUB_SHAPE_ID >> 13));
    // popping walks back in order, and the remainder after the last level is empty again
    PopResult popped;
    pop_index(popped, level2.value, 5);
    CHECK(ctx, popped.value == 2);
    PopResult popped2;
    pop_index(popped2, popped.remainder, 1000);
    CHECK(ctx, popped2.value == 777);
    CHECK(ctx, is_empty(popped2.remainder));
    // one child needs no bits, so the id is unchanged
    SubShapeIdBuilder single;
    push_index(single, root, 0, 1);
    CHECK(ctx, single.value == EMPTY_SUB_SHAPE_ID && single.current_bit == 0);
    pop_index(popped, EMPTY_SUB_SHAPE_ID, 1);
    CHECK(ctx, popped.value == 0 && is_empty(popped.remainder));
    // bits_for_count matches 32 - clz(count - 1)
    CHECK(ctx, bits_for_count(2) == 1 && bits_for_count(3) == 2 && bits_for_count(4) == 2 && bits_for_count(5) == 3 && bits_for_count(256) == 8 &&
                   bits_for_count(257) == 9);
}

void test_bitmask(TestCtx &ctx) {
    using namespace bitmask;
    CHECK(ctx, add_flag(0, 4) == 4);
    CHECK(ctx, remove_flag(7, 2) == 5);
    CHECK(ctx, toggle_flag(5, 1) == 4);
    CHECK(ctx, has_flag(6, 2) && !has_flag(6, 1));
    CHECK(ctx, does_not_have_flag(6, 1) && !does_not_have_flag(6, 4));
    CHECK(ctx, ALL_FLAG == 0x7fffffffu && NONE_FLAG == 0);
}

void test_dof(TestCtx &ctx) {
    CHECK(ctx, dof::dof(true, true, true, true, true, true) == dof::DOF_ALL);
    CHECK(ctx, dof::dof(true, true, true, false, false, false) == dof::DOF_TRANSLATION_ONLY);
    CHECK(ctx, dof::dof(false, false, false, true, true, true) == dof::DOF_ROTATION_ONLY);
    CHECK(ctx, dof::dof(false, true, false, false, false, true) == (dof::TRANSLATION_Y | dof::ROTATION_Z));
}
