#include "instance.h"
#include "test.h"
#include "util/collections.h"

struct Key3 {
    int32_t x, y, z;
    bool operator==(const Key3 &o) const { return x == o.x && y == o.y && z == o.z; }
};

/** a deliberately poor hash so probe runs and backshift removal get exercised. */
inline uint32_t hash_of(const Key3 &k) { return (uint32_t)(k.x + k.y * 3 + k.z * 5) & 7; }

void test_collections(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));

    // a zeroed Vec is empty; push grows it; swap_remove keeps the rest
    collections::Vec<uint32_t> v = {};
    CHECK(ctx, v.length == 0 && v.capacity == 0);
    for (uint32_t i = 0; i < 100; i++) CHECK(ctx, v.push(i * 10));
    CHECK(ctx, v.length == 100 && v.capacity >= 100);
    CHECK(ctx, v[0] == 0 && v[99] == 990);
    v.swap_remove(0);
    CHECK(ctx, v.length == 99 && v[0] == 990 && v[98] == 980);
    v.swap_remove(98);
    CHECK(ctx, v.length == 98);
    v.release();
    CHECK(ctx, v.data == nullptr && v.capacity == 0);

    // a zeroed Map is empty; set, get, replace, remove, and growth keep every live key findable
    collections::Map<Key3, uint32_t> m = {};
    CHECK(ctx, m.get({1, 2, 3}) == nullptr);
    CHECK(ctx, m.remove({1, 2, 3}) == false);
    for (int32_t i = 0; i < 500; i++) CHECK(ctx, m.set({i, -i, i * 2}, (uint32_t)i));
    CHECK(ctx, m.size == 500 && m.capacity >= 1000);
    bool found = true;
    for (int32_t i = 0; i < 500; i++) {
        const uint32_t *value = m.get({i, -i, i * 2});
        found = found && value != nullptr && *value == (uint32_t)i;
    }
    CHECK(ctx, found);
    CHECK(ctx, m.set({7, -7, 14}, 777) && *m.get({7, -7, 14}) == 777 && m.size == 500);
    for (int32_t i = 0; i < 500; i += 3) CHECK(ctx, m.remove({i, -i, i * 2}));
    bool right = true;
    for (int32_t i = 0; i < 500; i++) {
        const bool present = m.get({i, -i, i * 2}) != nullptr;
        right = right && present == (i % 3 != 0);
    }
    CHECK(ctx, right);
    CHECK(ctx, m.size == 500 - 167);
    m.release();
    CHECK(ctx, m.slots == nullptr && m.capacity == 0 && m.size == 0);
    CHECK(ctx, util::error() == 0);
}
