#include "tumble/util/error.h"
#include "test.h"
#include "tumble/util/collections.h"

struct Key3 {
    int32_t x, y, z;
    bool operator==(const Key3 &o) const { return x == o.x && y == o.y && z == o.z; }
};

/** a deliberately poor hash so probe runs and backshift removal get exercised. */
inline uint32_t hash_of(const Key3 &k) { return (uint32_t)(k.x + k.y * 3 + k.z * 5) & 7; }

void test_collections(TestCtx &ctx) {

    // a zeroed Vec is empty; push grows it; swap_remove keeps the rest
    collections::Array<uint32_t> v = {};
    CHECK(ctx, v.length == 0 && v.capacity == 0);
    for (uint32_t i = 0; i < 100; i++) v.push(i * 10);
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
    for (int32_t i = 0; i < 500; i++) m.set({i, -i, i * 2}, (uint32_t)i);
    CHECK(ctx, m.size == 500 && m.capacity >= 1000);
    bool found = true;
    for (int32_t i = 0; i < 500; i++) {
        const uint32_t *value = m.get({i, -i, i * 2});
        found = found && value != nullptr && *value == (uint32_t)i;
    }
    CHECK(ctx, found);
    m.set({7, -7, 14}, 777);
    CHECK(ctx, *m.get({7, -7, 14}) == 777 && m.size == 500);
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
    CHECK(ctx, util::error() == Error::NONE);
}

namespace {

/** an element with a lifetime: every one alive is counted, and each knows the value it was made from */
struct Counted {
    static int32_t alive;
    int32_t value;
    bool moved_from;

    explicit Counted(int32_t v) : value(v), moved_from(false) { alive++; }
    Counted(const Counted &o) : value(o.value), moved_from(false) { alive++; }
    Counted(Counted &&o) noexcept : value(o.value), moved_from(false) {
        o.moved_from = true;
        alive++;
    }
    Counted &operator=(const Counted &o) {
        value = o.value;
        moved_from = false;
        return *this;
    }
    Counted &operator=(Counted &&o) noexcept {
        value = o.value;
        moved_from = false;
        o.moved_from = true;
        return *this;
    }
    ~Counted() { alive--; }
};

int32_t Counted::alive = 0;

}  // namespace

void test_collections_array_lifetimes(TestCtx &ctx) {
    const size_t bytes = util::allocated_bytes();
    {
        collections::Array<Counted> a;
        // each push constructs one; growth moves them, leaving none behind
        for (int32_t i = 0; i < 20; i++) a.push(Counted(i));
        CHECK(ctx, a.length == 20 && Counted::alive == 20);
        bool kept = true;
        for (uint32_t i = 0; i < a.length; i++) kept = kept && a[i].value == (int32_t)i && !a[i].moved_from;
        CHECK(ctx, kept);

        // swap_remove moves the last into the hole and destroys the one it moved; pop destroys the last
        a.swap_remove(3);
        CHECK(ctx, a.length == 19 && a[3].value == 19 && Counted::alive == 19);
        a.swap_remove(18);
        CHECK(ctx, a.length == 18 && Counted::alive == 18);
        a.pop();
        CHECK(ctx, a.length == 17 && Counted::alive == 17);

        // a copy pushed is its own; a move leaves its source moved from
        Counted outside(100);
        a.push(outside);
    CHECK(ctx, a[17].value == 100 && Counted::alive == 19);
        a.push(static_cast<Counted &&>(outside));
    CHECK(ctx, outside.moved_from && Counted::alive == 20);

        // moving the array hands the elements over: none constructed or destroyed, the source empty
        collections::Array<Counted> b(static_cast<collections::Array<Counted> &&>(a));
        CHECK(ctx, a.data == nullptr && a.length == 0 && b.length == 19 && Counted::alive == 20);
        collections::Array<Counted> c;
        c.push(Counted(7));
        CHECK(ctx, Counted::alive == 21);
        c = static_cast<collections::Array<Counted> &&>(b);
        CHECK(ctx, c.length == 19 && b.length == 0 && Counted::alive == 20);

        // clear destroys every element and keeps the memory; the destructor releases it
        const uint32_t capacity = c.capacity;
        c.clear();
        CHECK(ctx, c.length == 0 && c.capacity == capacity && Counted::alive == 1);
    }
    CHECK(ctx, Counted::alive == 0 && util::allocated_bytes() == bytes);

    // zeroed memory is an empty array
    collections::Array<Counted> zeroed;
    util::zero(&zeroed, sizeof(zeroed));
    CHECK(ctx, zeroed.length == 0);
    zeroed.push(Counted(1));
    CHECK(ctx, Counted::alive == 1);
    zeroed.release();
    CHECK(ctx, Counted::alive == 0 && util::allocated_bytes() == bytes);
}
