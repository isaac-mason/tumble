#include "test.h"
#include "test_rng.h"
#include "tumble/util/alloc.h"
#include "tumble/util/sort.h"

namespace {

/** a key with many ties, and the position it started at to see stability */
struct Item {
    uint32_t key;
    uint32_t position;
};

bool key_before(const Item &a, const Item &b) { return a.key < b.key; }

/** ascending by key, and items with equal keys in the order they started in */
bool sorted_stably(const Item *items, uint32_t count) {
    for (uint32_t i = 1; i < count; i++) {
        if (items[i - 1].key > items[i].key) return false;
        if (items[i - 1].key == items[i].key && items[i - 1].position > items[i].position) return false;
    }
    return true;
}

void fill(Item *items, uint32_t count, uint32_t seed, uint32_t distinct_keys) {
    for (uint32_t i = 0; i < count; i++) items[i] = {(lcg32(seed) >> 8) % distinct_keys, i};
}

}  // namespace

void test_sort_insertion_is_stable(TestCtx &ctx) {
    Item items[64];
    bool ok = true;
    for (uint32_t seed = 1; seed <= 50; seed++) {
        const uint32_t count = seed % 64;
        fill(items, count, seed, 1 + seed % 7);
        sort::insertion_sort(items, items + count, key_before);
        ok = ok && sorted_stably(items, count);
    }
    CHECK(ctx, ok);

    // the copy hook moves every element the sort moves
    uint32_t copies = 0;
    Item reversed[4] = {{3, 0}, {2, 1}, {1, 2}, {0, 3}};
    sort::insertion_sort(reversed, reversed + 4, key_before, [&](Item &out, const Item &value) {
        out = value;
        copies++;
    });
    CHECK(ctx, sorted_stably(reversed, 4) && copies > 0);

    // an empty range and one element are left as they are
    sort::insertion_sort(items, items, key_before);
    items[0] = {9, 0};
    sort::insertion_sort(items, items + 1, key_before);
    CHECK(ctx, items[0].key == 9);
}

void test_sort_merge_matches_insertion(TestCtx &ctx) {
    const uint32_t max_count = 700;
    Item *merged = util::alloc<Item>(max_count);
    Item *inserted = util::alloc<Item>(max_count);
    bool ok = true;
    for (uint32_t round = 0; round < 30; round++) {
        const uint32_t count = round * 23 + round % 3;
        fill(merged, count, round + 100, 1 + round * 3);
        fill(inserted, count, round + 100, 1 + round * 3);
        ok = ok && sort::merge_sort(merged, merged + count, key_before);
        sort::insertion_sort(inserted, inserted + count, key_before);
        ok = ok && sorted_stably(merged, count);
        for (uint32_t i = 0; i < count; i++) ok = ok && merged[i].key == inserted[i].key && merged[i].position == inserted[i].position;
    }
    CHECK(ctx, ok);
    util::free(merged);
    util::free(inserted);
}
