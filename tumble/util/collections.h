#pragma once

// the two containers the core is built from, over the heap. both are plain structs: zeroed
// memory is an empty container, which is what `util::create<T>` gives an object holding one, and
// `release` gives the memory back. elements are trivially copyable; nothing is constructed.
//
// a Map key needs `uint32_t hash_of(const K &)` and `==`.

#include <stdint.h>

#include "util/alloc.h"
#include "util/assert.h"
#include "util/mem.h"

namespace collections {

template <typename T>
struct Vec {
    T *data;
    uint32_t length;
    uint32_t capacity;

    T &operator[](uint32_t i) {
        CC_ASSERT(i < length);
        return data[i];
    }
    const T &operator[](uint32_t i) const {
        CC_ASSERT(i < length);
        return data[i];
    }

    /** room for at least `wanted`; false (error set) when the memory cannot grow. */
    bool reserve(uint32_t wanted) {
        if (wanted <= capacity) return true;
        T *grown = util::alloc<T>(wanted);
        if (grown == nullptr) return false;
        if (data != nullptr) {
            util::copy(grown, data, length * sizeof(T));
            util::free(data);
        }
        data = grown;
        capacity = wanted;
        return true;
    }

    /** as reserve, keeping every element up to the old capacity: for a stack written past its length. */
    bool grow_preserving(uint32_t wanted) {
        if (wanted <= capacity) return true;
        T *grown = util::alloc<T>(wanted);
        if (grown == nullptr) return false;
        if (data != nullptr) {
            util::copy(grown, data, capacity * sizeof(T));
            util::free(data);
        }
        data = grown;
        capacity = wanted;
        return true;
    }

    bool push(const T &value) {
        if (length == capacity && !reserve(capacity == 0 ? 8 : capacity * 2)) return false;
        data[length++] = value;
        return true;
    }

    /** removes element i by moving the last one into its place; order is not kept. */
    void swap_remove(uint32_t i) {
        CC_ASSERT(i < length);
        data[i] = data[--length];
    }

    void release() {
        util::free(data);
        data = nullptr;
        length = 0;
        capacity = 0;
    }
};

/** a Vec released when it goes out of scope: a function's own scratch. */
template <typename T>
struct Owned {
    Vec<T> v = {};
    Owned() = default;
    Owned(const Owned &) = delete;
    Owned &operator=(const Owned &) = delete;
    ~Owned() { v.release(); }
};

/** open addressing with linear probing and backshift removal, kept at most half full. */
template <typename K, typename V>
struct Map {
    struct Slot {
        K key;
        V value;
        uint32_t used;
    };

    Slot *slots;
    /** a power of two, or 0 before the first set. */
    uint32_t capacity;
    uint32_t size;

    /** the value stored for key, or null. */
    V *get(const K &key) const {
        if (capacity == 0) return nullptr;
        const uint32_t mask = capacity - 1;
        for (uint32_t i = hash_of(key) & mask;; i = (i + 1) & mask) {
            Slot &slot = slots[i];
            if (!slot.used) return nullptr;
            if (slot.key == key) return &slot.value;
        }
    }

    /** stores value for key, replacing what was there; false (error set) when the memory cannot grow. */
    bool set(const K &key, const V &value) {
        if ((size + 1) * 2 > capacity && !grow()) return false;
        Slot &slot = probe(slots, capacity - 1, key);
        if (!slot.used) size++;
        slot.key = key;
        slot.value = value;
        slot.used = 1;
        return true;
    }

    /** forgets key; whether it was there. */
    bool remove(const K &key) {
        if (capacity == 0) return false;
        const uint32_t mask = capacity - 1;
        uint32_t hole = hash_of(key) & mask;
        for (;; hole = (hole + 1) & mask) {
            if (!slots[hole].used) return false;
            if (slots[hole].key == key) break;
        }
        // pull later entries of the same run back over the hole unless their home is past it
        for (uint32_t i = (hole + 1) & mask; slots[i].used; i = (i + 1) & mask) {
            const uint32_t home = hash_of(slots[i].key) & mask;
            const bool home_in_gap = hole <= i ? (home > hole && home <= i) : (home > hole || home <= i);
            if (home_in_gap) continue;
            slots[hole] = slots[i];
            hole = i;
        }
        slots[hole].used = 0;
        size--;
        return true;
    }

    void release() {
        util::free(slots);
        slots = nullptr;
        capacity = 0;
        size = 0;
    }

private:
    static Slot &probe(Slot *table, uint32_t mask, const K &key) {
        uint32_t i = hash_of(key) & mask;
        while (table[i].used && !(table[i].key == key)) i = (i + 1) & mask;
        return table[i];
    }

    bool grow() {
        const uint32_t wanted = capacity == 0 ? 64 : capacity * 2;
        Slot *table = util::alloc<Slot>(wanted);
        if (table == nullptr) return false;
        for (uint32_t i = 0; i < wanted; i++) table[i].used = 0;
        for (uint32_t i = 0; i < capacity; i++) {
            const Slot &slot = slots[i];
            if (slot.used) probe(table, wanted - 1, slot.key) = slot;
        }
        util::free(slots);
        slots = table;
        capacity = wanted;
        return true;
    }
};

}  // namespace collections
