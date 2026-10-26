#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/util/alloc.h"
#include "tumble/util/assert.h"
#include "tumble/util/construct.h"
#include "tumble/util/mem.h"

TUMBLE_NAMESPACE_BEGIN

// the containers own what they hold, as jolt's: destroyed, they free it; copied, the copy is deep; moved, the source is
// left empty. zeroed memory is an empty container. an allocation they cannot make traps (util::alloc), so nothing here
// fails
namespace collections {

/**
 * a growable list. an element that is trivially copyable moves as bytes; any other is constructed as it is added,
 * moved when the list grows and destroyed as it goes. `length` may be written directly (set past the old length after a
 * reserve, or back to 0) only for trivially copyable elements, which have no lives to begin or end
 */
template <typename T>
struct Array {
    T *data = nullptr;
    uint32_t length = 0;
    uint32_t capacity = 0;

    static constexpr bool TRIVIAL = __is_trivially_copyable(T);

    Array() = default;
    Array(const Array &other) { copy_from(other); }
    /** deep, into the memory it has when that is enough */
    Array &operator=(const Array &other) {
        if (this != &other) {
            clear();
            reserve(other.length);
            copy_elements(other);
        }
        return *this;
    }
    Array(Array &&other) noexcept : data(other.data), length(other.length), capacity(other.capacity) { other.forget(); }
    Array &operator=(Array &&other) noexcept {
        if (this != &other) {
            release();
            data = other.data;
            length = other.length;
            capacity = other.capacity;
            other.forget();
        }
        return *this;
    }
    ~Array() { release(); }

    T &operator[](uint32_t i) {
        TUMBLE_ASSERT(i < length);
        return data[i];
    }
    const T &operator[](uint32_t i) const {
        TUMBLE_ASSERT(i < length);
        return data[i];
    }

    /** room for at least `wanted`, the elements moved into it */
    void reserve(uint32_t wanted) {
        if (wanted <= capacity) return;
        T *grown = util::alloc<T>(wanted);
        if constexpr (TRIVIAL) {
            if (data != nullptr) util::copy(grown, data, length * sizeof(T));
        } else {
            for (uint32_t i = 0; i < length; i++) {
                util::construct<T>(&grown[i], util::move(data[i]));
                util::destruct(&data[i]);
            }
        }
        util::free(data, capacity);
        data = grown;
        capacity = wanted;
    }

    /** as reserve, keeping every element up to the old capacity: for a stack written past its length */
    void grow_preserving(uint32_t wanted) {
        static_assert(TRIVIAL, "only plain data lives past the length");
        if (wanted <= capacity) return;
        T *grown = util::alloc<T>(wanted);
        if (data != nullptr) util::copy(grown, data, capacity * sizeof(T));
        util::free(data, capacity);
        data = grown;
        capacity = wanted;
    }

    void push(const T &value) {
        if (length == capacity) {
            // the value may be one of ours, which growing moves: held aside first
            T held(value);
            grow();
            util::construct<T>(&data[length], util::move(held));
        } else {
            util::construct<T>(&data[length], value);
        }
        length++;
    }

    void push(T &&value) {
        if (length == capacity) {
            T held(util::move(value));
            grow();
            util::construct<T>(&data[length], util::move(held));
        } else {
            util::construct<T>(&data[length], util::move(value));
        }
        length++;
    }

    void pop() {
        TUMBLE_ASSERT(length > 0);
        util::destruct(&data[--length]);
    }

    /** removes element i by moving the last one into its place; order is not kept */
    void swap_remove(uint32_t i) {
        TUMBLE_ASSERT(i < length);
        if (i != length - 1) data[i] = util::move(data[length - 1]);
        pop();
    }

    /** `count` elements: new ones value initialised (zero for plain data), extra ones destroyed */
    void resize(uint32_t count) {
        if (count > capacity) reserve(count);
        for (uint32_t i = length; i < count; i++) util::construct<T>(&data[i]);
        for (uint32_t i = count; i < length; i++) util::destruct(&data[i]);
        length = count;
    }

    /** trades buffers with `other`, as a pair of moves would, without them */
    void swap(Array &other) {
        T *other_data = other.data;
        const uint32_t other_length = other.length;
        const uint32_t other_capacity = other.capacity;
        other.data = data;
        other.length = length;
        other.capacity = capacity;
        data = other_data;
        length = other_length;
        capacity = other_capacity;
    }

    /** every element destroyed, the memory kept */
    void clear() {
        if constexpr (!TRIVIAL) {
            for (uint32_t i = 0; i < length; i++) util::destruct(&data[i]);
        }
        length = 0;
    }

    /** every element destroyed and the memory freed, before the destructor would */
    void release() {
        clear();
        util::free(data, capacity);
        data = nullptr;
        capacity = 0;
    }

private:
    void grow() { reserve(capacity == 0 ? 8 : capacity * 2); }

    void copy_from(const Array &other) {
        data = nullptr;
        length = 0;
        capacity = 0;
        if (other.length == 0) return;
        reserve(other.length);
        copy_elements(other);
    }

    /** other's elements into this empty list, which has room for them */
    void copy_elements(const Array &other) {
        if constexpr (TRIVIAL) {
            if (other.length > 0) util::copy(data, other.data, other.length * sizeof(T));
        } else {
            for (uint32_t i = 0; i < other.length; i++) util::construct<T>(&data[i], other.data[i]);
        }
        length = other.length;
    }

    void forget() {
        data = nullptr;
        length = 0;
        capacity = 0;
    }
};

/** open addressing with linear probing and backshift removal, at most half full; K needs `hash_of(const K &)` and `==` */
template <typename K, typename V>
struct Map {
    static_assert(__is_trivially_copyable(K) && __is_trivially_copyable(V), "a Map copies its keys and values as bytes");

    struct Slot {
        K key;
        V value;
        uint32_t used;
    };

    Slot *slots = nullptr;
    /** a power of two, or 0 before the first set. */
    uint32_t capacity = 0;
    uint32_t size = 0;

    Map() = default;
    Map(const Map &other) { copy_from(other); }
    Map &operator=(const Map &other) {
        if (this != &other) {
            release();
            copy_from(other);
        }
        return *this;
    }
    Map(Map &&other) noexcept : slots(other.slots), capacity(other.capacity), size(other.size) { other.forget(); }
    Map &operator=(Map &&other) noexcept {
        if (this != &other) {
            release();
            slots = other.slots;
            capacity = other.capacity;
            size = other.size;
            other.forget();
        }
        return *this;
    }
    ~Map() { release(); }

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

    /** stores value for key, replacing what was there. */
    void set(const K &key, const V &value) {
        if ((size + 1) * 2 > capacity) grow();
        Slot &slot = probe(slots, capacity - 1, key);
        if (!slot.used) size++;
        slot.key = key;
        slot.value = value;
        slot.used = 1;
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

    /** every entry forgotten and the memory freed, before the destructor would */
    void release() {
        util::free(slots, capacity);
        forget();
    }

private:
    static Slot &probe(Slot *table, uint32_t mask, const K &key) {
        uint32_t i = hash_of(key) & mask;
        while (table[i].used && !(table[i].key == key)) i = (i + 1) & mask;
        return table[i];
    }

    void grow() {
        const uint32_t wanted = capacity == 0 ? 64 : capacity * 2;
        Slot *table = util::alloc<Slot>(wanted);
        for (uint32_t i = 0; i < wanted; i++) table[i].used = 0;
        for (uint32_t i = 0; i < capacity; i++) {
            const Slot &slot = slots[i];
            if (slot.used) probe(table, wanted - 1, slot.key) = slot;
        }
        util::free(slots, capacity);
        slots = table;
        capacity = wanted;
    }

    void copy_from(const Map &other) {
        forget();
        if (other.capacity == 0) return;
        slots = util::alloc<Slot>(other.capacity);
        util::copy(slots, other.slots, other.capacity * sizeof(Slot));
        capacity = other.capacity;
        size = other.size;
    }

    void forget() {
        slots = nullptr;
        capacity = 0;
        size = 0;
    }
};

}  // namespace collections

TUMBLE_NAMESPACE_END
