#pragma once

#include "tumble/config.h"

TUMBLE_NAMESPACE_BEGIN

// jolt's intrusive references: the count lives in the object, and Ref and RefConst keep it as they are made, copied,
// moved, assigned and destroyed. a type is counted by two functions found beside it, add_ref(const T *) and
// release_ref(const T *), the second freeing the object when it takes the count to 0. hold an object only by these;
// pass a plain pointer to whatever only reads it

template <typename T>
class Ref {
public:
    Ref() : pointer(nullptr) {}
    Ref(T *target) : pointer(target) { hold(); }
    Ref(const Ref &other) : pointer(other.pointer) { hold(); }
    Ref(Ref &&other) noexcept : pointer(other.pointer) { other.pointer = nullptr; }
    ~Ref() { let_go(pointer); }

    // the new one held before the old one goes, so letting go of the old can never free the new
    Ref &operator=(T *target) {
        if (pointer != target) {
            T *old = pointer;
            pointer = target;
            hold();
            let_go(old);
        }
        return *this;
    }
    Ref &operator=(const Ref &other) { return *this = other.pointer; }
    Ref &operator=(Ref &&other) noexcept {
        if (this != &other) {
            T *old = pointer;
            pointer = other.pointer;
            other.pointer = nullptr;
            if (old != pointer) let_go(old);
        }
        return *this;
    }

    operator T *() const { return pointer; }
    T *operator->() const { return pointer; }
    T &operator*() const { return *pointer; }
    T *get() const { return pointer; }

private:
    void hold() {
        if (pointer != nullptr) add_ref(static_cast<const T *>(pointer));
    }
    static void let_go(T *target) {
        if (target != nullptr) release_ref(static_cast<const T *>(target));
    }

    T *pointer;
};

template <typename T>
class RefConst {
public:
    RefConst() : pointer(nullptr) {}
    RefConst(const T *target) : pointer(target) { hold(); }
    RefConst(const RefConst &other) : pointer(other.pointer) { hold(); }
    RefConst(RefConst &&other) noexcept : pointer(other.pointer) { other.pointer = nullptr; }
    RefConst(const Ref<T> &other) : pointer(other.get()) { hold(); }
    ~RefConst() { let_go(pointer); }

    RefConst &operator=(const T *target) {
        if (pointer != target) {
            const T *old = pointer;
            pointer = target;
            hold();
            let_go(old);
        }
        return *this;
    }
    RefConst &operator=(const RefConst &other) { return *this = other.pointer; }
    RefConst &operator=(RefConst &&other) noexcept {
        if (this != &other) {
            const T *old = pointer;
            pointer = other.pointer;
            other.pointer = nullptr;
            if (old != pointer) let_go(old);
        }
        return *this;
    }

    operator const T *() const { return pointer; }
    const T *operator->() const { return pointer; }
    const T &operator*() const { return *pointer; }
    const T *get() const { return pointer; }

private:
    void hold() {
        if (pointer != nullptr) add_ref(pointer);
    }
    static void let_go(const T *target) {
        if (target != nullptr) release_ref(target);
    }

    const T *pointer;
};

TUMBLE_NAMESPACE_END
