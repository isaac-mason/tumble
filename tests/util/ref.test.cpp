#include "test.h"
#include "tumble/util/ref.h"

namespace {

struct Counted {
    mutable int32_t count = 0;
    bool *freed = nullptr;
};

void add_ref(const Counted *target) { target->count++; }
void release_ref(const Counted *target) {
    if (--target->count == 0 && target->freed != nullptr) *target->freed = true;
}

/** a node holding another: freed, it lets go of what it holds */
struct Node {
    mutable int32_t count = 0;
    bool freed = false;
    Ref<Node> child;
};
void add_ref(const Node *target) { target->count++; }
void release_ref(const Node *target) {
    if (--target->count > 0) return;
    Node *node = const_cast<Node *>(target);
    node->freed = true;
    node->child = nullptr;
}

}  // namespace

void test_ref_counts(TestCtx &ctx) {
    bool freed = false;
    Counted a;
    a.freed = &freed;
    {
        Ref<Counted> first = &a;
        CHECK(ctx, a.count == 1 && first.get() == &a && first == &a);
        Ref<Counted> copy = first;
        CHECK(ctx, a.count == 2);
        Ref<Counted> moved = static_cast<Ref<Counted> &&>(copy);
        CHECK(ctx, a.count == 2 && copy.get() == nullptr && moved.get() == &a);

        // assigning the same target changes nothing; nullptr lets go
        moved = &a;
        CHECK(ctx, a.count == 2);
        moved = nullptr;
        CHECK(ctx, a.count == 1 && moved.get() == nullptr);

        // a const reference from a counted one counts too
        RefConst<Counted> reader = first;
        CHECK(ctx, a.count == 2 && reader->count == 2);
        RefConst<Counted> taken = static_cast<RefConst<Counted> &&>(reader);
        CHECK(ctx, a.count == 2 && reader.get() == nullptr);

        // move assignment over another target lets go of the old one
        Counted b;
        Ref<Counted> other = &b;
        other = static_cast<Ref<Counted> &&>(first);
        CHECK(ctx, b.count == 0 && a.count == 2 && first.get() == nullptr);
        CHECK(ctx, !freed);
    }
    CHECK(ctx, a.count == 0 && freed);
}

void test_ref_holds_the_new_before_letting_go_of_the_old(TestCtx &ctx) {
    // a slot holding a parent is given the parent's only child: holding the child first keeps it alive while the
    // parent, freed by the same assignment, lets go of it
    Node child;
    Node parent;
    parent.child = &child;
    Ref<Node> slot = &parent;
    CHECK(ctx, parent.count == 1 && child.count == 1);
    slot = parent.child.get();
    CHECK(ctx, parent.freed && !child.freed && child.count == 1 && slot.get() == &child);
    slot = nullptr;
    CHECK(ctx, child.freed);
}
