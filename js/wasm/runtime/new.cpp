#include "wasm/runtime/heap.h"

// the freestanding build has no operator new; the offset allocator's own tables are the one user. `new T[n]` works
// because clang keeps the count in front of the block when a type needs destroying, and nothing here does
void *operator new(unsigned long bytes) { return tumble::runtime::heap_allocate((uint32_t)bytes); }
void *operator new[](unsigned long bytes) { return tumble::runtime::heap_allocate((uint32_t)bytes); }
void operator delete(void *p) noexcept { tumble::runtime::heap_free(p); }
void operator delete[](void *p) noexcept { tumble::runtime::heap_free(p); }
void operator delete(void *p, unsigned long) noexcept { tumble::runtime::heap_free(p); }
void operator delete[](void *p, unsigned long) noexcept { tumble::runtime::heap_free(p); }
