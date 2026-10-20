#include "tumble/util/object.h"

void *operator new(unsigned long bytes) { return tumble::util::alloc_bytes((uint32_t)bytes); }
void *operator new[](unsigned long bytes) { return tumble::util::alloc_bytes((uint32_t)bytes); }
void operator delete(void *p) noexcept { tumble::util::free(p); }
void operator delete[](void *p) noexcept { tumble::util::free(p); }
void operator delete(void *p, unsigned long) noexcept { tumble::util::free(p); }
void operator delete[](void *p, unsigned long) noexcept { tumble::util::free(p); }
