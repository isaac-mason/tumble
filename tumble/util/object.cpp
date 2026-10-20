#include "util/object.h"

void *operator new(unsigned long bytes) { return util::alloc_bytes((uint32_t)bytes); }
void *operator new[](unsigned long bytes) { return util::alloc_bytes((uint32_t)bytes); }
void operator delete(void *p) noexcept { util::free(p); }
void operator delete[](void *p) noexcept { util::free(p); }
void operator delete(void *p, unsigned long) noexcept { util::free(p); }
void operator delete[](void *p, unsigned long) noexcept { util::free(p); }
