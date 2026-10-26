#pragma once

/** an allocator that refuses, then an allocation: it sets the error word and traps, so this never returns */
void out_of_memory_traps();
