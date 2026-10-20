// stand-in for musl's features.h: the vendored math sources only need `hidden`.
#ifndef BONGLE_MUSL_FEATURES_H
#define BONGLE_MUSL_FEATURES_H

#define hidden __attribute__((__visibility__("hidden")))

#endif
