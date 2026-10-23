#!/bin/sh
# the pinned wasm toolchain: which llvm and binaryen releases, and where fetch-toolchain.sh puts them.
# sourced by fetch-toolchain.sh and toolchain.sh; ci keys its toolchain cache on this file.
LLVM_VERSION=23.1.3
BINARYEN_VERSION=133

TOOLCHAIN_CACHE="${TUMBLE_TOOLCHAIN_CACHE:-${XDG_CACHE_HOME:-$HOME/.cache}/tumble/wasm-toolchain}"
LLVM_DIR="$TOOLCHAIN_CACHE/llvm-$LLVM_VERSION"
BINARYEN_DIR="$TOOLCHAIN_CACHE/binaryen-$BINARYEN_VERSION"
