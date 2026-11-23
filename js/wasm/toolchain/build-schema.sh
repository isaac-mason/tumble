#!/bin/sh
# run: sh js/wasm/toolchain/build-schema.sh [--check]
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
. "$HERE/toolchain.sh"
BUILD="$ROOT/build/schema-$$"
mkdir -p "$BUILD"
trap 'rm -rf "$BUILD"' EXIT

compile_tree "$SRC" "-DWASM_SCHEMA"
compile_tree "$ROOT/js/wasm/runtime" "-DWASM_SCHEMA"
compile_set "$CXXFLAGS -DWASM_SCHEMA" "$ROOT/js/wasm/adapters.cpp" "$ROOT/js/wasm/bind/schema_main.cpp" "$ROOT/js/wasm/bindings.cpp"
link_wasm "$BUILD/tumble-schema.wasm" names
node "$ROOT/js/wasm/bind/generate.ts" "$BUILD/tumble-schema.wasm" "$@"
