#!/bin/sh
# run: sh wasm/toolchain/build-test.sh
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
. "$HERE/toolchain.sh"

BUILD="$ROOT/build/test-$$"
TESTS="$ROOT/tests"
RUNTIME="$ROOT/wasm/runtime"
mkdir -p "$BUILD"
trap 'rm -rf "$BUILD"' EXIT

# the test list: one TEST(fn) per test function, in file order. it lives at a fixed path, rewritten
# only when it changes, so the cached test objects stay current between runs.
LIST_DIR="$ROOT/build/test-list"
mkdir -p "$LIST_DIR"
: > "$BUILD/test_list.inc"
for f in $(find "$TESTS" -name '*.test.cpp' | sort); do
    for t in $(grep -o '^void test_[a-z0-9_]*(TestCtx' "$f" | sed 's/^void //; s/(TestCtx$//'); do
        echo "TEST($t)" >> "$BUILD/test_list.inc"
    done
done
cmp -s "$BUILD/test_list.inc" "$LIST_DIR/test_list.inc" || mv "$BUILD/test_list.inc" "$LIST_DIR/test_list.inc"

compile_tree "$SRC" "-DTUMBLE_DEBUG"
compile_tree "$RUNTIME" "-DTUMBLE_DEBUG"
compile_tree "$TESTS" "-DTUMBLE_DEBUG -I$TESTS -I$LIST_DIR"
compile_c
link_wasm "$BUILD/tumble-test.wasm" names
node "$HERE/scripts/check-imports.mjs" "$BUILD/tumble-test.wasm"
node "$TESTS/run.mjs" "$BUILD/tumble-test.wasm"
