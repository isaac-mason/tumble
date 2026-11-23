#!/bin/sh
# run: sh js/wasm/toolchain/build-test.sh
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
if [ -z "$SIMD" ]; then
    LOGS="$(cd "$HERE/../../.." && pwd)/build/test-logs-$$"
    mkdir -p "$LOGS"
    trap 'rm -rf "$LOGS"' EXIT
    for backend in 1 0; do
        status=0
        SIMD=$backend sh "$0" > "$LOGS/$backend" 2>&1 || status=$?
        cat "$LOGS/$backend"
        [ $status = 0 ] || exit $status
    done
    simd_digest=$(grep '^digest:' "$LOGS/1" | cut -d' ' -f2)
    scalar_digest=$(grep '^digest:' "$LOGS/0" | cut -d' ' -f2)
    if [ -z "$simd_digest" ] || [ "$simd_digest" != "$scalar_digest" ]; then
        echo "FAIL the backends' digests differ: simd $simd_digest, scalar $scalar_digest"
        exit 1
    fi
    echo "backends agree"
    exit 0
fi
. "$HERE/toolchain.sh"

BUILD="$ROOT/build/test-$$"
TESTS="$ROOT/tests"
RUNTIME="$ROOT/js/wasm/runtime"
mkdir -p "$BUILD"
trap 'rm -rf "$BUILD"' EXIT

LIST_DIR="$ROOT/build/test-list"
mkdir -p "$LIST_DIR"
: > "$BUILD/test_list.inc"
for f in $(find "$TESTS" -name '*.test.cpp' -not -path "$TESTS/native/*" | sort); do
    for t in $(grep -o '^void test_[a-z0-9_]*(TestCtx' "$f" | sed 's/^void //; s/(TestCtx$//'); do
        echo "TEST($t)" >> "$BUILD/test_list.inc"
    done
done
cmp -s "$BUILD/test_list.inc" "$LIST_DIR/test_list.inc" || mv "$BUILD/test_list.inc" "$LIST_DIR/test_list.inc"

compile_tree "$SRC" "-DTUMBLE_DEBUG"
compile_tree "$RUNTIME" "-DTUMBLE_DEBUG"
compile_set "$CXXFLAGS -DTUMBLE_DEBUG -I$TESTS -I$LIST_DIR" $(find "$TESTS" -name '*.cpp' -not -path "$TESTS/native/*" | sort)
link_wasm "$BUILD/tumble-test.wasm" names
node "$HERE/scripts/check-imports.mjs" "$BUILD/tumble-test.wasm"
echo "simd backend: $([ "$SIMD" = 0 ] && echo scalar || echo wasm simd128)"
node "$TESTS/run.mjs" "$BUILD/tumble-test.wasm"
