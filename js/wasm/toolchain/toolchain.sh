#!/bin/sh
# shared by build.sh and build-test.sh: the compiler, the flags, and the compile loop.

# WASM_CLANG / WASM_LD / WASM_OPT override the probe.
. "$(dirname "$0")/scripts/toolchain-pins.sh"
find_tool() {
    for candidate in "$@"; do
        if [ -x "$candidate" ]; then
            echo "$candidate"
            return
        fi
    done
}
if [ -z "$WASM_CLANG$WASM_LD$WASM_OPT" ] && { [ ! -x "$LLVM_DIR/bin/clang" ] || [ ! -x "$BINARYEN_DIR/bin/wasm-opt" ]; }; then
    echo "warning: the pinned wasm toolchain (llvm $LLVM_VERSION, binaryen $BINARYEN_VERSION) is not fetched; falling back to whatever llvm and binaryen are installed. run sh js/wasm/toolchain/scripts/fetch-toolchain.sh" >&2
fi
CLANG="${WASM_CLANG:-$(find_tool "$LLVM_DIR/bin/clang" /opt/homebrew/opt/llvm/bin/clang /usr/local/opt/llvm/bin/clang)}"
WASM_LD="${WASM_LD:-$(find_tool "$LLVM_DIR/bin/wasm-ld" /opt/homebrew/opt/llvm/bin/wasm-ld /usr/local/opt/llvm/bin/wasm-ld)}"
WASM_OPT="${WASM_OPT:-$(find_tool "$BINARYEN_DIR/bin/wasm-opt" /opt/homebrew/bin/wasm-opt /usr/local/bin/wasm-opt)}"

if [ ! -x "$CLANG" ] || [ ! -x "$WASM_LD" ] || [ ! -x "$WASM_OPT" ]; then
    echo "wasm toolchain not found: run sh js/wasm/toolchain/scripts/fetch-toolchain.sh, or set WASM_CLANG, WASM_LD and WASM_OPT" >&2
    exit 1
fi

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../../.." && pwd)"
SRC="$ROOT/tumble"
BUILD="${BUILD:-$ROOT/build/wasm}"

SIMD_FEATURE="-msimd128"
[ "${SIMD:-1}" = 0 ] && SIMD_FEATURE=""
FEATURES="$SIMD_FEATURE -mbulk-memory -mnontrapping-fptoint -msign-ext"
OPT="${WASM_OPT_LEVEL:--O3}"
COMMON="--target=wasm32 $OPT $FEATURES -nostdlib -nostdlibinc -ffreestanding -fvisibility=hidden -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-fast-math"
# -Wglobal-constructors: a static needing a constructor would put a constructor list in the
# module, and with no entry point the linker then runs that list on every export call.
CXXFLAGS="$COMMON -std=c++20 -Wall -Wextra -Werror -Wglobal-constructors -I$ROOT -I$ROOT/js"

LDFLAGS="--no-entry --import-memory --export-dynamic --export=__heap_base --export=__data_end --stack-first -z stack-size=1048576"

OBJS=""

JOBS="${WASM_JOBS:-$(sysctl -n hw.ncpu 2>/dev/null || nproc 2>/dev/null || echo 4)}"
OBJ_ROOT="$ROOT/build/obj"

# is_current <object> : the object and its dependency list exist and nothing listed is newer
is_current() {
    [ -f "$1" ] && [ -f "$1.deps" ] || return 1
    while IFS= read -r dep; do
        if [ ! -e "$dep" ] || [ "$dep" -nt "$1" ]; then return 1; fi
    done < "$1.deps"
    return 0
}

# compile_set <flags> <file...> : the given sources into the cache for those flags, stale ones in
# parallel, every object appended to OBJS
compile_set() {
    flags="$1"
    shift
    cache="$OBJ_ROOT/$(printf '%s' "$CLANG $flags" | cksum | cut -d' ' -f1)"
    stale=""
    for f in "$@"; do
        rel="${f#$ROOT/}"
        o="$cache/${rel%.*}.o"
        OBJS="$OBJS $o"
        is_current "$o" || stale="$stale $f"
    done
    [ -z "$stale" ] && return 0
    printf '%s\n' $stale | COMPILE_CLANG="$CLANG" COMPILE_FLAGS="$flags" COMPILE_CACHE="$cache" COMPILE_HERE="$ROOT" \
        xargs -P "$JOBS" -n 1 sh "$HERE/scripts/compile-one.sh" || exit 1
}

compile_tree() {
    compile_set "$CXXFLAGS $2" $(find "$1" -name '*.cpp' | sort)
}

link_wasm() {
    out="$1"
    if [ "$2" = "names" ]; then
        "$WASM_LD" $LDFLAGS -o "$out" $OBJS || exit 1
    else
        "$WASM_LD" $LDFLAGS --strip-debug -o "$out" $OBJS || exit 1
        "$WASM_OPT" $OPT --strip-debug --enable-simd --enable-bulk-memory --enable-nontrapping-float-to-int --enable-sign-ext "$out" -o "$out" || exit 1
    fi
}
