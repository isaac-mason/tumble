#!/bin/sh
# shared by build.sh and build-test.sh: the compiler, the flags, and the compile loop.
#
# the build is freestanding wasm32: no libc, no libm, no allocator, memory imported from JS. clang, wasm-ld and wasm-opt
# only; nothing of emscripten is used.

# the pinned llvm and binaryen releases, which scripts/fetch-toolchain.sh downloads. without them it falls back to brew's
# llvm and binaryen, with a warning: another clang may build different bytes.
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
    echo "warning: the pinned wasm toolchain (llvm $LLVM_VERSION, binaryen $BINARYEN_VERSION) is not fetched; falling back to whatever llvm and binaryen are installed. run sh wasm/toolchain/scripts/fetch-toolchain.sh" >&2
fi
CLANG="${WASM_CLANG:-$(find_tool "$LLVM_DIR/bin/clang" /opt/homebrew/opt/llvm/bin/clang /usr/local/opt/llvm/bin/clang)}"
WASM_LD="${WASM_LD:-$(find_tool "$LLVM_DIR/bin/wasm-ld" /opt/homebrew/opt/llvm/bin/wasm-ld /usr/local/opt/llvm/bin/wasm-ld)}"
WASM_OPT="${WASM_OPT:-$(find_tool "$BINARYEN_DIR/bin/wasm-opt" /opt/homebrew/bin/wasm-opt /usr/local/bin/wasm-opt)}"

if [ ! -x "$CLANG" ] || [ ! -x "$WASM_LD" ] || [ ! -x "$WASM_OPT" ]; then
    echo "wasm toolchain not found: run sh wasm/toolchain/scripts/fetch-toolchain.sh, or set WASM_CLANG, WASM_LD and WASM_OPT" >&2
    exit 1
fi

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
SRC="$ROOT/tumble"
# each build script picks its own object dir under build/ so that builds running at once (several
# people, or agents, testing in the same tree) never share objects or outputs.
BUILD="${BUILD:-$ROOT/build/wasm}"

FEATURES="-msimd128 -mbulk-memory -mnontrapping-fptoint -msign-ext"
# the optimisation level for clang and wasm-opt both; WASM_OPT_LEVEL overrides it. -O3 over -Os:
# +50 KB gzipped for 7 to 16% on meshing, translucency and light, a few % on physics, the same bits.
OPT="${WASM_OPT_LEVEL:--O3}"
# no -ffast-math anywhere: the bits must match between hosts and between runs.
COMMON="--target=wasm32 $OPT $FEATURES -nostdlib -nostdlibinc -ffreestanding -fvisibility=hidden -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-fast-math"
# ours: every warning is an error.
# -Wglobal-constructors: a static needing a constructor would put a constructor list in the
# module, and with no entry point the linker then runs that list on every export call.
CXXFLAGS="$COMMON -std=c++23 -Wall -Wextra -Werror -Wglobal-constructors -I$ROOT"
# musl's: compiled quiet, as C.
CFLAGS="$COMMON -std=c11 -w -I$SRC/math/musl"

LDFLAGS="--no-entry --import-memory --export-dynamic --export=__heap_base --export=__data_end --stack-first -z stack-size=1048576"

# the object list the compile functions append to, in a fixed order so the link is stable.
OBJS=""

# objects are cached under build/obj, one dir per compiler and flag set, so every build reuses what
# is current and -O3 and -Os, or debug and release, never share an object. a source compiles again
# only when it, or a header it includes (clang's dependency list), is newer than its object.
# compiles run in parallel; each writes to a temp name and moves into place whole, so builds
# running at once in one tree never see a half-written object.
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

# compile_tree <dir> <extra cxxflags> : every .cpp under dir
compile_tree() {
    compile_set "$CXXFLAGS $2" $(find "$1" -name '*.cpp' | sort)
}

# compile_c : the vendored c files, musl's
compile_c() {
    compile_set "$CFLAGS" $(find "$SRC/math/musl" -name '*.c' | sort)
}

# link_wasm <out> [names] : links $OBJS. "names" is the test and schema builds: the name section is
# kept so a trap reports its function, and wasm-opt is skipped, since re-optimising the debug binary
# took a minute and made it run slower. the shipped binary goes through wasm-opt and drops names.
link_wasm() {
    out="$1"
    if [ "$2" = "names" ]; then
        "$WASM_LD" $LDFLAGS -o "$out" $OBJS || exit 1
    else
        "$WASM_LD" $LDFLAGS --strip-debug -o "$out" $OBJS || exit 1
        "$WASM_OPT" $OPT --strip-debug --enable-simd --enable-bulk-memory --enable-nontrapping-float-to-int --enable-sign-ext "$out" -o "$out" || exit 1
    fi
}
