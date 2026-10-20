#!/bin/sh
# download the pinned llvm (clang + lld) and binaryen releases into the toolchain cache, checksum-verified, keeping only
# what the build runs: clang, wasm-ld, clang's builtin headers, and wasm-opt. macOS arm64 and Linux x64.
# run: sh wasm/toolchain/scripts/fetch-toolchain.sh   (TUMBLE_TOOLCHAIN_CACHE overrides where it goes)
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
. "$HERE/toolchain-pins.sh"

LLVM_MAJOR="${LLVM_VERSION%%.*}"
LLVM_URL="https://github.com/llvm/llvm-project/releases/download/llvmorg-$LLVM_VERSION"
BINARYEN_URL="https://github.com/WebAssembly/binaryen/releases/download/version_$BINARYEN_VERSION"

# llvm's macOS archive comes as .tar.xz, which macOS tar reads with nothing installed; the Linux one as .tar.zst, which
# is 40% smaller than its .tar.xz and needs zstd.
case "$(uname -s)-$(uname -m)" in
Darwin-arm64)
    LLVM_ARCHIVE="LLVM-$LLVM_VERSION-macOS-ARM64.tar.xz"
    LLVM_SHA256=3dcfd88aec10be204a24037bcb0dc0ad777882e9dbe8ac60e2639060d8a794b1
    BINARYEN_ARCHIVE="binaryen-version_$BINARYEN_VERSION-arm64-macos.tar.gz"
    BINARYEN_SHA256=ad66da82ac13f163e424b1643f16c6dfcccc98b5966296b43e52d3cab04f84a8
    ;;
Linux-x86_64)
    LLVM_ARCHIVE="LLVM-$LLVM_VERSION-Linux-X64.tar.zst"
    LLVM_SHA256=14d2f701eb68fb799001bdea6231048f3990690fa2f406d563555ff8f744daba
    BINARYEN_ARCHIVE="binaryen-version_$BINARYEN_VERSION-x86_64-linux.tar.gz"
    BINARYEN_SHA256=2dc9c7813f5375db93d96ead4b78222fcc3e2677bbb832297af4797782a37489
    ;;
*)
    echo "no pinned wasm toolchain for $(uname -s) $(uname -m); set WASM_CLANG, WASM_LD and WASM_OPT instead" >&2
    exit 1
    ;;
esac

sha256_of() {
    if command -v sha256sum >/dev/null 2>&1; then
        sha256sum "$1" | cut -d' ' -f1
    else
        shasum -a 256 "$1" | cut -d' ' -f1
    fi
}

# fetch <url> <file> <sha256> : downloads to file, refusing it if the checksum differs. a file already there passed
# the check on an earlier run that stopped before unpacking it.
fetch() {
    [ -f "$2" ] && return 0
    echo "fetching $1"
    curl -fL --retry 3 --progress-bar -o "$2.part" "$1"
    actual="$(sha256_of "$2.part")"
    if [ "$actual" != "$3" ]; then
        echo "checksum mismatch for $1: expected $3, got $actual" >&2
        exit 1
    fi
    mv "$2.part" "$2"
}

# unpack <archive> <dest> <member...> : the named members of the archive, its top directory stripped, into dest. the
# archive is removed once unpacked, and dest is moved into place whole, so a cut-short run leaves nothing half there.
unpack() {
    archive="$1"
    dest="$2"
    shift 2
    staging="$dest.staging"
    mkdir -p "$staging"
    case "$archive" in
    # llvm compresses with a 1 GiB window, past zstd's default 128 MiB decode limit
    *.tar.zst) zstd -dc --long=31 "$archive" | tar -x -C "$staging" --strip-components=1 "$@" ;;
    *) tar -xf "$archive" -C "$staging" --strip-components=1 "$@" ;;
    esac
    mv "$staging" "$dest"
    rm -f "$archive"
}

mkdir -p "$TOOLCHAIN_CACHE"

if [ -x "$LLVM_DIR/bin/clang" ] && [ -x "$LLVM_DIR/bin/wasm-ld" ]; then
    echo "llvm $LLVM_VERSION: $LLVM_DIR"
else
    case "$LLVM_ARCHIVE" in *.tar.zst) command -v zstd >/dev/null 2>&1 || {
        echo "zstd is needed to unpack $LLVM_ARCHIVE" >&2
        exit 1
    } ;; esac
    fetch "$LLVM_URL/$LLVM_ARCHIVE" "$TOOLCHAIN_CACHE/$LLVM_ARCHIVE" "$LLVM_SHA256"
    top="${LLVM_ARCHIVE%.tar.*}"
    # clang and wasm-ld are links to clang-<major> and lld, which come along
    unpack "$TOOLCHAIN_CACHE/$LLVM_ARCHIVE" "$LLVM_DIR" \
        "$top/bin/clang" "$top/bin/clang-$LLVM_MAJOR" "$top/bin/wasm-ld" "$top/bin/lld" "$top/lib/clang/$LLVM_MAJOR/include"
    echo "llvm $LLVM_VERSION: $LLVM_DIR"
fi

if [ -x "$BINARYEN_DIR/bin/wasm-opt" ]; then
    echo "binaryen $BINARYEN_VERSION: $BINARYEN_DIR"
else
    fetch "$BINARYEN_URL/$BINARYEN_ARCHIVE" "$TOOLCHAIN_CACHE/$BINARYEN_ARCHIVE" "$BINARYEN_SHA256"
    top="binaryen-version_$BINARYEN_VERSION"
    unpack "$TOOLCHAIN_CACHE/$BINARYEN_ARCHIVE" "$BINARYEN_DIR" "$top/bin/wasm-opt" "$top/lib"
    echo "binaryen $BINARYEN_VERSION: $BINARYEN_DIR"
fi
