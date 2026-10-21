#!/bin/sh
# js/dist/tumble.wasm: the library, the runtime and the generated extern "C" layer, optimised. the output is
# committed, so the package's users never need the toolchain. the schema is regenerated first, from the manifest.
# run: sh wasm/toolchain/build.sh
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
sh "$HERE/build-schema.sh"
. "$HERE/toolchain.sh"
mkdir -p "$ROOT/js/dist"
compile_tree "$SRC" ""
compile_tree "$ROOT/wasm/runtime" ""
compile_set "$CXXFLAGS" "$ROOT/wasm/adapters.cpp" "$ROOT/wasm/generated/exports.cpp"
link_wasm "$ROOT/js/dist/tumble.wasm"
node "$HERE/scripts/check-imports.mjs" "$ROOT/js/dist/tumble.wasm"
echo "built js/dist/tumble.wasm ($(wc -c < "$ROOT/js/dist/tumble.wasm") bytes)"
