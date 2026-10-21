#!/usr/bin/env bash
# headless.sh <name> "<examples>" "<title>" <frames> <out.png>
#   builds the labs for the web with only the given examples (example_<name>.cpp stems, ; separated) in
#   build/labs/headless-<name>, then renders <frames> frames of the example titled <title> in headless chromium and
#   saves the page. no window opens. e.g. labs/headless.sh mine "cast_ray;sensor" "Cast Ray" 240 /tmp/cast_ray.png
set -euo pipefail

root="$(cd "$(dirname "$0")/.." && pwd)"
name="$1"
examples="$2"
title="$3"
frames="$4"
out="$5"

if [ -z "${EMSDK:-}" ]; then
    export EMSDK="$HOME/Development/emsdk"
fi
if [ ! -d "$root/labs/node_modules/playwright" ]; then
    (cd "$root/labs" && npm install --silent)
fi

build="$root/build/labs/headless-$name"
deps="$root/build/labs/native/_deps"
fetched=()
if [ -d "$deps/sokol-src" ]; then
    fetched=(-DFETCHCONTENT_SOURCE_DIR_SOKOL="$deps/sokol-src" -DFETCHCONTENT_SOURCE_DIR_IMGUI="$deps/imgui-src"
        -DFETCHCONTENT_SOURCE_DIR_CGLTF="$deps/cgltf-src")
fi
cmake -S "$root" -B "$build" -G Ninja -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_TOOLCHAIN_FILE="$EMSDK/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake" \
    -DTUMBLE_BUILD_LABS=ON -DTUMBLE_BUILD_TESTS=OFF -DTUMBLE_BUILD_EXAMPLES=OFF \
    -DTUMBLE_LABS_EXAMPLES="$examples" "${fetched[@]}" >/dev/null
log="$build/headless-build.log"
if ! cmake --build "$build" >"$log" 2>&1; then
    grep -E "error" "$log" | grep -v "Building CXX" || cat "$log"
    exit 1
fi
grep -E "warning: " "$log" || true
node "$root/labs/screenshot.mjs" "$build/labs" "$out" "$title" --screenshot page --frames "$frames"
