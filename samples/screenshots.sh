#!/usr/bin/env bash
# screenshots.sh [id filter...]: renders each sample offscreen to samples/screenshots/<id>.png, the pictures the README
# links to the samples on the web. no window opens. macOS only
set -euo pipefail

root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"

if [ ! -f build/samples/native/build.ninja ]; then
    cmake --preset samples
fi
cmake --build --preset samples --target tumble_samples_screenshots

exec build/samples/native/samples/tumble_samples_screenshots "$@"
