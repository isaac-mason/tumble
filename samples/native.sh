#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"

if [ ! -f build/samples/native/build.ninja ]; then
    cmake --preset samples
fi
cmake --build --preset samples

exec build/samples/native/samples/tumble_samples "$@"
