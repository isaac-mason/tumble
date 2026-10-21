#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"

if [ ! -f build/labs/native/build.ninja ]; then
    cmake --preset labs
fi
cmake --build --preset labs

exec build/labs/native/labs/tumble_labs "$@"
