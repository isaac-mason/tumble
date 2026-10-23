#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"

port="${PORT:-8080}"

if [ -z "${EMSDK:-}" ]; then
    echo "EMSDK is not set. Install emsdk and run: source <emsdk>/emsdk_env.sh" >&2
    exit 1
fi

if [ ! -f build/samples/web/build.ninja ]; then
    cmake --preset samples-web
fi
cmake --build --preset samples-web

echo "serving http://localhost:${port}/tumble_samples.html (ctrl+c to stop)"
exec python3 -m http.server "$port" --bind 127.0.0.1 --directory build/samples/web/samples
