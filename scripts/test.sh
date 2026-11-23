#!/bin/sh
set -e
cd "$(dirname "$0")/.."

sh scripts/test-native.sh

sh js/wasm/toolchain/scripts/fetch-toolchain.sh
sh js/wasm/toolchain/build-test.sh

cd js
pnpm install --frozen-lockfile
pnpm run build:wasm
pnpm run typecheck
pnpm run build
pnpm test
