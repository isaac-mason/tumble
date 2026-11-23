#!/bin/sh
# every test, as ci runs them: native, then wasm on both simd backends, then js/dist/tumble.wasm rebuilt and the js
# package's. run: sh scripts/test.sh
set -e
cd "$(dirname "$0")/.."

# clang, as the library is built and tested locally
CC="${CC:-clang}" CXX="${CXX:-clang++}" cmake --preset release
cmake --build --preset release
ctest --preset release -j 4

sh js/wasm/toolchain/scripts/fetch-toolchain.sh
sh js/wasm/toolchain/build-test.sh

cd js
pnpm install --frozen-lockfile
pnpm run build:wasm
pnpm run typecheck
pnpm run build
pnpm test
