#!/bin/sh
set -e
cd "$(dirname "$0")/.."

export CC="${CC:-clang}" CXX="${CXX:-clang++}"
cmake --preset release
cmake --build --preset release
ctest --preset release -j 4

if g++ --version 2>/dev/null | grep -q "Free Software Foundation"; then
    CC=gcc CXX=g++ cmake --preset gcc
    cmake --build --preset gcc
    CC=gcc CXX=g++ ctest --preset gcc -j 4
else
    echo "no gcc: skipping the gcc build"
fi

sh js/wasm/toolchain/scripts/fetch-toolchain.sh
sh js/wasm/toolchain/build-test.sh

cd js
pnpm install --frozen-lockfile
pnpm run build:wasm
pnpm run typecheck
pnpm run build
pnpm test
