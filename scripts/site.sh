#!/bin/sh
# ./site, as github pages serves it: the page, the js examples under examples/, the c++ samples under samples/. built
# over the committed js/dist/tumble.wasm, and needs emsdk for the samples. run: sh scripts/site.sh
set -e
cd "$(dirname "$0")/.."

(cd js && pnpm install --frozen-lockfile && pnpm run build)
(cd js/examples && npm ci --no-audit --no-fund && npm run typecheck && npm run build)
(cd website && npm ci --no-audit --no-fund && npm run build)
cmake --preset samples-web
cmake --build --preset samples-web

rm -rf site
mkdir -p site/examples site/samples
cp -r website/dist/. site/
cp -r js/examples/dist/. site/examples/
samples=build/samples/web/samples
cp "$samples/tumble_samples.js" "$samples/tumble_samples.wasm" "$samples/tumble_samples.data" site/samples/
cp "$samples/tumble_samples.html" site/samples/index.html
echo "built: ./site"
