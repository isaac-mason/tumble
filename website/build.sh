#!/bin/sh
# the site in ./dist: its page, and the js examples under examples/. tumble.js is built first (cd js && pnpm run build),
# and each project's dependencies installed
set -e
cd "$(dirname "$0")"

rm -rf ./dist

echo "building the examples..."
(cd ../js/examples && npm run build)

echo "building the website..."
npm run build

echo "copying the examples..."
cp -r ../js/examples/dist ./dist/examples

echo "built: ./dist"
