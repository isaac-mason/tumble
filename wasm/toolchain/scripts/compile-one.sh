#!/bin/sh
# one source into the object cache, for toolchain.sh's compile_set: the object and its dependency
# list (one path a line) are written under temp names and moved into place, the list first.
f="$1"
rel="${f#$COMPILE_HERE/}"
o="$COMPILE_CACHE/${rel%.*}.o"
mkdir -p "$(dirname "$o")"
tmp="$o.$$"
"$COMPILE_CLANG" $COMPILE_FLAGS -MMD -MF "$tmp.d" -c "$f" -o "$tmp" || exit 255
sed -e 's/^[^:]*://' -e 's/\\$//' "$tmp.d" | tr ' ' '\n' | grep -v '^$' > "$tmp.deps"
mv "$tmp.deps" "$o.deps"
mv "$tmp" "$o"
