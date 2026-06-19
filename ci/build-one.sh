#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -ne 1 ]; then
    echo "usage: $0 <source.c>" >&2
    exit 2
fi

SRC=$1

if [ ! -f "$SRC" ]; then
    echo "source file not found: $SRC" >&2
    exit 1
fi

DIR=$(dirname "$SRC")
BASE=$(basename "$SRC" .c)
OUT="build/$DIR/$BASE"

mkdir -p "$(dirname "$OUT")"

echo "Building $SRC"
echo "Output: $OUT"

cc \
    -std=gnu11 \
    -Wall \
    -Wextra \
    -Werror=implicit-function-declaration \
    -I"$DIR" \
    "$SRC" \
    -o "$OUT" \
    -lgpiod
