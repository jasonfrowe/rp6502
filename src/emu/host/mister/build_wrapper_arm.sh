#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/../../../.." && pwd)"
SRC_DIR="$ROOT_DIR/src/emu/host/mister"
OUT_DIR="${OUT_DIR:-$ROOT_DIR/build/mister-wrapper-arm}"
OUT_BIN="${OUT_BIN:-$OUT_DIR/rp6502-mister-wrapper}"

CC_DEFAULT="/home/rowe/opt/arm-gnu-toolchain-13.2.Rel1-x86_64-arm-none-linux-gnueabihf/bin/arm-none-linux-gnueabihf-gcc"
CC="${CC:-$CC_DEFAULT}"

if [[ ! -x "$CC" ]]; then
    echo "error: ARM compiler not found or not executable: $CC" >&2
    exit 1
fi

mkdir -p "$OUT_DIR"

echo "Building ARM wrapper with: $CC"
"$CC" \
    -std=c11 \
    -O2 \
    -Wall \
    -Wextra \
    -Werror=implicit-function-declaration \
    -I"$SRC_DIR" \
    "$SRC_DIR/wrapper_main.c" \
    "$SRC_DIR/transport.c" \
    -o "$OUT_BIN"

echo "Built: $OUT_BIN"
