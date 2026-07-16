#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/../../../.." && pwd)"
SRC_DIR="$ROOT_DIR/src/emu/host/mister"
OUT_DIR="${OUT_DIR:-$ROOT_DIR/build/mister-tools-arm}"
WRAPPER_BIN="${WRAPPER_BIN:-$OUT_DIR/rp6502-mister-wrapper}"
CTL_BIN="${CTL_BIN:-$OUT_DIR/rp6502-mister-ctl}"

CC_DEFAULT="/home/rowe/opt/arm-gnu-toolchain-13.2.Rel1-x86_64-arm-none-linux-gnueabihf/bin/arm-none-linux-gnueabihf-gcc"
CC="${CC:-$CC_DEFAULT}"

if [[ ! -x "$CC" ]]; then
    echo "error: ARM compiler not found or not executable: $CC" >&2
    exit 1
fi

LDFLAGS_STATIC="${LDFLAGS_STATIC:--static -static-libgcc}"

mkdir -p "$OUT_DIR"

echo "Building ARM MiSTer tools with: $CC"
"$CC" \
    -std=c11 \
    -O2 \
    -Wall \
    -Wextra \
    -Werror=implicit-function-declaration \
    -I"$SRC_DIR" \
    "$SRC_DIR/wrapper_main.c" \
    "$SRC_DIR/transport.c" \
    $LDFLAGS_STATIC \
    -o "$WRAPPER_BIN"

"$CC" \
    -std=c11 \
    -O2 \
    -Wall \
    -Wextra \
    -Werror=implicit-function-declaration \
    -I"$SRC_DIR" \
    "$SRC_DIR/mister_ctl.c" \
    "$SRC_DIR/transport.c" \
    $LDFLAGS_STATIC \
    -o "$CTL_BIN"

echo "Built: $WRAPPER_BIN"
echo "Built: $CTL_BIN"
