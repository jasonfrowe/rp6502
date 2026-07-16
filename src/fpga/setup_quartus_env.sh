#!/usr/bin/env bash
set -euo pipefail

QUARTUS_BIN_DEFAULT="$HOME/intelFPGA_lite/17.0/quartus/bin"
QUARTUS_BIN="${QUARTUS_BIN:-$QUARTUS_BIN_DEFAULT}"

if [[ ! -d "$QUARTUS_BIN" ]]; then
    echo "error: Quartus bin directory not found: $QUARTUS_BIN" >&2
    exit 1
fi

export PATH="$QUARTUS_BIN:$PATH"

if ! command -v quartus_sh >/dev/null 2>&1; then
    echo "error: quartus_sh not found after PATH update" >&2
    exit 1
fi

echo "Quartus environment ready"
echo "  QUARTUS_BIN=$QUARTUS_BIN"
echo "  quartus_sh=$(command -v quartus_sh)"
