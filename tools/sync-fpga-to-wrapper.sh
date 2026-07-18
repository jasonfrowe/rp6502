#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<EOF
Usage:
  tools/sync-fpga-to-wrapper.sh [--dry-run]

Sync FPGA sources from this repository to the sibling 3s-mister-arm wrapper-core seed:
  src/fpga/  ->  ../3s-mister-arm/vendor/Menu_MiSTer/

Options:
  --dry-run   Show what would change without writing files.
  -h, --help  Show this help.

Environment:
  TARGET_REPO   Override sibling repo path (default: ../3s-mister-arm)
EOF
}

DRY_RUN=0
while [ "$#" -gt 0 ]; do
    case "$1" in
        --dry-run)
            DRY_RUN=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "unknown argument: $1" >&2
            usage >&2
            exit 1
            ;;
    esac
done

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC_DIR="${ROOT_DIR}/src/fpga"
TARGET_REPO="${TARGET_REPO:-${ROOT_DIR}/../3s-mister-arm}"
TARGET_DIR="${TARGET_REPO}/vendor/Menu_MiSTer"

if ! command -v rsync >/dev/null 2>&1; then
    echo "missing required command: rsync" >&2
    exit 1
fi

if [ ! -d "${SRC_DIR}" ]; then
    echo "missing source directory: ${SRC_DIR}" >&2
    exit 1
fi

if [ ! -d "${TARGET_REPO}" ]; then
    echo "missing target repository: ${TARGET_REPO}" >&2
    exit 1
fi

if [ ! -d "${TARGET_DIR}" ]; then
    echo "missing target directory: ${TARGET_DIR}" >&2
    exit 1
fi

if [ "${DRY_RUN}" -eq 1 ]; then
    echo "dry-run sync"
    echo "  from: ${SRC_DIR}/"
    echo "    to: ${TARGET_DIR}/"
    rsync -a --delete --exclude='.git' --dry-run "${SRC_DIR}/" "${TARGET_DIR}/"
    exit 0
fi

echo "syncing FPGA sources"
echo "  from: ${SRC_DIR}/"
echo "    to: ${TARGET_DIR}/"
rsync -a --delete --exclude='.git' "${SRC_DIR}/" "${TARGET_DIR}/"

echo "sync complete"
echo "next: build core from ${TARGET_REPO}"
echo "  export PATH=\"/home/rowe/intelFPGA_lite/17.0/quartus/bin:\$PATH\""
echo "  tools/mister-wrapper/build-core.sh --seed menu"
