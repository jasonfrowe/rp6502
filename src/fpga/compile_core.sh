#!/usr/bin/env bash
# 3S MiSTer helper for this branch.
# Goal: ARM emulator on HPS + FPGA-native video core.
#
# This wrapper intentionally does NOT support the old rp6502 stage1..stage10
# targets. It delegates to the active 3s toolchain scripts.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
S3_ROOT="${S3_ROOT:-/home/rowe/Software/rp6502/3s-mister-arm}"

MISTER_HOST="${MISTER_HOST:-mister}"
MISTER_USER="${MISTER_USER:-root}"
MISTER_PASS="${MISTER_PASS:-}"
if [[ -z "$MISTER_PASS" && -n "${MISTER_PASSWORD:-}" ]]; then
    MISTER_PASS="$MISTER_PASSWORD"
fi
HPS_BINARY_OVERRIDE="${HPS_BINARY_OVERRIDE:-}"
CORE_RBF_OVERRIDE="${CORE_RBF_OVERRIDE:-}"
ARM_IF_WRAPPER_BIN_REMOTE="${ARM_IF_WRAPPER_BIN_REMOTE:-/media/fat/rp6502-mister-wrapper}"
ARM_IF_CTL_BIN_REMOTE="${ARM_IF_CTL_BIN_REMOTE:-/media/fat/rp6502-mister-ctl}"
ARM_IF_REG_FILE_REMOTE="${ARM_IF_REG_FILE_REMOTE:-/tmp/rp6502_arm_if.bin}"
ARM_IF_RUNTIME_PATH="${ARM_IF_RUNTIME_PATH:-/bin/sleep}"
ARM_IF_RUNTIME_ARG="${ARM_IF_RUNTIME_ARG:-2}"

DO_FPGA=1
DO_HPS=1
DO_DEPLOY=0
DEPLOY_MODE="artifacts-only"
DO_ARM_IF_SMOKE=0

usage() {
    cat <<'EOF'
Usage: ./compile_core.sh [options]

Scope:
  Build/deploy the active 3s MiSTer wrapper stack:
  - FPGA core via tools/mister-wrapper/build-core.sh
  - HPS wrapper via tools/mister-wrapper/build-hps.sh
  - Deploy via tools/mister/misterctl.sh deploy-wrapper

Options:
  --no-fpga        Skip FPGA core build
  --no-daemon      Skip HPS wrapper build (legacy compatibility name)
  --no-hps         Skip HPS wrapper build
  --deploy         Deploy wrapper artifacts to MiSTer
  --full-deploy    Deploy full wrapper package (not only core/HPS artifacts)
  --host <host>    MiSTer host (default: mister)
  --user <user>    MiSTer user (default: root)
    --password <p>   MiSTer password (or use MISTER_PASS / MISTER_PASSWORD env)
    --smoke-arm-if   Run rp6502-mister-wrapper/ctl wait-ready+smoke on MiSTer
  --help           Show this help

Env overrides:
    HPS_BINARY_OVERRIDE=<path>  Use specific MiSTer_3S-ARM binary for packaging
    CORE_RBF_OVERRIDE=<path>    Use specific 3S-ARM*.rbf for packaging
    ARM_IF_WRAPPER_BIN_REMOTE=<path>  Remote wrapper binary path
    ARM_IF_CTL_BIN_REMOTE=<path>      Remote control utility path
    ARM_IF_REG_FILE_REMOTE=<path>     Remote shared transport file path
    ARM_IF_RUNTIME_PATH=<path>        Runtime path used by smoke command
    ARM_IF_RUNTIME_ARG=<arg>          Runtime argument used by smoke command

Legacy notes:
  --stage* / --template / --legacy options are out of scope on this branch and
  are ignored with a warning.
EOF
}

warn_ignored_legacy_mode() {
    local mode="$1"
    echo "warning: ignoring unsupported legacy target option: ${mode}" >&2
}

run_ssh() {
    local cmd="$1"
    if [[ -n "$MISTER_PASS" ]]; then
        if ! command -v sshpass >/dev/null 2>&1; then
            echo "error: password auth requested (MISTER_PASS/MISTER_PASSWORD) but sshpass is not installed" >&2
            exit 1
        fi
        SSHPASS="$MISTER_PASS" sshpass -e ssh -x -o StrictHostKeyChecking=accept-new "${MISTER_USER}@${MISTER_HOST}" "$cmd"
    else
        ssh -x -o StrictHostKeyChecking=accept-new "${MISTER_USER}@${MISTER_HOST}" "$cmd"
    fi
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-fpga)
            DO_FPGA=0
            shift
            ;;
        --no-daemon|--no-hps)
            DO_HPS=0
            shift
            ;;
        --deploy)
            DO_DEPLOY=1
            shift
            ;;
        --full-deploy)
            DEPLOY_MODE="full"
            shift
            ;;
        --host)
            MISTER_HOST="$2"
            shift 2
            ;;
        --user)
            MISTER_USER="$2"
            shift 2
            ;;
        --password)
            MISTER_PASS="$2"
            shift 2
            ;;
        --smoke-arm-if)
            DO_ARM_IF_SMOKE=1
            shift
            ;;
        --template|--legacy|--stage2|--stage3|--stage4|--stage5|--stage6|--stage7|--stage8|--stage9|--stage10)
            warn_ignored_legacy_mode "$1"
            shift
            ;;
        --help|-h)
            usage
            exit 0
            ;;
        *)
            echo "unknown option: $1" >&2
            usage
            exit 2
            ;;
    esac
done

if [[ ! -d "$S3_ROOT" ]]; then
    echo "error: 3s root not found: $S3_ROOT" >&2
    exit 1
fi

BUILD_CORE_SH="$S3_ROOT/tools/mister-wrapper/build-core.sh"
BUILD_HPS_SH="$S3_ROOT/tools/mister-wrapper/build-hps.sh"
PACKAGE_WRAPPER_SH="$S3_ROOT/tools/mister-wrapper/package-wrapper.sh"
MISTERCTL_SH="$S3_ROOT/tools/mister/misterctl.sh"

for required in "$BUILD_CORE_SH" "$BUILD_HPS_SH" "$PACKAGE_WRAPPER_SH" "$MISTERCTL_SH"; do
    if [[ ! -x "$required" ]]; then
        echo "error: required executable not found: $required" >&2
        exit 1
    fi
done

if [[ "$DO_FPGA" -eq 1 ]]; then
    echo "== 3s FPGA core build"
    "$BUILD_CORE_SH"
else
    echo "== Skipping FPGA core build"
fi

if [[ "$DO_HPS" -eq 1 ]]; then
    echo "== 3s HPS wrapper build"
    "$BUILD_HPS_SH"
else
    echo "== Skipping HPS wrapper build"
fi

if [[ "$DO_DEPLOY" -eq 1 ]]; then
    echo "== Packaging wrapper artifacts"

    runtime_pkg="$S3_ROOT/build/mister-clean-package"
    if [[ ! -d "$runtime_pkg" ]]; then
        runtime_pkg="$S3_ROOT/build/mister-empty-runtime"
        mkdir -p "$runtime_pkg"
    fi

    hps_bin="$HPS_BINARY_OVERRIDE"
    if [[ -z "$hps_bin" ]]; then
        hps_bin="$S3_ROOT/build/mister-wrapper-hps/MiSTer_3S-ARM"
    fi

    core_rbf="$CORE_RBF_OVERRIDE"
    if [[ -z "$core_rbf" ]]; then
        core_rbf="$(ls -1t "$S3_ROOT"/build/mister-wrapper-core/3S-ARM_*.rbf 2>/dev/null | head -1 || true)"
    fi

    if [[ ! -f "$hps_bin" ]]; then
        if [[ "$DO_HPS" -eq 0 ]]; then
            echo "error: HPS artifact missing while --no-daemon/--no-hps is set: $hps_bin" >&2
            echo "hint: run without --no-daemon or set HPS_BINARY_OVERRIDE=<path>" >&2
        else
            echo "error: expected HPS artifact not found after build: $hps_bin" >&2
        fi
        exit 2
    fi

    if [[ -z "$core_rbf" || ! -f "$core_rbf" ]]; then
        if [[ "$DO_FPGA" -eq 0 ]]; then
            echo "error: core artifact missing while --no-fpga is set" >&2
            echo "hint: run without --no-fpga or set CORE_RBF_OVERRIDE=<path>" >&2
        else
            echo "error: expected core artifact not found after build (3S-ARM_YYYYMMDD.rbf)" >&2
        fi
        exit 2
    fi

    "$PACKAGE_WRAPPER_SH" \
        --runtime-package "$runtime_pkg" \
        --hps-binary "$hps_bin" \
        --core-rbf "$core_rbf"

    pkg_root="${OUTPUT_DIR:-$S3_ROOT/build/mister-wrapper-package}"
    if [[ ! -d "$pkg_root" ]]; then
        echo "error: packaged wrapper root not found: $pkg_root" >&2
        exit 1
    fi

    echo "== Deploying to ${MISTER_USER}@${MISTER_HOST}"
    deploy_args=(
        --host "$MISTER_HOST"
        --user "$MISTER_USER"
    )
    if [[ -n "$MISTER_PASS" ]]; then
        deploy_args+=(--password "$MISTER_PASS")
    fi
    deploy_args+=(deploy-wrapper --src "$pkg_root")
    if [[ "$DEPLOY_MODE" == "artifacts-only" ]]; then
        deploy_args+=(--artifacts-only)
    fi

    "$MISTERCTL_SH" "${deploy_args[@]}"

    if [[ "$DO_ARM_IF_SMOKE" -eq 1 ]]; then
        echo "== ARM IF smoke on ${MISTER_USER}@${MISTER_HOST}"

        run_ssh "killall rp6502-mister-wrapper >/dev/null 2>&1 || true"
        run_ssh "nohup '${ARM_IF_WRAPPER_BIN_REMOTE}' --transport file --reg-file '${ARM_IF_REG_FILE_REMOTE}' --poll-ms 1 >/tmp/rp6502-wrapper.log 2>&1 &"
        run_ssh "'${ARM_IF_CTL_BIN_REMOTE}' --transport file --reg-file '${ARM_IF_REG_FILE_REMOTE}' --timeout-ms 4000 wait-ready"
        run_ssh "'${ARM_IF_CTL_BIN_REMOTE}' --transport file --reg-file '${ARM_IF_REG_FILE_REMOTE}' --runtime '${ARM_IF_RUNTIME_PATH}' --runtime-arg '${ARM_IF_RUNTIME_ARG}' --timeout-ms 4000 smoke"
        run_ssh "killall rp6502-mister-wrapper >/dev/null 2>&1 || true"
    fi
fi

echo "Done."
