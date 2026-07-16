#!/usr/bin/env bash
# Example MiSTer build helper.
# Builds:
# 1) FPGA core (Quartus -> .sof/.rbf)
# 2) HPS daemon (src/emu/mister/rp6502-mister)
# 3) Optional deploy to MiSTer over SCP.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

CORE_MODE="template"
CORE_NAME="rp6502_template_baseline"
CORE_PROJECT_DIR="$SCRIPT_DIR/template_control"
CORE_REVISION="Template"
SOF_FILE="$CORE_PROJECT_DIR/output_files/${CORE_REVISION}.sof"
RBF_FILE="$CORE_PROJECT_DIR/output_files/${CORE_REVISION}.rbf"
RELEASES_DIR="$SCRIPT_DIR/releases"
DATE_TAG="$(date +%Y%m%d)"
RELEASE_RBF="$RELEASES_DIR/${CORE_NAME}_${DATE_TAG}.rbf"

MISTER_HOST="${MISTER_HOST:-mister.home.arpa}"
MISTER_USER="${MISTER_USER:-root}"
MISTER_PASS="${MISTER_PASS:-}"
MISTER_SCP_OPTS="${MISTER_SCP_OPTS:--o StrictHostKeyChecking=accept-new}"
MISTER_GAMES_DIR="${MISTER_GAMES_DIR:-/media/fat/games/RP6502}"
MISTER_CORE_DIR="${MISTER_CORE_DIR:-/media/fat/_Computer}"
MISTER_DAEMON_DIR="${MISTER_DAEMON_DIR:-/media/fat}"
MISTER_CORE_FILE="${MISTER_CORE_FILE:-${CORE_NAME}.rbf}"
MISTER_ROM_PATH="${MISTER_ROM_PATH:-$REPO_ROOT/ROMS/LodeRunner.rp6502}"

DO_DEPLOY=0
DO_DAEMON=1
DO_FPGA=1

usage() {
    cat <<'EOF'
Usage: ./compile_core.sh [options]

Options:
    --template       Build/deploy template_control core (default)
    --stage2         Build/deploy template_rp6502_stage2 core
    --stage3         Build/deploy template_rp6502_stage3 core (CPU + RAM)
    --stage4         Build/deploy template_rp6502_stage4 core (CPU + RAM + RIA)
    --stage5         Build/deploy template_rp6502_stage5 core (Stage4 + host ctrl regs)
    --stage6         Build/deploy template_rp6502_stage6 core (Stage5 + ctrl write-through)
    --stage7         Build/deploy template_rp6502_stage7 core (Stage6 + ctrl read strobes)
    --stage8         Build/deploy template_rp6502_stage8 core (Stage7 + ctrl reads from RIA data)
    --stage9         Build/deploy template_rp6502_stage9 core (Stage8 + RAM/XRAM/stack host pass-through)
    --stage10        Build/deploy template_rp6502_stage10 core (standalone ROM loader, no daemon)
    --legacy         Build/deploy legacy rp6502_mister project
  --no-fpga        Skip Quartus core build
    --daemon         Force HPS daemon build (default: on)
  --no-daemon      Skip HPS daemon build (src/emu/mister)
  --deploy         Copy built artifacts to MiSTer via SCP
  --host <host>    MiSTer host (default: mister.home.arpa)
  --user <user>    MiSTer user (default: root)
  --help           Show this help

Notes:
  - For non-interactive deploy, export MISTER_PASS and install sshpass.
    - Optional: export MISTER_SCP_OPTS to override scp options.
    - Optional: export MISTER_CORE_FILE to rename deployed .rbf on MiSTer.
    - Optional: export MISTER_ROM_PATH to pick a ROM file. Set empty to disable ROM upload.
  - If MISTER_PASS is unset, normal scp/ssh auth is used.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --template)
            CORE_MODE="template"
            CORE_NAME="rp6502_template_baseline"
            CORE_PROJECT_DIR="$SCRIPT_DIR/template_control"
            CORE_REVISION="Template"
            SOF_FILE="$CORE_PROJECT_DIR/output_files/${CORE_REVISION}.sof"
            RBF_FILE="$CORE_PROJECT_DIR/output_files/${CORE_REVISION}.rbf"
            RELEASE_RBF="$RELEASES_DIR/${CORE_NAME}_${DATE_TAG}.rbf"
            MISTER_CORE_FILE="${MISTER_CORE_FILE:-${CORE_NAME}.rbf}"
            shift
            ;;
        --stage2)
            CORE_MODE="stage2"
            CORE_NAME="rp6502_stage2_shell"
            CORE_PROJECT_DIR="$SCRIPT_DIR/template_rp6502_stage2"
            CORE_REVISION="Template"
            SOF_FILE="$CORE_PROJECT_DIR/output_files/${CORE_REVISION}.sof"
            RBF_FILE="$CORE_PROJECT_DIR/output_files/${CORE_REVISION}.rbf"
            RELEASE_RBF="$RELEASES_DIR/${CORE_NAME}_${DATE_TAG}.rbf"
            shift
            ;;
        --stage3)
            CORE_MODE="stage3"
            CORE_NAME="rp6502_stage3_cpu_ram"
            CORE_PROJECT_DIR="$SCRIPT_DIR/template_rp6502_stage3"
            CORE_REVISION="Template"
            SOF_FILE="$CORE_PROJECT_DIR/output_files/${CORE_REVISION}.sof"
            RBF_FILE="$CORE_PROJECT_DIR/output_files/${CORE_REVISION}.rbf"
            RELEASE_RBF="$RELEASES_DIR/${CORE_NAME}_${DATE_TAG}.rbf"
            shift
            ;;
        --stage4)
            CORE_MODE="stage4"
            CORE_NAME="rp6502_stage4_ria"
            CORE_PROJECT_DIR="$SCRIPT_DIR/template_rp6502_stage4"
            CORE_REVISION="Template"
            SOF_FILE="$CORE_PROJECT_DIR/output_files/${CORE_REVISION}.sof"
            RBF_FILE="$CORE_PROJECT_DIR/output_files/${CORE_REVISION}.rbf"
            RELEASE_RBF="$RELEASES_DIR/${CORE_NAME}_${DATE_TAG}.rbf"
            shift
            ;;
        --stage5)
            CORE_MODE="stage5"
            CORE_NAME="rp6502_stage5_ctrl"
            CORE_PROJECT_DIR="$SCRIPT_DIR/template_rp6502_stage5"
            CORE_REVISION="Template"
            SOF_FILE="$CORE_PROJECT_DIR/output_files/${CORE_REVISION}.sof"
            RBF_FILE="$CORE_PROJECT_DIR/output_files/${CORE_REVISION}.rbf"
            RELEASE_RBF="$RELEASES_DIR/${CORE_NAME}_${DATE_TAG}.rbf"
            shift
            ;;
        --stage6)
            CORE_MODE="stage6"
            CORE_NAME="rp6502_stage6_ctrl_ria"
            CORE_PROJECT_DIR="$SCRIPT_DIR/template_rp6502_stage6"
            CORE_REVISION="Template"
            SOF_FILE="$CORE_PROJECT_DIR/output_files/${CORE_REVISION}.sof"
            RBF_FILE="$CORE_PROJECT_DIR/output_files/${CORE_REVISION}.rbf"
            RELEASE_RBF="$RELEASES_DIR/${CORE_NAME}_${DATE_TAG}.rbf"
            shift
            ;;
        --stage7)
            CORE_MODE="stage7"
            CORE_NAME="rp6502_stage7_ctrl_ria_rd"
            CORE_PROJECT_DIR="$SCRIPT_DIR/template_rp6502_stage7"
            CORE_REVISION="Template"
            SOF_FILE="$CORE_PROJECT_DIR/output_files/${CORE_REVISION}.sof"
            RBF_FILE="$CORE_PROJECT_DIR/output_files/${CORE_REVISION}.rbf"
            RELEASE_RBF="$RELEASES_DIR/${CORE_NAME}_${DATE_TAG}.rbf"
            shift
            ;;
        --stage8)
            CORE_MODE="stage8"
            CORE_NAME="rp6502_stage8_ctrl_ria_data"
            CORE_PROJECT_DIR="$SCRIPT_DIR/template_rp6502_stage8"
            CORE_REVISION="Template"
            SOF_FILE="$CORE_PROJECT_DIR/output_files/${CORE_REVISION}.sof"
            RBF_FILE="$CORE_PROJECT_DIR/output_files/${CORE_REVISION}.rbf"
            RELEASE_RBF="$RELEASES_DIR/${CORE_NAME}_${DATE_TAG}.rbf"
            shift
            ;;
        --stage9)
            CORE_MODE="stage9"
            CORE_NAME="rp6502_stage9_host_passthrough"
            CORE_PROJECT_DIR="$SCRIPT_DIR/template_rp6502_stage9"
            CORE_REVISION="Template"
            SOF_FILE="$CORE_PROJECT_DIR/output_files/${CORE_REVISION}.sof"
            RBF_FILE="$CORE_PROJECT_DIR/output_files/${CORE_REVISION}.rbf"
            RELEASE_RBF="$RELEASES_DIR/${CORE_NAME}_${DATE_TAG}.rbf"
            shift
            ;;
        --stage10)
            CORE_MODE="stage10"
            CORE_NAME="rp6502_stage10_standalone"
            CORE_PROJECT_DIR="$SCRIPT_DIR/template_rp6502_stage10"
            CORE_REVISION="Template"
            SOF_FILE="$CORE_PROJECT_DIR/output_files/${CORE_REVISION}.sof"
            RBF_FILE="$CORE_PROJECT_DIR/output_files/${CORE_REVISION}.rbf"
            RELEASE_RBF="$RELEASES_DIR/${CORE_NAME}_${DATE_TAG}.rbf"
            shift
            ;;
        --legacy)
            CORE_MODE="legacy"
            CORE_NAME="rp6502_mister"
            CORE_PROJECT_DIR="$SCRIPT_DIR"
            CORE_REVISION="rp6502_mister"
            SOF_FILE="$CORE_PROJECT_DIR/output_files/${CORE_REVISION}.sof"
            RBF_FILE="$CORE_PROJECT_DIR/${CORE_REVISION}.rbf"
            RELEASE_RBF="$RELEASES_DIR/${CORE_NAME}_${DATE_TAG}.rbf"
            MISTER_CORE_FILE="${MISTER_CORE_FILE:-${CORE_NAME}.rbf}"
            shift
            ;;
        --no-fpga)
            DO_FPGA=0
            shift
            ;;
        --no-daemon)
            DO_DAEMON=0
            shift
            ;;
        --daemon)
            DO_DAEMON=1
            shift
            ;;
        --deploy)
            DO_DEPLOY=1
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
        --help|-h)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1" >&2
            usage
            exit 2
            ;;
    esac
done

find_quartus_tool() {
    local tool="$1"
    if command -v "$tool" >/dev/null 2>&1; then
        command -v "$tool"
        return 0
    fi
    local fallback="$HOME/intelFPGA_lite/17.0/quartus/bin/$tool"
    if [[ -x "$fallback" ]]; then
        echo "$fallback"
        return 0
    fi
    return 1
}

copy_file() {
    local src="$1"
    local dst="$2"
    if [[ -n "$MISTER_PASS" ]]; then
        if ! command -v sshpass >/dev/null 2>&1; then
            echo "MISTER_PASS is set but sshpass is not installed." >&2
            exit 1
        fi
        sshpass -p "$MISTER_PASS" scp ${MISTER_SCP_OPTS} "$src" "$dst"
    else
        scp ${MISTER_SCP_OPTS} "$src" "$dst"
    fi
}

run_ssh() {
    local cmd="$1"
    local target="${MISTER_USER}@${MISTER_HOST}"
    if [[ -n "$MISTER_PASS" ]]; then
        if ! command -v sshpass >/dev/null 2>&1; then
            echo "MISTER_PASS is set but sshpass is not installed." >&2
            exit 1
        fi
        sshpass -p "$MISTER_PASS" ssh -o StrictHostKeyChecking=accept-new "$target" "$cmd"
    else
        ssh -o StrictHostKeyChecking=accept-new "$target" "$cmd"
    fi
}

local_sha256() {
    local path="$1"
    sha256sum "$path" | awk '{print $1}'
}

remote_sha256() {
    local path="$1"
    local out
    out="$(run_ssh "command -v sha256sum >/dev/null 2>&1 || { echo 'sha256sum not found on MiSTer'; exit 127; }; sha256sum '$path'")"
    printf '%s\n' "${out%% *}"
}

mkdir -p "$RELEASES_DIR"

if [[ "$DO_DEPLOY" -eq 1 && "$MISTER_PASS" == "YOUR_PASSWORD" ]]; then
    echo "MISTER_PASS is still the placeholder value 'YOUR_PASSWORD'. Set your real MiSTer password first." >&2
    exit 2
fi

if [[ "$DO_FPGA" -eq 1 ]]; then
    QUARTUS_SH="$(find_quartus_tool quartus_sh || true)"
    QUARTUS_CPF="$(find_quartus_tool quartus_cpf || true)"
    if [[ -z "$QUARTUS_SH" ]]; then
        echo "Quartus quartus_sh not found. Install Quartus 17.0.x or add quartus_sh to PATH." >&2
        exit 1
    fi

    echo "== Quartus compile: $CORE_MODE ($CORE_REVISION)"
    "$QUARTUS_SH" --flow compile "$CORE_PROJECT_DIR/$CORE_REVISION"

    if [[ ! -f "$SOF_FILE" ]]; then
        echo "Expected SOF not found: $SOF_FILE" >&2
        exit 1
    fi

    if [[ ! -f "$RBF_FILE" ]]; then
        if [[ -z "$QUARTUS_CPF" ]]; then
            echo "quartus_cpf not found and no RBF produced by compile: $RBF_FILE" >&2
            exit 1
        fi
        echo "== Generating RBF"
        "$QUARTUS_CPF" -c -q 50.0MHz -g 1.8V -n p "$SOF_FILE" "$RBF_FILE"
    fi

    cp "$RBF_FILE" "$RELEASE_RBF"
    echo "Generated: $RBF_FILE"
    echo "Release copy: $RELEASE_RBF"
fi

if [[ "$DO_DAEMON" -eq 1 ]]; then
    echo "== Building HPS daemon"
    make -C "$REPO_ROOT/src/emu/mister" clean
    make -C "$REPO_ROOT/src/emu/mister"
else
    echo "== Skipping HPS daemon build"
fi

if [[ "$DO_DEPLOY" -eq 1 ]]; then
    echo "== Deploying to ${MISTER_USER}@${MISTER_HOST}"

    if [[ -n "$MISTER_ROM_PATH" ]]; then
        if [[ -f "$MISTER_ROM_PATH" ]]; then
            copy_file "$MISTER_ROM_PATH" "${MISTER_USER}@${MISTER_HOST}:${MISTER_GAMES_DIR}/"
        else
            echo "Skipping ROM upload: local file not found: $MISTER_ROM_PATH"
        fi
    else
        echo "Skipping ROM upload: MISTER_ROM_PATH is empty"
    fi

    if [[ "$DO_DAEMON" -eq 1 ]]; then
        copy_file "$REPO_ROOT/src/emu/mister/rp6502-mister" "${MISTER_USER}@${MISTER_HOST}:${MISTER_DAEMON_DIR}/"

        if run_ssh "grep -a 'LW bridge access is disabled by default' '${MISTER_DAEMON_DIR}/rp6502-mister' >/dev/null"; then
            echo "WARNING: remote daemon still contains legacy LW bridge gate string." >&2
        else
            echo "Remote daemon check: legacy LW bridge gate string not present."
        fi
    fi

    if [[ -f "$RBF_FILE" ]]; then
        local_rbf_sha="$(local_sha256 "$RBF_FILE")"
        echo "Local RBF SHA256: $local_rbf_sha"
        echo "Deploying core as: ${MISTER_CORE_DIR}/${MISTER_CORE_FILE}"

        copy_file "$RBF_FILE" "${MISTER_USER}@${MISTER_HOST}:${MISTER_CORE_DIR}/${MISTER_CORE_FILE}"

        remote_rbf_path="${MISTER_CORE_DIR}/${MISTER_CORE_FILE}"
        remote_rbf_sha="$(remote_sha256 "$remote_rbf_path")"
        echo "Remote RBF SHA256: $remote_rbf_sha"

        if [[ "$local_rbf_sha" != "$remote_rbf_sha" ]]; then
            echo "ERROR: remote RBF hash mismatch after deploy: $remote_rbf_path" >&2
            exit 3
        fi

        echo "Remote core check: RBF hash matches."
    else
        echo "Skipping core deploy: local RBF not found: $RBF_FILE"
    fi

    echo "Deploy complete."
fi

echo "Done."
