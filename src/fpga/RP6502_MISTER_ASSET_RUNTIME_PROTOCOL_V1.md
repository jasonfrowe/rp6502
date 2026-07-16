# RP6502 MiSTer Asset/Runtime Protocol v1

## Scope
This document defines the first runtime launch payload contract between FPGA-side control writers and the ARM wrapper.

It extends the ARM IF v1 memory window with payload slots while keeping the existing control/status registers unchanged.

## Window Layout
Total shared window size: `0x400` bytes.

- `0x000-0x01f`: ARM IF v1 control/status registers (see `RP6502_MISTER_ARM_INTERFACE_V1.md`)
- `0x020-0x0ff`: reserved
- `0x100-0x1ff`: `RUNTIME_PATH` UTF-8/ASCII C-string, max 256 bytes including NUL
- `0x200-0x2ff`: `RUNTIME_ARG` UTF-8/ASCII C-string, max 256 bytes including NUL
- `0x300-0x3ff`: reserved for future payload fields

## Launch Rules
1. Writer populates `RUNTIME_PATH` (and optional `RUNTIME_ARG`) before issuing launch.
2. Writer sets `CTRL.REQ_LAUNCH=1` and increments `CMD_SEQ`.
3. ARM wrapper processes new command and updates `ACK_SEQ`.
4. On success, wrapper sets `STATUS.RUNTIME_RUNNING=1`.
5. On payload validation failure, wrapper sets `LAST_ERROR=bad_payload_path` and `STATUS.RUNTIME_ERROR=1`.

## Precedence Rules
- If wrapper process was started with `--runtime`, CLI runtime path overrides payload.
- If `--runtime` is not provided, wrapper reads `RUNTIME_PATH` from payload.
- `RUNTIME_ARG` is optional. Empty string means no argument.

## Validation Rules
- Payload strings must be NUL-terminated within their field size.
- `RUNTIME_PATH` must be non-empty and absolute (`/path/...`).
- Wrapper accepts printable ASCII only in v1 payload strings.

## Compatibility Notes
- Existing M1 register semantics (`CTRL`, `STATUS`, `CMD_SEQ`, `ACK_SEQ`, `HEARTBEAT`) are unchanged.
- FPGA-only M1 scaffolds that do not provide payload fields can continue to use wrapper CLI `--runtime`.
