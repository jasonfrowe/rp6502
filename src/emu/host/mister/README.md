# rp6502-mister-wrapper (M1 stub)

This binary is a Linux-side M1 scaffold for the MiSTer ARM/FPGA control handshake.

It maps a small register window from a file (default `/tmp/rp6502_arm_if.bin`),
updates heartbeat/status fields, and responds to launch/stop requests using
`CMD_SEQ`/`ACK_SEQ` semantics from `src/fpga/RP6502_MISTER_ARM_INTERFACE_V1.md`.

## Build

From `src/emu`:

`cmake --build --preset debug --target rp6502-mister-wrapper`

## Run

`build/emulator/debug/rp6502-mister-wrapper --runtime /bin/sleep --runtime-arg 60`

## Notes

- This is a scaffold only; it does not map real MiSTer HPS registers yet.
- M2/M3 will replace the file-backed register map with the real transport.
- Current intent is integration bring-up for launch/stop/error flows.
