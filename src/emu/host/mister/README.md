# rp6502-mister-wrapper (M1 stub)

This binary is a Linux-side M1 scaffold for the MiSTer ARM/FPGA control handshake.

It maps a small register window from a file (default `/tmp/rp6502_arm_if.bin`),
updates heartbeat/status fields, and responds to launch/stop requests using
`CMD_SEQ`/`ACK_SEQ` semantics from `src/fpga/RP6502_MISTER_ARM_INTERFACE_V1.md`.

Runtime launch payload slots are documented in
`src/fpga/RP6502_MISTER_ASSET_RUNTIME_PROTOCOL_V1.md`.

Architecture note:
- `wrapper_main.c` is orchestration and state machine.
- `transport.c` is the transport backend layer (file-backed M1 and `/dev/mem` path).
- This mirrors the 3s-mister-arm split between control wrapper logic and hardware-facing path.

## Build

From `src/emu`:

`cmake --build --preset debug --target rp6502-mister-wrapper`

Control helper:

`cmake --build --preset debug --target rp6502-mister-ctl`

### ARM cross-build (MiSTer Linux)

From repo root:

`src/emu/host/mister/build_wrapper_arm.sh`

Override compiler path if needed:

`CC=/path/to/arm-none-linux-gnueabihf-gcc src/emu/host/mister/build_wrapper_arm.sh`

## Run

`build/emulator/debug/rp6502-mister-wrapper --runtime /bin/sleep --runtime-arg 60`

### Transport selection

File-backed (default):

`build/emulator/debug/rp6502-mister-wrapper --transport file --reg-file /tmp/rp6502_arm_if.bin --runtime /bin/sleep --runtime-arg 60`

Devmem-backed (integration path):

`build/emulator/debug/rp6502-mister-wrapper --transport devmem --devmem /dev/mem --hps-base 0x<addr> --runtime /bin/sleep --runtime-arg 60`

## Control Helper

Print current register state:

`build/emulator/debug/rp6502-mister-ctl --transport file --reg-file /tmp/rp6502_arm_if.bin status`

Wait until wrapper advertises ready state:

`build/emulator/debug/rp6502-mister-ctl --transport file --reg-file /tmp/rp6502_arm_if.bin --timeout-ms 3000 wait-ready`

Launch via payload path/arg slots:

`build/emulator/debug/rp6502-mister-ctl --transport file --reg-file /tmp/rp6502_arm_if.bin --runtime /bin/sleep --runtime-arg 5 launch`

One-shot launch/stop smoke sequence:

`build/emulator/debug/rp6502-mister-ctl --transport file --reg-file /tmp/rp6502_arm_if.bin --runtime /bin/sleep --runtime-arg 2 smoke`

Stop/reset/clear:

`build/emulator/debug/rp6502-mister-ctl --transport file --reg-file /tmp/rp6502_arm_if.bin stop`

`build/emulator/debug/rp6502-mister-ctl --transport file --reg-file /tmp/rp6502_arm_if.bin reset`

`build/emulator/debug/rp6502-mister-ctl --transport file --reg-file /tmp/rp6502_arm_if.bin clear-error`

## Notes

- This is a scaffold only; it does not map real MiSTer HPS registers yet.
- M2/M3 will replace the file-backed register map with the real transport.
- Current intent is integration bring-up for launch/stop/error flows.
- Runtime source precedence:
	- `--runtime` CLI path (if provided) overrides payload launch path.
	- Otherwise wrapper reads `RUNTIME_PATH` / `RUNTIME_ARG` from the shared payload window.
