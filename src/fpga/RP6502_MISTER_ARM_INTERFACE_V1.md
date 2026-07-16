# RP6502 MiSTer ARM Interface v1 (M1 Scaffold)

## Scope
This document defines the minimal ARM<->FPGA boundary for M1 only:
- OSD-triggered launch/stop handshake
- Runtime state visibility
- Health/error reporting

Video/audio payload transport is intentionally deferred to M2/M3.

## Transport Model
- Control/status: lightweight register window over MiSTer HPS I/O path.
- Payload buffers: not part of M1.

## Register Map (v1)
Base: `RP6502_ARM_CTRL_BASE` (core-local decode)

- `0x00` `MAGIC` (RO)
  - Constant: `0x52503631` (`RP61`)
- `0x04` `IF_VERSION` (RO)
  - Constant: `0x00010000` (major.minor)
- `0x08` `CTRL` (RW)
  - Bit 0: `REQ_LAUNCH`
  - Bit 1: `REQ_STOP`
  - Bit 2: `REQ_RESET_RUNTIME`
  - Bit 3: `REQ_CLEAR_ERROR`
- `0x0C` `STATUS` (RO)
  - Bit 0: `ARM_PRESENT`
  - Bit 1: `RUNTIME_READY`
  - Bit 2: `RUNTIME_RUNNING`
  - Bit 3: `RUNTIME_ERROR`
  - Bit 4: `OSD_LOCKOUT` (optional)
- `0x10` `HEARTBEAT` (RO)
  - Monotonic counter incremented by ARM wrapper poll loop.
- `0x14` `LAST_ERROR` (RO)
  - `0`: none
  - `1`: launch_failed
  - `2`: runtime_crash
  - `3`: bad_payload_path
  - `4`: timeout_wait_ready
- `0x18` `CMD_SEQ` (RW)
  - Incremented by writer on each new request.
- `0x1C` `ACK_SEQ` (RO)
  - Mirrors last processed `CMD_SEQ`.

## Startup State Machine
1. FPGA reset: `STATUS=0`, `ACK_SEQ=0`.
2. ARM wrapper starts: sets `ARM_PRESENT=1`, `RUNTIME_READY=1`, `RUNTIME_RUNNING=0`.
3. OSD launch request:
  - FPGA/OSD writes `REQ_LAUNCH=1`, increments `CMD_SEQ`.
4. ARM observes new `CMD_SEQ`, clears `REQ_LAUNCH`, attempts launch.
5. On success:
  - `RUNTIME_RUNNING=1`, `RUNTIME_ERROR=0`, update `ACK_SEQ`.
6. OSD stop request:
  - set `REQ_STOP=1`, increment `CMD_SEQ`.
7. ARM stops runtime, sets `RUNTIME_RUNNING=0`, updates `ACK_SEQ`.

## Timing/Timeout Rules (M1)
- ARM poll interval target: 1 ms to 5 ms.
- Launch timeout: 3000 ms; on timeout set `LAST_ERROR=4`.
- Heartbeat stale threshold: 1000 ms equivalent.

## Error Handling Rules
- Any failed launch sets `RUNTIME_ERROR=1`, `LAST_ERROR!=0`.
- `REQ_CLEAR_ERROR` clears `LAST_ERROR` and `RUNTIME_ERROR` only when runtime is not running.
- `REQ_RESET_RUNTIME` is a hard stop + reinit of wrapper state.

## Out of Scope (Explicit)
- Framebuffer addresses and formats.
- Audio ring layout.
- `.rp6502` path/file transfer mechanism from OSD to ARM.
- Runtime asset I/O protocol.

These are handled in M2-M5 documents.
