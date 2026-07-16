# RP6502 MiSTer ARM + FPGA Native Pipeline Plan

## Goal
Run the RP6502 emulator/runtime on MiSTer Linux ARM while using MiSTer FPGA for native video/audio output and OSD-driven launch flow.

## Non-Goals
- Do not recreate full RP6502 syscall/runtime behavior in FPGA RTL.
- Do not depend on an always-on external daemon process.
- Do not treat .rp6502 as a flat ROM image.

## Architecture Sketch
- ARM side
  - Owns emulator runtime lifecycle.
  - Parses and loads .rp6502 program chunks exactly as emulator reference does.
  - Serves ROM:name runtime assets on demand from .rp6502 container.
  - Produces frame/audio buffers and timing feedback.
- FPGA side
  - Keeps MiSTer OSD and file selection entrypoints.
  - Provides native video output path from shared DDR/framebuffer region.
  - Provides audio output bridge from ARM-produced samples.
  - Exposes lightweight control/status registers (reset, run, frame counters, health).
- ARM<->FPGA contract
  - Versioned shared-memory/control interface.
  - Explicit startup/ready/running/error state machine.
  - Vsync and audio-consumer feedback counters for pacing.

## Milestones

### M0 - Branch + Baseline Freeze
Deliverables
- New attempt branch from main.
- Baseline note: current stage10 standalone loader retained as fallback artifact.
- Plan document committed.
Success
- Fresh branch exists and is the only target for new work.

### M1 - MiSTer Frontend Boundary (No Runtime Yet)
Deliverables
- OSD menu entries for RP6502 ARM launch and stop.
- Minimal control/status registers in FPGA for wrapper handshake.
- Wrapper stub process that can launch/stop a placeholder ARM app.
Success
- OSD can start/stop ARM app reliably with no OSD corruption.

### M2 - Native Video Path Wiring
Deliverables
- ARM writes test pattern frames into agreed memory layout.
- FPGA native video reader presents those frames at stable timing.
- Vsync feedback counter visible to ARM wrapper.
Success
- Stable image for 30+ minutes on target output path.
- No periodic pacing hitch beyond defined tolerance.

### M3 - Audio Pipeline Wiring
Deliverables
- ARM emits PCM through agreed buffer/ring contract.
- FPGA/MiSTer audio path consumes without underflow storms.
- Drift-control loop (rate/phase correction) integrated.
Success
- Continuous playback for 30+ minutes with no audible pops/dropouts.

### M4 - RP6502 Runtime Integration on ARM
Deliverables
- ARM wrapper launches real RP6502 emulator runtime.
- .rp6502 loader semantics match reference behavior.
- Runtime ROM:name asset reads routed through container scanner.
Success
- Known sample .rp6502 files boot and run with expected behavior.

### M5 - End-to-End OSD File Select + Launch
Deliverables
- OSD file selector chooses .rp6502 payload.
- Selected path handed to ARM runtime through wrapper contract.
- Clean restart/replace flow for consecutive launches.
Success
- User can pick and run different .rp6502 files repeatedly without reboot.

### M6 - Hardening + Failure Paths
Deliverables
- Defined error states: invalid header, chunk CRC fail, missing reset vector, missing assets.
- UI-safe failure reporting and recovery actions.
- Watchdog/restart policy for ARM runtime crash/hang.
Success
- Failure matrix passes without wedging OSD or requiring power cycle.

### M7 - Practical Test Bench
Deliverables
- Corpus of .rp6502 cases: valid, CRC-fail, missing-vector, asset-heavy.
- Host-side oracle checks against emulator reference behavior.
- On-device smoke script for launch, frame pacing, and audio continuity.
Success
- Repeatable pass/fail report generated for each candidate build.

## Suggested Work Order
1. Finish M1 before touching detailed video timing internals.
2. Land M2 and M3 independently with synthetic workloads first.
3. Integrate real runtime only after feedback loops are stable.
4. Gate merges on M7 smoke + oracle checks.

## Exit Criteria
- OSD-selectable .rp6502 launch works reliably.
- Runtime assets work (ROM:name semantics preserved).
- Native video/audio are stable and measurable.
- No always-on daemon requirement for normal usage.
