# FPGA RTL Scaffolds

This directory holds standalone RTL blocks for the RP6502 MiSTer ARM integration plan.

## `rp6502_arm_if_regs_v1.sv`

Minimal register bank for the v1 control/status contract documented in:
- `src/fpga/RP6502_MISTER_ARM_INTERFACE_V1.md`

Current purpose:
- Provide a clean integration unit for M1 before selecting the final MiSTer top-level insertion point.
- Keep address/field semantics stable while bus wiring evolves.

Notes:
- `MAGIC` and `IF_VERSION` are read-only constants.
- Other registers are writable/readable for early bring-up via HPS-side access.
- Bus is a simple single-cycle local interface placeholder (`bus_addr`, `bus_wr`, `bus_rd`, `bus_wdata`, `bus_rdata`).
