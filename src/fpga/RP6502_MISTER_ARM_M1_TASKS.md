# M1 Tasks: MiSTer Frontend Boundary

## Objective
Reach a reliable launch/stop handshake between MiSTer OSD and an ARM wrapper process, without integrating real RP6502 runtime yet.

## FPGA/Core Tasks
- Add control/status register decode for interface v1.
- Expose `MAGIC`, `IF_VERSION`, `CTRL`, `STATUS`, `CMD_SEQ`, `ACK_SEQ`, `LAST_ERROR`, `HEARTBEAT`.
- Wire OSD action hooks:
  - `Launch ARM Runtime` -> set `REQ_LAUNCH`, bump `CMD_SEQ`.
  - `Stop ARM Runtime` -> set `REQ_STOP`, bump `CMD_SEQ`.
  - `Clear Error` -> set `REQ_CLEAR_ERROR`, bump `CMD_SEQ`.
- Keep behavior no-op safe if ARM is absent.

## ARM Wrapper Tasks
- Implement poll loop reading control registers and writing status registers.
- Implement placeholder process launch command (dummy binary/script).
- Implement stop command and timeout handling.
- Maintain `HEARTBEAT` and `ACK_SEQ` semantics.

## Validation Tasks
- Boot with no ARM process: OSD remains stable; status reports not present.
- Start wrapper manually: status flips to present/ready.
- Trigger launch from OSD: command acknowledged and running set.
- Trigger stop from OSD: running cleared and ack advanced.
- Inject launch failure: error latched and clearable from OSD.

## Acceptance Criteria
- No OSD corruption/flicker from control path activity.
- `CMD_SEQ`/`ACK_SEQ` are monotonic and never regress.
- 20 repeated launch/stop cycles pass without wedging.
- Recovery from failed launch requires no board power cycle.

## Deliverables
- RTL/control implementation commit.
- ARM wrapper stub commit.
- Test log with pass/fail table for acceptance criteria.
