#pragma once

#include <stdint.h>

enum {
    RP6502_ARM_IF_MAGIC = 0x52503631u,
    RP6502_ARM_IF_VERSION = 0x00010000u,
};

enum {
    RP6502_ARM_IF_OFF_MAGIC = 0x00,
    RP6502_ARM_IF_OFF_VERSION = 0x04,
    RP6502_ARM_IF_OFF_CTRL = 0x08,
    RP6502_ARM_IF_OFF_STATUS = 0x0C,
    RP6502_ARM_IF_OFF_HEARTBEAT = 0x10,
    RP6502_ARM_IF_OFF_LAST_ERROR = 0x14,
    RP6502_ARM_IF_OFF_CMD_SEQ = 0x18,
    RP6502_ARM_IF_OFF_ACK_SEQ = 0x1C,
    RP6502_ARM_IF_REG_SIZE = 0x20,
};

enum {
    RP6502_ARM_CTRL_REQ_LAUNCH = (1u << 0),
    RP6502_ARM_CTRL_REQ_STOP = (1u << 1),
    RP6502_ARM_CTRL_REQ_RESET_RUNTIME = (1u << 2),
    RP6502_ARM_CTRL_REQ_CLEAR_ERROR = (1u << 3),
};

enum {
    RP6502_ARM_STATUS_PRESENT = (1u << 0),
    RP6502_ARM_STATUS_READY = (1u << 1),
    RP6502_ARM_STATUS_RUNNING = (1u << 2),
    RP6502_ARM_STATUS_ERROR = (1u << 3),
    RP6502_ARM_STATUS_OSD_LOCKOUT = (1u << 4),
};

enum {
    RP6502_ARM_ERR_NONE = 0,
    RP6502_ARM_ERR_LAUNCH_FAILED = 1,
    RP6502_ARM_ERR_RUNTIME_CRASH = 2,
    RP6502_ARM_ERR_BAD_PAYLOAD_PATH = 3,
    RP6502_ARM_ERR_TIMEOUT_WAIT_READY = 4,
};

static inline uint32_t rp6502_arm_if_read(volatile uint32_t *regs, uint32_t offset) {
    return regs[offset >> 2];
}

static inline void rp6502_arm_if_write(volatile uint32_t *regs, uint32_t offset, uint32_t value) {
    regs[offset >> 2] = value;
}
