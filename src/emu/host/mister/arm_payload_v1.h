#pragma once

#include <stdint.h>

/*
 * Payload extension region for ARM IF v1 scaffolding.
 * Register bank remains at 0x00-0x1f; payload starts at 0x100.
 */
enum {
    RP6502_ARM_IF_WINDOW_SIZE = 0x400,
    RP6502_ARM_PAYLOAD_V1_OFF_RUNTIME_PATH = 0x100,
    RP6502_ARM_PAYLOAD_V1_MAX_RUNTIME_PATH = 256,
    RP6502_ARM_PAYLOAD_V1_OFF_RUNTIME_ARG = 0x200,
    RP6502_ARM_PAYLOAD_V1_MAX_RUNTIME_ARG = 256,
};
