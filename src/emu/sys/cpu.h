/*
 * Copyright (c) 2026 Rumbledethumps
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef _EMU_SYS_CPU_H_
#define _EMU_SYS_CPU_H_

#include <stdbool.h>
#include <stdint.h>
#include "emu/chips/w65c02.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* The firmware contract cpu.c implements: cpu_init, cpu_active,
 * cpu_set_phi2_khz_run (clamped to [CPU_PHI2_MIN_KHZ, CPU_PHI2_MAX_KHZ],
 * quantized), cpu_get_phi2_khz_run, and the CPU_RP2350_KHZ / CPU_PHI2_*
 * constants. Wrapped here so C++ consumers get C linkage. */
#include "sys/cpu.h"
extern bool halted;
#define cpu_active() (!halted)

/* The master clock unit is 1/8 of a 256 MHz tick (2048 per microsecond), so
 * the PHI2 fractional divider lands on an integer per-cycle step. */

/* Warm restart (exec): reset the 65C02 core, keeping the clock and PHI2. */
void cpu_reset(void);

#include "emu/sys/mem.h"
#include "emu/sys/via.h"
#include "emu/chips/rp6502.h"
#include "emu/dbg/dbg.h"

extern m6502_t cpu;
extern uint64_t pins;

static inline uint64_t bus_cycle(uint64_t p)
{
    uint16_t addr = M6502_GET_ADDR(p);
    if (addr >= VIA_WINDOW_LO && addr <= RIA_WINDOW_HI)
        return p;
    if (p & M6502_RW)
    {
        M6502_SET_DATA(p, ram[addr]);
        if (__builtin_expect(dbg_watch_armed, 0))
            dbg_watch_access(addr, ram[addr], false);
    }
    else
    {
        ram[addr] = M6502_GET_DATA(p);
        if (__builtin_expect(dbg_watch_armed, 0))
            dbg_watch_access(addr, ram[addr], true);
    }
    return p;
}

static inline uint64_t cpu_tick(void)
{
    pins = m6502_tick(&cpu, pins);
    pins = via_tick(pins);
    pins = ria_tick(pins);
    pins = bus_cycle(pins);
    return pins;
}

static inline uint64_t cpu_tick_fast(void)
{
    pins = m6502_tick(&cpu, pins);
    if (__builtin_expect(via_active, 0))
        pins = via_tick_full(pins);
    if (__builtin_expect(ria_irq_asserted_cached, 0))
        pins |= M6502_IRQ;
    uint16_t addr = (uint16_t)pins;
    if (__builtin_expect(addr >= VIA_WINDOW_LO && addr <= RIA_WINDOW_HI, 0))
    {
        if (addr >= 0xFFE0u)
            pins = ria_tick_full(pins);
    }
    else
    {
        if ((uint32_t)pins & (uint32_t)M6502_RW)
        {
            pins = (pins & 0xFFFFFFFFFF00FFFFull) | (((uint64_t)ram[addr]) << 16);
        }
        else
        {
            ram[addr] = (uint8_t)((uint32_t)pins >> 16);
        }
    }
    return pins;
}

uint32_t cpu_step_8(void); /* 1/8-tick units advanced per 6502 cycle */

/* True on an opcode fetch (SYNC); out-writes the fetch PC and SP. */
bool cpu_opcode_fetch(uint64_t pins, uint16_t *pc, uint8_t *sp);

/* Program-halt gate: the CPU stops ticking once halted (the EXIT syscall, a
 * failed exec, or a --dap launch hold set it; ria_reset clears it on restart).
 * cpu_active() — the firmware contract — is its inverse. */
bool cpu_halted(void);
void cpu_set_halted(bool halted);

/* The live 65C02 instance, for the debugger UI + DAP register access (the
 * debug code casts to m6502_t*, which includes the chip header, so this need
 * not pull it in). */
void *cpu_chip(void); /* m6502_t* */

/* Optional per-CPU-cycle observer for the debugger UI. Display-only and MUST
 * NOT gate the CPU — dbg.c is the one authoritative engine. NULL when no
 * observer is registered. */
extern void (*cpu_dbg_cycle_cb)(uint64_t pins);

#ifdef __cplusplus
}
#endif

#endif /* _EMU_SYS_CPU_H_ */
