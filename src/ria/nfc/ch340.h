/*
 * Copyright (c) 2025 Rumbledethumps
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _RIA_NFC_CH340_H_
#define _RIA_NFC_CH340_H_

/* CH340 USB-to-serial chip driver
 * Implements vendor-specific USB protocol for CH340/CH341 chips
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// CH340 USB control request codes
#define CH340_REQ_READ_VERSION        0x5F
#define CH340_REQ_WRITE_REG           0x9A
#define CH340_REQ_READ_REG            0x95
#define CH340_REQ_SERIAL_INIT         0xA1
#define CH340_REQ_MODEM_CTRL          0xA4

// Reset CH340 driver state
void ch340_init(void);

// Task function - call repeatedly to manage driver state
void ch340_task(void);

// Send data over serial
bool ch340_write(uint8_t dev_addr, const uint8_t *data, size_t len);

// Receive data from serial (non-blocking)
int ch340_read(uint8_t dev_addr, uint8_t *buffer, size_t max_len);

// Check if device is ready
bool ch340_is_ready(uint8_t dev_addr);

// Manual mount callback (called from usb.c)
void ch340_mount(uint8_t dev_addr, uint16_t vid, uint16_t pid);
void ch340_umount(uint8_t dev_addr);
int ch340_count(void);

#endif /* _RIA_NFC_CH340_H_ */
