/*
 * Copyright (c) 2025 Rumbledethumps
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _RIA_NFC_NFC_H_
#define _RIA_NFC_NFC_H_

#include <stdbool.h>
#include <stdint.h>

void nfc_init(void);
void nfc_task(void);

// Called by USB driver when CDC/Serial device is mounted
void nfc_cdc_mounted(uint8_t dev_addr, uint16_t vid, uint16_t pid);
void nfc_cdc_unmounted(uint8_t dev_addr);

#endif /* _RIA_NFC_NFC_H_ */
