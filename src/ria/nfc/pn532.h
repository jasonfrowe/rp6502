/*
 * Copyright (c) 2025 Rumbledethumps
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _RIA_NFC_PN532_H_
#define _RIA_NFC_PN532_H_

#include <stdbool.h>
#include <stdint.h>

bool pn532_init(uint8_t dev_addr);
bool pn532_task(void);

#endif /* _RIA_NFC_PN532_H_ */
