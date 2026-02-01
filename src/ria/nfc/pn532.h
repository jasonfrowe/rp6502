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
bool pn532_read_tag(uint8_t *uid, uint8_t *len);

#endif /* _RIA_NFC_PN532_H_ */
