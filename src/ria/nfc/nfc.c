/*
 * Copyright (c) 2025 Rumbledethumps
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "nfc/nfc.h"
#include "nfc/ch340.h"
#include "nfc/pn532.h"
#include <stdio.h>

static bool nfc_cdc_connected = false;
static uint8_t nfc_dev_addr = 0;

void nfc_init(void) {
  ch340_init();
  pn532_init(0); // Addr 0 is invalid, waits for connect
}

void nfc_task(void) {
  ch340_task();
  if (nfc_cdc_connected) {
    pn532_task();
  }
}

void nfc_cdc_mounted(uint8_t dev_addr, uint16_t vid, uint16_t pid) {
  (void)vid;
  (void)pid;
  printf("NFC: CDC/Serial driver ready (Addr %d)\n", dev_addr);
  nfc_cdc_connected = true;
  nfc_dev_addr = dev_addr;

  // Start PN532 initialization
  pn532_init(dev_addr);
}

void nfc_cdc_unmounted(uint8_t dev_addr) {
  if (nfc_dev_addr == dev_addr) {
    nfc_cdc_connected = false;
    nfc_dev_addr = 0;
    printf("NFC: CDC/Serial device removed\n");
  }
}
