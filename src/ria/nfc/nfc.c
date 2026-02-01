/*
 * Copyright (c) 2025 Rumbledethumps
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "nfc/nfc.h"
#include "mon/mon.h"
#include "nfc/ch340.h"
#include "nfc/pn532.h"
#include "sys/lfs.h"
#include <pico/stdlib.h>
#include <stdio.h>
#include <string.h>

static bool nfc_cdc_connected = false;
static uint8_t nfc_dev_addr = 0;

// Debouncing
static uint8_t nfc_last_uid[10];
static uint8_t nfc_last_uid_len = 0;
static absolute_time_t nfc_last_time;

void nfc_init(void) {
  ch340_init();
  pn532_init(0); // Addr 0 is invalid, waits for connect
}

static void nfc_check_config(uint8_t *uid, uint8_t uid_len) {
  lfs_file_t lfs_file;
  LFS_FILE_CONFIG(lfs_file_config);

  // Config format: UID (Hex) = Command
  // e.g. 04A1B2C3 = LOAD GAME

  int err = lfs_file_opencfg(&lfs_volume, &lfs_file, "NFC.TXT", LFS_O_RDONLY,
                             &lfs_file_config);
  if (err < 0) {
    printf("NFC: No NFC.TXT found.\n");
    return;
  }

  char line[256];
  char uid_str[32];

  // Convert UID to hex string
  char *p = uid_str;
  for (int i = 0; i < uid_len; i++) {
    p += sprintf(p, "%02X", uid[i]);
  }

  bool executed = false;
  while (lfs_gets(line, sizeof(line), &lfs_volume, &lfs_file)) {
    // Simple parser
    // 1. Skip leading whitespace
    char *ptr = line;
    while (*ptr == ' ' || *ptr == '\t')
      ptr++;

    // 2. Check comment or empty
    if (*ptr == '#' || *ptr == ';' || *ptr == 0 || *ptr == '\r' || *ptr == '\n')
      continue;

    // 3. Check start matches UID
    if (strncasecmp(ptr, uid_str, strlen(uid_str)) == 0) {
      // Match!
      ptr += strlen(uid_str);
      // Skip space
      while (*ptr == ' ' || *ptr == '\t')
        ptr++;

      // Check separator '=' (optional but good practice)
      if (*ptr == '=') {
        ptr++;
        while (*ptr == ' ' || *ptr == '\t')
          ptr++;
      }

      // Strip newline
      char *end = ptr + strlen(ptr) - 1;
      while (end > ptr && (*end == '\r' || *end == '\n')) {
        *end = 0;
        end--;
      }

      if (strlen(ptr) > 0) {
        printf("NFC: Executing '%s'\n", ptr);
        mon_run_command(ptr);
        executed = true;
        break;
      }
    }
  }

  lfs_file_close(&lfs_volume, &lfs_file);

  if (!executed) {
    printf("NFC: Tag %s not found in NFC.TXT\n", uid_str);
  }
}

void nfc_task(void) {
  ch340_task();
  if (nfc_cdc_connected) {
    pn532_task();

    // Check for new tags
    uint8_t uid[10];
    uint8_t uid_len;
    if (pn532_read_tag(uid, &uid_len)) {
      absolute_time_t now = get_absolute_time();

      // Debounce: Same tag within 2 seconds?
      bool same_tag = (uid_len == nfc_last_uid_len) &&
                      (memcmp(uid, nfc_last_uid, uid_len) == 0);
      if (!same_tag || absolute_time_diff_us(nfc_last_time, now) > 2000000) {
        // New action
        memcpy(nfc_last_uid, uid, uid_len);
        nfc_last_uid_len = uid_len;
        nfc_last_time = now;

        nfc_check_config(uid, uid_len);
      }
    }
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
