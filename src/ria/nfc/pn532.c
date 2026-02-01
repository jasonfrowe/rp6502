/*
 * Copyright (c) 2025 Rumbledethumps
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "nfc/pn532.h"
#include "nfc/ch340.h"
#include <pico/stdlib.h>
#include <stdio.h>
#include <string.h>

// PN532 Commands
#define PN532_COMMAND_GETFIRMWAREVERSION 0x02
#define PN532_COMMAND_GETFIRMWAREVERSION 0x02
#define PN532_COMMAND_SAMCONFIGURATION 0x14
#define PN532_COMMAND_RFCONFIGURATION 0x32
#define PN532_COMMAND_INLISTPASSIVETARGET 0x4A

// Frame Consurction
#define PN532_PREAMBLE 0x00
#define PN532_STARTCODE1 0x00
#define PN532_STARTCODE2 0xFF
#define PN532_POSTAMBLE 0x00
#define PN532_HOSTTOPN532 0xD4
#define PN532_PN532TOHOST 0xD5

typedef enum {
  PN532_INIT_IDLE = 0,
  PN532_INIT_WAKEUP,

  PN532_INIT_WAIT_ACK,
  PN532_INIT_WAIT_RESPONSE,
  PN532_INIT_RF_CONFIG,
  PN532_INIT_RF_WAIT_ACK,
  PN532_INIT_DONE,
  PN532_POLL_SEND,
  PN532_POLL_WAIT_ACK,
  PN532_POLL_WAIT_RESPONSE,
  PN532_POLL_COOLDOWN
} pn532_init_state_t;

static struct {
  uint8_t dev_addr;
  pn532_init_state_t init_state;
  uint32_t init_timer;
  uint8_t tx_buffer[64];
  uint8_t rx_buffer[128]; // Increased size for safety
  size_t rx_idx;          // Accumulator index
  uint8_t last_uid[10];
  uint8_t last_uid_len;
  bool new_tag;
} pn532_state = {0};

static uint8_t pn532_calc_dcs(const uint8_t *data, size_t len) {
  uint8_t sum = 0;
  for (size_t i = 0; i < len; i++) {
    sum += data[i];
  }
  return ~sum + 1;
}

static size_t pn532_build_frame(uint8_t *buffer, uint8_t cmd,
                                const uint8_t *data, size_t data_len) {
  size_t pos = 0;
  uint8_t len = data_len + 2;
  uint8_t lcs = ~len + 1;

  buffer[pos++] = PN532_PREAMBLE;
  buffer[pos++] = PN532_STARTCODE1;
  buffer[pos++] = PN532_STARTCODE2;
  buffer[pos++] = len;
  buffer[pos++] = lcs;
  buffer[pos++] = PN532_HOSTTOPN532;
  buffer[pos++] = cmd;
  if (data && data_len > 0) {
    memcpy(&buffer[pos], data, data_len);
    pos += data_len;
  }
  uint8_t dcs = pn532_calc_dcs(&buffer[5], len);
  buffer[pos++] = dcs;
  buffer[pos++] = PN532_POSTAMBLE;
  return pos;
}

bool pn532_read_tag(uint8_t *uid, uint8_t *len) {
  if (!pn532_state.new_tag)
    return false;
  *len = pn532_state.last_uid_len;
  memcpy(uid, pn532_state.last_uid, *len);
  pn532_state.new_tag = false;
  return true;
}

bool pn532_init(uint8_t dev_addr) {
  memset(&pn532_state, 0, sizeof(pn532_state));
  pn532_state.dev_addr = dev_addr;
  if (dev_addr != 0) {
    pn532_state.init_state = PN532_INIT_WAKEUP;
    pn532_state.init_timer = to_ms_since_boot(get_absolute_time());
    printf("PN532: Starting async initialization for device %d\n", dev_addr);
  }
  return true;
}

bool pn532_task(void) {
  uint32_t now = to_ms_since_boot(get_absolute_time());

  switch (pn532_state.init_state) {
  case PN532_INIT_IDLE:
    break;

  case PN532_INIT_WAKEUP:
    if ((now - pn532_state.init_timer) > 100) { // 100ms settledown
      printf("PN532: Sending Atomic Wakeup + SAMConfiguration...\n");

      // Buffer: [Preamble (16)] + [Frame]
      uint8_t buffer[64];
      size_t pos = 0;

      // 1. Long Preamble (0x55 x 2 + 0x00 x 14)
      buffer[pos++] = 0x55;
      buffer[pos++] = 0x55;
      for (int i = 0; i < 14; i++)
        buffer[pos++] = 0x00;

      // 2. SAMConfig Frame (Appended directly)
      // Params: Normal Mode (01), Timeout 1s (14), IRQ (01)
      uint8_t params[] = {0x01, 0x14, 0x01};
      pos += pn532_build_frame(&buffer[pos], PN532_COMMAND_SAMCONFIGURATION,
                               params, sizeof(params));

      // Start listening (Expect ACK in response to Frame)
      ch340_read(pn532_state.dev_addr, pn532_state.rx_buffer,
                 sizeof(pn532_state.rx_buffer));

      // Send Atomic Blob
      ch340_write(pn532_state.dev_addr, buffer, pos);

      pn532_state.init_state = PN532_INIT_WAIT_ACK;
      pn532_state.init_timer = now;
    }
    break;

    // DEL: PN532_INIT_SEND_ACK and PN532_INIT_SENDING_FW_REQ
    // We jumped straight to WAIT_ACK

  case PN532_INIT_WAIT_ACK: {
    int len = ch340_read(pn532_state.dev_addr, pn532_state.rx_buffer,
                         sizeof(pn532_state.rx_buffer));
    if (len > 0) {
      printf("PN532 RX: ");
      for (int i = 0; i < len; i++)
        printf("%02X ", pn532_state.rx_buffer[i]);
      printf("\n");

      // Simple check for ACK (00 00 FF 00 FF 00)
      // In a real implementation we'd implement a proper sliding window or
      // buffer But for debug we want to see ANY data
      for (int i = 0; i < len - 5; i++) {
        if (pn532_state.rx_buffer[i] == 0x00 &&
            pn532_state.rx_buffer[i + 1] == 0x00 &&
            pn532_state.rx_buffer[i + 2] == 0xFF) {
          printf("PN532: ACK Received!\n");
          pn532_state.init_state = PN532_INIT_WAIT_RESPONSE;
          pn532_state.init_timer = now;
          return false;
        }
      }
    }

    if ((now - pn532_state.init_timer) > 1000) {
      printf("PN532: ACK timeout, retrying...\n");
      pn532_state.init_state = PN532_INIT_WAKEUP; // Retry full Awake logic
      pn532_state.init_timer = now;
    }
  } break;

  case PN532_INIT_WAIT_RESPONSE:
    // Check if we already have the response in the buffer (piggybacked on ACK)
    // Or normally we would trigger a read here.
    // For now, scan the buffer for the SAMConfig response (D5 15)
    for (int i = 0; i < 32; i++) { // Scan first part of buffer
      if (pn532_state.rx_buffer[i] == 0xD5 &&
          pn532_state.rx_buffer[i + 1] == 0x15) {
        printf("PN532: SAMConfiguration Successful! (Normal Mode Set)\n");
        pn532_state.init_state = PN532_INIT_DONE;
        return false;
      }
    }
    // TODO: If not found, maybe trigger another read?
    // But our atomic log showed it came in one chunk.
    pn532_state.init_state = PN532_INIT_RF_CONFIG; // Next step
    break;

  // Set MaxRetry to infinite so we don't miss tags
  case PN532_INIT_RF_CONFIG: {
    // MXRTY (Item 5) = 0xFF, 0x01, 0xFF (Infinite retries)
    uint8_t buffer[64];
    uint8_t params[] = {0x05, 0xFF, 0x01, 0xFF};
    size_t len = pn532_build_frame(buffer, PN532_COMMAND_RFCONFIGURATION,
                                   params, sizeof(params));
    ch340_write(pn532_state.dev_addr, buffer, len);
    ch340_read(pn532_state.dev_addr, pn532_state.rx_buffer,
               sizeof(pn532_state.rx_buffer));
    pn532_state.init_state = PN532_INIT_RF_WAIT_ACK;
    pn532_state.init_timer = now;
  } break;

  case PN532_INIT_RF_WAIT_ACK: {
    int bytes = ch340_read(pn532_state.dev_addr, pn532_state.rx_buffer,
                           sizeof(pn532_state.rx_buffer));
    if (bytes > 0) {
      // We ignore the actual response packet (D5 33) for now for speed
      // Just assume it worked if we got an ACK/Response
      pn532_state.init_state = PN532_INIT_DONE;
      return false;
    }
    if ((now - pn532_state.init_timer) > 200) {
      // Timeout, just proceed
      pn532_state.init_state = PN532_INIT_DONE;
    }
  } break;

  case PN532_INIT_DONE:
    // Initialized. Start polling loop.
    pn532_state.init_state = PN532_POLL_SEND;
    break;

  case PN532_POLL_SEND:
    if ((now - pn532_state.init_timer) > 100) { // Poll every 100ms
      // printf("PN532: Polling for targets...\n");
      uint8_t buffer[64];
      // InListPassiveTarget: MaxTg=1, BrTy=0 (106 kbps type A)
      uint8_t params[] = {0x01, 0x00};
      size_t len = pn532_build_frame(buffer, PN532_COMMAND_INLISTPASSIVETARGET,
                                     params, sizeof(params));

      ch340_write(pn532_state.dev_addr, buffer, len);

      // Reset accumulator
      pn532_state.rx_idx = 0;
      memset(pn532_state.rx_buffer, 0, sizeof(pn532_state.rx_buffer));

      // Prepare to read ACK (into start of buffer)
      ch340_read(pn532_state.dev_addr, pn532_state.rx_buffer,
                 sizeof(pn532_state.rx_buffer));

      pn532_state.init_state = PN532_POLL_WAIT_ACK;
      pn532_state.init_timer = now;
    }
    break;

  case PN532_POLL_WAIT_ACK: {
    // Check if we got ANY data (assuming it's ACK or start of it)
    int bytes = ch340_read(pn532_state.dev_addr,
                           &pn532_state.rx_buffer[pn532_state.rx_idx],
                           sizeof(pn532_state.rx_buffer) - pn532_state.rx_idx);
    if (bytes > 0) {
      pn532_state.rx_idx += bytes;
      // We assume if we got data, we can move to waiting for response
      // (Or checking if we have enough data to scan)
      // Since MaxRetry=Infinite, we won't get empty packets easily.
      pn532_state.init_state = PN532_POLL_WAIT_RESPONSE;
      pn532_state.init_timer = now;
    } else if (now - pn532_state.init_timer > 100) {
      // Timeout waiting for ACK? Reset.
      pn532_state.init_state = PN532_POLL_COOLDOWN;
      pn532_state.init_timer = now;
    }
  } break;

  case PN532_POLL_WAIT_RESPONSE: {
    // Keep reading into accumulator
    if (pn532_state.rx_idx < sizeof(pn532_state.rx_buffer)) {
      int bytes = ch340_read(
          pn532_state.dev_addr, &pn532_state.rx_buffer[pn532_state.rx_idx],
          sizeof(pn532_state.rx_buffer) - pn532_state.rx_idx);
      if (bytes > 0) {
        pn532_state.rx_idx += bytes;
        pn532_state.init_timer = now; // Refresh timeout on new data
      }
    }

    // Scan for response code D5 4B (InListPassiveTarget Response)
    uint8_t *buf = pn532_state.rx_buffer;
    // Ensure we have enough data to even check (at least 2 bytes)
    if (pn532_state.rx_idx >= 2) {
      for (size_t i = 0; i < pn532_state.rx_idx - 1; i++) {
        if (buf[i] == 0xD5 && buf[i + 1] == 0x4B) {
          // Found Header! Check if we have the rest...
          // Need [NbTg]...
          if (i + 2 < pn532_state.rx_idx) {
            uint8_t nb_tg = buf[i + 2];
            if (nb_tg > 0) {
              // Target Found!
              // Tg1 starts at buf[i+3]
              // [Tg#] [SENS_RES(2)] [SEL_RES(1)] [NFCIDLength(1)] [NFCID(N)]
              // Need up to NFCIDLength.
              // Min length check: i+8 (header)
              if (i + 8 < pn532_state.rx_idx) {
                uint8_t uid_len = buf[i + 7];
                if (i + 8 + uid_len <= pn532_state.rx_idx) {
                  uint8_t *uid = &buf[i + 8];

                  printf("PN532: Tag Found! UID Length: %d, UID: ", uid_len);
                  for (int j = 0; j < uid_len; j++)
                    printf("%02X ", uid[j]);
                  printf("\n");

                  // Store tag data
                  if (uid_len <= sizeof(pn532_state.last_uid)) {
                    memcpy(pn532_state.last_uid, uid, uid_len);
                    pn532_state.last_uid_len = uid_len;
                    pn532_state.new_tag = true;
                  }

                  // reset
                  pn532_state.init_state = PN532_POLL_COOLDOWN;
                  pn532_state.init_timer = now;
                  return false;
                }
              }
            } else {
              // 0 Targets. Still valid response.
              // Just cooldown.
              pn532_state.init_state = PN532_POLL_COOLDOWN;
              pn532_state.init_timer = now;
              return false;
            }
          }
        }
      }
    }

    // Timeout logic:
    // If we have data (rx_idx > 0) but didn't find a complete valid frame
    // within 200ms, assume garbage and reset.
    // If rx_idx == 0, we wait FOREVER (Infinite Polling).
    if (pn532_state.rx_idx > 0 && (now - pn532_state.init_timer > 200)) {
      pn532_state.init_state = PN532_POLL_COOLDOWN;
      pn532_state.init_timer = now;
    }
  } break;

  case PN532_POLL_COOLDOWN:
    if (now - pn532_state.init_timer > 500) { // Wait 500ms before re-arming
      pn532_state.init_state = PN532_POLL_SEND;
      pn532_state.init_timer = now;
    }
    break;
  }
  return false;
}
