/*
 * Copyright (c) 2025 Rumbledethumps
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "nfc/ch340.h"
#include "nfc/nfc.h"
#include <stdio.h>
#include <string.h>
#include <tusb.h>

// TinyUSB device driver for CH340/CH341 (Manual implementation)

// Transfer state for async operations
typedef enum {
  CH340_STATE_IDLE = 0,
  CH340_STATE_MOUNTED,
  CH340_STATE_RESET_EP_IN,
  CH340_STATE_RESET_EP_OUT,
  CH340_STATE_RESET_PULSE_ASSERT,
  CH340_STATE_RESET_PULSE_DEASSERT,
  CH340_STATE_SEQ_BAUD1,
  CH340_STATE_SEQ_BAUD2,
  CH340_STATE_SEQ_ENABLE,
  CH340_STATE_SEQ_LCR,
  CH340_STATE_SEQ_HANDSHAKE,
  CH340_STATE_SEQ_VERSION,
  CH340_STATE_READY,
} ch340_state_t;

static struct {
  uint8_t dev_addr;
  uint8_t ep_in;
  uint8_t ep_out;
  bool transfer_complete;
  uint32_t xferred_bytes;
  bool write_complete;
  bool read_complete;
  bool read_pending;
  ch340_state_t state;
  __attribute__((aligned(4))) uint8_t rx_buffer[256];
} ch340_driver = {0};

void ch340_init(void) { memset(&ch340_driver, 0, sizeof(ch340_driver)); }

// USB transfer complete callback for control transfers
static void ch340_control_complete_cb(tuh_xfer_t *xfer) {
  ch340_driver.transfer_complete = true;
  ch340_driver.xferred_bytes = xfer->actual_len;

  if (xfer->result != XFER_RESULT_SUCCESS) {
    printf("CH340: Control transfer failed: %d\n", xfer->result);
  }
}

// Send USB control request to CH340
static bool ch340_control_out(uint8_t dev_addr, uint8_t request, uint16_t value,
                              uint16_t index) {
  ch340_driver.transfer_complete = false;

  tuh_xfer_t xfer = {
      .daddr = dev_addr,
      .ep_addr = 0,
      .setup =
          &(tusb_control_request_t){.bmRequestType =
                                        0x40, // Host to device, vendor, device
                                    .bRequest = request,
                                    .wValue = value,
                                    .wIndex = index,
                                    .wLength = 0},
      .buffer = NULL,
      .complete_cb = ch340_control_complete_cb,
      .user_data = 0};

  return tuh_control_xfer(&xfer);
}

// Send USB control request (IN) to CH340
static bool ch340_control_in(uint8_t dev_addr, uint8_t request, uint16_t value,
                             uint16_t index, uint8_t *buffer, uint16_t len) {
  ch340_driver.transfer_complete = false;

  tuh_xfer_t xfer = {
      .daddr = dev_addr,
      .ep_addr = 0,
      .setup =
          &(tusb_control_request_t){.bmRequestType =
                                        0xC0, // Device to Host, vendor, device
                                    .bRequest = request,
                                    .wValue = value,
                                    .wIndex = index,
                                    .wLength = len},
      .buffer = buffer,
      .complete_cb = ch340_control_complete_cb,
      .user_data = 0};

  return tuh_control_xfer(&xfer);
}

void ch340_task(void) {
  switch (ch340_driver.state) {
  case CH340_STATE_IDLE:
    break;

  case CH340_STATE_MOUNTED:
    // Step 1: 0xA1 0xC29C 0xB2C9 (Reset/Init)
    printf("CH340: Step 1 - Init\n");
    if (ch340_control_out(ch340_driver.dev_addr, CH340_REQ_SERIAL_INIT, 0xC29C,
                          0xB2C9)) {
      ch340_driver.state = CH340_STATE_RESET_EP_IN;
    }
    break;

  case CH340_STATE_RESET_EP_IN:
    if (ch340_driver.transfer_complete) {
      printf("CH340: Resetting EP IN\n");
      // Clear Feature (Endpoint Halt) on EP IN
      // bmRequestType: 0x02 (Recip: Endpoint), bRequest: 0x01 (Clear Feature)
      tuh_xfer_t xfer = {
          .daddr = ch340_driver.dev_addr,
          .ep_addr = 0,
          .setup =
              &(tusb_control_request_t){.bmRequestType = 0x02,
                                        .bRequest = TUSB_REQ_CLEAR_FEATURE,
                                        .wValue = TUSB_REQ_FEATURE_EDPT_HALT,
                                        .wIndex = ch340_driver.ep_in,
                                        .wLength = 0},
          .complete_cb = ch340_control_complete_cb,
          .user_data = 0};
      ch340_driver.transfer_complete = false;
      if (tuh_control_xfer(&xfer)) {
        ch340_driver.state = CH340_STATE_RESET_EP_OUT;
      }
    }
    break;

  case CH340_STATE_RESET_EP_OUT:
    if (ch340_driver.transfer_complete) {
      printf("CH340: Resetting EP OUT\n");
      // Clear Feature (Endpoint Halt) on EP OUT
      tuh_xfer_t xfer = {
          .daddr = ch340_driver.dev_addr,
          .ep_addr = 0,
          .setup =
              &(tusb_control_request_t){.bmRequestType = 0x02,
                                        .bRequest = TUSB_REQ_CLEAR_FEATURE,
                                        .wValue = TUSB_REQ_FEATURE_EDPT_HALT,
                                        .wIndex = ch340_driver.ep_out,
                                        .wLength = 0},
          .complete_cb = ch340_control_complete_cb,
          .user_data = 0};
      ch340_driver.transfer_complete = false;
      if (tuh_control_xfer(&xfer)) {
        ch340_driver.state = CH340_STATE_RESET_PULSE_ASSERT;
      }
    }
    break;

  case CH340_STATE_RESET_PULSE_ASSERT:
    if (ch340_driver.transfer_complete) {
      // Assert DTR (Reset Low): 0x3C & ~0x20 = 0x1C
      // Bit 5 (0x20) is DTR. 0 = Active/Low.
      printf("CH340: Asserting DTR (Reset Low)\n");
      if (ch340_control_out(ch340_driver.dev_addr, CH340_REQ_MODEM_CTRL, 0x001C,
                            0x0000)) {
        ch340_driver.state = CH340_STATE_RESET_PULSE_DEASSERT;
      }
    }
    break;

  case CH340_STATE_RESET_PULSE_DEASSERT:
    if (ch340_driver.transfer_complete) {
      // Deassert DTR (Reset High): 0x3C (Default Linux idle)
      printf("CH340: Deasserting DTR (Run High)\n");
      // Add a small delay here? The state machine cycle time might be enough.
      // But let's rely on the control transfer time ~1ms.
      if (ch340_control_out(ch340_driver.dev_addr, CH340_REQ_MODEM_CTRL, 0x003C,
                            0x0000)) {
        ch340_driver.state = CH340_STATE_SEQ_BAUD1;
      }
    }
    break;

  case CH340_STATE_SEQ_BAUD1:
    if (ch340_driver.transfer_complete) {
      // Step 1.5: 0x9A 0x1312 0x9807 (Correct 115200 Baud for CH340)
      printf("CH340: Step 1.5 - Set Baud 115200 (Value: 0x9807)\n");
      if (ch340_control_out(ch340_driver.dev_addr, CH340_REQ_WRITE_REG, 0x1312,
                            0x9807)) {
        // Skip Prescaler (Step 1.6), jump to Handshake/Enable
        ch340_driver.state = CH340_STATE_SEQ_ENABLE;
      }
    }
    break;

  case CH340_STATE_SEQ_BAUD2:
    // Unused
    ch340_driver.state = CH340_STATE_SEQ_ENABLE;
    break;

  case CH340_STATE_SEQ_ENABLE:
    if (ch340_driver.transfer_complete) {
      // Step 2: 0xA4 0x003C 0x0000 (Modem Control: ~0xC3)
      // Linux driver sends ~LCR to 0xA4. ~0xC3 = 0x3C.
      printf("CH340: Step 2 - Modem Control (Linux Style: 0x3C)\n");
      if (ch340_control_out(ch340_driver.dev_addr, CH340_REQ_MODEM_CTRL, 0x003C,
                            0x0000)) {
        ch340_driver.state = CH340_STATE_SEQ_LCR;
      }
    }
    break;

  case CH340_STATE_SEQ_LCR:
    if (ch340_driver.transfer_complete) {
      // Step 3: 0x9A 0x2518 0x00C3 (Set Baud/LCR 115200, Enable RX/TX)
      printf("CH340: Step 3 - Set LCR (RX/TX ON)\n");
      if (ch340_control_out(ch340_driver.dev_addr, CH340_REQ_WRITE_REG, 0x2518,
                            0x00C3)) {
        ch340_driver.state = CH340_STATE_SEQ_HANDSHAKE;
      }
    }
    break;

  case CH340_STATE_SEQ_HANDSHAKE:
    if (ch340_driver.transfer_complete) {
      // Step 4: 0xA4 0x003C (Linux Handshake)
      printf("CH340: Step 4 - Final Handshake (Linux 0x3C)\n");
      if (ch340_control_out(ch340_driver.dev_addr, CH340_REQ_MODEM_CTRL, 0x003C,
                            0x0000)) {
        ch340_driver.state = CH340_STATE_SEQ_VERSION;
      }
    }
    break;

  case CH340_STATE_SEQ_VERSION:
    if (ch340_driver.transfer_complete) {
      // Step 5: 0x5F 0x0000 0x0000 (Read Version)
      printf("CH340: Step 5 - Read Version\n");
      if (ch340_control_in(ch340_driver.dev_addr, CH340_REQ_READ_VERSION,
                           0x0000, 0x0000, ch340_driver.rx_buffer, 2)) {
        ch340_driver.state = CH340_STATE_READY;
      }
    }
    break;

  case CH340_STATE_READY:
    if (ch340_driver.transfer_complete) {
      // Check version result
      uint8_t ver = ch340_driver.rx_buffer[0];
      printf("CH340: Version Read: 0x%02X\n", ver);

      ch340_driver.transfer_complete = false; // Clear flag
      printf("CH340: Initialization complete, device ready.\n");
      nfc_cdc_mounted(ch340_driver.dev_addr, 0x1A86, 0x7523);
    }
    break;
  }
}

// Bulk transfer callback
static void ch340_bulk_complete_cb(tuh_xfer_t *xfer) {
  if (xfer->result != XFER_RESULT_SUCCESS) {
    printf("CH340: Bulk xfer failed: %d (EP 0x%02x)\n", xfer->result,
           xfer->ep_addr);
  }

  if (xfer->ep_addr == ch340_driver.ep_in) {
    ch340_driver.read_complete = true;
    ch340_driver.read_pending = false;
    ch340_driver.xferred_bytes = xfer->actual_len;
    printf("CH340: Bulk IN complete, len=%lu\n", xfer->actual_len); // Debug!
  } else if (xfer->ep_addr == ch340_driver.ep_out) {
    ch340_driver.write_complete = true;
    printf("CH340: Bulk OUT complete, len=%lu\n",
           xfer->actual_len); // Confirm writes!
  }
}

bool ch340_write(uint8_t dev_addr, const uint8_t *data, size_t len) {
  if (ch340_driver.state != CH340_STATE_READY ||
      dev_addr != ch340_driver.dev_addr) {
    return false;
  }

  ch340_driver.write_complete = false;

  tuh_xfer_t xfer = {.daddr = dev_addr,
                     .ep_addr = ch340_driver.ep_out,
                     .buflen = len,
                     .buffer = (uint8_t *)data,
                     .complete_cb = ch340_bulk_complete_cb,
                     .user_data = 0};

  return tuh_edpt_xfer(&xfer);
}

void ch340_mount(uint8_t dev_addr, uint16_t vid, uint16_t pid) {
  printf("CH340: Device mounted - VID: %04x PID: %04x\n", vid, pid);

  if (vid == 0x1A86 && pid == 0x7523) {
    ch340_driver.dev_addr = dev_addr;
    ch340_driver.ep_in = 0;
    ch340_driver.ep_out = 0;

    // Get configuration descriptor
    uint8_t desc_buf[256];
    if (tuh_descriptor_get_configuration_sync(
            dev_addr, 0, desc_buf, sizeof(desc_buf)) != XFER_RESULT_SUCCESS) {
      printf("CH340: Failed to get configuration descriptor\n");
      return;
    }

    tusb_desc_endpoint_t ep_desc_in = {0};
    tusb_desc_endpoint_t ep_desc_out = {0};

    // Find and copy actual endpoint descriptors
    uint8_t *p_desc = desc_buf;
    uint8_t *p_end =
        desc_buf + ((tusb_desc_configuration_t *)desc_buf)->wTotalLength;
    p_desc += p_desc[0]; // Skip config

    while (p_desc < p_end) {
      if (p_desc[1] == TUSB_DESC_INTERFACE) {
        p_desc += p_desc[0];
        continue;
      }
      if (p_desc[1] == TUSB_DESC_ENDPOINT) {
        tusb_desc_endpoint_t *ep_desc = (tusb_desc_endpoint_t *)p_desc;
        if (ep_desc->bmAttributes.xfer == TUSB_XFER_BULK) {
          if (ep_desc->bEndpointAddress & 0x80) {
            memcpy(&ep_desc_in, ep_desc, sizeof(tusb_desc_endpoint_t));
            ch340_driver.ep_in = ep_desc->bEndpointAddress;
            printf("CH340: Found EP IN: 0x%02x (Size: %d)\n",
                   ch340_driver.ep_in, ep_desc_in.wMaxPacketSize);
          } else {
            memcpy(&ep_desc_out, ep_desc, sizeof(tusb_desc_endpoint_t));
            ch340_driver.ep_out = ep_desc->bEndpointAddress;
            printf("CH340: Found EP OUT: 0x%02x (Size: %d)\n",
                   ch340_driver.ep_out, ep_desc_out.wMaxPacketSize);
          }
        }
      }
      p_desc += p_desc[0];
    }

    if (ch340_driver.ep_in == 0 || ch340_driver.ep_out == 0) {
      printf("CH340: Failed to find bulk endpoints\n");
      return;
    }

    // Open endpoints using actual descriptors
    if (tuh_edpt_open(dev_addr, &ep_desc_in) &&
        tuh_edpt_open(dev_addr, &ep_desc_out)) {
      ch340_driver.state = CH340_STATE_MOUNTED;
    } else {
      printf("CH340: Failed to open endpoints\n");
    }
  }
}

void ch340_umount(uint8_t dev_addr) {
  if (dev_addr == ch340_driver.dev_addr) {
    printf("CH340: Device unmounted\n");
    ch340_driver.state = CH340_STATE_IDLE;
    ch340_driver.dev_addr = 0;
    nfc_cdc_unmounted(0);
  }
}

int ch340_read(uint8_t dev_addr, uint8_t *buffer, size_t max_len) {
  if (ch340_driver.state != CH340_STATE_READY ||
      dev_addr != ch340_driver.dev_addr) {
    return -1;
  }

  if (ch340_driver.read_complete) {
    int len = ch340_driver.xferred_bytes;
    if (len > (int)max_len)
      len = (int)max_len;

    if (buffer && len > 0) {
      memcpy(buffer, ch340_driver.rx_buffer, len);
    }

    ch340_driver.read_complete = false;
    return len;
  }

  if (!ch340_driver.read_pending) {
    ch340_driver.read_pending = true;
    ch340_driver.xferred_bytes = 0;

    tuh_xfer_t xfer = {.daddr = dev_addr,
                       .ep_addr = ch340_driver.ep_in,
                       .buflen = sizeof(ch340_driver.rx_buffer),
                       .buffer = ch340_driver.rx_buffer,
                       .complete_cb = ch340_bulk_complete_cb,
                       .user_data = 0};

    if (!tuh_edpt_xfer(&xfer)) {
      ch340_driver.read_pending = false;
      return -1;
    }
  }

  return 0;
}

bool ch340_is_ready(uint8_t dev_addr) {
  return (ch340_driver.state == CH340_STATE_READY) &&
         (ch340_driver.dev_addr == dev_addr);
}

int ch340_count(void) {
  return (ch340_driver.state != CH340_STATE_IDLE) ? 1 : 0;
}
