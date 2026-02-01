# NFC Support for RP6502 (CH340 + PN532)

This module provides support for generic **PN532 NFC/RFID readers** connected via a **CH340 USB-to-Serial** dongle.

## Hardware Support
- **Connector:** USB-C / USB-A
- **Chipset:** CH340 / CH341 (USB-to-Serial)
- **NFC Module:** PN532 (NXP)
- **Verified Device:** "Smart NFC/RFID Reader" Dongles (commonly found on AliExpress/Amazon)

## Technical Implementation Details

### 1. CH340 Driver (`ch340.c`)
The CH340 is a "dumb" USB-to-Serial converter that requires a vendor-specific initialization sequence. This driver manually implements the USB control transfers (Vendor Class) to configure the chip, as TinyUSB does not include a native CH340 driver.

**Critical Driver Quirks Discovered:**
*   **Baud Rate (115200):** The correct USB register values for 115200 baud are `0x9807` writing to register `0x1312`. (Many open-source sources incorrectly list `0xCC09`, which is ~230400 result).
*   **Handshake/Modem Control:** The Linux driver style of sending `0x3C` to register `0xA4` is required to enable the RX/TX lines.
*   **DTR Pulse:** Some dongles wire the PN532 `RST` pin to the CH340 `DTR` line. The driver asserts DTR (Low) then releases it (High) during initialization to ensure the PN532 is not stuck in reset.
*   **Endpoint Sizes:** The driver dynamically reads the `wMaxPacketSize` from the device descriptors (typically 32 bytes) instead of assuming 64 bytes, preventing silent packet drops.

### 2. PN532 Protocol (`pn532.c`)
The PN532 uses a binary instruction set. Because it is connected via a "slow" UART bridge, timing is critical.

**Wakeup Sequence:**
The PN532 has a low-power mode that requires a specific "Preamble" to wake up.
*   **Method:** "Atomic Wakeup"
*   **Sequence:** `0x55 0x55` + `0x00` x 14 (Preamble) followed *immediately* by the `SAMConfiguration` command frame.
*   **Why:** Sending the Preamble and Command separately (with a delay) caused the PN532 to wake up, see silence, and go back to sleep before the command arrived. Sending them as a single USB transfer ensures reliable communication.

### 3. Usage
The NFC reader is automatically detected at plug-in.
*   **Status:** Use the `status` command in the monitor to see connected devices (e.g., "1 NFC reader").
*   **Integration:** The driver hooks into `usb.c` and is managed by the RIA task scheduler.

## Configuration (`NFC.TXT`)
To map NFC tags to commands, create a file named `NFC.TXT` in the root of the USB drive or SD card (littlefs).

**Format:**
```text
UID_HEX = COMMAND
```

**Example:**
```text
# Launch Elite when this tag is scanned
04A1B2C3 = LOAD ELITE

# List files for this tag
12345678 = LS

# Comments start with #
```

**Finding UIDs:**
If a tag is scanned but not found in `NFC.TXT`, the RIA will print the UID to the USB console:
`NFC: Tag 04A1B2C3 not found in NFC.TXT`
You can copy this UID into your config file.

## Future Work
- Map NFC Tags to keyboard macros or file loading events. (Partially Implemented via mon_run_command)

