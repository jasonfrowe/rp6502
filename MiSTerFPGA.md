# MiSTer FPGA Port of Picocomputer (RP6502)

This document provides build, compilation, and deployment instructions for running the Picocomputer (RP6502) emulator on the MiSTer FPGA.

The emulator utilizes a hybrid architecture:
1. **ARM HPS (Linux)**: Runs the W65C02 CPU emulation loop, the RIA interface, and the Sokol-based audio/input controllers.
2. **FPGA**: Handles VGA timing, resolution scaling, composite video mixing, scanlines, and HDMI/analog video output.

---

## Repository Structure

* **`src/emu/`**: Main C/C++ emulator code run on the ARM CPU.
* **`src/fpga/`**: SystemVerilog source code (`menu.sv`, `rtl/`, `sys/`) and the Quartus Prime project configuration files (`menu.qpf` / `menu.qsf`) for building the FPGA core bitstream.

---

## 1. Building the ARM Emulator (`3s-arm`)

To cross-compile the emulator for the 32-bit ARM Cortex-A9 processor on the DE10-Nano, use the cross-compiler toolchain and CMake.

### Prerequisites

You must have the ARM GNU toolchain in your path. Example:
```bash
export PATH="/home/rowe/opt/arm-gnu-toolchain-13.2.Rel1-x86_64-arm-none-linux-gnueabihf/bin/:$PATH"
```

### Build Commands

From the root of the `rp6502` repository, execute:

```bash
# Configure the build in Release mode with Cortex-A9 target architecture options
cmake -S src/emu -B build-mister -DCMAKE_BUILD_TYPE=Release -DMISTER=ON

# Compile the target
cmake --build build-mister --parallel 4
```

This compiles a statically-linked binary `build-mister/rp6502-emu` optimized for the Cyclone V Cortex-A9 CPU pipelines (using `-mcpu=cortex-a9 -mfloat-abi=hard -mfpu=neon`).

---

## 2. Building the OSD Wrapper Daemon (`MiSTer_3S-ARM`)

The OSD wrapper daemon wraps input handling, OSD config, and the core launch sequence. Because it dynamically links against standard system libraries, it **must** be compiled inside the Docker build environment to target the exact dynamic glibc version (GLIBC 2.27/3.2.0 EABI5) present on the MiSTer FPGA Linux system.

### Build Commands

In the `3s-mister-arm` folder:

```bash
# Clean the host PATH from any local ARM compiler to force Docker build mode
env PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" tools/mister-wrapper/build-hps.sh
```

The output will be generated at `build/mister-wrapper-hps/MiSTer_3S-ARM`.

---

## 3. Deploying to the MiSTer FPGA

Copy the compiled binaries and dummy files to their respective locations on the MiSTer.

### Paths and Commands

1. **OSD Wrapper Daemon**:
   ```bash
   scp build/mister-wrapper-hps/MiSTer_3S-ARM root@mister.home.arpa:/media/fat/MiSTer_3S-ARM
   ssh root@mister.home.arpa "chmod +x /media/fat/MiSTer_3S-ARM"
   ```

2. **ARM Emulator Binary**:
   ```bash
   scp build-mister/rp6502-emu root@mister.home.arpa:/media/fat/games/3s-arm/bin/3s-arm
   ssh root@mister.home.arpa "chmod +x /media/fat/games/3s-arm/bin/3s-arm"
   ```

3. **Dummy AFS Verification Signature**:
   The wrapper verifies an AFS file signature on startup. Deploy a placeholder containing the correct `AFS\0` magic header:
   ```bash
   # Generates a dummy resource file containing little-endian 0x00534641 (AFS\0)
   printf '\x41\x46\x53\x00' > SF33RD.AFS
   scp SF33RD.AFS root@mister.home.arpa:/media/fat/games/3s-arm/resources/SF33RD.AFS
   ```

4. **FPGA Core Bitstream**:
   Build the RBF file from the Quartus project located at `src/fpga/menu.qpf` and deploy it to:
   `/media/fat/_Other/3S-ARM.rbf`

---

## 4. Key Systems Notes

### Input Handling (`EVIOCGRAB`)
To prevent keyboard strokes from leaking into the background Linux console terminal while playing, the MiSTer wrapper daemon defaults to grabbing all keyboard inputs exclusively. 

During the launch handoff:
1. The wrapper releases its grab by calling `input_switch(0)`.
2. The emulator binary takes exclusive control by calling `ioctl(fd, EVIOCGRAB, 1)` on `/dev/input/event*` devices.
3. This allows the emulator to intercept all keyboard events directly. When the emulator exits, the file descriptors are closed and the grab is released.

### CPU Performance Timing
The emulator cycle tick loop is highly cycle-accurate, causing the single-core CPU usage on the dual-core Cortex-A9 to run flat out. Timings are output to `/media/fat/games/3s-arm/logs/last-run.log` every second to measure frame performance (e.g. `cpu`, `vga` render, `video` DDR3 copy, `audio` synth).
