# MiSTer FPGA Port of Picocomputer (RP6502)

This document provides build, compilation, and deployment instructions for running the Picocomputer (RP6502) emulator on the MiSTer FPGA.

The emulator utilizes a hybrid architecture:
1. **ARM HPS (Linux)**: Runs the W65C02 CPU emulation loop, the RIA interface, and the Sokol-based audio/input controllers.
2. **FPGA**: Handles VGA timing, resolution scaling, composite video mixing, scanlines, and HDMI/analog video output.

---

## Repository Structure

* **`src/emu/`**: Main C/C++ emulator code run on the ARM CPU.
* **`src/fpga/`**: SystemVerilog source code (`menu.sv`, `rtl/`, `sys/`) and the Quartus Prime project configuration files (`menu.qpf` / `menu.qsf`) for building the FPGA core bitstream.

For MiSTer wrapper-core builds, this repository's `src/fpga/` tree is mirrored to the sibling repository `../3s-mister-arm/vendor/Menu_MiSTer/`. Use the helper script before building the core there:

```bash
tools/sync-fpga-to-wrapper.sh
```

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
   The FPGA bitstream is built from the sibling repository `../3s-mister-arm` (not from this `rp6502` tree).

   First, sync any local FPGA edits from this repo into the wrapper seed:
   ```bash
   tools/sync-fpga-to-wrapper.sh
   ```

   From `../3s-mister-arm`:
   ```bash
   # 1) Put Quartus 17 tools on PATH
   export PATH="/home/rowe/intelFPGA_lite/17.0/quartus/bin:$PATH"

   # 2) Optional sanity check (shows whether local Quartus mode is available)
   tools/mister-wrapper/build-core.sh --check-env

   # 3) Build the core (default seed is menu)
   tools/mister-wrapper/build-core.sh --seed menu
   ```

   Output artifact:
   - `../3s-mister-arm/build/mister-wrapper-core/3S-ARM_YYYYMMDD.rbf`

   Deploy the generated RBF to:
   - `/media/fat/_Other/3S-ARM.rbf`

   Example deploy:
   ```bash
   scp ../3s-mister-arm/build/mister-wrapper-core/3S-ARM_*.rbf root@mister.home.arpa:/media/fat/_Other/3S-ARM.rbf
   ```

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

---

## 5. Optimization & Performance Tuning (8 MHz 60 FPS)

During porting, the Cyclone V HPS (dual-core 32-bit Cortex-A9 from 2007) encountered VSYNC dropouts (dropping to exactly 30 FPS due to the "VSYNC cliff") when emulating the W65C02 CPU at its full **8 MHz** clock speed. 

We applied several critical optimizations to achieve a stable, rock-solid **60 FPS** at the target 8 MHz CPU clock:

### Architectural & Compiler Optimizations
1. **Parallel VGA Scanline Rendering (Asynchronous Core Handoff)**:
   * By default, VGA scanline compositing took ~5.3 ms of the main loop. 
   * We offloaded the entire video pipeline (VGA scanline rendering, compositing, DDR3 frame copying, and FPGA buffer swapping) to the second HPS core (**CPU 1**).
   * At the end of a frame, the main thread copies the active scanline programs (`g_prog`) to a 20 KB shadow buffer (`shadow_g_prog`). The background thread on CPU 1 renders the frame in parallel while the main emulator thread on CPU 0 immediately begins executing CPU cycles for the next frame.
   * This dropped the main thread's VGA compositing overhead from **5.3 ms to 0.0 ms**.
2. **Double-Buffered Asynchronous DDR3 Frame Copies**:
   * Removed all uncached memory writes (small 768-byte copies) directly to `/dev/mem` from the scanline loop, routing them into cached RAM.
   * Frame copies to DDR3 are handled in a single highly-optimized 172 KB burst copy on CPU 1, avoiding memory transaction stalls on CPU 0.
3. **CPU Core Affinity Pinning**:
   * Pinned the main emulator thread to **CPU 0** and the background rendering/copy thread to **CPU 1** using `pthread_setaffinity_np`. This isolates the timing-sensitive cycle execution loop from OS context switches, cache invalidation, and interference from other system processes (like the main `MiSTer` binary).
4. **32-Bit Hot-Loop Optimization (Cortex-A9 64-bit Math Bypass)**:
   * Since the Cortex-A9 is a 32-bit CPU, 64-bit comparisons and timing additions in the loop condition (`clock_8 < deadline_8` and `clock_8 += step_8`) were causing significant register pressure and stack spilling.
   * We optimized `run_until` to compute the required cycle steps upfront as a 32-bit integer, running the hot loop as a simple 32-bit decrement-and-branch loop.
   * Replaced 64-bit divisions (`n * 4096000ull / 63`) on the scanline boundary with a precalculated O(1) table lookup (`scanline_deadlines[528]`).
   * Explicitly cast `pins` to `uint32_t` and `uint16_t` for address extraction and status checking in `cpu_tick_fast()`, ensuring all shifts, masks, and tests compile to fast, single-cycle 32-bit instructions.
5. **Cached Interrupt Assertions**:
   * Cached the level-triggered `ria_irq_asserted()` check into a global boolean variable (`ria_irq_asserted_cached`) updated only when the IRQ registers actually change, avoiding redundant memory structure accesses on every single clock cycle.
6. **Compiler Optimizations**:
   * Passed aggressive `-Ofast -funroll-loops` targeting the Cortex-A9 FPU/NEON pipeline (`-mcpu=cortex-a9 -mfpu=neon`) in `src/emu/CMakeLists.txt`.

### Resulting Timing Metrics (8 MHz)
* **VGA Compositing**: 0.00 ms (Main Thread) / ~5.3 ms (CPU 1)
* **Video DDR3 Copy**: 0.02 ms (Main Thread) / ~2.0 ms (CPU 1)
* **CPU Cycle Emulation**: ~17.3 ms (under active SSH logging) / **~13.8 ms** (natively)
* **Native Frame Duration**: **~14.8 ms** (comfortably under the 16.6 ms VSYNC deadline, yielding a solid 60 FPS)

---

## 6. Build & Compilation Toolchain Reference

### Toolchain Details
* **Cross-Compiler**: ARM GNU Toolchain (`arm-none-linux-gnueabihf`) version 13.2.Rel1.
* **Build System**: CMake (configured in `src/emu/CMakeLists.txt`).

### Step-by-Step Compilation Commands
```bash
# 1. Export path to your toolchain binaries
export PATH="/home/rowe/opt/arm-gnu-toolchain-13.2.Rel1-x86_64-arm-none-linux-gnueabihf/bin/:$PATH"

# 2. Clean previous build folders (optional)
rm -rf build-mister

# 3. Configure the CMake project for MiSTer target cross-compilation
cmake -S src/emu -B build-mister -DCMAKE_BUILD_TYPE=Release -DMISTER=ON

# 4. Build the emulator target with parallel compilation
cmake --build build-mister --parallel 4
```
