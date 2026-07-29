# MiSTer FPGA Port of Picocomputer (RP6502)

This document provides build, compilation, and deployment instructions for running the Picocomputer (RP6502) emulator on the MiSTer FPGA.

The emulator utilizes a hybrid architecture:
1. **ARM HPS (Linux)**: Runs the W65C02 CPU emulation loop, the RIA interface, and the Sokol-based audio/input controllers.
2. **FPGA**: Handles VGA timing, resolution scaling, composite video mixing, scanlines, and HDMI/analog video output.

---

## Repository Structure

* **`src/emu/`**: Main C/C++ emulator code run on the ARM CPU.
* **`vendor/RP6502-mister-arm/`**: Tracked MiSTer wrapper/core submodule. This is the active source of truth for MiSTer-specific wrapper and FPGA-core work, including `vendor/Menu_MiSTer/`, `vendor/Main_MiSTer/`, and the `tools/mister-wrapper/` build scripts.
* **`src/fpga/`**: Legacy local FPGA tree from the earlier mirror-based workflow. It is not part of the active MiSTer core build path now that the wrapper/core repository is tracked as a submodule.

## MiSTer Resolution Notes

The emulator supports the same canvas sizes exposed by `vga_set_canvas()`:

| Canvas | Use |
|--------|-----|
| `640x480` | Boot console and terminal work |
| `320x240` | Standard low-res game canvas |
| `320x180` | Letterboxed/16:9-style low-res canvas |
| `640x360` | Widescreen high-res canvas |

The 640x480 console mode is not lost. It remains the default boot console canvas, and canvas code `3` still maps to `640x480` for full-height text and menu work.

The MiSTer video path is still 384x224 on the FPGA side, so the emulator scales or centers the selected canvas into that output. That means the user-facing resolution choice is a canvas policy, not a change to the native 384x224 DDR3 format.

---

## 1. Building the ARM Emulator (`rp6502-emu`)

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

## 2. Building the OSD Wrapper Daemon (`MiSTer_RP6502-ARM`)

The OSD wrapper daemon wraps input handling, OSD config, and the core launch sequence. Because it dynamically links against standard system libraries, it **must** be compiled inside the Docker build environment to target the exact dynamic glibc version (GLIBC 2.27/3.2.0 EABI5) present on the MiSTer FPGA Linux system.

### Build Commands

From the `vendor/RP6502-mister-arm` submodule:

```bash
cd vendor/RP6502-mister-arm

# Clean the host PATH from any local ARM compiler to force Docker build mode
env PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" tools/mister-wrapper/build-hps.sh
```

The output will be generated at `vendor/RP6502-mister-arm/build/mister-wrapper-hps/MiSTer_RP6502-ARM` when invoked from the `rp6502` repo root.

---

## 3. Deploying to the MiSTer FPGA

Copy the compiled binaries and dummy files to their respective locations on the MiSTer.

The example deploy commands below assume you are running them from the `rp6502` repo root.

### Paths and Commands

1. **OSD Wrapper Daemon**:
   ```bash
   scp vendor/RP6502-mister-arm/build/mister-wrapper-hps/MiSTer_RP6502-ARM root@mister.home.arpa:/media/fat/MiSTer_RP6502-ARM
   ssh root@mister.home.arpa "chmod +x /media/fat/MiSTer_RP6502-ARM"
   ```

2. **ARM Emulator Binary**:
   ```bash
   scp build-mister/rp6502-emu root@mister.home.arpa:/media/fat/games/RP6502/bin/rp6502-emu
   ssh root@mister.home.arpa "chmod +x /media/fat/games/RP6502/bin/rp6502-emu"
   ```

3. **Dummy AFS Verification Signature**:
   The wrapper verifies an AFS file signature on startup. Deploy a placeholder containing the correct `AFS\0` magic header:
   ```bash
   # Generates a dummy resource file containing little-endian 0x00534641 (AFS\0)
   printf '\x41\x46\x53\x00' > SF33RD.AFS
   scp SF33RD.AFS root@mister.home.arpa:/media/fat/games/RP6502/resources/SF33RD.AFS
   ```

4. **FPGA Core Bitstream**:
   The FPGA bitstream is now built from the tracked wrapper/core submodule instead of the old sibling checkout. No sync step is required.

   From `vendor/RP6502-mister-arm`:
   ```bash
   cd vendor/RP6502-mister-arm

   # 1) Put Quartus 17 tools on PATH
   export PATH="/home/rowe/intelFPGA_lite/17.0/quartus/bin:$PATH"

   # 2) Optional sanity check (shows whether local Quartus mode is available)
   tools/mister-wrapper/build-core.sh --check-env

   # 3) Build the core (default seed is menu)
   tools/mister-wrapper/build-core.sh --seed menu
   ```

   Output artifact:
   - `vendor/RP6502-mister-arm/build/mister-wrapper-core/RP6502_YYYYMMDD.rbf` from the `rp6502` repo root

   Deploy the generated RBF to:
   - `/media/fat/_Other/` (keep the dated filename, e.g. `RP6502_YYYYMMDD.rbf`)

   Example deploy:
   ```bash
   scp vendor/RP6502-mister-arm/build/mister-wrapper-core/RP6502_*.rbf root@mister.home.arpa:/media/fat/_Other/
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
The emulator cycle tick loop is highly cycle-accurate, causing the single-core CPU usage on the dual-core Cortex-A9 to run flat out. Timings are output to `/media/fat/games/RP6502/logs/last-run.log` every second to measure frame performance (e.g. `cpu`, `vga` render, `video` DDR3 copy, `audio` synth).

### OSD ROM Selection
The RP6502 OSD includes a `Select ROM` file picker entry that accepts both `.rp6502` and `.bin` files.

The selected path is saved in MiSTer config storage (`/media/fat/config/RP6502.f1`) and is applied by the wrapper on each launch/restart. If no selection exists, if the selected file is missing, or if the extension is unsupported, the wrapper falls back to the emulator's default ROM behavior.

### OSD PHI2 Clock Selection
The RP6502 OSD includes a `PHI2 (kHz)` option with `2000`, `4000`, and `8000` values. The wrapper passes this value to `rp6502-emu` as `--phi2 <khz>` on launch.

PHI2 changes are restart-applied: selecting a new PHI2 value updates the pending launch setting, and the new clock takes effect after `Restart`.

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
