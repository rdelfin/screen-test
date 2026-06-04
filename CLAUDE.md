# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**screen-test** is an embedded Rust firmware project for the ESP32-S3 microcontroller that drives an 800×480 E-Ink Spectra6 display. The project uses the ESP HAL for hardware abstraction and Embassy for async task management.

### Technology Stack

- **MCU**: ESP32-S3 (Xtensa architecture)
- **Toolchain**: Rust nightly (esp channel)
- **Hardware Abstraction**: esp-hal v1.0
- **RTOS**: esp-rtos with Embassy executor
- **Wireless**: Wi-Fi and Bluetooth Low Energy (via esp-radio)
- **Display Driver**: Custom eink-spectra6-driver crate
- **Logging**: defmt + RTT (Real-Time Transfer)

## Project Structure

The workspace contains two members:

1. **screen-test** (root binary crate)
   - `/src/bin/main.rs` - Entry point with Embassy async runtime
   - `/src/lib.rs` - Minimal lib for testing infrastructure
   - `/build.rs` - Linker configuration and error handling helpers

2. **eink-spectra6-driver** (library crate)
   - `/eink-spectra6-driver/src/lib.rs` - E-Ink display controller
   - Implements the EpaperPort interface for SPI communication with the display

## Build System

### Prerequisites

Install the ESP Rust toolchain:

```bash
rustup install esp
```

### Build Commands

**Build for debug (optimized with `opt-level = "s"`)**:
```bash
cargo build
```

**Build for release (aggressive optimizations)**:
```bash
cargo build --release
```

**Build only the driver library**:
```bash
cargo build -p eink-spectra6-driver
```

### Configuration

- **Target**: `xtensa-esp32s3-none-elf` (configured in `.cargo/config.toml`)
- **Custom rustflags**: Enable stack protection and link custom linker scripts
- **Build std**: Uses unstable feature to rebuild `alloc` and `core` for the target

## Running and Flashing

### Flash the Firmware

The project uses **probe-rs** as the default runner (configured in `.cargo/config.toml`):

```bash
cargo run --release
```

This compiles the binary and flashes it to the connected ESP32-S3 via a debug probe. The runner:
- Uses `--chip=esp32s3` to identify the target
- Enables `--preverify` to verify before flashing
- Captures hard faults with `--catch-hardfault`
- Disables location info with `--no-location` for binary size reduction

### Serial Communication / Logging

The firmware uses **defmt** for structured logging over RTT (Real-Time Transfer). Output appears in the console after `cargo run`:

```
Embassy initialized!
EpaperPort initialized!
Displayed checkerboard!
Hello world!
...
```

Set the log level via the `DEFMT_LOG` environment variable (default: "info" per `.cargo/config.toml`):
```bash
DEFMT_LOG=debug cargo run --release
```

## Testing

Run tests on the device:

```bash
cargo test --lib
```

Tests use the `embedded-test` crate with xtensa-semihosting. The project includes a basic sanity test in both the root lib and the driver library.

## Code Architecture

### Main Application Flow

**Entry Point** (`src/bin/main.rs`):
1. Initializes the ESP32-S3 MCU with maximum CPU clock
2. Sets up two heap allocators (total ~100 KB) for normal and COEX-reserved memory
3. Starts the Embassy executor with a timer group
4. Initializes the E-Ink display via `EpaperPort`
5. Initializes radio (Wi-Fi and BLE) using esp-radio
6. Sets up the Trouble BLE stack for GATT services
7. Enters an async event loop logging "Hello world!" every second

### Display Driver Architecture

**EpaperPort** (`eink-spectra6-driver/src/lib.rs`):

The driver abstracts communication with the Spectra6 display via a 4-wire SPI bus plus control lines:
- **SPI3**: Data transmission (40 MHz, mode 0)
- **DC (Data/Command)**: GPIO toggled to select register vs. data mode
- **RST (Reset)**: GPIO for display reset sequence
- **BUSY**: Input pin to poll display readiness (HIGH = ready, LOW = processing)

**Key Methods**:
- `new()` - Initializes SPI, GPIO pins, and runs display boot sequence
- `send_command()` / `send_data()` / `send_data_buf()` - Low-level register access
- `wait_busy()` - Busy-wait loop for display readiness
- `reset()` - Hardware reset with 50ms delays
- `init()` - Sends 12+ initialization commands specific to Spectra6
- `display_sample()` - Renders a Mexican flag (demonstrates color palette and text rendering)
- `display_checkerboard()` - Renders multicolor diagonal stripes (exercises all 6 colors)
- `display()` - Stub for general-purpose image rendering

**Color Palette** (4-bit per pixel):
- 0x0 = Black
- 0x1 = White
- 0x2 = Yellow
- 0x3 = Red
- 0x5 = Blue
- 0x6 = Green

Display resolution is 800×480. Data is encoded at 4 bits per pixel with 2 pixels packed per byte.

## Linting and Code Quality

Check for common issues:

```bash
cargo clippy
```

Configuration in `.clippy.toml`:
- Stack size threshold: 1024 bytes (warns on stack frames exceeding this)

The main crate enforces two lint policies:
- Deny `clippy::mem_forget` (unsafe with ESP types holding buffers)
- Deny `clippy::large_stack_frames` (except in main)

## Development Workflow

### Common Tasks

1. **Modify display rendering**: Edit `eink-spectra6-driver/src/lib.rs` methods like `display_sample()` or implement `display()` for custom graphics
2. **Add async tasks**: Spawn new tasks in main.rs using the Embassy executor
3. **Modify hardware pins**: Change GPIO assignments in `main.rs` EpaperPort instantiation
4. **Adjust radio/BLE stack**: Modify esp-radio features and Trouble stack configuration in main.rs
5. **Change SPI frequency or mode**: Update `Config::default()` in the driver's `new()` method

### Debugging

- Logs appear via RTT; adjust `DEFMT_LOG` for verbosity
- The build script provides helpful hints for linker errors (e.g., missing `defmt.x`, `esp_rtos` issues)
- Use `--always-print-stacktrace` (enabled by default in cargo config) to see crashes in detail

### Workspace Management

The root crate (`screen-test`) depends on the driver as a local path dependency:
```toml
eink-spectra6-driver = { path = "eink-spectra6-driver" }
```

Changes to the driver are automatically rebuilt when building the root crate.

## E-Ink Display Reference

### Hardware

The display is a **Waveshare 7.3inch e-Paper HAT (E)** — an E Ink Spectra 6 (E6) full-colour panel:

- Resolution: 800 × 480 pixels
- Colours: 6 (Black, White, Yellow, Red, Blue, Green) — ACeP pigment technology
- Interface: 4-wire SPI (Mode 0, CPOL=0, CPHA=0)
- Refresh time: ~15–30 seconds for a full ACeP colour refresh (normal); no partial refresh support
- Data encoding: 4 bits per pixel, 2 pixels packed per byte (high nibble = left/even pixel, low nibble = right/odd pixel)
- Total pixel data per frame: 800/2 × 480 = **192,000 bytes**

### Documentation Links

- [7.3inch e-Paper HAT (E) Wiki](https://www.waveshare.com/wiki/7.3inch_e-Paper_HAT_(E))
- [7.3inch e-Paper HAT (E) Manual](https://www.waveshare.com/wiki/7.3inch_e-Paper_HAT_(E)_Manual)
- [7.3inch e-Paper (F) Application Note PDF](https://files.waveshare.com/upload/8/86/7.3inch_e-Paper_(F)_Application_Note_Reference.pdf) — closest public reference for the SPI register map and init sequence (the (E) Spectra6 variant shares the same controller architecture)
- [7.3inch e-Paper HAT (F) Wiki](https://www.waveshare.com/wiki/7.3inch_e-Paper_HAT_(F)) — ACeP 7-color predecessor; init sequences are nearly identical

### SPI Protocol

CS is managed in **software** (not hardware SPI CS). The DC pin selects register vs. data mode:

| Signal | Meaning |
|--------|---------|
| DC LOW + CS pulse | Command byte |
| DC HIGH + CS pulse | Single data byte |
| DC HIGH + CS held LOW | Streaming pixel data (DTM mode) |

**Data Transfer Mode (DTM)**: opened by command `0x10`. CS must stay LOW for the entire 192,000-byte pixel stream. Releasing CS mid-stream resets the controller's internal byte counter and corrupts the frame.

### Key Register Commands

| Command | Name | Notes |
|---------|------|-------|
| `0xAA` | Sequence label | Must be sent first with 6 magic bytes `[0x49,0x55,0x20,0x08,0x09,0x18]` |
| `0x01` | Power setting | `0x3F` |
| `0x00` | Panel setting | `[0x5F, 0x69]` |
| `0x03` | PLL control | `[0x00, 0x54, 0x00, 0x44]` |
| `0x04` | Power on | Assert after init; also assert before every refresh |
| `0x05` | Booster soft-start (phase A/B) | `[0x40, 0x1F, 0x1F, 0x2C]` |
| `0x06` | Booster soft-start (phase C) | **Must be re-sent before every `0x12` refresh** — `[0x6F, 0x1F, 0x17, 0x49]` |
| `0x08` | Booster soft-start (phase D) | `[0x6F, 0x1F, 0x1F, 0x22]` |
| `0x10` | Data Transfer Mode (DTM) | Opens pixel data stream |
| `0x12` | Display refresh | Triggers the ACeP full refresh cycle |
| `0x02` | Power off | Send after refresh completes; wait BUSY before next init |
| `0x30` | OSC setting | `0x03` |
| `0x50` | VCOM setting | `0x3F` |
| `0x60` | T_vcom/T_dis setting | `[0x02, 0x00]` |
| `0x61` | Resolution | `[0x03, 0x20, 0x01, 0xE0]` = 800×480 |
| `0x84` | Unknown | `0x01` (must NOT be `0x00`) |
| `0xE3` | Power saving | `0x2F` |

### BUSY Pin

- **HIGH** = display idle / ready to accept commands
- **LOW** = display busy processing

Poll with 10 ms delay between reads. Long BUSY periods (~15–30 s) are normal during the ACeP refresh cycle after `0x12`. If BUSY never goes HIGH after `0x12`, the most likely cause is missing `0x06` (booster not re-armed).

### Refresh Power State Machine

The controller tracks whether the panel is powered on. Key invariants:

1. `0x04` (power on) is a **no-op if already powered**; it only asserts BUSY when transitioning from powered-off.
2. `turn_on_display()` must call `0x04` → wait BUSY → `0x06` → `0x12` → wait BUSY → `0x02` → wait BUSY, in that order.
3. After `0x02` (power off), the booster state is reset; `0x06` **must** be re-sent before the next `0x12`.
4. On first boot (especially after extended power-off), send a full white-fill frame immediately after `init()` to put the panel in a known state — the driver does this in `new()` via `clear()`.

### Color Palette

| Code | Color  |
|------|--------|
| `0x0` | Black  |
| `0x1` | White  |
| `0x2` | Yellow |
| `0x3` | Red    |
| `0x5` | Blue   |
| `0x6` | Green  |

Note: code `0x4` and `0x7` are unused/undefined on this panel.

## Dependencies of Note

- **esp-hal**: Low-level hardware abstraction; features include SPI, GPIO, timers, clock control
- **esp-rtos**: RTOS scheduler integration; enables Embassy on ESP32-S3
- **embassy-executor**: Async runtime
- **esp-radio**: Wi-Fi and BLE radio abstractions
- **trouble-host**: BLE stack (GATT support)
- **defmt**: Structured logging macros
- **panic-rtt-target**: RTT-based panic handler
- **anyhow/thiserror**: Error handling in driver

## Cargo.lock

The project uses `Cargo.lock` (checked in). This ensures reproducible builds across environments, especially important for embedded projects where toolchain versions matter.
