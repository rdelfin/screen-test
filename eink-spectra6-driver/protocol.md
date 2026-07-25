# SPD1656 / Waveshare 7.3″ e-Paper HAT (E) — SPI Protocol Reference

Controller IC: **Solomon Systech SPD1656** (ACeP variant for the Spectra 6 panel).  
Source: SPD1656 v1.1 datasheet (Solomon Systech, Jan 2020) + Waveshare 7in3e reference driver.

---

## SPI Bus

- Mode 0 (CPOL=0, CPHA=0), MSB first, up to 40 MHz.
- CS is managed in **software** — the hardware CS line is not used.
- DC pin selects register vs. data mode:

| DC | CS | Meaning |
|----|----|---------|
| LOW  | pulsed LOW→HIGH per byte | Command byte |
| HIGH | pulsed LOW→HIGH per byte | Single data byte |
| HIGH | held LOW for entire stream | Streaming pixel data (DTM mode) |

---

## Frame Buffer

- 800 × 480 pixels, 4 bits per pixel.
- Two pixels packed per byte: high nibble = even (left) pixel, low nibble = odd (right) pixel.
- Total per frame: 800 / 2 × 480 = **192,000 bytes**.
- Opened by command `0x10` (DTM). CS must stay LOW for the entire 192,000-byte stream —
  releasing CS mid-stream resets the controller's internal byte counter and corrupts the frame.

---

## Color Palette (4-bit codes)

| Code | Color  |
|------|--------|
| `0x0` | Black  |
| `0x1` | White  |
| `0x2` | Yellow |
| `0x3` | Red    |
| `0x5` | Blue   |
| `0x6` | Green  |

Codes `0x4` and `0x7` are undefined on this panel.

---

## Register Reference

### `0xAA` CMDH — Command Header / Unlock

**Data:** `[0x49, 0x55, 0x20, 0x08, 0x09, 0x18]`

Fixed magic signature that must be the first command after every reset. It unlocks the
register interface — without it the controller ignores subsequent configuration writes.

---

### `0x01` PWR — Power Regulator

**Data:** `0x3F`

| Bits | Field    | Value | Meaning |
|------|----------|-------|---------|
| A[3] | VCM_HZ   | 1     | VCOM floating (high-Z) during startup; becomes active after PON |
| A[2] | VS_EN    | 1     | Internal DC-DC generates source voltages VSH / VSL |
| A[1] | VSC_EN   | 1     | Internal DC-DC generates source LV voltages VSH_LV / VSL_LV |
| A[0] | VG_EN    | 1     | Internal DC-DC generates gate voltages VGH / VGL |

The full PWR register supports up to 5 bytes (A–E) for precise voltage selection:
- B[1:0] VG_LVL: gate voltage (VGH 17–20 V, default 19 V)
- C[5:0] VSHC_LVL: VSH_LV positive voltage (3.0–15.0 V in 0.2 V steps, default 4.0 V)
- D[5:0] VSLC_LVL: VSL_LV negative voltage (−3.0 to −15.0 V, default −4.0 V)
- E[5:0] VSLC_LVL2: optional VSL_LV2 voltage; E[7] enables the regulator

Sending only byte A leaves B–E at their power-on defaults.

---

### `0x00` PSR — Panel Setting Register

**Data:** `[0x53, 0x69]`

**Byte A = `0x53` = `0b01010011`:**

| Bits   | Field   | Value | Meaning |
|--------|---------|-------|---------|
| A[7:6] | RES[1:0]| `01`  | Resolution hint (overridden by TRES `0x61`; actual 800×480) |
| A[5]   | RESA    | `0`   | Resolution sub-select |
| A[4]   | —       | `1`   | Reserved |
| A[3]   | UD      | `0`   | Gate scan direction: up (G0 → Gn-1) |
| A[2]   | SHL     | `0`   | Source shift direction: right (S0 → Sn-1) |
| A[1]   | SHD_N   | `1`   | Booster and regulator ON |
| A[0]   | RST_N   | `1`   | Normal operation (not in soft reset) |

An interesting note is that you can change bytes A3 and A2 to change the orientation of the
display. Flipping the value of `UD` and `SHL` has the practical effect of flipping the image 180
degrees.

**Byte B = `0x69` = `0b01101001`:**

| Bits | Field      | Value | Meaning |
|------|------------|-------|---------|
| B[7] | LUT_SEL    | `0`   | Load waveform LUT from OTP flash (not register) |
| B[2] | VG_OFF     | `0`   | Gate rails on power-off: VGH=VDD, VGL=0 V (not floating) |
| B[1] | VCOM_OFF   | `0`   | VCOM on power-off: 0 V (not floating) |
| B[0] | VS_OFF     | `1`   | Source rails on power-off: floating |

B[6:3] are extended bits specific to the Spectra6 variant and are not in the public SPD1656 spec.

---

### `0x03` PFS — Power-Off Sequence Setting

**Data:** `[0x00, 0x54, 0x00, 0x44]`

Controls the order and frame-count delay in which the panel voltages are lowered during POF (`0x02`).

**Byte A = `0x00`:**

| Bits   | Field        | Value | Meaning |
|--------|--------------|-------|---------|
| A[5:4] | T_VDS_OFF    | `00`  | Base timing unit = 1 frame |

**Byte B = `0x54` = `0b01010100`:**

| Bits   | Field      | Value | Meaning |
|--------|------------|-------|---------|
| B[5:4] | VG_OFF     | `01`  | Gate power (VGH/VGL) off after 1 × T_VDS_OFF (1 frame) |
| B[3:2] | VCOM_OFF   | `01`  | VCOM off after 1 × T_VDS_OFF (1 frame) |
| B[1:0] | VS_OFF     | `00`  | Source power off at 0 × T_VDS_OFF (immediately) |

Bytes C (`0x00`) and D (`0x44`) are extended fields for the additional voltage rails
(VSH_LV, VSL_LV) required by the 6-color ACeP panel; they are not in the public spec.

---

### `0x04` PON — Power On

No data bytes. Enables VGH, VGL, VSH, VSL, VCOM. Asserts BUSY (LOW) while ramping;
waits for BUSY to return HIGH before sending further commands.
No-op if the panel is already powered on — BUSY is not asserted in that case.

---

### `0x05` BTST-A / `0x06` BTST-B / `0x08` BTST-C — Booster Soft Start

The public SPD1656 BTST register encodes three booster phases (A, B, C) in one command at
address `0x05`. The Spectra6 7in3e variant distributes the phases across three separate
command addresses to support the additional voltage rails required by 6-color ACeP.

`0x06` **must be re-sent before every `0x12` refresh** to re-arm the booster after POF.

Each byte in each command uses the same encoding:

| Bits  | Field           | Meaning |
|-------|-----------------|---------|
| [7:6] | Phase period    | `00`=10 ms, `01`=20 ms, `10`=30 ms, `11`=40 ms |
| [5:3] | Drive strength  | `010`=1, `011`=2, `100`=3, `101`=4, `110`=5, `111`=6 (strongest) |
| [2:0] | Min off-time    | `000`=0.26 µs, `001`=0.31, `010`=0.36, `011`=0.52, `100`=0.77, `101`=1.61, `110`=3.43, `111`=6.77 µs |

**`0x05`: `[0x40, 0x1F, 0x1F, 0x2C]`**

| Byte | Raw    | Period | Strength | Min Off |
|------|--------|--------|----------|---------|
| Ph A | `0x40` | 20 ms  | 1 (rsvd) | 0.26 µs |
| Ph B | `0x1F` | 10 ms  | 2        | 6.77 µs |
| Ph C | `0x1F` | 10 ms  | 2        | 6.77 µs |
| Ph D | `0x2C` | 10 ms  | 4        | 0.77 µs |

**`0x06`: `[0x6F, 0x1F, 0x17, 0x49]`**

| Byte | Raw    | Period | Strength | Min Off |
|------|--------|--------|----------|---------|
| Ph A | `0x6F` | 20 ms  | 5        | 6.77 µs |
| Ph B | `0x1F` | 10 ms  | 2        | 6.77 µs |
| Ph C | `0x17` | 10 ms  | 2        | 6.77 µs |
| Ph D | `0x49` | 20 ms  | 1        | 0.31 µs |

**`0x08`: `[0x6F, 0x1F, 0x1F, 0x22]`**

| Byte | Raw    | Period | Strength | Min Off |
|------|--------|--------|----------|---------|
| Ph A | `0x6F` | 20 ms  | 5        | 6.77 µs |
| Ph B | `0x1F` | 10 ms  | 2        | 6.77 µs |
| Ph C | `0x1F` | 10 ms  | 2        | 6.77 µs |
| Ph D | `0x22` | 10 ms  | 3        | 0.36 µs |

---

### `0x10` DTM — Data Transfer Mode

No data bytes for the command itself. Opens the pixel data stream: set DC HIGH,
hold CS LOW, then clock out all 192,000 bytes. Release CS when done.

---

### `0x12` DRF — Display Refresh

**Data:** `0x00`

Triggers the ACeP full-panel refresh cycle (~15–30 s). Asserts BUSY (LOW) for the entire
duration. If BUSY never goes HIGH after this command, the most likely cause is `0x06`
(booster re-arm) not having been sent before `0x12`.

---

### `0x02` POF — Power Off

**Data:** `0x00`

Shuts down VGH, VGL, VSH, VSL, VCOM in the sequence defined by PFS (`0x03`).
Asserts BUSY while ramping down; wait for BUSY HIGH before next command.
After POF the booster state is reset — `0x06` must be re-sent before the next `0x12`.

---

### `0x30` PLL — Oscillator / Frame Rate

**Data:** `0x03`

| Bits   | Field  | Value | Meaning |
|--------|--------|-------|---------|
| A[5:3] | M[2:0] | `000` | PLL M divider |
| A[2:0] | N[2:0] | `011` | PLL N divider |

The 6-bit value selects the internal frame rate from a lookup table. POR default `0x3C`
gives 50 Hz; `0x03` selects approximately 50 Hz as well (low end of the 25–50 Hz band).
ACeP panels do not need a high internal frame rate — the 15–30 s refresh is paced by
the BUSY pin, not the oscillator.

---

### `0x50` CDI — VCOM and Data Interval

**Data:** `0x3F` = `0b00111111`

| Bits   | Field     | Value  | Meaning |
|--------|-----------|--------|---------|
| A[7:5] | VBD[2:0]  | `001`  | Border pixel: Gray1 |
| A[3:0] | CDI[3:0]  | `0xF`  | VCOM/data interval = 2 Hsync periods (minimum; default is 10) |

Reducing CDI to minimum (2) shortens blanking between VCOM transitions and pixel data,
prioritising transition speed.

---

### `0x60` TCON — Timing Controller

**Data:** `[0x02, 0x00]`

**Byte A = `0x02` = `0b00000010`:**

| Bits   | Field     | Value  | Meaning |
|--------|-----------|--------|---------|
| A[7:4] | S2G[3:0]  | `0000` | Source-to-Gate non-overlap = 4 units × 500 ns = 2 µs |
| A[3:0] | G2S[3:0]  | `0010` | Gate-to-Source non-overlap = 12 units × 500 ns = 6 µs (default) |

Byte B (`0x00`) is an extended byte for the Spectra6 variant not in the public spec.

---

### `0x61` TRES — Resolution

**Data:** `[0x03, 0x20, 0x01, 0xE0]`

`0x0320` = 800 (horizontal source outputs), `0x01E0` = 480 (vertical gate outputs).
Overrides the resolution hint in PSR (`0x00`).

---

### `0x84` — Extended / T_VDCS

**Data:** `0x01`

Not in the public SPD1656 command table. The reverse-engineering gist for the ED2208-GCA
panel labels it **T_VDCS** (a VCOM-DC timing register). Value `0x01` enables the function;
`0x00` disables it and causes display malfunction. Likely arms VCOM-DC compensation
required by the 6-color ACeP panel before the first power-on.

---

### `0xE3` PWS — Power Saving

**Data:** `0x2F` = `0b00101111`

| Bits   | Field        | Value  | Meaning |
|--------|--------------|--------|---------|
| A[7:4] | VCOM_W[3:0]  | `0010` | VCOM power-saving window = 2 line periods around each voltage zero-crossing |
| A[3:0] | SD_W[3:0]    | `1111` | Source power-saving window = 15 × 500 ns = 7.5 µs |

When a VCOM or source output transitions through zero (positive↔negative), current draw
is reduced for the specified window. POR default is `0x00` (disabled).

---

## Refresh Power State Machine

```
boot / after POF
      │
      ▼
  send_sequence_label (0xAA)
  send_power_setting  (0x01)
  send_panel_setting  (0x00)
  send_pll_control    (0x03)
  send_booster_start_ab (0x05)
  send_booster_start_c  (0x06)
  send_booster_start_d  (0x08)
  send_osc_setting    (0x30)
  send_vcom_setting   (0x50)
  send_tvcom_tdis_setting (0x60)
  send_resolution     (0x61)
  send_reg84          (0x84)
  send_power_saving   (0xE3)
      │
      ▼
  send_power_on (0x04) ──► wait BUSY HIGH
      │
      ▼
  [write pixel data via begin_pixels / send_pixels_chunk / end_pixels]
      │
      ▼
  turn_on_display:
    send_power_on     (0x04) ──► wait BUSY HIGH  (no-op if already powered)
    send_booster_start_c (0x06)                  (re-arms booster after POF)
    send_display_refresh (0x12) ──► wait BUSY HIGH (~15-30 s)
    send_power_off    (0x02) ──► wait BUSY HIGH
      │
      └─── back to [write pixel data] for next frame
```

Key invariants:
1. `0x06` must be re-sent before every `0x12`; POF resets the booster state.
2. `0x04` is a no-op (no BUSY assertion) if the panel is already powered on.
3. On first boot, send a full white-fill frame immediately after init to put the
   panel in a known state (done automatically by `EpaperPort::new` via `clear()`).

---

## BUSY Pin

- **HIGH** = display idle, ready to accept commands
- **LOW**  = display busy (power transition, refresh in progress)

Poll with 10 ms delay between reads. Long BUSY periods (~15–30 s) are normal during
the ACeP refresh cycle triggered by `0x12`.
