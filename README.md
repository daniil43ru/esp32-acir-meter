# ESP32 ACIR Meter — internal resistance of 18650 cells

An **AC internal-resistance (IR) meter** for lithium cells (18650, 21700 and
similar), built on an ESP32 with a Kelvin (4-wire) sense arrangement.

> **Project status: prototype, NOT verified on hardware.**
> The firmware compiles and the signal-processing math is covered by unit tests,
> but the circuit has never been assembled on a breadboard by the author.
> The schematics below are a best-effort design that a reviewer should
> independently check. See [What still needs doing](#what-still-needs-doing).

---

## Why

Internal resistance is the main indicator of a cell's health. A fresh 18650 sits
at 15–25 mΩ; a worn one at 60–150 mΩ. Capacity testers rarely report it, but IR
is what decides whether a cell can deliver high current without sagging.

Most DIY IR meters use a *load-pulse* method: apply a current step, watch the
voltage sag, divide. That includes the resistance of the probes and contacts,
which for an 18650 (5–20 mΩ contact) is comparable to the quantity being
measured — a 50 % error is typical.

This project uses the **Kelvin (4-wire) method** instead: the excitation current
runs through one pair of wires, while the voltage is sensed through a second
pair that carries no current. Contact and lead resistance drops out physically.

## How it works

```
ESP32 GPIO25 ── 1 kHz square ── Cdc 10µF ── Rforce ── Rshunt ──┬── cell +
                                                               │
                                                             [ Ri ]
                                                               │
                                                        cell − ── GND

sense (no current in these wires):
   cell + ── Cin 1µF ── LM358 IN+ ── ×48 ── GPIO35
```

The 1 kHz AC excitation avoids polarising the cell, and a **synchronous
detector (lock-in)** in firmware extracts the first harmonic while rejecting
noise and DC offsets.

## Firmware highlights

| Feature | Why |
|---|---|
| Lock-in detection, 2000 samples | Rejects noise; the naive `min/max` envelope overestimates by up to +95 % at 5 mV noise |
| Offset removal in I/Q | Vector subtraction, not scalar |
| Median of 5 measurements | Removes impulsive interference |
| Calibration against a known resistor | Absorbs part tolerances via `K_CAL` |
| Diagnostics screen | Phase, signal level, saturation flag |
| OLED UI | Main / Voltmeter / Diagnostics / Calibrate screens |

## Numbers (from `tools/model_check.py`)

| Quantity | Value |
|---|---|
| Excitation, first harmonic | 2.10 V peak (4/π × 1.65) |
| Excitation current | **17.21 mA** peak |
| LM358 gain | 48 (1 + 47k/1k) |
| Resolution | **≈ 0.98 mΩ** per ADC LSB |
| Output at Ri = 50 mΩ | 41 mV peak at the op-amp output |
| Output at Ri = 200 mΩ | 165 mV peak — far from the 1.65 V rail |

## Build

```bash
arduino-cli core install esp32:esp32
arduino-cli lib install "Adafruit SSD1306" "Adafruit GFX Library" "Adafruit BusIO"
arduino-cli compile --fqbn esp32:esp32:esp32 --output-dir build acir_kelvin
```

The sketch file must be named `acir_kelvin.ino` to match the folder name —
`arduino-cli` refuses to build otherwise.

## Verify the math (no hardware needed)

```bash
python tools/model_check.py    # signal chain, resolution, calibration effect
python tools/test_lockin.py    # asserts on the lock-in algorithm, exits 1 on failure
python tools/test_settling.py  # op-amp settling time vs measurement window
python tools/test_timing.py    # effect of loop overrun on the detector
```

`test_lockin.py` really asserts — it returns a non-zero exit code when a check
fails. (An earlier version printed "all tests passed" unconditionally, which was
a false confirmation; that is fixed.)

## Wiring notes

- **`Cdc` must be non-polarised** (ceramic or film). An electrolytic will see
  reverse bias.
- Power and sense wires to the cell must be **separate pairs**, joined only at
  the cell terminal.
- The ESP32 ADC input must not exceed 3.3 V.

## Known limitations

1. **The firmware's over-voltage check is an indication, not protection.**
   With a 1:2 divider, 9 V at the terminals puts 4.5 V on GPIO34 — above the
   ESP32 absolute maximum of 3.6 V. The pin is damaged *before* firmware can
   react. **Add a 3.3 V Zener (or a Schottky clamp to 3.3 V) on GPIO34.**
2. **This is "3.5-wire", not 4-wire.** The cell's negative terminal returns
   directly to board GND, so the drop across its negative lead is still inside
   the measured loop (≈ 0.8 mV at 17 mA over 50 mΩ). Calibration compensates
   it only because the contact resistance is similar both times.
3. **GPIO output resistance is assumed, not measured.** `R_GPIO = 20 Ω` is a
   datasheet typical (12–33 Ω). Calibration absorbs it, but the current figure
   shown on screen can be off by ±10 % until you measure your own board.
4. **LM358 is a poor choice for this.** Its input offset (2–7 mV) and noise
   floor are at the edge for milliohm work, and the input common-mode range is
   only 150 mV below the rails. A MCP6002 or an ADS1115 would be better.
5. **The Wokwi simulator cannot validate this circuit.** Wokwi treats an MCU
   output as a DC level, so the coupling capacitor blocks everything and the
   meter reads 0 mΩ.

## What still needs doing

Help is very welcome, especially from anyone who can run the hardware.

- [ ] **Assemble on a breadboard and report measured values.** This is the
      single most valuable contribution — everything downstream depends on it.
- [ ] **Measure the real duration of `analogRead()`** on a 240 MHz ESP32.
      The sampling loop assumes a 25 µs slot; if `analogRead` takes longer the
      detector still works (amplitude is a hypotenuse of I/Q, so a constant
      phase shift cancels) but the effective sample rate changes.
- [ ] **Measure GPIO output resistance** on your board and set `R_GPIO`.
- [ ] **Compare readings against a reference instrument** (HIOKI BT3561,
      RC3563, or a good milliohm meter) across 5–10 known cells.
- [ ] **Add the hardware over-voltage clamp** to the schematic.
- [ ] **Persist `K_CAL` in NVS** (Preferences) so calibration survives reboot.
- [ ] **Improve the sense return** to a true 4-wire arrangement (sense− to the
      cell's negative post rather than board GND).
- [ ] **Replace the LM358** with a lower-noise, rail-to-rail input op-amp.
- [ ] **Evaluate sine excitation** (PWM + low-pass filter) instead of a square
      wave, to remove the first-harmonic correction entirely.

## Prior art reading

- [opengreenenergy/DIY-Smart-Multipurpose-Battery-Tester](https://github.com/opengreenenergy/DIY-Smart-Multipurpose-Battery-Tester) — ESP32, capacity + IR, load-pulse method
- [ArminJo/Ultimate-Battery-Tester](https://github.com/ArminJo/Ultimate-Battery-Tester) — ESR measurement with a discharge graph
- [ludwich66/RC3563](https://github.com/ludwich66/RC3563) — software for a commercial milliohm meter, a useful reference target
- [NuclearKev/4-wire](https://github.com/NuclearKev/4-wire) — a 4-wire Kelvin measurement lab

## License

MIT — see `LICENSE`.
