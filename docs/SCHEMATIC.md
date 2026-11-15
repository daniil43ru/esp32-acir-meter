# Schematic and wiring

**Status: design only. Not verified on hardware. Please review.**

If you spot an error in this schematic, please open an issue — that is exactly
what this repository needs most.

---

## Parts list

| Ref | Value | Notes |
|---|---|---|
| U1 | LM358 | Dual op-amp, single supply 3.3 V. Only half A is used |
| ESP32 | ESP32 DevKit v4 | Built-in signal generator (GPIO25) and ADC |
| Display | SSD1306 OLED 128×64 | I²C, 0x3C, on GPIO21 (SDA) / GPIO22 (SCL) |
| Cdc | 10 µF **non-polarised** | Ceramic or film. Blocks DC through the cell |
| Cin | 1 µF | AC-couples the sense input |
| Rforce | 100 Ω | Sets the excitation current |
| Rshunt | 1 Ω | 1 % preferred; (currently not read by firmware) |
| Rf | 47 kΩ | Feedback resistor → gain 48 |
| Rg | 1 kΩ | Gain resistor. Crg below blocks DC at DC-gain = 1 |
| Crg | 10 µF | DC block. **τ = Rg·Crg = 10 ms** (drives settling time) |
| Rb1, Rb2 | 100 kΩ / 100 kΩ | Bias IN+ to 1.65 V |
| R1, R2 | 100 kΩ / 100 kΩ | 1:2 divider for cell voltage, to GPIO34 |
| D1 | **3.3 V Zener (recommended)** | Clamp on GPIO34. **Not in the current diagram — please add it** |
| Buttons | 2× pushbutton | MODE on GPIO4, CAL on GPIO26, to GND |

## Signal chain

```
                    ┌────────────────────── excitation (current path) ───────────────────────┐
                    │                                                                        │
  GPIO25 ──1 kHz──► Cdc ──► Rforce ──► Rshunt ──┬──► [ cell + ]
   (square,                100 Ω      1 Ω       │
    0–3.3 V)                                    │
                                                │         ┌──────┐
                                                └────────►│  Ri  │
                                                          └──────┘
                                                             │
                                              [ cell − ] ◄───┘
                                                    │
                                                   GND


                    ┌──────────────────── sense (no current) ──────────────────────────────────┐
                    │                                                                        │
  [ cell + ] ──► Cin ──► IN+ ─┐
                               ├──► LM358(A) ──► OUT ──► GPIO35
                    Rf 47k ◄───┤
                               │
                       IN− ────┘
                        │
                       Rg ──► Crg ──► GND

  Bias:  Rb1 (3.3 V) ──► IN+ ◄── Rb2 (GND)      → 1.65 V DC operating point
```

## Why the gain is 48 for AC but 1 for DC

`Crg` in series with `Rg`:
- **AC (1 kHz):** `|Z_C| = 1/(2π·1000·10µF) ≈ 15.9 Ω`, tiny vs `Rg = 1 kΩ`, so
  `|K| = 1 + 47k/1k ≈ 48` (phase error 0.9°).
- **DC:** `Crg` is an open circuit, so the `Rg` branch carries no current and the
  stage is a **voltage follower with gain 1**. That keeps the `Rf`/`Rg` divider
  from multiplying the LM358 input offset by 48.

## Settling

`τ = Rg × Crg = 1 kΩ × 10 µF = 10 ms`. The firmware waits `SETTLE_ON_MS = 50`
(5τ, 99.3 %) before measuring. Measuring earlier biases the result.

The input network also settles through `Rb1 ∥ Rb2 = 50 kΩ` loading `Cin = 1 µF`
→ `τ ≈ 50 ms`, so `setup()` waits 600 ms before the first reading.

## Excitation current

GPIO25 emits a **0–3.3 V square wave**, not a sine. The lock-in detector
extracts only the **first harmonic**, whose amplitude is

```
V_fund = (4/π) · (V_pp / 2) = (4/π) · 1.65 ≈ 2.10 V peak
```

With `Rforce + Rshunt + R_GPIO ≈ 121 Ω` and `|Z_C| ≈ 15.9 Ω`:

```
I ≈ 2.10 / √(121² + 15.9²) ≈ 17.2 mA peak
```

Using the square's peak (1.65 V) instead of its fundamental understates the
current by 21.5 % and overstates Ri by 27 %.

## Known schematic issues a contributor should look at

1. **No over-voltage clamp.** GPIO34 reaches 4.5 V with 9 V at the terminals.
   Add a Zener or Schottky clamp.
2. **Sense return is board GND, not the cell's negative post** → "3.5 wires".
   The negative lead's IR drop is included in the measurement.
3. **`Rshunt` is not measured**, only assumed. Using the LM358's spare half as a
   shunt amplifier would give a true current reading.
4. **`Cdc` polarity.** A non-polarised part is required; the DC bias across it
   equals `V_excitation_mean − V_cell ≈ 1.65 V − V_cell`, which is **negative**
   for a 4.2 V cell.
5. **LM358 input common-mode range** is `V+ − 1.5 V = 1.8 V`, so biasing IN+ at
   1.65 V leaves only 150 mV of margin. A rail-to-rail input op-amp would be
   safer.
