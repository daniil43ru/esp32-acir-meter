---
name: Hardware measurement report
about: You built this and measured something — tell us the numbers
title: "[HW] Measured values on <board>"
labels: hardware
---

**This is the most valuable contribution to this repo right now.**
The design has never been verified on a breadboard, so any real measurement
is new information.

## Setup

- ESP32 board / chip revision:
- LM358 (or alternative part):
- Cell / resistor under test:
- Reference instrument used (if any):
- Firmware commit:

## Measured values

Paste the Serial output (`115200` baud) here. The firmware prints one line per
second:

```
U=... V | I=... mA | sig=... mV | off=... mV | ph=... | Ri=... mOhm | K=... | sat=0
```

```
(paste here)
```

## Against the model

`tools/model_check.py` predicts:

| Ri | U at cell | LM358 output | ADC counts |
|---|---|---|---|
| 20 mΩ | 0.344 mV | 16.5 mV | 20.5 |
| 50 mΩ | 0.861 mV | 41.3 mV | 51.3 |
| 80 mΩ | 1.377 mV | 66.1 mV | 82.0 |
| 200 mΩ | 3.443 mV | 165.3 mV | 205.1 |

## Did anything differ from the prediction?

- [ ] Values matched within 5 %
- [ ] Values off by 5–20 %
- [ ] Values off by more than 20 %
- [ ] The meter read 0 / garbage

## Other observations

Any of these are useful:

- How long does `analogRead()` actually take on your board? (add `micros()`
  around it and print)
- What is the real GPIO output resistance? (load the pin, measure the droop)
- Does the phase reading stay near 0° for a resistive load? A large phase
  indicates capacitance in the cabling.
- Did `K_CAL` need a large correction? A `K_CAL` far from 1.0 means the
  component tolerances are significant.
