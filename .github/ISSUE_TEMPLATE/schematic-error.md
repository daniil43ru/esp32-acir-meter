---
name: Schematic error
about: Something in the circuit is wrong or unsafe
title: "[SCHEMA] "
labels: schematic
---

## What looks wrong

<!-- Component reference and what you believe is incorrect -->

## Evidence

<!-- Calculations, datasheet limits, simulation results, or a photo of the board -->

## Suggested fix

<!-- What the circuit should do instead -->

## Severity

- [ ] **Unsafe** — can damage the ESP32 or the cell
- [ ] **Wrong** — the measurement will be incorrect
- [ ] **Suboptimal** — works, but could be better

---

If the issue is that the circuit cannot work at all, please explain the
physical mechanism, not just "it won't work". Numbers help: what current flows,
what voltage appears at each node, against which datasheet limit.
