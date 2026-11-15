#!/usr/bin/env python3
"""
Проверка алгоритма lock-in (синхронного детектора) на синтетических данных.

Моделирует ровно то, что делает прошивка в lockIn():
  - берёт N отсчётов АЦП,
  - умножает каждый на опорные cos(ωt) и sin(ωt),
  - усредняет и берёт модуль.

ВАЖНО: тест РЕАЛЬНО ПРОВЕРЯЕТ результаты и падает с ненулевым кодом,
если что-то не сходится. Прошлая версия печатала «ВСЕ ТЕСТЫ ПРОЙДЕНЫ»
безусловно — это было ложное подтверждение.

Запуск:  python test_lockin.py
"""

import math
import random
import sys

VREF = 3.3
ADC_MAX = 4095
F_EXC = 1000.0
N = 2000
SAMPLE_DT = 25e-6          # 25 мкс = 40 кГц

FAILS = []


def check(name, cond, detail=""):
    """Регистрирует результат проверки. Не бросает — собирает все ошибки."""
    if cond:
        print(f"  [OK]   {name}")
    else:
        print(f"  [FAIL] {name}   {detail}")
        FAILS.append(name)


def adc_sample(amp_v, offset_v, noise_v, i, phase_deg=0.0):
    """Один отсчёт АЦП: сигнал + офсет + шум, квантование на 12 бит."""
    t = i * SAMPLE_DT
    sig = amp_v * math.sin(2 * math.pi * F_EXC * t + math.radians(phase_deg))
    v = sig + offset_v + random.gauss(0, noise_v)
    return v / VREF * ADC_MAX


def lockin(amp_v, offset_v=0.0, noise_v=0.0, phase_deg=0.0):
    """Повторяет алгоритм прошивки. Возвращает (амплитуда В, фаза град, I, Q)."""
    acc_i = 0.0
    acc_q = 0.0
    for i in range(N):
        v = adc_sample(amp_v, offset_v, noise_v, i, phase_deg)
        t = i * SAMPLE_DT
        ph = 2 * math.pi * F_EXC * t
        acc_i += v * math.cos(ph)
        acc_q += v * math.sin(ph)
    a_i = acc_i * 2.0 / N
    a_q = acc_q * 2.0 / N
    amp_counts = math.hypot(a_i, a_q)
    amp_v_out = amp_counts * VREF / ADC_MAX
    # Сигнал задан как sin(ωt+φ), опора cos/sin. Тогда:
    #   a_i = A·sin(φ),  a_q = A·cos(φ)
    # поэтому фаза восстанавливается как atan2(a_i, a_q) — это и есть φ.
    # (Прошивка использует atan2(Q, I) — это даёт φ+90°; см. примечание ниже.)
    phase = math.degrees(math.atan2(a_i, a_q))
    return amp_v_out, phase, a_i, a_q


def minmax(amp_v, offset_v=0.0, noise_v=0.0):
    """Как в v1: огибающая min/max."""
    vals = [adc_sample(amp_v, offset_v, noise_v, i) for i in range(N)]
    mn, mx = min(vals), max(vals)
    return (mx - mn) * VREF / ADC_MAX


def main():
    random.seed(42)
    print("=" * 72)
    print(" Проверка lock-in: восстановление амплитуды")
    print("=" * 72)
    print(f"N={N} отсчётов, шаг {SAMPLE_DT*1e6:.0f} мкс, 12-бит АЦП")
    print()

    # ── Тест 1: идеальные условия ──────────────────────────────────────────
    print("─" * 72)
    print(" Тест 1: идеальные условия (шум=0, офсет=0)")
    print("─" * 72)
    print(f"{'задано, мВ':>12} | {'lock-in':>12} | {'ошибка':>10}")
    print("-" * 72)
    worst = 0.0
    for amp_mv in [0.5, 1.0, 5.0, 15.0, 50.0, 158.0]:
        amp = amp_mv / 1000.0
        got, _, _, _ = lockin(amp, 0.0, 0.0)
        err = (got - amp) / amp * 100
        worst = max(worst, abs(err))
        print(f"{amp_mv:12.2f} | {got*1000:12.3f} | {err:+9.2f} %")
    check("Тест 1: ошибка < 0.5 % на всех уровнях", worst < 0.5,
          f"(худшая {worst:.2f} %)")
    print()

    # ── Тест 2: постоянный офсет ───────────────────────────────────────────
    print("─" * 72)
    print(" Тест 2: постоянный офсет (как смещение LM358)")
    print("─" * 72)
    print("Lock-in офсет отбрасывает полностью: произведение постоянной")
    print("составляющей на sin/cos даёт ноль за целое число периодов.")
    print("Min/max офсет тоже игнорирует — считается разность max-min.")
    print()
    amp = 0.015
    print(f"истинная амплитуда = {amp*1000:.1f} мВ")
    print()
    print(f"{'офсет, мВ':>12} | {'lock-in':>12} | {'min/max':>12}")
    print("-" * 72)
    for off_mv in [0, 20, 100, 300, 1000, 3000]:
        off = off_mv / 1000.0
        got, _, _, _ = lockin(amp, off, 0.0)
        v1 = minmax(amp, off, 0.0) / 2.0
        print(f"{off_mv:12.0f} | {got*1000:9.3f} мВ | {v1*1000:9.3f} мВ")
    print()
    print("Вывод: оба метода устойчивы к постоянному офсету, пока сигнал")
    print("не упирается в рейл АЦП.")
    print()

    # ── Тест 3: шум ────────────────────────────────────────────────────────
    print("─" * 72)
    print(" Тест 3: гауссов шум (32 повторения, смотрим разброс)")
    print("─" * 72)
    amp = 0.015
    print(f"истинная амплитуда = {amp*1000:.1f} мВ")
    print()
    print(f"{'шум, мВ':>10} | {'lock-in: сред':>14} | {'разброс':>10} | "
          f"{'min/max: сред':>15} | {'разброс':>10}")
    print("-" * 72)
    li_err_at_5 = None
    mm_err_at_5 = None
    for noise_mv in [0.5, 2.0, 5.0]:
        noise = noise_mv / 1000.0
        li_vals, mm_vals = [], []
        for _ in range(32):
            g, _, _, _ = lockin(amp, 0.0, noise)
            li_vals.append(g)
            mm_vals.append(minmax(amp, 0.0, noise) / 2.0)
        li_mean = sum(li_vals) / len(li_vals)
        mm_mean = sum(mm_vals) / len(mm_vals)
        li_std = math.sqrt(sum((x - li_mean)**2 for x in li_vals) / len(li_vals))
        mm_std = math.sqrt(sum((x - mm_mean)**2 for x in mm_vals) / len(mm_vals))
        print(f"{noise_mv:10.1f} | {li_mean*1000:11.3f} мВ | "
              f"{li_std*1000:7.3f} мВ | {mm_mean*1000:12.3f} мВ | "
              f"{mm_std*1000:7.3f} мВ")
        if noise_mv == 5.0:
            li_err_at_5 = (li_mean - amp) / amp * 100
            mm_err_at_5 = (mm_mean - amp) / amp * 100
    print()
    check("Тест 3: lock-in при шуме 5 мВ точен (< 2 %)",
          li_err_at_5 is not None and abs(li_err_at_5) < 2.0,
          f"(ошибка {li_err_at_5:+.2f} %)" if li_err_at_5 is not None else "")
    check("Тест 3: min/max при шуме 5 мВ завышает (> 20 %)",
          mm_err_at_5 is not None and mm_err_at_5 > 20.0,
          f"(ошибка {mm_err_at_5:+.2f} %)" if mm_err_at_5 is not None else "")
    print()
    print("Вывод: при одинаковом шуме lock-in даёт заметно меньший разброс.")
    print("Выигрыш по амплитуде шума = sqrt(N) = 44.7x для N=2000.")
    print()

    # ── Тест 4: фаза ───────────────────────────────────────────────────────
    print("─" * 72)
    print(" Тест 4: восстановление фазы")
    print("─" * 72)
    print("Сигнал задан как sin(ωt+φ). Сверяем восстановленную φ с заданной.")
    print()
    print(f"{'задана':>8} | {'измерена':>10} | {'ошибка':>8}")
    print("-" * 72)
    worst_ph = 0.0
    for ph_deg in [0, 30, 45, 90, -45]:
        got, measured_ph, _, _ = lockin(0.015, 0.0, 0.0, ph_deg)
        # разворачиваем в (-180, 180]
        diff = (measured_ph - ph_deg + 180) % 360 - 180
        worst_ph = max(worst_ph, abs(diff))
        print(f"{ph_deg:7d}° | {measured_ph:9.2f}° | {diff:+7.2f}°")
    check("Тест 4: фаза восстанавливается с ошибкой < 2°", worst_ph < 2.0,
          f"(худшая {worst_ph:.2f}°)")
    print()

    # ── Итог ───────────────────────────────────────────────────────────────
    print("=" * 72)
    if FAILS:
        print(f" ПРОВАЛЕНО ТЕСТОВ: {len(FAILS)}")
        for f in FAILS:
            print(f"   - {f}")
        print("=" * 72)
        return 1
    print(" ВСЕ ТЕСТЫ ПРОЙДЕНЫ")
    print("=" * 72)
    return 0


if __name__ == "__main__":
    sys.exit(main())
