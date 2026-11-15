#!/usr/bin/env python3
"""
Проверка, насколько реальный тайминг loop-lock-in искажает результат.

В прошивке опора строится на ПРЕДПОЛАГАЕМОМ времени i*25 мкс, а реальный
момент отсчёта сдвинут на время analogRead (~15-20 мкс на ESP32 при 12 бит).
Предполагаем, что analogRead занимает ADC_US и выполняется ДО сохранения
отсчёта, а ожидание while досыпает остаток до границы 25 мкс.

Этот скрипт считает амплитуду и фазу для разных ADC_US и показывает, во
сколько раз падает результат (если сдвиг фазы произвольный — амплитуда
умножается на |cos(фазо-ошибка)|).

Запуск:  python test_timing.py
"""

import math

VREF = 3.3
ADC_MAX = 4095
F_EXC = 1000.0
N = 2000
STEP_US = 25.0


def simulate(adc_us, amp_v=0.015):
    """Считает то же, что прошивка, с учётом задержки analogRead."""
    acc_i = 0.0
    acc_q = 0.0
    for i in range(N):
        # реальный момент отсчёта: начало слота + задержка на оцифровку
        t_real = (i * STEP_US + adc_us) * 1e-6
        sig = amp_v * math.sin(2 * math.pi * F_EXC * t_real)

        # прошивка считает время как i*STEP, не зная про задержку
        t_assumed = (i * STEP_US) * 1e-6
        ph = 2 * math.pi * F_EXC * t_assumed

        v = sig / VREF * ADC_MAX
        acc_i += v * math.cos(ph)
        acc_q += v * math.sin(ph)

    a_i = acc_i * 2.0 / N
    a_q = acc_q * 2.0 / N
    amp = math.hypot(a_i, a_q) * VREF / ADC_MAX
    phase = math.degrees(math.atan2(a_q, a_i))
    return amp, phase


def main():
    print("=" * 70)
    print(" Влияние задержки analogRead на lock-in")
    print("=" * 70)
    print(f"шаг слота = {STEP_US:.0f} мкс, период сигнала = {1e6/F_EXC:.0f} мкс")
    print("Сдвиг по фазе = 360 * adc_us / 1000 (градусов)")
    print()
    print(f"{'задержка, мкс':>14} | {'сдвиг фазы':>11} | {'амплитуда':>11} | "
          f"{'ошибка':>9}")
    print("-" * 70)

    true_amp = 0.015
    for adc_us in [0, 5, 10, 15, 20, 25, 50, 125, 250]:
        amp, _ = simulate(adc_us)
        err = (amp - true_amp) / true_amp * 100
        shift = 360.0 * adc_us / (1e6 / F_EXC)
        print(f"{adc_us:14.0f} | {shift:10.1f}° | {amp*1000:8.3f} мВ | "
              f"{err:+8.1f} %")

    print()
    print("─" * 70)
    print(" Вывод")
    print("─" * 70)
    print("Если analogRead короче шага (25 мкс), задержка постоянна и всего")
    print("лишь сдвигает фазу — на амплитуду почти не влияет (она берётся")
    print("как гипотенуза I/Q). Ломается всё только если analogRead ДОЛЬШЕ")
    print("шага: тогда опора «едет» и результат портится.")
    print()
    print("Критично: НАСТОЯЩИЙ шаг дискретизации = max(25 мкс, время")
    print("analogRead). Проверить на железе через micros() до/после analogRead.")
    print()
    print("Дополнительно: 40 кГц при сигнале 1 кГц — это ровно 40 точек на")
    print("период, то есть целое число. Алиасинг не появляется, но любой")
    print("дрейф частоты из-за RC-цепи даёт биения.")


if __name__ == "__main__":
    main()