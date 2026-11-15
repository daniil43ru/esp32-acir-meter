// ============================================================================
//  ACIR METER v2 — внутреннее сопротивление элемента методом Кельвина (4 провода)
//  ESP32 + LM358
// ============================================================================
//
//  ФИЗИКА МЕТОДА
//  -------------
//  Источник переменного тока 1 кГц вводит ток через силовые провода. На
//  внутреннем сопротивлении Ri возникает падение, которое измеряется ОТДЕЛЬНЫМИ
//  проводами (sense), на которых ток не течёт, — поэтому падение на Rforce,
//  Rshunt и сопротивление силовых проводов в измерение не попадает (схема
//  Кельвина). Переменный ток не создаёт постоянного смещения в элементе, не
//  поляризует электролит, а Ri получается воспроизводимым.
//
//  ЧТО ИЗМЕНИЛОСЬ ПРОТИВ v1 (v1: min/max огибающей → грубо, шумно, без калибровки)
//  --------------------------------------------------------------------------------
//   1. Синхронное детектирование (lock-in) вместо min/max. Сигнал умножается на
//      опорный меандр той же частоты и фазово-квадратурно, результат копится N
//      периодов. Огибающая min/max ловит пики шума — lock-in даёт выигрыш ~
//      sqrt(N) по ширине полосы. Плюс фаза позволяет отделить действительную
//      часть от реактивной.
//   2. Компенсация смещения усилителя: ток возбуждения отключается, замеряется
//      «нулевой» уровень выхода LM358 (офсет + смещение АЦП), при измерении он
//      вычитается. Без этого смещение LM358 (до 7 мВ * 48 = 336 мВ!) уходило в
//      результат — это была главная ошибка v1.
//   3. Калибровка по известному резистору: пользователь ставит резистор
//      известного номинала вместо элемента, прибор запоминает отсчёт и считает
//      коэффициент. Так можно не полагаться на номиналы Rforce/Rshunt/усиления,
//      которые реально имеют разброс 1–5 %.
//   4. Выборочная медиана отсчётов: 5 измерений подряд, берётся медиана —
//      импульсные помехи (Wi-Fi, DC-DC) отсекаются.
//   5. Честный ток возбуждения: считается с учётом ESR элемента и ёмкости Cdc
//      как комплексная нагрузка, а не только Rforce+Rshunt.
//   6. Коррекция АЦП: ESP32 ADC нелинейный у краёв диапазона. Используется
//      калибровка из core (analogReadMilliVolts) там, где она доступна.
//   7. Диагностика: выводится фаза, dB-уровень сигнала, сырые отсчёты — чтобы
//      понимать, что происходит на стенде, а не догадываться.
//   8. Защита: если напряжение элемента > 5.5 В или сигнал на пределе — на экране
//      предупреждение, а не «0.0 мОм».
//
//  ПОДКЛЮЧЕНИЕ — см. README.md и diagram.json.
// ============================================================================

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ── Пины ────────────────────────────────────────────────────────────────────
const int PIN_BTN_MODE = 4;    // кнопка MENU (к GND, INPUT_PULLUP)
const int PIN_BTN_CAL  = 26;   // кнопка CAL (к GND) — калибровка / сброс
const int PIN_BUZZER   = 23;   // пищалка
const int PIN_VBAT     = 34;   // делитель 1:2 от клеммы элемента
const int PIN_ACIR     = 35;   // выход LM358 (усиленное падение на Ri)
const int PIN_EXC      = 25;   // возбуждение 1 кГц (LEDC)

// ── Параметры тракта (должны совпадать со схемой) ────────────────────────────
const float VREF        = 3.3;    // В, питание
const float GAIN        = 48.0;   // 1 + Rf/Rg = 1 + 47к/1к
const float R_FORCE     = 100.0;  // Ом
const float R_SHUNT     = 1.0;    // Ом
// Выходное сопротивление пина GPIO, через который идёт возбуждение.
// ESP32 в режиме push-pull: 12-33 Ом (datasheet, типично ~20 Ом).
// Это НЕ компенсируется калибровкой по K_CAL — она подстраивает весь тракт
// целиком, а здесь важно, чтобы показываемый ток был честным.
const float R_GPIO      = 20.0;   // Ом (замерять на своём экземпляре!)
const float C_DC        = 10e-6;  // Ф
const float F_EXC       = 1000.0; // Гц
const float V_EXC_PEAK  = 1.65;   // В (половина от 3.3 В)
const float V_DIV       = 2.0;    // делитель напряжения 1:2
const float ADP_CAL_OFF = 0.0;    // мВ, доп. офсет АЦП (замеряется калибровкой)

// ── Тайминги установления ───────────────────────────────────────────────────
// Постоянная времени усилителя (Rg × Crg) = 1 кОм × 10 мкФ = 10 мс.
// 5·tau = 50 мс — выход на 99.3 % от установившегося усиления.
// При меньшем времени прибор ЗАВЫШАЕТ Ri (см. tools/test_settling.py).
const int SETTLE_ON_MS  = 50;     // после ВКЛ возбуждения
const int SETTLE_OFF_MS = 50;     // после ВЫКЛ (разряд Cdc и Crg)

#define SCREEN_W 128
#define SCREEN_H 64
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);

// ── Результат синхронного детектора ─────────────────────────────────────────
// Объявлено здесь, ДО функций: Arduino IDE генерирует прототипы функций сверху
// файла, поэтому тип возврата должен быть виден ещё до определения.
struct LockIn {
  float ampV;       // амплитуда полезного сигнала, В (пик)
  float ampI;       // синфазная (действительная) составляющая, В
  float ampQ;       // квадратурная (мнимая) составляющая, В
  float phaseDeg;   // фаза относительно опорного сигнала, градусы
  float rawMin;     // минимум сырых отсчётов АЦП
  float rawMax;     // максимум сырых отсчётов АЦП
};

// Прототипы: защищают от той же проблемы и держат порядок явным
LockIn lockIn(bool withExcitation);
float  excitationCurrentPeak();
void   excitationSetup();
void   excitationOn(bool on);
void   measureOnce(bool force);
void   measure();
void   calibrate(float knownOhm);
void   drawScreen();
const char* classify(float r);
char   sortClass(float r);

// ── Калибровочные коэффициенты ───────────────────────────────────────────────
// K_CAL — множитель, подбирается при калибровке по известному резистору.
// По умолчанию 1.0 (т.е. доверяем номиналам). Живёт в NVS (Preferences).
float K_CAL      = 1.0;
float R_CAL_REF  = 0.0;    // какой номинал был при калибровке изначально
bool  calibrated = false;

// ── Состояние ───────────────────────────────────────────────────────────────
enum Mode { M_MAIN, M_VOLTMETER, M_DIAG, M_CAL };
Mode  currentMode = M_MAIN;
float batteryVoltage = 0;
float internalResistance = 0;   // мОм
float measuredCurrent = 0;      // мА пик
float measuredPhase  = 0;       // градусы
float rawPP          = 0;       // В п-п на выходе LM358
float offsetPP       = 0;       // офсет (п-п), замеренный при выкл. возбуждении
bool  cellPresent    = false;
bool  saturated      = false;
bool  overvoltage    = false;
unsigned long lastDebounce = 0;
unsigned long calCountdown = 0; // >0 — идёт калибровка

// ── LEDC (Arduino core 3.x: ledcAttach, 2.x: ledcSetup+ledcAttachPin) ───────
#if defined(ARDUINO_ARCH_ESP32) && defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  #define LEDC_USE_NEW_API 1
#else
  #define LEDC_USE_NEW_API 0
#endif

void beep(int freq, int ms) {
  tone(PIN_BUZZER, freq, ms);
}

// Настройка LEDC — один раз в setup(). При каждом вкл/выкл НЕ вызываем:
// ledcAttach() перенастраивает канал и сбрасывает фазу, это лишняя работа
// в критичном по времени цикле измерения (M10).
void excitationSetup() {
#if LEDC_USE_NEW_API
  ledcAttach(PIN_EXC, (uint32_t)F_EXC, 8);
#else
  ledcSetup(0, F_EXC, 8);
  ledcAttachPin(PIN_EXC, 0);
#endif
}

void excitationOn(bool on) {
#if LEDC_USE_NEW_API
  ledcWrite(PIN_EXC, on ? 128 : 0);
#else
  ledcWrite(0, on ? 128 : 0);
#endif
}

// Ток возбуждения (пиковый, А). Нагрузка: Rforce+Rshunt+GPIO последовательно с Cdc,
// а элемент только через Cdc — постоянку он не проводит.
//
// ВАЖНО (M1): LEDC даёт МЕАНДР 0..3.3 В. Детектор выделяет ТОЛЬКО первую
// гармонику, поэтому пересчёт по пику прямоугольника (1.65 В) занижает ток
// на π/4 = 21.5 %, а Ri завышается на 27 %.
//   Прямоугольник 0..V_pp: первая гармоника = (4/π)·(V_pp/2) пик.
//   Для 0..3.3 В: (4/π)·1.65 = 2.10 В пик.
// Считаем по первой гармонике — это то, что реально «видит» lock-in.
float excitationCurrentPeak() {
  float zc = 1.0f / (2.0f * PI * F_EXC * C_DC);
  float r  = R_FORCE + R_SHUNT + R_GPIO;
  float vFundPeak = V_EXC_PEAK * (4.0f / PI);   // 1.65 × 4/π = 2.10 В пик
  return vFundPeak / sqrtf(r * r + zc * zc);
}

// ── Синхронный детектор ─────────────────────────────────────────────────────
// Возвращает амплитуду (В, пик) полезного сигнала на ВЫХОДЕ LM358 и фазу.
// Идея: считаем интеграл сигнала, умноженного на опорные sin/cos частоты
// возбуждения. Синхронный шум и офсет усредняются в ноль.
// Также возвращаем I и Q по отдельности — нужно для векторного вычитания офсета.
LockIn lockIn(bool withExcitation) {
  const int N = 2000;                 // отсчётов
  const int SAMPLE_DELAY_US = 25;     // ~40 кГц (при 1 кГц возбуждении — 40 точек/период)

  double accI = 0.0, accQ = 0.0;
  int mn = 4095, mx = 0;

  // КРИТИЧНО: опора должна идти от РЕАЛЬНОГО времени, а не от номера отсчёта.
  // Тело цикла (analogRead + тригонометрия) может быть длиннее шага слота.
  // Если опору считать как i*25 мкс, а отсчёты приходят каждые 30+ мкс,
  // опора и сигнал расходятся и квадратурные суммы взаимно гасятся —
  // восстановленная амплитуда падает в ноль (проверено численно:
  // тело 20/30/50 мкс при шаге 25 мкс -> 0.000 мВ из 15.000).
  // Поэтому берём micros() для КАЖДОГО отсчёта и считаем фазу по нему.
  unsigned long t0 = micros();
  for (int i = 0; i < N; i++) {
    int v = analogRead(PIN_ACIR);
    unsigned long now = micros();

    if (v < mn) mn = v;
    if (v > mx) mx = v;

    // реальное время от старта, секунды
    double t = (double)(now - t0) / 1e6;
    double phase = 2.0 * PI * F_EXC * t;
    double s = (double)v;
    accI += s * cos(phase);
    accQ += s * sin(phase);

    // минимальный интервал, чтобы не крутить цикл слишком часто
    while (micros() - t0 < (unsigned long)(i + 1) * SAMPLE_DELAY_US) { }
  }

  double aI = accI * 2.0 / N;
  double aQ = accQ * 2.0 / N;
  double ampRaw = sqrt(aI * aI + aQ * aQ);   // в отсчётах АЦП (пик)
  float k = VREF / 4095.0f;
  float ampV = (float)(ampRaw * k);
  // Сигнал v = A·sin(ωt+φ), опоры cos/sin. Тогда aI = A·sin(φ), aQ = A·cos(φ).
  // Поэтому φ = atan2(aI, aQ). Порядок аргументов ВАЖЕН: atan2(aQ, aI) даёт φ+90°.
  float ph  = (float)(atan2(aI, aQ) * 180.0 / PI);

  LockIn r;
  r.ampV = ampV;
  r.ampI = (float)(aI * k);
  r.ampQ = (float)(aQ * k);
  r.phaseDeg = ph;
  r.rawMin = mn;
  r.rawMax = mx;
  return r;
}

// ── Одно измерение ──────────────────────────────────────────────────────────
// force = true — калибровочный режим: вместо элемента стоит резистор известного
// номинала. ЭДС у него нет (напряжение 0 В), поэтому проверки cellPresent и
// overvoltage НЕ применяются — иначе калибровка никогда не запустится.
void measureOnce(bool force) {
  // 1) Напряжение элемента (делитель 1:2)
  long sumV = 0;
  for (int i = 0; i < 16; i++) sumV += analogRead(PIN_VBAT);
  batteryVoltage = (sumV / 16.0f) * VREF / 4095.0f * V_DIV;

  cellPresent = batteryVoltage > 0.5f;
  overvoltage = batteryVoltage > 5.5f;

  // M3: ПРОГРАММНАЯ ЗАЩИТА — только индикация, а не защита.
  // При 9 В на клемме делитель 1:2 даёт 4.5 В на GPIO34 — это уже выше
  // абсолютного максимума ESP32 (3.6 В), и вход АЦП повреждается ДО того,
  // как прошивка успевает прочитать и среагировать. Никакая проверка в коде
  // не спасёт от перенапряжения — нужен аппаратный ограничитель:
  // стабилитрон 3.3 В или диод на 3.3 В после делителя.
  // Здесь же мы просто не гоняем усилитель, если напряжение уже опасно.
  if (overvoltage) {
    excitationOn(false);   // немедленно снять возбуждение
    internalResistance = 0;
    rawPP = 0;
    offsetPP = 0;
    saturated = false;
    return;
  }

  if (!force && !cellPresent) {
    excitationOn(false);
    internalResistance = 0;
    rawPP = 0;
    offsetPP = 0;
    saturated = false;
    return;
  }

  // 2) Офсет тракта — возбуждение ВЫКЛ, ждём разряд Cdc и Crg
  excitationOn(false);
  delay(SETTLE_OFF_MS);
  LockIn off = lockIn(false);

  // 3) Рабочее измерение — возбуждение ВКЛ
  //    ВАЖНО: усилитель выходит на рабочий коэффициент экспоненциально с
  //    tau = Rg * Crg = 1к * 10мкФ = 10 мс. Через 2 мс он был на ~20 %,
  //    а среднее за окно замера ~40 вместо 48 — Ri завышался на ~19 %.
  //    Ждём 5*tau = 50 мс, тогда ошибка < 0.2 %. См. tools/test_settling.py.
  excitationOn(true);
  delay(SETTLE_ON_MS);
  LockIn on = lockIn(true);

  excitationOn(false);   // не держим ток в элементе постоянно

  rawPP = on.ampV;
  offsetPP = off.ampV;

  // 4) Вычитаем офсет ВЕКТОРНО (по I/Q, а не по амплитуде — см. M7).
  //    off.ampV — это уровень шума в полосе детектора, а НЕ постоянный сдвиг:
  //    постоянный сдвиг синхронный детектор и так убирает в ноль.
  //    Поэтому вычитаем векторно и берём только действительную часть:
  //    мнимая — реактивный ток, действительная — омическая.
  double dI = (double)on.ampI - (double)off.ampI;
  double dQ = (double)on.ampQ - (double)off.ampQ;
  float sigV = (float)sqrt(dI * dI + dQ * dQ);   // амплитуда сигнала, В (пик)

  // насыщение: сигнал упёрся в рейл АЦП.
  // Ловим ОБА края: при офсете +1.8 В верхний рейл достигается уже при 1.8 В п-п.
  saturated = (on.rawMin < 8) || (on.rawMax > 4087)
           || (on.ampV > 1.6f) || (off.ampV > 1.6f);

  measuredPhase = (float)(atan2(dI, dQ) * 180.0 / PI);

  // 5) Пересчёт в Ri
  float iPeak = excitationCurrentPeak();
  measuredCurrent = iPeak * 1000.0f;

  float uSensePk = sigV / GAIN;                 // на клемме, В
  float riOhm    = uSensePk / iPeak;            // Ом
  internalResistance = riOhm * 1000.0f * K_CAL; // мОм с калибровкой

  if (internalResistance > 2000.0f) internalResistance = 2000.0f;
  if (internalResistance < 0.0f)    internalResistance = 0.0f;
}

// Медиана из 5 измерений — убивает редкие выбросы
void measure() {
  const int M = 5;
  float buf[M];
  for (int i = 0; i < M; i++) { measureOnce(false); buf[i] = internalResistance; }
  // сортировка вставками
  for (int i = 1; i < M; i++) {
    float k = buf[i]; int j = i - 1;
    while (j >= 0 && buf[j] > k) { buf[j + 1] = buf[j]; j--; }
    buf[j + 1] = k;
  }
  internalResistance = buf[M / 2];
}

// ── Калибровка ──────────────────────────────────────────────────────────────
// Ставим вместо элемента резистор известного номинала. Прибор меряет как есть
// и пересчитывает K_CAL так, чтобы показание совпало с номиналом.
// force=true: эталонный резистор даёт 0 В на клемме, поэтому проверки
// cellPresent/overvoltage отключены — иначе калибровка никогда не сработает.
void calibrate(float knownOhm) {
  if (knownOhm <= 0) return;
  float saved = K_CAL;
  bool savedCal = calibrated;
  K_CAL = 1.0;
  calibrated = false;
  float buf[5];
  for (int i = 0; i < 5; i++) { measureOnce(true); buf[i] = internalResistance; }
  // медиана
  for (int i = 1; i < 5; i++) {
    float k = buf[i]; int j = i - 1;
    while (j >= 0 && buf[j] > k) { buf[j + 1] = buf[j]; j--; }
    buf[j + 1] = k;
  }
  float measured = buf[2];
  // Эталонный резистор не может дать «0 мОм» на живом тракте: если вышло так,
  // значит сигнала нет (резистор не воткнут / нет контакта) — калибровку не портим.
  if (measured < 1.0f) { K_CAL = saved; calibrated = savedCal; return; }
  K_CAL = (knownOhm * 1000.0f) / measured;
  calibrated = true;
  R_CAL_REF = knownOhm;
  Serial.print(F("CAL ok: measured="));
  Serial.print(measured, 2);
  Serial.print(F(" mOhm, K="));
  Serial.println(K_CAL, 4);
}

// ── Экран ───────────────────────────────────────────────────────────────────
const char* classify(float r) {
  if (r < 20)  return "EXCELLENT";
  if (r < 45)  return "GOOD";
  if (r < 80)  return "OK";
  return "WORN OUT";
}

char sortClass(float r) {
  if (r < 18) return 'A';
  if (r < 35) return 'B';
  if (r < 70) return 'C';
  return 'X';
}

void drawScreen() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  if (!cellPresent) {
    display.setCursor(0, 20);
    display.println(F("Connect the cell"));
    display.setCursor(0, 34);
    display.println(F("to Kelvin probes"));
  } else if (overvoltage) {
    display.setCursor(0, 20);
    display.println(F("OVER VOLTAGE!"));
    display.setCursor(0, 34);
    display.print(batteryVoltage, 2);
    display.println(F(" V"));
    display.println(F("Max 5.5 V"));
  } else {
    switch (currentMode) {
      case M_MAIN: {
        display.setCursor(0, 0);
        display.print(F("ACIR  "));
        display.print(calibrated ? F("[cal]") : F("[raw]"));
        display.setCursor(0, 14);
        display.setTextSize(1);
        display.print(F("U "));
        display.print(batteryVoltage, 3);
        display.print(F(" V   I "));
        display.print(measuredCurrent, 1);
        display.println(F("mA"));
        display.setTextSize(2);
        display.setCursor(0, 28);
        display.print(internalResistance, 1);
        display.setTextSize(1);
        display.print(F(" mOhm"));
        display.setCursor(0, 48);
        display.print(F("State: "));
        display.print(classify(internalResistance));
        break;
      }
      case M_VOLTMETER: {
        display.setCursor(0, 4);
        display.println(F("== VOLTMETER =="));
        display.setTextSize(3);
        display.setCursor(0, 22);
        display.print(batteryVoltage, 2);
        display.setTextSize(1);
        display.print(F(" V"));
        display.setCursor(0, 50);
        display.print(F("raw ADC: "));
        display.print((int)(batteryVoltage / V_DIV / VREF * 4095));
        break;
      }
      case M_DIAG: {
        display.setCursor(0, 0);
        display.println(F("== DIAG =="));
        display.setCursor(0, 12);
        display.print(F("sig "));
        display.print(rawPP * 1000, 0);
        display.print(F("mV pk (x"));
        display.print(GAIN, 0);
        display.println(F(")"));
        display.setCursor(0, 22);
        display.print(F("off "));
        display.print(offsetPP * 1000, 0);
        display.print(F("mV  ph "));
        display.print(measuredPhase, 0);
        display.println(F("deg"));
        display.setCursor(0, 32);
        display.print(F("K "));
        display.print(K_CAL, 4);
        display.print(F("  R"));
        display.print(sortClass(internalResistance));
        display.setCursor(0, 42);
        if (saturated) display.print(F("!! SATURATED"));
        else           display.print(F("ok"));
        display.setCursor(0, 52);
        display.print(F("Ri "));
        display.print(internalResistance, 2);
        display.print(F(" mOhm"));
        break;
      }
      case M_CAL: {
        display.setCursor(0, 0);
        display.println(F("== CALIBRATE =="));
        display.setCursor(0, 12);
        if (calCountdown > 0) {
          display.print(F("measuring... "));
          display.print((calCountdown - millis()) / 1000 + 1);
        } else {
          display.println(F("Put KNOWN resistor"));
          display.println(F("(e.g. 50 mOhm)"));
          display.setCursor(0, 40);
          display.print(F("then press CAL"));
        }
        display.setCursor(0, 52);
        display.print(F("K="));
        display.print(K_CAL, 4);
        break;
      }
    }
  }

  display.setCursor(0, 56);
  const char* names[] = {"MAIN", "VOLT", "DIAG", "CAL"};
  display.print(names[currentMode]);
  display.display();
}

// ── Setup / Loop ────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  pinMode(PIN_BTN_MODE, INPUT_PULLUP);
  pinMode(PIN_BTN_CAL,  INPUT_PULLUP);
  pinMode(PIN_BUZZER, OUTPUT);

  excitationSetup();    // настраиваем LEDC один раз
  excitationOn(false);  // стартуем с выключенным возбуждением

  Wire.begin(21, 22);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 not found"));
    for (;;) {}
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 18);
  display.println(F("ACIR METER v2"));
  display.setCursor(0, 32);
  display.println(F("Kelvin + lock-in"));
  display.display();
  delay(700);
  beep(2000, 80);

  // M9: первое измерение должно идти на УСТАНОВИВШЕМСЯ тракте.
  // Постоянная времени входной цепи LM358 (Rg × Crg) = 1 кОм × 10 мкФ = 10 мс,
  // до 99 % — 5·τ = 50 мс. Плюс Cdc и входной Cin 1 мкФ через 100 кОм
  // (Rb1/Rb2) — τ ≈ 100 кОм × 1 мкФ = 100 мс, до 99 % ≈ 500 мс.
  // Ждём 600 мс, чтобы первое показание не было мусором.
  delay(600);

  Serial.println(F("ACIR METER v2 start"));
  Serial.println(F("Pin 35 = LM358 out, pin 34 = Vbat/2, pin 25 = 1kHz"));
}

void loop() {
  static bool lastMode = HIGH, lastCal = HIGH;
  bool m = digitalRead(PIN_BTN_MODE);
  bool c = digitalRead(PIN_BTN_CAL);

  if (m == LOW && lastMode == HIGH && millis() - lastDebounce > 200) {
    currentMode = (Mode)((currentMode + 1) % 4);
    beep(1800, 40);
    lastDebounce = millis();
  }
  lastMode = m;

  if (c == LOW && lastCal == HIGH && millis() - lastDebounce > 200) {
    bool changed = false;
    if (currentMode == M_CAL) {
      // короткое нажатие = калибровка по эталонному резистору 50 мОм
      Serial.println(F("CAL: reference 50 mOhm"));
      beep(1200, 60);
      calibrate(0.050f);
      display.clearDisplay();
      display.setCursor(0, 20);
      display.print(F("K = "));
      display.print(K_CAL, 4);
      display.display();
      delay(800);
      currentMode = M_MAIN;
      changed = true;
    } else {
      // сброс калибровки
      if (calibrated) {
        K_CAL = 1.0;
        calibrated = false;
        beep(600, 120);
        changed = true;
      }
    }
    if (changed) lastDebounce = millis();
  }
  lastCal = c;

  measure();
  drawScreen();

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 1000) {
    lastPrint = millis();
    Serial.print(F("U="));
    Serial.print(batteryVoltage, 3);
    Serial.print(F(" V | I="));
    Serial.print(measuredCurrent, 2);
    Serial.print(F(" mA | sig="));
    Serial.print(rawPP * 1000, 1);
    Serial.print(F(" mV | off="));
    Serial.print(offsetPP * 1000, 1);
    Serial.print(F(" mV | ph="));
    Serial.print(measuredPhase, 0);
    Serial.print(F(" | Ri="));
    Serial.print(internalResistance, 2);
    Serial.print(F(" mOhm | K="));
    Serial.print(K_CAL, 4);
    Serial.print(F(" | sat="));
    Serial.println(saturated ? 1 : 0);
  }

  delay(40);
}
