/*
  arduino_lfsr_pwm_examples.ino

  Demonstrates several PWM implementation strategies and compares their
  behavior by measuring achieved duty ratio. Intended as a reference for
  integrating LFSR-based pseudo-random PWM (precomputed LUT of 16-bit masks)
  alongside classic approaches.

  Implementations included:
    1) CLASSIC_ANALOGWRITE: Use Arduino's built-in analogWrite (hardware PWM)
    2) TIMER_SOFTWARE: Timer interrupt driven software PWM (fixed-frequency)
    3) LFSR_PWM: Timer interrupt driven pseudo-random PWM using an LFSR bitstream
    4) DDA_PWM: Accumulator-style (sigma-delta-like) software PWM as baseline

  The sketch measures the realized fill ratio over a measurement window and
  prints results over Serial so you can compare jitter, short-window accuracy,
  CPU usage (approximation), and spectral distribution.

  Notes:
    - This code is written for AVR-based Arduino (UNO/Nano) using Timer1. It
      uses OCR1A CTC interrupts. On other platforms adapt timers accordingly.
    - For real deployment you will want to replace the small demo LUT below
      with a full 256-entry LUT produced by the python generator and store it
      in PROGMEM (`#include "lfsr_lut.h"`).
    - LFSR masks are 16-bit; we use the LSB as the output bit.

  Author: Generated for user (reference example)
*/

#include <Arduino.h>
#include <avr/pgmspace.h>

// Select which implementations to compile (you can enable multiple and switch at runtime)
#define MODE_CLASSIC_ANALOGWRITE 0
#define MODE_TIMER_SOFTWARE      1
#define MODE_LFSR_PWM            2
#define MODE_DDA_PWM             3

// Change this to pick the active mode at runtime
volatile uint8_t active_mode = MODE_LFSR_PWM;

const uint8_t OUT_PIN = 9; // OC1A on UNO (works with analogWrite on pin 9)

// Timer tick frequency for software modes (Hz)
// Choose high enough so the output appears smooth. 31250 is a commonly used value for Timer1 with prescaler=1.
const uint32_t TICK_HZ = 31250UL;

// Measurement window: count ticks and compute duty ratio over this many ticks
const uint32_t MEAS_TICKS = TICK_HZ / 10; // measure over 0.1s window

// ----- Simple demo LUT (replace with generated full table) -----
// For demo purposes only: small set of example masks for levels 0, 64, 128, 192, 255
// In production include a generated lfsr_lut.h with 256 entries in PROGMEM.
const uint16_t demo_masks[] PROGMEM = {
  0x0001, // level 0 -> trivial (always 1 LSB?) (placeholder)
  0xb400, // sample maximal-length mask (demo)
  0xace1, // another demo mask
  0x9d2c, // demo
  0xffff  // level 255 -> all ones (not a valid LFSR, placeholder for full ON)
};

// Map 0..255 level to mask index in demo masks for this toy example
uint8_t demo_index_for_level(uint8_t level) {
  if (level == 255) return 4;
  if (level >= 192) return 3;
  if (level >= 128) return 2;
  if (level >= 64)  return 1;
  return 0;
}

// ----- LFSR routines -----
static inline uint16_t lfsr_fibonacci_step(uint16_t state, uint16_t mask) {
  // Fibonacci-style: newbit = parity(state & mask), shift right, insert newbit at MSB
  uint16_t feedback = (__builtin_parity(state & mask) & 1);
  return (state >> 1) | (feedback << 15);
}

static inline uint16_t lfsr_galois_step(uint16_t state, uint16_t mask) {
  // Galois LFSR: if LSB==1 then state = (state >> 1) ^ mask; else state >>= 1;
  // Here, mask must be the precomputed Galois-feedback word for fast step.
  if (state & 1) {
    state = (state >> 1) ^ mask;
  } else {
    state = (state >> 1);
  }
  return state;
}

// Fast inline wrapper used inside ISR depending on flag
volatile bool use_galois = true;

// ----- DDA (Accumulator) PWM -----
uint32_t dda_acc = 0;

// Desired PWM level (0..255)
volatile uint8_t pwm_level = 128;

// Measurement counters (updated in ISR)
volatile uint32_t tick_count = 0;
volatile uint32_t high_count = 0;
volatile bool meas_ready = false;

// LFSR state
volatile uint16_t lfsr_state = 1; // must be non-zero
volatile uint16_t lfsr_mask_current = 0xb400; // placeholder

// For TIMER_SOFTWARE mode we use a simple up-counter compare
volatile uint8_t sw_counter = 0;

// ----- ISR: Timer1 CTC compare -----
ISR(TIMER1_COMPA_vect) {
  uint8_t out = 0;
  switch (active_mode) {
    case MODE_CLASSIC_ANALOGWRITE:
      // analogWrite driven by hardware, ISR only used for measurement
      out = digitalRead(OUT_PIN);
      break;

    case MODE_TIMER_SOFTWARE: {
      // fixed-frequency software PWM using an 8-bit counter
      sw_counter++;
      if (sw_counter < pwm_level) {
        digitalWrite(OUT_PIN, HIGH);
        out = 1;
      } else {
        digitalWrite(OUT_PIN, LOW);
        out = 0;
      }
    } break;

    case MODE_LFSR_PWM: {
      // Advance LFSR and output its LSB
      uint16_t s = lfsr_state;
      uint16_t m = lfsr_mask_current;
      if (use_galois) {
        s = lfsr_galois_step(s, m);
      } else {
        s = lfsr_fibonacci_step(s, m);
      }
      lfsr_state = s;
      out = s & 1;
      digitalWrite(OUT_PIN, out ? HIGH : LOW);
    } break;

    case MODE_DDA_PWM: {
      // 8-bit DDA / accumulator method
      dda_acc += pwm_level;
      if ((dda_acc & 0xFF00u) != 0) {
        digitalWrite(OUT_PIN, HIGH);
        out = 1;
        dda_acc &= 0x00FFu; // keep accumulator small
      } else {
        digitalWrite(OUT_PIN, LOW);
        out = 0;
      }
    } break;
  }

  // Measurement accounting
  tick_count++;
  if (out) high_count++;
  if (tick_count >= MEAS_TICKS) {
    meas_ready = true;
    // Do not reset counters here (will reset in main loop with interrupts disabled)
  }
}

// ----- Timer setup -----
void setup_timer1_ctc(uint32_t tick_hz) {
  noInterrupts();
  TCCR1A = 0; // normal port operation, CTC uses OCR1A
  TCCR1B = 0;
  TCNT1 = 0;
  // Compute OCR1A: CPU / prescaler / tick_hz - 1
  const uint32_t F_CPU_HZ = F_CPU;
  uint16_t prescaler = 1;
  uint16_t csbits = _BV(CS10); // prescaler=1
  uint32_t ocr = (F_CPU_HZ / (prescaler * tick_hz)) - 1;
  if (ocr > 0xFFFF) {
    // try prescaler 8
    prescaler = 8;
    csbits = _BV(CS11);
    ocr = (F_CPU_HZ / (prescaler * tick_hz)) - 1;
  }
  if (ocr > 0xFFFF) {
    // fall back to lower tick_hz (make it safe)
    ocr = 65535;
  }
  OCR1A = (uint16_t)ocr;
  TCCR1B |= _BV(WGM12); // CTC mode
  TCCR1B |= csbits;
  TIMSK1 |= _BV(OCIE1A); // enable compare interrupt
  interrupts();
}

void stop_timer1() {
  noInterrupts();
  TIMSK1 &= ~_BV(OCIE1A);
  TCCR1A = 0;
  TCCR1B = 0;
  interrupts();
}

// ----- Utility: load mask for a desired level -----
uint16_t load_mask_for_level(uint8_t level) {
  // In production: read from PROGMEM table with 256 entries
  // demo: map into demo_masks
  uint8_t idx = demo_index_for_level(level);
  uint16_t m = pgm_read_word_near(demo_masks + idx);
  return m;
}

void apply_pwm_level(uint8_t level) {
  pwm_level = level;
  // Update mask for LFSR mode
  lfsr_mask_current = load_mask_for_level(level);
  // Optionally choose seed to avoid short cycles: keep lfsr_state non-zero
  if (lfsr_state == 0) lfsr_state = 1;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) ;
  pinMode(OUT_PIN, OUTPUT);
  digitalWrite(OUT_PIN, LOW);

  // Default mode and level
  active_mode = MODE_LFSR_PWM;
  apply_pwm_level(128);
  use_galois = true;

  // Setup timer
  setup_timer1_ctc(TICK_HZ);

  Serial.println("LFSR PWM examples - ready");
  Serial.println("Modes: 0=analogWrite,1=timer_sw,2=lfsr,3=dda");
}

uint32_t last_print = 0;

void loop() {
  // Example control: rotate through a few duty levels automatically and print results
  static uint32_t next_change_ms = 0;
  static uint8_t level = 0;
  uint32_t now = millis();

  // Every 2 seconds step the level
  if (now >= next_change_ms) {
    level += 32; // steps of 12.5%
    apply_pwm_level(level);
    next_change_ms = now + 2000;
    Serial.print("Set level = "); Serial.println(level);
  }

  // When measurement window completed, read counts and report
  if (meas_ready) {
    noInterrupts();
    uint32_t t = tick_count;
    uint32_t h = high_count;
    tick_count = 0;
    high_count = 0;
    meas_ready = false;
    interrupts();

    float achieved = (t == 0) ? 0.0f : ((float)h / (float)t);
    Serial.print("Mode="); Serial.print(active_mode);
    Serial.print(" Level="); Serial.print(pwm_level);
    Serial.print(" Achieved="); Serial.print(achieved * 100.0f, 2);
    Serial.print("%  ticks="); Serial.print(t);
    Serial.print(" high="); Serial.println(h);
  }

  // Simple user commands via Serial (non-blocking)
  if (Serial.available()) {
    char c = Serial.read();
    if (c >= '0' && c <= '3') {
      active_mode = c - '0';
      Serial.print("Switched to mode "); Serial.println(active_mode);
    } else if (c == 'g') {
      use_galois = !use_galois;
      Serial.print("use_galois = "); Serial.println(use_galois);
    } else if (c == '+') {
      uint8_t newlvl = min(255, pwm_level + 8);
      apply_pwm_level(newlvl);
      Serial.print("level -> "); Serial.println(newlvl);
    } else if (c == '-') {
      uint8_t newlvl = max(0, (int)pwm_level - 8);
      apply_pwm_level(newlvl);
      Serial.print("level -> "); Serial.println(newlvl);
    }
  }

  // light background task
  delay(10);
}

/*
  Explanation of benefits demonstrated by this sketch:

  - CLASSIC_ANALOGWRITE (hardware PWM): extremely low CPU cost, stable carrier
    frequency, good for driving power stages. However, fixed frequency and
    carrier harmonics are deterministic. Resolution is bounded by timer width.

  - TIMER_SOFTWARE: flexible frequency and resolution, but uses ISR and
    per-tick computation; CPU cost grows with tick frequency. Deterministic
    waveform like hardware PWM but with higher jitter (if other ISRs run).

  - LFSR_PWM: produces a pseudo-random bitstream whose long-run density equals
    the target duty ratio (when using a mask with appropriate ones/period).
    Benefits: spreads spectral energy (reduces narrowband EMI), can use high
    tick rates with very cheap per-tick computation (one shift/xor), and
    single-bit output has naturally low switching transitions. Requires LUT
    precomputation to pick masks with the desired ratio and good spectral
    properties. Short-window accuracy depends on LFSR period relative to
    measurement window; choosing masks with period >> window helps.

  - DDA_PWM (Accumulator): very low compute, good average behavior, tends to
    produce tonal spectral components (can be worse for EMI than LFSR), but is
    cheap and easy.

  Use this sketch to compare achieved duty ratio, jitter, and CPU impact.
  Replace the demo LUT with the full 256-entry PROGMEM LUT produced by the
  python generator for real testing.
*/
