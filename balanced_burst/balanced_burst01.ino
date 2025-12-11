/*
 * arduino_lfsr_pwm_examples_burst.ino
 *
 * Adds a "balanced LFSR burst" mode:
 * - Each ISR can perform a short burst of LFSR advances and output updates
 *   until a target number of occurrences of the current "target bit"
 *   (0 or 1) is observed (then the target toggles for next ISR).
 * - Measurement counts actual bit samples produced during bursts so the
 *   reported duty is samples_high / samples_total.
 *
 * Important: this sketch uses direct port writes for speed inside ISR.
 */

#include <Arduino.h>
#include <avr/pgmspace.h>
#include "arduino_lut.h"

#define LFSR_LUT_SIZE 256

// Modes
#define MODE_CLASSIC_ANALOGWRITE 0
#define MODE_TIMER_SOFTWARE      1
#define MODE_LFSR_PWM            2
#define MODE_DDA_PWM             3

volatile uint8_t active_mode = MODE_LFSR_PWM;
const uint8_t OUT_PIN = 9; // OC1A on UNO (PB1)

// Timer tick frequency for software modes (Hz)
const uint32_t TICK_HZ = 31250UL;

// Measurement window: count interrupts and compute duty ratio over this many ISR calls
const uint32_t MEAS_TICKS = TICK_HZ / 10; // 0.1s

// ----- Burst configuration -----
#define BALANCED_BURST_ENABLED 1     // 0 = disabled (single-step LFSR), 1 = enabled
#define BURST_TARGET_OCCURRENCES 4   // occurrences of target bit to collect per ISR (tune)
#define BURST_MAX_ITERS 64           // safety cap to avoid runaway ISR time

// ----- DDA (Accumulator) PWM -----
uint32_t dda_acc = 0;

// Desired PWM level or LUT index
volatile uint8_t pwm_level = 128;

// Measurement counters (updated in ISR)
volatile uint32_t isr_tick_count = 0;    // number of ISR calls
volatile uint32_t bit_sample_count = 0; // number of LFSR output samples seen (includes bursts)
volatile uint32_t high_sample_count = 0; // number of '1' samples
volatile bool meas_ready = false;

// LFSR state and mask
volatile uint16_t lfsr_state = 1; 
volatile uint16_t lfsr_mask_current = 0xB400u;
volatile uint16_t lfsr_threshold = 0;

// For TIMER_SOFTWARE mode
volatile uint8_t sw_counter = 0;

// internal burst toggle (0/1). toggles each ISR so bursts are balanced over time.
volatile uint8_t burst_target_toggle = 0;

// Fast port macros for pin 9 (PB1 on Uno)
#if defined(PORTB) && defined(PB1)
  #define OUT_PORT PORTB
  #define OUT_MASK _BV(PB1)
#else
  #define OUT_PORT PORTB
  #define OUT_MASK _BV(PORTB1)
#endif

// Portable parity function for AVR (inline)
inline uint8_t parity16(uint16_t x) {
  x ^= x >> 8;
  x ^= x >> 4;
  x ^= x >> 2;
  x ^= x >> 1;
  return x & 1;
}

uint16_t load_lfsr_params(uint8_t level_index, uint16_t *seed_out) {
  uint8_t idx = level_index;
  uint16_t mask = pgm_read_word_near(&lfsr_pwm_lut[idx].mask);
  uint16_t seed = pgm_read_word_near(&lfsr_pwm_lut[idx].seed);
  *seed_out = seed;
  return mask;
}

void apply_pwm_level(uint8_t level) {
  pwm_level = level;
  if (active_mode == MODE_LFSR_PWM) {
    uint16_t new_seed;
    lfsr_mask_current = load_lfsr_params(level, &new_seed);
    lfsr_threshold = new_seed;
    lfsr_state = new_seed ? new_seed : 1;
  }
}

// ----- ISR: Timer1 CTC compare -----
ISR(TIMER1_COMPA_vect) {
  uint32_t local_bit_samples = 0;
  uint32_t local_highs = 0;

  switch (active_mode) {
    case MODE_CLASSIC_ANALOGWRITE: {
      uint8_t out = (digitalRead(OUT_PIN) ? 1 : 0);
      local_bit_samples = 1;
      local_highs = out;
    } break;

    case MODE_TIMER_SOFTWARE: {
      sw_counter++;
      uint8_t out = (sw_counter < pwm_level) ? 1 : 0;
      // direct port write for speed
      if (out) OUT_PORT |= OUT_MASK; else OUT_PORT &= ~OUT_MASK;
      local_bit_samples = 1;
      local_highs = out;
    } break;

    case MODE_LFSR_PWM: {
#if BALANCED_BURST_ENABLED
      // Balanced burst: collect BURST_TARGET_OCCURRENCES of "target" bit
      uint8_t target = burst_target_toggle & 1;
      uint8_t found = 0;
      uint16_t iters = 0;

      while ((found < BURST_TARGET_OCCURRENCES) && (iters < BURST_MAX_ITERS)) {
        uint8_t out_bit = (lfsr_state > lfsr_threshold) ? 1 : 0;
        // drive output pin directly (fast)
        if (out_bit) OUT_PORT |= OUT_MASK; else OUT_PORT &= ~OUT_MASK;

        // count
        local_bit_samples++;
        if (out_bit) local_highs++;

        // advance LFSR
        uint16_t feedback = parity16(lfsr_state & lfsr_mask_current);
        lfsr_state = (lfsr_state >> 1) | (feedback << 15);

        if (out_bit == target) found++;

        iters++;
      }
      // toggle target for next ISR so bursts remain balanced
      burst_target_toggle ^= 1;
#else
      // single-step LFSR (original behavior)
      uint8_t out_bit = (lfsr_state > lfsr_threshold) ? 1 : 0;
      if (out_bit) OUT_PORT |= OUT_MASK; else OUT_PORT &= ~OUT_MASK;
      local_bit_samples = 1;
      local_highs = out_bit;
      uint16_t feedback = parity16(lfsr_state & lfsr_mask_current);
      lfsr_state = (lfsr_state >> 1) | (feedback << 15);
#endif
    } break;

    case MODE_DDA_PWM: {
      dda_acc += pwm_level;
      uint8_t out = ((dda_acc & 0xFF00u) != 0) ? 1 : 0;
      if (out) OUT_PORT |= OUT_MASK; else OUT_PORT &= ~OUT_MASK;
      if (out) {
        local_highs = 1;
        dda_acc &= 0x00FFu;
      } else {
        local_highs = 0;
      }
      local_bit_samples = 1;
    } break;
  }

  // Measurement accounting: accumulate samples and ISR ticks
  if (local_bit_samples) {
    bit_sample_count += local_bit_samples;
    high_sample_count += local_highs;
  } else {
    // safety: count the ISR as one sample if something odd happened
    bit_sample_count++;
  }

  isr_tick_count++;
  if (isr_tick_count >= MEAS_TICKS) {
    meas_ready = true;
    // counters will be reset in main loop with interrupts disabled
  }
}

// ----- Timer setup/stop (unchanged) -----
void setup_timer1_ctc(uint32_t tick_hz) {
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  const uint32_t F_CPU_HZ = F_CPU;
  uint16_t prescaler = 1;
  uint16_t csbits = _BV(CS10);
  uint32_t ocr = (F_CPU_HZ / (prescaler * tick_hz)) - 1;
  if (ocr > 0xFFFF) {
    prescaler = 8;
    csbits = _BV(CS11);
    ocr = (F_CPU_HZ / (prescaler * tick_hz)) - 1;
  }
  if (ocr > 0xFFFF) {
    ocr = 65535;
  }
  OCR1A = (uint16_t)ocr;
  TCCR1B |= _BV(WGM12);
  TCCR1B |= csbits;
  TIMSK1 |= _BV(OCIE1A);
  interrupts();
}

void stop_timer1() {
  noInterrupts();
  TIMSK1 &= ~_BV(OCIE1A);
  TCCR1A = 0;
  TCCR1B = 0;
  interrupts();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) ;
  pinMode(OUT_PIN, OUTPUT);
  digitalWrite(OUT_PIN, LOW);
  active_mode = MODE_LFSR_PWM;
  apply_pwm_level(128);
  setup_timer1_ctc(TICK_HZ);
  Serial.println("LFSR PWM with balanced burst - ready");
  Serial.println("Modes: 0=analogWrite,1=timer_sw,2=lfsr,3=dda");
}

void loop() {
  static uint32_t next_change_ms = 0;
  static uint8_t level_index = 0;
  uint32_t now = millis();

  if (now >= next_change_ms) {
    level_index++;
    apply_pwm_level(level_index);
    next_change_ms = now + 200;
  }

  if (meas_ready) {
    noInterrupts();
    uint32_t ticks = isr_tick_count;
    uint32_t bits = bit_sample_count;
    uint32_t highs = high_sample_count;
    isr_tick_count = 0;
    bit_sample_count = 0;
    high_sample_count = 0;
    meas_ready = false;
    interrupts();

    float achieved = (bits == 0) ? 0.0f : ((float)highs / (float)bits);
    Serial.print("Mode="); Serial.print(active_mode);
    Serial.print(" LUT_Idx="); Serial.print(pwm_level);
    Serial.print(" Achieved="); Serial.print(achieved * 100.0f, 2);
    Serial.print("% ISR_ticks="); Serial.print(ticks);
    Serial.print(" bit_samples="); Serial.print(bits);
    Serial.print(" highs="); Serial.println(highs);
  }

  if (Serial.available()) {
    char c = Serial.read();
    if (c >= '0' && c <= '3') {
      active_mode = c - '0';
      uint8_t new_idx = (uint8_t)(pwm_level + 1);
      apply_pwm_level(new_idx);
      Serial.print("Switched to mode "); Serial.println(active_mode);
    } else if (c == '+') {
      uint8_t new_idx = (uint8_t)(pwm_level + 1);
      apply_pwm_level(new_idx);
      Serial.print("LUT Index -> "); Serial.println(new_idx);
    } else if (c == '-') {
      uint8_t new_idx = (uint8_t)(pwm_level - 1);
      apply_pwm_level(new_idx);
      Serial.print("LUT Index -> "); Serial.println(new_idx);
    }
  }

  delay(10);
}
