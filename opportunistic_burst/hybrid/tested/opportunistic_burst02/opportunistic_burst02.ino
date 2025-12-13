/*
  hybrid_fibonacci_lut_replace.ino (OPTIMIZED + last-period time)

  - Records only last period length (steps) and how many ms that period took.
  - Uses micros() for timing (start at first step after level applied,
    end when LFSR returns to the anchor state).
*/

#include <Arduino.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include "arduino_lut.h"

// ---------- CONFIG ----------
const uint8_t OUT_PIN = 9;
#define OUT_PORT PORTB
#define OUT_BIT PB1
#define OUT_MASK (1 << OUT_BIT)

#define LFSR_LUT_SIZE 256

volatile uint8_t pwm_level_idx = 32;

const uint32_t SAMPLE_HZ = 4000UL;
const uint32_t TICK_HZ = 4000UL;
const uint16_t JITTER_MIN_MS = 1;
const uint16_t JITTER_MAX_MS = 5;

const uint32_t MAX_MAIN_BURST_SAMPLES = 40000UL;
const uint32_t MAX_ISR_BURST_SAMPLES = 16UL;
const uint16_t ISR_MICROBURST_ITER_CAP = 8;

const uint8_t MAIN_MICROBURST_OCCURRENCES = 8;
const uint16_t MAIN_MICROBURST_ITER_CAP = 128;
const uint16_t MAIN_EXIT_FLIP_CAP = 16;
const uint8_t ISR_EXIT_FLIP_CAP = 16;

// Galois-style stepping can be faster on AVR; ensure LUT masks match Galois usage
#define USE_GALOIS_LFSR 0

// ---------- LFSR state ----------
volatile uint16_t lfsr_state = 1;
volatile uint16_t lfsr_mask_current = 0xB400u;
volatile uint16_t lfsr_threshold = 0;

// Anchor state for period detection (set when level applied)
volatile uint16_t cycle_anchor_state = 1;

// ---------- bookkeeping ----------
volatile uint32_t shared_sample_count = 0;
volatile uint32_t shared_high_count = 0;
volatile uint32_t sample_credit = 0;
volatile uint32_t credit_fp_acc = 0;
volatile uint8_t isr_burst_target_toggle = 0;

uint8_t main_burst_start_target = 0;
uint32_t last_print_ms = 0;

// ---------- last-period stats ----------
volatile uint32_t cycle_counter = 0;         // steps since last anchor hit
volatile uint32_t last_cycle_len = 0;        // last observed cycle length (steps)
volatile uint32_t last_cycle_ms = 0;         // last observed cycle duration in milliseconds
volatile unsigned long start_cycle_us = 0;   // microsecond timestamp of cycle start (0 when not running)

// ---------- PARITY helper ----------
static inline uint8_t parity16_fast(uint16_t x) {
#ifdef __GNUC__
  return __builtin_popcount(x) & 1;
#else
  x ^= x >> 8;
  x ^= x >> 4;
  x ^= x >> 2;
  x ^= x >> 1;
  return x & 1;
#endif
}

// ---------- LUT load ----------
static inline uint16_t load_lfsr_params(uint8_t level_index, uint16_t *seed_out) {
  uint16_t mask = pgm_read_word_near(&lfsr_pwm_lut[level_index].mask);
  uint16_t seed = pgm_read_word_near(&lfsr_pwm_lut[level_index].seed);
  *seed_out = seed;
  return mask;
}

void reset_cycle_tracking_for_level() {
  cycle_counter = 0;
  last_cycle_len = 0;
  last_cycle_ms = 0;
  start_cycle_us = 0;
}

void apply_pwm_level(uint8_t level_idx) {
  uint16_t new_seed, new_mask;
  new_mask = load_lfsr_params(level_idx, &new_seed);

  uint8_t sreg = SREG;
  cli();
  lfsr_mask_current = new_mask;
  lfsr_threshold = new_seed;
  lfsr_state = new_seed ? new_seed : 1;
  cycle_anchor_state = lfsr_state; // anchor for period detection
  reset_cycle_tracking_for_level();
  pwm_level_idx = level_idx;
  SREG = sreg;
}

// ---------- LFSR step + last-period timing ----------
static inline uint8_t lfsr_step_and_out() __attribute__((always_inline));
static inline uint8_t lfsr_step_and_out() {
  uint16_t s = lfsr_state;

#if USE_GALOIS_LFSR
  uint8_t lsb = s & 1;
  s >>= 1;
  if (lsb) s ^= lfsr_mask_current;
#else
  uint8_t fb = parity16_fast(s & lfsr_mask_current);
  s = (s >> 1) | ((uint16_t)fb << 15);
#endif

  s |= (s == 0);
  lfsr_state = s;

  // Start timing on the first step after reset/anchor
  cycle_counter++;
  if (cycle_counter == 1) {
    // record when cycle started (micros is safe here)
    start_cycle_us = micros();
  }

  if (s == cycle_anchor_state) {
    // Completed a full period
    unsigned long end_us = micros();
    uint32_t len = cycle_counter;
    cycle_counter = 0;
    // compute ms duration (handles wraparound because unsigned math)
    uint32_t dur_ms = (uint32_t)((end_us - start_cycle_us) / 1000UL);
    last_cycle_len = len;
    last_cycle_ms = dur_ms;
    start_cycle_us = 0;
  }

  return (s > lfsr_threshold) ? 1 : 0;
}

// ---------- FAST port write ----------
static inline void write_output(uint8_t val) __attribute__((always_inline));
static inline void write_output(uint8_t val) {
  if (val) {
    OUT_PORT |= OUT_MASK;
  } else {
    OUT_PORT &= ~OUT_MASK;
  }
}

static inline uint8_t read_output() __attribute__((always_inline));
static inline uint8_t read_output() {
  return (OUT_PORT & OUT_MASK) ? 1 : 0;
}

// ---------- ISR microburst ----------
static uint8_t isr_microburst_and_flip(uint8_t target, uint8_t want_occurrences) {
  uint8_t found = 0;
  uint8_t produced = 0;
  uint16_t iter = 0;

  const uint8_t entering = read_output();

  while (found < want_occurrences && iter < ISR_MICROBURST_ITER_CAP && produced < MAX_ISR_BURST_SAMPLES) {
    uint8_t out = lfsr_step_and_out();
    write_output(out);

    shared_sample_count++;
    shared_high_count += out;

    found += (out == target);
    produced++;
    iter++;
  }

  uint8_t flip_attempts = 0;
  uint8_t exit_bit = read_output();

  while (exit_bit == entering && flip_attempts < ISR_EXIT_FLIP_CAP && produced < MAX_ISR_BURST_SAMPLES) {
    uint8_t out = lfsr_step_and_out();
    write_output(out);

    shared_sample_count++;
    shared_high_count += out;

    produced++;
    flip_attempts++;
    exit_bit = read_output();
  }

  return produced;
}

// ---------- OPTIMIZED main burst ----------
uint32_t main_produce_balanced_batch_and_flip(uint32_t total_samples) {
  if (total_samples == 0) return 0;
  if (total_samples > MAX_MAIN_BURST_SAMPLES) {
    total_samples = MAX_MAIN_BURST_SAMPLES;
  }

  uint32_t desired_ones = ((uint32_t)total_samples * pwm_level_idx + 128) / 256;
  uint32_t ones_left = desired_ones;
  uint32_t zeros_left = total_samples - desired_ones;
  uint32_t produced = 0;
  uint8_t target = main_burst_start_target;

  const uint8_t entering = read_output();

  while ((ones_left | zeros_left) != 0) {
    uint32_t want_occ = MAIN_MICROBURST_OCCURRENCES;

    if (target) {
      if (ones_left == 0) { target = 0; continue; }
      if (ones_left < want_occ) want_occ = ones_left;
    } else {
      if (zeros_left == 0) { target = 1; continue; }
      if (zeros_left < want_occ) want_occ = zeros_left;
    }

    uint32_t found = 0;
    uint32_t iter = 0;

    while (found < want_occ && iter < MAIN_MICROBURST_ITER_CAP && produced < total_samples) {
      uint8_t out = lfsr_step_and_out();
      write_output(out);

      shared_sample_count++;
      shared_high_count += out;

      found += (out == target);
      produced++;
      iter++;
    }

    if (target) {
      ones_left = (found <= ones_left) ? (ones_left - found) : 0;
    } else {
      zeros_left = (found <= zeros_left) ? (zeros_left - found) : 0;
    }

    if (produced >= total_samples) break;
    target ^= 1;
  }

  uint16_t flip_iters = 0;
  uint8_t exit_bit = read_output();

  while (exit_bit == entering && flip_iters < MAIN_EXIT_FLIP_CAP && produced < MAX_MAIN_BURST_SAMPLES) {
    uint8_t out = lfsr_step_and_out();
    write_output(out);

    shared_sample_count++;
    shared_high_count += out;

    produced++;
    flip_iters++;
    exit_bit = read_output();
  }

  main_burst_start_target ^= 1;
  return produced;
}

// ---------- Timer1 ISR ----------
ISR(TIMER1_COMPA_vect) {
  credit_fp_acc += SAMPLE_HZ;

  if (credit_fp_acc >= TICK_HZ) {
    uint32_t delta = credit_fp_acc / TICK_HZ;
    credit_fp_acc -= delta * TICK_HZ;
    sample_credit += delta;
  }

  if (sample_credit > 0) {
    uint8_t target = isr_burst_target_toggle & 1;
    isr_burst_target_toggle ^= 1;

    uint8_t produced = isr_microburst_and_flip(target, 1);
    sample_credit = (produced >= sample_credit) ? 0 : (sample_credit - produced);
  }
}

// ---------- Timer1 setup ----------
void setup_timer1_ctc(uint32_t tick_hz) {
  uint8_t sreg = SREG;
  cli();

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  uint16_t prescaler = 1;
  uint8_t csbits = (1 << CS10);
  uint32_t ocr = (F_CPU / tick_hz) - 1;

  if (ocr > 0xFFFF) {
    prescaler = 8;
    csbits = (1 << CS11);
    ocr = (F_CPU / (8UL * tick_hz)) - 1;
    if (ocr > 0xFFFF) ocr = 0xFFFF;
  }

  OCR1A = (uint16_t)ocr;
  TCCR1B = (1 << WGM12) | csbits;
  TIMSK1 = (1 << OCIE1A);

  SREG = sreg;
}

// ---------- setup / loop ----------
void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(OUT_PIN, OUTPUT);
  digitalWrite(OUT_PIN, LOW);

  apply_pwm_level(pwm_level_idx);
  randomSeed(analogRead(A0));
  setup_timer1_ctc(TICK_HZ);

  last_print_ms = millis();
  Serial.println(F("Optimized Fibonacci-LFSR LUT PWM (last-period time)"));
  Serial.print(F("Initial LUT idx = ")); Serial.println(pwm_level_idx);
  Serial.print(F("SAMPLE_HZ=")); Serial.print(SAMPLE_HZ);
  Serial.print(F(" TICK_HZ=")); Serial.println(TICK_HZ);
  Serial.println(F("Commands: + increase, - decrease"));
}

void loop() {
  delay(random(JITTER_MIN_MS, JITTER_MAX_MS + 1));

  uint8_t sreg = SREG;
  cli();
  TIMSK1 &= ~(1 << OCIE1A);
  uint32_t credits = sample_credit;
  sample_credit = 0;
  SREG = sreg;

  if (credits > 0) {
    if (credits > MAX_MAIN_BURST_SAMPLES) credits = MAX_MAIN_BURST_SAMPLES;
    main_produce_balanced_batch_and_flip(credits);
  }

  cli();
  TIMSK1 |= (1 << OCIE1A);
  sei();

  uint32_t now_ms = millis();
  if (now_ms - last_print_ms >= 1000UL) {
    sreg = SREG;
    cli();
    uint32_t samples = shared_sample_count;
    uint32_t highs = shared_high_count;
    shared_sample_count = 0;
    shared_high_count = 0;
    uint32_t pending = sample_credit;

    // atomic grab of last-period stats
    uint32_t last_len = last_cycle_len;
    uint32_t last_ms = last_cycle_ms;
    SREG = sreg;

    float achieved = (samples > 0) ? ((float)highs / (float)samples) : 0.0f;
    Serial.print(F("LUT_idx=")); Serial.print(pwm_level_idx);
    Serial.print(F(" samples=")); Serial.print(samples);
    Serial.print(F(" achieved=")); Serial.print(achieved * 100.0f, 3);
    Serial.print(F("% pending=")); Serial.print(pending);

    Serial.print(F(" last_steps=")); Serial.print(last_len);
    Serial.print(F(" last_ms=")); Serial.println(last_ms);

    last_print_ms = now_ms;
  }

  if (Serial.available()) {
    char c = Serial.read();
    if (c == '+') {
      uint8_t new_idx = pwm_level_idx + 1;
      apply_pwm_level(new_idx);
      Serial.print(F("LUT idx -> ")); Serial.println(new_idx);
    } else if (c == '-') {
      uint8_t new_idx = pwm_level_idx - 1;
      apply_pwm_level(new_idx);
      Serial.print(F("LUT idx -> ")); Serial.println(new_idx);
    }
  }
}
