/*
  hybrid_fibonacci_lut_replace.ino

  Hybrid opportunistic + ISR-assisted balanced PWM using a Fibonacci LFSR, reading
  per-level mask/seed (threshold) from PROGMEM LUT (arduino_lut.h). Designed as
  an enhanced drop-in replacement for your original LUT-based LFSR PWM.

  - Uses only the Timer1 Compare interrupt (OCIE1A) as the microburst ISR.
  - During opportunistic main bursts the code disables only OCIE1A so the ISR
    does not run while main burst is producing samples (avoids races with microburst).
  - LFSR step is Fibonacci: feedback = parity(state & mask), state = (state >> 1) | (fb << 15)
  - Output decision is threshold-based: out = (lfsr_state > lfsr_threshold) ? 1 : 0
  - Reads mask and seed from PROGMEM LUT via load_lfsr_params(level, &seed).
  - Balanced microbursts and exit-state flip logic preserved.
*/

#include <Arduino.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

// include your LUT header (must provide lfsr_pwm_lut[256].mask and .seed in PROGMEM)
#include "arduino_lut.h" // <-- must be present in project

// ---------- CONFIG ----------
const uint8_t OUT_PIN = 9;            // UNO pin 9 = PORTB1
#define OUT_PORT PORTB
#define OUT_MASK _BV(PB1)

// PWM LUT size (expected 256)
#define LFSR_LUT_SIZE 256

// runtime PWM index (0..255), settable by serial or code
volatile uint8_t pwm_level_idx = 128; // default LUT index ~50%

// Logical sample & tick rates
const uint32_t SAMPLE_HZ = 20000UL;   // logical LFSR sample rate (target)
const uint32_t TICK_HZ = 1000UL;      // Timer1 tick rate for crediting + microbursts

// jitter simulation (main loop)
const uint16_t JITTER_MIN_MS = 10;
const uint16_t JITTER_MAX_MS = 200;

// safety caps
const uint32_t MAX_MAIN_BURST_SAMPLES = 8000UL;  // max samples main will produce in one wake
const uint32_t MAX_ISR_BURST_SAMPLES = 8UL;      // max samples ISR may produce in one tick
const uint16_t ISR_MICROBURST_ITER_CAP = 32;     // LFSR iteration cap inside ISR microburst

// main microburst tuning
const uint8_t MAIN_MICROBURST_OCCURRENCES = 4;
const uint16_t MAIN_MICROBURST_ITER_CAP = 2048;
const uint16_t MAIN_EXIT_FLIP_CAP = 64;          // extra attempts to flip exit state in main

// ISR tiny flip attempts
const uint8_t ISR_EXIT_FLIP_CAP = 4;

// ---------- LFSR state (Fibonacci) ----------
volatile uint16_t lfsr_state = 1;           // current state (volatile because used in ISR)
volatile uint16_t lfsr_mask_current = 0xB400u;   // mask (from LUT)
volatile uint16_t lfsr_threshold = 0;       // threshold/seed (from LUT)

// ---------- bookkeeping ----------
volatile uint64_t shared_sample_count = 0;
volatile uint64_t shared_high_count = 0;

volatile uint32_t sample_credit = 0;        // accumulated logical samples to produce
volatile uint32_t credit_fp_acc = 0;        // fixed-point accumulator for samples per tick

volatile uint8_t isr_burst_target_toggle = 0;
uint8_t main_burst_start_target = 0;

uint32_t last_print_ms = 0;

// ---------- parity function ----------
inline uint8_t parity16(uint16_t x) {
  x ^= x >> 8;
  x ^= x >> 4;
  x ^= x >> 2;
  x ^= x >> 1;
  return x & 1;
}

// ---------- LUT load utility ----------
uint16_t load_lfsr_params(uint8_t level_index, uint16_t *seed_out) {
  uint8_t idx = level_index; // ensures 0..255
  uint16_t mask = pgm_read_word_near(&lfsr_pwm_lut[idx].mask);
  uint16_t seed = pgm_read_word_near(&lfsr_pwm_lut[idx].seed);
  *seed_out = seed;
  return mask;
}

// Apply PWM LUT index (loads mask + seed -> threshold, sets LFSR state)
void apply_pwm_level(uint8_t level_idx) {
  // Small critical section while updating LFSR params to avoid ISR races.
  noInterrupts();
  uint16_t new_seed = 0;
  uint16_t new_mask = load_lfsr_params(level_idx, &new_seed);

  lfsr_mask_current = new_mask;
  lfsr_threshold = new_seed;
  lfsr_state = new_seed ? new_seed : 1; // ensure non-zero
  pwm_level_idx = level_idx;
  interrupts();
}

// ---------- low-level LFSR step & output (Fibonacci) ----------
static inline uint8_t lfsr_step_and_out() {
  // compute feedback = parity(state & mask)
  uint16_t s = lfsr_state;
  uint16_t fb = (uint16_t)parity16((uint16_t)(s & lfsr_mask_current));
  s = (uint16_t)((s >> 1) | (fb << 15));
  if (s == 0) s = 1;
  lfsr_state = s;

  // output decision uses threshold comparison (seed/threshold from LUT)
  uint8_t out = (s > lfsr_threshold) ? 1 : 0;
  return out;
}

// ---------- ISR microburst (tiny, balanced, and try exit flip) ----------
static inline uint8_t isr_microburst_and_flip(uint8_t target, uint8_t want_occurrences) {
  uint8_t found = 0;
  uint8_t produced = 0;
  uint16_t iter = 0;

  // entering state
  uint8_t entering = (OUT_PORT & OUT_MASK) ? 1 : 0;

  while ((found < want_occurrences) && (iter < ISR_MICROBURST_ITER_CAP) && (produced < MAX_ISR_BURST_SAMPLES)) {
    uint8_t out = lfsr_step_and_out();
    if (out) OUT_PORT |= OUT_MASK; else OUT_PORT &= ~OUT_MASK;
    shared_sample_count++;
    if (out) shared_high_count++;
    if (out == target) found++;
    produced++;
    iter++;
  }

  // tiny bounded exit flip attempts
  uint8_t exit_bit = (OUT_PORT & OUT_MASK) ? 1 : 0;
  uint8_t flip_attempts = 0;
  while ((exit_bit == entering) && (flip_attempts < ISR_EXIT_FLIP_CAP) && (produced < MAX_ISR_BURST_SAMPLES)) {
    uint8_t out = lfsr_step_and_out();
    if (out) OUT_PORT |= OUT_MASK; else OUT_PORT &= ~OUT_MASK;
    shared_sample_count++;
    if (out) shared_high_count++;
    produced++;
    flip_attempts++;
    exit_bit = (OUT_PORT & OUT_MASK) ? 1 : 0;
  }

  return produced;
}

// ---------- main balanced batch with flip (run while microburst ISR disabled only) ----------
uint32_t main_produce_balanced_batch_and_flip(uint32_t total_samples) {
  if (total_samples == 0) return 0;
  if (total_samples > MAX_MAIN_BURST_SAMPLES) total_samples = MAX_MAIN_BURST_SAMPLES;

  uint32_t desired_ones = (uint32_t)(((uint64_t)total_samples * (uint64_t)pwm_level_idx + 128ULL) / 256ULL);
  uint32_t desired_zeros = total_samples - desired_ones;
  uint32_t ones_left = desired_ones;
  uint32_t zeros_left = desired_zeros;
  uint32_t produced = 0;
  uint8_t target = main_burst_start_target & 1;

  uint8_t entering = (OUT_PORT & OUT_MASK) ? 1 : 0;

  while ((ones_left > 0) || (zeros_left > 0)) {
    uint32_t want_occ = MAIN_MICROBURST_OCCURRENCES;
    if (target) {
      if (ones_left == 0) { target ^= 1; continue; }
      if (ones_left < want_occ) want_occ = ones_left;
    } else {
      if (zeros_left == 0) { target ^= 1; continue; }
      if (zeros_left < want_occ) want_occ = zeros_left;
    }

    uint32_t found = 0;
    uint32_t iter = 0;
    while ((found < want_occ) && (iter < MAIN_MICROBURST_ITER_CAP) && (produced < total_samples)) {
      uint8_t out = lfsr_step_and_out();
      if (out) OUT_PORT |= OUT_MASK; else OUT_PORT &= ~OUT_MASK;
      shared_sample_count++;
      if (out) shared_high_count++;
      if (out == target) found++;
      produced++;
      iter++;
    }

    if (target) {
      if (found <= ones_left) ones_left -= found; else ones_left = 0;
    } else {
      if (found <= zeros_left) zeros_left -= found; else zeros_left = 0;
    }

    if (produced >= total_samples) break;
    target ^= 1;
  }

  // bounded exit flip attempts
  uint16_t flip_iters = 0;
  uint8_t exit_bit = (OUT_PORT & OUT_MASK) ? 1 : 0;
  while ((exit_bit == entering) && (flip_iters < MAIN_EXIT_FLIP_CAP) && (produced < MAX_MAIN_BURST_SAMPLES)) {
    uint8_t out = lfsr_step_and_out();
    if (out) OUT_PORT |= OUT_MASK; else OUT_PORT &= ~OUT_MASK;
    shared_sample_count++;
    if (out) shared_high_count++;
    produced++;
    flip_iters++;
    exit_bit = (OUT_PORT & OUT_MASK) ? 1 : 0;
  }

  // toggle starting target for next wake to help long-term balance
  main_burst_start_target ^= 1;

  return produced;
}

// ---------- Timer1 ISR: credit accrual + tiny microburst ----------
ISR(TIMER1_COMPA_vect) {
  // accumulate SAMPLE_HZ per tick using fixed-point method (avoid float)
  credit_fp_acc += (uint32_t)SAMPLE_HZ;
  if (credit_fp_acc >= (uint32_t)TICK_HZ) {
    uint32_t delta = credit_fp_acc / (uint32_t)TICK_HZ;
    credit_fp_acc = credit_fp_acc % (uint32_t)TICK_HZ;
    sample_credit = sample_credit + delta;
  }

  // tiny balanced microburst to chip away credit
  if (sample_credit > 0) {
    uint8_t target = (isr_burst_target_toggle & 1);
    isr_burst_target_toggle ^= 1;

    uint8_t produced = isr_microburst_and_flip(target, 1); // aim for 1 occurrence
    if ((uint32_t)produced >= sample_credit) sample_credit = 0;
    else sample_credit -= produced;
  }
}

// ---------- Timer1 CTC setup ----------
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
  if (ocr > 0xFFFF) ocr = 65535;
  OCR1A = (uint16_t)ocr;
  TCCR1B |= _BV(WGM12);
  TCCR1B |= csbits;
  TIMSK1 |= _BV(OCIE1A);
  interrupts();
}

// ---------- setup / loop ----------
void setup() {
  Serial.begin(115200);
  while (!Serial) ;
  pinMode(OUT_PIN, OUTPUT);
  digitalWrite(OUT_PIN, LOW);

  // initial LUT-based config
  apply_pwm_level(pwm_level_idx);

  // seed random for jitter
  randomSeed(analogRead(A0));

  setup_timer1_ctc(TICK_HZ);

  last_print_ms = millis();
  Serial.println("Hybrid Fibonacci-LFSR LUT PWM (drop-in replacement)");
  Serial.print("Initial LUT idx = "); Serial.println(pwm_level_idx);
  Serial.print("SAMPLE_HZ="); Serial.print(SAMPLE_HZ);
  Serial.print(" TICK_HZ="); Serial.println(TICK_HZ);
  Serial.println("Commands: + increase LUT index, - decrease LUT index");
}

void loop() {
  // simulate jittered idle
  uint16_t d = random(JITTER_MIN_MS, JITTER_MAX_MS + 1);
  delay(d);

  // disable only Timer1 Compare interrupt (OCIE1A) and atomically take credits
  noInterrupts();                 // brief global disable while toggling TIMSK1
  TIMSK1 &= ~_BV(OCIE1A);         // disable microburst ISR
  uint32_t credits = sample_credit;
  sample_credit = 0;
  interrupts();

  // main opportunistic burst (balanced) while microburst ISR is disabled
  if (credits > 0) {
    if (credits > MAX_MAIN_BURST_SAMPLES) credits = MAX_MAIN_BURST_SAMPLES;
    // produce balanced samples
    main_produce_balanced_batch_and_flip(credits);
  }

  // re-enable Timer1 compare interrupt
  noInterrupts();
  TIMSK1 |= _BV(OCIE1A);
  interrupts();

  // periodic reporting
  uint32_t now_ms = millis();
  if ((now_ms - last_print_ms) >= 1000UL) {
    noInterrupts();
    uint64_t s = shared_sample_count;
    uint64_t h = shared_high_count;
    shared_sample_count = 0;
    shared_high_count = 0;
    uint32_t pending = sample_credit;
    interrupts();

    float achieved = (s == 0) ? 0.0f : (float)h / (float)s;
    Serial.print("LUT_idx="); Serial.print(pwm_level_idx);
    Serial.print(" samples="); Serial.print(s);
    Serial.print(" achieved="); Serial.print(achieved * 100.0f, 3);
    Serial.print("% pendingCredits=");
    Serial.println(pending);

    last_print_ms = now_ms;
  }

  // Serial control to change LUT index (and reload mask/seed)
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '+') {
      uint8_t new_idx = (uint8_t)(pwm_level_idx + 1);
      apply_pwm_level(new_idx);
      Serial.print("LUT idx -> "); Serial.println(new_idx);
    } else if (c == '-') {
      uint8_t new_idx = (uint8_t)(pwm_level_idx - 1);
      apply_pwm_level(new_idx);
      Serial.print("LUT idx -> "); Serial.println(new_idx);
    }
  }
}
