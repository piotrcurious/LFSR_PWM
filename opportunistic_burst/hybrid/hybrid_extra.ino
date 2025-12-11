/*
  hybrid_opportunistic_isr_balanced_flip.ino

  Hybrid opportunistic + ISR-assisted balanced LFSR bursts with exit-state flipping.

  - Main loop: disables interrupts, takes accumulated sample credit, performs a
    balanced opportunistic burst (capped), and attempts to flip the output
    relative to the entering state by doing a bounded number of extra LFSR steps.
  - ISR: increments credit and performs a very small balanced microburst; after
    microburst it attempts a few extra LFSR steps (bounded very small) to flip exit state.
  - All bursts are balanced (alternate 1/0 targets) and bounded to avoid long blocking.
  - Direct port writes used for speed; shared counters are volatile.
*/

#include <Arduino.h>
#include <avr/interrupt.h>

// ---------------- CONFIGURATION ----------------

// Output pin: UNO pin 9 -> PORTB1
const uint8_t OUT_PIN = 9;
#define OUT_PORT PORTB
#define OUT_MASK _BV(PB1)

// Desired duty expressed 0..255 (0%..~100%)
volatile uint8_t pwm_level = 128; // ~50%

// Logical sample rate (targeted long-term, Hz).
const uint32_t SAMPLE_HZ = 20000UL; // 20 kHz default

// Timer1 tick frequency (Hz) - ISR runs at this frequency to update credit & do tiny microbursts.
const uint32_t TICK_HZ = 1000UL; // 1 kHz default

// Main-loop jitter simulation (ms)
const uint16_t JITTER_MIN_MS = 10;
const uint16_t JITTER_MAX_MS = 200;

// Safety caps
const uint32_t MAX_MAIN_BURST_SAMPLES = 8000UL;  // cap main opportunistic burst
const uint32_t MAX_ISR_BURST_SAMPLES = 8UL;     // cap production inside ISR per tick

// Micro-burst tuning for main opportunistic bursts
const uint8_t MAIN_MICROBURST_OCCURRENCES = 4;    // occurrences per microburst
const uint16_t MAIN_MICROBURST_ITER_CAP = 2048;   // safety cap per microburst

// ISR microburst caps and flip attempt cap (must be tiny)
const uint8_t ISR_MICROBURST_OCCURRENCES = 1;     // aim per tiny microburst
const uint8_t ISR_MICROBURST_ITER_CAP = 32;       // tries inside microburst
const uint8_t ISR_EXIT_FLIP_CAP = 4;             // extra steps to try flip before leaving ISR

// Main exit flip cap (main can do slightly more work)
const uint16_t MAIN_EXIT_FLIP_CAP = 64;

// ---------------- LFSR & state ----------------
uint16_t lfsr_state = 0xACE1u; // seed
const uint16_t LFSR_FEEDBACK = 0xB400u; // standard Galois feedback

// Shared measurement counters (updated both in ISR and main)
volatile uint64_t shared_sample_count = 0;
volatile uint64_t shared_high_count = 0;

// sample credit bookkeeping
volatile uint32_t sample_credit = 0;
volatile uint32_t credit_fp_acc = 0; // fixed-point accumulator

volatile uint8_t isr_burst_target_toggle = 0;   // toggles per ISR microburst
uint8_t main_burst_start_target = 0;            // toggles per main wake

uint32_t last_print_ms = 0;

// ---------- utility inline functions ----------
static inline uint16_t lfsr_step(uint16_t s) {
  if (s & 1u) s = (s >> 1) ^ LFSR_FEEDBACK;
  else s = (s >> 1);
  return s ? s : 1;
}

static inline uint8_t lfsr_out_bit(uint16_t s, uint8_t level) {
  uint8_t top8 = (uint8_t)(s >> 8);
  return (top8 < level) ? 1 : 0;
}

// read current physical output bit (0/1)
static inline uint8_t read_current_output_bit() {
  return (OUT_PORT & OUT_MASK) ? 1 : 0;
}

// produce one sample and update shared counters (used by both main and ISR microbursts)
static inline void produce_one_shared_no_interrupts() {
  lfsr_state = lfsr_step(lfsr_state);
  uint8_t out = lfsr_out_bit(lfsr_state, pwm_level);
  if (out) OUT_PORT |= OUT_MASK; else OUT_PORT &= ~OUT_MASK;
  shared_sample_count++;
  if (out) shared_high_count++;
}

// ---------- ISR microburst: tiny, bounded, balanced, attempts exit flip ----------
static inline uint8_t isr_produce_microburst_and_flip(uint8_t target, uint8_t want_occurrences) {
  uint8_t found = 0;
  uint8_t produced = 0;
  uint16_t iter = 0;

  // record entering state
  uint8_t entering = read_current_output_bit();

  while ((found < want_occurrences) && (iter < ISR_MICROBURST_ITER_CAP) && (produced < MAX_ISR_BURST_SAMPLES)) {
    lfsr_state = lfsr_step(lfsr_state);
    uint8_t out = lfsr_out_bit(lfsr_state, pwm_level);
    if (out) OUT_PORT |= OUT_MASK; else OUT_PORT &= ~OUT_MASK;
    shared_sample_count++;
    if (out) shared_high_count++;
    if (out == target) found++;
    produced++;
    iter++;
  }

  // attempt a very small number of extra steps to flip exit state if still equal to entering
  uint8_t exit_bit = read_current_output_bit();
  uint8_t flip_attempts = 0;
  while ((exit_bit == entering) && (flip_attempts < ISR_EXIT_FLIP_CAP) && (produced < MAX_ISR_BURST_SAMPLES)) {
    lfsr_state = lfsr_step(lfsr_state);
    uint8_t out = lfsr_out_bit(lfsr_state, pwm_level);
    if (out) OUT_PORT |= OUT_MASK; else OUT_PORT &= ~OUT_MASK;
    shared_sample_count++;
    if (out) shared_high_count++;
    produced++;
    flip_attempts++;
    exit_bit = read_current_output_bit();
  }

  return produced;
}

// ---------- main balanced batch: larger, can disable interrupts; attempts exit flip ----------
uint32_t main_produce_balanced_batch_and_flip(uint32_t total_samples) {
  if (total_samples == 0) return 0;
  if (total_samples > MAX_MAIN_BURST_SAMPLES) total_samples = MAX_MAIN_BURST_SAMPLES;

  uint32_t desired_ones = (uint32_t)(((uint64_t)total_samples * (uint64_t)pwm_level + 128ULL) / 256ULL);
  uint32_t desired_zeros = total_samples - desired_ones;
  uint32_t ones_left = desired_ones;
  uint32_t zeros_left = desired_zeros;
  uint32_t produced = 0;
  uint8_t target = main_burst_start_target & 1;

  // entering state (read port)
  uint8_t entering = read_current_output_bit();

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
      lfsr_state = lfsr_step(lfsr_state);
      uint8_t out = lfsr_out_bit(lfsr_state, pwm_level);
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

  // Attempt exit flip with a bounded number of extra iterations
  uint16_t flip_iters = 0;
  uint8_t exit_bit = read_current_output_bit();
  while ((exit_bit == entering) && (flip_iters < MAIN_EXIT_FLIP_CAP) && (produced < MAX_MAIN_BURST_SAMPLES)) {
    lfsr_state = lfsr_step(lfsr_state);
    uint8_t out = lfsr_out_bit(lfsr_state, pwm_level);
    if (out) OUT_PORT |= OUT_MASK; else OUT_PORT &= ~OUT_MASK;
    shared_sample_count++;
    if (out) shared_high_count++;
    produced++;
    flip_iters++;
    exit_bit = read_current_output_bit();
  }

  // flip starting target for next wake
  main_burst_start_target ^= 1;

  return produced;
}

// ---------- Timer1 ISR: update credit + tiny microburst + exit flip ----------
ISR(TIMER1_COMPA_vect) {
  // update fixed-point accumulator to add SAMPLE_HZ per tick
  credit_fp_acc += (uint32_t)SAMPLE_HZ;
  if (credit_fp_acc >= (uint32_t)TICK_HZ) {
    uint32_t delta = credit_fp_acc / (uint32_t)TICK_HZ;
    credit_fp_acc = credit_fp_acc % (uint32_t)TICK_HZ;
    sample_credit = sample_credit + delta;
  }

  // tiny balanced microburst to gradually consume credit
  if (sample_credit > 0) {
    uint8_t target = (isr_burst_target_toggle & 1);
    isr_burst_target_toggle ^= 1;

    uint8_t produced = isr_produce_microburst_and_flip(target, ISR_MICROBURST_OCCURRENCES);

    // decrement credit safely
    if ((uint32_t)produced >= sample_credit) sample_credit = 0;
    else sample_credit -= produced;
  }
}

// ---------- Timer setup ----------
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
  if (ocr > 0xFFFF) { ocr = 65535; }
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

// ---------- setup / loop ----------
void setup() {
  Serial.begin(115200);
  while (!Serial) ;
  pinMode(OUT_PIN, OUTPUT);
  digitalWrite(OUT_PIN, LOW);

  // seed lfsr and random
  lfsr_state = 0xACE1u;
  randomSeed(analogRead(A0));

  setup_timer1_ctc(TICK_HZ);
  last_print_ms = millis();

  Serial.println("Hybrid opportunistic+ISR balanced LFSR PWM (exit-flip enabled)");
  Serial.print("SAMPLE_HZ="); Serial.print(SAMPLE_HZ);
  Serial.print(" TICK_HZ="); Serial.println(TICK_HZ);
  Serial.println("Commands: + increase duty, - decrease duty");
}

void loop() {
  // simulated jitter idle
  uint16_t d = random(JITTER_MIN_MS, JITTER_MAX_MS + 1);
  delay(d);

  // atomically fetch & clear credit
  noInterrupts();
  uint32_t credits = sample_credit;
  sample_credit = 0;
  interrupts();

  // opportunistic main burst (disable interrupts during burst as requested)
  if (credits > 0) {
    if (credits > MAX_MAIN_BURST_SAMPLES) credits = MAX_MAIN_BURST_SAMPLES;
    noInterrupts();
    main_produce_balanced_batch_and_flip(credits);
    interrupts();
  }

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
    Serial.print("pwm=");
    Serial.print(pwm_level);
    Serial.print(" samples=");
    Serial.print(s);
    Serial.print(" achieved=");
    Serial.print(achieved * 100.0f, 3);
    Serial.print("% pendingCredits=");
    Serial.println(pending);

    last_print_ms = now_ms;
  }

  // serial control
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '+') {
      pwm_level = (uint8_t)(pwm_level + 1);
      Serial.print("pwm -> "); Serial.println(pwm_level);
    } else if (c == '-') {
      pwm_level = (uint8_t)(pwm_level - 1);
      Serial.print("pwm -> "); Serial.println(pwm_level);
    }
  }
}
