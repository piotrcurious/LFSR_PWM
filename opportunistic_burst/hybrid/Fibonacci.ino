/*
  hybrid_fibonacci_isr_limited_disable.ino

  Hybrid opportunistic + ISR-assisted balanced LFSR bursts using a Fibonacci LFSR.
  During opportunistic main bursts the sketch disables *only* the Timer1 compare
  interrupt (microburst ISR) so the ISR microbursts do not run while the main
  burst is producing samples. This reduces race complexity while keeping other
  hardware interrupts (if any) available.

  Key changes from previous Galois version:
   - Fibonacci LFSR implemented via parity(mask & state) and right shift.
   - Main burst disables TIMSK1 OCIE1A only (not global interrupts) while copying
     and zeroing credits and while performing the main burst.
   - Exit-state flipping is preserved.
*/

#include <Arduino.h>
#include <avr/interrupt.h>

// ---------- CONFIGURATION ----------
const uint8_t OUT_PIN = 9;                   // UNO pin 9 = PORTB1
#define OUT_PORT PORTB
#define OUT_MASK _BV(PB1)

// PWM level (0..255)
volatile uint8_t pwm_level = 128;           // ~50%

// Logical sample rate to emulate (Hz)
const uint32_t SAMPLE_HZ = 20000UL;         // tune to your MCU capability

// Timer1 tick rate (Hz) for crediting and tiny ISR microbursts
const uint32_t TICK_HZ = 1000UL;            // 1 kHz default

// Jitter simulation (ms)
const uint16_t JITTER_MIN_MS = 10;
const uint16_t JITTER_MAX_MS = 200;

// Safety caps
const uint32_t MAX_MAIN_BURST_SAMPLES = 8000UL;  // cap for main opportunistic burst
const uint32_t MAX_ISR_BURST_SAMPLES = 8UL;      // cap for ISR microburst (samples)
const uint16_t ISR_MICROBURST_ITER_CAP = 32;     // limit LFSR steps in ISR microburst

// Main microburst tuning
const uint8_t MAIN_MICROBURST_OCCURRENCES = 4;
const uint16_t MAIN_MICROBURST_ITER_CAP = 2048;
const uint16_t MAIN_EXIT_FLIP_CAP = 64;          // extra steps to try flip at end of main burst

// ISR microburst flip attempts (tiny)
const uint8_t ISR_EXIT_FLIP_CAP = 4;

// ---------- LFSR (Fibonacci) ----------
volatile uint16_t lfsr_state = 0xA5A5u;    // non-zero seed (volatile because used in ISR)
const uint16_t LFSR_MASK = 0xB400u;        // taps mask for parity (example polynomial)

// ---------- bookkeeping ----------
volatile uint64_t shared_sample_count = 0; // samples produced (ISR+main)
volatile uint64_t shared_high_count = 0;   // number of '1' samples

// credit (how many logical samples pending to be produced)
volatile uint32_t sample_credit = 0;

// fixed-point accumulator for SAMPLE_HZ / TICK_HZ conversion
volatile uint32_t credit_fp_acc = 0;

// toggles to alternate microburst targets
volatile uint8_t isr_burst_target_toggle = 0;
uint8_t main_burst_start_target = 0;

// reporting
uint32_t last_print_ms = 0;

// ---------- small utility: parity of 16-bit (Fibonacci feedback) ----------
static inline uint8_t parity16(uint16_t x) {
  x ^= x >> 8;
  x ^= x >> 4;
  x ^= x >> 2;
  x ^= x >> 1;
  return x & 1;
}

// ---------- helpers ----------

// read current output logic level (0 or 1)
static inline uint8_t read_output_bit() {
  return (OUT_PORT & OUT_MASK) ? 1 : 0;
}

// produce one Fibonacci LFSR step, write pin, update counters
// This is used both by ISR and main burst code.
static inline void produce_one_sample_shared() {
  // Fibonacci right-shift LFSR:
  // feedback = parity(state & LFSR_MASK)
  // state = (state >> 1) | (feedback << 15)
  uint16_t s = lfsr_state;
  uint16_t fb = (uint16_t)parity16((uint16_t)(s & LFSR_MASK));
  s = (uint16_t)((s >> 1) | (fb << 15));
  if (s == 0) s = 1; // avoid zero state
  lfsr_state = s;

  // map top 8 bits to output decision
  uint8_t top8 = (uint8_t)(s >> 8);
  uint8_t out = (top8 < pwm_level) ? 1 : 0;

  if (out) OUT_PORT |= OUT_MASK; else OUT_PORT &= ~OUT_MASK;

  shared_sample_count++;
  if (out) shared_high_count++;
}

// ---------- ISR microburst (tiny, balanced, attempts flip) ----------
static inline uint8_t isr_microburst_and_flip(uint8_t target, uint8_t want_occurrences) {
  uint8_t found = 0;
  uint8_t produced = 0;
  uint16_t iter = 0;
  uint8_t entering = read_output_bit();

  while ((found < want_occurrences) && (iter < ISR_MICROBURST_ITER_CAP) && (produced < MAX_ISR_BURST_SAMPLES)) {
    // produce one Fibonacci sample and update counters
    uint16_t s = lfsr_state;
    uint16_t fb = (uint16_t)parity16((uint16_t)(s & LFSR_MASK));
    s = (uint16_t)((s >> 1) | (fb << 15));
    if (s == 0) s = 1;
    lfsr_state = s;

    uint8_t top8 = (uint8_t)(s >> 8);
    uint8_t out = (top8 < pwm_level) ? 1 : 0;
    if (out) OUT_PORT |= OUT_MASK; else OUT_PORT &= ~OUT_MASK;
    shared_sample_count++;
    if (out) shared_high_count++;
    if (out == target) found++;
    produced++;
    iter++;
  }

  // tiny bounded attempts to flip exit state if it's equal to entering
  uint8_t exit_bit = read_output_bit();
  uint8_t flip_attempts = 0;
  while ((exit_bit == entering) && (flip_attempts < ISR_EXIT_FLIP_CAP) && (produced < MAX_ISR_BURST_SAMPLES)) {
    uint16_t s = lfsr_state;
    uint16_t fb = (uint16_t)parity16((uint16_t)(s & LFSR_MASK));
    s = (uint16_t)((s >> 1) | (fb << 15));
    if (s == 0) s = 1;
    lfsr_state = s;

    uint8_t top8 = (uint8_t)(s >> 8);
    uint8_t out = (top8 < pwm_level) ? 1 : 0;
    if (out) OUT_PORT |= OUT_MASK; else OUT_PORT &= ~OUT_MASK;
    shared_sample_count++;
    if (out) shared_high_count++;
    produced++;
    flip_attempts++;
    exit_bit = read_output_bit();
  }

  return produced;
}

// ---------- main balanced batch with exit-flip (runs with microburst ISR disabled) ----------
uint32_t main_produce_balanced_batch_and_flip(uint32_t total_samples) {
  if (total_samples == 0) return 0;
  if (total_samples > MAX_MAIN_BURST_SAMPLES) total_samples = MAX_MAIN_BURST_SAMPLES;

  uint32_t desired_ones = (uint32_t)(((uint64_t)total_samples * (uint64_t)pwm_level + 128ULL) / 256ULL);
  uint32_t desired_zeros = total_samples - desired_ones;
  uint32_t ones_left = desired_ones;
  uint32_t zeros_left = desired_zeros;
  uint32_t produced = 0;
  uint8_t target = main_burst_start_target & 1;
  uint8_t entering = read_output_bit();

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
      uint16_t s = lfsr_state;
      uint16_t fb = (uint16_t)parity16((uint16_t)(s & LFSR_MASK));
      s = (uint16_t)((s >> 1) | (fb << 15));
      if (s == 0) s = 1;
      lfsr_state = s;

      uint8_t top8 = (uint8_t)(s >> 8);
      uint8_t out = (top8 < pwm_level) ? 1 : 0;
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
  uint8_t exit_bit = read_output_bit();
  while ((exit_bit == entering) && (flip_iters < MAIN_EXIT_FLIP_CAP) && (produced < MAX_MAIN_BURST_SAMPLES)) {
    uint16_t s = lfsr_state;
    uint16_t fb = (uint16_t)parity16((uint16_t)(s & LFSR_MASK));
    s = (uint16_t)((s >> 1) | (fb << 15));
    if (s == 0) s = 1;
    lfsr_state = s;

    uint8_t top8 = (uint8_t)(s >> 8);
    uint8_t out = (top8 < pwm_level) ? 1 : 0;
    if (out) OUT_PORT |= OUT_MASK; else OUT_PORT &= ~OUT_MASK;
    shared_sample_count++;
    if (out) shared_high_count++;
    produced++;
    flip_iters++;
    exit_bit = read_output_bit();
  }

  main_burst_start_target ^= 1;
  return produced;
}

// ---------- Timer1 ISR: credit accumulation + tiny microburst ----------
ISR(TIMER1_COMPA_vect) {
  // accumulate SAMPLE_HZ per tick and convert to integer credits using TICK_HZ
  credit_fp_acc += (uint32_t)SAMPLE_HZ;
  if (credit_fp_acc >= (uint32_t)TICK_HZ) {
    uint32_t delta = credit_fp_acc / (uint32_t)TICK_HZ;
    credit_fp_acc = credit_fp_acc % (uint32_t)TICK_HZ;
    sample_credit = sample_credit + delta;
  }

  // tiny balanced microburst to consume a little credit if present
  if (sample_credit > 0) {
    uint8_t target = (isr_burst_target_toggle & 1);
    isr_burst_target_toggle ^= 1;

    uint8_t produced = isr_microburst_and_flip(target, 1); // aim for 1 occurrence
    if ((uint32_t)produced >= sample_credit) sample_credit = 0;
    else sample_credit -= produced;
  }
}

// ---------- Timer1 setup ----------
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

  // LFSR seed and RNG
  lfsr_state = 0xA5A5u;
  randomSeed(analogRead(A0));

  setup_timer1_ctc(TICK_HZ);
  last_print_ms = millis();

  Serial.println("Hybrid Fibonacci-LFSR balanced PWM (disable only microburst ISR during main burst)");
  Serial.print("SAMPLE_HZ="); Serial.print(SAMPLE_HZ);
  Serial.print(" TICK_HZ="); Serial.println(TICK_HZ);
  Serial.println("Commands: + increase duty, - decrease duty");
}

void loop() {
  // simulate jitter
  uint16_t d = random(JITTER_MIN_MS, JITTER_MAX_MS + 1);
  delay(d);

  // -- disable only the Timer1 compare interrupt (microburst ISR), and atomically take credits --
  noInterrupts();
  TIMSK1 &= ~_BV(OCIE1A);        // disable Timer1 compare interrupt (microburst)
  uint32_t credits = sample_credit;
  sample_credit = 0;
  interrupts();
  // At this point the microburst ISR won't run (OCIE1A cleared). If an ISR was already executing
  // it will finish; future ISR invocations are prevented until we set OCIE1A again.

  // perform main opportunistic balanced burst while microburst ISR disabled
  if (credits > 0) {
    if (credits > MAX_MAIN_BURST_SAMPLES) credits = MAX_MAIN_BURST_SAMPLES;
    // we can perform bursting now - interrupts are allowed except microburst ISR
    // for extra safety we can still prevent nested interrupts during tight loops if desired,
    // but here we rely on OCIE1A being cleared as requested.
    main_produce_balanced_batch_and_flip(credits);
  }

  // re-enable Timer1 compare interrupt (microburst ISR)
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

  // serial controls
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
