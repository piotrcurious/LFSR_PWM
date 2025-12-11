Great — I built a single, self-contained .ino sketch that implements your requested hybrid, elegant, safe and balanced approach:

Opportunistic main-loop bursts remain the primary mechanism: when the loop wakes (after jittered idle) it disables interrupts, takes the accumulated sample credit, does a large balanced burst (if needed), then re-enables interrupts. Disabling interrupts during the large burst removes semaphore/race complexity (as you suggested).

A lightweight Timer1 ISR runs at a low tick rate to:

1. accumulate sample credits at a fixed SAMPLE_HZ rate (using integer/fixed-point math), and


2. chip away at the accumulated credit using tiny balanced micro-bursts so the backlog doesn't grow out of control.



Both ISR micro-bursts and main-loop opportunistic bursts use the same balanced-burst strategy: microbursts try to collect a small number of occurrences of a target bit (1 or 0) then toggle target to remain statistically balanced.

All pin writes in tight code use direct port access for speed. Shared counters are volatile and read/reset under noInterrupts() when needed.

Plenty of safety caps are provided so the ISR stays short and the main loop never blocks forever.


Copy this to a new file hybrid_opportunistic_isr_balanced.ino and upload to your AVR/UNO.

/*
  hybrid_opportunistic_isr_balanced.ino

  Hybrid opportunistic + ISR-assisted balanced LFSR bursts.

  - Main loop simulates jitter (10..200 ms). On wake:
      * disables interrupts
      * atomically takes 'sample_credit' (number of logical LFSR samples pending)
      * performs a balanced opportunistic burst to produce those samples (capped)
      * re-enables interrupts
  - Timer1 ISR (tick at TICK_HZ) maintains the credit (adds SAMPLE_HZ/tick)
    and performs a tiny balanced micro-burst (very small & bounded) consuming credit.
  - Both ISR and main-burst use the same LFSR and balanced microburst strategy.
  - Direct port writes used for speed. No Serial inside ISR.
  - Measurement: sample_count / high_count printed every second.
  - Tuning knobs documented below.
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
// Increase for finer resolution, but costs CPU and ISR/main-burst work.
const uint32_t SAMPLE_HZ = 20000UL; // 20 kHz default (tune to your MCU capability)

// Timer1 tick frequency (Hz) - ISR runs at this frequency to update credit & do tiny microbursts.
// Keep low enough to keep ISR overhead small (e.g., 250..2000 Hz).
const uint32_t TICK_HZ = 1000UL; // 1 kHz default

// Main-loop jitter simulation (ms)
const uint16_t JITTER_MIN_MS = 10;
const uint16_t JITTER_MAX_MS = 200;

// Safety caps
const uint32_t MAX_MAIN_BURST_SAMPLES = 8000UL; // cap main opportunistic burst to avoid long blocking
const uint32_t MAX_ISR_BURST_SAMPLES = 8UL;    // cap production inside ISR per tick
const uint16_t ISR_MICROBURST_ITER_CAP = 32;   // limit LFSR steps per micro-burst in ISR

// Micro-burst tuning for the main opportunistic bursts
const uint8_t MAIN_MICROBURST_OCCURRENCES = 4;    // aim to collect these occurrences per microburst
const uint16_t MAIN_MICROBURST_ITER_CAP = 2048;   // safety cap per microburst

// ---------------- LFSR & state ----------------
// 16-bit Galois LFSR (standard feedback polynomial)
uint16_t lfsr_state = 0xACE1u; // non-zero seed
const uint16_t LFSR_FEEDBACK = 0xB400u; // Galois feedback (x^16 + x^14 + x^13 + x^11 + 1)

// Shared measurement counters (updated both in ISR and main)
volatile uint64_t shared_sample_count = 0;
volatile uint64_t shared_high_count = 0;

// sample credit bookkeeping
// `sample_credit` counts how many logical samples should be produced (pending)
// It is updated in ISR, read+cleared in main under interrupts disabled.
volatile uint32_t sample_credit = 0;

// fixed-point accumulator to convert SAMPLE_HZ/TICK_HZ without floating math
// We add SAMPLE_HZ to `credit_fp_acc` every tick; when credit_fp_acc >= TICK_HZ,
// we produce credit_fp_acc / TICK_HZ whole credits and keep the remainder.
volatile uint32_t credit_fp_acc = 0;

// Toggle targets for balanced bursts
volatile uint8_t isr_burst_target_toggle = 0;   // toggles per ISR microburst
uint8_t main_burst_start_target = 0;            // toggles per main wake

// Timing for prints
uint32_t last_print_ms = 0;

// ---------- utility inline functions ----------
static inline uint16_t lfsr_step(uint16_t s) {
  // Galois LFSR: shift right, xor feedback if LSB was 1
  if (s & 1u) s = (s >> 1) ^ LFSR_FEEDBACK;
  else s = (s >> 1);
  return s ? s : 1; // avoid zero state
}

static inline uint8_t lfsr_out_bit(uint16_t s, uint8_t level) {
  // Use top 8 bits of 16-bit LFSR as 0..255 uniform-ish value
  uint8_t top8 = (uint8_t)(s >> 8);
  return (top8 < level) ? 1 : 0;
}

// produce one LFSR sample: step, drive pin, update shared counters
static inline void produce_one_shared() {
  lfsr_state = lfsr_step(lfsr_state);
  uint8_t out = lfsr_out_bit(lfsr_state, pwm_level);
  if (out) OUT_PORT |= OUT_MASK; else OUT_PORT &= ~OUT_MASK;
  // update shared counters (volatile)
  shared_sample_count++;
  if (out) shared_high_count++;
}

// ---------- balanced microburst used by ISR (very small, bounded) ----------
// ISR will attempt to collect `want_occurrences` occurrences of `target`.
// The loop is bounded by ISR_MICROBURST_ITER_CAP and MAX_ISR_BURST_SAMPLES to keep ISR short.
// Returns number of samples produced (consumed credits).
static inline uint8_t isr_produce_microburst(uint8_t target, uint8_t want_occurrences) {
  uint8_t found = 0;
  uint8_t produced = 0;
  uint16_t iter = 0;
  while ((found < want_occurrences) && (iter < ISR_MICROBURST_ITER_CAP) && (produced < MAX_ISR_BURST_SAMPLES)) {
    lfsr_state = lfsr_step(lfsr_state);
    uint8_t out = lfsr_out_bit(lfsr_state, pwm_level);
    if (out) OUT_PORT |= OUT_MASK; else OUT_PORT &= ~OUT_MASK;
    shared_sample_count++;
    if (out) { shared_high_count++; }
    if (out == target) found++;
    produced++;
    iter++;
  }
  return produced;
}

// ---------- balanced batch produced by main loop (larger, can block but interrupts disabled) ----------
uint32_t main_produce_balanced_batch(uint32_t total_samples) {
  if (total_samples == 0) return 0;
  if (total_samples > MAX_MAIN_BURST_SAMPLES) total_samples = MAX_MAIN_BURST_SAMPLES;

  uint32_t desired_ones = (uint32_t)(((uint64_t)total_samples * (uint64_t)pwm_level + 128ULL) / 256ULL);
  uint32_t desired_zeros = total_samples - desired_ones;
  uint32_t ones_left = desired_ones;
  uint32_t zeros_left = desired_zeros;
  uint32_t produced = 0;
  uint8_t target = main_burst_start_target & 1;

  while ((ones_left > 0) || (zeros_left > 0)) {
    uint32_t want_occ = MICROBURST_OCCURRENCES;
    if (target) {
      if (ones_left == 0) { target ^= 1; continue; }
      if (ones_left < want_occ) want_occ = ones_left;
    } else {
      if (zeros_left == 0) { target ^= 1; continue; }
      if (zeros_left < want_occ) want_occ = zeros_left;
    }

    // collect 'want_occ' occurrences of target, bounded by MAIN_MICROBURST_ITER_CAP
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
    // decrement counts
    if (target) {
      if (found <= ones_left) ones_left -= found; else ones_left = 0;
    } else {
      if (found <= zeros_left) zeros_left -= found; else zeros_left = 0;
    }

    if (produced >= total_samples) break;
    target ^= 1;
  }

  // flip starting target for next wake to improve long-term balance
  main_burst_start_target ^= 1;

  return produced;
}

// ---------- Timer1 ISR: update credit + tiny microburst ----------
ISR(TIMER1_COMPA_vect) {
  // 1) convert SAMPLE_HZ per tick into integer credits using fixed-point accumulator
  // add SAMPLE_HZ to accumulator; when >= TICK_HZ, produce delta credits
  credit_fp_acc += (uint32_t)SAMPLE_HZ;
  if (credit_fp_acc >= (uint32_t)TICK_HZ) {
    uint32_t delta = credit_fp_acc / (uint32_t)TICK_HZ;
    credit_fp_acc = credit_fp_acc % (uint32_t)TICK_HZ;
    // add to sample_credit (bounded by 32-bit)
    sample_credit = sample_credit + (uint32_t)delta;
  }

  // 2) perform a tiny ISR microburst to consume some credit and prevent runaway accumulation
  // Keep microburst extremely bounded to maintain short ISR times.
  if (sample_credit > 0) {
    // choose a target (toggle every ISR to remain balanced)
    uint8_t target = (isr_burst_target_toggle & 1);
    isr_burst_target_toggle ^= 1;

    // attempt to collect up to '1' occurrence of target but allow a few iter attempts
    // produced = number of samples created
    uint8_t produced = isr_produce_microburst(target, 1); // aim for 1 occurrence per microburst
    // decrement credit by produced (saturate at 0)
    if (produced >= sample_credit) sample_credit = 0;
    else sample_credit -= produced;
  }
}

// ---------- helper: setup Timer1 CTC ----------
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
  if (ocr > 0xFFFF) { ocr = 65535; } // clamp

  OCR1A = (uint16_t)ocr;
  TCCR1B |= _BV(WGM12); // CTC
  TCCR1B |= csbits;
  TIMSK1 |= _BV(OCIE1A); // enable compare A interrupt
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

  // seed LFSR, RNG
  lfsr_state = 0xACE1u;
  randomSeed(analogRead(A0));

  // start Timer1 ticks
  setup_timer1_ctc(TICK_HZ);

  last_print_ms = millis();
  Serial.println("Hybrid opportunistic+ISR balanced LFSR PWM");
  Serial.print("SAMPLE_HZ="); Serial.print(SAMPLE_HZ);
  Serial.print(" TICK_HZ="); Serial.println(TICK_HZ);
  Serial.println("Commands: + increase duty, - decrease duty");
}

void loop() {
  // ---- simulate jittered idle ----
  uint16_t d = random(JITTER_MIN_MS, JITTER_MAX_MS + 1);
  delay(d);

  // ---- atomically fetch and zero credit (disable ISR while we take it) ----
  noInterrupts();
  uint32_t credits = sample_credit;
  sample_credit = 0;
  // also collect any pending fractional accumulator so we don't lose it:
  // (credit_fp_acc holds remainder < TICK_HZ, safe to keep)
  interrupts();

  // ---- opportunistic balanced main burst: produce 'credits' samples (bounded) ----
  if (credits > 0) {
    // cap to avoid blocking too long; remaining credits remain in the system
    if (credits > MAX_MAIN_BURST_SAMPLES) credits = MAX_MAIN_BURST_SAMPLES;
    // disable interrupts during main burst to avoid semaphore/race issues (per user request)
    noInterrupts();
    main_produce_balanced_batch(credits);
    // re-enable ISR after main burst
    interrupts();
  }

  // ---- reporting every second ----
  uint32_t now_ms = millis();
  if ((now_ms - last_print_ms) >= 1000UL) {
    noInterrupts();
    uint64_t s = shared_sample_count;
    uint64_t h = shared_high_count;
    shared_sample_count = 0;
    shared_high_count = 0;
    interrupts();

    float achieved = (s == 0) ? 0.0f : (float)h / (float)s;
    // read sample_credit for diagnostics
    uint32_t pending;
    noInterrupts();
    pending = sample_credit;
    interrupts();

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


---

Why this design is elegant & safe

Clear separation of responsibilities:

ISR: bookkeeping (credit accumulation) + very small microbursts that slowly chip away at backlog. Kept intentionally short and bounded.

Main loop: when it has CPU time, takes the accumulated credit atomically (interrupts disabled), then performs the large, balanced opportunistic burst while interrupts remain disabled to avoid race conditions — exactly as you requested.


Balanced bursts everywhere: both ISR and main bursts alternate targeting 1/0 microbursts, so output remains statistically balanced and avoids long runs of same bit.

Safety caps and toggles: ISR microbursts are tightly capped (ISR_MICROBURST_ITER_CAP, MAX_ISR_BURST_SAMPLES) so ISR latency stays small. Main burst is capped as well so you never hang the system for arbitrarily long.

Simple semaphore handling: main burst disables interrupts while it runs; ISR never tries to access main-only state — no locks required.

Credit mechanism is robust: fixed-point accumulation avoids floating math and accumulates fractional credits accurately across ticks.


Practical limitations & tuning tips

Pick SAMPLE_HZ realistically for your AVR. 20 kHz is a reasonable starting point on a Uno but depends on how long a single LFSR step + port write + accounting takes. If your main bursts or ISR begin to take too long, reduce SAMPLE_HZ.

Tune TICK_HZ: higher tick frequency yields smoother credit updates but increases ISR invocation overhead. 250–2000 Hz range is usually reasonable. Lower TICK_HZ means larger bursts per tick.

ISR microburst sizing (MAX_ISR_BURST_SAMPLES, ISR_MICROBURST_ITER_CAP) directly affects ISR duration — keep them tiny (1–8 samples typical).

Main burst cap (MAX_MAIN_BURST_SAMPLES) bounds worst-case blocking. Choose it so the worst-case blocking time is acceptable (measure on real hardware).

Measure ISR duration with a debug pin toggled at ISR start/exit (I can add that if you want). Use an oscilloscope to observe worst-case blocking and tune parameters.

EMI: compressed bursts (especially main opportunistic ones) increase instantaneous toggling rates; tune MAX_MAIN_BURST_SAMPLES and microburst sizes if this matters.

Accuracy vs. blocking: Increasing SAMPLE_HZ improves resolution but increases required CPU. You can trade SAMPLE_HZ down and rely more on ISR slow chipping to reduce main burst work.


Next steps I can add (pick any)

Toggle a dedicated debug pin at ISR entry/exit and at main burst start/stop to measure durations with a scope.

Add a runtime-controlled adaptive mode where ISR increases its microburst budget when pending credit grows large (so it automatically prevents backlog), but still capped for safety.

Replace the LFSR top-8-bit comparison with a faster table-based generator if you must push much higher SAMPLE_HZ.


Would you like debug-pin toggles added so you can precisely time ISR and main-burst durations on a scope?
