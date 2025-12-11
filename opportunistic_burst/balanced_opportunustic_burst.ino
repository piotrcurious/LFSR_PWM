/*
  opportunistic_lfsr_balanced_burst.ino

  Opportunistic balanced-burst LFSR PWM prototype.

  - Main loop simulates jitter (10..200 ms).
  - On wake it computes how many logical samples were missed and produces
    that many LFSR samples in a balanced sequence of micro-bursts.
  - Balanced micro-bursts alternate target bit (1/0) so bursts don't bias
    long-term statistics and avoid long runs of the same bit.
  - No "simulate LFSR advance" and no held-state decompensation.
*/

#include <Arduino.h>

// Output pin (pin 9 on UNO -> PB1)
const uint8_t OUT_PIN = 9;
#define OUT_PORT PORTB
#define OUT_MASK _BV(PB1)

// Desired duty expressed 0..255 (0%..~100%)
volatile uint8_t pwm_level = 128; // ~50%

// Logical sample rate to approximate over time (Hz)
const uint32_t SAMPLE_HZ = 50000UL; // 50 kHz logical sample rate

// Safety caps
const uint32_t MAX_BURST_SAMPLES = 20000UL; // max samples produced in a single wake
const uint32_t PRINT_INTERVAL_MS = 1000UL; // report every second

// Micro-burst tuning
const uint8_t MICROBURST_OCCURRENCES = 4;   // aim to collect this many occurrences of target per micro-burst
const uint16_t MICROBURST_ITER_CAP = 1024;  // max LFSR steps per micro-burst (safety)

// LFSR (16-bit Galois)
uint16_t lfsr_state = 0xACE1u; // seed, non-zero
const uint16_t LFSR_FEEDBACK = 0xB400u;

// Measurement accumulators
volatile uint64_t sample_count = 0;
volatile uint64_t high_count = 0;

// Timing helpers
uint32_t last_wake_us = 0;
uint32_t last_print_ms = 0;

// Burst toggling so successive wakes start with alternate targets
uint8_t burst_start_target = 0; // 0 or 1, toggled each wake for extra balance

// Random jitter helper
void simulated_sleep_with_jitter() {
  uint16_t d = random(10, 201); // ms
  delay(d);
}

// LFSR Galois step
static inline uint16_t lfsr_step(uint16_t s) {
  if (s & 1u) {
    s = (s >> 1) ^ LFSR_FEEDBACK;
  } else {
    s = (s >> 1);
  }
  return s ? s : 1; // avoid zero state
}

// Map LFSR state -> output bit compared to pwm_level
static inline uint8_t lfsr_out_bit(uint16_t s, uint8_t level) {
  // Top 8 bits give a uniform-ish 0..255 value
  uint8_t top8 = (uint8_t)(s >> 8);
  return (top8 < level) ? 1 : 0;
}

// Produce one LFSR step and write pin, update counters
static inline void produce_one_and_count() {
  lfsr_state = lfsr_step(lfsr_state);
  uint8_t out = lfsr_out_bit(lfsr_state, pwm_level);
  if (out) {
    OUT_PORT |= OUT_MASK;
    high_count++;
  } else {
    OUT_PORT &= ~OUT_MASK;
  }
  sample_count++;
}

// Produce up to 'count' samples in a tight loop (no microburst targeting).
// Returns number produced.
uint32_t produce_raw_samples(uint32_t count) {
  if (count > MAX_BURST_SAMPLES) count = MAX_BURST_SAMPLES;
  for (uint32_t i = 0; i < count; ++i) produce_one_and_count();
  return count;
}

// Produce a "balanced" set of samples approximating desired ones/zeros.
// total_samples: how many logical samples to produce now (capped).
// This function alternates micro-bursts targeting 1 and 0, aiming to reach desired_ones/zeros.
uint32_t produce_balanced_batch(uint32_t total_samples) {
  if (total_samples == 0) return 0;
  if (total_samples > MAX_BURST_SAMPLES) total_samples = MAX_BURST_SAMPLES;

  // Desired counts (rounded)
  uint32_t desired_ones = (uint32_t)(((uint64_t)total_samples * (uint64_t)pwm_level + 128ULL) / 256ULL);
  uint32_t desired_zeros = total_samples - desired_ones;

  uint32_t ones_left = desired_ones;
  uint32_t zeros_left = desired_zeros;
  uint32_t produced = 0;

  uint8_t target = burst_start_target & 1; // start target toggled each wake for extra balance

  while ((ones_left > 0) || (zeros_left > 0)) {
    // pick how many occurrences to aim for in this microburst
    uint32_t want_occ = MICROBURST_OCCURRENCES;
    if (target) {
      if (ones_left == 0) { target ^= 1; continue; } // switch if none left
      if (ones_left < want_occ) want_occ = ones_left;
    } else {
      if (zeros_left == 0) { target ^= 1; continue; }
      if (zeros_left < want_occ) want_occ = zeros_left;
    }

    // collect 'want_occ' occurrences of 'target' bit, with per-microburst safety cap
    uint32_t found = 0;
    uint16_t iter = 0;
    while ((found < want_occ) && (iter < MICROBURST_ITER_CAP)) {
      // step LFSR and output
      lfsr_state = lfsr_step(lfsr_state);
      uint8_t out = lfsr_out_bit(lfsr_state, pwm_level);
      if (out) {
        OUT_PORT |= OUT_MASK;
        high_count++;
      } else {
        OUT_PORT &= ~OUT_MASK;
      }
      sample_count++;
      produced++;
      if (out == target) found++;
      iter++;
      // safety: if produced hits absolute cap, stop everything
      if (produced >= MAX_BURST_SAMPLES) break;
    }

    if (target) {
      // reduce remaining ones by the number of matched ones we found
      if (found <= ones_left) ones_left -= found;
      else ones_left = 0;
    } else {
      if (found <= zeros_left) zeros_left -= found;
      else zeros_left = 0;
    }

    // If microburst loop exited due to iter cap or produced cap and still left to satisfy,
    // continue but note we might not meet the target fully this wake.
    if (produced >= MAX_BURST_SAMPLES) break;

    // toggle target for next microburst to balance runs
    target ^= 1;
  }

  // Toggle starting target for next wake to keep long-term balance across wakes
  burst_start_target ^= 1;

  return produced;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) ;
  pinMode(OUT_PIN, OUTPUT);
  digitalWrite(OUT_PIN, LOW);
  randomSeed(analogRead(A0));
  last_wake_us = micros();
  last_print_ms = millis();

  Serial.println("Opportunistic balanced-burst LFSR PWM prototype");
  Serial.print("Target SAMPLE_HZ = ");
  Serial.println(SAMPLE_HZ);
  Serial.println("Commands: '+' increase duty, '-' decrease duty");
}

void loop() {
  // jittered idle
  simulated_sleep_with_jitter();

  // compute elapsed time
  uint32_t now_us = micros();
  uint32_t elapsed_us = now_us - last_wake_us;
  last_wake_us = now_us;

  // compute number of logical samples to produce
  // samples = floor(elapsed_us * SAMPLE_HZ / 1e6)
  uint64_t numerator = (uint64_t)elapsed_us * (uint64_t)SAMPLE_HZ;
  uint32_t samples_to_produce = (uint32_t)(numerator / 1000000ULL);

  // produce balanced batch representing desired duty
  if (samples_to_produce > 0) {
    produce_balanced_batch(samples_to_produce);
  }

  // periodic reporting
  uint32_t now_ms = millis();
  if ((now_ms - last_print_ms) >= PRINT_INTERVAL_MS) {
    noInterrupts();
    uint64_t s = sample_count;
    uint64_t h = high_count;
    sample_count = 0;
    high_count = 0;
    interrupts();

    float achieved = (s == 0) ? 0.0f : (float)h / (float)s;
    Serial.print("pwm_level="); Serial.print(pwm_level);
    Serial.print(" samples="); Serial.print(s);
    Serial.print(" achieved="); Serial.print(achieved * 100.0f, 3);
    Serial.println("%");
    last_print_ms = now_ms;
  }

  // serial controls
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '+') {
      pwm_level = (uint8_t)(pwm_level + 1);
      Serial.print("pwm_level -> "); Serial.println(pwm_level);
    } else if (c == '-') {
      pwm_level = (uint8_t)(pwm_level - 1);
      Serial.print("pwm_level -> "); Serial.println(pwm_level);
    }
  }
}
