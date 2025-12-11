/*
  opportunistic_lfsr_burst_accounting.ino

  Opportunistic LFSR burst with held-state accounting.

  Behavior:
  - During idle the physical pin remains at last_out.
  - On wake we credit idle_time_samples as last_out samples.
  - We compute desired number of '1's for the idle interval and produce
    additional LFSR-generated '1' samples to compensate the deficit.
  - Optionally we can advance the LFSR state during idle (SIMULATE_LFSR_ADVANCE).
*/

#include <Arduino.h>

// Output pin (pin 9 on UNO -> PB1)
const uint8_t OUT_PIN = 9;
#define OUT_PORT PORTB
#define OUT_MASK _BV(PB1)

// Desired duty expressed 0..255 (0%..~100%)
volatile uint8_t pwm_level = 128; // ~50%

// Target logical sample rate (Hz)
const uint32_t SAMPLE_HZ = 50000UL; // logical sample rate target

// Safety caps
const uint32_t MAX_BURST_SAMPLES = 20000UL;
const uint32_t PRINT_INTERVAL_MS = 1000UL;

// LFSR state and polynomial (Galois)
uint16_t lfsr_state = 0xACE1u; // seed
const uint16_t LFSR_FEEDBACK = 0xB400u;

// Measurement accumulators
volatile uint64_t sample_count = 0;
volatile uint64_t high_count = 0;

// Timing
uint32_t last_wake_us = 0;
uint32_t last_print_ms = 0;

// Track last physical output state left during idle (0 or 1)
uint8_t last_out = 0;

// Option: when idle, also advance LFSR state privately (true) or not (false).
// Advancing during idle reduces correlation between bursts but does NOT change
// the fact that the pin stayed constant during idle.
#define SIMULATE_LFSR_ADVANCE 0

// LFSR step (Galois)
static inline uint16_t lfsr_step(uint16_t s) {
  if (s & 1u) {
    s = (s >> 1) ^ LFSR_FEEDBACK;
  } else {
    s = (s >> 1);
  }
  return s ? s : 1;
}

static inline uint8_t lfsr_out_bit(uint16_t s, uint8_t level) {
  uint8_t top8 = (uint8_t)(s >> 8);
  return (top8 < level) ? 1 : 0;
}

// produce N LFSR samples (tight loop), update last_out, counters, return produced
uint32_t produce_samples(uint32_t count) {
  uint32_t produced = 0;
  if (count > MAX_BURST_SAMPLES) count = MAX_BURST_SAMPLES;
  for (uint32_t i = 0; i < count; ++i) {
    lfsr_state = lfsr_step(lfsr_state);
    uint8_t out = lfsr_out_bit(lfsr_state, pwm_level);
    if (out) {
      OUT_PORT |= OUT_MASK;
      high_count++;
    } else {
      OUT_PORT &= ~OUT_MASK;
    }
    last_out = out;
    produced++;
  }
  sample_count += produced;
  return produced;
}

// Optionally advance LFSR state N times without toggling the pin or counting samples
uint32_t advance_lfsr_without_output(uint32_t n) {
  // Cap for safety
  if (n > MAX_BURST_SAMPLES) n = MAX_BURST_SAMPLES;
  for (uint32_t i = 0; i < n; ++i) {
    lfsr_state = lfsr_step(lfsr_state);
  }
  return n;
}

// Simulate random idle delay
void simulated_sleep_with_jitter() {
  uint16_t d = random(10, 201); // ms
  delay(d);
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  pinMode(OUT_PIN, OUTPUT);
  digitalWrite(OUT_PIN, LOW);
  last_out = 0;
  OUT_PORT &= ~OUT_MASK;
  randomSeed(analogRead(A0));

  last_wake_us = micros();
  last_print_ms = millis();

  Serial.println("Opportunistic LFSR (accounting for held output)");
  Serial.print("SAMPLE_HZ="); Serial.println(SAMPLE_HZ);
  Serial.println("Commands: '+' increase duty, '-' decrease duty");
}

void loop() {
  // Idle with jitter
  simulated_sleep_with_jitter();

  // Time since last wake
  uint32_t now_us = micros();
  uint32_t elapsed_us = now_us - last_wake_us;
  last_wake_us = now_us;

  // Compute how many logical samples correspond to the idle time
  // samples = round(elapsed_us * SAMPLE_HZ / 1e6)
  uint64_t numerator = (uint64_t)elapsed_us * (uint64_t)SAMPLE_HZ;
  uint32_t idle_samples = (uint32_t)(numerator / 1000000ULL);

  if (idle_samples > 0) {
    // 1) Credit the idle interval as having produced 'idle_samples' outputs equal to last_out
    sample_count += idle_samples;
    if (last_out) high_count += idle_samples;

    // 2) Optionally advance LFSR internally to decorrelate (no physical output)
#if SIMULATE_LFSR_ADVANCE
    // Be conservative to avoid huge advancing cost if idle_samples large
    // Cap the internal advance to MAX_BURST_SAMPLES to avoid blocking too long
    uint32_t to_advance = idle_samples;
    if (to_advance > MAX_BURST_SAMPLES) to_advance = MAX_BURST_SAMPLES;
    advance_lfsr_without_output(to_advance);
#endif

    // 3) Compute how many '1's should have occurred during these idle_samples
    // desired_ones = round(idle_samples * pwm_level / 256)
    uint32_t desired_ones = (uint32_t)(((uint64_t)idle_samples * (uint64_t)pwm_level + 128ULL) / 256ULL);

    // 4) Count how many ones we've already credited = last_out ? idle_samples : 0
    uint32_t credited_ones = last_out ? idle_samples : 0;

    // 5) Deficit (extra '1's to produce now to reach desired fraction)
    int32_t deficit = (int32_t)desired_ones - (int32_t)credited_ones;
    if (deficit <= 0) {
      // nothing to produce to satisfy desired duty for the idle interval.
      // However, you may still want to advance LFSR state a bit in background,
      // or produce a small number of samples to keep randomness fresh. For now: do nothing.
    } else {
      // produce 'deficit' number of 1s using LFSR runs (we can't force LFSR to output 1 on demand,
      // so we iterate until we've observed deficit many 1s, capping iterations by MAX_BURST_SAMPLES)
      uint32_t produced = 0;
      uint32_t found_ones = 0;
      uint32_t iter = 0;
      const uint32_t ITER_CAP = MAX_BURST_SAMPLES; // safety
      while ((found_ones < (uint32_t)deficit) && (iter < ITER_CAP)) {
        // step one LFSR and test output
        lfsr_state = lfsr_step(lfsr_state);
        uint8_t out = lfsr_out_bit(lfsr_state, pwm_level);
        // write pin and count
        if (out) {
          OUT_PORT |= OUT_MASK;
          high_count++;
          found_ones++;
        } else {
          OUT_PORT &= ~OUT_MASK;
        }
        last_out = out;
        produced++;
        iter++;
      }
      sample_count += produced;
      // If we hit ITER_CAP before satisfying deficit, we leave remaining deficit unserved;
      // choose to log or lower expectations (not implemented here).
    }
  } // end idle_samples>0

  // Periodically report measured duty
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
    Serial.print("% last_out="); Serial.println(last_out);
    last_print_ms = now_ms;
  }

  // Serial control
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
