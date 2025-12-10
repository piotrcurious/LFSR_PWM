/*
 * arduino_lfsr_pwm_examples.ino
 *
 * Demonstrates several PWM implementation strategies and compares their
 * behavior by measuring achieved duty ratio. Integrates LFSR-based
 * pseudo-random PWM using a precomputed Look-Up Table (LUT) of 16-bit masks
 * and initial seeds.
 *
 * NOTE: This version correctly reads the mask and seed from the
 * 'lfsr_pwm_lut[]' structure defined in 'arduino_lut.h' and assumes a full
 * 256-entry LUT size.
 *
 * Implementations included:
 * 1) CLASSIC_ANALOGWRITE: Use Arduino's built-in analogWrite (hardware PWM)
 * 2) TIMER_SOFTWARE: Timer interrupt driven software PWM (fixed-frequency)
 * 3) LFSR_PWM: Timer interrupt driven pseudo-random PWM using an LFSR bitstream
 * 4) DDA_PWM: Accumulator-style (sigma-delta-like) software PWM as baseline
 *
 * Author: Generated for user (reference example)
 */

#include <Arduino.h>
#include <avr/pgmspace.h>

// Assuming arduino_lut.h is included and contains the structure definition and data.
#include "arduino_lut.h"

// Define the size of the LUT based on the user's feedback (256 entries)
#define LFSR_LUT_SIZE 256

// Select which implementations to compile (you can enable multiple and switch at runtime)
#define MODE_CLASSIC_ANALOGWRITE 0
#define MODE_TIMER_SOFTWARE    1
#define MODE_LFSR_PWM        2
#define MODE_DDA_PWM       3

// Change this to pick the active mode at runtime
volatile uint8_t active_mode = MODE_LFSR_PWM;

const uint8_t OUT_PIN = 9; // OC1A on UNO (works with analogWrite on pin 9)

// Timer tick frequency for software modes (Hz)
const uint32_t TICK_HZ = 31250UL;

// Measurement window: count ticks and compute duty ratio over this many ticks
const uint32_t MEAS_TICKS = TICK_HZ / 10; // measure over 0.1s window


// ----- LFSR routines (Galois is usually faster) -----
static inline uint16_t lfsr_fibonacci_step(uint16_t state, uint16_t mask) {
  // Fibonacci-style LFSR step
  uint16_t feedback = (__builtin_parity(state & mask) & 1);
  return (state >> 1) | (feedback << 15);
}

static inline uint16_t lfsr_galois_step(uint16_t state, uint16_t mask) {
  // Galois LFSR step: if LSB==1 then state = (state >> 1) ^ mask; else state >>= 1;
  if (state & 1) {
    state = (state >> 1) ^ mask;
  } else {
    state = (state >> 1);
  }
  return state;
}

// Flag to switch between Galois and Fibonacci (Fibonacci is now the default)
volatile bool use_galois = false;

// ----- DDA (Accumulator) PWM -----
uint32_t dda_acc = 0;

// Desired PWM level (0..255 duty cycle) OR LUT Index (0..LFSR_LUT_SIZE-1)
volatile uint8_t pwm_level = 128;

// Measurement counters (updated in ISR)
volatile uint32_t tick_count = 0;
volatile uint32_t high_count = 0;
volatile bool meas_ready = false;

// LFSR state and mask (used in MODE_LFSR_PWM)
volatile uint16_t lfsr_state = 1; // must be non-zero
volatile uint16_t lfsr_mask_current = 0xB400u; // Default mask (L0)

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
      // Note: pwm_level here is 0-255 duty cycle, not the LUT index
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
      dda_acc += pwm_level; // Note: pwm_level here is 0-255 duty cycle
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

// ----- Utility: load mask and seed for a desired level from PROGMEM -----
// Returns the mask and updates the seed_out pointer with the initial seed.
uint16_t load_lfsr_params(uint8_t level_index, uint16_t *seed_out) {

  uint8_t idx = level_index;
  // Since the LUT size is 256 (LFSR_LUT_SIZE), and idx is uint8_t, it's always in range.

  // Read the mask and seed from PROGMEM
  // lfsr_pwm_lut is an array of structs stored in flash (PROGMEM).
  uint16_t mask = pgm_read_word_near(&lfsr_pwm_lut[idx].mask);
  uint16_t seed = pgm_read_word_near(&lfsr_pwm_lut[idx].seed);

  *seed_out = seed; // Pass seed back via pointer
  return mask;   // Return mask
}

void apply_pwm_level(uint8_t level) {
  pwm_level = level;

  // Update mask and get seed for LFSR mode ONLY
  if (active_mode == MODE_LFSR_PWM) {
    uint16_t new_seed;

    // Note: 'level' is used as the LUT index (0-255) here
    lfsr_mask_current = load_lfsr_params(level, &new_seed);

    // Initialize the LFSR state with the seed from the LUT
    lfsr_state = new_seed;
    // Ensure it is not zero (LFSRs stall at state 0). If seed is 0, use 1.
    if (lfsr_state == 0) lfsr_state = 1;
  }
  // For other modes, pwm_level is used as the 0-255 duty cycle target.
}

void setup() {
  Serial.begin(115200);
  while (!Serial) ;
  pinMode(OUT_PIN, OUTPUT);
  digitalWrite(OUT_PIN, LOW);

  // Default mode and level (using LUT index 128, approx 50% duty)
  active_mode = MODE_LFSR_PWM;
  apply_pwm_level(128); // Set to LUT Index 128
  use_galois = false; // <-- Default set to Fibonacci LFSR

  // Setup timer
  setup_timer1_ctc(TICK_HZ);

  Serial.println("LFSR PWM examples - ready");
  Serial.println("Modes: 0=analogWrite,1=timer_sw,2=lfsr,3=dda");
}

uint32_t last_print = 0;

void loop() {
  // Example control: rotate through all 256 duty levels (LUT indices 0 to 255)
  static uint32_t next_change_ms = 0;
  static uint8_t level_index = 0; // Use index 0 to LFSR_LUT_SIZE - 1
  uint32_t now = millis();

  // Step the index every 200 milliseconds (to cycle through 256 levels in ~51 seconds)
  if (now >= next_change_ms) {
    // Cycle index 0, 1, ..., 255, 0, ...
    // Since level_index is uint8_t, it naturally wraps from 255 to 0 on increment.
    level_index++;
    
    // Apply the settings associated with this LUT index
    apply_pwm_level(level_index);
    
    next_change_ms = now + 200; // Change every 200 ms
    Serial.print("Switched to LUT Index = "); Serial.println(level_index);
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
    Serial.print(" LUT_Idx="); Serial.print(pwm_level);
    Serial.print(" Achieved="); Serial.print(achieved * 100.0f, 2);
    Serial.print("% ticks="); Serial.print(t);
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
      Serial.print("use_galois = "); Serial.println(use_galois ? "true (Galois)" : "false (Fibonacci)");
      // Re-apply current level to reset LFSR with the LUT seed
      apply_pwm_level(pwm_level);
    } else if (c == '+') {
      // Move to the next index in the LUT, wrapping around 0-255
      uint8_t new_idx = (uint8_t)(pwm_level + 1);
      apply_pwm_level(new_idx);
      Serial.print("LUT Index -> "); Serial.println(new_idx);
    } else if (c == '-') {
      // Move to the previous index, wrapping around 0-255
      uint8_t new_idx = (uint8_t)(pwm_level - 1);
      apply_pwm_level(new_idx);
      Serial.print("LUT Index -> "); Serial.println(new_idx);
    }
  }

  // light background task
  delay(1);
}

/*
 * Explanation of benefits demonstrated by this sketch:
 * (Unchanged from original)
 */
