// LUT entry structure (4 bytes per entry)
struct PWMLUTEntry {
  uint16_t mask1;
  uint16_t seed1;
  // Second LFSR could use different length for coprime periods
  // Or store both in same structure:
};

// Extended structure (8 bytes per entry) - still reasonable
struct DualPWMLUTEntry {
  uint16_t mask1;
  uint16_t seed1;
  uint16_t mask2;
  uint16_t seed2;
};

// PWM update with dual LFSR XOR
uint16_t lfsr_state1 = 1;
uint16_t lfsr_state2 = 1;
uint16_t lfsr_mask1, lfsr_mask2;

void pwm_update() {
  // Advance LFSR 1
  uint16_t feedback1 = __builtin_parity(lfsr_state1 & lfsr_mask1) & 1;
  lfsr_state1 = (lfsr_state1 >> 1) | (feedback1 << 15);
  
  // Advance LFSR 2
  uint16_t feedback2 = __builtin_parity(lfsr_state2 & lfsr_mask2) & 1;
  lfsr_state2 = (lfsr_state2 >> 1) | (feedback2 << 15);
  
  // XOR combination - no threshold needed!
  digitalWrite(PWM_PIN, (lfsr_state1 ^ lfsr_state2) & 1 ? HIGH : LOW);
}
