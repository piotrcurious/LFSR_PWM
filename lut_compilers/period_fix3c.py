import os
import struct
import numpy as np

# --- Configuration ---
BIN_DIR = "8bit_set"
OUTPUT_FILE = "lut.h"

# --- Selection Criteria ---
# Priority 1: Minimize duty cycle error (absolute difference from target)
# Priority 2: Minimize LFSR period (shorter periods = faster cycling)

# Maximum acceptable duty cycle error (as a fraction, e.g., 0.02 = 2%)
MAX_DUTY_CYCLE_ERROR = 0.02

# Maximum acceptable LFSR period (in cycles)
# Set to None to allow any period
MAX_PERIOD = None  # e.g., 65535 for 16-bit max

# --- Data Structures ---
ENTRY_SIZE = 4
ENTRY_FORMAT = '<HH'  # Little-endian, two unsigned shorts (Mask, Seed)

# --- Fibonacci LFSR & Parity Helpers ---

def parity16(n):
    """
    Calculates the 16-bit parity (odd/even number of set bits).
    Returns 1 for odd parity (odd number of set bits), 0 for even parity.
    This mimics the logic: __builtin_parity(n) & 1.
    """
    # XOR all 16 bits
    n ^= n >> 8
    n ^= n >> 4
    n ^= n >> 2
    n ^= n >> 1
    return n & 1

def fibonacci_lfsr_threshold_step(state, mask, threshold):
    """
    Performs one step of the 16-bit Fibonacci LFSR with Threshold comparison output.

    The C logic was:
      out = (lfsr_state > lfsr_threshold) ? 1 : 0;
      feedback = parity16(lfsr_state & lfsr_mask_current);
      lfsr_state = (lfsr_state >> 1) | (feedback << 15);
      
    Note: The 'mask' argument here is the 'lfsr_mask_current' (taps).
    """
    # 1. Output based on comparison (lfsr_state > lfsr_threshold)
    output_bit = 1 if state > threshold else 0

    # 2. Advance LFSR state (Fibonacci implementation)
    # The mask defines the tap positions.
    feedback = parity16(state & mask)
    
    # Right shift by 1, and insert feedback bit into the MSB (bit 15)
    next_state = (state >> 1) | (feedback << 15)
    
    return next_state, output_bit

# --- LFSR Period & Duty Cycle Calculator (MODIFIED) ---

def calculate_lfsr_properties(mask, seed, target_duty_cycle, pwm_period=65536):
    """
    Calculates the LFSR period and effective duty cycle over the PWM period.
    
    The 'seed' value from the bin file is now interpreted as the 'lfsr_threshold'
    since we are optimizing for both the mask (taps) and the threshold.
    """
    # For a target PWM level P (0-255), the duty cycle is P/255.
    # The 'seed' in the bin file is now used as the 'lfsr_threshold'.
    lfsr_threshold = seed 
    
    # Edge case: If the threshold is 65535, the output is always 0 (unless state is 65536, which is impossible for 16-bit).
    # If the threshold is 0, the output is always 1 (unless state is 0).
    if lfsr_threshold == 65535 and seed != 0: # Note: state cannot be 65536, so only check for > threshold
        return pwm_period, 0.0, target_duty_cycle 
    if lfsr_threshold == 0 and seed != 0:
         return pwm_period, 1.0, abs(1.0 - target_duty_cycle)

    # seed the seed 
    initial_lfsr_state = seed
    
    # Step 1: Find the LFSR period
    state = initial_lfsr_state
    
    # We must track both the state and the *output* sequence to find the true duty cycle period.
    # However, the period is solely determined by the LFSR state sequence.
    period = 0
    
    # We will simulate for up to pwm_period * 2 just in case, but keep the check simple.
    for step in range(1, pwm_period + 1):
        state, output_bit = fibonacci_lfsr_threshold_step(state, mask, lfsr_threshold)
        
        if state == initial_lfsr_state:
            # Found the period (state sequence repeats)
            period = step
            break
    
    if period == 0:
        # Did not find period within pwm_period, use max
        period = pwm_period 
    
    # Step 2: Calculate duty cycle incrementally over the full PWM period
    state = initial_lfsr_state
    
    # Use Kahan summation for the duty cycle to minimize floating-point errors
    duty_sum = 0.0
    compensation = 0.0  # Compensation for lost low-order bits
    
    for step in range(pwm_period):
        state, output_bit = fibonacci_lfsr_threshold_step(state, mask, lfsr_threshold)
        
        # Incremental duty cycle contribution: output_bit / pwm_period
        y = (output_bit / pwm_period) - compensation
        t = duty_sum + y
        compensation = (t - duty_sum) - y
        duty_sum = t
    
    duty_cycle = duty_sum
    duty_error = abs(duty_cycle - target_duty_cycle)
    
    # Returning the threshold instead of the seed (as the seed slot is now the threshold)
    return period, duty_cycle, duty_error

# --- Main Logic (NOT MODIFIED, uses the new calculate_lfsr_properties) ---

def load_mask_seed_pairs(pwm_level):
    """Loads the (Mask, Threshold) pairs for a given PWM level."""
    # Renamed: 'seed' is now 'threshold'
    bin_file = os.path.join(BIN_DIR, f"0x{pwm_level:02X}.bin")

    if not os.path.exists(bin_file):
        print(f"Warning: Bin file not found for PWM {pwm_level:02X}. Skipping.")
        return []

    with open(bin_file, 'rb') as f:
        data = f.read()
    
    num_entries = len(data) // ENTRY_SIZE
    mask_threshold_pairs = []
    
    for i in range(num_entries):
        start = i * ENTRY_SIZE
        # We now interpret 'seed' as 'threshold' from the bin file
        mask, threshold = struct.unpack(ENTRY_FORMAT, data[start:start + ENTRY_SIZE])
        mask_threshold_pairs.append({'mask': mask, 'threshold': threshold})
    
    return mask_threshold_pairs

def evaluate_and_select(pwm_level, mask_threshold_pairs):
    """
    Evaluates all candidates based on duty cycle error and period.
    Selects the best candidate (lowest duty error, then shortest period).
    """
    target_duty_cycle = pwm_level / 255.0
    
    candidates = []
    
    print(f"Evaluating {len(mask_threshold_pairs)} candidates... ", end="", flush=True)
    
    for entry in mask_threshold_pairs:
        mask = entry['mask']
        threshold = entry['threshold']
        
        # Calculate LFSR properties
        # The 'threshold' is passed in the 'seed' argument slot
        period, duty_cycle, duty_error = calculate_lfsr_properties(
            mask, threshold, target_duty_cycle # 'threshold' is now in the 'seed' slot
        )
        
        # Apply hard constraints
        if duty_error > MAX_DUTY_CYCLE_ERROR:
            continue
        
        if MAX_PERIOD is not None and period > MAX_PERIOD:
            continue
        
        candidates.append({
            'mask': mask,
            'threshold': threshold,
            'period': period,
            'duty_cycle': duty_cycle,
            'duty_error': duty_error
        })
    
    if not candidates:
        print(f"No candidates met criteria (checked {len(mask_threshold_pairs)} pairs).")
        return None
    
    # Sort by: 1) Lowest duty error, 2) Shortest period
    candidates.sort(key=lambda x: (x['duty_error'], x['period']))
    
    best = candidates[0]
    print(f"Selected: Period={best['period']}, Duty={best['duty_cycle']:.4f} (target={target_duty_cycle:.4f}), Error={best['duty_error']:.6f}")
    
    return best

def generate_lut_h():
    """Generates the LUT header file with optimized LFSR pairs."""
    final_lut = []
    
    print(f"\n--- Compiling {OUTPUT_FILE} (Fibonacci/Threshold) ---\n")
    
    for i in range(256):
        print(f"PWM Level {i} (0x{i:02X}): ", end="")
        
        mask_threshold_pairs = load_mask_seed_pairs(i)
        
        if not mask_threshold_pairs:
            final_lut.append(None)
            print("No data.")
            continue
        
        selected = evaluate_and_select(i, mask_threshold_pairs)
        final_lut.append(selected)
    
    # --- Write Header File ---
    with open(OUTPUT_FILE, 'w') as f:
        f.write("// Arduino LFSR PWM LUT: mask, threshold (16-bit)\n") # Updated comment
        f.write("// Generated by lfsr_lut_compiler.py (Fibonacci LFSR with Threshold)\n") # Updated comment
        f.write("// Selection Criteria: Minimize duty cycle error, then minimize period.\n")
        f.write(f"// Constraints: Max Duty Error={MAX_DUTY_CYCLE_ERROR}, Max Period={MAX_PERIOD if MAX_PERIOD else 'None'}\n")
        f.write("#include <stdint.h>\n")
        f.write("#if defined(__AVR__)\n")
        f.write("#include <avr/pgmspace.h>\n")
        f.write("#else\n")
        f.write("#define PROGMEM\n")
        f.write("#define pgm_read_word(addr) (*(addr))\n")
        f.write("#define pgm_read_dword(addr) (*(addr))\n")
        f.write("#endif\n\n")
        
        f.write("// Format: { mask, threshold, period, duty_cycle_x1000 }\n") # Updated comment
        f.write("// duty_cycle_x1000 is the actual duty cycle * 1000 for reference\n")
        # Note: The struct still uses 'seed' for the second element, matching the C-code structure
        f.write("const struct { uint16_t mask; uint16_t seed; uint16_t period; uint16_t duty_x1000; } lfsr_pwm_lut[] PROGMEM = {\n")
        
        for i, entry in enumerate(final_lut):
            if entry:
                duty_x1000 = int(entry['duty_cycle'] * 1000)
                # Note: 'threshold' is written into the 'seed' field
                line = f"  {{ 0x{entry['mask']:04x}u, 0x{entry['threshold']:04x}u, {entry['period']:5d}u, {duty_x1000:4d}u }}, // PWM {i}"
            else:
                # Fallback: simple maximal-length LFSR (mask, *threshold* is now the second value)
                # The fallback threshold here should be chosen to give ~50% duty cycle, e.g., 65536/2 = 32768 (0x8000)
                line = f"  {{ 0x8003u, 0x8000u, 65535u, 500u }}, // PWM {i} - FALLBACK"
            
            f.write(line + "\n")
        
        f.write("};\n")
    
    print(f"\n✓ Successfully generated {OUTPUT_FILE} with 256 entries.")
    
    # Statistics
    valid_entries = [e for e in final_lut if e is not None]
    if valid_entries:
        avg_period = sum(e['period'] for e in valid_entries) / len(valid_entries)
        avg_error = sum(e['duty_error'] for e in valid_entries) / len(valid_entries)
        print(f"\nStatistics ({len(valid_entries)} valid entries):")
        print(f"  Average Period: {avg_period:.1f}")
        print(f"  Average Duty Error: {avg_error:.6f}")
        print(f"  Fallback Entries: {256 - len(valid_entries)}")

if __name__ == "__main__":
    if not os.path.isdir(BIN_DIR):
        print(f"Error: Required directory '{BIN_DIR}' not found.")
        print("Please run the LFSR analyzer first to generate candidate pairs.")
    else:
        generate_lut_h()
