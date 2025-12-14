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

# --- LFSR Period & Duty Cycle Calculator ---

def lfsr_step(state, mask):
    """Performs one step of the LFSR: returns next state and output bit."""
    lsb = state & 1
    state >>= 1
    if lsb:
        state ^= mask
    return state, lsb

def calculate_lfsr_properties(mask, seed, target_duty_cycle, pwm_period=65536):
    """
    Calculates the LFSR period and effective duty cycle over the PWM period.
    
    Uses incremental calculation to avoid numerical errors from large intermediate values.
    The effective duty cycle is computed incrementally over the full PWM period,
    accounting for the LFSR repeating continuously.
    
    Returns:
        period: Number of steps before LFSR repeats
        duty_cycle: Effective duty cycle ratio over the full PWM period
        duty_error: Absolute difference from target duty cycle
    """
    if seed == 0:
        # Degenerate case: all zeros
        return pwm_period, 0.0, target_duty_cycle
    
    # Step 1: Find the LFSR period
    state = seed
    initial_state = seed
    period = 0
    
    for step in range(1, pwm_period + 1):
        state, bit = lfsr_step(state, mask)
        
        if state == initial_state:
            # Found the period
            period = step
            break
    
    if period == 0:
        # Did not find period within pwm_period
        period = pwm_period
    
    # Step 2: Calculate duty cycle incrementally over the full PWM period
    state = initial_state
    ones_count = 0
    
    # Use Kahan summation for the duty cycle to minimize floating-point errors
    duty_sum = 0.0
    compensation = 0.0  # Compensation for lost low-order bits
    
    for step in range(pwm_period):
        state, bit = lfsr_step(state, mask)
        
        # Incremental duty cycle contribution: bit / pwm_period
        # Using Kahan summation to maintain precision
        y = (bit / pwm_period) - compensation
        t = duty_sum + y
        compensation = (t - duty_sum) - y
        duty_sum = t
    
    duty_cycle = duty_sum
    duty_error = abs(duty_cycle - target_duty_cycle)
    
    return period, duty_cycle, duty_error

# --- Main Logic ---

def load_mask_seed_pairs(pwm_level):
    """Loads the (Mask, Seed) pairs for a given PWM level."""
    bin_file = os.path.join(BIN_DIR, f"0x{pwm_level:02X}.bin")

    if not os.path.exists(bin_file):
        print(f"Warning: Bin file not found for PWM {pwm_level:02X}. Skipping.")
        return []

    with open(bin_file, 'rb') as f:
        data = f.read()
    
    num_entries = len(data) // ENTRY_SIZE
    mask_seed_pairs = []
    
    for i in range(num_entries):
        start = i * ENTRY_SIZE
        mask, seed = struct.unpack(ENTRY_FORMAT, data[start:start + ENTRY_SIZE])
        mask_seed_pairs.append({'mask': mask, 'seed': seed})
    
    return mask_seed_pairs

def evaluate_and_select(pwm_level, mask_seed_pairs):
    """
    Evaluates all candidates based on duty cycle error and period.
    Selects the best candidate (lowest duty error, then shortest period).
    """
    target_duty_cycle = pwm_level / 255.0
    
    candidates = []
    
    print(f"Evaluating {len(mask_seed_pairs)} candidates... ", end="", flush=True)
    
    for entry in mask_seed_pairs:
        mask = entry['mask']
        seed = entry['seed']
        
        # Calculate LFSR properties
        period, duty_cycle, duty_error = calculate_lfsr_properties(
            mask, seed, target_duty_cycle
        )
        
        # Apply hard constraints
        if duty_error > MAX_DUTY_CYCLE_ERROR:
            continue
        
        if MAX_PERIOD is not None and period > MAX_PERIOD:
            continue
        
        candidates.append({
            'mask': mask,
            'seed': seed,
            'period': period,
            'duty_cycle': duty_cycle,
            'duty_error': duty_error
        })
    
    if not candidates:
        print(f"No candidates met criteria (checked {len(mask_seed_pairs)} pairs).")
        return None
    
    # Sort by: 1) Lowest duty error, 2) Shortest period
    candidates.sort(key=lambda x: (x['duty_error'], x['period']))
    
    best = candidates[0]
    print(f"Selected: Period={best['period']}, Duty={best['duty_cycle']:.4f} (target={target_duty_cycle:.4f}), Error={best['duty_error']:.6f}")
    
    return best

def generate_lut_h():
    """Generates the LUT header file with optimized LFSR pairs."""
    final_lut = []
    
    print(f"\n--- Compiling {OUTPUT_FILE} ---\n")
    
    for i in range(256):
        print(f"PWM Level {i} (0x{i:02X}): ", end="")
        
        mask_seed_pairs = load_mask_seed_pairs(i)
        
        if not mask_seed_pairs:
            final_lut.append(None)
            print("No data.")
            continue
        
        selected = evaluate_and_select(i, mask_seed_pairs)
        final_lut.append(selected)
    
    # --- Write Header File ---
    with open(OUTPUT_FILE, 'w') as f:
        f.write("// Arduino LFSR PWM LUT: mask, seed (16-bit)\n")
        f.write("// Generated by lfsr_lut_compiler.py\n")
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
        
        f.write("// Format: { mask, seed, period, duty_cycle_x1000 }\n")
        f.write("// duty_cycle_x1000 is the actual duty cycle * 1000 for reference\n")
        f.write("const struct { uint16_t mask; uint16_t seed; uint16_t period; uint16_t duty_x1000; } lfsr_pwm_lut[] PROGMEM = {\n")
        
        for i, entry in enumerate(final_lut):
            if entry:
                duty_x1000 = int(entry['duty_cycle'] * 1000)
                line = f"  {{ 0x{entry['mask']:04x}u, 0x{entry['seed']:04x}u, {entry['period']:5d}u, {duty_x1000:4d}u }}, // PWM {i}"
            else:
                # Fallback: simple maximal-length LFSR
                line = f"  {{ 0x8003u, 0x0001u, 65535u, 500u }}, // PWM {i} - FALLBACK"
            
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
