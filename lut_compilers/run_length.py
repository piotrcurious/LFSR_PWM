import os
import struct
import numpy as np

# --- Configuration ---
BIN_DIR = "8bit_set"
CACHE_DIR = "duty_cycle_cache"
OUTPUT_FILE = "lut.h"

# --- Selection Criteria ---
# Weight factors for scoring (higher weight = more important)
DUTY_CYCLE_WEIGHT = 0.7  # 70% importance on duty cycle accuracy
RUN_LENGTH_WEIGHT = 0.3  # 30% importance on run length minimization

# Maximum acceptable duty cycle error (absolute difference from target)
MAX_DUTY_CYCLE_ERROR = 0.02  # 2% maximum error

# --- Data Structures ---
ENTRY_SIZE = 4
ENTRY_FORMAT = '<HH'  # Little-endian, two unsigned shorts (mask, seed)

# Duty Cycle Cache: duty_cycle (float), max_run_length (uint16), avg_run_length (float)
DUTY_CACHE_SIZE = 10
DUTY_CACHE_FORMAT = '<fHf'

# --- LFSR Simulation for Duty Cycle Analysis ---

def simulate_lfsr(mask, seed, target_duty_cycle, period=65535):
    """
    Simulates an LFSR and computes:
    - Actual duty cycle (ratio of 1s)
    - Maximum run length (longest consecutive sequence of same bit)
    - Average run length
    """
    if seed == 0:
        seed = 1  # Avoid zero state
    
    state = seed
    ones_count = 0
    current_run = 1
    current_bit = state & 1
    max_run = 1
    run_lengths = []
    
    # Simulate for one full period or up to 65535 steps
    for _ in range(min(period, 65535)):
        bit = state & 1
        ones_count += bit
        
        # Track run lengths
        if bit == current_bit:
            current_run += 1
        else:
            run_lengths.append(current_run)
            max_run = max(max_run, current_run)
            current_run = 1
            current_bit = bit
        
        # LFSR step: shift right and XOR feedback
        feedback = 0
        temp = state & mask
        while temp:
            feedback ^= (temp & 1)
            temp >>= 1
        
        state = (state >> 1) | (feedback << 15)
        
        # Check if we've completed a cycle
        if state == seed:
            break
    
    # Final run
    if current_run > 0:
        run_lengths.append(current_run)
        max_run = max(max_run, current_run)
    
    total_steps = min(period, 65535)
    duty_cycle = ones_count / total_steps
    avg_run = np.mean(run_lengths) if run_lengths else 1.0
    
    return duty_cycle, max_run, avg_run


def calculate_duty_cycle_error(actual_duty, target_duty):
    """Calculate absolute error between actual and target duty cycle."""
    return abs(actual_duty - target_duty)


def load_data(pwm_level):
    """Loads (Mask, Seed) pairs and duty cycle data for a given PWM level."""
    
    bin_file = os.path.join(BIN_DIR, f"0x{pwm_level:02X}.bin")
    cache_file = os.path.join(CACHE_DIR, f"0x{pwm_level:02X}.duty")
    
    if not os.path.exists(bin_file):
        print(f"Warning: Bin file not found for PWM {pwm_level:02X}. Skipping.")
        return [], []
    
    # Load (Mask, Seed) Pairs
    with open(bin_file, 'rb') as f:
        data = f.read()
    
    num_entries = len(data) // ENTRY_SIZE
    mask_seed_pairs = []
    for i in range(num_entries):
        start = i * ENTRY_SIZE
        mask, seed = struct.unpack(ENTRY_FORMAT, data[start:start + ENTRY_SIZE])
        mask_seed_pairs.append({'mask': mask, 'seed': seed})
    
    # Load Duty Cycle Data if available
    duty_data = []
    if os.path.exists(cache_file):
        with open(cache_file, 'rb') as f:
            data = f.read()
        
        num_duty_entries = len(data) // DUTY_CACHE_SIZE
        if num_duty_entries != num_entries:
            print(f"Warning: Duty cycle cache size mismatch for {pwm_level:02X}.")
            return mask_seed_pairs, []
        
        for i in range(num_duty_entries):
            start = i * DUTY_CACHE_SIZE
            duty, max_run, avg_run = struct.unpack(DUTY_CACHE_FORMAT, data[start:start + DUTY_CACHE_SIZE])
            duty_data.append({'duty_cycle': duty, 'max_run': max_run, 'avg_run': avg_run})
    
    return mask_seed_pairs, duty_data


def analyze_and_select(pwm_level, mask_seed_pairs, duty_data):
    """
    Analyzes candidates and selects the best one based on:
    1. Minimum duty cycle error
    2. Minimum run length
    """
    
    target_duty_cycle = pwm_level / 255.0
    candidates = []
    
    has_duty_data = len(duty_data) == len(mask_seed_pairs)
    
    print(f"Target duty cycle: {target_duty_cycle:.4f}")
    
    # Analyze each candidate
    for i, entry in enumerate(mask_seed_pairs):
        if has_duty_data:
            # Use cached data
            duty = duty_data[i]['duty_cycle']
            max_run = duty_data[i]['max_run']
            avg_run = duty_data[i]['avg_run']
        else:
            # Simulate LFSR to compute metrics
            duty, max_run, avg_run = simulate_lfsr(
                entry['mask'], 
                entry['seed'], 
                target_duty_cycle
            )
        
        # Calculate duty cycle error
        duty_error = calculate_duty_cycle_error(duty, target_duty_cycle)
        
        # Filter: reject if duty cycle error exceeds maximum
        if duty_error > MAX_DUTY_CYCLE_ERROR:
            continue
        
        # Normalize metrics for scoring (0-1 range)
        # Lower is better for both metrics
        norm_duty_error = duty_error / MAX_DUTY_CYCLE_ERROR  # 0 = perfect, 1 = max acceptable
        norm_run_length = min(max_run / 100.0, 1.0)  # Normalize, cap at 100
        
        # Calculate weighted score (lower is better)
        score = (DUTY_CYCLE_WEIGHT * norm_duty_error + 
                 RUN_LENGTH_WEIGHT * norm_run_length)
        
        candidates.append({
            'entry': entry,
            'duty_error': duty_error,
            'max_run': max_run,
            'avg_run': avg_run,
            'score': score
        })
    
    if not candidates:
        print(f"No candidates met criteria (total: {len(mask_seed_pairs)})")
        return None
    
    # Sort by score (lowest/best first)
    candidates.sort(key=lambda x: x['score'])
    
    best = candidates[0]
    print(f"Selected: duty_error={best['duty_error']:.5f}, max_run={best['max_run']}, "
          f"avg_run={best['avg_run']:.2f}, score={best['score']:.4f} "
          f"({len(candidates)}/{len(mask_seed_pairs)} passed)")
    
    return best['entry']


def generate_lut_h():
    """Generates the lut.h header file with 256 optimized LFSR pairs."""
    
    final_lut = []
    
    print(f"\n--- Compiling {OUTPUT_FILE} ---\n")
    print(f"Selection weights: Duty Cycle={DUTY_CYCLE_WEIGHT}, Run Length={RUN_LENGTH_WEIGHT}\n")
    
    for i in range(256):
        print(f"PWM {i:3d} (0x{i:02X}): ", end="")
        
        mask_seed_pairs, duty_data = load_data(i)
        
        if not mask_seed_pairs:
            final_lut.append(None)
            print("No data")
            continue
        
        selected = analyze_and_select(i, mask_seed_pairs, duty_data)
        final_lut.append(selected)
    
    # Write header file
    with open(OUTPUT_FILE, 'w') as f:
        f.write("// Arduino LFSR PWM LUT: mask, seed (16-bit)\n")
        f.write("// Generated by lfsr_lut_compiler.py\n")
        f.write("// Selection Criteria: Minimize Duty Cycle Error and Run Length\n")
        f.write(f"// Weights: Duty Cycle={DUTY_CYCLE_WEIGHT}, Run Length={RUN_LENGTH_WEIGHT}\n")
        f.write(f"// Max Duty Cycle Error: {MAX_DUTY_CYCLE_ERROR}\n")
        f.write("#include <stdint.h>\n")
        f.write("#if defined(__AVR__)\n")
        f.write("#include <avr/pgmspace.h>\n")
        f.write("#else\n")
        f.write("#define PROGMEM\n")
        f.write("#define pgm_read_word(addr) (*(addr))\n")
        f.write("#define pgm_read_dword(addr) (*(addr))\n")
        f.write("#endif\n\n")
        
        f.write("const struct { uint16_t mask; uint16_t seed; } lfsr_pwm_lut[] PROGMEM = {\n")
        
        for i, entry in enumerate(final_lut):
            if entry:
                line = f"  {{ 0x{entry['mask']:04x}u, 0x{entry['seed']:04x}u }}"
            else:
                # Fallback: simple primitive polynomial
                line = f"  {{ 0x8003u, 0x8000u }}"
            
            comment = f"  // PWM {i} (0x{i:02X})"
            if not entry:
                comment += " - NO CANDIDATE (fallback)"
            
            f.write(line + "," + comment + "\n")
        
        f.write("};\n")
    
    print(f"\n✓ Successfully generated {OUTPUT_FILE}")


if __name__ == "__main__":
    if not os.path.isdir(BIN_DIR):
        print(f"Error: Directory '{BIN_DIR}' not found.")
        print("Please ensure LFSR data is available.")
    else:
        # Create cache directory if it doesn't exist
        os.makedirs(CACHE_DIR, exist_ok=True)
        generate_lut_h()
