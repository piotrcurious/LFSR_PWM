import os
import struct
import numpy as np

# --- Configuration ---
# Directory structure assumed to be relative to this script
BIN_DIR = "8bit_set"
CACHE_DIR = "spectral_cache"
OUTPUT_FILE = "lut.h"

# --- Filtering Constraints (Example Values) ---
# These must match the constraints you set during your analysis to ensure stability/quality.
# We will generate a separate LUT for each of the 256 PWM levels (0-255).
# If the constraints are too strict, some PWM levels might be empty.

# Constraint 1: Spectral Purity (Error/Noise)
MAX_SPECTRAL_ERROR = 0.05  # Max allowable spectral noise (e.g., 5% of total spectral energy)

# Constraint 2: Frequency Stability/Range
# Normalized frequency (0.0 to 1.0, where 1.0 is Nyquist, typically Fs/2)
MIN_PEAK_FREQ = 0.20       # Minimum normalized frequency (prevents very slow cycles)
MAX_PEAK_FREQ = 0.70       # Maximum normalized frequency (prevents too fast/noisy cycles)

# Constraint 3: Minimum size for a PWM level
# If a PWM level (bin) has fewer stable candidates than this, we skip it.
MIN_CANDIDATES_PER_BIN = 100 
# -----------------------------------------------

# --- Data Structures ---
# 16-bit Mask, 16-bit Seed
ENTRY_SIZE = 4 
ENTRY_FORMAT = '<HH' # Little-endian, two unsigned shorts

# Spectral Data: Peak Frequency (float), Error (float)
SPECTRAL_SIZE = 8
SPECTRAL_FORMAT = '<ff' 

# --- Main Logic ---

def load_data(pwm_level):
    """Loads the (Mask, Seed) pairs and Spectral data for a given PWM level (bin)."""
    
    bin_file = os.path.join(BIN_DIR, f"0x{pwm_level:02X}.bin")
    cache_file = os.path.join(CACHE_DIR, f"0x{pwm_level:02X}.spec")

    if not os.path.exists(bin_file):
        print(f"Warning: Bin file not found for PWM {pwm_level:02X}. Skipping.")
        return [], []

    # 1. Load (Mask, Seed) Pairs
    with open(bin_file, 'rb') as f:
        data = f.read()
    
    num_entries = len(data) // ENTRY_SIZE
    mask_seed_pairs = []
    for i in range(num_entries):
        start = i * ENTRY_SIZE
        mask, seed = struct.unpack(ENTRY_FORMAT, data[start:start + ENTRY_SIZE])
        mask_seed_pairs.append({'mask': mask, 'seed': seed})
    
    # 2. Load Spectral Data
    spectral_data = []
    if os.path.exists(cache_file):
        with open(cache_file, 'rb') as f:
            data = f.read()
        
        num_spec_entries = len(data) // SPECTRAL_SIZE
        if num_spec_entries != num_entries:
            print(f"Warning: Spectral cache size mismatch for {pwm_level:02X}. Skipping spectral filter.")
            return mask_seed_pairs, []
            
        for i in range(num_spec_entries):
            start = i * SPECTRAL_SIZE
            freq, error = struct.unpack(SPECTRAL_FORMAT, data[start:start + SPECTRAL_SIZE])
            spectral_data.append({'freq': freq, 'error': error})
    else:
        print(f"Warning: Spectral cache not found for {pwm_level:02X}. Skipping spectral filter.")
        
    return mask_seed_pairs, spectral_data

def apply_filters_and_select(mask_seed_pairs, spectral_data):
    """Applies constraints and selects a suitable candidate from the filtered list."""
    
    # 1. Filter candidates
    filtered_candidates = []
    has_spectral_data = len(spectral_data) == len(mask_seed_pairs)
    
    for i, (entry, spec) in enumerate(zip(mask_seed_pairs, spectral_data)):
        
        if has_spectral_data:
            # Apply spectral filters
            if spec['error'] > MAX_SPECTRAL_ERROR:
                continue
            if spec['freq'] < MIN_PEAK_FREQ or spec['freq'] > MAX_PEAK_FREQ:
                continue
        
        # All checks passed, add to list
        filtered_candidates.append(entry)

    # 2. Select the final entry
    if len(filtered_candidates) < MIN_CANDIDATES_PER_BIN:
        # If filtering is too strict, and we have enough original candidates, 
        # we might want to relax the filter for this bin, or just skip.
        if len(mask_seed_pairs) >= MIN_CANDIDATES_PER_BIN:
            print(f"  --> Only {len(filtered_candidates)} candidates found. Skipping this PWM level.")
        else:
            print(f"  --> Only {len(mask_seed_pairs)} original points. Skipping this PWM level.")
        return None
    
    # For simplicity, we choose the candidate with the lowest spectral error 
    # among the filtered set (if spectral data was available and used).
    if has_spectral_data:
        # Sort the filtered candidates based on spectral error (lowest is best)
        # This requires recreating the filtered list with spectral data
        final_list = []
        for i, (entry, spec) in enumerate(zip(mask_seed_pairs, spectral_data)):
             if entry in filtered_candidates: # Check if it passed filters
                 final_list.append((entry, spec['error']))
        
        # Sort by error (index 1)
        final_list.sort(key=lambda x: x[1])
        
        # Return the best (lowest error) entry
        return final_list[0][0]
    else:
        # If no spectral data, pick the first stable candidate found.
        # In a real system, you might want to pick a random one to prevent bias.
        return filtered_candidates[0]


def generate_lut_h():
    """Compiles the final lut.h file with 256 selected LFSR pairs."""
    
    final_lut = []
    
    print(f"\n--- Compiling {OUTPUT_FILE} ---\n")
    
    for i in range(256):
        print(f"Processing PWM Level {i} (0x{i:02X})... ", end="")
        
        # Load the data for this bin
        mask_seed_pairs, spectral_data = load_data(i)
        
        if not mask_seed_pairs:
            # No data in the bin at all
            final_lut.append(None)
            print("No data.")
            continue
            
        # Apply filters and select the best candidate
        selected_entry = apply_filters_and_select(mask_seed_pairs, spectral_data)
        
        if selected_entry:
            final_lut.append(selected_entry)
            print(f"Selected: {{ 0x{selected_entry['mask']:04x}u, 0x{selected_entry['seed']:04x}u }}. Total candidates: {len(mask_seed_pairs)}")
        else:
            # Failed to find a stable candidate
            final_lut.append(None)
            print("Failed to find stable candidate based on constraints.")
            
    # --- Write Header File ---
    with open(OUTPUT_FILE, 'w') as f:
        # Preamble
        f.write("// Arduino LFSR PWM LUT: mask, seed (16-bit)\n")
        f.write("// Generated by lfsr_lut_compiler.py\n")
        f.write(f"// Filters: Max Error={MAX_SPECTRAL_ERROR}, Freq Range=[{MIN_PEAK_FREQ}, {MAX_PEAK_FREQ}]\n")
        f.write("#include <stdint.h>\n")
        f.write("#if defined(__AVR__)\n")
        f.write("#include <avr/pgmspace.h>\n")
        f.write("#else\n")
        f.write("#define PROGMEM\n")
        f.write("#define pgm_read_word(addr) (*(addr))\n")
        f.write("#define pgm_read_dword(addr) (*(addr))\n")
        f.write("#endif\n\n")

        # LUT Definition
        f.write("const struct { uint16_t mask; uint16_t seed; } lfsr_pwm_lut[] PROGMEM = {\n")
        
        for i, entry in enumerate(final_lut):
            if entry:
                # Format: { 0x03cfu, 0xffa7u } 
                line = f"  {{ 0x{entry['mask']:04x}u, 0x{entry['seed']:04x}u }}, // PWM {i} (0x{i:02X})"
            else:
                # Fallback to a safe, though likely unused/default pair for stability
                # Using {1, 1} which is a lock-up (0% or 100% duty cycle) for safety
                line = f"  {{ 0x0001u, 0x0001u }}, // PWM {i} (0x{i:02X}) - NO STABLE CANDIDATE FOUND!"

            f.write(line + "\n")
            
        f.write("};\n")
        
    print(f"\nSuccessfully generated {OUTPUT_FILE} with {256} entries.")

if __name__ == "__main__":
    generate_lut_h()

