import os
import struct
import numpy as np

# --- Configuration ---
# Directory structure assumed to be relative to this script
BIN_DIR = "8bit_set"
CACHE_DIR = "spectral_cache"
OUTPUT_FILE = "lut.h"

# --- Filtering Constraints ---
# These values determine which candidates are considered "stable" enough for selection.

# Constraint 1: Absolute Spectral Purity (Error/Noise)
# Max allowable spectral noise (e.g., 5% of total spectral energy).
# Candidates with higher error are rejected outright.
MAX_SPECTRAL_ERROR = 0.05  

# Constraint 2: Frequency Stability/Range
# Normalized frequency (0.0 to 1.0, where 1.0 is Nyquist, typically Fs/2)
# Candidates with peak frequencies outside this range are rejected.
MIN_PEAK_FREQ = 0.20       
MAX_PEAK_FREQ = 0.70       
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
            print(f"Warning: Spectral cache size mismatch for {pwm_level:02X}. Skipping spectral analysis.")
            return mask_seed_pairs, []
            
        for i in range(num_spec_entries):
            start = i * SPECTRAL_SIZE
            freq, error = struct.unpack(SPECTRAL_FORMAT, data[start:start + SPECTRAL_SIZE])
            spectral_data.append({'freq': freq, 'error': error})
    else:
        print(f"Warning: Spectral cache not found for {pwm_level:02X}. Cannot apply spectral filter.")
        
    return mask_seed_pairs, spectral_data

def apply_filters_and_select(pwm_level, mask_seed_pairs, spectral_data):
    """Applies constraints and selects the candidate with the LEAST spectral error."""
    
    candidates_with_error = []
    has_spectral_data = len(spectral_data) == len(mask_seed_pairs)
    
    if not has_spectral_data:
        # If no spectral data, we cannot sort based on error, so we skip selection.
        print("Cannot sort by error. No spectral data.")
        return None

    # 1. Filter candidates based on absolute constraints
    for i, (entry, spec) in enumerate(zip(mask_seed_pairs, spectral_data)):
        
        # Absolute Error Constraint
        if spec['error'] > MAX_SPECTRAL_ERROR:
            continue
        
        # Frequency Range Constraint
        if spec['freq'] < MIN_PEAK_FREQ or spec['freq'] > MAX_PEAK_FREQ:
            continue
        
        # Candidate passed all filters
        candidates_with_error.append((entry, spec['error']))

    # 2. Select the final entry (Lowest Quantization Error)
    
    if not candidates_with_error:
        print(f"No stable candidates ({len(mask_seed_pairs)} total points) met all spectral criteria.")
        return None

    # Sort the filtered list by spectral error (index 1), ascending (lowest error is best)
    candidates_with_error.sort(key=lambda x: x[1])
    
    # The best candidate is the one at index 0 after sorting
    best_candidate = candidates_with_error[0][0]
    best_error = candidates_with_error[0][1]
    
    print(f"Selected best candidate (Error: {best_error:.4f}). {len(candidates_with_error)} passed filters.")

    return best_candidate


def generate_lut_h():
    """Compiles the final lut.h file with 256 selected LFSR pairs."""
    
    final_lut = []
    
    print(f"\n--- Compiling {OUTPUT_FILE} ---\n")
    
    for i in range(256):
        print(f"Processing PWM Level {i} (0x{i:02X})... ", end="")
        
        # Load the data for this bin
        mask_seed_pairs, spectral_data = load_data(i)
        
        if not mask_seed_pairs:
            final_lut.append(None)
            print("No data in bin.")
            continue
            
        # Apply filters and select the best candidate
        selected_entry = apply_filters_and_select(i, mask_seed_pairs, spectral_data)
        
        if selected_entry:
            final_lut.append(selected_entry)
        else:
            # Failed to find a stable candidate
            final_lut.append(None)
            
    # --- Write Header File ---
    with open(OUTPUT_FILE, 'w') as f:
        # Preamble
        f.write("// Arduino LFSR PWM LUT: mask, seed (16-bit)\n")
        f.write("// Generated by lfsr_lut_compiler.py\n")
        f.write("// Selection Criteria: Lowest Spectral Error (Quantization Noise).\n")
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
                # Fallback to a safe, non-zero pair for stability in case of error
                # Using {0x8003u, 0x8000u} (a stable primitive polynomial mask)
                line = f"  {{ 0x8003u, 0x8000u }}, // PWM {i} (0x{i:02X}) - NO STABLE CANDIDATE FOUND! (Using default fallback)"

            f.write(line + "\n")
            
        f.write("};\n")
        
    print(f"\nSuccessfully generated {OUTPUT_FILE} with {256} entries.")

if __name__ == "__main__":
    # Ensure the directory structure exists before running
    if not os.path.isdir(BIN_DIR) or not os.path.isdir(CACHE_DIR):
        print(f"Error: Required directories '{BIN_DIR}' and '{CACHE_DIR}' not found.")
        print("Please run the LFSR analyzer and spectral analyzer first.")
    else:
        generate_lut_h()

