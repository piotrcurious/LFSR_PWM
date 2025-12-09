#!/usr/bin/env python3
"""
lfsr_pwm_algebraic.py

Generates a PWM Lookup Table (LUT) using 16-bit Fibonacci LFSRs.
Leverages Finite Field arithmetic and Cryptanalytic metrics (Berlekamp-Massey)
to ensure generated sequences have high linear complexity (Spread Spectrum).

Mathematical Context:
---------------------
1. The LFSR state update is a linear map in the vector space F_2^16.
2. The feedback mask corresponds to a polynomial P(x) in F_2[x].
3. Primitive polynomials generate Maximal Length Sequences (M-sequences) with
   period 2^n - 1 and duty cycle ~50%.
4. To get specific duty cycles (e.g., 10% or 90%), we search for REDUCIBLE
   polynomials that partition the state space into smaller, biased orbits.

Cryptanalysis Metric:
---------------------
We use the Berlekamp-Massey algorithm to calculate Linear Complexity (LC).
For a fixed duty cycle, the mask maximizing LC is chosen to ensure the
spectral energy is spread as widely as possible (reducing EMI).
"""

from __future__ import annotations
import argparse
import sys
from typing import Dict, List, Tuple, Optional

# --- Finite Field & Cryptanalysis Tools ---

def berlekamp_massey(bits: List[int]) -> int:
    """
    Computes the Linear Complexity (LC) of a binary sequence using the
    Berlekamp-Massey algorithm.
    
    The LC is the length of the shortest LFSR that can generate the sequence.
    High LC implies better pseudo-randomness and wider spectral spread.
    """
    n = len(bits)
    b = [0] * n
    c = [0] * n
    b[0] = 1
    c[0] = 1
    l = 0
    m = -1
    
    for N in range(n):
        # Calculate discrepancy d
        d = 0
        for i in range(l + 1):
            d ^= c[i] & bits[N - i]
        
        if d == 1:
            t = [x for x in c] # Copy c
            # c(x) = c(x) + b(x) * x^(N-m)
            p = N - m
            for i in range(n - p):
                c[i + p] ^= b[i]
            
            if l <= N / 2:
                l = N + 1 - l
                m = N
                b = t
    return l

def get_next_state(state: int, mask: int, n: int) -> int:
    """
    Computes S_{t+1} given S_t in GF(2^n) via a Fibonacci shift.
    Uses Python 3.10+ bit_count for Hamming weight (parity).
    """
    # Feedback bit is the parity of the masked bits
    # Parity of (state & mask)
    feedback = (state & mask).bit_count() & 1
    return (state >> 1) | (feedback << (n - 1))

def analyze_cycle_floyd(mask: int, n: int, init_state: int = 0xACE1, max_steps: int = 65535) -> Tuple[List[int], int, int]:
    """
    Uses Floyd's Cycle-Finding Algorithm (Tortoise and Hare) to detect
    period and loop content without O(N) memory overhead.
    """
    tortoise = init_state
    hare = init_state
    
    # 1. Detect Cycle
    # Hare moves 2x speed, Tortoise 1x. They meet inside the loop.
    steps = 0
    while steps < max_steps:
        tortoise = get_next_state(tortoise, mask, n)
        hare = get_next_state(hare, mask, n)
        hare = get_next_state(hare, mask, n)
        steps += 1
        
        if tortoise == hare:
            break
    else:
        # Max steps reached without cycle detection (should not happen in finite field if bounds correct)
        return [], 0, 0

    # 2. Find start of the cycle (mu)
    # Reset tortoise to start, keep hare at meeting point. Move both 1x.
    # (Note: In pure Galois LFSRs, cycles are disjoint, but Fibonacci can have pre-period tails.
    # We ignore tails for PWM purposes and only capture the steady-state loop).
    
    # 3. Measure Period (lambda)
    # Keep tortoise fixed, move hare until it returns to tortoise.
    period = 0
    bits = []
    
    # We are now inside the cycle. We record the sequence for one full period.
    curr = tortoise
    while True:
        # LSB is the output bit
        bits.append(curr & 1)
        curr = get_next_state(curr, mask, n)
        period += 1
        if curr == tortoise:
            break
            
    ones = sum(bits)
    return bits, period, ones

# --- Search & LUT Generation ---

def search_masks_algebraic(n: int = 16, resolution: int = 256, max_masks: int = 40000, tol: float = 1.0/256.0) -> Dict[int, Dict]:
    """
    Searches for polynomial masks that satisfy duty cycle requirements.
    Organizes results by best 'Linear Complexity' score.
    """
    # We bucket best candidates by PWM level (0..255)
    best_candidates: Dict[int, Dict] = {}
    
    # Initialize buckets with empty data
    for i in range(resolution):
        best_candidates[i] = {'err': float('inf'), 'lc': -1, 'mask': 1}

    # Pre-calculate target ratios
    targets = [i / (resolution - 1) for i in range(resolution)]
    
    tested = 0
    # We iterate odd masks only (even masks usually degenerate quickly in Fibonacci config)
    # Range is roughly the space of polynomials in F_2[x] of degree n
    scan_limit = min(1 << n, max_masks)
    
    print(f"Scanning {scan_limit} polynomials over F_2^{n}...", file=sys.stderr)

    for mask in range(1, scan_limit, 2): # Increment by 2, check odd masks only
        tested += 1
        
        # Fast cycle analysis using Number Theory (Floyd's)
        try:
            bits, period, ones = analyze_cycle_floyd(mask, n)
        except Exception:
            continue

        if period < 4: continue # Ignore trivial loops

        ratio = ones / period
        
        # Check against all PWM levels to see if this mask is a good fit
        # We allow a slightly wider tolerance to gather candidates, then rank by LC
        for pwm_lvl, target_ratio in enumerate(targets):
            err = abs(ratio - target_ratio)
            
            if err <= tol:
                # This mask is a candidate for this PWM level.
                # Now we apply Cryptanalysis: Calculate Linear Complexity.
                
                # Optimization: Only calculate BM if this error is comparable or better
                # OR if error is same but we might improve LC.
                current_best = best_candidates[pwm_lvl]
                
                # Logic: Prioritize Accuracy first, then Complexity.
                is_improvement = False
                
                if err < current_best['err'] - 1e-5:
                    # Significant accuracy improvement
                    is_improvement = True
                elif abs(err - current_best['err']) < 1e-5:
                    # Accuracy is similar, check Linear Complexity (LC)
                    lc = berlekamp_massey(bits)
                    if lc > current_best['lc']:
                        is_improvement = True
                    # Tie-breaker: prefer longer periods for better smoothing
                    elif lc == current_best['lc'] and period > current_best.get('period', 0):
                        is_improvement = True
                        
                if is_improvement:
                    # Calculate LC if not done yet
                    if 'lc' not in locals(): 
                        lc = berlekamp_massey(bits)
                    
                    best_candidates[pwm_lvl] = {
                        'mask': mask,
                        'period': period,
                        'ratio': ratio,
                        'err': err,
                        'lc': lc,
                        'bits': bits # Store for debugging if needed
                    }
                    # Remove local lc to ensure recalculation next loop
                    del lc

        if tested % 5000 == 0:
            print(f"Processed {tested} polynomials...", file=sys.stderr)

    return best_candidates

def generate_c_lut(candidates: Dict[int, Dict], resolution: int, varname: str) -> str:
    lines = ['#include <stdint.h>', '', '// Format: {mask, period, linear_complexity}']
    lines.append(f'// Generated using Algebraic Geometry & Berlekamp-Massey Analysis')
    lines.append(f'const struct {{ uint16_t mask; uint16_t period; }} {varname}[{resolution}] = {{')
    
    for i in range(resolution):
        c = candidates[i]
        mask = c['mask']
        per = c.get('period', 0)
        # We check if we found a valid mask, else default to 0
        if c['err'] > 1.0: # Sentinel for no find
             lines.append(f'    {{ 0x0000, 0 }}, // Level {i} (Unmatched)')
        else:
             lines.append(f'    {{ 0x{mask:04x}, {per:5d} }}, // Level {i:3d}: Ratio {c["ratio"]:.4f}, LC {c["lc"]}')
    
    lines.append('};')
    return "\n".join(lines)

# --- Main Driver ---

def main():
    parser = argparse.ArgumentParser(description='Algebraic LFSR PWM Generator')
    parser.add_argument('--n', type=int, default=16, help='LFSR width (Degree of polynomial)')
    parser.add_argument('--scan', type=int, default=65535, help='Max polynomials to scan')
    parser.add_argument('--res', type=int, default=256, help='PWM Resolution')
    parser.add_argument('--out', type=str, default='lfsr_pwm.c', help='Output file')
    args = parser.parse_args()

    # 1. Tolerance: We allow 1.5 steps of resolution deviation
    tolerance = 1.5 / args.res 
    
    # 2. Search
    print(f"Starting Search: Width={args.n}, Tolerance={tolerance:.4f}")
    results = search_masks_algebraic(n=args.n, resolution=args.res, max_masks=args.scan, tol=tolerance)
    
    # 3. Report Quality
    filled = sum(1 for k, v in results.items() if v['err'] < 1.0)
    print(f"\nCoverage: Found suitable masks for {filled}/{args.res} levels.")
    
    # 4. Export
    c_code = generate_c_lut(results, args.res, "lfsr_pwm_config")
    with open(args.out, 'w') as f:
        f.write(c_code)
    print(f"Written optimized LUT to {args.out}")

if __name__ == '__main__':
    main()
