#!/usr/bin/env python3
"""
lfsr_pwm_lut_generator.py

Searches 16-bit LFSR feedback masks to build a lookup table (LUT) mapping 8-bit PWM
levels (0..255) to LFSR configurations that approximate the requested duty-cycle
ratio.

Approach summary:
- Use a 16-bit Fibonacci-style LFSR: state -> (state >> 1) | (feedback << (n-1))
  where feedback is parity(state & mask).
- For each candidate feedback mask (1..(1<<n)-1) we simulate the LFSR until it
  repeats or until a hard limit and measure the period length and number of 1s
  in the output bitstream (we use the LSB as output).
- For each PWM level (0..255) we pick a mask whose ones/period ratio is closest
  to the desired ratio (d/255) and also satisfies a spectral/autocorrelation
  heuristic (low short-lag autocorrelation).
- Exports a C-friendly LUT (array of uint16_t masks) and optional metadata.

Notes and limitations:
- Brute-forcing all 65535 non-zero masks is feasible but will take some time
  (tens of seconds to a few minutes in pure Python). Use --max-masks to limit
  the search for quick experiments.
- The method matches long-run ratio over the full period of that LFSR. If you
  need precise short-window matching (e.g. windows of 256 samples), additional
  processing (searching for starting offsets that produce exact counts in a
  window) should be added.

Usage:
    python3 lfsr_pwm_lut_generator.py --n 16 --window 256 --tolerance 1/256

"""

from __future__ import annotations
import argparse
import math
import sys
from typing import Dict, List, Tuple, Optional


def parity(x: int) -> int:
    # fast parity using builtin bit_count
    return x.bit_count() & 1


def simulate_lfsr(mask: int, n: int = 16, init_state: int = 1, max_steps: Optional[int] = None) -> Tuple[List[int], int]:
    """Simulate Fibonacci LFSR with given mask and return output bits list and period.

    - mask: integer whose set bits indicate tap positions (0..n-1) used to compute feedback.
    - output bit chosen as LSB of the state (state & 1). Initial state must be non-zero.
    """
    if init_state == 0:
        raise ValueError("Initial LFSR state must be non-zero")
    if max_steps is None:
        max_steps = (1 << n)  # safe upper bound

    seen = set()
    state = init_state & ((1 << n) - 1)
    bits: List[int] = []
    step = 0
    while state not in seen and step < max_steps:
        seen.add(state)
        out = state & 1
        bits.append(out)
        fb = parity(state & mask)
        state = (state >> 1) | (fb << (n - 1))
        step += 1
    period = len(bits)
    return bits, period


def ones_count_from_bits(bits: List[int]) -> int:
    return sum(bits)


def autocorrelation_metric(bits: List[int], max_lag: int = 64) -> float:
    """Compute a simple normalized autocorrelation metric over lags 1..max_lag.

    Returns the RMS of autocorrelation values (should be small for good pseudo-random sequences).
    """
    if len(bits) == 0:
        return float('inf')
    n = len(bits)
    mean = sum(bits) / n
    var = sum((b - mean) ** 2 for b in bits) / n
    if var == 0:
        return float('inf')
    ac_vals = []
    max_lag = min(max_lag, n - 1)
    for lag in range(1, max_lag + 1):
        s = 0.0
        for i in range(n - lag):
            s += (bits[i] - mean) * (bits[i + lag] - mean)
        ac = s / ((n - lag) * var)
        ac_vals.append(ac)
    # RMS of autocorrelation magnitudes
    rms = math.sqrt(sum(a * a for a in ac_vals) / len(ac_vals)) if ac_vals else 0.0
    return rms


def search_masks(n: int = 16, min_period: int = 3, max_masks: Optional[int] = None, acorr_limit: float = 0.15) -> List[Dict]:
    """Brute force candidate masks. Returns a list of dicts with mask metadata.

    Each dict contains: mask, period, ones, ratio, acorr
    """
    candidates = []
    max_mask_value = (1 << n)
    tested = 0
    for mask in range(1, max_mask_value):
        # optional early stop
        if max_masks is not None and tested >= max_masks:
            break
        tested += 1
        try:
            bits, period = simulate_lfsr(mask, n=n, init_state=1, max_steps=(1 << n))
        except Exception as e:
            # skip problematic masks
            continue
        if period < min_period:
            continue
        ones = ones_count_from_bits(bits)
        ratio = ones / period
        acorr = autocorrelation_metric(bits, max_lag=64)
        if acorr > acorr_limit:
            # skip masks with bad short-lag autocorrelation
            continue
        candidates.append({
            'mask': mask,
            'period': period,
            'ones': ones,
            'ratio': ratio,
            'acorr': acorr,
        })
        # small progress indicator
        if tested % 4096 == 0:
            print(f"Searched {tested} masks, found {len(candidates)} candidates so far...", file=sys.stderr)
    print(f"Finished search: tested {tested} masks, candidates {len(candidates)}", file=sys.stderr)
    return candidates


def build_lut(candidates: List[Dict], resolution: int = 256, tol: float = 1/256) -> Tuple[List[int], List[Tuple[int, float]]]:
    """For each pwm level in 0..resolution-1, pick the best candidate whose ones/period is within tol.

    Returns LUT (list of chosen mask integers) and metadata list of (mask, achieved_ratio)
    If no candidate is within tol, picks the closest candidate anyway.
    """
    lut: List[int] = []
    meta: List[Tuple[int, float]] = []
    for d in range(resolution):
        desired = d / (resolution - 1)  # map 0..255 -> 0..1
        best = None
        best_err = float('inf')
        for c in candidates:
            err = abs(c['ratio'] - desired)
            if err < best_err:
                best_err = err
                best = c
                if err <= tol:
                    break
        if best is None:
            # fallback: just use mask 1
            lut.append(1)
            meta.append((1, 0.0))
        else:
            lut.append(best['mask'])
            meta.append((best['mask'], best['ratio']))
    return lut, meta


def export_c_array(lut: List[int], varname: str = 'lfsr_masks', filename: Optional[str] = None) -> str:
    """Return a C source snippet for a uint16_t LUT. Optionally write to filename."""
    lines = []
    lines.append('#include <stdint.h>')
    lines.append(f'const uint16_t {varname}[{len(lut)}] = {{')
    per_line = 8
    for i in range(0, len(lut), per_line):
        chunk = lut[i:i+per_line]
        lines.append('    ' + ', '.join(f'0x{v:04x}' for v in chunk) + (',' if i+per_line < len(lut) else ''))
    lines.append('};')
    text = '\n'.join(lines) + '\n'
    if filename:
        with open(filename, 'w') as f:
            f.write(text)
    return text


def parse_args():
    p = argparse.ArgumentParser(description='LFSR PWM LUT generator')
    p.add_argument('--n', type=int, default=16, help='LFSR width in bits')
    p.add_argument('--min-period', type=int, default=3, help='Minimum period to accept')
    p.add_argument('--max-masks', type=int, default=None, help='Limit number of masks to test (for quick runs)')
    p.add_argument('--acorr-limit', type=float, default=0.18, help='Max RMS autocorr to accept')
    p.add_argument('--tol', type=float, default=1/256, help='Duty ratio tolerance when picking mask')
    p.add_argument('--resolution', type=int, default=256, help='PWM resolution (entries in LUT)')
    p.add_argument('--out', type=str, default='lfsr_lut.c', help='Output C filename to write LUT')
    return p.parse_args()


def main():
    args = parse_args()
    print('Searching LFSR masks...')
    candidates = search_masks(n=args.n, min_period=args.min_period, max_masks=args.max_masks, acorr_limit=args.acorr_limit)
    if not candidates:
        print('No candidates found. Try relaxing filters (acorr_limit higher or min_period lower) or increase max_masks.', file=sys.stderr)
        sys.exit(1)
    print(f'Building LUT with resolution {args.resolution} and tolerance {args.tol}...')
    lut, meta = build_lut(candidates, resolution=args.resolution, tol=args.tol)
    c_text = export_c_array(lut, varname='lfsr_masks', filename=args.out)
    print(f'Wrote C LUT to {args.out}')
    # Also print a small summary of the first 16 entries
    print('\n# Summary (first 16 levels):')
    for i in range(min(16, len(meta))):
        mask, ratio = meta[i]
        print(f'Level {i:3d}: mask=0x{mask:04x}, ratio={ratio:.6f}')


if __name__ == '__main__':
    main()
