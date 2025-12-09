#!/usr/bin/env python3
"""
lfsr_pwm_algebraic_multiproc.py

Updated LFSR PWM LUT generator with exhaustive cycle decomposition per-mask
and multiprocessing support.

Key improvements vs original:
- Enumerates *all disjoint cycles* for each mask (not just single seed).
- Scans full mask range by default (1..2^n-1); optional `--scan` can limit masks.
- Picks the *best* cycle per PWM level (minimizing error) and uses BM LC
  as a tie-breaker (computed lazily only when a candidate improves a bucket).
- Multiprocessing: masks are split into chunks processed in parallel; each
  worker returns a per-level best-candidate set which the parent merges.

Usage example:
    python3 lfsr_pwm_algebraic_multiproc.py --n 16 --scan 65535 --res 256 --out lfsr_pwm.c --processes 8

"""

from __future__ import annotations
import argparse
import sys
import math
from typing import Dict, List, Tuple, Optional
from multiprocessing import Pool, cpu_count

# ----------------------- Core algorithms -----------------------

def berlekamp_massey(bits: List[int]) -> int:
    """Compute linear complexity (LC) using Berlekamp-Massey for a binary seq."""
    n = len(bits)
    b = [0] * n
    c = [0] * n
    b[0] = 1
    c[0] = 1
    l = 0
    m = -1

    for N in range(n):
        d = 0
        for i in range(l + 1):
            d ^= c[i] & bits[N - i]
        if d == 1:
            t = c.copy()
            p = N - m
            for i in range(n - p):
                c[i + p] ^= b[i]
            if l <= N // 2:
                l = N + 1 - l
                m = N
                b = t
    return l


def get_next_state(state: int, mask: int, n: int) -> int:
    """Fibonacci-style 1-bit feedback shift right; feedback inserted at MSB."""
    feedback = (state & mask).bit_count() & 1
    return (state >> 1) | (feedback << (n - 1))


def decompose_cycles_for_mask(mask: int, n: int) -> List[Tuple[List[int], int]]:
    """
    Enumerate all disjoint cycles for this mask by scanning the state graph.
    Returns a list of (bits_list, period) for each cycle found (skips state 0).
    """
    max_state = 1 << n
    visited = bytearray(max_state)
    cycles: List[Tuple[List[int], int]] = []

    for s in range(1, max_state):
        if visited[s]:
            continue
        trace = {}
        seq = []
        curr = s
        step = 0
        while True:
            if visited[curr]:
                # This path leads into a previously seen cycle or node: mark path visited and stop
                for t in seq:
                    visited[t] = 1
                break
            if curr in trace:
                # Found a cycle: nodes trace[curr:] form the cycle
                cycle_nodes = seq[trace[curr]:]
                bits = [node & 1 for node in cycle_nodes]
                cycles.append((bits, len(bits)))
                for t in seq:
                    visited[t] = 1
                break
            trace[curr] = step
            seq.append(curr)
            step += 1
            curr = get_next_state(curr, mask, n)
    return cycles


# ----------------------- Worker / search -----------------------

def _init_bucket(resolution: int) -> Dict[int, Dict]:
    return {i: {'err': float('inf'), 'lc': -1, 'mask': 0, 'period': 0, 'ratio': 0.0} for i in range(resolution)}


def process_mask_chunk(args) -> Dict[int, Dict]:
    """
    Worker function to process a contiguous chunk of masks.
    args: (mask_start, mask_end, n, resolution)
    Returns a per-PWM-level best_candidates dict for this chunk.
    """
    mask_start, mask_end, n, resolution = args
    targets = [i / (resolution - 1) for i in range(resolution)]
    best = _init_bucket(resolution)

    for mask in range(mask_start, mask_end):
        if mask == 0:
            continue
        try:
            cycles = decompose_cycles_for_mask(mask, n)
        except Exception:
            continue

        for bits, period in cycles:
            if period < 1:
                continue
            ones = sum(bits)
            ratio = ones / period

            # Consider this cycle for every PWM level; compute LC lazily only when it helps
            for pwm_lvl, target in enumerate(targets):
                err = abs(ratio - target)
                current = best[pwm_lvl]

                # Improvement criteria: smaller err OR same err but longer period OR same err/period and better LC
                if err + 1e-15 < current['err'] or (abs(err - current['err']) < 1e-15 and period > current.get('period', 0)):
                    # Compute LC lazily because BM is costly
                    lc = berlekamp_massey(bits)
                    # Accept on better err OR equal err but higher LC OR equal err+LC but longer period
                    accept = False
                    if err + 1e-15 < current['err']:
                        accept = True
                    elif abs(err - current['err']) < 1e-15:
                        if lc > current['lc']:
                            accept = True
                        elif lc == current['lc'] and period > current.get('period', 0):
                            accept = True
                    if accept:
                        best[pwm_lvl] = {
                            'mask': mask,
                            'period': period,
                            'ratio': ratio,
                            'err': err,
                            'lc': lc,
                            # keep a short sample for debugging (truncate large cycles)
                            'sample_bits': bits if period <= 256 else bits[:256]
                        }
    return best


def merge_buckets(master: Dict[int, Dict], chunk: Dict[int, Dict]) -> None:
    """Merge a worker's per-level results into master in-place."""
    for lvl, entry in chunk.items():
        if entry['err'] == float('inf'):
            continue
        cur = master[lvl]
        e_err = entry['err']
        # Choose by minimal error first
        if e_err + 1e-15 < cur['err']:
            master[lvl] = entry
        elif abs(e_err - cur['err']) < 1e-15:
            # tie-break by LC then period
            if entry['lc'] > cur['lc'] or (entry['lc'] == cur['lc'] and entry['period'] > cur.get('period', 0)):
                master[lvl] = entry


def search_masks_algebraic(n: int = 16,
                           resolution: int = 256,
                           max_masks: Optional[int] = None,
                           processes: int = 0) -> Dict[int, Dict]:
    """
    Distribute mask scanning across workers. Returns best candidate per PWM level.
    """
    max_state_space = 1 << n
    if max_masks is None:
        max_masks = max_state_space
    scan_limit = min(max_state_space, max_masks)

    if processes <= 0:
        processes = max(1, cpu_count())

    print(f"Scanning masks [1..{scan_limit-1}] over F_2^{n} using {processes} processes...", file=sys.stderr)

    # Decide chunking: split the full mask range into `processes` chunks (simple balanced split)
    masks = list(range(1, scan_limit))
    total_masks = len(masks)
    if total_masks == 0:
        return _init_bucket(resolution)

    chunk_size = max(1, total_masks // processes)
    ranges = []
    i = 0
    while i < total_masks:
        start_mask = masks[i]
        end_index = min(i + chunk_size, total_masks)
        end_mask = masks[end_index - 1] + 1  # end is exclusive
        ranges.append((start_mask, end_mask, n, resolution))
        i = end_index

    # If there are more processes than ranges, adjust
    processes_to_use = min(processes, len(ranges))

    master = _init_bucket(resolution)

    if processes_to_use == 1:
        # Single-process path (easy to debug)
        for r in ranges:
            chunk_res = process_mask_chunk(r)
            merge_buckets(master, chunk_res)
    else:
        with Pool(processes=processes_to_use) as pool:
            for idx, chunk_res in enumerate(pool.imap_unordered(process_mask_chunk, ranges, chunksize=1), 1):
                print(f"Merging chunk {idx}/{len(ranges)}...", file=sys.stderr)
                merge_buckets(master, chunk_res)

    return master


# ----------------------- C LUT Export -----------------------

def generate_c_lut(candidates: Dict[int, Dict], resolution: int, varname: str) -> str:
    lines = ['#include <stdint.h>', '', '// Format: {mask, period}']
    lines.append(f'// Generated using exhaustive cycle decomposition + Berlekamp-Massey')
    lines.append(f'const struct {{ uint16_t mask; uint16_t period; }} {varname}[{resolution}] = {{')

    for i in range(resolution):
        c = candidates[i]
        mask = c.get('mask', 0)
        per = c.get('period', 0)
        if c['err'] == float('inf'):
            lines.append(f'    {{ 0x0000, 0 }}, // Level {i:3d} (Unmatched)')
        else:
            lines.append(f'    {{ 0x{mask:04x}, {per:5d} }}, // Level {i:3d}: Ratio {c["ratio"]:.6f}, err {c["err"]:.6g}, LC {c["lc"]}')

    lines.append('};')
    return "\n".join(lines)


# ----------------------- CLI -----------------------

def main():
    parser = argparse.ArgumentParser(description='Algebraic LFSR PWM LUT generator (multiprocess)')
    parser.add_argument('--n', type=int, default=16, help='LFSR width (degree)')
    parser.add_argument('--scan', type=int, default=(1 << 16), help='Max polynomials to scan (default = 2^n)')
    parser.add_argument('--res', type=int, default=256, help='PWM resolution (buckets)')
    parser.add_argument('--out', type=str, default='lfsr_pwm.c', help='Output C filename')
    parser.add_argument('--processes', type=int, default=0, help='Multiprocessing worker count (0=auto)')
    parser.add_argument('--treat-extremes-const', action='store_true', help='Force level 0 and highest to constant outputs')
    args = parser.parse_args()

    n = args.n
    resolution = args.res
    max_masks = args.scan
    processes = args.processes

    if max_masks <= 1:
        print('scan must be > 1', file=sys.stderr)
        sys.exit(1)

    print(f"Starting Search: Width={n}, Resolution={resolution}, Masks<= {max_masks}, Processes={processes}")

    results = search_masks_algebraic(n=n, resolution=resolution, max_masks=max_masks, processes=processes)

    # If user asked extremes to be constants, set them explicitly
    if args.treat_extremes_const:
        # Level 0 -> always 0 (mask 0), Level max -> always 1 (we use mask 0xffff with period 1 as sentinel)
        results[0] = {'mask': 0x0000, 'period': 1, 'ratio': 0.0, 'err': 0.0, 'lc': 0}
        results[resolution - 1] = {'mask': 0xffff, 'period': 1, 'ratio': 1.0, 'err': 0.0, 'lc': 0}

    # Coverage report
    found = sum(1 for v in results.values() if v['err'] != float('inf'))
    print(f"Coverage: Found candidates for {found}/{resolution} levels.")

    # Export C LUT
    c_code = generate_c_lut(results, resolution, 'lfsr_pwm_config')
    with open(args.out, 'w') as f:
        f.write(c_code)
    print(f"Written LUT to {args.out}")


if __name__ == '__main__':
    main()
