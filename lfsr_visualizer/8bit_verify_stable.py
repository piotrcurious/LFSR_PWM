#!/usr/bin/env python3
"""
verify_dataset_signed_positive_only.py

Verifies that bins contain only entries coming from positive source values.
Also computes expected per-bin counts but only counting positive source values.

Usage:
    python3 verify_dataset_signed_positive_only.py --input lfsr_map.bin --bins 8bit_set
"""

import os
import sys
import argparse
import mmap
import struct
from collections import defaultdict

try:
    import numpy as np
    HAVE_NUMPY = True
except Exception:
    HAVE_NUMPY = False

DIM_SIZE = 65536
ENTRY_SIZE = 4
SOURCE_BYTES = 2

def pwm8_from_int16_positive(v):
    """Assumes v is int16 and > 0. Maps signed domain to 0..255."""
    # Use same mapping as analyzer for positive values
    return ((int(v) ) * 255) // 32767

def compute_expected_counts_positive_numpy(src_path, block_rows=256):
    expected = np.zeros(256, dtype=np.uint64)
    mm = np.memmap(src_path, dtype=np.int16, mode='r', shape=(DIM_SIZE, DIM_SIZE))
    rows = DIM_SIZE
    for start in range(0, rows, block_rows):
        end = min(start + block_rows, rows)
        block = mm[start:end].astype(np.int32, copy=False)
        # mask positives only
        mask_pos = block > 0
        # compute pwm for all entries (but we'll zero out non-positive later)
        pwm_block = ((block) * 255) // 32767
        pwm_block[~mask_pos] = -1  # mark invalids
        flat = pwm_block.ravel()
        # filter out -1 and bincount the rest
        valid = flat[flat >= 0].astype(np.int64)
        if valid.size:
            counts = np.bincount(valid, minlength=256)
            expected += counts.astype(expected.dtype)
        if ((start // block_rows) & 0x7) == 0x7:
            print(f"[expected_counts] {end}/{rows} rows processed ({(end/rows)*100:.2f}%)", end='\r', flush=True)
    print()
    return expected

def compute_expected_counts_positive_mmap(src_path):
    expected = [0] * 256
    with open(src_path, "rb") as f:
        mm = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
        total_bytes = mm.size()
        assert total_bytes == DIM_SIZE * DIM_SIZE * SOURCE_BYTES
        row_bytes = DIM_SIZE * SOURCE_BYTES
        for r in range(DIM_SIZE):
            off = r * row_bytes
            chunk = mm[off: off + row_bytes]
            mv = memoryview(chunk)
            for i in range(0, len(mv), 2):
                val = int.from_bytes(mv[i:i+2], byteorder='little', signed=True)
                if val > 0:
                    b = pwm8_from_int16_positive(val)
                    expected[b] += 1
            if (r & 0x1FFF) == 0x1FFF:
                print(f"[expected_counts] {r}/{DIM_SIZE} rows processed", end='\r', flush=True)
        mm.close()
    print()
    return expected

def read_bin_pairs(path):
    with open(path, "rb") as f:
        data = f.read()
    if len(data) % ENTRY_SIZE != 0:
        raise ValueError("Invalid bin file size")
    return struct.iter_unpack("<HH", data)

def analyze_bins(src_path, bins_dir, expected_counts):
    # open source for random access
    if HAVE_NUMPY:
        src = np.memmap(src_path, dtype=np.int16, mode='r', shape=(DIM_SIZE, DIM_SIZE))
    else:
        sf = open(src_path, "rb")
        src_mm = mmap.mmap(sf.fileno(), 0, access=mmap.ACCESS_READ)

    total_expected = int(expected_counts.sum()) if HAVE_NUMPY else sum(expected_counts)
    total_actual = 0

    for b in range(256):
        fname = os.path.join(bins_dir, f"0x{b:02X}.bin")
        if not os.path.exists(fname):
            print(f"Bin 0x{b:02X}: MISSING (expected {expected_counts[b]})")
            continue
        size = os.path.getsize(fname)
        if size % ENTRY_SIZE != 0:
            print(f"Bin 0x{b:02X}: invalid size {size}")
            continue
        count = size // ENTRY_SIZE
        total_actual += count

        violations = 0
        viol_examples = []
        min_mask = 65535; max_mask = 0; min_seed = 65535; max_seed = 0
        unique_masks = set()

        # iterate pairs
        for (mask, seed) in read_bin_pairs(fname):
            if mask < min_mask: min_mask = mask
            if mask > max_mask: max_mask = mask
            if seed < min_seed: min_seed = seed
            if seed > max_seed: max_seed = seed
            unique_masks.add(mask)

            if HAVE_NUMPY:
                sval = int(src[mask, seed])
            else:
                off = (mask * DIM_SIZE + seed) * SOURCE_BYTES
                sval = struct.unpack_from("<h", src_mm, off)[0]

            if sval <= 0:
                violations += 1
                if len(viol_examples) < 20:
                    viol_examples.append((mask, seed, sval))
        print(f"Bin 0x{b:02X}: actual={count:,} expected={expected_counts[b]:,} diff={count - expected_counts[b]:,} violations={violations}")
        print(f"    mask: min={min_mask} max={max_mask} seeds: min={min_seed} max={max_seed} unique_masks={len(unique_masks)}")
        if violations:
            print("    Examples of entries coming from non-positive source values:")
            for ex in viol_examples:
                print("      ", ex)

    print("\nTotals: expected_sum=", total_expected, " actual_sum=", total_actual)
    if not HAVE_NUMPY:
        src_mm.close()
        sf.close()

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--input", "-i", required=True)
    p.add_argument("--bins",  "-b", required=True)
    p.add_argument("--no-numpy", action="store_true")
    args = p.parse_args()

    if args.no_numpy:
        global HAVE_NUMPY
        HAVE_NUMPY = False

    if not os.path.exists(args.input) or not os.path.isdir(args.bins):
        print("Invalid paths")
        sys.exit(2)

    print("Using numpy:", HAVE_NUMPY)
    print("Computing expected per-bin counts (only positive source values)...")
    if HAVE_NUMPY:
        expected = compute_expected_counts_positive_numpy(args.input)
    else:
        expected = compute_expected_counts_positive_mmap(args.input)

    print("Analyzing bins for invalid (non-positive) entries...")
    analyze_bins(args.input, args.bins, expected)

if __name__ == '__main__':
    main()
