#!/usr/bin/env python3
"""
verify_dataset.py

Verify consistency between the source signed int16 dataset (lfsr_map.bin)
and the binned outputs in directory '8bit_set/0xXX.bin'.

Usage:
    python3 verify_dataset.py --input lfsr_map.bin --bins 8bit_set

Notes:
- Source format: signed int16 values laid out as [mask][seed] where DIM_SIZE=65536.
- Each bin file is a sequence of little-endian pairs (uint16 mask, uint16 seed).
- Mapping used to compute bin index:
    pwm8 = ((int16_value + 32768) * 255) // 65535
"""

import os
import sys
import argparse
import mmap
import struct
import math
from collections import defaultdict

# Try to import numpy (fast path). If not available, fall back.
try:
    import numpy as np
    HAVE_NUMPY = True
except Exception:
    HAVE_NUMPY = False

DIM_SIZE = 65536
ENTRY_SIZE = 4  # bytes per (uint16, uint16)
SOURCE_DTYPE_BYTES = 2  # int16 on disk
TOTAL_POINTS = DIM_SIZE * DIM_SIZE

def pwm8_from_int16_scalar(v: int) -> int:
    # v is signed int16 (-32768..32767)
    return ((int(v) + 32768) * 255) // 65535

def compute_expected_counts_numpy(src_memmap_path, block_rows=256):
    """
    Compute expected per-bin counts using numpy.memmap.
    Process rows in blocks to vectorize and reduce Python overhead.
    Returns a numpy array of dtype=np.uint64 length 256.
    """
    expected = np.zeros(256, dtype=np.uint64)

    # memory-map the whole file as int16 and shape it (DIM_SIZE, DIM_SIZE)
    mm = np.memmap(src_memmap_path, dtype=np.int16, mode='r', shape=(DIM_SIZE, DIM_SIZE))

    # Process in blocks of rows
    rows = DIM_SIZE
    for start in range(0, rows, block_rows):
        end = min(start + block_rows, rows)
        block = mm[start:end]                # shape (block_rows, DIM_SIZE)
        # promote to int32 to avoid overflow when adding 32768
        block32 = block.astype(np.int32, copy=False)
        # compute pwm for entire block vectorized
        # ensure integer division (//) yields integer dtype
        pwm_block = ((block32 + 32768) * 255) // 65535  # dtype: int32
        # Flatten and bincount
        flat = pwm_block.ravel()
        counts = np.bincount(flat.astype(np.int64), minlength=256)  # int64
        # Cast to expected dtype before adding to avoid ufunc casting issues
        expected += counts.astype(expected.dtype)
        if ((start // block_rows) & 0x7) == 0x7:  # progress every 8 blocks (tunable)
            pct = (end / rows) * 100.0
            print(f"[expected_counts] {end}/{rows} rows processed ({pct:.2f}%)", end='\r', flush=True)

    print()  # newline after progress
    return expected

def compute_expected_counts_mmap(src_path):
    """Compute expected counts without numpy using mmap and struct; slower but works."""
    expected = [0] * 256
    with open(src_path, "rb") as f:
        mm = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
        total_bytes = mm.size()
        assert total_bytes == DIM_SIZE * DIM_SIZE * SOURCE_DTYPE_BYTES
        # process row by row (each row is DIM_SIZE int16 values = DIM_SIZE*2 bytes)
        row_bytes = DIM_SIZE * SOURCE_DTYPE_BYTES
        for row in range(DIM_SIZE):
            offset = row * row_bytes
            chunk = mm[offset: offset + row_bytes]
            mv = memoryview(chunk)
            # iterate 2-byte steps
            for i in range(0, len(mv), 2):
                val = int.from_bytes(mv[i:i+2], byteorder='little', signed=True)
                b = pwm8_from_int16_scalar(val)
                expected[b] += 1
            if (row & 0x1FFF) == 0x1FFF:
                print(f"[expected_counts] {row}/{DIM_SIZE} rows processed", end='\r', flush=True)
        mm.close()
    print()
    return expected

def read_bin_file_numpy(path):
    """Read a bin file into an Nx2 uint16 numpy array (mask, seed)."""
    import numpy as np
    fsize = os.path.getsize(path)
    if fsize % ENTRY_SIZE != 0:
        raise ValueError(f"Bin file {path} has invalid size {fsize}")
    count = fsize // ENTRY_SIZE
    if count == 0:
        return np.empty((0,2), dtype=np.uint16)
    b = open(path, "rb").read()
    arr = np.frombuffer(b, dtype=np.uint16).reshape(-1, 2)
    return arr  # shape (count,2)

def read_bin_file_mmap_iter(path):
    """Generator yielding (mask, seed) pairs from bin file without numpy."""
    with open(path, "rb") as f:
        mm = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
        size = mm.size()
        if size % ENTRY_SIZE != 0:
            mm.close()
            raise ValueError(f"Invalid bin file size: {path} size {size}")
        for off in range(0, size, ENTRY_SIZE):
            mask, seed = struct.unpack_from("<HH", mm, off)
            yield mask, seed
        mm.close()

def analyze_bins(src_path, bins_dir, expected_counts):
    """For each bin file, compute stats and verify membership correctness."""
    # open source memmap if numpy exists for fast random access
    if HAVE_NUMPY:
        src_mm = np.memmap(src_path, dtype=np.int16, mode='r', shape=(DIM_SIZE, DIM_SIZE))
    else:
        src_f = open(src_path, "rb")
        src_mm = mmap.mmap(src_f.fileno(), 0, access=mmap.ACCESS_READ)

    summary = []
    total_actual = 0
    total_expected = int(expected_counts.sum()) if HAVE_NUMPY else sum(expected_counts)

    for bin_idx in range(256):
        fname = os.path.join(bins_dir, f"0x{bin_idx:02X}.bin")
        if not os.path.exists(fname):
            print(f"[WARN] Missing bin file {fname}; expected_count={expected_counts[bin_idx]}")
            actual_count = 0
            summary.append((bin_idx, 0, expected_counts[bin_idx], 0, 0, 0, 0, 0))
            continue

        fsize = os.path.getsize(fname)
        if fsize % ENTRY_SIZE != 0:
            print(f"[ERROR] bin {fname} has weird size {fsize}")
            continue
        actual_count = fsize // ENTRY_SIZE
        total_actual += actual_count

        min_mask = 65535
        max_mask = 0
        min_seed = 65535
        max_seed = 0
        unique_masks = set()
        mismatches = []
        mismatches_found = 0

        if HAVE_NUMPY:
            arr = read_bin_file_numpy(fname)  # shape (n,2)
            if arr.shape[0] != actual_count:
                print(f"[WARN] read size mismatch {fname}")
            if actual_count > 0:
                masks = arr[:,0].astype(np.uint32)
                seeds = arr[:,1].astype(np.uint32)
                min_mask = int(masks.min())
                max_mask = int(masks.max())
                min_seed = int(seeds.min())
                max_seed = int(seeds.max())
                unique_masks_count = int(np.unique(masks).size)
                # vectorized fetch of source values
                src_vals = src_mm[masks, seeds].astype(np.int32)
                pwm8s = ((src_vals + 32768) * 255) // 65535
                bad_idx = np.nonzero(pwm8s != bin_idx)[0]
                mismatches_found = int(bad_idx.size)
                if mismatches_found:
                    for bi in bad_idx[:20]:
                        m = int(masks[bi]); s = int(seeds[bi]); sv = int(src_vals[bi]); expb = int(pwm8s[bi])
                        mismatches.append((m, s, sv, expb))
            else:
                unique_masks_count = 0
        else:
            unique_masks_count = 0
            for mask, seed in read_bin_file_mmap_iter(fname):
                if mask < min_mask: min_mask = mask
                if mask > max_mask: max_mask = mask
                if seed < min_seed: min_seed = seed
                if seed > max_seed: max_seed = seed
                unique_masks.add(mask)
                offset = (mask * DIM_SIZE + seed) * SOURCE_DTYPE_BYTES
                raw = src_mm[offset: offset + 2]
                sval = struct.unpack_from("<h", raw)[0]  # int16
                expb = pwm8_from_int16_scalar(sval)
                if expb != bin_idx:
                    if mismatches_found < 20:
                        mismatches.append((mask, seed, sval, expb))
                    mismatches_found += 1
            unique_masks_count = len(unique_masks)

        expected_count = int(expected_counts[bin_idx]) if HAVE_NUMPY else expected_counts[bin_idx]

        # Print per-bin summary
        print(f"Bin 0x{bin_idx:02X}: actual={actual_count:10,} expected={expected_count:10,} diff={actual_count-expected_count:10,} "
              f"mismatches={mismatches_found:6}", flush=True)

        print(f"    mask: min={min_mask} max={max_mask}    seed: min={min_seed} max={max_seed}    unique_masks={unique_masks_count}")
        if mismatches_found:
            print("    Examples of mismatches (mask,seed,source_val,expected_bin):")
            for ex in mismatches:
                print(f"       {ex}")
        summary.append((bin_idx, actual_count, expected_count, actual_count-expected_count, mismatches_found, min_mask, max_mask, unique_masks_count))

    # final checks
    print("\nSUMMARY:")
    print(f"  total expected sum = {total_expected:,}")
    print(f"  total actual   sum = {total_actual:,}")
    if total_expected != total_actual:
        print("  -> TOTAL MISMATCH between expected and actual counts!")
    else:
        print("  -> totals match.")

    if not HAVE_NUMPY:
        src_mm.close()
        src_f.close()

    return summary

def main():
    p = argparse.ArgumentParser(description="Verify lfsr_map.bin vs 8bit_set bins")
    p.add_argument("--input", "-i", required=True, help="Path to signed int16 input file (lfsr_map.bin)")
    p.add_argument("--bins", "-b", required=True, help="Directory containing 0x00.bin .. 0xFF.bin")
    p.add_argument("--no-numpy", action="store_true", help="Force no-numpy slow path (for testing)")
    args = p.parse_args()

    if args.no_numpy:
        global HAVE_NUMPY
        HAVE_NUMPY = False

    if not os.path.exists(args.input):
        print("Input file not found:", args.input)
        sys.exit(2)
    if not os.path.isdir(args.bins):
        print("Bins dir not found or not a directory:", args.bins)
        sys.exit(2)

    print("Using numpy:", HAVE_NUMPY)
    print("Computing expected per-bin counts from source...")
    if HAVE_NUMPY:
        expected_counts = compute_expected_counts_numpy(args.input)
    else:
        expected_counts = compute_expected_counts_mmap(args.input)

    print("Analyzing bin files and verifying membership...")
    summary = analyze_bins(args.input, args.bins, expected_counts)

    print("\nDone.")

if __name__ == "__main__":
    main()
