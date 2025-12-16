// lfsr_accel.hpp
#pragma once
#include <cstdint>
#include <cstring>
#include <vector>
#include <fstream>
#include <iostream>
#include <unordered_map>
#include <immintrin.h> // AVX2 / AVX512 intrinsics
#include <cpuid.h>
#include <stdexcept>
#include <algorithm>
#include <nlohmann/json.hpp> // put single-header in include path

using json = nlohmann::json;
using u64 = uint64_t;
using u32 = uint32_t;

// -------------------- Config loader --------------------
struct Config {
    int n = 16;
    int resolution = 256;
    int threads = 0;
    u64 max_masks = 0;
    double lc_weight = 0.01;
    double period_weight = 0.001;
    double spectral_weight = 10.0;
    int min_taps = 1;
    double min_period_ratio = 0.0;
    int min_sample_bits = 1024;
    int max_sample_bits = 16384;
    int block_size = 256;
    double max_block_error = 0.1;
    bool export_bins = true;
    std::string out_dir = "8bit_set";
    std::string simd_preference = "auto"; // "auto", "avx512", "avx2", "scalar"
    int vector_batch_size = 64;
};

static Config load_config_from_file(const std::string &path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Cannot open config file: " + path);
    json j; f >> j;
    Config c;
    if (j.contains("engine")) {
        auto e = j["engine"];
        if (e.contains("n")) c.n = e["n"];
        if (e.contains("resolution")) c.resolution = e["resolution"];
        if (e.contains("threads")) c.threads = e["threads"];
    }
    if (j.contains("search")) {
        auto s = j["search"];
        if (s.contains("max_masks")) c.max_masks = s["max_masks"];
        if (s.contains("min_taps")) c.min_taps = s["min_taps"];
        if (s.contains("min_irreducible_degree")) ; // handled elsewhere
        if (s.contains("min_period_ratio")) c.min_period_ratio = s["min_period_ratio"];
    }
    if (j.contains("sampling")) {
        auto s = j["sampling"];
        if (s.contains("min_sample_bits")) c.min_sample_bits = s["min_sample_bits"];
        if (s.contains("max_sample_bits")) c.max_sample_bits = s["max_sample_bits"];
        if (s.contains("block_size")) c.block_size = s["block_size"];
        if (s.contains("max_block_error")) c.max_block_error = s["max_block_error"];
    }
    if (j.contains("scoring")) {
        auto s = j["scoring"];
        if (s.contains("lc_weight")) c.lc_weight = s["lc_weight"];
        if (s.contains("period_weight")) c.period_weight = s["period_weight"];
        if (s.contains("spectral_weight")) c.spectral_weight = s["spectral_weight"];
    }
    if (j.contains("export")) {
        auto e = j["export"];
        if (e.contains("export_bins")) c.export_bins = e["export_bins"];
        if (e.contains("out_dir")) c.out_dir = e["out_dir"];
    }
    if (j.contains("hardware")) {
        auto h = j["hardware"];
        if (h.contains("simd_preference")) c.simd_preference = h["simd_preference"];
        if (h.contains("vector_batch_size")) c.vector_batch_size = h["vector_batch_size"];
    }
    return c;
}

// -------------------- CPU feature detection --------------------
struct CPUFeatures {
    bool avx2 = false;
    bool avx512 = false;
    bool pclmul = false;
};

static CPUFeatures detect_cpu_features() {
    CPUFeatures f{};
    unsigned int eax, ebx, ecx, edx;

    // Basic CPUID(1)
    if (!__get_cpuid(1, &eax, &ebx, &ecx, &edx)) return f;
    // bit 28 of ECX: AVX (SSE extensions); but need XSAVE test in OS
    bool has_avx = (ecx & bit_AVX) != 0;

    // Check extended features CPUID(7,0)
    if (!__get_cpuid_count(7, 0, &eax, &ebx, &ecx, &edx)) return f;
    f.avx2 = (ebx & bit_AVX2) != 0;
    // AVX-512 bits in EBX (several bits): check for AVX512F
    const unsigned int AVX512F_BIT = (1u << 16); // EBX bit 16 in CPUID leaf 7
    f.avx512 = (ebx & AVX512F_BIT) != 0;

    // PCLMUL (carry-less multiply)
    f.pclmul = (ecx & bit_PCLMUL) != 0;

    // OS support: very minimal; assume if feature bits present it's OK; for production
    // check XGETBV and OSXSAVE properly. (left as an exercise).
    return f;
}

// -------------------- AVX2 vector popcount (bytes) --------------------
// Compute the popcount (number of 1s) in a byte array quickly using AVX2
// Uses 4-bit nibble lookup table technique and _mm256_sad_epu8 to accumulate.
static inline uint64_t popcount_bytes_avx2(const uint8_t* data, size_t len) {
    if (len == 0) return 0;
    size_t i = 0;
    const __m256i low_mask = _mm256_set1_epi8(0x0F);

    // lookup table: popcount for nibble 0..15 (16 bytes)
    const __m256i lut = _mm256_setr_epi8(
         0,1,1,2,1,2,2,3,1,2,2,3,2,3,3,4,
         0,1,1,2,1,2,2,3,1,2,2,3,2,3,3,4
    );

    uint64_t total = 0;
    const uint8_t* p = data;
    // process 32 bytes per loop
    while (len - i >= 32) {
        __m256i v = _mm256_loadu_si256(reinterpret_cast<const __m256i*>(p + i));
        // low nibble
        __m256i lo = _mm256_and_si256(v, low_mask);
        // high nibble
        __m256i hi = _mm256_and_si256(_mm256_srli_epi16(v, 4), low_mask);
        __m256i plo = _mm256_shuffle_epi8(lut, lo);
        __m256i phi = _mm256_shuffle_epi8(lut, hi);
        __m256i pop8 = _mm256_add_epi8(plo, phi);
        // sum bytes to 4 x 64-bit lanes via sad
        __m256i sad = _mm256_sad_epu8(pop8, _mm256_setzero_si256()); // yields 4 x 64-bit sums
        // extract 4 lanes
        uint64_t lane0 = (uint64_t)_mm256_extract_epi64(sad, 0);
        uint64_t lane1 = (uint64_t)_mm256_extract_epi64(sad, 1);
        uint64_t lane2 = (uint64_t)_mm256_extract_epi64(sad, 2);
        uint64_t lane3 = (uint64_t)_mm256_extract_epi64(sad, 3);
        total += lane0 + lane1 + lane2 + lane3;
        i += 32;
    }
    // tail
    while (i < len) {
        total += __builtin_popcount((unsigned int)p[i]);
        ++i;
    }
    return total;
}

// Generic popcount wrapper: chooses avx2 or scalar fallback
static inline uint64_t array_popcount(const uint8_t* data, size_t len, const CPUFeatures &f) {
    if (f.avx2 && len >= 32) return popcount_bytes_avx2(data, len);
    // scalar fallback
    uint64_t s = 0;
    for (size_t i = 0; i < len; ++i) s += (uint64_t)(data[i] & 1);
    return s;
}

// -------------------- Vectorized window-check example --------------------
//
// Given ones_flags[] (bytes: 0/1), compute circular windows of length B (block_size)
// at a set of start positions, and test whether any window error exceeds max_block_error.
// We call array_popcount to compute ones-in-window quickly for large windows.
//
// Returns: true if candidate passes (no window exceeds max_block_error).
static inline bool sliding_window_check_fast(const uint8_t* ones_flags, int period,
                                             int block_size, const double target,
                                             const CPUFeatures &f, int max_checks = 16384) {
    // build prefix-sum style via chunk popcounts for efficiency
    // we'll evaluate at deterministically chosen starts that cover gcd residues
    int g = std::gcd(period, block_size);
    if (g < 1) g = 1;
    // build starts covering residues
    std::vector<int> starts;
    starts.reserve(std::min(period, max_checks + g));
    for (int r = 0; r < g && (int)starts.size() < max_checks; ++r) starts.push_back(r);
    for (int r = 0; (int)starts.size() < max_checks && r < period; ++r) {
        int val = (r * g) % period;
        starts.push_back(val);
    }
    std::sort(starts.begin(), starts.end());
    starts.erase(std::unique(starts.begin(), starts.end()), starts.end());
    if ((int)starts.size() > max_checks) starts.resize(max_checks);

    // To compute ones in window [s, s+block_size), we'll use popcount on the contiguous
    // region in the circular sense: if wrap occurs, we do two popcounts.
    double max_block_error_seen = 0.0;
    double prev_block_duty = -1.0;

    for (int s : starts) {
        int a = s;
        int b = s + block_size; // exclusive
        uint64_t ones_in_window = 0;
        if (b <= period) {
            ones_in_window = array_popcount(ones_flags + a, block_size, f);
        } else { // wrap
            int len1 = period - a;
            ones_in_window = array_popcount(ones_flags + a, len1, f);
            int len2 = b - period;
            ones_in_window += array_popcount(ones_flags + 0, len2, f);
        }
        double block_duty = (double)ones_in_window / (double)block_size;
        double block_error = std::fabs(block_duty - target);
        if (block_error > max_block_error_seen) max_block_error_seen = block_error;
        if (block_error > 1e-12 + /*max_block_error param passed by caller*/ 0.1) {
            return false; // quick reject (caller should pass their max)
        }
        if (prev_block_duty > -0.5) {
            double delta = std::fabs(block_duty - prev_block_duty);
            // caller checks delta against PWM bin later
        }
        prev_block_duty = block_duty;
    }
    // if we reach here, candidate passed the checked starts (still probabilistic if we limited starts)
    return true;
}
