// lfsr_pwm_threshold_search_improved.cpp
// Improved LFSR PWM threshold search with per-block (default 256-step) error constraint
// Build: g++ -std=c++17 -O3 -march=native -flto -funroll-loops -o lfsr_pwm_threshold_search_improved lfsr_pwm_threshold_search_improved.cpp

#include <bits/stdc++.h>
#include <atomic>
#include <thread>
#include <mutex>
#include <getopt.h>
#include <chrono>
#include <functional>

using u64 = uint64_t;
using u32 = uint32_t;

struct LFSRConfig {
    u64 mask = 0;
    u64 seed = 0;      // Doubles as both starting state AND threshold
    int period = 0;
    int lc = 0;
    double actual_duty = 0.0;
    double error = 0.0;
    double score = 0.0;
};

static inline int popcount_u64(u64 x) { return __builtin_popcountll(x); }

// Berlekamp–Massey over GF(2) to estimate linear complexity
int berlekamp_massey(const std::vector<uint8_t> &bits) {
    int n = (int)bits.size();
    if (n == 0) return 0;
    std::vector<uint8_t> C(n,0), B(n,0);
    C[0]=1; B[0]=1;
    int L=0, m=-1;
    for (int N=0; N<n; ++N) {
        int d=0;
        for (int i=0;i<=L;++i) d ^= (C[i] & bits[N-i]);
        if (d) {
            std::vector<uint8_t> T = C;
            int p = N - m;
            for (int i = 0; i + p < n; ++i) C[i+p] ^= B[i];
            if (L <= N/2) {
                L = N + 1 - L;
                m = N;
                B.swap(T);
            }
        }
    }
    return L;
}

// Next state for Fibonacci-style right shift LFSR
static inline u64 next_state(u64 state, u64 mask, int n) {
    u64 fb = __builtin_parityll(state & mask);
    return (state >> 1) | (fb << (n - 1));
}

// FNV-1a 64-bit hash for a sequence of 0/1 bytes
static uint64_t fnv1a_hash_bits(const std::vector<uint8_t> &bits) {
    const uint64_t FNV_OFFSET = 1469598103934665603ULL;
    const uint64_t FNV_PRIME  = 1099511628211ULL;
    uint64_t h = FNV_OFFSET;
    for (uint8_t b : bits) {
        h ^= (uint64_t)(b & 1);
        h *= FNV_PRIME;
    }
    return h;
}

// Find all cycles for a mask and return vector of cycles (each cycle is vector<u64>)
std::vector<std::vector<u64>> find_all_cycles_raw(u64 mask, int n) {
    const u64 max_state = (1ULL << n);
    std::vector<uint8_t> visited(max_state, 0);
    std::vector<std::vector<u64>> cycles;

    for (u64 s = 1; s < max_state; ++s) {
        if (visited[s]) continue;

        // Walk until we hit a visited state, collecting chain
        std::vector<u64> chain;
        chain.reserve(1024);
        u64 curr = s;
        while (!visited[curr]) {
            visited[curr] = 1;
            chain.push_back(curr);
            curr = next_state(curr, mask, n);
        }

        // Find where the detected loop starts in chain (if at all)
        auto it = std::find(chain.begin(), chain.end(), curr);
        if (it == chain.end()) {
            // Curr is part of a previously-seen cycle; nothing new to add
            continue;
        }

        size_t cycle_start_idx = it - chain.begin();
        size_t cycle_len = chain.size() - cycle_start_idx;
        if (cycle_len <= 1) continue; // skip trivial 1-state cycles

        std::vector<u64> cycle;
        cycle.reserve(cycle_len);
        for (size_t k = 0; k < cycle_len; ++k) {
            cycle.push_back(chain[cycle_start_idx + k]);
        }
        cycles.push_back(std::move(cycle));
    }

    return cycles;
}

struct Options {
    int n = 16;
    int resolution = 256;
    int threads = 0;
    u64 max_masks = 0;  // 0 = scan all
    std::string output = "lfsr_pwm_lut.h";
    double lc_weight = 0.01;
    double period_weight = 0.001;
    unsigned seed = 0;
    int min_taps = 2;  // Minimum number of taps (bits set in mask)
    double min_period_ratio = 0.0; // skip cycles shorter than ratio * (1<<n)
    int min_sample_bits = 0; // computed from n
    int block_size = 256; // block size for windowed error calculation
    double max_block_error = 1.0; // allowed max error per block (1.0 = effectively disabled)
};

Options parse_args(int argc, char **argv) {
    Options opt;
    static struct option long_options[] = {
        {"n", required_argument, nullptr, 'n'},
        {"res", required_argument, nullptr, 'r'},
        {"threads", required_argument, nullptr, 't'},
        {"max-masks", required_argument, nullptr, 'm'},
        {"output", required_argument, nullptr, 'o'},
        {"lc-weight", required_argument, nullptr, 'l'},
        {"period-weight", required_argument, nullptr, 'p'},
        {"seed", required_argument, nullptr, 's'},
        {"min-taps", required_argument, nullptr, 'T'},
        {"min-period-ratio", required_argument, nullptr, 'q'},
        {"block-size", required_argument, nullptr, 'b'},
        {"max-block-error", required_argument, nullptr, 'e'},
        {nullptr, 0, nullptr, 0}
    };
    
    int c;
    while ((c = getopt_long(argc, argv, "n:r:t:m:o:l:p:s:T:q:b:e:", long_options, nullptr)) != -1) {
        switch (c) {
            case 'n': opt.n = std::atoi(optarg); break;
            case 'r': opt.resolution = std::atoi(optarg); break;
            case 't': opt.threads = std::atoi(optarg); break;
            case 'm': opt.max_masks = std::stoull(optarg); break;
            case 'o': opt.output = optarg; break;
            case 'l': opt.lc_weight = std::stod(optarg); break;
            case 'p': opt.period_weight = std::stod(optarg); break;
            case 's': opt.seed = (unsigned)std::stoul(optarg); break;
            case 'T': opt.min_taps = std::atoi(optarg); break;
            case 'q': opt.min_period_ratio = std::stod(optarg); break;
            case 'b': opt.block_size = std::atoi(optarg); break;
            case 'e': opt.max_block_error = std::stod(optarg); break;
        }
    }
    
    if (opt.seed == 0) {
        opt.seed = (unsigned)std::chrono::system_clock::now().time_since_epoch().count();
    }
    if (opt.threads <= 0) {
        opt.threads = std::max(1u, std::thread::hardware_concurrency());
    }

    opt.min_sample_bits = 0; // filled later (needs n)
    
    return opt;
}

// Evaluate a single mask: for each cycle, for each target compute the best threshold candidate
// candidate_counts is used to increment the number of candidates that pass the configured block constraints per target
void evaluate_mask_for_all_levels(u64 mask, int n, const std::vector<double> &targets,
                                   std::vector<LFSRConfig> &best_configs,
                                   std::vector<uint32_t> &candidate_counts,
                                   std::mutex &merge_mutex,
                                   double lc_weight, double period_weight,
                                   double min_period_ratio,
                                   int min_sample_bits, int max_sample_bits,
                                   int block_size, double max_block_error, int resolution) {
    // Find cycles
    std::vector<std::vector<u64>> cycles = find_all_cycles_raw(mask, n);
    if (cycles.empty()) return;

    // Thread-local LC cache
    static thread_local std::unordered_map<uint64_t,int> lc_cache;

    double max_bonus = lc_weight + period_weight; // safe upper bound for pruning

    // Per-target local best (to merge later)
    std::vector<LFSRConfig> local_configs(targets.size());
    for (auto &cfg : local_configs) cfg.score = std::numeric_limits<double>::infinity();

    // Per-target local candidate counts to aggregate at merge time
    std::vector<uint32_t> local_counts(targets.size(), 0);

    const u64 max_state = (1ULL << n);

    for (const auto &cycle : cycles) {
        int period = (int)cycle.size();

        // Optionally skip very tiny cycles that can't match fine-grain PWM levels
        if (min_period_ratio > 0.0) {
            double ratio = (double)period / (double)(max_state);
            if (ratio < min_period_ratio) continue;
        }

        // Build a sorted list of numeric state values (ascending)
        std::vector<u64> sorted = cycle;
        std::sort(sorted.begin(), sorted.end());

        // Map state -> index in cycle (to know where to start sampling)
        std::unordered_map<u64,int> state_to_idx;
        state_to_idx.reserve(cycle.size()*2);
        for (int idx = 0; idx < period; ++idx) state_to_idx[cycle[idx]] = idx;

        // For each target, determine the nearest achievable k/period and evaluate
        for (size_t ti = 0; ti < targets.size(); ++ti) {
            double target = targets[ti];

            // integer number of ones we'd like
            int k_ideal = (int)std::round(target * (double)period);
            // clamp to [0, period-1] because using a state as threshold can give at most period-1 ones
            if (k_ideal < 0) k_ideal = 0;
            if (k_ideal > period-1) k_ideal = period-1;

            // evaluate up to three candidates: k_ideal and maybe k_ideal +/- 1 for tie/rounding robustness
            int k_candidates[3];
            int k_count = 0;
            k_candidates[k_count++] = k_ideal;
            if (k_ideal > 0) k_candidates[k_count++] = k_ideal - 1;
            if (k_ideal < period-1) k_candidates[k_count++] = k_ideal + 1;

            for (int kc = 0; kc < k_count; ++kc) {
                int k = k_candidates[kc];

                // j index in sorted that yields exactly k ones: j = period - k - 1
                int j = period - k - 1;
                if (j < 0) continue;
                if (j >= period) continue;
                u64 threshold_val = sorted[j];

                // find index in cycle where threshold_val occurs
                auto itidx = state_to_idx.find(threshold_val);
                if (itidx == state_to_idx.end()) continue;
                int start_idx = itidx->second;

                // compute actual duty (should equal k/period)
                double duty = (double)k / (double)period;
                double error = fabs(duty - target);

                // Quick pruning using optimistic bonuses
                double min_possible_score = error - max_bonus;
                if (min_possible_score >= local_configs[ti].score) continue;

                // Determine sampling length such that we partition the run into block-sized windows covering the entire cycle.
                // Number of windows needed to cover period (ceiling)
                int num_windows = (period + block_size - 1) / block_size;
                if (num_windows < 1) num_windows = 1;
                int sample_len = num_windows * block_size;
                // Ensure at least min_sample_bits for stable LC calculation
                sample_len = std::max(sample_len, min_sample_bits);
                if (sample_len > max_sample_bits) sample_len = max_sample_bits;

                // Build sampled PWM bits starting at start_idx (candidate starting state equals threshold)
                std::vector<uint8_t> sample_bits;
                sample_bits.reserve(sample_len);
                for (int s = 0; s < sample_len; ++s) {
                    u64 st = cycle[(start_idx + s) % period];
                    sample_bits.push_back((uint8_t)(st > threshold_val ? 1 : 0));
                }

                // BLOCK-BASED EVALUATION:
                // Partition sample_bits into block_size windows (the last window may wrap around cycle but we've repeated states by modulo above).
                int actual_block_size = block_size;
                int actual_num_windows = sample_len / actual_block_size;
                if (actual_num_windows < 1) actual_num_windows = 1;

                double max_block_error_seen = 0.0;
                double max_block_duty_delta = 0.0;
                double prev_block_duty = -1.0;

                for (int w = 0; w < actual_num_windows; ++w) {
                    int base = w * actual_block_size;
                    int ones = 0;
                    for (int q = 0; q < actual_block_size; ++q) {
                        ones += sample_bits[base + q];
                    }
                    double block_duty = (double)ones / (double)actual_block_size;
                    double block_error = fabs(block_duty - target);
                    if (block_error > max_block_error_seen) max_block_error_seen = block_error;
                    if (prev_block_duty >= -0.5) {
                        double delta = fabs(block_duty - prev_block_duty);
                        if (delta > max_block_duty_delta) max_block_duty_delta = delta;
                    }
                    prev_block_duty = block_duty;
                }

                // PWM quantization bin (targets are produced as i/(resolution-1))
                double pwm_bin = 1.0 / (double)(std::max(2, resolution - 1));

                // Enforce the configured constraints:
                //  - maximum error per block must be <= max_block_error
                //  - changes in block duty between adjacent windows must not exceed the PWM quantization bin
                if (max_block_error_seen > max_block_error) {
                    continue; // reject candidate
                }
                if (max_block_duty_delta > pwm_bin) {
                    continue; // reject candidate
                }

                // Candidate passed block-based constraints -> increment local candidate count
                local_counts[ti]++;

                // hash & LC caching
                uint64_t h = fnv1a_hash_bits(sample_bits);
                int cand_lc;
                auto it = lc_cache.find(h);
                if (it != lc_cache.end()) {
                    cand_lc = it->second;
                } else {
                    cand_lc = berlekamp_massey(sample_bits);
                    lc_cache.emplace(h, cand_lc);
                }

                // Compose score using LC and period bonuses as before; record candidate's "error" as max_block_error_seen
                double lc_term = (double)cand_lc / (double)n;
                double period_term = (period > 0) ? (std::log2((double)period) / (double)n) : 0.0;
                double lc_bonus = lc_weight * lc_term;
                double period_bonus = period_weight * period_term;
                double score = max_block_error_seen - lc_bonus - period_bonus; // use block-based error in score

                if (score < local_configs[ti].score) {
                    local_configs[ti].mask = mask;
                    local_configs[ti].seed = threshold_val;
                    local_configs[ti].period = period;
                    local_configs[ti].lc = cand_lc;
                    local_configs[ti].actual_duty = duty;
                    local_configs[ti].error = max_block_error_seen;
                    local_configs[ti].score = score;
                }
            } // kc
        } // targets
    } // cycles

    // Merge into global
    std::lock_guard<std::mutex> lock(merge_mutex);
    for (size_t i = 0; i < targets.size(); ++i) {
        if (local_configs[i].score < best_configs[i].score) {
            best_configs[i] = local_configs[i];
        }
        candidate_counts[i] += local_counts[i];
    }
}

// Write Arduino LUT with just mask and seed (seed doubles as threshold)
void write_arduino_lut(const std::vector<LFSRConfig> &configs, int n, const std::string &filename) {
    std::ofstream f(filename);
    if (!f) {
        std::cerr << "ERROR: cannot open " << filename << " for writing\n";
        return;
    }
    
    f << "// Arduino LFSR PWM LUT: mask, seed (" << n << "-bit)\n";
    f << "// PWM output: (state > seed) ? HIGH : LOW\n";
    f << "// Seed doubles as both starting state AND threshold\n";
    f << "#include <stdint.h>\n";
    f << "#if defined(__AVR__)\n#include <avr/pgmspace.h>\n";
    f << "#else\n#define PROGMEM\n";
    f << "#define pgm_read_word(addr) (*(addr))\n";
    f << "#define pgm_read_dword(addr) (*(addr))\n";
    f << "#endif\n\n";
    
    if (n <= 16) {
        f << "const struct { uint16_t mask; uint16_t seed; } lfsr_pwm_lut[] PROGMEM = {\n";
        for (size_t i = 0; i < configs.size(); ++i) {
            const LFSRConfig &c = configs[i];
            f << "  { 0x" << std::hex << std::setw(4) << std::setfill('0') << (uint16_t)c.mask
              << "u, 0x" << std::setw(4) << (uint16_t)c.seed << "u }";
            f << std::dec << ", // " << std::setw(3) << i 
              << ": duty=" << std::fixed << std::setprecision(4) << c.actual_duty
              << " err=" << std::scientific << std::setprecision(2) << c.error
              << " P=" << std::dec << c.period << " LC=" << c.lc << "\n";
        }
        f << "};\n\n";
        
        // Add example Arduino usage code
        f << "/* Example Arduino usage:\n\n";
        f << "uint16_t lfsr_state;\n";
        f << "uint16_t lfsr_mask;\n";
        f << "uint16_t lfsr_threshold;\n\n";
        f << "void setup_pwm(uint8_t level) {\n";
        f << "  lfsr_mask = pgm_read_word(&lfsr_pwm_lut[level].mask);\n";
        f << "  lfsr_threshold = pgm_read_word(&lfsr_pwm_lut[level].seed);\n";
        f << "  lfsr_state = lfsr_threshold;  // Start at threshold value\n";
        f << "}\n\n";
        f << "void pwm_update() {\n";
        f << "  // Output based on comparison\n";
        f << "  digitalWrite(PWM_PIN, (lfsr_state > lfsr_threshold) ? HIGH : LOW);\n";
        f << "  \n";
        f << "  // Advance LFSR state\n";
        f << "  uint16_t feedback = __builtin_parity(lfsr_state & lfsr_mask) & 1;\n";
        f << "  lfsr_state = (lfsr_state >> 1) | (feedback << 15);\n";
        f << "}\n";
        f << "*/\n";
    } else {
        f << "const struct { uint32_t mask; uint32_t seed; } lfsr_pwm_lut[] PROGMEM = {\n";
        for (size_t i = 0; i < configs.size(); ++i) {
            const LFSRConfig &c = configs[i];
            f << "  { 0x" << std::hex << std::setw(8) << std::setfill('0') << (uint32_t)c.mask
              << "u, 0x" << std::setw(8) << (uint32_t)c.seed << "u }";
            f << std::dec << ", // " << std::setw(3) << i
              << ": duty=" << std::fixed << std::setprecision(4) << c.actual_duty
              << " err=" << std::scientific << std::setprecision(2) << c.error
              << " P=" << std::dec << c.period << " LC=" << c.lc << "\n";
        }
        f << "};\n\n";
        
        // Add example Arduino usage code
        f << "/* Example Arduino usage:\n\n";
        f << "uint32_t lfsr_state;\n";
        f << "uint32_t lfsr_mask;\n";
        f << "uint32_t lfsr_threshold;\n\n";
        f << "void setup_pwm(uint8_t level) {\n";
        f << "  lfsr_mask = pgm_read_dword(&lfsr_pwm_lut[level].mask);\n";
        f << "  lfsr_threshold = pgm_read_dword(&lfsr_pwm_lut[level].seed);\n";
        f << "  lfsr_state = lfsr_threshold;\n";
        f << "}\n\n";
        f << "void pwm_update() {\n";
        f << "  digitalWrite(PWM_PIN, (lfsr_state > lfsr_threshold) ? HIGH : LOW);\n";
        f << "  uint32_t feedback = __builtin_parity(lfsr_state & lfsr_mask) & 1;\n";
        f << "  lfsr_state = (lfsr_state >> 1) | (feedback << " << (n-1) << ");\n";
        f << "}\n";
        f << "*/\n";
    }
    
    f.close();
}

int main(int argc, char **argv) {
    Options opt = parse_args(argc, argv);
    if (opt.n < 8 || opt.n > 20) {
        std::cerr << "n should be between 8 and 20 (practical limits)\n";
        return 1;
    }
    opt.min_sample_bits = std::max(8 * opt.n, 512);

    std::cerr << "LFSR PWM Threshold Search (improved, block-based constraint)\n";
    std::cerr << "n=" << opt.n << " bits, resolution=" << opt.resolution
              << ", threads=" << opt.threads << "\n";
    std::cerr << "LC weight=" << opt.lc_weight << ", period weight=" << opt.period_weight << "\n";
    std::cerr << "Min taps=" << opt.min_taps << ", min_period_ratio=" << opt.min_period_ratio << "\n";
    std::cerr << "Block-size=" << opt.block_size << " steps, max-block-error=" << opt.max_block_error << "\n";

    // Generate target duty cycles
    std::vector<double> targets(opt.resolution);
    for (int i = 0; i < opt.resolution; ++i) {
        targets[i] = (double)i / (double)(opt.resolution - 1);
    }
    
    // Generate mask list
    u64 max_mask = (1ULL << opt.n);
    u64 scan_limit = (opt.max_masks == 0) ? max_mask : std::min(opt.max_masks, max_mask);
    
    std::vector<u64> masks;
    masks.reserve(scan_limit - 1);
    for (u64 m = 1; m < scan_limit; ++m) {
        if (popcount_u64(m) >= opt.min_taps) masks.push_back(m);
    }
    
    // Shuffle for load balancing
    std::mt19937_64 rng(opt.seed);
    std::shuffle(masks.begin(), masks.end(), rng);
    
    std::cerr << "Scanning " << masks.size() << " masks (filtered by min_taps=" << opt.min_taps << ")...\n";
    
    // Initialize best configs
    std::vector<LFSRConfig> best_configs(opt.resolution);
    for (auto &cfg : best_configs) cfg.score = std::numeric_limits<double>::infinity();

    // Candidate counts per target (how many threshold candidates passed the block-based constraints)
    std::vector<uint32_t> candidate_counts(opt.resolution, 0);

    std::mutex merge_mutex;
    std::atomic<size_t> masks_processed{0};
    
    auto worker = [&](size_t start_idx, size_t end_idx) {
        for (size_t i = start_idx; i < end_idx; ++i) {
            evaluate_mask_for_all_levels(masks[i], opt.n, targets, best_configs,
                                         candidate_counts,
                                         merge_mutex, opt.lc_weight, opt.period_weight,
                                         opt.min_period_ratio,
                                         opt.min_sample_bits, 16384, // max_sample_bits
                                         opt.block_size, opt.max_block_error, opt.resolution);
            size_t processed = masks_processed.fetch_add(1) + 1;
            if (processed % 5000 == 0) {
                std::cerr << "Processed " << processed << " / " << masks.size() << " masks\r" << std::flush;
            }
        }
    };
    
    size_t masks_per_thread = (masks.size() + opt.threads - 1) / opt.threads;
    std::vector<std::thread> threads;
    for (int t = 0; t < opt.threads; ++t) {
        size_t start = t * masks_per_thread;
        size_t end = std::min(masks.size(), start + masks_per_thread);
        if (start < end) threads.emplace_back(worker, start, end);
    }
    for (auto &th : threads) th.join();

    std::cerr << "\nProcessed " << masks.size() << " masks\n";

    // Statistics
    double avg_error = 0.0;
    double max_error = 0.0;
    int unmatched = 0;
    for (const auto &cfg : best_configs) {
        if (cfg.score == std::numeric_limits<double>::infinity()) {
            unmatched++;
        } else {
            avg_error += cfg.error;
            max_error = std::max(max_error, cfg.error);
        }
    }
    if (opt.resolution > unmatched) avg_error /= (opt.resolution - unmatched);

    std::cerr << "Matched levels: " << (opt.resolution - unmatched) << " / " << opt.resolution << "\n";
    std::cerr << "Average block-based error: " << std::scientific << avg_error << "\n";
    std::cerr << "Maximum block-based error: " << max_error << "\n";

    // Print candidate counts per PWM step
    std::cerr << "\nCandidates passing block constraints per PWM level (index : duty -> count):\n";
    for (int i = 0; i < opt.resolution; ++i) {
        std::cerr << std::setw(3) << i << " : " << std::fixed << std::setprecision(4) << targets[i]
                  << " -> " << candidate_counts[i] << "\n";
    }

    // Duty coverage quick report
    std::cerr << "\nDuty cycle coverage:\n";
    auto count_range = [&](double lo, double hi) {
        int c=0;
        for (int i=0;i<opt.resolution;++i) {
            if (targets[i] >= lo && targets[i] <= hi && best_configs[i].score != std::numeric_limits<double>::infinity()) c++;
        }
        return c;
    };
    std::cerr << "  0.00-0.10: " << count_range(0.0,0.10) << " levels covered\n";
    std::cerr << "  0.90-1.00: " << count_range(0.90,1.00) << " levels covered\n";

    write_arduino_lut(best_configs, opt.n, opt.output);
    std::cerr << "\nWritten Arduino LUT to " << opt.output << "\n";

    return 0;
}
