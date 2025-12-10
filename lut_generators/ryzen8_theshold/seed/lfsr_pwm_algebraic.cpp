// lfsr_pwm_threshold_search.cpp
// LFSR PWM using state-threshold comparison where seed doubles as threshold.
// Approach: output = (state > seed), giving compact {mask, seed} LUT format
//
// Build: g++ -std=c++17 -O3 -march=native -flto -funroll-loops -o lfsr_pwm_algebraic lfsr_pwm_algebraic.cpp

#include <bits/stdc++.h>
#include <atomic>
#include <thread>
#include <mutex>
#include <getopt.h>
#include <chrono>

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

// Calculate duty cycle when using start_state as both starting point AND threshold
double calculate_duty_cycle_from_state(u64 mask, u64 start_state, int period, int n) {
    if (period <= 0) return 0.0;

    int ones = 0;
    u64 state = start_state;
    u64 threshold = start_state;  // Use starting state as threshold

    for (int i = 0; i < period; ++i) {
        if (state > threshold) ones++;
        state = next_state(state, mask, n);
    }

    return (double)ones / (double)period;
}

// Find all cycles for a mask and return candidates with their duty cycles
struct CycleCandidate {
    u64 start_state;
    int period;
    double duty_cycle;
    std::vector<uint8_t> sample_bits;
};

std::vector<CycleCandidate> find_all_cycles(u64 mask, int n) {
    const u64 max_state = (1ULL << n);
    std::vector<uint8_t> visited(max_state, 0);
    std::vector<CycleCandidate> candidates;

    for (u64 s = 1; s < max_state; ++s) {
        if (visited[s]) continue;

        // Trace this cycle
        std::vector<u64> cycle;
        cycle.reserve(1024);
        u64 curr = s;

        while (!visited[curr]) {
            visited[curr] = 1;
            cycle.push_back(curr);
            curr = next_state(curr, mask, n);
        }

        // Check if we have a complete cycle
        if (curr == s && cycle.size() > 2) {
            CycleCandidate cand;
            cand.start_state = s;
            cand.period = (int)cycle.size();

            // Calculate duty cycle using s as threshold
            cand.duty_cycle = calculate_duty_cycle_from_state(mask, s, cand.period, n);

            // Sample bits for LC estimation
            int sample_len = std::min(256, (int)cycle.size());
            cand.sample_bits.reserve(sample_len);
            for (int i = 0; i < sample_len; ++i) {
                cand.sample_bits.push_back((uint8_t)(cycle[i] & 1ULL));
            }

            candidates.push_back(std::move(cand));
        }
    }

    return candidates;
}

// Evaluate a mask and find best seed (which acts as both start state and threshold) for each PWM level
void evaluate_mask_for_all_levels(u64 mask, int n, const std::vector<double> &targets,
                                   std::vector<LFSRConfig> &best_configs,
                                   std::mutex &merge_mutex,
                                   double lc_weight, double period_weight) {

    // Find all cycles for this mask
    std::vector<CycleCandidate> candidates = find_all_cycles(mask, n);

    if (candidates.empty()) return;

    // Calculate LC once (use longest cycle's bits)
    int best_period = 0;
    const CycleCandidate *longest = nullptr;
    for (const auto &cand : candidates) {
        if (cand.period > best_period) {
            best_period = cand.period;
            longest = &cand;
        }
    }

    int lc = longest ? berlekamp_massey(longest->sample_bits) : 0;

    // For each PWM level, find the candidate (seed/threshold) with closest duty cycle
    std::vector<LFSRConfig> local_configs(targets.size());
    for (auto &cfg : local_configs) {
        cfg.score = std::numeric_limits<double>::infinity();
    }

    for (const auto &cand : candidates) {
        for (size_t i = 0; i < targets.size(); ++i) {
            double target = targets[i];
            double error = fabs(cand.duty_cycle - target);

            // Score: balance error with quality metrics
            double lc_bonus = lc_weight * lc / (double)n;
            double period_bonus = period_weight * log2(cand.period) / (double)n;
            double score = error - lc_bonus - period_bonus;

            if (score < local_configs[i].score) {
                local_configs[i].mask = mask;
                local_configs[i].seed = cand.start_state;  // Seed doubles as threshold!
                local_configs[i].period = cand.period;
                local_configs[i].lc = lc;
                local_configs[i].actual_duty = cand.duty_cycle;
                local_configs[i].error = error;
                local_configs[i].score = score;
            }
        }
    }

    // Merge with global best
    std::lock_guard<std::mutex> lock(merge_mutex);
    for (size_t i = 0; i < targets.size(); ++i) {
        if (local_configs[i].score < best_configs[i].score) {
            best_configs[i] = local_configs[i];
        }
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
    f << "#define pgm_read_dword(addr) (*(addr))\n#endif\n\n";

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
    }

    f << "};\n\n";

    // Add example Arduino usage code
    f << "/* Example Arduino usage:\n\n";
    if (n <= 16) {
        f << "uint16_t lfsr_state;\n";
        f << "uint16_t lfsr_mask;\n";
        f << "uint16_t lfsr_threshold;\n\n";
        f << "// Portable parity function for AVR\n";
        f << "inline uint8_t parity16(uint16_t x) {\n";
        f << "  x ^= x >> 8;\n";
        f << "  x ^= x >> 4;\n";
        f << "  x ^= x >> 2;\n";
        f << "  x ^= x >> 1;\n";
        f << "  return x & 1;\n";
        f << "}\n\n";
        f << "void setup_pwm(uint8_t level) {\n";
        f << "  lfsr_mask = pgm_read_word(&lfsr_pwm_lut[level].mask);\n";
        f << "  lfsr_threshold = pgm_read_word(&lfsr_pwm_lut[level].seed);\n";
        f << "  lfsr_state = lfsr_threshold;  // Start at threshold value\n";
        f << "}\n\n";
        f << "void pwm_update() {\n";
        f << "  // Output based on comparison (Fibonacci LFSR)\n";
        f << "  digitalWrite(PWM_PIN, (lfsr_state > lfsr_threshold) ? HIGH : LOW);\n";
        f << "  \n";
        f << "  // Advance LFSR state: feedback = parity(state & mask)\n";
        f << "  uint16_t feedback = parity16(lfsr_state & lfsr_mask);\n";
        f << "  lfsr_state = (lfsr_state >> 1) | (feedback << 15);\n";
        f << "}\n";
    } else {
        f << "uint32_t lfsr_state;\n";
        f << "uint32_t lfsr_mask;\n";
        f << "uint32_t lfsr_threshold;\n\n";
        f << "// Portable parity function\n";
        f << "inline uint8_t parity32(uint32_t x) {\n";
        f << "  x ^= x >> 16;\n";
        f << "  x ^= x >> 8;\n";
        f << "  x ^= x >> 4;\n";
        f << "  x ^= x >> 2;\n";
        f << "  x ^= x >> 1;\n";
        f << "  return x & 1;\n";
        f << "}\n\n";
        f << "void setup_pwm(uint8_t level) {\n";
        f << "  lfsr_mask = pgm_read_dword(&lfsr_pwm_lut[level].mask);\n";
        f << "  lfsr_threshold = pgm_read_dword(&lfsr_pwm_lut[level].seed);\n";
        f << "  lfsr_state = lfsr_threshold;\n";
        f << "}\n\n";
        f << "void pwm_update() {\n";
        f << "  // Output based on comparison (Fibonacci LFSR)\n";
        f << "  digitalWrite(PWM_PIN, (lfsr_state > lfsr_threshold) ? HIGH : LOW);\n";
        f << "  \n";
        f << "  // Advance LFSR state: feedback = parity(state & mask)\n";
        f << "  uint32_t feedback = parity32(lfsr_state & lfsr_mask);\n";
        f << "  lfsr_state = (lfsr_state >> 1) | (feedback << " << (n-1) << ");\n";
        f << "}\n";
    }
    f << "*/\n";

    f.close();
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
        {nullptr, 0, nullptr, 0}
    };

    int c;
    while ((c = getopt_long(argc, argv, "n:r:t:m:o:l:p:s:T:", long_options, nullptr)) != -1) {
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
        }
    }

    if (opt.seed == 0) {
        opt.seed = (unsigned)std::chrono::system_clock::now().time_since_epoch().count();
    }
    if (opt.threads <= 0) {
        opt.threads = std::max(1u, std::thread::hardware_concurrency());
    }

    return opt;
}

int main(int argc, char **argv) {
    Options opt = parse_args(argc, argv);

    if (opt.n < 8 || opt.n > 20) {
        std::cerr << "n should be between 8 and 20 (practical limits)\n";
        return 1;
    }

    std::cerr << "LFSR PWM Threshold Search (seed doubles as threshold)\n";
    std::cerr << "n=" << opt.n << " bits, resolution=" << opt.resolution
              << ", threads=" << opt.threads << "\n";
    std::cerr << "LC weight=" << opt.lc_weight << ", period weight=" << opt.period_weight << "\n";
    std::cerr << "Min taps=" << opt.min_taps << "\n";

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
        // Filter by minimum number of taps
        if (popcount_u64(m) >= opt.min_taps) {
            masks.push_back(m);
        }
    }

    // Shuffle for better load balancing
    std::mt19937_64 rng(opt.seed);
    std::shuffle(masks.begin(), masks.end(), rng);

    std::cerr << "Scanning " << masks.size() << " masks (filtered by min_taps=" << opt.min_taps << ")...\n";

    // Initialize best configs with worst possible scores
    std::vector<LFSRConfig> best_configs(opt.resolution);
    for (auto &cfg : best_configs) {
        cfg.score = std::numeric_limits<double>::infinity();
    }

    std::mutex merge_mutex;
    std::atomic<size_t> masks_processed{0};

    auto worker = [&](size_t start_idx, size_t end_idx) {
        for (size_t i = start_idx; i < end_idx; ++i) {
            evaluate_mask_for_all_levels(masks[i], opt.n, targets, best_configs,
                                        merge_mutex, opt.lc_weight, opt.period_weight);

            size_t processed = masks_processed.fetch_add(1) + 1;
            if (processed % 5000 == 0) {
                std::cerr << "Processed " << processed << " / " << masks.size() << " masks\r" << std::flush;
            }
        }
    };

    // Launch threads
    size_t masks_per_thread = (masks.size() + opt.threads - 1) / opt.threads;
    std::vector<std::thread> threads;

    for (int t = 0; t < opt.threads; ++t) {
        size_t start = t * masks_per_thread;
        size_t end = std::min(masks.size(), start + masks_per_thread);
        if (start < end) {
            threads.emplace_back(worker, start, end);
        }
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

    if (opt.resolution > unmatched) {
        avg_error /= (opt.resolution - unmatched);
    }

    std::cerr << "Matched levels: " << (opt.resolution - unmatched) << " / " << opt.resolution << "\n";
    std::cerr << "Average error: " << std::scientific << avg_error << "\n";
    std::cerr << "Maximum error: " << max_error << "\n";

    // Show duty cycle distribution
    std::cerr << "\nDuty cycle coverage:\n";
    std::cerr << "  0.00-0.10: ";
    int count = 0;
    for (int i = 0; i < opt.resolution; ++i) {
        if (targets[i] <= 0.10 && best_configs[i].score != std::numeric_limits<double>::infinity()) count++;
    }
    std::cerr << count << " levels covered\n";

    std::cerr << "  0.90-1.00: ";
    count = 0;
    for (int i = 0; i < opt.resolution; ++i) {
        if (targets[i] >= 0.90 && best_configs[i].score != std::numeric_limits<double>::infinity()) count++;
    }
    std::cerr << count << " levels covered\n";

    // Write output
    write_arduino_lut(best_configs, opt.n, opt.output);
    std::cerr << "\nWritten Arduino LUT to " << opt.output << "\n";

    return 0;
}
