// lfsr_pwm_threshold_search_improved_export.cpp
// Improved LFSR PWM threshold search with circular sliding-window (wrap-around) error constraint
// and export of valid candidates into binary visualization bins.
// Changes (summary):
//  - Reduce aliasing / phase-bias by ensuring window start coverage across residues modulo gcd(period, block_size).
//  - Use small numeric tolerances (epsilon) for floating comparisons.
//  - Cap LCM-based expansions and avoid exploding evaluation lengths.
//  - Evaluate linear-complexity (Berlekamp–Massey) across a few phase offsets and use the median result to reduce phase bias.
//  - Make sampling/stride selection deterministic and ensure we always cover distinct alignments relevant to wrap-around behavior.
//  - Minor defensive fixes (includes, caps) and clearer comments.
// Build: g++ -std=c++17 -O3 -march=native -flto -funroll-loops -pthread -o lfsr_pwm_threshold_search_improved_export lfsr_pwm_threshold_search_improved_export.cpp

#include <bits/stdc++.h>
#include <atomic>
#include <thread>
#include <mutex>
#include <getopt.h>
#include <chrono>
#include <functional>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <numeric> // for gcd/lcm

using u64 = uint64_t;
using u32 = uint32_t;

/**
 * @brief Configuration struct for a found LFSR PWM candidate.
 */
struct LFSRConfig {
    u64 mask = 0;
    u64 seed = 0;      // Doubles as both starting state AND threshold (kept for LUT compatibility)
    int period = 0;
    int lc = 0;
    double actual_duty = 0.0;
    double error = 0.0;
    double score = 0.0;
};

/**
 * @brief Wrapper for the built-in popcount for 64-bit integers.
 */
static inline int popcount_u64(u64 x) {
    return __builtin_popcountll(x);
}

// --- Berlekamp–Massey Algorithm Context ---
// The Berlekamp–Massey algorithm is used to determine the length of the shortest LFSR
// that can generate a given binary sequence. This length is the linear complexity (LC).
// A higher LC (closer to the LFSR's bit-width 'n') suggests better statistical properties for the PWM.


/**
 * @brief Berlekamp–Massey over GF(2) to estimate linear complexity.
 * @param bits The binary sequence (0/1) as a vector of uint8_t.
 * @return The linear complexity L.
 */
int berlekamp_massey(const std::vector<uint8_t> &bits) {
    int n = (int)bits.size();
    if (n == 0) return 0;

    std::vector<uint8_t> C(n, 0), B(n, 0);
    C[0] = 1;
    B[0] = 1;
    int L = 0, m = -1;

    for (int N = 0; N < n; ++N) {
        int d = 0;
        // Calculate discrepancy d = bits[N] ^ sum(C[i] * bits[N-i])
        for (int i = 0; i <= L; ++i) {
            d ^= (C[i] & bits[N - i]);
        }

        if (d) {
            std::vector<uint8_t> T = C;
            int p = N - m;
            // Update connection polynomial C = C ^ (B * x^p)
            for (int i = 0; i + p < n; ++i) {
                C[i + p] ^= B[i];
            }

            if (L <= N / 2) {
                // Update B and L
                L = N + 1 - L;
                m = N;
                B.swap(T);
            }
        }
    }
    return L;
}

// --- LFSR Architecture Context ---
// This code uses a Fibonacci-style right shift LFSR. The feedback bit is the XOR
// of the tapped bits (defined by the mask) and is shifted into the most significant bit (MSB).


/**
 * @brief Next state for Fibonacci-style right shift LFSR.
 * @param state The current LFSR state.
 * @param mask The tap mask.
 * @param n The bit-width of the LFSR.
 * @return The next state.
 */
static inline u64 next_state(u64 state, u64 mask, int n) {
    u64 fb = __builtin_parityll(state & mask);
    return (state >> 1) | (fb << (n - 1));
}

/**
 * @brief FNV-1a 64-bit hash for a sequence of 0/1 bytes.
 * @param bits The binary sequence.
 * @return The 64-bit FNV-1a hash.
 */
static uint64_t fnv1a_hash_bits(const std::vector<uint8_t> &bits) {
    const uint64_t FNV_OFFSET = 1469598103934665603ULL;
    const uint64_t FNV_PRIME = 1099511628211ULL;
    uint64_t h = FNV_OFFSET;
    for (uint8_t b : bits) {
        h ^= (uint64_t)(b & 1);
        h *= FNV_PRIME;
    }
    return h;
}

/**
 * @brief Find all cycles for a mask (excluding the trivial 0-state cycle).
 * @param mask The LFSR tap mask.
 * @param n The bit-width.
 * @return A vector of cycles, where each cycle is a vector of u64 states.
 */
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

/**
 * @brief Command line options structure.
 */
struct Options {
    int n = 16;
    int resolution = 256;
    int threads = 0;
    u64 max_masks = 0;  // 0 = scan all
    std::string output = "lfsr_pwm_lut.h";
    double lc_weight = 0.01;
    double period_weight = 0.001;
    unsigned seed = 0;
    int min_taps = 1;  // Minimum number of taps (bits set in mask)
    double min_period_ratio = 0.0; // skip cycles shorter than ratio * (1<<n)
    int min_sample_bits = 0; // computed from n
    int block_size = 256; // window size for error check
    double max_block_error = 0.1; // allowed max error per window (1.0 = effectively off)
    std::string out_dir = "8bit_set";
    bool export_bins = true;
};

/**
 * @brief Parses command line arguments.
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return Populated Options struct.
 */
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
        {"out-dir", required_argument, nullptr, 'd'},
        {"no-export-bins", no_argument, nullptr, 1},
        {nullptr, 0, nullptr, 0}
    };

    int c;
    while ((c = getopt_long(argc, argv, "n:r:t:m:o:l:p:s:T:q:b:e:d:", long_options, nullptr)) != -1) {
        if (c == 1) { // no-export-bins
            opt.export_bins = false;
            continue;
        }
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
            case 'd': opt.out_dir = optarg; break;
        }
    }

    if (opt.seed == 0) {
        opt.seed = (unsigned)std::chrono::system_clock::now().time_since_epoch().count();
    }
    if (opt.threads <= 0) {
        opt.threads = std::max(1u, std::thread::hardware_concurrency());
    }

    // ensure some minimal sampling but not huge
    opt.min_sample_bits = std::max(8 * opt.n, 256);

    return opt;
}

// ---------------------- Export support ----------------------

static std::vector<int> out_fds;          // one fd per 256 bins
//static std::vector<std::mutex> file_mutexes;
static std::vector<std::unique_ptr<std::mutex>> file_mutexes;
static bool use_16bit_entry = true;

/**
 * @brief Ensures a directory exists, creating it if necessary.
 */
void ensure_directory(const std::string &dir) {
    struct stat st{};
    if (stat(dir.c_str(), &st) == -1) {
        if (mkdir(dir.c_str(), 0755) != 0) {
            std::perror("mkdir");
            std::exit(1);
        }
    }
}

/**
 * @brief Prepares and opens all 256 binary output files.
 */
void prepare_output_files(const std::string &dir) {
    ensure_directory(dir);
    out_fds.assign(256, -1);

    // Build mutex pointers in-place (unique_ptr avoids moving std::mutex).
    file_mutexes.clear();
    file_mutexes.reserve(256);

    char path[512];
    for (int i = 0; i < 256; ++i) {
        snprintf(path, sizeof(path), "%s/0x%02X.bin", dir.c_str(), i);
        // open for write only, create, and truncate to start fresh
        int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
        if (fd < 0) {
            std::perror("open");
            std::exit(1);
        }
        out_fds[i] = fd;

        // create a mutex owned by a unique_ptr and append it
        file_mutexes.emplace_back(std::make_unique<std::mutex>());
    }
}


/**
 * @brief Writes all bytes from a buffer, handling partial writes.
 */
static ssize_t full_write_fd(int fd, const void* buf, size_t count) {
    const char* p = static_cast<const char*>(buf);
    while (count) {
        ssize_t w = write(fd, p, count);
        if (w < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        p += w;
        count -= w;
    }
    return 0;
}

/**
 * @brief Closes all open output files.
 */
void close_output_files() {
    for (int fd : out_fds)
        if (fd >= 0)
            close(fd);
    out_fds.clear();
    file_mutexes.clear(); // unique_ptr destructs the mutexes
}

/**
 * @brief Helper to export a candidate (mask, seed) to a particular PWM bin index (0..255).
 */

inline void export_candidate_bin(int pwm_index, u64 mask, u64 seed) {
    if (pwm_index < 0 || pwm_index > 255) return;
    int fd = out_fds[pwm_index];
    if (fd < 0) return;

    // dereference the unique_ptr to get the mutex
    std::lock_guard<std::mutex> lock(*file_mutexes[pwm_index]);

    if (use_16bit_entry) {
        uint16_t m16 = static_cast<uint16_t>(mask);
        uint16_t s16 = static_cast<uint16_t>(seed);
        struct { uint16_t mask; uint16_t seed; } e;
        e.mask = m16;
        e.seed = s16;
        full_write_fd(fd, &e, sizeof(e));
    } else {
        uint32_t m32 = static_cast<uint32_t>(mask);
        uint32_t s32 = static_cast<uint32_t>(seed);
        struct { uint32_t mask; uint32_t seed; } e;
        e.mask = m32;
        e.seed = s32;
        full_write_fd(fd, &e, sizeof(e));
    }
}

// ---------------------- Evaluation ----------------------

/**
 * @brief Evaluates a single mask for all target duty levels.
 *
 * @param mask The LFSR tap mask.
 * @param n The LFSR bit-width.
 * @param targets The vector of target duty cycles.
 * @param best_configs Global vector of best LFSRConfig found so far (to be merged into).
 * @param candidate_counts Global vector of counts of valid candidates per target (to be aggregated).
 * @param merge_mutex Mutex for protecting access to best_configs and candidate_counts.
 * @param lc_weight Weight for linear complexity in the scoring.
 * @param period_weight Weight for cycle period in the scoring.
 * @param min_period_ratio Minimum required period ratio.
 * @param min_sample_bits Minimum sample length for LC calculation.
 * @param max_sample_bits Maximum sample length for LC calculation.
 * @param block_size Window size for the sliding window error constraint.
 * @param max_block_error Maximum allowed error within a sliding window.
 * @param resolution The PWM resolution (number of target steps).
 * @param export_bins_flag Whether to export binary candidates.
 */
void evaluate_mask_for_all_levels(u64 mask, int n, const std::vector<double> &targets,
                                  std::vector<LFSRConfig> &best_configs,
                                  std::vector<uint32_t> &candidate_counts,
                                  std::mutex &merge_mutex, double lc_weight,
                                  double period_weight, double min_period_ratio,
                                  int min_sample_bits, int max_sample_bits,
                                  int block_size, double max_block_error,
                                  int resolution, bool export_bins_flag) {
    // Find cycles
    std::vector<std::vector<u64>> cycles = find_all_cycles_raw(mask, n);
    if (cycles.empty()) return;

    // Thread-local LC cache
    static thread_local std::unordered_map<uint64_t, int> lc_cache;

    // small epsilon for floating point comparisons
    const double EPS = 1e-12;

    double max_bonus = lc_weight + period_weight; // safe upper bound for pruning

    // Per-target local best (to merge later)
    std::vector<LFSRConfig> local_configs(targets.size());
    for (auto &cfg : local_configs) cfg.score = std::numeric_limits<double>::infinity();

    // Per-target local candidate counts to aggregate at merge time
    std::vector<uint32_t> local_counts(targets.size(), 0);

    const u64 max_state = (1ULL << n);

    // Maximum number of window start positions to check exhaustively.
    const int MAX_WINDOWS_CHECK = 16384;

    // How many phase offsets to use when estimating LC (reduce phase-bias)
    const int LC_PHASES = 3;

    for (const auto &cycle : cycles) {
        int period = (int)cycle.size();

        // Optionally skip very tiny cycles
        if (min_period_ratio > 0.0) {
            double ratio = (double)period / (double)(max_state);
            if (ratio < min_period_ratio) continue;
        }

        // Build a sorted list of numeric state values (ascending)
        std::vector<u64> sorted = cycle;
        std::sort(sorted.begin(), sorted.end());

        // Map state -> index in cycle (to know where to start sampling)
        std::unordered_map<u64, int> state_to_idx;
        state_to_idx.reserve(cycle.size() * 2);
        for (int idx = 0; idx < period; ++idx) state_to_idx[cycle[idx]] = idx;

        // For each target, determine the nearest achievable k/period and evaluate
        for (size_t ti = 0; ti < targets.size(); ++ti) {
            double target = targets[ti];

            // integer number of ones we'd like
            int k_ideal = (int)std::round(target * (double)period);
            if (k_ideal < 0) k_ideal = 0;
            if (k_ideal > period - 1) k_ideal = period - 1;

            // evaluate up to three candidates: k_ideal and maybe k_ideal +/- 1
            int k_candidates[3];
            int k_count = 0;
            k_candidates[k_count++] = k_ideal;
            if (k_ideal > 0) k_candidates[k_count++] = k_ideal - 1;
            if (k_ideal < period - 1) k_candidates[k_count++] = k_ideal + 1;

            for (int kc = 0; kc < k_count; ++kc) {
                int k = k_candidates[kc];

                // j index in sorted that yields exactly k ones: j = period - k - 1
                int j = period - k - 1;
                if (j < 0 || j >= period) continue;
                u64 threshold_val = sorted[j];

                // find index in cycle where threshold_val occurs (this will be the 'seed' state)
                auto itidx = state_to_idx.find(threshold_val);
                if (itidx == state_to_idx.end()) continue;
                int seed_idx = itidx->second;

                // compute actual duty (should equal k/period)
                double duty = (double)k / (double)period;
                double error = fabs(duty - target);

                // Quick pruning using optimistic bonuses
                double min_possible_score = error - max_bonus;
                if (min_possible_score + EPS >= local_configs[ti].score) continue;

                // Build ones_flags for the cycle: 1 if state > threshold, else 0
                std::vector<int> ones_flags(period);
                int ones_count_period = 0;
                for (int p = 0; p < period; ++p) {
                    int on = (cycle[p] > threshold_val) ? 1 : 0;
                    ones_flags[p] = on;
                    ones_count_period += on;
                }
                // If ones count per period doesn't match k, recompute duty
                if (ones_count_period != k) {
                    duty = (double)ones_count_period / (double)period;
                    error = fabs(duty - target);
                }

                // Prepare prefix sums on an extended array so any circular block of length block_size can be queried:
                int ext_len = period + block_size;
                std::vector<int> prefix(ext_len + 1, 0);
                for (int idx = 0; idx < ext_len; ++idx) {
                    prefix[idx + 1] = prefix[idx] + ones_flags[idx % period];
                }

                // Decide how many start positions to check:
                int check_positions = period;
                if (period > MAX_WINDOWS_CHECK) {
                    check_positions = MAX_WINDOWS_CHECK;
                }

                // To avoid aliasing, ensure we cover all residues modulo gcd(period, block_size).
                int g = std::gcd(period, block_size);
                if (g < 1) g = 1;

                // Build deterministic start position iterator that ensures we touch each residue class modulo g.
                std::vector<int> starts;
                starts.reserve(std::min(period, check_positions + g)); // reserve enough space
                // first push the first representative for each residue class
                for (int r = 0; r < g && (int)starts.size() < check_positions; ++r) starts.push_back(r);
                // then fill remaining slots by stepping by g across the period
                // use a deterministic spread of positions that covers the period
                for (int r = 0; (int)starts.size() < check_positions && r < period; ++r) {
                    int val = (r * g) % period; // deterministic spread, likely to repeat
                    starts.push_back(val);
                }

                // remove duplicates and trim to max check_positions
                std::sort(starts.begin(), starts.end());
                starts.erase(std::unique(starts.begin(), starts.end()), starts.end());
                if ((int)starts.size() > check_positions) starts.resize(check_positions);

                double max_block_error_seen = 0.0;
                double max_block_duty_delta = 0.0;
                double prev_block_duty = -1.0;
                bool window_reject = false;

                // Check circular sliding windows
                for (int sidx = 0; sidx < (int)starts.size(); ++sidx) {
                    int start_pos = starts[sidx];
                    // Wrap-around is handled by prefix sums: prefix[start_pos + block_size] uses the extended array
                    int ones_in_window = prefix[start_pos + block_size] - prefix[start_pos];
                    double block_duty = (double)ones_in_window / (double)block_size;
                    double block_error = fabs(block_duty - target);

                    if (block_error > max_block_error_seen) max_block_error_seen = block_error;

                    if (prev_block_duty >= -0.5) {
                        double delta = fabs(block_duty - prev_block_duty);
                        if (delta > max_block_duty_delta) max_block_duty_delta = delta;
                    }
                    prev_block_duty = block_duty;

                    if (block_error > max_block_error + EPS) {
                        window_reject = true;
                        break;
                    }
                }

                if (window_reject) continue;

                // Constraint: the maximum duty change between adjacent windows should not exceed the PWM step size
                double pwm_bin = 1.0 / (double)(std::max(2, resolution - 1));
                if (max_block_duty_delta > pwm_bin + 1e-9) {
                    continue;
                }

                // Candidate passed sliding-window checks -> increment local candidate count
                local_counts[ti]++;

                // Export candidate (mask, seed) to bin files if requested
                if (export_bins_flag) {
                    int pwm_index = (int)ti;
                    if (pwm_index < 0) pwm_index = 0;
                    if (pwm_index > 255) pwm_index = 255;
                    export_candidate_bin(pwm_index, mask, threshold_val);
                }

                // Build sample bits for LC/hashing. Use a few phase offsets.
                int sample_len = std::min(max_sample_bits, std::max(min_sample_bits, period));
                // limit sample_len to something sane
                sample_len = std::min(sample_len, 16384);

                std::vector<int> lc_vals;
                lc_vals.reserve(LC_PHASES);
                for (int phase = 0; phase < LC_PHASES; ++phase) {
                    int phase_offset = (seed_idx + phase * (period / std::max(1, LC_PHASES))) % period;
                    std::vector<uint8_t> sample_bits;
                    sample_bits.reserve(sample_len);
                    for (int s = 0; s < sample_len; ++s) {
                        int idx = (phase_offset + s) % period;
                        sample_bits.push_back((uint8_t)ones_flags[idx]);
                    }
                    uint64_t h = fnv1a_hash_bits(sample_bits);
                    int cand_lc;
                    auto it = lc_cache.find(h);
                    if (it != lc_cache.end()) {
                        cand_lc = it->second;
                    } else {
                        cand_lc = berlekamp_massey(sample_bits);
                        lc_cache.emplace(h, cand_lc);
                    }
                    lc_vals.push_back(cand_lc);
                }
                // median of LC values to reduce phase bias
                std::sort(lc_vals.begin(), lc_vals.end());
                int cand_lc = lc_vals[lc_vals.size() / 2];

                // Compose score using LC and period bonuses; record candidate's "error" as max_block_error_seen
                double lc_term = (double)cand_lc / (double)std::max(1, n);
                double period_term = (period > 0) ? (std::log2((double)period) / (double)std::max(1, n)) : 0.0;
                double lc_bonus = lc_weight * lc_term;
                double period_bonus = period_weight * period_term;
                double score = max_block_error_seen - lc_bonus - period_bonus; // use block-based error in score

                if (score + EPS < local_configs[ti].score) {
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

/**
 * @brief Writes the final best LFSR configurations into an Arduino-compatible header file LUT.
 */
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
        f << "  lfsr_state = lfsr_threshold;  // Start at threshold value (keeps behavior consistent with search)\n";
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
        f << "  lfsr_state = (lfsr_state >> 1) | (feedback << " << (n - 1) << ");\n";
        f << "}\n";
        f << "*/\n";
    }

    f.close();
}

/**
 * @brief Main function. Parses arguments, sets up threads, runs the search, and reports results.
 */
int main(int argc, char **argv) {
    Options opt = parse_args(argc, argv);
    if (opt.n < 8 || opt.n > 32) {
        std::cerr << "n should be between 8 and 32 (practical limits)\n";
        return 1;
    }
    // ensure reasonable cap
    opt.min_sample_bits = std::clamp(opt.min_sample_bits, 64, 16384);

    std::cerr << "LFSR PWM Threshold Search (improved, circular sliding-window, export)\n";
    std::cerr << "n=" << opt.n << " bits, resolution=" << opt.resolution
              << ", threads=" << opt.threads << "\n";
    std::cerr << "LC weight=" << opt.lc_weight << ", period weight=" << opt.period_weight << "\n";
    std::cerr << "Min taps=" << opt.min_taps << ", min_period_ratio=" << opt.min_period_ratio << "\n";
    std::cerr << "Block-size=" << opt.block_size << " steps, max-block-error=" << opt.max_block_error << "\n";
    std::cerr << "Export bins: " << (opt.export_bins ? "ENABLED" : "DISABLED") << " (out_dir=" << opt.out_dir << ")\n";

    // if exporting, prepare files
    if (opt.export_bins) {
        use_16bit_entry = (opt.n <= 16);
        prepare_output_files(opt.out_dir);
    }

    // Generate target duty cycles
    std::vector<double> targets(opt.resolution);
    for (int i = 0; i < opt.resolution; ++i) {
        targets[i] = (double)i / (double)(opt.resolution - 1);
    }

    // Generate mask list
    u64 max_mask = (1ULL << opt.n);
    u64 scan_limit = (opt.max_masks == 0) ? max_mask : std::min(opt.max_masks, max_mask);

    std::vector<u64> masks;
    masks.reserve((size_t)std::max<u64>(1, scan_limit - 1));
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

    // Candidate counts per target
    std::vector<uint32_t> candidate_counts(opt.resolution, 0);

    std::mutex merge_mutex;
    std::atomic<size_t> masks_processed{0};

    // Worker thread function
    auto worker = [&](size_t start_idx, size_t end_idx) {
        // max_sample_bits is capped by min_sample_bits logic
        int max_sample_bits_cap = 16384; 
        
        for (size_t i = start_idx; i < end_idx; ++i) {
            evaluate_mask_for_all_levels(masks[i], opt.n, targets, best_configs,
                                         candidate_counts,
                                         merge_mutex, opt.lc_weight, opt.period_weight,
                                         opt.min_period_ratio,
                                         opt.min_sample_bits, max_sample_bits_cap,
                                         opt.block_size, opt.max_block_error, opt.resolution,
                                         opt.export_bins);
            size_t processed = masks_processed.fetch_add(1) + 1;
            if (processed % 5000 == 0) {
                std::cerr << "Processed " << processed << " / " << masks.size() << " masks\r" << std::flush;
            }
        }
    };

    // Dispatch threads
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
    std::cerr << "\nCandidates passing sliding-window constraints per PWM level (index : duty -> count):\n";
    for (int i = 0; i < opt.resolution; ++i) {
        std::cerr << std::setw(3) << i << " : " << std::fixed << std::setprecision(4) << targets[i]
                  << " -> " << candidate_counts[i] << "\n";
    }

    // Duty coverage quick report
    std::cerr << "\nDuty cycle coverage:\n";
    auto count_range = [&](double lo, double hi) {
        int c = 0;
        for (int i = 0; i < opt.resolution; ++i) {
            if (targets[i] >= lo && targets[i] <= hi && best_configs[i].score != std::numeric_limits<double>::infinity()) c++;
        }
        return c;
    };
    std::cerr << "  0.00-0.10: " << count_range(0.0, 0.10) << " levels covered\n";
    std::cerr << "  0.90-1.00: " << count_range(0.90, 1.00) << " levels covered\n";

    write_arduino_lut(best_configs, opt.n, opt.output);
    std::cerr << "\nWritten Arduino LUT to " << opt.output << "\n";

    if (opt.export_bins) {
        close_output_files();
        std::cerr << "Wrote visualization bins to: " << opt.out_dir << "\n";
    }

    return 0;
}
