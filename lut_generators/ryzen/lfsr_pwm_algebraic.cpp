// lfsr_pwm_algebraic.cpp
// C++17 port of lfsr_pwm_algebraic_multiproc.py optimized for Ryzen 7840HS.
// Build suggestion (see README below):
// g++ -std=c++17 -O3 -march=znver4 -mtune=znver4 -flto -funroll-loops -fno-exceptions -fomit-frame-pointer -o lfsr_pwm_algebraic lfsr_pwm_algebraic.cpp
//
// Usage example:
// ./lfsr_pwm_algebraic --n 16 --scan 65535 --res 256 --out lfsr_pwm.c --threads 8
//
// Notes:
// - Use --threads 0 to auto-detect hardware_concurrency().
// - For very large n (>=24) memory and time blow up (visited arrays size = 2^n). Typical use: n <= 20.

#include <bits/stdc++.h>
#include <atomic>
#include <thread>
#include <mutex>
#include <getopt.h>

using u64 = uint64_t;
using u32 = uint32_t;

struct Candidate {
    double err = std::numeric_limits<double>::infinity();
    int lc = -1;
    u64 mask = 0;
    int period = 0;
    double ratio = 0.0;
    std::vector<uint8_t> sample_bits; // truncated sample
};

static inline int popcount_u64(u64 x) { return __builtin_popcountll(x); }

// Berlekamp–Massey over GF(2). bits is a vector of 0/1 (uint8_t).
int berlekamp_massey(const std::vector<uint8_t> &bits) {
    int n = (int)bits.size();
    if (n == 0) return 0;
    std::vector<uint8_t> C(n, 0), B(n, 0);
    C[0] = 1; B[0] = 1;
    int L = 0, m = -1;
    for (int N = 0; N < n; ++N) {
        // compute discrepancy d
        int d = 0;
        for (int i = 0; i <= L; ++i) {
            d ^= (C[i] & bits[N - i]);
        }
        if (d) {
            std::vector<uint8_t> T = C;
            int p = N - m;
            for (int i = 0; i + p < n; ++i) {
                C[i + p] ^= B[i];
            }
            if (L <= N / 2) {
                L = N + 1 - L;
                m = N;
                B.swap(T);
            }
        }
    }
    return L;
}

// Compute next state for Fibonacci-style right shift with mask: feedback = parity(state & mask)
static inline u64 next_state(u64 state, u64 mask, int n) {
    u64 feedback = __builtin_parityll(state & mask);
    return (state >> 1) | (feedback << (n - 1));
}

// Decompose cycles for a mask. Returns vector of pairs (bits_vector, period).
// This function is performance-critical.
// We implement per-traversal stamps to avoid clearing large arrays frequently.
void decompose_cycles_for_mask(u64 mask, int n,
                               std::vector<std::vector<uint8_t>> &out_bits,
                               std::vector<int> &out_periods) {
    const u64 max_state = (1ULL << n);
    // visited_global: marked when a state is known (part of any previously discovered path/cycle)
    // Instead of reallocating, assume caller gave fresh arrays sized to max_state and cleared.
    // However to be safe here, allocate local visited and stamp arrays.
    std::vector<uint8_t> visited(max_state, 0); // 0/1 visited
    // per-traversal stamp and pos arrays:
    std::vector<int> stamp(max_state, 0);
    std::vector<int> pos(max_state, -1);
    int traversal_id = 1;

    std::vector<u64> seq;
    seq.reserve(1024);

    for (u64 s = 1; s < max_state; ++s) {
        if (visited[s]) continue;
        u64 curr = s;
        seq.clear();
        // traverse until hit visited or repeat within this traversal
        while (true) {
            if (visited[curr]) {
                // mark all nodes in seq visited and stop
                for (u64 node : seq) visited[node] = 1;
                break;
            }
            if (stamp[curr] == traversal_id) {
                // found cycle: nodes from pos[curr] .. end are cycle
                int start_idx = pos[curr];
                int period = (int)seq.size() - start_idx;
                if (period > 0) {
                    std::vector<uint8_t> bits;
                    bits.reserve(period);
                    for (int i = start_idx; i < (int)seq.size(); ++i) {
                        bits.push_back((uint8_t)(seq[i] & 1ULL));
                    }
                    out_bits.push_back(std::move(bits));
                    out_periods.push_back(period);
                }
                for (u64 node : seq) visited[node] = 1;
                break;
            }
            // mark stamp and store position
            stamp[curr] = traversal_id;
            pos[curr] = (int)seq.size();
            seq.push_back(curr);
            curr = next_state(curr, mask, n);
        }
        ++traversal_id;
        // occasionally reset stamp arrays to avoid overflow (unlikely, but safe)
        if (traversal_id == std::numeric_limits<int>::max()) {
            std::fill(stamp.begin(), stamp.end(), 0);
            traversal_id = 1;
        }
    }
}

// Initialize a bucket vector of Candidates
std::vector<Candidate> init_bucket(int resolution) {
    return std::vector<Candidate>(resolution, Candidate());
}

// Worker function for a contiguous chunk [mask_start, mask_end)
void process_mask_range(u64 mask_start, u64 mask_end, int n, int resolution,
                        std::vector<Candidate> &out_bucket,
                        std::atomic<size_t> *progress_ptr = nullptr)
{
    const double eps = 1e-15;
    std::vector<double> targets(resolution);
    for (int i = 0; i < resolution; ++i) targets[i] = double(i) / double(resolution - 1);

    out_bucket = init_bucket(resolution);

    // temporary containers reused per-mask to reduce allocs
    std::vector<std::vector<uint8_t>> cycles_bits;
    std::vector<int> cycles_periods;
    cycles_bits.reserve(256);
    cycles_periods.reserve(256);

    for (u64 mask = mask_start; mask < mask_end; ++mask) {
        if (mask == 0) continue;
        cycles_bits.clear();
        cycles_periods.clear();
        // collect cycles
        decompose_cycles_for_mask(mask, n, cycles_bits, cycles_periods);

        for (size_t ci = 0; ci < cycles_bits.size(); ++ci) {
            const std::vector<uint8_t> &bits = cycles_bits[ci];
            int period = cycles_periods[ci];
            if (period < 1) continue;
            int ones = 0;
            // sum bits
            for (int b : bits) ones += b;
            double ratio = double(ones) / double(period);

            // evaluate for each PWM level
            for (int pwm = 0; pwm < resolution; ++pwm) {
                double target = targets[pwm];
                double err = fabs(ratio - target);
                Candidate &cur = out_bucket[pwm];

                bool better_err = (err + eps < cur.err);
                bool same_err_better_period = (fabs(err - cur.err) < eps && period > cur.period);
                if (!better_err && !same_err_better_period) continue;

                // lazy compute LC only if this candidate might replace current or tie-break
                int lc = berlekamp_massey(bits);

                bool accept = false;
                if (better_err) accept = true;
                else if (fabs(err - cur.err) < eps) {
                    if (lc > cur.lc) accept = true;
                    else if (lc == cur.lc && period > cur.period) accept = true;
                }
                if (accept) {
                    Candidate cand;
                    cand.err = err;
                    cand.lc = lc;
                    cand.mask = mask;
                    cand.period = period;
                    cand.ratio = ratio;
                    if (period <= 256) cand.sample_bits = bits;
                    else cand.sample_bits.assign(bits.begin(), bits.begin() + 256);
                    cur = std::move(cand);
                }
            }
        }

        if (progress_ptr && (++(*progress_ptr) % 1000 == 0)) {
            // optional progress update from caller
        }
    }
}

// Merge chunk bucket into master in-place
void merge_buckets(std::vector<Candidate> &master, const std::vector<Candidate> &chunk) {
    const double eps = 1e-15;
    int resolution = (int)master.size();
    for (int i = 0; i < resolution; ++i) {
        const Candidate &e = chunk[i];
        if (e.err == std::numeric_limits<double>::infinity()) continue;
        Candidate &m = master[i];
        if (e.err + eps < m.err) {
            m = e;
        } else if (fabs(e.err - m.err) < eps) {
            if (e.lc > m.lc || (e.lc == m.lc && e.period > m.period)) {
                m = e;
            }
        }
    }
}

void write_c_lut(const std::vector<Candidate> &candidates, int resolution, const std::string &varname, const std::string &filename) {
    std::ofstream f(filename);
    if (!f) {
        std::cerr << "ERROR: cannot open " << filename << " for writing\n";
        return;
    }
    f << "#include <stdint.h>\n\n";
    f << "// Format: { mask, period }\n";
    f << "// Generated by lfsr_pwm_algebraic.cpp\n";
    f << "const struct { uint32_t mask; uint32_t period; } " << varname << "[" << resolution << "] = {\n";
    for (int i = 0; i < resolution; ++i) {
        const Candidate &c = candidates[i];
        if (c.err == std::numeric_limits<double>::infinity()) {
            f << "    { 0x00000000u, 0u }, // Level " << std::setw(3) << i << " (Unmatched)\n";
        } else {
            std::ostringstream ss;
            ss << std::hex << std::setw(0) << std::nouppercase << std::setfill('0') << c.mask;
            f << "    { 0x" << std::setw(8) << std::setfill('0') << std::hex << (uint32_t)c.mask
              << std::dec << "u, " << std::setw(3) << c.period << "u }, // Level " << std::setw(3) << i
              << ": Ratio " << std::fixed << std::setprecision(6) << c.ratio
              << ", err " << std::scientific << std::setprecision(6) << c.err
              << ", LC " << std::dec << c.lc << "\n";
        }
    }
    f << "};\n";
    f.close();
}

// Simple command-line argument parser
struct Options {
    int n = 16;
    u64 scan = (1ULL << 16);
    int res = 256;
    std::string out = "lfsr_pwm.c";
    int threads = 0;
    bool treat_extremes_const = false;
};

Options parse_args(int argc, char **argv) {
    Options opt;
    static struct option long_options[] = {
        {"n", required_argument, nullptr, 'n'},
        {"scan", required_argument, nullptr, 's'},
        {"res", required_argument, nullptr, 'r'},
        {"out", required_argument, nullptr, 'o'},
        {"threads", required_argument, nullptr, 't'},
        {"treat-extremes-const", no_argument, nullptr, 'e'},
        {nullptr, 0, nullptr, 0}
    };
    int optc;
    while ((optc = getopt_long(argc, argv, "n:s:r:o:t:e", long_options, nullptr)) != -1) {
        switch (optc) {
            case 'n': opt.n = std::atoi(optarg); break;
            case 's': opt.scan = std::stoull(optarg); break;
            case 'r': opt.res = std::atoi(optarg); break;
            case 'o': opt.out = std::string(optarg); break;
            case 't': opt.threads = std::atoi(optarg); break;
            case 'e': opt.treat_extremes_const = true; break;
            default: break;
        }
    }
    return opt;
}

int main(int argc, char **argv) {
    Options opt = parse_args(argc, argv);

    if (opt.scan <= 1) {
        std::cerr << "scan must be > 1\n";
        return 1;
    }
    if (opt.n <= 0 || opt.n > 31) {
        std::cerr << "n must be between 1 and 31 (practical limit due to memory/time)\n";
        return 1;
    }

    u64 max_state_space = (1ULL << opt.n);
    u64 max_masks = std::min(opt.scan, max_state_space);
    std::cerr << "Starting Search: Width=" << opt.n << ", Resolution=" << opt.res
              << ", Masks<= " << max_masks << "\n";

    int n_threads = opt.threads <= 0 ? std::max(1u, std::thread::hardware_concurrency()) : opt.threads;
    // create mask list [1 .. max_masks-1]
    std::vector<u64> masks;
    masks.reserve((size_t)std::max<u64>(0, max_masks - 1));
    for (u64 m = 1; m < max_masks; ++m) masks.push_back(m);
    size_t total_masks = masks.size();
    if (total_masks == 0) {
        auto empty_bucket = init_bucket(opt.res);
        write_c_lut(empty_bucket, opt.res, "lfsr_pwm_config", opt.out);
        return 0;
    }

    // chunk masks into roughly n_threads parts
    std::vector<std::pair<size_t,size_t>> ranges;
    ranges.reserve(n_threads);
    size_t chunk_size = std::max<size_t>(1, total_masks / n_threads);
    size_t i = 0;
    while (i < total_masks) {
        size_t start = i;
        size_t end = std::min(total_masks, i + chunk_size);
        ranges.emplace_back(start, end);
        i = end;
    }
    size_t threads_to_use = std::min<size_t>(n_threads, ranges.size());

    std::cerr << "Using " << threads_to_use << " threads over " << ranges.size() << " ranges\n";

    std::vector<Candidate> master = init_bucket(opt.res);
    std::mutex merge_mutex;
    std::atomic<size_t> processed_ranges{0};

    // For each range spawn a thread (but cap at threads_to_use parallel)
    size_t range_count = ranges.size();
    std::atomic<size_t> next_range_idx{0};
    std::vector<std::thread> workers;
    workers.reserve(threads_to_use);

    auto worker_func = [&](int worker_id) {
        while (true) {
            size_t idx = next_range_idx.fetch_add(1);
            if (idx >= range_count) break;
            auto [start_idx, end_idx] = ranges[idx];
            u64 mask_start = masks[start_idx];
            u64 mask_end = masks[end_idx - 1] + 1; // exclusive
            std::vector<Candidate> bucket_chunk;
            std::atomic<size_t> progress{0};
            process_mask_range(mask_start, mask_end, opt.n, opt.res, bucket_chunk, &progress);
            {
                std::lock_guard<std::mutex> lk(merge_mutex);
                merge_buckets(master, bucket_chunk);
                processed_ranges.fetch_add(1);
                std::cerr << "Merged range " << (idx+1) << "/" << range_count << " (masks " << mask_start << "-" << (mask_end-1) << ")\n";
            }
        }
    };

    for (size_t t = 0; t < threads_to_use; ++t) {
        workers.emplace_back(worker_func, (int)t);
    }
    for (auto &th : workers) th.join();

    // Optionally force extremes
    if (opt.treat_extremes_const) {
        // level 0 -> always 0, level max -> always 1 (mask = all-ones sentinel)
        master[0].mask = 0x00000000u;
        master[0].period = 1;
        master[0].ratio = 0.0;
        master[0].err = 0.0;
        master[0].lc = 0;
        master[opt.res - 1].mask = (opt.n >= 32 ? 0xffffffffu : ((1u << opt.n) - 1u));
        master[opt.res - 1].period = 1;
        master[opt.res - 1].ratio = 1.0;
        master[opt.res - 1].err = 0.0;
        master[opt.res - 1].lc = 0;
    }

    size_t found = 0;
    for (const auto &c : master) if (c.err != std::numeric_limits<double>::infinity()) ++found;
    std::cerr << "Coverage: Found candidates for " << found << "/" << opt.res << " levels.\n";

    write_c_lut(master, opt.res, "lfsr_pwm_config", opt.out);
    std::cerr << "Written LUT to " << opt.out << "\n";
    return 0;
}
