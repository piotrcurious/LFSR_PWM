// lfsr_pwm_search.cpp
// Single-file LFSR PWM mask search and Arduino LUT generator.
// Searches only stored mask width (trunc-bits) and writes Arduino LUT with mask+seed only.
//
// Build: g++ -std=c++17 -O3 -march=znver4 -mtune=znver4 -flto -funroll-loops -fno-exceptions -fomit-frame-pointer -o lfsr_pwm_search lfsr_pwm_search.cpp

#include <bits/stdc++.h>
#include <atomic>
#include <thread>
#include <mutex>
#include <getopt.h>
#include <chrono>
#include <sstream>
#include <unordered_map>

using u64 = uint64_t;
using u32 = uint32_t;
using u16 = uint16_t;

struct Candidate {
    double err = std::numeric_limits<double>::infinity();   // raw absolute error
    double score = std::numeric_limits<double>::infinity(); // err + penalty/period used for sorting
    int lc = -1;
    u64 mask = 0;   // stored mask (search domain)
    u64 seed = 0;   // representative starting state for the chosen cycle
    int period = 0;
    double ratio = 0.0;
    std::vector<uint8_t> sample_bits;
};

static inline int popcount_u64(u64 x) { return __builtin_popcountll(x); }

// Berlekamp–Massey over GF(2)
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

// next state for Fibonacci-style right shift LFSR: feedback = parity(state & mask)
static inline u64 next_state(u64 state, u64 mask, int n) {
    u64 fb = __builtin_parityll(state & mask);
    return (state >> 1) | (fb << (n - 1));
}

// Decompose cycles for a mask (over full width n).
// Outputs: bit vectors (LSB samples), periods, representative states (the first state of each cycle)
void decompose_cycles_for_mask(u64 mask, int n,
                               std::vector<std::vector<uint8_t>> &out_bits,
                               std::vector<int> &out_periods,
                               std::vector<u64> &out_rep_state)
{
    const u64 max_state = (1ULL << n);
    // visited + stamp arrays sized 2^n
    std::vector<uint8_t> visited(max_state, 0);
    std::vector<int> stamp(max_state, 0);
    std::vector<int> pos(max_state, -1);
    int traversal_id = 1;
    std::vector<u64> seq; seq.reserve(1024);

    for (u64 s = 1; s < max_state; ++s) {
        if (visited[s]) continue;
        seq.clear();
        u64 curr = s;
        while (true) {
            if (visited[curr]) {
                for (u64 v : seq) visited[v] = 1;
                break;
            }
            if (stamp[curr] == traversal_id) {
                int start_idx = pos[curr];
                int period = (int)seq.size() - start_idx;
                if (period > 0) {
                    std::vector<uint8_t> bits; bits.reserve(period);
                    for (int i = start_idx; i < (int)seq.size(); ++i) bits.push_back((uint8_t)(seq[i] & 1ULL));
                    out_bits.push_back(std::move(bits));
                    out_periods.push_back(period);
                    out_rep_state.push_back(seq[start_idx]); // non-zero representative
                }
                for (u64 v : seq) visited[v] = 1;
                break;
            }
            stamp[curr] = traversal_id;
            pos[curr] = (int)seq.size();
            seq.push_back(curr);
            curr = next_state(curr, mask, n);
        }
        ++traversal_id;
        if (traversal_id == std::numeric_limits<int>::max()) {
            std::fill(stamp.begin(), stamp.end(), 0);
            traversal_id = 1;
        }
    }
}

// push candidate into top-K vector using score ordering (ascending). Tie-break: larger LC, larger period.
void push_topk(std::vector<Candidate> &vec, const Candidate &cand, size_t K) {
    auto cmp = [](const Candidate &a, const Candidate &b) {
        const double eps = 1e-15;
        if (a.score + eps < b.score) return true;
        if (b.score + eps < a.score) return false;
        if (a.lc != b.lc) return a.lc > b.lc;
        return a.period > b.period;
    };
    auto it = std::lower_bound(vec.begin(), vec.end(), cand, [&](const Candidate &x, const Candidate &y){ return cmp(x,y); });
    vec.insert(it, cand);
    if (vec.size() > K) vec.resize(K);
}

// Write a standard C LUT (mask, period) for inspection / debugging
void write_c_lut(const std::vector<Candidate> &candidates, int resolution, const std::string &varname, const std::string &filename) {
    std::ofstream f(filename);
    if (!f) { std::cerr << "ERROR: cannot open " << filename << " for writing\n"; return; }
    f << "#include <stdint.h>\n\n";
    f << "// Format: { mask, period }\n";
    f << "// Generated by lfsr_pwm_search.cpp\n";
    f << "const struct { uint32_t mask; uint32_t period; } " << varname << "[" << resolution << "] = {\n";
    for (int i = 0; i < resolution; ++i) {
        const Candidate &c = candidates[i];
        if (c.err == std::numeric_limits<double>::infinity()) {
            f << "    { 0x00000000u, 0u }, // Level " << std::setw(3) << i << " (Unmatched)\n";
        } else {
            std::ostringstream ss;
            ss << std::hex << std::setw(8) << std::setfill('0') << (uint32_t)c.mask;
            f << "    { 0x" << ss.str() << std::dec << "u, " << std::setw(3) << c.period << "u }, // Level " << std::setw(3) << i
              << ": Ratio " << std::fixed << std::setprecision(6) << c.ratio
              << ", err " << std::scientific << std::setprecision(6) << c.err
              << ", LC " << std::dec << c.lc << "\n";
        }
    }
    f << "};\n";
    f.close();
}

// Write Arduino LUT that contains ONLY mask and seed per level.
// If trunc_bits <= 16 uses uint16_t entries, otherwise uses uint32_t entries.
void write_arduino_lut_mask_seed(const std::vector<Candidate> &candidates, int resolution, int trunc_bits, const std::string &filename) {
    std::ofstream f(filename);
    if (!f) { std::cerr << "ERROR: cannot open " << filename << " for writing\n"; return; }
    f << "// Arduino LUT: mask + seed (truncated to " << trunc_bits << " bits)\n";
    f << "#include <stdint.h>\n";
    f << "#if defined(__AVR__)\n#include <avr/pgmspace.h>\n#else\n#define PROGMEM\n#define pgm_read_word(addr) (*(addr))\n#define pgm_read_dword(addr) (*(addr))\n#endif\n\n";

    u64 trunc_mask = (trunc_bits >= 64 ? ~0ULL : ((1ULL << trunc_bits) - 1ULL));

    if (trunc_bits <= 16) {
        f << "const struct { uint16_t mask; uint16_t seed; } lfsr_pwm_lut[] PROGMEM = {\n";
        for (int i=0;i<resolution;++i) {
            const Candidate &c = candidates[i];
            if (c.err == std::numeric_limits<double>::infinity()) {
                f << "  { 0x0000u, 0x0001u }, // Level " << i << " (unmatched)\n";
            } else {
                uint16_t m = (uint16_t)(c.mask & trunc_mask & 0xFFFFu);
                uint16_t s = (uint16_t)(c.seed & trunc_mask & 0xFFFFu);
                f << "  { 0x" << std::hex << std::setw(4) << std::setfill('0') << m
                  << "u, 0x" << std::setw(4) << s << "u }, "
                  << std::dec << "// Level " << i << " Ratio " << std::fixed << std::setprecision(6) << c.ratio
                  << ", err " << std::scientific << std::setprecision(6) << c.err << ", LC " << c.lc << "\n";
            }
        }
        f << "};\n";
    } else {
        // larger than 16 bits: use uint32_t entries
        f << "const struct { uint32_t mask; uint32_t seed; } lfsr_pwm_lut[] PROGMEM = {\n";
        for (int i=0;i<resolution;++i) {
            const Candidate &c = candidates[i];
            if (c.err == std::numeric_limits<double>::infinity()) {
                f << "  { 0x00000000u, 0x00000001u }, // Level " << i << " (unmatched)\n";
            } else {
                uint32_t m = (uint32_t)(c.mask & trunc_mask);
                uint32_t s = (uint32_t)(c.seed & trunc_mask);
                std::ostringstream oss;
                oss << "Ratio " << std::fixed << std::setprecision(6) << c.ratio
                    << ", err " << std::scientific << std::setprecision(6) << c.err
                    << ", LC " << c.lc;
                f << "  { 0x" << std::hex << std::setw(8) << std::setfill('0') << m
                  << "u, 0x" << std::setw(8) << s << "u }, // " << oss.str() << "\n" << std::dec;
            }
        }
        f << "};\n";
    }
    f.close();
}

// Command-line options
struct Options {
    int n = 16;                     // LFSR width in bits (must equal trunc_bits)
    int trunc_bits = 16;            // stored mask width; we force n == trunc_bits
    u64 scan = 0;                   // max masks to scan (0 => full domain 1<<trunc_bits)
    int res = 256;                  // PWM resolution
    size_t topk = 6;                // top-K candidates per level
    int threads = 0;
    int iterations = 1;
    unsigned seed = 0;
    std::string out = "lfsr_pwm.c";
    std::string arduino_out;        // if empty, don't write Arduino LUT
    bool treat_extremes_const = false;
    double period_penalty = 1e-3;
    int min_period = 3;
    int reuse_limit = 1;
};

Options parse_args(int argc, char **argv) {
    Options opt;
    static struct option long_options[] = {
        {"n", required_argument, nullptr, 'n'},
        {"trunc-bits", required_argument, nullptr, 'b'},
        {"scan", required_argument, nullptr, 's'},
        {"res", required_argument, nullptr, 'r'},
        {"topk", required_argument, nullptr, 'k'},
        {"threads", required_argument, nullptr, 't'},
        {"iters", required_argument, nullptr, 'i'},
        {"seed", required_argument, nullptr, 'S'},
        {"out", required_argument, nullptr, 'o'},
        {"arduino-out", required_argument, nullptr, 'a'},
        {"treat-extremes-const", no_argument, nullptr, 'e'},
        {"period-penalty", required_argument, nullptr, 'p'},
        {"min-period", required_argument, nullptr, 'm'},
        {"reuse-limit", required_argument, nullptr, 1001},
        {nullptr,0,nullptr,0}
    };
    int optc;
    while ((optc = getopt_long(argc, argv, "n:b:s:r:k:t:i:S:o:a:ep:m:", long_options, nullptr)) != -1) {
        switch (optc) {
            case 'n': opt.n = std::atoi(optarg); break;
            case 'b': opt.trunc_bits = std::atoi(optarg); break;
            case 's': opt.scan = std::stoull(optarg); break;
            case 'r': opt.res = std::atoi(optarg); break;
            case 'k': opt.topk = std::stoul(optarg); break;
            case 't': opt.threads = std::atoi(optarg); break;
            case 'i': opt.iterations = std::atoi(optarg); break;
            case 'S': opt.seed = (unsigned)std::stoul(optarg); break;
            case 'o': opt.out = std::string(optarg); break;
            case 'a': opt.arduino_out = std::string(optarg); break;
            case 'e': opt.treat_extremes_const = true; break;
            case 'p': opt.period_penalty = std::stod(optarg); break;
            case 'm': opt.min_period = std::atoi(optarg); break;
            case 1001: opt.reuse_limit = std::atoi(optarg); break;
            default: break;
        }
    }
    if (opt.seed == 0) opt.seed = (unsigned)std::chrono::system_clock::now().time_since_epoch().count();
    if (opt.trunc_bits <= 0) opt.trunc_bits = opt.n;
    return opt;
}

int main(int argc, char **argv) {
    Options opt = parse_args(argc, argv);

    // Require trunc_bits == n to avoid truncation collisions
    if (opt.n != opt.trunc_bits) {
        std::cerr << "NOTE: forcing n == trunc-bits to avoid truncation collisions. (You passed n=" << opt.n << " trunc-bits=" << opt.trunc_bits << ")\n";
        opt.n = opt.trunc_bits;
    }

    if (opt.n <= 0 || opt.n > 24) {
        std::cerr << "n must be between 1 and 24 (practical limit due to memory/time). Use smaller n.\n";
        return 1;
    }
    if (opt.res <= 1) { std::cerr << "res must be > 1\n"; return 1; }
    if (opt.topk < 1) opt.topk = 1;
    if (opt.iterations < 1) opt.iterations = 1;
    if (opt.threads <= 0) opt.threads = std::max(1u, std::thread::hardware_concurrency());
    if (opt.min_period < 1) opt.min_period = 1;
    if (opt.reuse_limit < 1) opt.reuse_limit = 1;

    u64 mask_space = (1ULL << opt.trunc_bits);
    u64 scan_limit = (opt.scan == 0) ? mask_space : std::min(opt.scan, mask_space);
    if (scan_limit <= 1) { std::cerr << "nothing to scan\n"; return 1; }

    std::vector<u64> masks;
    masks.reserve((size_t)(scan_limit - 1));
    for (u64 m = 1; m < scan_limit; ++m) masks.push_back(m);
    size_t total_masks = masks.size();

    std::cerr << "Search domain: n=" << opt.n << " trunc_bits=" << opt.trunc_bits << " masks<= " << (scan_limit-1)
              << " PWM res=" << opt.res << " topK=" << opt.topk << " threads=" << opt.threads << " iters=" << opt.iterations << "\n";
    std::cerr << "period_penalty=" << opt.period_penalty << " min_period=" << opt.min_period << " reuse_limit=" << opt.reuse_limit
              << " treat_extremes_const=" << (opt.treat_extremes_const ? "yes":"no") << "\n";

    // partition indices for threads
    size_t n_threads = std::min<size_t>(opt.threads, total_masks);
    size_t chunk = std::max<size_t>(1, total_masks / n_threads);
    std::vector<std::pair<size_t,size_t>> ranges;
    size_t idx = 0;
    while (idx < total_masks) {
        size_t start = idx;
        size_t end = std::min(total_masks, idx + chunk);
        ranges.emplace_back(start, end);
        idx = end;
    }
    n_threads = std::min(n_threads, ranges.size());
    std::cerr << "Using " << n_threads << " worker threads over " << ranges.size() << " ranges\n";

    // master containers
    std::vector<Candidate> master_best(opt.res);
    std::vector<std::vector<Candidate>> master_alts(opt.res);
    std::mutex merge_mutex;

    // targets
    std::vector<double> targets(opt.res);
    for (int i=0;i<opt.res;++i) targets[i] = double(i) / double(opt.res - 1);

    // helper: evaluate a mask exactly and populate best_local and alts_local
    auto evaluate_mask_exact = [&](u64 mask, std::vector<Candidate> &best_local, std::vector<std::vector<Candidate>> &alts_local) {
        std::vector<std::vector<uint8_t>> cycles_bits;
        std::vector<int> cycles_periods;
        std::vector<u64> cycles_rep;
        decompose_cycles_for_mask(mask, opt.n, cycles_bits, cycles_periods, cycles_rep);
        const double eps = 1e-15;

        for (size_t ci=0; ci<cycles_bits.size(); ++ci) {
            const std::vector<uint8_t> &bits = cycles_bits[ci];
            int period = cycles_periods[ci];
            if (period <= 0) continue;
            int ones = 0;
            for (uint8_t b : bits) ones += b;
            double ratio = double(ones) / double(period);
            int lc = berlekamp_massey(bits);
            u64 rep_state = cycles_rep[ci];
            // Build candidate template
            for (int pwm=0; pwm<opt.res; ++pwm) {
                double err = fabs(ratio - targets[pwm]);
                Candidate cand;
                cand.err = err;
                cand.mask = mask;
                cand.seed = rep_state;
                cand.period = period;
                cand.lc = lc;
                cand.ratio = ratio;
                if (period <= 256) cand.sample_bits = bits;
                else cand.sample_bits.assign(bits.begin(), bits.begin() + 256);
                // compute score with penalty (avoid division by zero)
                double penalty = (period > 0 ? (opt.period_penalty / double(period)) : opt.period_penalty);
                cand.score = cand.err + penalty;

                // reject too-short cycles for non-extremes
                if (period < opt.min_period) {
                    if (!(opt.treat_extremes_const && (pwm == 0 || pwm == opt.res - 1))) {
                        continue;
                    }
                }

                // update best_local using score
                Candidate &cur = best_local[pwm];
                bool better = (cand.score + eps < cur.score);
                bool tiepref = (fabs(cand.score - cur.score) < eps && (cand.lc > cur.lc || (cand.lc == cur.lc && cand.period > cur.period)));
                if (better || tiepref) cur = cand;
                push_topk(alts_local[pwm], cand, opt.topk);
            }
        }
    };

    // iterative shuffled search
    for (int iter = 0; iter < opt.iterations; ++iter) {
        unsigned iter_seed = opt.seed + (unsigned)iter * 0x9e3779b9u;
        std::mt19937_64 rng(iter_seed);
        std::shuffle(masks.begin(), masks.end(), rng);
        std::cerr << "Iteration " << (iter+1) << "/" << opt.iterations << " seed=" << iter_seed << " shuffled masks\n";

        std::atomic<size_t> next_range_idx{0};
        std::vector<std::thread> workers;
        workers.reserve(n_threads);

        for (size_t th = 0; th < n_threads; ++th) {
            workers.emplace_back([&,th]() {
                std::vector<Candidate> local_best(opt.res);
                std::vector<std::vector<Candidate>> local_alts(opt.res);
                for (int i=0;i<opt.res;++i) local_alts[i].reserve(8);

                while (true) {
                    size_t ridx = next_range_idx.fetch_add(1);
                    if (ridx >= ranges.size()) break;
                    auto [start_i, end_i] = ranges[ridx];
                    for (size_t mi = start_i; mi < end_i; ++mi) {
                        u64 mask = masks[mi];
                        if (mask == 0) continue;
                        evaluate_mask_exact(mask, local_best, local_alts);
                    }
                }

                // merge local into master
                std::lock_guard<std::mutex> lk(merge_mutex);
                for (int pwm=0;pwm<opt.res;++pwm) {
                    const Candidate &c = local_best[pwm];
                    if (c.score < std::numeric_limits<double>::infinity()) {
                        Candidate &g = master_best[pwm];
                        const double eps = 1e-15;
                        if (c.score + eps < g.score) g = c;
                        else if (fabs(c.score - g.score) < eps) {
                            if (c.lc > g.lc || (c.lc == g.lc && c.period > g.period)) g = c;
                        }
                    }
                    // merge alts
                    for (const Candidate &alt : local_alts[pwm]) push_topk(master_alts[pwm], alt, opt.topk);
                }
            });
        }

        for (auto &th : workers) th.join();
        std::cerr << "Iteration " << (iter+1) << " merged results\n";
    }

    // ensure master_best included in master_alts
    for (int pwm=0;pwm<opt.res;++pwm) {
        if (master_best[pwm].score < std::numeric_limits<double>::infinity()) push_topk(master_alts[pwm], master_best[pwm], opt.topk);
    }

    // Final uniqueness-aware assignment (on stored mask)
    std::vector<Candidate> final_lut(opt.res);
    std::unordered_map<u64,int> usage; usage.reserve(opt.res * 2);

    // assign in order of increasing score (preserve best matches)
    std::vector<int> order(opt.res);
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b){
        return master_best[a].score < master_best[b].score;
    });

    for (int lvl : order) {
        Candidate chosen;
        bool got = false;
        // prefer alternatives with stored mask usage < reuse_limit
        for (const Candidate &cand : master_alts[lvl]) {
            int used = usage[cand.mask];
            if (used < opt.reuse_limit) {
                chosen = cand;
                got = true;
                usage[cand.mask] = used + 1;
                break;
            }
        }
        // fallback: pick best alt regardless of reuse limit (first element)
        if (!got && !master_alts[lvl].empty()) {
            chosen = master_alts[lvl][0];
            usage[chosen.mask] += 1;
            got = true;
        }
        if (!got) final_lut[lvl] = Candidate();
        else final_lut[lvl] = chosen;
    }

    // Optionally force extremes to constant 0/1 (override uniqueness)
    if (opt.treat_extremes_const) {
        final_lut[0].mask = 0x00000000u;
        final_lut[0].seed = 0x00000001u; // seed 1 is safe for a constant-off mask (mask==0 produces no feedback; storing seed for completeness)
        final_lut[0].period = 1;
        final_lut[0].ratio = 0.0;
        final_lut[0].err = 0.0;
        final_lut[0].score = 0.0;
        final_lut[0].lc = 0;
        u64 allones = (opt.trunc_bits >= 64 ? ~0ULL : ((1ULL << opt.trunc_bits) - 1ULL));
        final_lut[opt.res - 1].mask = allones;
        final_lut[opt.res - 1].seed = allones; // all-ones seed
        final_lut[opt.res - 1].period = 1;
        final_lut[opt.res - 1].ratio = 1.0;
        final_lut[opt.res - 1].err = 0.0;
        final_lut[opt.res - 1].score = 0.0;
        final_lut[opt.res - 1].lc = 0;
    }

    // Stats & diagnostics
    size_t covered = 0;
    for (const auto &c : final_lut) if (c.err != std::numeric_limits<double>::infinity()) ++covered;
    std::unordered_set<u64> unique_masks;
    for (const auto &p : usage) if (p.second > 0) unique_masks.insert(p.first);
    std::cerr << "Final coverage: " << covered << "/" << opt.res << ", unique stored masks used: " << unique_masks.size() << "\n";

    int short_threshold = std::max(1, opt.min_period);
    int shortcnt = 0;
    for (int i=0;i<opt.res;++i) if (final_lut[i].err != std::numeric_limits<double>::infinity() && final_lut[i].period < short_threshold) ++shortcnt;
    std::cerr << "Levels with period < " << short_threshold << ": " << shortcnt << "\n";

    // Write outputs
    write_c_lut(final_lut, opt.res, "lfsr_pwm_config", opt.out);
    std::cerr << "Written C LUT to " << opt.out << "\n";
    if (!opt.arduino_out.empty()) {
        write_arduino_lut_mask_seed(final_lut, opt.res, opt.trunc_bits, opt.arduino_out);
        std::cerr << "Written Arduino LUT (mask+seed) to " << opt.arduino_out << "\n";
    }

    return 0;
}
