// lfsr_pwm_search.cpp
// Search LFSR masks restricted to the stored width (trunc_bits), with period penalty and min_period.
// Single-file C++17 program.
//
// Key design choices:
// - Search masks in [1 .. (1<<trunc_bits)-1] (so stored/truncated mask == mask searched).
// - For each mask, decompose cycles on the full LFSR width 'n' (mask occupies low bits).
// - Candidate score = err + period_penalty / period (prefer longer period).
// - Reject cycles with period < min_period for non-extreme PWM levels.
// - Keep top-K candidates per level, do greedy unique assignment up to reuse_limit.
// - Multithreaded over mask index ranges.
//
// Note: cycle decomposition allocates arrays of size 2^n; keep n reasonably small (typical n <= 20).

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

struct Candidate {
    double err = std::numeric_limits<double>::infinity();
    double score = std::numeric_limits<double>::infinity(); // err + penalty/period (for ranking)
    int lc = -1;
    u64 mask = 0;     // truncated/stored mask (we search in this domain)
    int period = 0;
    double ratio = 0.0;
    std::vector<uint8_t> sample_bits;
};

static inline int popcount_u64(u64 x) { return __builtin_popcountll(x); }

// Berlekamp–Massey for LC over GF(2)
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
            for (int i=0; i + p < n; ++i) C[i+p] ^= B[i];
            if (L <= N/2) {
                L = N + 1 - L;
                m = N;
                B.swap(T);
            }
        }
    }
    return L;
}

// next state for Fibonacci right shift LFSR with mask (mask uses low bits positions)
static inline u64 next_state(u64 state, u64 mask, int n) {
    // parity(state & mask)
    u64 fb = __builtin_parityll(state & mask);
    return (state >> 1) | (fb << (n - 1));
}

// Decompose cycles for a given mask (on full width n). Produces cycles bit-vectors and periods.
void decompose_cycles_for_mask(u64 mask, int n,
                               std::vector<std::vector<uint8_t>> &out_bits,
                               std::vector<int> &out_periods)
{
    const u64 max_state = (1ULL << n);
    // visited / stamp arrays (size max_state)
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

// Write C LUT (mask, period) for each resolution level
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

// Write Arduino LUT (stores lower trunc_bits bits as uint16_t by default)
void write_arduino_lut(const std::vector<Candidate> &candidates, int resolution, int trunc_bits, const std::string &filename) {
    std::ofstream f(filename);
    if (!f) { std::cerr << "ERROR: cannot open " << filename << " for writing\n"; return; }
    f << "// lfsr_pwm_search.cpp\n";
    f << "// Arduino LUT (truncated to " << trunc_bits << " bits)\n";
    f << "#include <stdint.h>\n";
    f << "#if defined(__AVR__)\n#include <avr/pgmspace.h>\n#else\n#define PROGMEM\n#define pgm_read_word(addr) (*(addr))\n#endif\n\n";
    f << "const uint16_t lfsr_pwm_masks_16bit[] PROGMEM = {\n";
    u64 mask_trunc_all = (trunc_bits >= 64 ? ~0ULL : ((1ULL << trunc_bits) - 1ULL));
    for (int i = 0; i < resolution; ++i) {
        const Candidate &c = candidates[i];
        uint16_t m16;
        std::string comment;
        if (c.err == std::numeric_limits<double>::infinity()) {
            m16 = (uint16_t)(mask_trunc_all & 0xFFFFu);
            comment = "Unmatched";
        } else {
            m16 = (uint16_t)(c.mask & mask_trunc_all & 0xFFFFu);
            std::ostringstream oss;
            oss << "Ratio " << std::fixed << std::setprecision(6) << c.ratio
                << ", err " << std::scientific << std::setprecision(6) << c.err
                << ", LC " << c.lc;
            comment = oss.str();
        }
        f << " 0x" << std::hex << std::setw(4) << std::setfill('0') << std::nouppercase << m16 << "u, // " << comment << "\n" << std::dec;
    }
    f << "};\n";
    f.close();
}

// Options
struct Options {
    int n = 16;                    // LFSR width
    int trunc_bits = 16;           // stored/truncated mask width (search domain)
    u64 max_masks = 0;             // if zero: full 1<<trunc_bits
    int res = 256;                 // PWM resolution
    size_t topk = 6;
    int threads = 0;
    int iterations = 1;
    unsigned seed = 0;
    std::string out = "lfsr_pwm.c";
    std::string arduino_out;
    bool treat_extremes_const = false;
    double period_penalty = 1e-3;  // score penalty factor
    int min_period = 3;            // reject cycles shorter than this (for non-extremes)
    int reuse_limit = 1;           // allowed reuse of same stored mask
};

// parse args
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
            case 's': opt.max_masks = std::stoull(optarg); break;
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
    if (opt.trunc_bits <= 0) opt.trunc_bits = std::min(opt.n, 16);
    return opt;
}

int main(int argc, char **argv) {
    Options opt = parse_args(argc, argv);

    if (opt.n <= 0 || opt.n > 31) {
        std::cerr << "n must be between 1 and 31\n"; return 1;
    }
    if (opt.trunc_bits <= 0 || opt.trunc_bits > opt.n) {
        std::cerr << "trunc-bits must be in [1..n]\n"; return 1;
    }
    if (opt.res <= 1) { std::cerr << "res must be > 1\n"; return 1; }
    if (opt.topk < 1) opt.topk = 1;
    if (opt.iterations < 1) opt.iterations = 1;
    if (opt.threads <= 0) opt.threads = std::max(1u, std::thread::hardware_concurrency());

    // prepare mask search domain
    u64 max_mask_space = (1ULL << opt.trunc_bits);
    u64 scan_limit = opt.max_masks == 0 ? max_mask_space : std::min(max_mask_space, opt.max_masks);
    if (scan_limit <= 1) {
        std::cerr << "nothing to scan (scan_limit <= 1)\n";
        return 1;
    }
    std::vector<u64> masks;
    masks.reserve((size_t)(scan_limit - 1));
    for (u64 m = 1; m < scan_limit; ++m) masks.push_back(m);
    size_t total_masks = masks.size();

    std::cerr << "LFSR width n = " << opt.n << ", searching stored masks width = " << opt.trunc_bits
              << " (masks <= " << scan_limit-1 << "), PWM res = " << opt.res
              << ", topK=" << opt.topk << ", threads=" << opt.threads << ", iters=" << opt.iterations << "\n";
    std::cerr << "period_penalty=" << opt.period_penalty << ", min_period=" << opt.min_period
              << ", reuse_limit=" << opt.reuse_limit << ", treat_extremes_const=" << (opt.treat_extremes_const? "yes":"no") << "\n";

    // partition masks indices into ranges for threading (indices into masks vector)
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
    std::cerr << "Ranges: " << ranges.size() << ", using threads: " << n_threads << "\n";

    // master containers
    std::vector<Candidate> master_best(opt.res);
    std::vector<std::vector<Candidate>> master_alts(opt.res);
    std::mutex merge_mutex;

    // Precompute targets
    std::vector<double> targets(opt.res);
    for (int i=0;i<opt.res;++i) targets[i] = double(i) / double(opt.res - 1);

    // function to evaluate a single mask (exact decomposition)
    auto evaluate_mask = [&](u64 mask, std::vector<Candidate> &best_local, std::vector<std::vector<Candidate>> &alts_local) {
        std::vector<std::vector<uint8_t>> cycles_bits;
        std::vector<int> cycles_periods;
        cycles_bits.reserve(256); cycles_periods.reserve(256);
        decompose_cycles_for_mask(mask, opt.n, cycles_bits, cycles_periods);
        const double eps = 1e-15;
        for (size_t ci=0; ci<cycles_bits.size(); ++ci) {
            const std::vector<uint8_t> &bits = cycles_bits[ci];
            int period = cycles_periods[ci];
            if (period <= 0) continue;
            int ones = 0;
            for (uint8_t b : bits) ones += b;
            double ratio = double(ones) / double(period);
            int lc = berlekamp_massey(bits);
            // produce candidate for each pwm level (comparison uses score)
            for (int pwm=0; pwm < opt.res; ++pwm) {
                double err = fabs(ratio - targets[pwm]);
                Candidate cand;
                cand.err = err;
                cand.mask = mask;
                cand.period = period;
                cand.lc = lc;
                cand.ratio = ratio;
                if (period <= 256) cand.sample_bits = bits;
                else cand.sample_bits.assign(bits.begin(), bits.begin() + 256);
                // compute score (err + penalty/period). For very short periods period==0 shouldn't happen.
                double penalty = (period > 0 ? (opt.period_penalty / double(period)) : opt.period_penalty);
                cand.score = cand.err + penalty;

                // reject too-short cycles for non-extreme levels (unless user explicitly allows extremes by post-forcing)
                if (period < opt.min_period) {
                    if (!(opt.treat_extremes_const && (pwm == 0 || pwm == opt.res - 1))) {
                        // drop (skip) this candidate for this level
                        continue;
                    }
                }

                // update best_local
                Candidate &cur = best_local[pwm];
                bool better = (cand.score + eps < cur.score);
                bool tiepref = (fabs(cand.score - cur.score) < eps && (cand.lc > cur.lc || (cand.lc == cur.lc && cand.period > cur.period)));
                if (better || tiepref) {
                    cur = cand;
                }

                // push into alts
                push_topk(alts_local[pwm], cand, opt.topk);
            }
        }
    };

    // Main iterative search: shuffle masks each iter to diversify discovery
    for (int iter = 0; iter < opt.iterations; ++iter) {
        unsigned iter_seed = opt.seed + (unsigned)iter * 0x9e3779b9u;
        std::mt19937_64 rng(iter_seed);
        std::shuffle(masks.begin(), masks.end(), rng);
        std::cerr << "Iteration " << (iter+1) << "/" << opt.iterations << " seed=" << iter_seed << " shuffling masks\n";

        // thread-local work
        std::atomic<size_t> next_range{0};
        std::vector<std::thread> workers;
        workers.reserve(n_threads);

        for (size_t th = 0; th < n_threads; ++th) {
            workers.emplace_back([&,th]() {
                // local accumulators
                std::vector<Candidate> local_best(opt.res);
                std::vector<std::vector<Candidate>> local_alts(opt.res);
                for (int i=0;i<opt.res;++i) local_alts[i].reserve(8);

                while (true) {
                    size_t ridx = next_range.fetch_add(1);
                    if (ridx >= ranges.size()) break;
                    auto [start_i, end_i] = ranges[ridx];
                    for (size_t mi = start_i; mi < end_i; ++mi) {
                        u64 mask = masks[mi];
                        if (mask == 0) continue;
                        // evaluate exact (we reduced domain so exact is feasible)
                        evaluate_mask(mask, local_best, local_alts);
                    }
                }

                // merge local -> master
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
                    // merge alt lists
                    for (const Candidate &alt : local_alts[pwm]) push_topk(master_alts[pwm], alt, opt.topk);
                }
            });
        }
        for (auto &th : workers) th.join();
        std::cerr << "Iteration " << (iter+1) << " done - merged results\n";
    }

    // ensure master_best included in master_alts
    for (int pwm=0;pwm<opt.res;++pwm) {
        if (master_best[pwm].score < std::numeric_limits<double>::infinity()) push_topk(master_alts[pwm], master_best[pwm], opt.topk);
    }

    // Final uniqueness-aware greedy assignment on the stored/truncated mask (we already searched that domain)
    std::vector<Candidate> final_lut(opt.res);
    std::unordered_map<u64,int> usage; usage.reserve(opt.res*2);

    // assign levels in order of increasing best score so we preserve the best matches first
    std::vector<int> order(opt.res); std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b){
        return master_best[a].score < master_best[b].score;
    });

    for (int lvl : order) {
        Candidate chosen;
        bool got = false;
        // try alternatives first (they are sorted by score)
        for (const Candidate &cand : master_alts[lvl]) {
            int used = usage[cand.mask];
            if (used < opt.reuse_limit) { chosen = cand; got = true; usage[cand.mask] = used + 1; break; }
        }
        // fallback: pick best alt with minimal score (already first) even if reuse exceeded
        if (!got && !master_alts[lvl].empty()) {
            chosen = master_alts[lvl][0];
            usage[chosen.mask] += 1;
            got = true;
        }
        if (!got) {
            final_lut[lvl] = Candidate(); // unmatched
        } else {
            final_lut[lvl] = chosen;
        }
    }

    // Optionally enforce extremes as constants (0% and 100%) overriding uniqueness
    if (opt.treat_extremes_const) {
        // level 0 -> all 0 (mask 0 stored is not valid in our search, but we write explicit 0)
        final_lut[0].mask = 0x00000000u;
        final_lut[0].period = 1;
        final_lut[0].ratio = 0.0;
        final_lut[0].err = 0.0;
        final_lut[0].score = 0.0;
        final_lut[0].lc = 0;
        // level max -> all ones for trunc bits
        u64 allones = (opt.trunc_bits >= 64 ? ~0ULL : ((1ULL << opt.trunc_bits) - 1ULL));
        final_lut[opt.res - 1].mask = allones;
        final_lut[opt.res - 1].period = 1;
        final_lut[opt.res - 1].ratio = 1.0;
        final_lut[opt.res - 1].err = 0.0;
        final_lut[opt.res - 1].score = 0.0;
        final_lut[opt.res - 1].lc = 0;
    }

    // Stats
    size_t covered = 0;
    for (const auto &c : final_lut) if (c.err != std::numeric_limits<double>::infinity()) ++covered;
    std::unordered_set<u64> unique_masks;
    for (const auto &pr : usage) if (pr.second > 0) unique_masks.insert(pr.first);
    std::cerr << "Final: coverage " << covered << "/" << opt.res << ", unique stored masks used: " << unique_masks.size() << "\n";

    // For debugging: count how many final entries have short period < threshold
    int short_thresh = std::max(1, opt.min_period);
    int shortcnt = 0;
    for (int i=0;i<opt.res;++i) if (final_lut[i].err != std::numeric_limits<double>::infinity() && final_lut[i].period < short_thresh) ++shortcnt;
    std::cerr << "Levels with period < " << short_thresh << ": " << shortcnt << "\n";

    // Output LUTs
    write_c_lut(final_lut, opt.res, "lfsr_pwm_config", opt.out);
    std::cerr << "Written C LUT to " << opt.out << "\n";
    if (!opt.arduino_out.empty()) {
        write_arduino_lut(final_lut, opt.res, opt.trunc_bits, opt.arduino_out);
        std::cerr << "Written Arduino LUT to " << opt.arduino_out << "\n";
    }

    return 0;
}
