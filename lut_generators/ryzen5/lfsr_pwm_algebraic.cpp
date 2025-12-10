// lfsr_pwm_algebraic_topk_fixed.cpp
// Based on previous top-K implementation, fixes truncation/uniqueness issues and adds reuse-limit.
// Build same as before: g++ -std=c++17 -O3 ... -o lfsr_pwm_algebraic_topk_fixed lfsr_pwm_algebraic_topk_fixed.cpp

#include <bits/stdc++.h>
#include <atomic>
#include <thread>
#include <mutex>
#include <getopt.h>
#include <random>
#include <chrono>
#include <unordered_map>

using u64 = uint64_t;
using u32 = uint32_t;

struct Candidate {
    double err = std::numeric_limits<double>::infinity();
    int lc = -1;
    u64 mask = 0;
    int period = 0;
    double ratio = 0.0;
    std::vector<uint8_t> sample_bits;
};

static inline int popcount_u64(u64 x) { return __builtin_popcountll(x); }

int berlekamp_massey(const std::vector<uint8_t> &bits) {
    int n = (int)bits.size();
    if (n == 0) return 0;
    std::vector<uint8_t> C(n, 0), B(n, 0);
    C[0] = 1; B[0] = 1;
    int L = 0, m = -1;
    for (int N = 0; N < n; ++N) {
        int d = 0;
        for (int i = 0; i <= L; ++i) d ^= (C[i] & bits[N - i]);
        if (d) {
            std::vector<uint8_t> T = C;
            int p = N - m;
            for (int i = 0; i + p < n; ++i) C[i + p] ^= B[i];
            if (L <= N / 2) {
                L = N + 1 - L;
                m = N;
                B.swap(T);
            }
        }
    }
    return L;
}

static inline u64 next_state(u64 state, u64 mask, int n) {
    u64 feedback = __builtin_parityll(state & mask);
    return (state >> 1) | (feedback << (n - 1));
}

void decompose_cycles_for_mask(u64 mask, int n,
                              std::vector<std::vector<uint8_t>> &out_bits,
                              std::vector<int> &out_periods) {
    const u64 max_state = (1ULL << n);
    std::vector<uint8_t> visited(max_state, 0);
    std::vector<int> stamp(max_state, 0);
    std::vector<int> pos(max_state, -1);
    int traversal_id = 1;
    std::vector<u64> seq; seq.reserve(1024);

    for (u64 s = 1; s < max_state; ++s) {
        if (visited[s]) continue;
        u64 curr = s;
        seq.clear();
        while (true) {
            if (visited[curr]) { for (u64 node : seq) visited[node] = 1; break; }
            if (stamp[curr] == traversal_id) {
                int start_idx = pos[curr];
                int period = (int)seq.size() - start_idx;
                if (period > 0) {
                    std::vector<uint8_t> bits; bits.reserve(period);
                    for (int i = start_idx; i < (int)seq.size(); ++i) bits.push_back((uint8_t)(seq[i] & 1ULL));
                    out_bits.push_back(std::move(bits));
                    out_periods.push_back(period);
                }
                for (u64 node : seq) visited[node] = 1;
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

// push into top-K sorted vector (ascending by err, tie-breaker: lc desc, period desc)
void push_topk(std::vector<Candidate> &vec, const Candidate &cand, size_t K) {
    auto cmp = [](const Candidate &a, const Candidate &b) {
        const double eps = 1e-15;
        if (a.err + eps < b.err) return true;
        if (b.err + eps < a.err) return false;
        if (a.lc != b.lc) return a.lc > b.lc;
        return a.period > b.period;
    };
    auto it = std::lower_bound(vec.begin(), vec.end(), cand, [&](const Candidate &x, const Candidate &y){ return cmp(x,y); });
    vec.insert(it, cand);
    if (vec.size() > K) vec.resize(K);
}

// evaluate mask exactly and produce candidates per level (invokes decomposition + BM)
void evaluate_mask_exact_into_topk(u64 mask, int n, int resolution,
                                   std::vector<Candidate> &best_out,
                                   std::vector<std::vector<Candidate>> &alts_out,
                                   size_t topk)
{
    std::vector<std::vector<uint8_t>> cycles_bits;
    std::vector<int> cycles_periods;
    decompose_cycles_for_mask(mask, n, cycles_bits, cycles_periods);
    const double eps = 1e-15;
    std::vector<double> targets(resolution);
    for (int i = 0; i < resolution; ++i) targets[i] = double(i) / double(resolution - 1);

    for (size_t ci = 0; ci < cycles_bits.size(); ++ci) {
        const auto &bits = cycles_bits[ci];
        int period = cycles_periods[ci];
        if (period < 1) continue;
        int ones = 0;
        for (int b: bits) ones += b;
        double ratio = double(ones) / double(period);
        int lc = berlekamp_massey(bits);

        Candidate cand_base;
        cand_base.lc = lc;
        cand_base.mask = mask;
        cand_base.period = period;
        cand_base.ratio = ratio;
        if (period <= 256) cand_base.sample_bits = bits;
        else cand_base.sample_bits.assign(bits.begin(), bits.begin() + 256);

        for (int pwm = 0; pwm < resolution; ++pwm) {
            double err = fabs(ratio - targets[pwm]);
            Candidate cand = cand_base;
            cand.err = err;

            // best_out
            Candidate &cur = best_out[pwm];
            bool better_err = (err + 1e-15 < cur.err);
            bool tie_better = (fabs(err - cur.err) < 1e-15 && (cand.lc > cur.lc || (cand.lc == cur.lc && cand.period > cur.period)));
            if (better_err || tie_better) cur = cand;

            // alts
            push_topk(alts_out[pwm], cand, topk);
        }
    }
}

// Write C LUT (full mask, period) - unchanged formatting semantics
void write_c_lut(const std::vector<Candidate> &candidates, int resolution, const std::string &varname, const std::string &filename) {
    std::ofstream f(filename);
    if (!f) { std::cerr << "ERROR: cannot open " << filename << " for writing\n"; return; }
    f << "#include <stdint.h>\n\n";
    f << "// Format: { mask, period }\n";
    f << "// Generated by lfsr_pwm_algebraic_topk_fixed.cpp\n";
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

// Write Arduino LUT: store truncated masks (trunc_bits) as uint16_t by default
void write_arduino_lut_trunc(const std::vector<Candidate> &candidates, int resolution, int n_bits, int trunc_bits, const std::string &filename) {
    std::ofstream f(filename);
    if (!f) { std::cerr << "ERROR: cannot open " << filename << " for writing\n"; return; }
    f << "// lfsr_pwm_algebraic_topk_fixed.cpp\n";
    f << "// Generated Arduino LUT (truncated to " << trunc_bits << " bits)\n";
    f << "#include <stdint.h>\n";
    f << "#if defined(__AVR__)\n#include <avr/pgmspace.h>\n#else\n#define PROGMEM\n#define pgm_read_word(addr) (*(addr))\n#endif\n\n";

    bool fits16 = (trunc_bits <= 16);
    if (!fits16) {
        f << "// trunc_bits > 16, storing lower 16 bits only (user requested trunc_bits=" << trunc_bits << ")\n";
    }

    f << "const uint16_t lfsr_pwm_masks_16bit[] PROGMEM = {\n";
    uint16_t all_ones = (trunc_bits >= 16 ? 0xFFFFu : (uint16_t)(((1ULL<<trunc_bits)-1ULL) & 0xFFFFu));
    uint16_t trivial_off = 0x0000u;
    uint16_t placeholder = 0x0001u;

    for (int i = 0; i < resolution; ++i) {
        const Candidate &c = candidates[i];
        uint16_t mask16;
        std::string comment;
        if (i == 0) {
            if (c.err != std::numeric_limits<double>::infinity()) mask16 = (uint16_t)(c.mask & ((1ULL<<trunc_bits)-1));
            else mask16 = placeholder;
            comment = "Level 0 (0% duty)";
        } else if (i == resolution-1) {
            if (c.err != std::numeric_limits<double>::infinity()) mask16 = (uint16_t)(c.mask & ((1ULL<<trunc_bits)-1));
            else mask16 = all_ones;
            comment = "Level max (100% duty)";
        } else {
            if (c.err == std::numeric_limits<double>::infinity()) {
                mask16 = all_ones;
                comment = "Unmatched";
            } else {
                mask16 = (uint16_t)(c.mask & ((1ULL<<trunc_bits)-1));
                std::ostringstream oss;
                oss << "Ratio " << std::fixed << std::setprecision(6) << c.ratio
                    << ", err " << std::scientific << std::setprecision(6) << c.err
                    << ", LC " << std::dec << c.lc;
                comment = oss.str();
            }
        }
        f << " 0x" << std::hex << std::setw(4) << std::setfill('0') << std::nouppercase << mask16 << "u, // " << comment << "\n" << std::dec;
    }
    f << "};\n";
    f.close();
}

struct Options {
    int n = 16;
    u64 scan = (1ULL << 16);
    int res = 256;
    std::string out = "lfsr_pwm.c";
    std::string arduino_out;
    int threads = 0;
    bool treat_extremes_const = false;
    int iterations = 1;
    unsigned seed = 0;
    size_t topk = 8;
    // uniqueness/truncation options
    int trunc_bits = -1; // default: min(n,16)
    int reuse_limit = 1; // allow each truncated mask to be used this many times
};

Options parse_args(int argc, char **argv) {
    Options opt;
    static struct option long_options[] = {
        {"n", required_argument, nullptr, 'n'},
        {"scan", required_argument, nullptr, 's'},
        {"res", required_argument, nullptr, 'r'},
        {"out", required_argument, nullptr, 'o'},
        {"arduino-out", required_argument, nullptr, 'a'},
        {"threads", required_argument, nullptr, 't'},
        {"treat-extremes-const", no_argument, nullptr, 'e'},
        {"iters", required_argument, nullptr, 'i'},
        {"seed", required_argument, nullptr, 'S'},
        {"topk", required_argument, nullptr, 'k'},
        {"trunc-bits", required_argument, nullptr, 1001},
        {"reuse-limit", required_argument, nullptr, 1002},
        {nullptr,0,nullptr,0}
    };
    int optc;
    while ((optc = getopt_long(argc, argv, "n:s:r:o:a:t:ei:S:k:", long_options, nullptr)) != -1) {
        switch (optc) {
            case 'n': opt.n = std::atoi(optarg); break;
            case 's': opt.scan = std::stoull(optarg); break;
            case 'r': opt.res = std::atoi(optarg); break;
            case 'o': opt.out = std::string(optarg); break;
            case 'a': opt.arduino_out = std::string(optarg); break;
            case 't': opt.threads = std::atoi(optarg); break;
            case 'e': opt.treat_extremes_const = true; break;
            case 'i': opt.iterations = std::atoi(optarg); break;
            case 'S': opt.seed = (unsigned)std::stoul(optarg); break;
            case 'k': opt.topk = (size_t)std::stoul(optarg); break;
            case 1001: opt.trunc_bits = std::atoi(optarg); break;
            case 1002: opt.reuse_limit = std::atoi(optarg); break;
            default: break;
        }
    }
    if (opt.seed == 0) opt.seed = (unsigned)std::chrono::system_clock::now().time_since_epoch().count();
    if (opt.topk < 1) opt.topk = 1;
    if (opt.trunc_bits <= 0) opt.trunc_bits = std::min(opt.n, 16);
    if (opt.reuse_limit < 1) opt.reuse_limit = 1;
    return opt;
}

int main(int argc, char **argv) {
    Options opt = parse_args(argc, argv);

    if (opt.scan <= 1) { std::cerr << "scan must be > 1\n"; return 1; }
    if (opt.n <= 0 || opt.n > 31) { std::cerr << "n must be between 1 and 31\n"; return 1; }
    if (opt.iterations < 1) opt.iterations = 1;

    u64 max_state_space = (1ULL << opt.n);
    u64 max_masks = std::min(opt.scan, max_state_space);
    std::cerr << "Starting: n=" << opt.n << " res=" << opt.res << " masks<= " << max_masks
              << " iters=" << opt.iterations << " topK=" << opt.topk
              << " trunc_bits=" << opt.trunc_bits << " reuse_limit=" << opt.reuse_limit << "\n";

    int n_threads = opt.threads <= 0 ? std::max(1u, std::thread::hardware_concurrency()) : opt.threads;

    // build masks vector
    std::vector<u64> masks;
    masks.reserve((size_t)std::max<u64>(0, max_masks - 1));
    for (u64 m = 1; m < max_masks; ++m) masks.push_back(m);
    size_t total_masks = masks.size();
    if (total_masks == 0) {
        std::vector<Candidate> empty(opt.res);
        write_c_lut(empty, opt.res, "lfsr_pwm_config", opt.out);
        if (!opt.arduino_out.empty()) write_arduino_lut_trunc(empty, opt.res, opt.n, opt.trunc_bits, opt.arduino_out);
        return 0;
    }

    // split into ranges of indices so shuffling is safe
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

    // master containers
    std::vector<Candidate> master_best(opt.res);
    std::vector<std::vector<Candidate>> master_alts(opt.res);

    std::mutex merge_mutex;

    for (int iter = 0; iter < opt.iterations; ++iter) {
        unsigned iter_seed = opt.seed + (unsigned)iter * 0x9e3779b9u;
        std::mt19937_64 rng_shuffle(iter_seed);
        std::shuffle(masks.begin(), masks.end(), rng_shuffle);
        std::cerr << "Iteration " << (iter+1) << "/" << opt.iterations << " seed=" << iter_seed << "\n";

        std::atomic<size_t> next_range_idx{0};
        auto worker_fn = [&](int worker_id) {
            std::mt19937_64 rng_local(iter_seed ^ ((uint64_t)(worker_id+1) * 0x9e3779b97f4a7c15ULL));
            std::vector<Candidate> local_best(opt.res);
            std::vector<std::vector<Candidate>> local_alts(opt.res);

            while (true) {
                size_t idx = next_range_idx.fetch_add(1);
                if (idx >= ranges.size()) break;
                auto [start_idx, end_idx] = ranges[idx];
                for (size_t mi = start_idx; mi < end_idx; ++mi) {
                    u64 mask = masks[mi];
                    if (mask == 0) continue;
                    // exact evaluation (we keep this simpler: you can add sampling/culling if you want)
                    std::vector<Candidate> best_local(opt.res);
                    std::vector<std::vector<Candidate>> alts_local(opt.res);
                    for (int k=0;k<opt.res;++k) alts_local[k].reserve(4);
                    evaluate_mask_exact_into_topk(mask, opt.n, opt.res, best_local, alts_local, opt.topk);

                    // merge into local
                    for (int lvl = 0; lvl < opt.res; ++lvl) {
                        const Candidate &c = best_local[lvl];
                        if (c.err != std::numeric_limits<double>::infinity()) {
                            Candidate &cur = local_best[lvl];
                            const double eps = 1e-15;
                            if (c.err + eps < cur.err) cur = c;
                            else if (fabs(c.err - cur.err) < eps) {
                                if (c.lc > cur.lc || (c.lc == cur.lc && c.period > cur.period)) cur = c;
                            }
                        }
                        for (const Candidate &alt : alts_local[lvl]) push_topk(local_alts[lvl], alt, opt.topk);
                    }
                }
            }

            // merge local into master
            std::lock_guard<std::mutex> lk(merge_mutex);
            for (int lvl = 0; lvl < opt.res; ++lvl) {
                const Candidate &c = local_best[lvl];
                if (c.err != std::numeric_limits<double>::infinity()) {
                    Candidate &gcur = master_best[lvl];
                    const double eps = 1e-15;
                    if (c.err + eps < gcur.err) gcur = c;
                    else if (fabs(c.err - gcur.err) < eps) {
                        if (c.lc > gcur.lc || (c.lc == gcur.lc && c.period > gcur.period)) gcur = c;
                    }
                }
                for (const Candidate &alt : local_alts[lvl]) push_topk(master_alts[lvl], alt, opt.topk);
            }
        };

        std::vector<std::thread> workers;
        for (size_t t = 0; t < threads_to_use; ++t) workers.emplace_back(worker_fn, (int)t);
        for (auto &th : workers) th.join();
        std::cerr << "Iteration " << (iter+1) << " merged\n";
    }

    // ensure master_best is included in alts
    for (int lvl=0; lvl < opt.res; ++lvl) {
        if (master_best[lvl].err != std::numeric_limits<double>::infinity()) push_topk(master_alts[lvl], master_best[lvl], opt.topk);
    }

    // Final uniqueness-aware assignment using truncated masks and reuse_limit
    int trunc = std::min(opt.trunc_bits, opt.n);
    u64 trunc_mask = (trunc >= 64 ? ~0ULL : ((1ULL << trunc) - 1ULL));
    std::unordered_map<u32,int> usage_count; usage_count.reserve(opt.res * 2);

    // order levels by confidence (smallest best err first)
    std::vector<int> order(opt.res);
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b){
        return master_best[a].err < master_best[b].err;
    });

    std::vector<Candidate> final_lut(opt.res);

    for (int lvl : order) {
        Candidate chosen;
        bool assigned = false;

        // try to pick from alternatives preferring truncated masks under reuse_limit
        for (const Candidate &cand : master_alts[lvl]) {
            u32 tmask = (u32)(cand.mask & trunc_mask);
            int used = usage_count[tmask];
            if (used < opt.reuse_limit) {
                chosen = cand;
                usage_count[tmask] = used + 1;
                assigned = true;
                break;
            }
        }

        // if none under reuse_limit, pick best alt that minimally worsens error (fallback)
        if (!assigned) {
            // pick candidate with minimal err + small penalty for overused truncated mask
            double best_score = 1e300;
            int idx_best = -1;
            for (size_t j = 0; j < master_alts[lvl].size(); ++j) {
                const Candidate &cand = master_alts[lvl][j];
                u32 tmask = (u32)(cand.mask & trunc_mask);
                int used = usage_count[tmask];
                double penalty = 0.001 * used; // small penalty per existing usage (tunable)
                double score = cand.err + penalty;
                if (score < best_score) { best_score = score; idx_best = (int)j; }
            }
            if (idx_best >= 0) {
                chosen = master_alts[lvl][idx_best];
                u32 tmask = (u32)(chosen.mask & trunc_mask);
                usage_count[tmask] = usage_count[tmask] + 1;
                assigned = true;
            }
        }

        if (!assigned) {
            // nothing discovered for this level -> leave as unmatched (infinite)
            final_lut[lvl] = Candidate();
        } else {
            final_lut[lvl] = chosen;
        }
    }

    // Optionally force extremes (override uniqueness)
    if (opt.treat_extremes_const) {
        final_lut[0].mask = 0x00000000u;
        final_lut[0].period = 1;
        final_lut[0].ratio = 0.0;
        final_lut[0].err = 0.0;
        final_lut[0].lc = 0;
        u64 all_ones = (opt.n >= 64 ? 0xffffffffffffffffULL : ((1ULL << opt.n) - 1ULL));
        final_lut[opt.res - 1].mask = all_ones;
        final_lut[opt.res - 1].period = 1;
        final_lut[opt.res - 1].ratio = 1.0;
        final_lut[opt.res - 1].err = 0.0;
        final_lut[opt.res - 1].lc = 0;
    }

    // Stats
    size_t found = 0;
    for (const auto &c : final_lut) if (c.err != std::numeric_limits<double>::infinity()) ++found;
    size_t unique_trunc = usage_count.size();
    std::cerr << "Final coverage: " << found << "/" << opt.res << ", unique truncated masks used: " << unique_trunc << "\n";

    write_c_lut(final_lut, opt.res, "lfsr_pwm_config", opt.out);
    std::cerr << "Written C LUT to " << opt.out << "\n";
    if (!opt.arduino_out.empty()) {
        write_arduino_lut_trunc(final_lut, opt.res, opt.n, opt.trunc_bits, opt.arduino_out);
        std::cerr << "Written Arduino LUT (truncated to " << opt.trunc_bits << " bits) to " << opt.arduino_out << "\n";
    }

    return 0;
}
