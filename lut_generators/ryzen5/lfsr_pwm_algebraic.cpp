// lfsr_pwm_algebraic_search.cpp
// Improved search: sampling + culling + local hill-climb (simulated annealing) + top-K alternatives
// Build: g++ -std=c++17 -O3 -march=znver4 -mtune=znver4 -flto -funroll-loops -fno-exceptions -fomit-frame-pointer -o lfsr_pwm_algebraic_search lfsr_pwm_algebraic_search.cpp

#include <bits/stdc++.h>
#include <atomic>
#include <thread>
#include <mutex>
#include <getopt.h>
#include <random>
#include <chrono>
#include <unordered_set>

using u64 = uint64_t;
using u32 = uint32_t;

struct Candidate {
    double err = std::numeric_limits<double>::infinity();
    int lc = -1;
    u64 mask = 0;
    int period = 0;
    double ratio = 0.0;
    std::vector<uint8_t> sample_bits; // truncated sample for debugging
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

// full decomposition, unchanged
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

// quick sampling estimator: run `seed_count` short walks of length `walk_len` from random non-zero states
// returns estimated ones-ratio (double in [0,1])
double estimate_mask_ratio_fast(u64 mask, int n, size_t seed_count, size_t walk_len, std::mt19937_64 &rng) {
    if (mask == 0) return 0.0;
    const u64 max_state = (1ULL << n);
    std::uniform_int_distribution<u64> dist(1, max_state - 1);
    uint64_t ones = 0;
    uint64_t total = 0;
    for (size_t s = 0; s < seed_count; ++s) {
        u64 st = dist(rng);
        for (size_t t = 0; t < walk_len; ++t) {
            ones += (st & 1ULL);
            ++total;
            st = next_state(st, mask, n);
        }
    }
    if (total == 0) return 0.0;
    return double(ones) / double(total);
}

// small helper to compute exact candidates for a mask (calls decompose and BM) — used only after passing sampling filter
void evaluate_mask_exact(u64 mask, int n, int resolution, std::vector<Candidate> &out_best_per_level, std::vector<std::vector<Candidate>> &out_alts_per_level, size_t topk) {
    std::vector<std::vector<uint8_t>> cycles_bits;
    std::vector<int> cycles_periods;
    cycles_bits.reserve(256);
    cycles_periods.reserve(256);
    decompose_cycles_for_mask(mask, n, cycles_bits, cycles_periods);
    const double eps = 1e-15;
    std::vector<double> targets(resolution);
    for (int i = 0; i < resolution; ++i) targets[i] = double(i) / double(resolution - 1);

    for (size_t ci = 0; ci < cycles_bits.size(); ++ci) {
        const std::vector<uint8_t> &bits = cycles_bits[ci];
        int period = cycles_periods[ci];
        if (period < 1) continue;
        int ones = 0;
        for (int b : bits) ones += b;
        double ratio = double(ones) / double(period);
        int lc = berlekamp_massey(bits);
        Candidate cand;
        cand.err = 0.0; cand.lc = lc; cand.mask = mask; cand.period = period; cand.ratio = ratio;
        if (period <= 256) cand.sample_bits = bits;
        else cand.sample_bits.assign(bits.begin(), bits.begin() + 256);

        for (int pwm = 0; pwm < resolution; ++pwm) {
            double err = fabs(ratio - targets[pwm]);
            Candidate cur = out_best_per_level[pwm];
            bool better_err = (err + eps < cur.err);
            bool same_err_better_period = (fabs(err - cur.err) < eps && period > cur.period);
            if (better_err || same_err_better_period) {
                Candidate accepted = cand;
                accepted.err = err;
                out_best_per_level[pwm] = accepted;
            }
            // push to alts, keep topk using custom comparator
            auto &vec = out_alts_per_level[pwm];
            // insertion sort style (top-K ascending by err, tie-breaker lc desc, period desc)
            auto cmp = [&](const Candidate &a, const Candidate &b) {
                const double eps2 = 1e-15;
                if (a.err + eps2 < b.err) return true;
                if (b.err + eps2 < a.err) return false;
                if (a.lc != b.lc) return a.lc > b.lc;
                return a.period > b.period;
            };
            auto it = std::lower_bound(vec.begin(), vec.end(), cand, [&](const Candidate &x, const Candidate &y){ return cmp(x,y); });
            vec.insert(it, cand);
            if (vec.size() > topk) vec.resize(topk);
        }
    }
}

// small helper to insert into top-K vector (ascending)
void push_topk_local(std::vector<Candidate> &vec, const Candidate &cand, size_t K) {
    auto cmp = [](const Candidate &a, const Candidate &b){
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

// local refinement: simulated annealing style flips up to `max_iters` times starting from `mask0` with objective of minimizing |ratio - target|
// uses fast estimator for inner loop and exact evaluate_mask_exact only for final acceptance
u64 local_refine_mask(u64 mask0, int n, double target_ratio,
                      size_t sample_seed_count, size_t sample_walk_len,
                      size_t max_iters, double init_temp,
                      std::mt19937_64 &rng)
{
    u64 best_mask = mask0;
    double best_est_ratio = estimate_mask_ratio_fast(mask0, n, sample_seed_count, sample_walk_len, rng);
    double best_err = fabs(best_est_ratio - target_ratio);
    u64 cur_mask = mask0;
    double cur_est_ratio = best_est_ratio;
    double cur_err = best_err;
    std::uniform_int_distribution<int> bit_dist(0, n-1);
    std::uniform_real_distribution<double> uni(0.0, 1.0);

    for (size_t it = 0; it < max_iters; ++it) {
        // temperature schedule
        double T = init_temp * (1.0 - double(it) / double(max_iters));
        // propose: flip 1..k bits (k small, e.g., 1 or 2)
        int flips = 1 + (it % 2); // alternate 1 and 2 flips
        u64 cand_mask = cur_mask;
        for (int f = 0; f < flips; ++f) {
            int b = bit_dist(rng);
            cand_mask ^= (1ULL << b);
            // ensure mask nonzero (we want non-trivial masks)
            if (cand_mask == 0) cand_mask ^= 1ULL;
        }
        double est_ratio = estimate_mask_ratio_fast(cand_mask, n, sample_seed_count, sample_walk_len, rng);
        double est_err = fabs(est_ratio - target_ratio);
        bool accept = false;
        if (est_err + 1e-15 < cur_err) accept = true;
        else {
            // simulated annealing acceptance
            double p = exp((cur_err - est_err) / std::max(1e-12, T));
            if (uni(rng) < p) accept = true;
        }
        if (accept) {
            cur_mask = cand_mask;
            cur_est_ratio = est_ratio;
            cur_err = est_err;
            if (cur_err + 1e-15 < best_err) {
                best_err = cur_err;
                best_mask = cur_mask;
            }
        }
    }
    return best_mask;
}

// small helpers to write LUTs (same semantics as original)
void write_c_lut(const std::vector<Candidate> &candidates, int resolution, const std::string &varname, const std::string &filename) {
    std::ofstream f(filename);
    if (!f) { std::cerr << "ERROR: cannot open " << filename << " for writing\n"; return; }
    f << "#include <stdint.h>\n\n";
    f << "// Format: { mask, period }\n";
    f << "// Generated by lfsr_pwm_algebraic_search.cpp\n";
    f << "const struct { uint32_t mask; uint32_t period; } " << varname << "[" << resolution << "] = {\n";
    for (int i = 0; i < resolution; ++i) {
        const Candidate &c = candidates[i];
        if (c.err == std::numeric_limits<double>::infinity()) {
            f << "    { 0x00000000u, 0u }, // Level " << std::setw(3) << i << " (Unmatched)\n";
        } else {
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

void write_arduino_lut(const std::vector<Candidate> &candidates, int resolution, int n_bits, const std::string &filename) {
    std::ofstream f(filename);
    if (!f) { std::cerr << "ERROR: cannot open " << filename << " for writing\n"; return; }
    f << "// lfsr_pwm_algebraic_search.cpp\n";
    f << "// Generated by lfsr_pwm_algebraic_search.cpp\n";
    f << "// LFSR Bit Width (n) used for calculation: " << n_bits << " bits\n";
    f << "// PWM Resolution: " << resolution << " levels\n";
    f << "#include <stdint.h>\n";
    f << "#if defined(__AVR__)\n";
    f << "#include <avr/pgmspace.h>\n";
    f << "#else\n";
    f << "#define PROGMEM\n";
    f << "#define pgm_read_word(addr) (*(addr))\n";
    f << "#endif\n\n";

    f << "const uint16_t lfsr_pwm_masks_16bit[] PROGMEM = {\n";
    uint16_t all_ones_mask = 0xFFFFu;
    uint16_t trivial_off_mask = 0x0000u;
    uint16_t trivial_placeholder_mask = 0x0001u;

    for (int i = 0; i < resolution; ++i) {
        const Candidate &c = candidates[i];
        uint16_t mask_val;
        std::string comment;
        if (i == 0) {
            mask_val = (c.err == 0.0 && c.ratio == 0.0) ? trivial_off_mask : trivial_placeholder_mask;
            if (c.err != std::numeric_limits<double>::infinity()) mask_val = (uint16_t)c.mask;
            comment = "Level 0 (0% duty)";
        } else if (i == resolution - 1) {
            mask_val = all_ones_mask;
            if (c.err != std::numeric_limits<double>::infinity()) mask_val = (uint16_t)c.mask;
            comment = "Level max (100% duty)";
        } else {
            if (c.err == std::numeric_limits<double>::infinity()) {
                mask_val = all_ones_mask;
                comment = "Unmatched";
            } else {
                mask_val = (uint16_t)c.mask;
                std::ostringstream oss;
                oss << "Ratio " << std::fixed << std::setprecision(6) << c.ratio
                    << ", err " << std::scientific << std::setprecision(6) << c.err
                    << ", LC " << std::dec << c.lc;
                comment = oss.str();
            }
        }
        f << " 0x" << std::setw(4) << std::setfill('0') << std::hex << std::nouppercase << mask_val << "u, // " << comment << "\n";
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
    // sampling/refine options
    size_t sample_seed_count = 8;
    size_t sample_walk_len = 128;
    double cull_factor = 2.0; // allow masks up to cull_factor * best_est_err to go to exact eval
    size_t sa_iters = 200;
    double sa_temp = 0.05;
    bool allow_refine = true;
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
        {"sample-seeds", required_argument, nullptr, 1001},
        {"sample-walk", required_argument, nullptr, 1002},
        {"cull-factor", required_argument, nullptr, 1003},
        {"sa-iters", required_argument, nullptr, 1004},
        {"sa-temp", required_argument, nullptr, 1005},
        {"no-refine", no_argument, nullptr, 1006},
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
            case 1001: opt.sample_seed_count = std::stoul(optarg); break;
            case 1002: opt.sample_walk_len = std::stoul(optarg); break;
            case 1003: opt.cull_factor = std::stod(optarg); break;
            case 1004: opt.sa_iters = std::stoul(optarg); break;
            case 1005: opt.sa_temp = std::stod(optarg); break;
            case 1006: opt.allow_refine = false; break;
            default: break;
        }
    }
    if (opt.seed == 0) opt.seed = (unsigned)std::chrono::system_clock::now().time_since_epoch().count();
    if (opt.topk < 1) opt.topk = 1;
    return opt;
}

int main(int argc, char **argv) {
    Options opt = parse_args(argc, argv);

    if (opt.scan <= 1) { std::cerr << "scan must be > 1\n"; return 1; }
    if (opt.n <= 0 || opt.n > 31) { std::cerr << "n must be between 1 and 31\n"; return 1; }
    if (opt.iterations < 1) opt.iterations = 1;

    u64 max_state_space = (1ULL << opt.n);
    u64 max_masks = std::min(opt.scan, max_state_space);
    std::cerr << "Search: n=" << opt.n << " res=" << opt.res << " masks<= " << max_masks
              << " iters=" << opt.iterations << " topK=" << opt.topk << " seed=" << opt.seed << "\n";
    std::cerr << "Sampling: seeds=" << opt.sample_seed_count << " walk=" << opt.sample_walk_len << " cull=" << opt.cull_factor << "\n";
    std::cerr << "Refine: SA iters=" << opt.sa_iters << " temp=" << opt.sa_temp << " enabled=" << (opt.allow_refine ? "yes":"no") << "\n";

    int n_threads = opt.threads <= 0 ? std::max(1u, std::thread::hardware_concurrency()) : opt.threads;

    // masks list
    std::vector<u64> masks;
    masks.reserve((size_t)std::max<u64>(0, max_masks - 1));
    for (u64 m = 1; m < max_masks; ++m) masks.push_back(m);
    size_t total_masks = masks.size();
    if (total_masks == 0) {
        std::vector<Candidate> empty(opt.res);
        write_c_lut(empty, opt.res, "lfsr_pwm_config", opt.out);
        if (!opt.arduino_out.empty()) write_arduino_lut(empty, opt.res, opt.n, opt.arduino_out);
        return 0;
    }

    // ranges over indices
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
    std::cerr << "Threads: " << threads_to_use << " ranges: " << ranges.size() << "\n";

    // master containers
    std::vector<Candidate> master_best(opt.res);
    std::vector<std::vector<Candidate>> master_alts(opt.res);

    std::mutex merge_mutex;
    std::atomic<size_t> next_range_idx;
    next_range_idx = 0;

    // shared RNG seed base
    std::mt19937_64 base_rng(opt.seed);

    // per-iteration loop (shuffle masks each iter)
    for (int iter = 0; iter < opt.iterations; ++iter) {
        unsigned iter_seed = opt.seed + (unsigned)iter * 0x9e3779b9u;
        std::mt19937_64 rng_shuffle(iter_seed);
        std::shuffle(masks.begin(), masks.end(), rng_shuffle);
        std::cerr << "Iter " << (iter+1) << "/" << opt.iterations << " seed=" << iter_seed << " shuffled masks\n";

        // worker lambda
        std::atomic<size_t> global_progress{0};
        auto worker_fn = [&](int worker_id) {
            // thread-local RNG
            std::mt19937_64 rng_local(iter_seed ^ (0x9e3779b97f4a7c15ULL * (worker_id+1)));
            // local bests and alts (to reduce lock frequency)
            std::vector<Candidate> local_best(opt.res);
            std::vector<std::vector<Candidate>> local_alts(opt.res);

            // local estimation best errors to apply culling heuristics
            std::vector<double> local_best_est_err(opt.res, std::numeric_limits<double>::infinity());

            while (true) {
                size_t idx = next_range_idx.fetch_add(1);
                if (idx >= ranges.size()) break;
                auto [start_idx, end_idx] = ranges[idx];

                for (size_t mi = start_idx; mi < end_idx; ++mi) {
                    u64 mask = masks[mi];
                    if (mask == 0) continue;

                    // cheap estimate
                    double est_ratio = estimate_mask_ratio_fast(mask, opt.n, opt.sample_seed_count, opt.sample_walk_len, rng_local);

                    // evaluate estimated error per level quickly (without LC) to decide if we do exact
                    bool maybe_promising = false;
                    for (int pwm = 0; pwm < opt.res; ++pwm) {
                        double target = double(pwm) / double(opt.res - 1);
                        double est_err = fabs(est_ratio - target);
                        // if it's better than local best estimate* cull_factor, mark promising
                        if (est_err <= local_best_est_err[pwm] * opt.cull_factor || local_alts[pwm].size() < opt.topk) {
                            maybe_promising = true;
                        }
                        // update local_best_est_err quickly (keep min)
                        if (est_err < local_best_est_err[pwm]) local_best_est_err[pwm] = est_err;
                    }

                    if (!maybe_promising) {
                        ++global_progress;
                        continue; // skip heavy decomposition
                    }

                    // otherwise do exact evaluation (decompose cycles + BM)
                    std::vector<Candidate> best_per_level_local(opt.res);
                    std::vector<std::vector<Candidate>> alts_per_level_local(opt.res);
                    for (int k = 0; k < opt.res; ++k) alts_per_level_local[k].reserve(4);

                    evaluate_mask_exact(mask, opt.n, opt.res, best_per_level_local, alts_per_level_local, opt.topk);

                    // merge local results into thread-local structures
                    for (int pwm = 0; pwm < opt.res; ++pwm) {
                        const Candidate &c = best_per_level_local[pwm];
                        if (c.err != std::numeric_limits<double>::infinity()) {
                            Candidate &cur = local_best[pwm];
                            const double eps = 1e-15;
                            if (c.err + eps < cur.err) cur = c;
                            else if (fabs(c.err - cur.err) < eps) {
                                if (c.lc > cur.lc || (c.lc == cur.lc && c.period > cur.period)) cur = c;
                            }
                        }
                        // merge alts
                        for (const Candidate &alt : alts_per_level_local[pwm]) {
                            push_topk_local(local_alts[pwm], alt, opt.topk);
                        }
                    }
                    ++global_progress;
                }
            }

            // merge thread-local results into global master with a lock
            {
                std::lock_guard<std::mutex> lk(merge_mutex);
                for (int pwm = 0; pwm < opt.res; ++pwm) {
                    const Candidate &c = local_best[pwm];
                    if (c.err != std::numeric_limits<double>::infinity()) {
                        Candidate &gcur = master_best[pwm];
                        const double eps = 1e-15;
                        if (c.err + eps < gcur.err) gcur = c;
                        else if (fabs(c.err - gcur.err) < eps) {
                            if (c.lc > gcur.lc || (c.lc == gcur.lc && c.period > gcur.period)) gcur = c;
                        }
                    }
                    // merge alts
                    for (const Candidate &alt : local_alts[pwm]) {
                        push_topk_local(master_alts[pwm], alt, opt.topk);
                    }
                }
            }
        };

        // spawn threads
        next_range_idx = 0;
        std::vector<std::thread> workers;
        for (size_t t = 0; t < threads_to_use; ++t) workers.emplace_back(worker_fn, (int)t);
        for (auto &th : workers) th.join();
        std::cerr << "Iteration " << (iter+1) << " complete\n";
    }

    // Merge master_best into master_alts lists (so final assignment can pick them)
    for (int lvl = 0; lvl < opt.res; ++lvl) {
        if (master_best[lvl].err != std::numeric_limits<double>::infinity()) {
            push_topk_local(master_alts[lvl], master_best[lvl], opt.topk);
        }
    }

    // Post-process: uniqueness-aware greedy assignment using master_alts
    std::vector<Candidate> final_lut(opt.res);
    std::unordered_set<u64> used_masks;
    used_masks.reserve(opt.res * 2);

    // order levels by smallest best err first (conservative assignment)
    std::vector<int> order(opt.res);
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b){
        return master_best[a].err < master_best[b].err;
    });

    for (int idx_level : order) {
        Candidate chosen;
        bool found = false;
        for (const Candidate &cand : master_alts[idx_level]) {
            if (used_masks.find(cand.mask) == used_masks.end()) {
                chosen = cand; found = true; break;
            }
        }
        if (!found) {
            if (master_best[idx_level].err != std::numeric_limits<double>::infinity()
                && used_masks.find(master_best[idx_level].mask) == used_masks.end()) {
                chosen = master_best[idx_level];
                found = true;
            } else if (!master_alts[idx_level].empty()) {
                chosen = master_alts[idx_level][0];
                found = true;
            }
        }
        if (chosen.err != std::numeric_limits<double>::infinity()) {
            used_masks.insert(chosen.mask);
            final_lut[idx_level] = chosen;
        } else {
            final_lut[idx_level] = Candidate();
        }
    }

    // Optional refine per-level using simulated annealing-like local search (uses fast estimator to select best neighbor)
    if (opt.allow_refine) {
        std::mt19937_64 refine_rng(opt.seed ^ 0xdecafbad);
        for (int lvl = 0; lvl < opt.res; ++lvl) {
            if (final_lut[lvl].err == std::numeric_limits<double>::infinity()) continue;
            double target = double(lvl) / double(opt.res - 1);
            u64 seed_mask = final_lut[lvl].mask;
            u64 best_mask = local_refine_mask(seed_mask, opt.n, target,
                                             opt.sample_seed_count, opt.sample_walk_len,
                                             opt.sa_iters, opt.sa_temp, refine_rng);
            if (best_mask != seed_mask) {
                // evaluate exact for acceptance
                std::vector<Candidate> tmp_best(opt.res);
                std::vector<std::vector<Candidate>> tmp_alts(opt.res);
                for (int k=0;k<opt.res;++k) tmp_alts[k].reserve(4);
                evaluate_mask_exact(best_mask, opt.n, opt.res, tmp_best, tmp_alts, 1);
                // if tmp_best[lvl] offers improvement, accept
                const Candidate &cand = tmp_best[lvl];
                if (cand.err + 1e-15 < final_lut[lvl].err) {
                    final_lut[lvl] = cand;
                } else {
                    // no improvement in exact eval -> discard
                }
            }
        }
    }

    // Optionally force extremes
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

    size_t found = 0;
    for (const auto &c : final_lut) if (c.err != std::numeric_limits<double>::infinity()) ++found;
    std::cerr << "Final coverage: " << found << "/" << opt.res << " unique masks used: " << used_masks.size() << "\n";

    write_c_lut(final_lut, opt.res, "lfsr_pwm_config", opt.out);
    std::cerr << "Wrote C LUT to " << opt.out << "\n";
    if (!opt.arduino_out.empty()) {
        write_arduino_lut(final_lut, opt.res, opt.n, opt.arduino_out);
        std::cerr << "Wrote Arduino LUT to " << opt.arduino_out << "\n";
    }

    return 0;
}
