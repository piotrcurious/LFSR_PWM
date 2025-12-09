// lfsr_pwm_algebraic_topk.cpp
// Adds top-K alternatives per PWM level and a uniqueness-aware postprocessing
// Build: g++ -std=c++17 -O3 -march=znver4 -mtune=znver4 -flto -funroll-loops -fno-exceptions -fomit-frame-pointer -o lfsr_pwm_algebraic_topk lfsr_pwm_algebraic_topk.cpp

#include <bits/stdc++.h>
#include <atomic>
#include <thread>
#include <mutex>
#include <getopt.h>
#include <random>
#include <chrono>

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

// bucket produced by a worker: best candidate per level + alternatives per level
struct Bucket {
    std::vector<Candidate> best; // size = resolution, best per level
    std::vector<std::vector<Candidate>> alts; // size = resolution, top-K alternatives
};

std::vector<Candidate> init_bucket_best(int resolution) {
    return std::vector<Candidate>(resolution, Candidate());
}

// helper to insert candidate into sorted top-K vector (ascending by err, tie-break as original)
void push_topk(std::vector<Candidate> &vec, const Candidate &cand, size_t K) {
    // Insert maintaining ascending err; on tie prefer higher LC then larger period
    auto cmp = [](const Candidate &a, const Candidate &b) {
        const double eps = 1e-15;
        if (a.err + eps < b.err) return true;
        if (b.err + eps < a.err) return false;
        if (a.lc != b.lc) return a.lc > b.lc;
        return a.period > b.period;
    };
    // find insertion position
    auto it = std::lower_bound(vec.begin(), vec.end(), cand, [&](const Candidate &x, const Candidate &y) { return cmp(x,y); });
    vec.insert(it, cand);
    if (vec.size() > K) vec.resize(K);
}

void process_mask_range(const std::vector<u64> &masks, size_t start_idx, size_t end_idx, int n, int resolution,
                        size_t topk, Bucket &out_bucket,
                        std::atomic<size_t> *progress_ptr = nullptr)
{
    const double eps = 1e-15;
    std::vector<double> targets(resolution);
    for (int i = 0; i < resolution; ++i) targets[i] = double(i) / double(resolution - 1);

    out_bucket.best = init_bucket_best(resolution);
    out_bucket.alts.assign(resolution, std::vector<Candidate>());

    std::vector<std::vector<uint8_t>> cycles_bits;
    std::vector<int> cycles_periods;
    cycles_bits.reserve(256);
    cycles_periods.reserve(256);

    for (size_t mi = start_idx; mi < end_idx; ++mi) {
        u64 mask = masks[mi];
        if (mask == 0) continue;
        cycles_bits.clear();
        cycles_periods.clear();
        decompose_cycles_for_mask(mask, n, cycles_bits, cycles_periods);

        for (size_t ci = 0; ci < cycles_bits.size(); ++ci) {
            const std::vector<uint8_t> &bits = cycles_bits[ci];
            int period = cycles_periods[ci];
            if (period < 1) continue;
            int ones = 0;
            for (int b : bits) ones += b;
            double ratio = double(ones) / double(period);

            for (int pwm = 0; pwm < resolution; ++pwm) {
                double target = targets[pwm];
                double err = fabs(ratio - target);
                Candidate &cur = out_bucket.best[pwm];

                bool better_err = (err + eps < cur.err);
                bool same_err_better_period = (fabs(err - cur.err) < eps && period > cur.period);
                if (!better_err && !same_err_better_period) {
                    // still push into alts if it's reasonably close (heuristic): keep alternatives that are within, say, 3x best err or top-K
                    // But since we don't know global best yet, we still compute LC and push into local alts.
                }

                // compute LC only for promising candidate or to store alt (we keep it)
                Candidate cand;
                cand.err = err;
                cand.mask = mask;
                cand.period = period;
                cand.ratio = ratio;
                cand.lc = berlekamp_massey(bits);
                if (period <= 256) cand.sample_bits = bits;
                else cand.sample_bits.assign(bits.begin(), bits.begin() + 256);

                // try to update best
                bool accept_best = false;
                if (better_err) accept_best = true;
                else if (fabs(err - cur.err) < eps) {
                    if (cand.lc > cur.lc) accept_best = true;
                    else if (cand.lc == cur.lc && period > cur.period) accept_best = true;
                }
                if (accept_best) out_bucket.best[pwm] = cand;

                // push to local alts
                push_topk(out_bucket.alts[pwm], cand, topk);
            }
        }

        if (progress_ptr && (++(*progress_ptr) % 1000 == 0)) {
            // caller can report progress
        }
    }
}

// merge a worker bucket into master: merge best and merge top-K alternatives
void merge_bucket_into_master(std::vector<Candidate> &master_best,
                              std::vector<std::vector<Candidate>> &master_alts,
                              const Bucket &chunk,
                              size_t topk)
{
    const double eps = 1e-15;
    int resolution = (int)master_best.size();
    for (int i = 0; i < resolution; ++i) {
        const Candidate &e = chunk.best[i];
        if (e.err != std::numeric_limits<double>::infinity()) {
            Candidate &m = master_best[i];
            if (e.err + eps < m.err) m = e;
            else if (fabs(e.err - m.err) < eps) {
                if (e.lc > m.lc || (e.lc == m.lc && e.period > m.period)) m = e;
            }
        }
        // merge alternatives: append then keep top-K
        auto &vec = master_alts[i];
        for (const Candidate &c : chunk.alts[i]) {
            push_topk(vec, c, topk);
        }
    }
}

// Write LUTs (unchanged semantics); trimmed formatting for brevity but equivalent to original
void write_c_lut(const std::vector<Candidate> &candidates, int resolution, const std::string &varname, const std::string &filename) {
    std::ofstream f(filename);
    if (!f) { std::cerr << "ERROR: cannot open " << filename << " for writing\n"; return; }
    f << "#include <stdint.h>\n\n";
    f << "// Format: { mask, period }\n";
    f << "// Generated by lfsr_pwm_algebraic_topk.cpp\n";
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
    f << "// lfsr_pwm_algebraic_topk.cpp\n";
    f << "// Generated by lfsr_pwm_algebraic_topk.cpp\n";
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
    std::cerr << "Starting: n=" << opt.n << " res=" << opt.res << " masks(per iter)<=" << max_masks
              << " iters=" << opt.iterations << " topK=" << opt.topk << " seed=" << opt.seed << "\n";

    int n_threads = opt.threads <= 0 ? std::max(1u, std::thread::hardware_concurrency()) : opt.threads;

    std::vector<u64> masks;
    masks.reserve((size_t)std::max<u64>(0, max_masks - 1));
    for (u64 m = 1; m < max_masks; ++m) masks.push_back(m);
    size_t total_masks = masks.size();
    if (total_masks == 0) {
        auto empty_bucket = init_bucket_best(opt.res);
        write_c_lut(empty_bucket, opt.res, "lfsr_pwm_config", opt.out);
        if (!opt.arduino_out.empty()) write_arduino_lut(empty_bucket, opt.res, opt.n, opt.arduino_out);
        return 0;
    }

    // ranges over indices (so safe to shuffle masks)
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

    // master containers: best + top-K alternatives
    std::vector<Candidate> master_best = init_bucket_best(opt.res);
    std::vector<std::vector<Candidate>> master_alts(opt.res);

    std::mutex merge_mutex;

    for (int iter = 0; iter < opt.iterations; ++iter) {
        unsigned iter_seed = opt.seed + (unsigned)iter * 0x9e3779b9u;
        std::mt19937_64 rng(iter_seed);
        std::shuffle(masks.begin(), masks.end(), rng);
        std::cerr << "Iteration " << (iter+1) << "/" << opt.iterations << " (seed=" << iter_seed << ")\n";

        std::atomic<size_t> next_range_idx{0};
        std::vector<std::thread> workers; workers.reserve(threads_to_use);

        auto worker_fn = [&](int worker_id) {
            while (true) {
                size_t idx = next_range_idx.fetch_add(1);
                if (idx >= ranges.size()) break;
                auto [start_idx, end_idx] = ranges[idx];
                Bucket chunk;
                std::atomic<size_t> progress{0};
                process_mask_range(masks, start_idx, end_idx, opt.n, opt.res, opt.topk, chunk, &progress);
                {
                    std::lock_guard<std::mutex> lk(merge_mutex);
                    merge_bucket_into_master(master_best, master_alts, chunk, opt.topk);
                    std::cerr << "Iter " << (iter+1) << " merged range " << (idx+1) << "/" << ranges.size()
                              << " indices " << start_idx << "-" << (end_idx-1) << "\n";
                }
            }
        };

        for (size_t t = 0; t < threads_to_use; ++t) workers.emplace_back(worker_fn, (int)t);
        for (auto &th : workers) th.join();
        std::cerr << "Iteration " << (iter+1) << " done.\n";
    }

    // At this point master_best holds best per level, master_alts holds top-K alternatives (ascending by error)
    // Post-process: greedy unique assignment using master_alts first, fallback to master_best
    std::vector<Candidate> final_lut(opt.res);
    std::unordered_set<u64> used_masks;
    used_masks.reserve(opt.res * 2);

    // Construct for each level a sorted list of candidates to try: start with master_best then its alts (sorted)
    for (int lvl = 0; lvl < opt.res; ++lvl) {
        // merge master_best[lvl] into master_alts[lvl] list (keeping topK)
        if (master_best[lvl].err != std::numeric_limits<double>::infinity()) {
            push_topk(master_alts[lvl], master_best[lvl], opt.topk);
        }
    }

    // Greedy: for each level, pick first candidate whose mask is unused
    // Ordering of levels: attempt to assign from lowest error to highest may be better to preserve quality:
    // Build order indices sorted by best error ascending to assign most confident levels first.
    std::vector<int> order(opt.res);
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        return master_best[a].err < master_best[b].err;
    });

    for (int idx : order) {
        Candidate chosen;
        bool found = false;
        // try alts first
        for (const Candidate &c : master_alts[idx]) {
            if (used_masks.find(c.mask) == used_masks.end()) {
                chosen = c;
                found = true;
                break;
            }
        }
        if (!found) {
            // fallback to best if it's available or simply the best alt (could be duplicate)
            if (master_best[idx].err != std::numeric_limits<double>::infinity() &&
                used_masks.find(master_best[idx].mask) == used_masks.end()) {
                chosen = master_best[idx];
                found = true;
            } else if (!master_alts[idx].empty()) {
                chosen = master_alts[idx][0]; // may duplicate
                found = true;
            } else {
                // leave unmatched (infinite) - keep empty candidate
                chosen = Candidate();
            }
        }
        if (chosen.err != std::numeric_limits<double>::infinity()) {
            used_masks.insert(chosen.mask);
            final_lut[idx] = chosen;
        } else {
            final_lut[idx] = Candidate();
        }
    }

    // Optionally force extremes (overrides uniqueness)
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
    std::cerr << "Final coverage after uniqueness assignment: " << found << "/" << opt.res << " unique masks used: " << used_masks.size() << "\n";

    write_c_lut(final_lut, opt.res, "lfsr_pwm_config", opt.out);
    std::cerr << "Wrote C LUT to " << opt.out << "\n";
    if (!opt.arduino_out.empty()) {
        write_arduino_lut(final_lut, opt.res, opt.n, opt.arduino_out);
        std::cerr << "Wrote Arduino LUT to " << opt.arduino_out << "\n";
    }

    return 0;
}
