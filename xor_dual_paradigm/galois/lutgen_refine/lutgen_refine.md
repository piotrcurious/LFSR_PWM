Nice — understood. Below is an improved single-file LUT generator (lutgen_refine.cpp) that is directly compatible with the pairs binary format produced/consumed by the spectral-prune tool and the earlier LUT generator. It:

Uses the exact same mask cache binary layout ("MLUT" header, versioned) described previously.

Accepts a simple pairs binary input (raw u16 mask1, u16 mask2, mask1, mask2, ...) via --pairs pairs.bin.
If --pairs is provided, the program only evaluates pairs from that file (fast refinement loop). If not provided, it enumerates candidate pairs (from masks/seeds or from mask cache).

Supports a --regenerate mode that rebuilds the full mask cache (all 16-bit masks) and writes masks_cache.bin.

Produces a pwm_lut.h C header with a PWMLUTEntry[] array (same layout as before) containing the best pair for each duty target.

Optionally writes a pruned pairs binary (--write-pairs pruned_pairs.bin) containing only pairs that were kept/selected during evaluation (so the spectral-prune tool can operate on a smaller set next).

Multi-threaded, tuned for modern Ryzen (use -O3 -march=native), uses Galois LFSRs, XOR mixing, supports non-maximal masks, and includes an optional spectral-entropy scoring component.


Save as lutgen_refine.cpp, compile and run as shown below.


---

Build

g++ -O3 -march=native -std=c++17 -pthread -o lutgen_refine lutgen_refine.cpp


---

Quick usage examples

1. Regenerate the full mask cache (slow, heavy CPU):



./lutgen_refine --regenerate --cache masks_cache.bin --threads 12 --sample-seeds 64

2. Generate LUT from a precomputed pairs.bin (evaluate only those pairs), with 5% duty steps:



./lutgen_refine --pairs pairs.bin --cache masks_cache.bin --out pwm_lut.h --step 5 --samples 65536 --threads 12

3. Full run (no pairs input) — search candidate masks/seeds and produce LUT and pruned pairs:



./lutgen_refine --cache masks_cache.bin --out pwm_lut.h --write-pairs pruned_pairs.bin --step 5 --samples 65536 --threads 12


---

The code — lutgen_refine.cpp

// lutgen_refine.cpp
// Refined LUT generator compatible with simple pairs.bin (u16,u16) and masks_cache.bin ("MLUT")
// - Galois LFSR (16-bit), XOR mixing
// - If --pairs is provided: evaluate only those pairs (fast refinement)
// - Optionally regenerate full mask cache (--regenerate)
// - Multi-threaded
// - Outputs pwm_lut.h and optional pruned pairs binary for the next stage
//
// Build:
//   g++ -O3 -march=native -std=c++17 -pthread -o lutgen_refine lutgen_refine.cpp

#include <bits/stdc++.h>
using namespace std;
using u8  = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;
using u64 = uint64_t;
using cd  = complex<double>;

// -------------------- Cache layout (compatible) --------------------
static const char CACHE_MAGIC[4] = {'M','L','U','T'};
static const uint16_t CACHE_VERSION = 1;

#pragma pack(push,1)
struct MaskCacheRecord {
    u16 mask;             // 2
    u32 rep_period;       // 4
    u8  good_seed_pct;    // 1
    u8  is_maximal;       // 1
    u16 max_sampled_period;// 2
    u8  reserved[1];      // padding -> total 12 bytes
};
#pragma pack(pop)

// -------------------- simple pairs.bin format --------------------
// plain file with consecutive u16 mask1, u16 mask2 entries (no header)

// -------------------- Galois LFSR (16-bit) --------------------
static inline u16 galois_step(u16 state, u16 mask) {
    u16 out = state & 1u;
    state >>= 1;
    if (out) state ^= mask;
    return state;
}

// -------------------- utilities --------------------
uint64_t gcd_u64(uint64_t a, uint64_t b) { while (b) { uint64_t t = a % b; a = b; b = t; } return a; }
uint64_t lcm_u64(uint64_t a, uint64_t b) { if (a==0||b==0) return 0; return (a / gcd_u64(a,b)) * b; }
size_t next_pow2(size_t v) {
    if (v == 0) return 1;
    --v; v |= v >> 1; v |= v >> 2; v |= v >> 4; v |= v >> 8; v |= v >> 16;
    if (sizeof(size_t) > 4) v |= v >> 32;
    return ++v;
}

// small in-place radix-2 FFT (complex<double>), adequate for PSD
void fft(vector<cd>& a) {
    size_t n = a.size();
    for (size_t i = 1, j = 0; i < n; ++i) {
        size_t bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) swap(a[i], a[j]);
    }
    for (size_t len = 2; len <= n; len <<= 1) {
        double ang = 2.0 * M_PI / (double)len;
        cd wlen(cos(ang), sin(ang));
        for (size_t i = 0; i < n; i += len) {
            cd w(1.0, 0.0);
            for (size_t j = 0; j < len/2; ++j) {
                cd u = a[i+j];
                cd v = a[i+j+len/2] * w;
                a[i+j] = u + v;
                a[i+j+len/2] = u - v;
                w *= wlen;
            }
        }
    }
}

// spectral entropy normalized [0..1]
double spectral_entropy(const vector<double>& psd) {
    double tot = 0.0;
    for (double v : psd) tot += v;
    if (tot <= 0.0) return 0.0;
    double ent = 0.0;
    for (double v : psd) {
        double p = v / tot;
        if (p > 0.0) ent -= p * log(p);
    }
    double maxent = log((double)psd.size());
    if (maxent <= 0.0) return 0.0;
    return ent / maxent;
}

// -------------------- simulation & metrics --------------------
struct SimResult {
    double duty;         // fraction of ones
    uint32_t period;     // simulated period length
    uint32_t max_run;    // maximal run of consecutive ones
    double spec_entropy; // normalized spectral entropy (optional)
};

// simulate pair (advance both Galois LFSRs) up to maxSteps or LCM period
SimResult simulate_pair(u16 mask1, u16 seed1, u16 mask2, u16 seed2, uint32_t maxSteps, bool compute_spectral, uint32_t spectral_min_len) {
    // compute individual periods (cap at maxSteps)
    auto lfsr_period = [&](u16 seed, u16 mask)->uint32_t {
        if (seed == 0) return 1;
        u16 s = seed; uint32_t cnt = 0;
        do {
            s = galois_step(s, mask);
            ++cnt;
            if (cnt >= maxSteps) return cnt;
        } while (s != seed);
        return cnt;
    };
    uint32_t p1 = lfsr_period(seed1, mask1);
    uint32_t p2 = lfsr_period(seed2, mask2);
    uint64_t L = lcm_u64(p1, p2);
    uint32_t simLen = (uint32_t)min<uint64_t>(L == 0 ? maxSteps : L, maxSteps);
    if (simLen < 16) simLen = min<uint32_t>(maxSteps, 16);

    u16 s1 = seed1, s2 = seed2;
    uint32_t ones = 0;
    uint32_t max_run = 0, cur_run = 0;

    vector<uint8_t> bits;
    if (compute_spectral && simLen >= spectral_min_len) bits.reserve(simLen);

    for (uint32_t i = 0; i < simLen; ++i) {
        s1 = galois_step(s1, mask1);
        s2 = galois_step(s2, mask2);
        uint8_t out = (uint8_t)(((s1 ^ s2) & 1u) ? 1u : 0u);
        ones += out;
        if (out) { cur_run++; if (cur_run > max_run) max_run = cur_run; }
        else cur_run = 0;
        if (compute_spectral && simLen >= spectral_min_len) bits.push_back(out);
    }

    double duty = (double)ones / (double)simLen;
    double spec_ent = 0.0;
    if (compute_spectral && bits.size() >= spectral_min_len) {
        size_t N = next_pow2(bits.size());
        vector<cd> buf(N);
        for (size_t i = 0; i < bits.size(); ++i) buf[i] = cd((double)bits[i], 0.0);
        for (size_t i = bits.size(); i < N; ++i) buf[i] = cd(0.0, 0.0);
        fft(buf);
        size_t nbins = N/2;
        vector<double> psd(nbins);
        double tot = 0.0;
        for (size_t i = 0; i < nbins; ++i) { double m = abs(buf[i]); double p = m*m; psd[i] = p; tot += p; }
        if (tot > 0.0) spec_ent = spectral_entropy(psd);
        else spec_ent = 0.0;
    }

    return { duty, simLen, max_run, spec_ent };
}

// scoring: lower is better (duty error primary, then spectral penalty & run penalty)
// weights adjustable
double score_candidate(double duty, double target, double specEnt, uint32_t maxRun, uint32_t simLen) {
    double duty_err = fabs(duty - target); // [0..1]
    double spec_pen = 1.0 - specEnt;       // higher entropy -> lower penalty
    double run_pen = (double)maxRun / (double)simLen;
    const double w_duty = 0.70, w_spec = 0.20, w_run = 0.10;
    return w_duty * duty_err + w_spec * spec_pen + w_run * run_pen;
}

// -------------------- I/O helpers --------------------

// read simple pairs bin (u16,u16)
bool read_pairs_simple(const string &fname, vector<pair<u16,u16>>& out) {
    ifstream ifs(fname, ios::binary);
    if (!ifs) return false;
    out.clear();
    // get file length
    ifs.seekg(0, ios::end);
    size_t sz = (size_t)ifs.tellg();
    if (sz % 4 != 0) return false; // must be pairs of u16
    ifs.seekg(0, ios::beg);
    size_t npairs = sz / 4;
    out.reserve(npairs);
    for (size_t i = 0; i < npairs; ++i) {
        u16 m1 = 0, m2 = 0;
        ifs.read(reinterpret_cast<char*>(&m1), sizeof(m1));
        ifs.read(reinterpret_cast<char*>(&m2), sizeof(m2));
        out.emplace_back(m1, m2);
    }
    return true;
}

// read mask cache file (MLUT)
bool read_cache(const string &fname, vector<MaskCacheRecord>& out) {
    ifstream ifs(fname, ios::binary);
    if (!ifs) return false;
    char magic[4];
    ifs.read(magic, 4);
    if (memcmp(magic, CACHE_MAGIC, 4) != 0) return false;
    uint16_t ver; ifs.read(reinterpret_cast<char*>(&ver), sizeof(ver));
    if (ver != CACHE_VERSION) return false;
    u32 count; ifs.read(reinterpret_cast<char*>(&count), sizeof(count));
    out.resize(count);
    ifs.read(reinterpret_cast<char*>(out.data()), count * sizeof(MaskCacheRecord));
    return !ifs.fail();
}

// write mask cache in same format (regenerate)
bool write_cache(const string &fname, const vector<MaskCacheRecord>& recs) {
    ofstream ofs(fname, ios::binary);
    if (!ofs) return false;
    ofs.write(CACHE_MAGIC, 4);
    ofs.write(reinterpret_cast<const char*>(&CACHE_VERSION), sizeof(CACHE_VERSION));
    u32 count = (u32)recs.size();
    ofs.write(reinterpret_cast<const char*>(&count), sizeof(count));
    ofs.write(reinterpret_cast<const char*>(recs.data()), recs.size()*sizeof(MaskCacheRecord));
    return !ofs.fail();
}

// write final header pwm_lut.h
bool write_header(const string &fname, const vector<array<uint16_t,6>>& entries, const vector<int>& targetPercents) {
    // PWMLUTEntry struct: mask1,seed1,mask2,seed2,duty_pct,period
    ofstream ofs(fname);
    if (!ofs) return false;
    ofs << "// Auto-generated PWM LUT\n#pragma once\n#include <stdint.h>\n\n";
    ofs << "struct PWMLUTEntry { uint16_t mask1; uint16_t seed1; uint16_t mask2; uint16_t seed2; uint8_t duty_pct; uint32_t period; };\n\n";
    ofs << "static const PWMLUTEntry PWM_LUT[] = {\n";
    for (size_t i = 0; i < entries.size(); ++i) {
        auto &e = entries[i];
        ofs << "  { 0x" << hex << setw(4) << setfill('0') << e[0]
            << ", 0x" << setw(4) << e[1]
            << ", 0x" << setw(4) << e[2]
            << ", 0x" << setw(4) << e[3] << dec
            << ", " << (int)e[4]
            << ", " << (uint32_t)e[5] << " }, // target " << targetPercents[i] << "%\n";
    }
    ofs << "};\n";
    return true;
}

// write pruned pairs simple binary
bool write_pairs_simple(const string &fname, const vector<pair<u16,u16>>& pairs) {
    ofstream ofs(fname, ios::binary);
    if (!ofs) return false;
    for (auto &p : pairs) {
        ofs.write(reinterpret_cast<const char*>(&p.first), sizeof(u16));
        ofs.write(reinterpret_cast<const char*>(&p.second), sizeof(u16));
    }
    return !ofs.fail();
}

// -------------------- mask cache regeneration (full masks 1..0xFFFF) --------------------
// this generates a compact record per mask; sampling seeds to mark 'good_seed_pct'
void regenerate_cache(const string &out_file, uint32_t sample_seeds, uint32_t max_steps, size_t threads) {
    cerr << "Regenerating full mask cache (masks 1..0xFFFF) ...\n";
    vector<MaskCacheRecord> recs;
    recs.reserve(0xFFFF);
    // sample seeds: deterministic RNG
    auto next_rand = [](uint64_t &s)->u16 {
        s ^= s << 13; s ^= s >> 7; s ^= s << 17;
        return (u16)(s & 0xFFFFu);
    };
    uint64_t rng = 0xC0FFEE123456ULL;
    // precompute seed list
    vector<u16> seeds; seeds.reserve(sample_seeds);
    seeds.push_back(1);
    while (seeds.size() < sample_seeds) {
        u16 s = next_rand(rng);
        if (s == 0) continue;
        seeds.push_back(s);
    }

    // threaded job: process mask ranges
    mutex out_mu; atomic<uint32_t> processed{0};
    uint32_t maxmask = 0xFFFFu;
    uint32_t batch = 2048;
    vector<thread> workers;
    atomic<uint32_t> next_base{1};

    auto worker = [&](){
        vector<MaskCacheRecord> local;
        while (true) {
            uint32_t base = next_base.fetch_add(batch);
            if (base > maxmask) break;
            uint32_t end = min<uint32_t>(maxmask, base + batch - 1);
            for (uint32_t m = base; m <= end; ++m) {
                u16 mask = (u16)m;
                // canonical rep period using seed=1
                auto lfsr_period = [&](u16 seed)->uint32_t {
                    if (seed == 0) return 1;
                    u16 s = seed; uint32_t cnt = 0;
                    do { s = galois_step(s, mask); ++cnt; if (cnt >= max_steps) return cnt; } while (s != seed);
                    return cnt;
                };
                uint32_t rep = lfsr_period(1);
                uint32_t maxp = 0;
                uint32_t good = 0;
                for (u16 s : seeds) { uint32_t p = lfsr_period(s); if (p > maxp) maxp = p; if (p >= 1024) ++good; }
                MaskCacheRecord r;
                r.mask = mask;
                r.rep_period = rep;
                r.max_sampled_period = (u16)min<uint32_t>(maxp, 0xFFFFu);
                r.good_seed_pct = (u8)min<uint32_t>(100u, (good * 100u) / seeds.size());
                r.is_maximal = (rep >= 0xFFFFu) ? 1 : 0;
                r.reserved[0] = 0;
                local.push_back(r);
                processed.fetch_add(1, memory_order_relaxed);
            }
            // append local
            { lock_guard lk(out_mu); recs.insert(recs.end(), local.begin(), local.end()); local.clear(); }
        }
    };

    size_t nth = max<size_t>(1, threads);
    for (size_t t = 0; t < nth; ++t) workers.emplace_back(worker);
    // progress monitor
    while (processed.load() < maxmask) {
        this_thread::sleep_for(chrono::milliseconds(300));
        uint32_t p = processed.load();
        double pct = 100.0 * (double)p / (double)maxmask;
        cerr << "\rProcessed masks: " << p << "/" << maxmask << " (" << fixed << setprecision(2) << pct << "%)   " << flush;
    }
    for (auto &th : workers) if (th.joinable()) th.join();
    sort(recs.begin(), recs.end(), [](auto &a, auto &b){ return a.mask < b.mask; });
    if (!write_cache(out_file, recs)) { cerr << "\nFailed to write cache\n"; exit(1); }
    cerr << "\nCache written: " << out_file << " entries=" << recs.size() << "\n";
}

// -------------------- main LUT generation --------------------
int main(int argc, char** argv) {
    // defaults
    string cache_file = "masks_cache.bin";
    string pairs_file = "";         // if provided, read and evaluate only these pairs
    string out_header = "pwm_lut.h";
    string write_pruned_pairs = ""; // optional output pruned pairs (simple u16,u16 file)
    bool do_regenerate_cache = false;
    size_t threads = thread::hardware_concurrency();
    uint32_t samples = 32768;       // simulation cap
    int step_percent = 5;           // target step (0..100), e.g. 5 => 0,5,10,...,100
    bool use_spectral = true;
    uint32_t spectral_min_len = 1024; // min length to compute spectral entropy
    uint32_t sample_seeds = 4;      // seeds per LFSR to try when evaluating a pair (small set)
    // simple seed set (non-zero)
    vector<u16> seedSet = { 1u, 0xACE1u, 0xBEEFu, 0x1234u };

    // parse CLI
    for (int i = 1; i < argc; ++i) {
        string a = argv[i];
        if (a == "--help" || a == "-h") {
            cerr << "Usage: lutgen_refine [--regenerate] [--cache file] [--pairs pairs.bin] [--out header.h] "
                 << "[--write-pairs file] [--step N] [--samples N] [--threads N] [--no-spectral]\n";
            return 0;
        } else if (a == "--regenerate") do_regenerate_cache = true;
        else if (a == "--cache" && i+1 < argc) cache_file = argv[++i];
        else if (a == "--pairs" && i+1 < argc) pairs_file = argv[++i];
        else if (a == "--out" && i+1 < argc) out_header = argv[++i];
        else if (a == "--write-pairs" && i+1 < argc) write_pruned_pairs = argv[++i];
        else if (a == "--step" && i+1 < argc) step_percent = stoi(argv[++i]);
        else if (a == "--samples" && i+1 < argc) samples = (uint32_t)stoul(argv[++i]);
        else if (a == "--threads" && i+1 < argc) threads = (size_t)stoul(argv[++i]);
        else if (a == "--no-spectral") use_spectral = false;
        else if (a == "--sample-seeds" && i+1 < argc) sample_seeds = (uint32_t)stoul(argv[++i]);
        else {
            cerr << "Unknown arg: " << a << "\n"; return 1;
        }
    }

    // if requested, regenerate cache
    if (do_regenerate_cache) regenerate_cache(cache_file, 64, samples, threads);

    // try read cache
    vector<MaskCacheRecord> cache;
    bool have_cache = read_cache(cache_file, cache);
    if (!have_cache) {
        cerr << "Warning: mask cache not found (" << cache_file << "). Proceeding without cache.\n";
    } else {
        cerr << "Loaded mask cache entries: " << cache.size() << "\n";
    }

    // If pairs file provided, load it and evaluate only those pairs
    vector<pair<u16,u16>> candidate_pairs;
    bool using_pairs_file = false;
    if (!pairs_file.empty()) {
        if (!read_pairs_simple(pairs_file, candidate_pairs)) {
            cerr << "Failed to read pairs file: " << pairs_file << "\n";
            return 1;
        }
        using_pairs_file = true;
        cerr << "Loaded pairs file: " << candidate_pairs.size() << " pairs -> will evaluate only these.\n";
    }

    // If no pairs input, generate candidate pairs from mask cache (or simple small search)
    if (!using_pairs_file) {
        // if we have cache, pick candidate masks filtered by cache stats to reduce space
        vector<u16> masks;
        if (have_cache) {
            for (auto &r : cache) {
                // quick prefilter: prefer masks with at least some good seeds OR maximal
                if (r.is_maximal || r.good_seed_pct >= 10) masks.push_back(r.mask);
            }
            cerr << "Candidate masks after cache prefilter: " << masks.size() << "\n";
        } else {
            // fallback: generate a small mask list (e.g. small weight masks) - keep manageable
            for (u16 m = 1; m != 0; ++m) {
                // include only masks with low hamming weight to keep tractable if no cache
                int wt = __builtin_popcount((unsigned)m);
                if (wt <= 4) masks.push_back(m);
                if (masks.size() >= 2048) break;
            }
            cerr << "Generated fallback mask set: " << masks.size() << "\n";
        }

        // pair every mask with every mask (may be large). To keep search practical, we will restrict seeds & masks:
        // build candidate_pairs as all combinations (m1,m2)
        candidate_pairs.reserve((size_t)masks.size() * masks.size());
        for (u16 m1 : masks) for (u16 m2 : masks) candidate_pairs.emplace_back(m1, m2);
        cerr << "Candidate pairs count: " << candidate_pairs.size() << "\n";
    }

    // Prepare targets
    vector<double> targets;
    vector<int> targetPercents;
    for (int pct = 0; pct <= 100; pct += max(1, step_percent)) {
        targets.push_back((double)pct / 100.0);
        targetPercents.push_back(pct);
    }
    size_t Ntargets = targets.size();
    cerr << "Targets: " << Ntargets << " steps (0.." << 100 << " step=" << step_percent << ")\n";

    // result containers: best candidate per target
    struct Best { double score; u16 m1,s1,m2,s2; double duty; uint32_t period; uint32_t maxrun; double spec; };
    vector<Best> best(Ntargets, {1e9,0,0,0,0,0,0,0.0});
    mutex best_mu;

    // basic multi-threaded evaluation over candidate_pairs
    atomic<size_t> idx{0}, processed{0};
    size_t total_jobs = candidate_pairs.size();
    size_t nth = max<size_t>(1, threads);
    cerr << "Starting evaluation threads: " << nth << "\n";

    // prepare seed sets (try several seeds for robustness)
    vector<u16> seeds;
    if (sample_seeds == 0) seeds = seedSet;
    else {
        // deterministic simple rng to generate sample_seeds seeds
        uint64_t s = 0xC0FFEE12345ULL;
        seeds.push_back(1);
        while (seeds.size() < sample_seeds) {
            s ^= s << 13; s ^= s >> 7; s ^= s << 17;
            u16 v = (u16)(s & 0xFFFFu);
            if (v == 0) continue;
            seeds.push_back(v);
        }
    }

    auto worker = [&]() {
        size_t i;
        while ((i = idx.fetch_add(1)) < total_jobs) {
            u16 m1 = candidate_pairs[i].first, m2 = candidate_pairs[i].second;
            // try a small set of seed pairs to find good scoring behavior
            for (u16 s1 : seeds) {
                if (s1 == 0) continue;
                for (u16 s2 : seeds) {
                    if (s2 == 0) continue;
                    // quick skip degenerate states
                    if (s1 == 0 || s2 == 0) continue;
                    SimResult sim = simulate_pair(m1, s1, m2, s2, samples, use_spectral, spectral_min_len);
                    // evaluate for every target (we'll update bests if better)
                    for (size_t ti = 0; ti < Ntargets; ++ti) {
                        double sc = score_candidate(sim.duty, targets[ti], sim.spec_entropy, sim.max_run, sim.period);
                        bool update = false;
                        { // scope lock only when needed
                            if (sc < best[ti].score) update = true;
                        }
                        if (update) {
                            lock_guard lk(best_mu);
                            if (sc < best[ti].score) {
                                best[ti] = { sc, m1, s1, m2, s2, sim.duty, sim.period, sim.max_run, sim.spec_entropy };
                            }
                        }
                    }
                }
            }
            auto p = ++processed;
            if ((p & 0x3FFF) == 0) {
                double pct = 100.0 * (double)p / (double)total_jobs;
                cerr << "\rProgress: " << p << "/" << total_jobs << " (" << fixed << setprecision(2) << pct << "%)   " << flush;
            }
        }
    };

    vector<thread> workers;
    for (size_t t = 0; t < nth; ++t) workers.emplace_back(worker);
    for (auto &th : workers) if (th.joinable()) th.join();
    cerr << "\nEvaluation done. Selecting best candidates...\n";

    // Build header entries array
    vector<array<uint16_t,6>> entries; // mask1,seed1,mask2,seed2,duty_pct,period
    vector<pair<u16,u16>> selectedPairs; selectedPairs.reserve(Ntargets);
    for (size_t ti = 0; ti < Ntargets; ++ti) {
        auto &b = best[ti];
        if (b.score >= 1e8) {
            // no candidate found -> emit degenerate (mask=0) or skip; here we push zeros
            entries.push_back({0,0,0,0,(uint16_t)targetPercents[ti],0});
        } else {
            uint8_t duty_pct = (uint8_t)round(b.duty * 100.0);
            entries.push_back({b.m1, b.s1, b.m2, b.s2, (uint16_t)duty_pct, (uint16_t)min<uint32_t>(b.period, 0xFFFFu)});
            selectedPairs.emplace_back(b.m1, b.m2);
        }
    }

    if (!write_header(out_header, entries, targetPercents)) {
        cerr << "Failed to write header: " << out_header << "\n"; return 1;
    }
    cerr << "Wrote LUT header: " << out_header << "\n";

    if (!write_pruned_pairs.empty()) {
        // write selected pairs (unique)
        sort(selectedPairs.begin(), selectedPairs.end());
        selectedPairs.erase(unique(selectedPairs.begin(), selectedPairs.end()), selectedPairs.end());
        if (!write_pairs_simple(write_pruned_pairs, selectedPairs)) {
            cerr << "Failed to write pruned pairs: " << write_pruned_pairs << "\n";
        } else {
            cerr << "Wrote pruned pairs file: " << write_pruned_pairs << " entries=" << selectedPairs.size() << "\n";
        }
    }

    cerr << "Done.\n";
    return 0;
}


---

Notes, recommendations & how the two tools cooperate

Workflow I recommend

1. Run lutgen_refine --regenerate once to build the full masks_cache.bin (heavy).


2. Run the initial LUT generation (no pairs) to get pwm_lut.h and optionally --write-pairs pruned_pairs.bin producing a reduced pair set.


3. Run the spectral pruning tool (spectral_prune_compatible) on pruned_pairs.bin to remove harmonic/low-frequency pairs (it will output pruned_pairs.bin compatible).


4. Re-run lutgen_refine --pairs pruned_pairs.bin to evaluate only the pruned subset and produce a refined pwm_lut.h. Repeat the refine ↔ spectral-prune loop until you converge on a compact, high-quality pair set.



Compatibility

The pairs.bin format used here is the simple u16,u16 format that spectral-prune accepts and emits. Both programs therefore interoperate smoothly.

The mask cache file masks_cache.bin format matches the earlier tool. Both programs read it for prefiltering and faster operation.


Tuning

Increase --samples to compute metrics over longer simulated sequences (higher fidelity; slower).

Enable/disable spectral scoring via --no-spectral if you prefer purely duty-oriented selection.

Use --sample-seeds to control how many seed combinations are tried for robustness.



If you want, I can:

add an option to store the best seed pair for each retained mask pair in the pruned pairs.bin (so the MCU can load masks+seeds directly), or

produce a small Python inspection script to visualize per-pair PSD/duty to help tune thresholds.


Which would you like next?
