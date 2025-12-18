// lutgen_full.cpp
// Multithreaded 16-bit mask cache + progressive pruning for XOR-Galois LFSR PWM generator
// - Use --regenerate to build full mask cache (masks_cache.bin)
// - Use --prune to load cache and prune mask pairs to produce pwm_lut.h or pairs.bin
// Build: g++ -O3 -march=native -std=c++17 -pthread -o lutgen_full lutgen_full.cpp

#include <bits/stdc++.h>
using namespace std;
using u8  = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;
using u64 = uint64_t;

// ---------------------------- Configurable defaults ----------------------------
static const char* CACHE_MAGIC = "MLUT"; // mask LUT cache
static const uint16_t CACHE_VERSION = 1;

struct MaskCacheRecord {
    u16 mask;             // 2
    u32 rep_period;       // 4
    u8  good_seed_pct;    // 1 (0..100) percent of sampled seeds considered 'good'
    u8  is_maximal;       // 1 flag
    u16 max_sampled_period;// 2
    u8  reserved[1];      // padding -> total 12 bytes
};

// Galois step (16-bit LFSR): out = state & 1; state >>=1; if(out) state ^= mask;
static inline u16 galois_step(u16 state, u16 mask) {
    u16 out = state & 1u;
    state >>= 1;
    if (out) state ^= mask;
    return state;
}

// Compute period starting from seed (nonzero). capped at maxcap
u32 lfsr_period(u16 seed, u16 mask, u32 maxcap) {
    if (seed == 0) return 1;
    u16 s = seed;
    u32 cnt = 0;
    do {
        s = galois_step(s, mask);
        ++cnt;
        if (cnt >= maxcap) return cnt;
    } while (s != seed);
    return cnt;
}

// ---------------------------- Thread pool ----------------------------
struct ThreadPool {
    vector<thread> workers;
    deque<function<void()>> tasks;
    mutex mu;
    condition_variable cv;
    bool stopping = false;
    ThreadPool(size_t n=1) {
        for (size_t i=0;i<n;++i) workers.emplace_back([this]{ this->worker(); });
    }
    ~ThreadPool() {
        {
            unique_lock lk(mu);
            stopping = true;
            cv.notify_all();
        }
        for (auto &t: workers) if (t.joinable()) t.join();
    }
    void worker(){
        while(true){
            function<void()> job;
            { unique_lock lk(mu);
              cv.wait(lk, [&]{ return stopping || !tasks.empty(); });
              if (stopping && tasks.empty()) return;
              job = move(tasks.front()); tasks.pop_front();
            }
            job();
        }
    }
    void enqueue(function<void()> f){
        { lock_guard lk(mu); tasks.push_back(move(f)); }
        cv.notify_one();
    }
};

// ---------------------------- CLI parse ----------------------------
struct Options {
    bool regenerate = false;
    bool prune = false;
    string cache_file = "masks_cache.bin";
    string out_header = "pwm_lut.h";
    string out_pairs = "pairs.bin";
    size_t threads = thread::hardware_concurrency();
    u32 sample_seeds = 64;        // how many random seeds to sample per mask when building cache
    u32 max_steps = 65536;        // simulation cap / period cap
    double long_period_thresh = 1024; // threshold to count seed as producing 'long' period
    u32 tolerance_counts = 2;     // tolerance in 0..255 counts for 256-step PWM map
    bool verbose = true;
    vector<u16> seed_list;        // if empty use default small set for pruning phases
} opt;

// ---------------------------- Utilities ----------------------------
static inline uint64_t gcd_u64(uint64_t a, uint64_t b){
    while(b){ uint64_t t=a%b; a=b; b=t; } return a;
}
static inline uint64_t lcm_u64(uint64_t a, uint64_t b){
    if (a==0 || b==0) return 0;
    return (a / gcd_u64(a,b)) * b;
}

// small deterministic RNG for sampling seeds (useful cross-run repeatability)
struct FastRNG {
    uint64_t s;
    FastRNG(uint64_t seed=0xC0FFEE123456789ULL){ s = seed ? seed : 1; }
    uint32_t next_u32(){ s ^= s << 13; s ^= s >> 7; s ^= s << 17; return (uint32_t)(s & 0xFFFFFFFFu); }
    u16 next_u16(){ return (u16)(next_u32() & 0xFFFFu); }
};

// ---------------------------- Cache generation ----------------------------
bool write_cache(const string &fname, const vector<MaskCacheRecord> &records){
    ofstream ofs(fname, ios::binary);
    if(!ofs) return false;
    ofs.write(CACHE_MAGIC, 4);
    ofs.write(reinterpret_cast<const char*>(&CACHE_VERSION), sizeof(CACHE_VERSION));
    u32 count = (u32)records.size();
    ofs.write(reinterpret_cast<const char*>(&count), sizeof(count));
    ofs.write(reinterpret_cast<const char*>(records.data()), records.size()*sizeof(MaskCacheRecord));
    return !ofs.fail();
}

bool read_cache(const string &fname, vector<MaskCacheRecord> &records){
    ifstream ifs(fname, ios::binary);
    if(!ifs) return false;
    char magic[4]; ifs.read(magic,4);
    if(strncmp(magic, CACHE_MAGIC, 4) != 0) return false;
    uint16_t ver; ifs.read(reinterpret_cast<char*>(&ver), sizeof(ver));
    if(ver != CACHE_VERSION) return false;
    u32 count; ifs.read(reinterpret_cast<char*>(&count), sizeof(count));
    records.resize(count);
    ifs.read(reinterpret_cast<char*>(records.data()), count * sizeof(MaskCacheRecord));
    return !ifs.fail();
}

// Build full mask cache (all masks 1..0xFFFF) sampling seeds per mask.
void build_full_cache(const string &out_file) {
    if (opt.verbose) cerr << "Regenerating full mask cache -> " << out_file << "\n";
    const u32 maxmask = 0xFFFFu;
    vector<MaskCacheRecord> cache;
    cache.reserve(maxmask);

    // seeds to sample: random non-zero seeds, plus seed=1 canonical
    FastRNG rng(0xDEADBEEFUL);
    vector<u16> sampled_seeds;
    sampled_seeds.push_back(1u);
    // ensure we don't pick zero seeds
    for (u32 i=0; sampled_seeds.size() < opt.sample_seeds; ++i){
        u16 s = rng.next_u16();
        if (s == 0) continue;
        sampled_seeds.push_back(s);
    }

    // worker threads to compute mask records in ranges
    ThreadPool pool(max<size_t>(1, opt.threads));
    mutex out_mu;
    atomic<u32> processed{0};

    const u32 batch_sz = 1024; // per-task chunk of masks
    for (u32 base = 1; base <= maxmask; base += batch_sz) {
        u32 end = min<u32>(maxmask, base + batch_sz - 1);
        pool.enqueue([=,&cache,&out_mu,&processed,&sampled_seeds]() {
            vector<MaskCacheRecord> local;
            local.reserve(end-base+1);
            for (u32 m = base; m <= end; ++m) {
                u16 mask = (u16)m;
                // For each mask sample the seeds: compute period, find max, count 'good' long periods
                u32 maxp = 0;
                u32 good = 0;
                u32 rep_period = lfsr_period(1u, mask, opt.max_steps); // canonical seed=1
                for (u16 s : sampled_seeds) {
                    if (s == 0) continue;
                    u32 p = lfsr_period(s, mask, opt.max_steps);
                    if (p > maxp) maxp = p;
                    if ((double)p >= opt.long_period_thresh) ++good;
                }
                MaskCacheRecord r;
                r.mask = mask;
                r.rep_period = rep_period;
                r.max_sampled_period = (u16)min<u32>(maxp, 0xFFFF);
                r.good_seed_pct = (u8)min<u32>(100u, (u32)((good * 100u) / sampled_seeds.size()));
                // quick maximal-length check: rep_period == 2^16 -1
                r.is_maximal = (rep_period >= 0xFFFFu) ? 1 : 0;
                r.reserved[0] = 0;
                local.push_back(r);
                processed.fetch_add(1, memory_order_relaxed);
            }
            // append local results
            {
                lock_guard lk(out_mu);
                cache.insert(cache.end(), local.begin(), local.end());
            }
        });
    }

    // destructor will wait for tasks to finish
    // busy-wait progress
    while ((u32)cache.size() < 0xFFFFu - 0u) {
        this_thread::sleep_for(std::chrono::milliseconds(300));
        // break when all processed done
        if (processed.load() >= 0xFFFFu) break;
        if (opt.verbose) {
            uint32_t p = processed.load();
            double pct = 100.0 * (double)p / (double)0xFFFFu;
            cerr << "\rMasks processed: " << p << "/" << 0xFFFFu << " (" << fixed << setprecision(2) << pct << "%)    " << flush;
        }
    }
    if (opt.verbose) cerr << "\nCache entries built: " << cache.size() << "\n";
    // sort by mask to have deterministic order
    sort(cache.begin(), cache.end(), [](auto &a, auto &b){ return a.mask < b.mask; });

    if (!write_cache(out_file, cache)) {
        cerr << "Failed to write cache file\n";
        exit(1);
    }
    if (opt.verbose) cerr << "Cache written: " << out_file << "\n";
}

// ---------------------------- Pair evaluation/pruning ----------------------------

// A small seed set used for pair-level simulation by default (user can override by providing a file or list)
vector<u16> default_prune_seeds() {
    return { 0x1u, 0xACE1u, 0xBEEFu, 0x1234u, 0xFFFFu };
}

// Simulate two masks with each chosen seed pair to compute duty mapped to 256 steps
// Return true if pair passes tolerance for at least one seed pair combination
bool evaluate_mask_pair_for_256(u16 mask1, u16 mask2,
                                const vector<u16>& seeds1, const vector<u16>& seeds2,
                                u32 max_steps, u32 tolerance_counts) {
    // Quick check: if either mask had rep_period == 1 (from cache) we might skip, but here we call explicitly
    for (u16 s1 : seeds1) {
        if (s1 == 0) continue;
        u32 p1 = lfsr_period(s1, mask1, max_steps);
        if (p1 <= 1) continue; // degenerate
        for (u16 s2 : seeds2) {
            if (s2 == 0) continue;
            u32 p2 = lfsr_period(s2, mask2, max_steps);
            if (p2 <= 1) continue;
            // compute combined period (lcm capped)
            uint64_t L = lcm_u64((uint64_t)p1, (uint64_t)p2);
            uint32_t simLen = (uint32_t)min<uint64_t>(L, max_steps);
            if (simLen == 0) continue;
            // simulate
            u16 st1 = s1, st2 = s2;
            u32 ones = 0;
            for (u32 i = 0; i < simLen; ++i) {
                st1 = galois_step(st1, mask1);
                st2 = galois_step(st2, mask2);
                ones += (uint32_t)(((st1 ^ st2) & 1u) ? 1u : 0u);
            }
            // map to 0..255
            double duty = (double)ones / (double)simLen;
            int mapped = (int)round(duty * 255.0);
            // compute nearest ideal (rounded)
            double exact = duty * 255.0;
            int err = (int)abs(exact - (double)mapped);
            if ((u32)err <= tolerance_counts) {
                // acceptable for at least one seed pair
                return true;
            }
        }
    }
    return false;
}

// Prune mask pairs using cache and produce C header with good pairs
void prune_using_cache(const string &cache_file, const string &out_header) {
    if (opt.verbose) cerr << "Loading cache: " << cache_file << "\n";
    vector<MaskCacheRecord> cache;
    if (!read_cache(cache_file, cache)) {
        cerr << "Failed to read cache file\n"; exit(1);
    }
    if (opt.verbose) cerr << "Cache entries: " << cache.size() << "\n";

    // Build list of candidate masks (apply simple pre-filter using cached stats)
    vector<u16> candidates;
    candidates.reserve(cache.size());
    for (const auto &r : cache) {
        // filter: require that at least X% sampled seeds were 'good' OR is_maximal
        // this avoids masks where almost all seeds fall into trivial cycles.
        if (r.is_maximal || r.good_seed_pct >= 20) { // 20% threshold default; you can tweak
            candidates.push_back(r.mask);
        }
    }
    if (opt.verbose) cerr << "Candidate masks after prefilter: " << candidates.size() << "\n";

    // seeds to test for pair evaluation:
    vector<u16> seeds1 = default_prune_seeds();
    vector<u16> seeds2 = seeds1;

    // evaluate pair combinations (candidates^2) but prune progressively and store hits
    vector<array<u16,4>> accepted_pairs; // mask1, mask2, seed1 (first that passed), seed2
    mutex accept_mu;
    atomic<u64> jobsDone{0}, totalJobs{0};
    for (size_t i = 0; i < candidates.size(); ++i) totalJobs += candidates.size();

    ThreadPool pool(max<size_t>(1, opt.threads));
    for (size_t i = 0; i < candidates.size(); ++i) {
        u16 m1 = candidates[i];
        for (size_t j = 0; j < candidates.size(); ++j) {
            u16 m2 = candidates[j];
            pool.enqueue([=,&seeds1,&seeds2,&accepted_pairs,&accept_mu,&jobsDone]() {
                bool ok = evaluate_mask_pair_for_256(m1, m2, seeds1, seeds2, opt.max_steps, opt.tolerance_counts);
                if (ok) {
                    // record pair and example seed pair (we store 0 for seeds for now)
                    lock_guard lk(accept_mu);
                    accepted_pairs.push_back({m1, m2, 0u, 0u});
                }
                jobsDone.fetch_add(1, memory_order_relaxed);
            });
        }
    }

    // wait for finish
    while (jobsDone.load() < totalJobs) {
        this_thread::sleep_for(chrono::milliseconds(200));
        if (opt.verbose) {
            double pct = 100.0 * (double)jobsDone.load() / (double)totalJobs;
            cerr << "\rPairs tested: " << jobsDone << "/" << totalJobs << " (" << fixed << setprecision(2) << pct << "%)   " << flush;
        }
    }
    if (opt.verbose) cerr << "\nAccepted pairs: " << accepted_pairs.size() << "\n";

    // Write header with accepted pairs (you can later refine to include seeds/score)
    ofstream ofs(out_header);
    if (!ofs) { cerr << "Cannot open " << out_header << "\n"; exit(1); }
    ofs << "// Auto-generated PWM LUT pairs (mask1, mask2). Format: mask1, mask2\n";
    ofs << "#pragma once\n#include <stdint.h>\n\nstruct PairEntry { uint16_t mask1; uint16_t mask2; };\n";
    ofs << "static const PairEntry PWM_PAIRS[] = {\n";
    for (auto &p : accepted_pairs) {
        ofs << "  { 0x" << hex << setw(4) << setfill('0') << p[0]
            << ", 0x" << setw(4) << p[1] << dec << " },\n";
    }
    ofs << "};\n";
    ofs.close();
    if (opt.verbose) cerr << "Header written: " << out_header << "\n";
}

// ---------------------------- Main / CLI ----------------------------
void print_help_and_exit() {
    cerr << "Usage: lutgen_full [--regenerate] [--prune] [--cache file] [--out-header file]\n"
         << "  --regenerate        Build full masks cache (heavy)\n"
         << "  --prune             Load cache and prune pairs -> produces header\n"
         << "  --cache <file>      Cache filename (default masks_cache.bin)\n         "
         << "--out-header <file>  Output C header (default pwm_lut.h)\n"
         << "  --threads N         Worker threads (default hardware concurrency)\n"
         << "  --sample-seeds N    Seeds sampled per mask when regenerating (default 64)\n"
         << "  --max-steps N       Simulation cap when computing periods (default 65536)\n"
         << "  --tolerance N       Tolerance in 0..255 counts for 256-step mapping (default 2)\n";
    exit(1);
}

int main(int argc, char** argv) {
    if (argc == 1) print_help_and_exit();
    for (int i=1;i<argc;++i){
        string a = argv[i];
        if (a == "--regenerate") opt.regenerate = true;
        else if (a == "--prune") opt.prune = true;
        else if (a == "--cache" && i+1<argc) opt.cache_file = argv[++i];
        else if (a == "--out-header" && i+1<argc) opt.out_header = argv[++i];
        else if (a == "--threads" && i+1<argc) opt.threads = stoul(argv[++i]);
        else if (a == "--sample-seeds" && i+1<argc) opt.sample_seeds = stoul(argv[++i]);
        else if (a == "--max-steps" && i+1<argc) opt.max_steps = stoul(argv[++i]);
        else if (a == "--tolerance" && i+1<argc) opt.tolerance_counts = stoul(argv[++i]);
        else { cerr << "Unknown arg: " << a << "\n"; print_help_and_exit(); }
    }

    if (!opt.regenerate && !opt.prune) {
        cerr << "Specify at least --regenerate or --prune\n"; return 1;
    }

    if (opt.regenerate) build_full_cache(opt.cache_file);
    if (opt.prune) prune_using_cache(opt.cache_file, opt.out_header);

    return 0;
}
