Below is the improved, ready-to-compile spectral pruning program. It is explicitly compatible with the LUT generator pipeline we designed:

Input pairs binary format: simple sequence of uint16_t mask1, uint16_t mask2, ... (no header). The tool also accepts the legacy "PAPR" binary format and textual/C-header/CSV pairs for convenience.

Mask cache format: the "MLUT" binary layout used by the LUT generator (same MaskCacheRecord).

Output: a simple pairs binary (same simple u16,u16 sequence) containing only accepted pairs. Optionally writes a C header for quick inspection.

Multi-threaded, configurable thresholds for spectral entropy, dominant peak fraction, low-frequency energy fraction, and sliding-window duty stability.

Option to attempt multiple seed pairs (default sensible set) — a pair is kept if any tested seed pair meets the spectral/stability criteria.


Save as spectral_prune_refined.cpp and compile:

g++ -O3 -march=native -std=c++17 -pthread -o spectral_prune_refined spectral_prune_refined.cpp

Run example:

./spectral_prune_refined --cache masks_cache.bin --pairs pairs.bin \
  --out pruned_pairs.bin --out-h pruned_pairs.h --threads 12 \
  --max-steps 65536 --window 256 --entropy 0.82 --peak 0.06 --lowfreq 0.10 --stability 0.004


---

// spectral_prune_refined.cpp
// Refined spectral pruning for dual-LFSR PWM pairs (compatible with lutgen_refine pipeline)
// - Accepts simple pairs binary (u16,u16) or legacy "PAPR" or textual headers/CSV
// - Loads masks cache ("MLUT") if available for informational use (not required)
// - Outputs pruned simple pairs binary (u16,u16) and optional C header
// - Multi-threaded, configurable thresholds
//
// Build:
//   g++ -O3 -march=native -std=c++17 -pthread -o spectral_prune_refined spectral_prune_refined.cpp

#include <bits/stdc++.h>
using namespace std;
using u8  = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;
using u64 = uint64_t;
using cd  = complex<double>;

// ---------------------- Cache & legacy formats ----------------------
static const char CACHE_MAGIC[4] = {'M','L','U','T'};
static const uint16_t CACHE_VERSION = 1;

#pragma pack(push,1)
struct MaskCacheRecord {
    u16 mask;
    u32 rep_period;
    u8  good_seed_pct;
    u8  is_maximal;
    u16 max_sampled_period;
    u8  reserved[1];
};
#pragma pack(pop)

static const char LEGACY_PAIR_MAGIC[4] = {'P','A','P','R'};
#pragma pack(push,1)
struct PairLegacyEntry {
    u16 mask1;
    u16 mask2;
    u16 seed1;
    u16 seed2;
    u8  reserved[2];
};
#pragma pack(pop)

// ---------------------- Galois LFSR step ----------------------
static inline u16 galois_step(u16 state, u16 mask) {
    u16 out = state & 1u;
    state >>= 1;
    if (out) state ^= mask;
    return state;
}

// ---------------------- utilities ----------------------
uint64_t gcd_u64(uint64_t a, uint64_t b){ while(b){ uint64_t t=a%b; a=b; b=t;} return a; }
uint64_t lcm_u64(uint64_t a, uint64_t b){ if (a==0||b==0) return 0; return (a/gcd_u64(a,b))*b; }
size_t next_pow2(size_t v){ if(v==0) return 1; --v; v |= v>>1; v |= v>>2; v |= v>>4; v |= v>>8; v |= v>>16; if(sizeof(size_t)>4) v |= v>>32; return ++v; }

// in-place radix-2 FFT
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

// ---------------------- config & CLI ----------------------
struct Config {
    string cache_file = "masks_cache.bin";
    string pairs_file = "";
    string out_pairs = "pruned_pairs.bin"; // simple u16,u16 output
    string out_header = "";                 // optional C header output
    size_t threads = thread::hardware_concurrency();
    u32 max_steps = 65536;
    u32 window = 256;
    double entropy_thresh = 0.82;
    double peak_thresh = 0.06;
    double lowfreq_thresh = 0.10;
    double stability_thresh = 0.004;
    bool verbose = true;
    vector<u16> test_seeds; // if empty use defaults
} cfg;

void print_help() {
    cerr <<
    "spectral_prune_refined --pairs <file> [options]\n"
    "Options:\n"
    "  --cache <file>         masks cache (MLUT) [optional]\n"
    "  --pairs <file>         input pairs (simple u16/u16 binary or legacy PAPR or C header/CSV)\n"
    "  --out <file>           output pruned pairs binary (simple u16/u16) [default pruned_pairs.bin]\n"
    "  --out-h <file>         optional C header output (text)\n"
    "  --threads N            worker threads (default hardware concurrency)\n"
    "  --max-steps N          simulation cap / LCM cap (default 65536)\n"
    "  --window N             sliding window size for duty stability (default 256)\n"
    "  --entropy f            min spectral entropy [0..1] (default 0.82)\n"
    "  --peak f               max allowed fraction in any single PSD bin (default 0.06)\n"
    "  --lowfreq f            max allowed low-frequency energy fraction (default 0.10)\n"
    "  --stability f          max stddev of sliding-window duty (default 0.004)\n"
    "  --seeds a,b,c          comma-separated seed values (hex allowed, e.g. 0xACE1)\n"
    "  --help\n";
}

// ---------------------- I/O: read cache & pairs ----------------------
bool read_cache(const string &fname, vector<MaskCacheRecord>& out) {
    ifstream ifs(fname, ios::binary);
    if (!ifs) return false;
    char magic[4]; ifs.read(magic,4);
    if (memcmp(magic, CACHE_MAGIC, 4) != 0) return false;
    uint16_t ver; ifs.read(reinterpret_cast<char*>(&ver), sizeof(ver));
    if (ver != CACHE_VERSION) return false;
    u32 count; ifs.read(reinterpret_cast<char*>(&count), sizeof(count));
    out.resize(count);
    ifs.read(reinterpret_cast<char*>(out.data()), count * sizeof(MaskCacheRecord));
    return !ifs.fail();
}

// Parse pairs file: try legacy magic, then simple u16/u16 binary, else fallback to textual parse
bool read_pairs_multi(const string &fname, vector<pair<u16,u16>>& out) {
    out.clear();
    ifstream ifs(fname, ios::binary);
    if (!ifs) { cerr << "Failed to open pairs file: " << fname << "\n"; return false; }
    // read first 4 bytes
    char head[4];
    ifs.read(head, 4);
    if (!ifs) return false;
    if (memcmp(head, LEGACY_PAIR_MAGIC, 4) == 0) {
        // legacy format: PAPR + u32 count + entries
        u32 count; ifs.read(reinterpret_cast<char*>(&count), sizeof(count));
        for (u32 i = 0; i < count; ++i) {
            PairLegacyEntry e; ifs.read(reinterpret_cast<char*>(&e), sizeof(e));
            out.emplace_back(e.mask1, e.mask2);
        }
        return true;
    }
    // not legacy: check file size for simple u16 pairs
    ifs.seekg(0, ios::end);
    size_t sz = (size_t)ifs.tellg();
    if (sz % 4 == 0 && sz >= 4) {
        // simple u16 pairs binary
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
    // fallback to textual parse (C header or CSV)
    ifs.close();
    ifstream tfs(fname);
    if (!tfs) return false;
    string line;
    while (getline(tfs, line)) {
        // strip comments
        auto pos = line.find("//");
        if (pos != string::npos) line = line.substr(0,pos);
        // try { 0xAAAA, 0xBBBB } pattern
        size_t brace = line.find('{');
        if (brace != string::npos) {
            size_t endb = line.find('}', brace);
            string inside = (endb==string::npos) ? line.substr(brace+1) : line.substr(brace+1, endb-brace-1);
            stringstream ss(inside); string a,b;
            if (getline(ss,a,',')) {
                if (getline(ss,b,',')) {
                    auto trim = [](string s){ while(!s.empty() && isspace((unsigned char)s.front())) s.erase(s.begin()); while(!s.empty() && isspace((unsigned char)s.back())) s.pop_back(); return s; };
                    a = trim(a); b = trim(b);
                    if (!a.empty() && !b.empty()) {
                        try { u16 m1 = (u16)stoul(a, nullptr, 0); u16 m2 = (u16)stoul(b, nullptr, 0); out.emplace_back(m1,m2); } catch(...) {}
                    }
                }
            }
        } else {
            // CSV style
            stringstream ss(line); string a,b;
            if (getline(ss,a,',')) {
                if (getline(ss,b)) {
                    try { u16 m1 = (u16)stoul(a, nullptr, 0); u16 m2 = (u16)stoul(b, nullptr, 0); out.emplace_back(m1,m2); } catch(...) {}
                }
            }
        }
    }
    return true;
}

// write simple u16/u16 pairs binary
bool write_pairs_simple(const string &fname, const vector<pair<u16,u16>>& pairs) {
    ofstream ofs(fname, ios::binary);
    if (!ofs) return false;
    for (auto &p : pairs) {
        ofs.write(reinterpret_cast<const char*>(&p.first), sizeof(p.first));
        ofs.write(reinterpret_cast<const char*>(&p.second), sizeof(p.second));
    }
    return !ofs.fail();
}

// write optional C header for inspection
bool write_header(const string &fname, const vector<pair<u16,u16>>& pairs) {
    ofstream ofs(fname);
    if (!ofs) return false;
    ofs << "// Auto-generated pruned pairs\n#pragma once\n#include <stdint.h>\n\nstruct PairEntry { uint16_t mask1; uint16_t mask2; };\nstatic const PairEntry PRUNED_PAIRS[] = {\n";
    for (auto &p : pairs) {
        ofs << "  { 0x" << hex << setw(4) << setfill('0') << p.first << ", 0x" << setw(4) << p.second << " },\n" << dec;
    }
    ofs << "};\n";
    return true;
}

// ---------------------- metrics computation ----------------------
struct PairMetrics {
    double duty;        // overall duty
    double duty_std;    // sliding-window stddev
    double spec_entropy;
    double max_bin_frac;
    double lowfreq_frac;
};

// evaluate one seed pair: simulate up to maxSteps (LCM of individual periods capped), compute:
 // - bits stream
 // - sliding-window duty stddev
 // - spectral PSD -> entropy, max bin fraction, low-frequency fraction
PairMetrics evaluate_seed_pair(u16 mask1, u16 seed1, u16 mask2, u16 seed2, u32 maxSteps, u32 window, bool computeFFT=true) {
    // compute small period function
    auto lfsr_period = [&](u16 seed, u16 mask)->u32 {
        if (seed == 0) return 1;
        u16 s = seed; u32 cnt = 0;
        do {
            s = galois_step(s, mask);
            ++cnt;
            if (cnt >= maxSteps) return cnt;
        } while (s != seed);
        return cnt;
    };
    u32 p1 = lfsr_period(seed1, mask1);
    u32 p2 = lfsr_period(seed2, mask2);
    uint64_t L = lcm_u64(p1, p2);
    u32 simLen = (u32)min<uint64_t>(L==0 ? maxSteps : L, maxSteps);
    if (simLen < 16) simLen = min<uint32_t>(maxSteps, 16);

    vector<uint8_t> bits; bits.reserve(simLen);
    u16 s1 = seed1, s2 = seed2;
    u32 ones = 0;
    for (u32 i=0;i<simLen;++i) {
        s1 = galois_step(s1, mask1);
        s2 = galois_step(s2, mask2);
        uint8_t out = ((s1 ^ s2) & 1u) ? 1u : 0u;
        bits.push_back(out);
        ones += out;
    }
    double duty = (double)ones / (double)simLen;

    // sliding-window duty stddev
    vector<double> wduties;
    if (window >= 1 && window <= simLen) {
        u32 w = window;
        u32 cur = 0;
        for (u32 i=0;i<w;++i) cur += bits[i];
        wduties.push_back((double)cur / (double)w);
        for (u32 i=w;i<simLen;++i) { cur += bits[i]; cur -= bits[i-w]; wduties.push_back((double)cur / (double)w); }
    } else {
        wduties.push_back(duty);
    }
    double mean = 0.0;
    for (double v : wduties) mean += v;
    mean /= (double)wduties.size();
    double var = 0.0;
    for (double v : wduties) var += (v-mean)*(v-mean);
    var /= (double)wduties.size();
    double duty_std = sqrt(var);

    double spec_entropy_norm = 0.0, max_bin_frac = 0.0, lowfreq_frac = 0.0;
    if (computeFFT) {
        size_t N = next_pow2(bits.size());
        vector<cd> buf(N);
        for (size_t i=0;i<bits.size();++i) buf[i] = cd((double)bits[i], 0.0);
        for (size_t i=bits.size(); i<N; ++i) buf[i] = cd(0.0,0.0);
        fft(buf);
        size_t nbins = N/2;
        vector<double> psd(nbins, 0.0);
        double tot = 0.0;
        for (size_t i=0;i<nbins;++i) { double m = abs(buf[i]); double p = m*m; psd[i] = p; tot += p; }
        if (tot > 0.0) {
            spec_entropy_norm = spectral_entropy(psd);
            // skip DC (index 0) when measuring peaks/lowfreq
            double maxb = 0.0;
            for (size_t i=1;i<nbins;++i) if (psd[i] > maxb) maxb = psd[i];
            max_bin_frac = maxb / tot;
            size_t K = max<size_t>(1, nbins/64);
            double low = 0.0;
            for (size_t i=1;i<=K && i<nbins;++i) low += psd[i];
            lowfreq_frac = low / tot;
        }
    }

    return { duty, duty_std, spec_entropy_norm, max_bin_frac, lowfreq_frac };
}

// ---------------------- worker loop & main ----------------------
int main(int argc, char** argv) {
    if (argc == 1) { print_help(); return 0; }
    // parse args
    for (int i=1;i<argc;++i) {
        string a = argv[i];
        if (a == "--help" || a == "-h") { print_help(); return 0; }
        else if (a == "--cache" && i+1<argc) cfg.cache_file = argv[++i];
        else if (a == "--pairs" && i+1<argc) cfg.pairs_file = argv[++i];
        else if (a == "--out" && i+1<argc) cfg.out_pairs = argv[++i];
        else if (a == "--out-h" && i+1<argc) cfg.out_header = argv[++i];
        else if (a == "--threads" && i+1<argc) cfg.threads = stoull(argv[++i]);
        else if (a == "--max-steps" && i+1<argc) cfg.max_steps = (u32)stoul(argv[++i]);
        else if (a == "--window" && i+1<argc) cfg.window = (u32)stoul(argv[++i]);
        else if (a == "--entropy" && i+1<argc) cfg.entropy_thresh = stod(argv[++i]);
        else if (a == "--peak" && i+1<argc) cfg.peak_thresh = stod(argv[++i]);
        else if (a == "--lowfreq" && i+1<argc) cfg.lowfreq_thresh = stod(argv[++i]);
        else if (a == "--stability" && i+1<argc) cfg.stability_thresh = stod(argv[++i]);
        else if (a == "--seeds" && i+1<argc) {
            string s = argv[++i];
            cfg.test_seeds.clear();
            stringstream ss(s); string tok;
            while (getline(ss, tok, ',')) {
                if (tok.empty()) continue;
                try { u16 v = (u16)stoul(tok, nullptr, 0); cfg.test_seeds.push_back(v); } catch(...) {}
            }
        } else {
            cerr << "Unknown arg: " << a << "\n"; print_help(); return 1;
        }
    }

    if (cfg.pairs_file.empty()) { cerr << "Missing --pairs <file>\n"; return 1; }

    // load mask cache if available (informational)
    vector<MaskCacheRecord> cache;
    bool have_cache = read_cache(cfg.cache_file, cache);
    if (have_cache && cfg.verbose) cerr << "Loaded mask cache entries: " << cache.size() << "\n";
    else if (cfg.verbose) cerr << "No mask cache loaded or not found (continuing)\n";

    // read pairs
    vector<pair<u16,u16>> pairs;
    if (!read_pairs_multi(cfg.pairs_file, pairs)) { cerr << "Failed to read pairs file\n"; return 1; }
    if (cfg.verbose) cerr << "Loaded pairs: " << pairs.size() << "\n";
    if (pairs.empty()) { cerr << "No pairs to process\n"; return 1; }

    // default seeds if none provided
    if (cfg.test_seeds.empty()) cfg.test_seeds = { 1u, 0xACE1u, 0xBEEFu, 0x1234u, 0xFFFFu };

    vector<pair<u16,u16>> accepted;
    accepted.reserve(pairs.size()/10);
    mutex accept_mu;

    atomic<size_t> index{0}, processed{0};
    size_t total = pairs.size();
    size_t nth = max<size_t>(1, cfg.threads);

    // worker function: evaluate assigned pairs; accept if any seed pair passes all tests
    auto worker = [&]() {
        size_t i;
        while ((i = index.fetch_add(1)) < total) {
            u16 m1 = pairs[i].first, m2 = pairs[i].second;
            bool keep = false;
            for (u16 s1 : cfg.test_seeds) {
                if (s1 == 0) continue;
                for (u16 s2 : cfg.test_seeds) {
                    if (s2 == 0) continue;
                    PairMetrics pm = evaluate_seed_pair(m1, s1, m2, s2, cfg.max_steps, cfg.window, true);
                    if (pm.duty_std <= cfg.stability_thresh &&
                        pm.spec_entropy >= cfg.entropy_thresh &&
                        pm.max_bin_frac <= cfg.peak_thresh &&
                        pm.lowfreq_frac <= cfg.lowfreq_thresh) {
                        keep = true;
                        break;
                    }
                }
                if (keep) break;
            }
            if (keep) {
                lock_guard lk(accept_mu);
                accepted.emplace_back(m1, m2);
            }
            size_t done = ++processed;
            if (cfg.verbose && (done & 0x3FF) == 0) {
                double pct = 100.0 * (double)done / (double)total;
                cerr << "\rProgress: " << done << "/" << total << " (" << fixed << setprecision(2) << pct << "%)    " << flush;
            }
        }
    };

    // spawn threads
    vector<thread> workers;
    for (size_t t = 0; t < nth; ++t) workers.emplace_back(worker);
    for (auto &th : workers) if (th.joinable()) th.join();
    if (cfg.verbose) cerr << "\nAccepted pairs: " << accepted.size() << "\n";

    // dedupe and sort for stable output
    sort(accepted.begin(), accepted.end());
    accepted.erase(unique(accepted.begin(), accepted.end()), accepted.end());
    if (cfg.verbose) cerr << "Unique accepted pairs: " << accepted.size() << "\n";

    // write simple binary output
    if (!write_pairs_simple(cfg.out_pairs, accepted)) { cerr << "Failed to write output pairs: " << cfg.out_pairs << "\n"; return 1; }
    if (cfg.verbose) cerr << "Pruned pairs written: " << cfg.out_pairs << "\n";

    // optional header
    if (!cfg.out_header.empty()) {
        if (!write_header(cfg.out_header, accepted)) { cerr << "Failed to write header: " << cfg.out_header << "\n"; return 1; }
        if (cfg.verbose) cerr << "C header written: " << cfg.out_header << "\n";
    }

    return 0;
}


---

If you want next:

I can add a --best-seed-scan mode that exhaustively searches seeds (or searches until a pair of seeds passing metrics is found) and writes the accepted pairs in a legacy PAPR layout including the best seed1/seed2 — useful for making the MCU load pairs + seed values directly.

Or I can add a CSV/JSON metrics export for offline inspection (per-pair entropy, max-peak, lowfreq, duty_std) so you can tune thresholds with real numbers.


Which enhancement should I add?
