// lfsr_morphism_search.cpp
// Config-driven LFSR PWM search with GF(2) polynomial factoring, morphism-derived seeds,
// AVX2-accelerated popcount window checks, Arduino LUT export.
//
// Key fixes:
//  - Keep top-K candidates per bin during search (thread-safe).
//  - Final verification pass after search: re-enumerate candidates under hardware comparator (threshold==seed),
//    compute exact full-cycle duty if possible (fall back to sampling when necessary), run sliding-window check
//    with desired bin center, and pick the candidate with smallest duty error (and acceptable block error).
//
// Hardware semantics preserved: comparator uses threshold == seed, output-before-step.

#include <bits/stdc++.h>
#include <immintrin.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include "nlohmann/json.hpp"
#include "cpuid.h"

using json = nlohmann::json;
using u64 = uint64_t;
using u32 = uint32_t;

struct Config {
    int n = 16;
    int resolution = 256;
    int threads = 0;
    u64 max_masks = 0;
    int min_taps = 1;
    double min_period_ratio = 0.0;
    int min_sample_bits = 1024;
    int max_sample_bits = 16384;
    int block_size = 256;
    double max_block_error = 0.1;
    double lc_weight = 0.01;
    double period_weight = 0.001;
    double spectral_weight = 10.0;
    double duty_weight = 1000.0; // used during search ranking
    int top_k = 8; // keep top-K candidates per bin
    std::string out_dir = "8bit_set";
    bool export_bins = true;
    unsigned seed = 0;
    std::string simd_preference = "auto";
    int seeds_per_mask = 16;
    bool final_verify_full_period = false;
};

static Config load_config(const std::string &path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("cannot open config file");
    json j; f >> j;
    Config c;
    if (j.contains("engine")) {
        auto e = j["engine"];
        if (e.contains("n")) c.n = e["n"];
        if (e.contains("resolution")) c.resolution = e["resolution"];
        if (e.contains("threads")) c.threads = e["threads"];
        if (e.contains("seed")) c.seed = e["seed"];
    }
    if (j.contains("search")) {
        auto s = j["search"];
        if (s.contains("max_masks")) c.max_masks = s["max_masks"];
        if (s.contains("min_taps")) c.min_taps = s["min_taps"];
        if (s.contains("min_period_ratio")) c.min_period_ratio = s["min_period_ratio"];
        if (s.contains("seeds_per_mask")) c.seeds_per_mask = s["seeds_per_mask"];
        if (s.contains("top_k")) c.top_k = s["top_k"];
    }
    if (j.contains("sampling")) {
        auto s = j["sampling"];
        if (s.contains("min_sample_bits")) c.min_sample_bits = s["min_sample_bits"];
        if (s.contains("max_sample_bits")) c.max_sample_bits = s["max_sample_bits"];
        if (s.contains("block_size")) c.block_size = s["block_size"];
        if (s.contains("max_block_error")) c.max_block_error = s["max_block_error"];
    }
    if (j.contains("scoring")) {
        auto s = j["scoring"];
        if (s.contains("lc_weight")) c.lc_weight = s["lc_weight"];
        if (s.contains("period_weight")) c.period_weight = s["period_weight"];
        if (s.contains("spectral_weight")) c.spectral_weight = s["spectral_weight"];
        if (s.contains("duty_weight")) c.duty_weight = s["duty_weight"];
    }
    if (j.contains("export")) {
        auto e = j["export"];
        if (e.contains("out_dir")) c.out_dir = e["out_dir"];
        if (e.contains("export_bins")) c.export_bins = e["export_bins"];
    }
    if (j.contains("hardware")) {
       auto h = j["hardware"];
       if (h.contains("simd_preference") && h["simd_preference"].is_string())
        c.simd_preference = h["simd_preference"];
    }

    if (c.threads <= 0) c.threads = std::max(1u, std::thread::hardware_concurrency());
    c.min_sample_bits = std::clamp(c.min_sample_bits, 64, 16384);
    c.max_sample_bits = std::clamp(c.max_sample_bits, c.min_sample_bits, 65536);
    return c;
}

// CPU detection
struct CPUFeatures { bool avx2=false; };
static CPUFeatures detect_cpu() {
    CPUFeatures f{};
#if defined(__x86_64__) || defined(__i386__)
    unsigned int a,b,c,d;
    if (__get_cpuid_max(0, nullptr) >= 7) {
        __cpuid_count(7, 0, a, b, c, d);
        f.avx2 = (b & (1<<5)) != 0;
    }
#endif
    return f;
}

// GF(2) helpers (as before)
static inline int poly_deg(u64 p) {
    if (p==0) return -1;
    return 63 - __builtin_clzll(p);
}
static inline u64 poly_mul(u64 a, u64 b) {
    u64 r = 0;
    while (b) {
        if (b & 1) r ^= a;
        b >>= 1;
        a <<= 1;
    }
    return r;
}
static inline u64 poly_mod(u64 a, u64 mod) {
    int dm = poly_deg(mod);
    if (dm < 0) return 0;
    while (poly_deg(a) >= dm) {
        int shift = poly_deg(a) - dm;
        a ^= (mod << shift);
    }
    return a;
}
static inline u64 poly_mulmod(u64 a, u64 b, u64 mod) {
    return poly_mod(poly_mul(a,b), mod);
}
static inline u64 poly_powmod(u64 a, u64 e, u64 mod) {
    u64 r = 1;
    while (e) {
        if (e & 1) r = poly_mulmod(r, a, mod);
        a = poly_mulmod(a, a, mod);
        e >>= 1;
    }
    return r;
}
static inline u64 poly_gcd(u64 a, u64 b) {
    while (b) {
        int da = poly_deg(a), db = poly_deg(b);
        if (da < db) std::swap(a,b), std::swap(da,db);
        a ^= (b << (da - db));
        int dda = poly_deg(a);
        if (dda < 0) return b;
    }
    return a;
}
static bool is_irreducible(u64 f) {
    int d = poly_deg(f);
    if (d <= 0) return false;
    u64 x = 2; u64 xp = x;
    for (int i = 1; i <= d/1; ++i) {
        xp = poly_powmod(xp, 2, f);
        u64 g = poly_gcd(xp ^ x, f);
        if (g != 1 && g != f) return false;
    }
    return true;
}

// Berlekamp factorization (kept as-is)
static std::pair<u64,u64> poly_divmod(u64 a, u64 b) { if (b==0) throw std::runtime_error("poly_divmod"); int db=poly_deg(b); u64 q=0, r=a; while(true){int dr=poly_deg(r); if(dr<db||dr<0) break; int shift=dr-db; q |= (1ULL<<shift); r ^= (b<<shift);} return {q,r}; }
static u64 poly_div_exact(u64 a,u64 b){ auto pr=poly_divmod(a,b); if (pr.second!=0) return 0ULL; return pr.first; }
static inline u64 poly_from_bits(u64 v){ return v; }

static std::vector<u64> berlekamp_nullspace_basis(u64 f) {
    int n = poly_deg(f); if (n<=0) return {};
    std::vector<u64> basis(n);
    u64 xpoly = 2ULL; u64 cur = xpoly;
    for (int c=0;c<n;++c) { if (c==0) cur = xpoly % f; else cur = poly_powmod(cur,2ULL,f); basis[c]=cur; }
    std::vector<u64> M(n,0ULL);
    for (int r=0;r<n;++r) {
        u64 row=0;
        for (int c=0;c<n;++c) if ((basis[c]>>r)&1ULL) row |= (1ULL<<c);
        row ^= (1ULL<<r);
        M[r]=row;
    }
    std::vector<u64> A=M; std::vector<int> pivot_col(n,-1);
    int row=0;
    for (int col=0; col<n && row<n; ++col) {
        int sel=-1;
        for (int r=row;r<n;++r) if ((A[r]>>col)&1ULL){sel=r;break;}
        if (sel==-1) continue;
        std::swap(A[row], A[sel]);
        pivot_col[row]=col;
        for (int r=0;r<n;++r) if (r!=row && ((A[r]>>col)&1ULL)) A[r]^=A[row];
        ++row;
    }
    int rank=row;
    std::vector<int> is_pivot(n,0);
    for (int r=0;r<rank;++r) if (pivot_col[r]>=0) is_pivot[pivot_col[r]]=1;
    std::vector<int> free_cols;
    for (int c=0;c<n;++c) if (!is_pivot[c]) free_cols.push_back(c);
    std::vector<u64> nullvecs;
    for (int fc: free_cols) {
        u64 vec = 0; vec |= (1ULL<<fc);
        for (int r = rank-1; r>=0; --r) {
            int pc = pivot_col[r];
            if (pc < 0) continue;
            u64 mask = A[r] & ~(1ULL << pc);
            int parity = __builtin_parityll(mask & vec);
            if (parity) vec |= (1ULL << pc);
        }
        nullvecs.push_back(vec);
    }
    return nullvecs;
}
static void berlekamp_factor_rec(u64 f, std::vector<u64> &out, std::mt19937_64 &rng) {
    if (f==0||f==1) return;
    if (is_irreducible(f)) { out.push_back(f); return; }
    int n=poly_deg(f); if (n<=1) { out.push_back(f); return; }
    auto nullvecs = berlekamp_nullspace_basis(f);
    if (nullvecs.empty()) { out.push_back(f); return; }
    std::uniform_int_distribution<uint64_t> ud(1, UINT64_MAX);
    const int MAX_TRIES=60;
    for (int attempt=0; attempt<MAX_TRIES; ++attempt) {
        u64 comb = 0;
        for (size_t i=0;i<nullvecs.size();++i) if ((ud(rng)&0xFFFFFFFFULL)&1ULL) comb ^= nullvecs[i];
        if (comb==0) comb = nullvecs[ud(rng)%nullvecs.size()];
        if ((ud(rng) & 0x3ULL)==0) comb ^= 1ULL;
        u64 a = poly_from_bits(comb) & ((n>=64) ? ~0ULL : ((1ULL<<n)-1));
        if (a==0||a==1) continue;
        u64 g = poly_gcd(a,f);
        if (g!=1 && g!=f) {
            u64 q = poly_div_exact(f,g);
            if (q==0) {
                auto pr = poly_divmod(f,g);
                if (pr.second != 0) { out.push_back(f); return; }
                q = pr.first;
            }
            berlekamp_factor_rec(g,out,rng);
            berlekamp_factor_rec(q,out,rng);
            return;
        }
    }
    for (u64 v : nullvecs) {
        u64 a = poly_from_bits(v) & ((poly_deg(f) >= 64) ? ~0ULL : ((1ULL<<poly_deg(f))-1));
        if (a==0||a==1) continue;
        u64 g = poly_gcd(a,f);
        if (g!=1 && g!=f) {
            u64 q = poly_div_exact(f,g);
            if (q==0) {
                auto pr = poly_divmod(f,g);
                if (pr.second != 0) { out.push_back(f); return; }
                q = pr.first;
            }
            berlekamp_factor_rec(g,out,rng);
            berlekamp_factor_rec(q,out,rng);
            return;
        }
    }
    out.push_back(f);
}
static std::vector<u64> berlekamp_factor(u64 f) {
    std::vector<u64> res;
    if (f==0||f==1) return res;
    while ((f & 1ULL) == 0ULL) { res.push_back(2ULL); f >>= 1; if (f==0) return res; }
    std::mt19937_64 rng((uint64_t)123456789);
    berlekamp_factor_rec(f,res,rng);
    std::sort(res.begin(), res.end(), [](u64 a,u64 b){ return poly_deg(a) < poly_deg(b); });
    return res;
}

// Berlekamp-Massey (as before)
int berlekamp_massey(const std::vector<uint8_t> &bits) {
    int n = (int)bits.size();
    if (n==0) return 0;
    std::vector<uint8_t> C(n,0), B(n,0);
    C[0]=1; B[0]=1;
    int L=0, m=-1;
    for (int N=0; N<n; ++N) {
        uint8_t d = bits[N];
        for (int i=1;i<=L;++i) d ^= (C[i] & bits[N-i]);
        if (d) {
            std::vector<uint8_t> T=C;
            int p = N - m;
            for (int i=0;i+p<n;++i) C[i+p] ^= B[i];
            if (L <= N/2) { L = N + 1 - L; m = N; B = T; }
        }
    }
    return L;
}

// LFSR step
static inline u64 lfsr_next(u64 state, u64 mask, int n) {
    u64 fb = __builtin_parityll(state & mask);
    return (state >> 1) | (fb << (n - 1));
}

// popcount helpers (AVX2 fast path kept)
static uint64_t popcount_bytes_avx2(const uint8_t* data, size_t len) {
#if defined(__AVX2__)
    const __m256i lut = _mm256_setr_epi8(
         0,1,1,2,1,2,2,3,1,2,2,3,2,3,3,4,
         0,1,1,2,1,2,2,3,1,2,2,3,2,3,3,4
    );
    const __m256i low_mask = _mm256_set1_epi8(0x0F);
    uint64_t total = 0;
    size_t i = 0;
    while (len - i >= 32) {
        __m256i v = _mm256_loadu_si256((const __m256i*)(data + i));
        __m256i lo = _mm256_and_si256(v, low_mask);
        __m256i hi = _mm256_and_si256(_mm256_srli_epi16(v, 4), low_mask);
        __m256i plo = _mm256_shuffle_epi8(lut, lo);
        __m256i phi = _mm256_shuffle_epi8(lut, hi);
        __m256i pop8 = _mm256_add_epi8(plo, phi);
        __m256i sad = _mm256_sad_epu8(pop8, _mm256_setzero_si256());
        total += (uint64_t)_mm256_extract_epi64(sad, 0)
               + (uint64_t)_mm256_extract_epi64(sad, 1)
               + (uint64_t)_mm256_extract_epi64(sad, 2)
               + (uint64_t)_mm256_extract_epi64(sad, 3);
        i += 32;
    }
    while (i < len) { total += __builtin_popcount((unsigned)data[i]); ++i; }
    return total;
#else
    uint64_t s = 0;
    for (size_t i=0;i<len;++i) s += (uint64_t)__builtin_popcount((unsigned)data[i]);
    return s;
#endif
}
static inline uint64_t buffer_popcount(const uint8_t* data, size_t len, const CPUFeatures &f) {
    if (f.avx2 && len >= 32) return popcount_bytes_avx2(data, len);
    uint64_t s = 0;
    for (size_t i=0;i<len;++i) s += (uint64_t)__builtin_popcount((unsigned)data[i]);
    return s;
}

// sliding-window check (no heap-defaults)
bool sliding_window_check_sampled(const std::vector<uint8_t> &ones_flags, int period,
                                  int block_size, double target, const CPUFeatures &f,
                                  int check_limit, double max_allowed_block_error,
                                  double &out_max_block_error, double &out_max_block_delta) {
    if (period <= 0) { out_max_block_error = 1.0; out_max_block_delta = 1.0; return false; }
    int g = std::gcd(period, block_size); if (g < 1) g = 1;
    std::vector<int> starts; starts.reserve(std::min(period, check_limit + g));
    for (int r=0; r<g && (int)starts.size() < check_limit; ++r) starts.push_back(r);
    for (int r=0; (int)starts.size() < check_limit && r < period; ++r) {
        int val = (r * g) % period; starts.push_back(val);
    }
    std::sort(starts.begin(), starts.end());
    starts.erase(std::unique(starts.begin(), starts.end()), starts.end());
    if ((int)starts.size() > check_limit) starts.resize(check_limit);

    double prev_block = -1.0;
    double max_block_err = 0.0, max_block_delta = 0.0;
    for (int s : starts) {
        int a = s; int b = s + block_size;
        uint64_t ones = 0;
        if (b <= period) {
            ones = buffer_popcount(ones_flags.data() + a, block_size, f);
        } else {
            int len1 = period - a;
            ones = buffer_popcount(ones_flags.data() + a, len1, f);
            int len2 = b - period;
            ones += buffer_popcount(ones_flags.data(), len2, f);
        }
        double block_duty = (double)ones / (double)block_size;
        double block_err = fabs(block_duty - target);
        if (block_err > max_block_err) max_block_err = block_err;
        if (prev_block >= -0.5) {
            double d = fabs(block_duty - prev_block);
            if (d > max_block_delta) max_block_delta = d;
        }
        prev_block = block_duty;
        if (block_err > max_allowed_block_error) {
            out_max_block_error = max_block_err;
            out_max_block_delta = max_block_delta;
            return false;
        }
    }
    out_max_block_error = max_block_err;
    out_max_block_delta = max_block_delta;
    return true;
}

// enumerate cycle & count outputs (output-before-step semantics)
static int enumerate_cycle_and_count(u64 init_state, u64 mask, int n,
                                     const std::function<uint8_t(u64)>& comparator,
                                     uint64_t max_steps, uint64_t &out_ones) {
    if (max_steps == 0) return -1;
    u64 s = init_state; out_ones = 0;
    for (uint64_t step = 0; step < max_steps; ++step) {
        uint8_t b = comparator(s);
        out_ones += b;
        s = lfsr_next(s, mask, n);
        if (s == init_state) return (int)(step + 1);
    }
    return -1;
}

// comparator factory
static inline std::function<uint8_t(u64)> make_threshold_comparator(u64 threshold, int n) {
    u64 mask_n = (n==64) ? ~0ULL : ((1ULL<<n)-1);
    threshold &= mask_n;
    return [threshold, mask_n](u64 s)->uint8_t { return (uint8_t)(((s & mask_n) > threshold) ? 1 : 0); };
}

// config structures for candidates
struct LFSRConfig {
    u64 mask=0;
    u64 seed=0;
    int period=0;
    int lc=0;
    double actual_duty=0.0;
    double error=0.0;
    double score=0.0;
    double low_freq_energy=0.0;
};

// export helpers
void ensure_dir(const std::string &d) {
    struct stat st;
    if (stat(d.c_str(), &st) == -1) mkdir(d.c_str(), 0755);
}
void export_candidate_simple(const std::string &dir, int idx, u64 mask, u64 seed) {
    ensure_dir(dir);
    char path[512];
    snprintf(path, sizeof(path), "%s/0x%02X.bin", dir.c_str(), idx & 0xFF);
    FILE *f = fopen(path, "ab");
    if (!f) return;
    uint16_t m16 = (uint16_t)mask, s16 = (uint16_t)seed;
    fwrite(&m16, sizeof(m16), 1, f);
    fwrite(&s16, sizeof(s16), 1, f);
    fclose(f);
}

double compute_low_freq_energy_sample(const std::vector<int> &bits, int period, int harmonics=5) {
    if (period < harmonics*2) return 0.0;
    double total = 0.0;
    for (int k=1;k<=harmonics;++k) {
        double re=0, im=0;
        double step = -2.0 * M_PI * k / (double)period;
        for (int n=0;n<period;++n) {
            double v = (double)bits[n] - 0.5;
            double theta = step * n;
            re += v * cos(theta);
            im += v * sin(theta);
        }
        double mag = sqrt(re*re + im*im) / (double)period;
        total += mag*mag;
    }
    return total;
}

std::vector<u64> generate_morphism_seeds(u64 conn_poly, int n, int wanted) {
    std::vector<u64> seeds;
    u64 mask_n = (n==64) ? ~0ULL : ((1ULL<<n)-1);
    u64 cur = 1ULL; // x^0
    for (int k=1; (int)seeds.size() < wanted && k < 4 * wanted; ++k) {
        cur = poly_mulmod(cur, 2ULL, conn_poly);
        u64 seed = cur & mask_n;
        if (seed != 0 && std::find(seeds.begin(), seeds.end(), seed) == seeds.end()) seeds.push_back(seed);
    }
    for (u64 s = 1; (int)seeds.size() < wanted && s < (1ULL<<std::min(20,n)); ++s) {
        if (std::find(seeds.begin(), seeds.end(), s) == seeds.end()) seeds.push_back(s & mask_n);
    }
    return seeds;
}

static inline int popcount_u64(u64 x) { return __builtin_popcountll(x); }

// MAIN
int main(int argc, char **argv) {
    if (argc < 2) { std::cerr << "Usage: " << argv[0] << " config.json\n"; return 1; }
    Config cfg = load_config(argv[1]);
    CPUFeatures cf = detect_cpu();

    std::cerr << "n=" << cfg.n << " res=" << cfg.resolution << " threads=" << cfg.threads << " top_k=" << cfg.top_k << "\n";
    if (cfg.seed == 0) cfg.seed = (unsigned)std::chrono::system_clock::now().time_since_epoch().count();
    std::mt19937_64 rng(cfg.seed);

    // consistent target mapping: use denominator (resolution - 1) so endpoints are 0..1 inclusive
    std::vector<double> targets(cfg.resolution);
    for (int i=0;i<cfg.resolution;++i) targets[i] = (double)i / (double)(cfg.resolution - 1);

    u64 max_mask = (1ULL << cfg.n);
    u64 scan_limit = (cfg.max_masks == 0) ? max_mask : std::min(cfg.max_masks, max_mask);
    std::vector<u64> masks;
    masks.reserve(static_cast<size_t>(std::max<u64>(1, scan_limit - 1)));
    for (u64 m=1; m<scan_limit; ++m) if (popcount_u64(m) >= cfg.min_taps) masks.push_back(m);
    std::shuffle(masks.begin(), masks.end(), rng);

    if (cfg.export_bins) ensure_dir(cfg.out_dir);

    // top-K candidates per bin (thread-safe via merge_mutex)
    std::vector<std::vector<LFSRConfig>> top_candidates(cfg.resolution);
    std::mutex merge_mutex;
    std::atomic<size_t> processed_masks{0};
    std::vector<uint32_t> candidate_counts(cfg.resolution, 0);

    // Worker searches and populates top_candidates
    auto worker = [&](size_t start, size_t end){
        u64 mask_n = (cfg.n==64) ? ~0ULL : ((1ULL<<cfg.n)-1);
        for (size_t idx = start; idx < end; ++idx) {
            u64 mask = masks[idx];
            u64 conn = (1ULL << cfg.n);
            for (int i=0;i<cfg.n;++i) if ((mask>>i)&1ULL) conn |= (1ULL<<i);

            std::vector<u64> factors = berlekamp_factor(conn);
            uint64_t lcm = 1;
            for (u64 fac : factors) {
                int d = poly_deg(fac); if (d<=0) continue;
                uint64_t bound = ((1ULL<<d) - 1);
                uint64_t g = std::gcd(lcm, bound);
                if (bound / g > 0 && lcm <= UINT64_MAX / (bound / g)) lcm = (lcm / g) * bound;
                else lcm = std::numeric_limits<uint64_t>::max();
            }
            double ratio = (double)lcm / (double)((1ULL<<cfg.n));
            if (cfg.min_period_ratio > 0.0 && ratio < cfg.min_period_ratio) { processed_masks.fetch_add(1); continue; }

            std::vector<u64> seeds = generate_morphism_seeds(conn, cfg.n, cfg.seeds_per_mask);
            for (u64 seed : seeds) {
                if (seed == 0) continue;
                // comparator = threshold == seed (hardware)
                auto comparator = make_threshold_comparator(seed, cfg.n);

                // enumeration budget (as before)
                uint64_t max_enum = (uint64_t)cfg.max_sample_bits * 8ULL;
                if (lcm != std::numeric_limits<uint64_t>::max() && lcm > 0 && lcm <= (1ULL<<22)) max_enum = lcm;
                else {
                    if (cfg.n <= 20) max_enum = std::max<uint64_t>(max_enum, (1ULL<<20));
                    else if (cfg.n <= 24) max_enum = std::max<uint64_t>(max_enum, (1ULL<<18));
                }
                const uint64_t HARD_ENUM_CAP = (1ULL<<22);
                if (max_enum > HARD_ENUM_CAP) max_enum = HARD_ENUM_CAP;

                uint64_t ones_count = 0;
                int period = enumerate_cycle_and_count(seed, mask, cfg.n, comparator, max_enum, ones_count);

                std::vector<uint8_t> ones;
                double measured_duty = 0.0;

                if (period > 0) {
                    measured_duty = (double)ones_count / (double)period;
                    if ((uint64_t)period <= (uint64_t)cfg.max_sample_bits) {
                        ones.reserve(period);
                        u64 s2 = seed;
                        for (int t=0;t<period;++t) { ones.push_back(comparator(s2)); s2 = lfsr_next(s2, mask, cfg.n); }
                    } else {
                        int sample_len = std::min(cfg.max_sample_bits, std::max(cfg.min_sample_bits, cfg.n * 8));
                        sample_len = std::min(sample_len, 16384);
                        ones.clear(); ones.reserve(sample_len);
                        u64 s2 = seed; uint64_t ones_total_sample = 0;
                        for (int t=0;t<sample_len;++t) { uint8_t b = comparator(s2); ones.push_back(b); ones_total_sample += b; s2 = lfsr_next(s2, mask, cfg.n); }
                        if (ones_count == 0) measured_duty = (double)ones_total_sample / (double)sample_len;
                    }
                } else {
                    int sample_len = std::min(cfg.max_sample_bits, std::max(cfg.min_sample_bits, cfg.n * 8));
                    sample_len = std::min(sample_len, 16384);
                    ones.clear(); ones.reserve(sample_len);
                    u64 s2 = seed; uint64_t ones_total = 0;
                    for (int t=0;t<sample_len;++t) { uint8_t b = comparator(s2); ones.push_back(b); ones_total += b; s2 = lfsr_next(s2, mask, cfg.n); }
                    measured_duty = (double)ones_total / (double)sample_len;
                    period = sample_len;
                }

                // require ones non-empty
                if (ones.empty()) continue;

                // sliding-window check against measured duty? keep check against measured duty to avoid accepting obviously wrong streams
                double max_block_err = 0.0, max_block_delta = 0.0;
                bool pass = sliding_window_check_sampled(ones, period, cfg.block_size, measured_duty, cf, 4096, cfg.max_block_error, max_block_err, max_block_delta);
                if (!pass) continue;

                int lc = berlekamp_massey(ones);

                int sample_for_spectrum = (int)ones.size() >= period && period>0 ? period : (int)ones.size();
                std::vector<int> intbits(sample_for_spectrum);
                for (int i=0;i<sample_for_spectrum;++i) intbits[i] = ones[i];
                double lf = compute_low_freq_energy_sample(intbits, sample_for_spectrum, 5);

                double lc_term = (double)lc / std::max(1, cfg.n);
                double period_term = (lcm>0 && lcm < UINT64_MAX) ? (log2((double)std::max<uint64_t>(1,lcm)) / (double)std::max(1,cfg.n)) : 0.0;
                double lc_bonus = cfg.lc_weight * lc_term;
                double period_bonus = cfg.period_weight * period_term;
                double spectral_penalty = cfg.spectral_weight * lf;

                // **Important**: we compute score using duty error vs *measured* duty only for ranking,
                // the final selection uses explicit verification against target.
                // But measured_duty maps to nearest bin to know where to insert.
                int bin = (int)std::round(measured_duty * (cfg.resolution - 1));
                bin = std::clamp(bin, 0, cfg.resolution - 1);

                // candidate score (for ranking) uses measured duty error vs bin center
                double desired = (double)bin / (double)(cfg.resolution - 1);
                double duty_error = fabs(measured_duty - desired);
                double score = duty_error * cfg.duty_weight + max_block_err - lc_bonus - period_bonus + spectral_penalty;

                LFSRConfig cand;
                cand.mask = mask; cand.seed = seed; cand.period = period; cand.lc = lc;
                cand.actual_duty = measured_duty; cand.error = max_block_err; cand.score = score; cand.low_freq_energy = lf;

                // insert into top-K list for bin
                {
                    std::lock_guard<std::mutex> lock(merge_mutex);
                    candidate_counts[bin]++;
                    auto &vec = top_candidates[bin];
                    vec.push_back(cand);
                    // keep sorted ascending by score, trim to top_k
                    std::sort(vec.begin(), vec.end(), [](const LFSRConfig &a, const LFSRConfig &b){
                        return a.score < b.score;
                    });
                    if ((int)vec.size() > cfg.top_k) vec.resize(cfg.top_k);
                    if (cfg.export_bins) export_candidate_simple(cfg.out_dir, bin, mask, seed);
                }
            } // seeds
            size_t p = processed_masks.fetch_add(1) + 1;
            if ((p % 1000) == 0) std::cerr << "Processed " << p << " / " << masks.size() << " masks\r" << std::flush;
        }
    };

    // launch threads
    size_t per = (masks.size() + cfg.threads - 1) / cfg.threads;
    std::vector<std::thread> threads;
    for (int t=0;t<cfg.threads;++t) {
        size_t s = t*per; size_t e = std::min(masks.size(), s + per);
        if (s < e) threads.emplace_back(worker, s, e);
    }
    for (auto &th : threads) th.join();

    std::cerr << "\nSearch complete. Verifying top candidates per bin...\n";

    // final verification & selection: re-evaluate each top candidate under comparator==seed, pick the candidate with smallest duty error
    std::vector<LFSRConfig> final_best(cfg.resolution);
    for (int i=0;i<cfg.resolution;++i) final_best[i].score = std::numeric_limits<double>::infinity();

    u64 mask_n = (cfg.n==64) ? ~0ULL : ((1ULL<<cfg.n)-1);
    const uint64_t FINAL_ENUM_CAP = (1ULL<<22); // safe cap for re-enumeration (~4M)
    double tolerance = 0.5 / (double)(cfg.resolution - 1); // half-bin tolerance

    for (int bin=0; bin<cfg.resolution; ++bin) {
        double desired = targets[bin];
        auto &vec = top_candidates[bin];
        if (vec.empty()) continue;

        // track best verification result for this bin
        bool found_valid = false;
        double best_err = 1e9;
        LFSRConfig chosen;

        for (auto &cand : vec) {
            // re-create comparator from seed (hardware semantics)
            auto comparator = make_threshold_comparator(cand.seed, cfg.n);

            // try to enumerate full cycle (allow larger cap but bounded)
            uint64_t ones_count = 0;
            uint64_t cap = std::min<uint64_t>(FINAL_ENUM_CAP, std::max<uint64_t>((uint64_t)cand.period, (uint64_t)cfg.max_sample_bits * 16ULL));
            int period = enumerate_cycle_and_count(cand.seed, cand.mask, cfg.n, comparator, cap, ones_count);

            std::vector<uint8_t> ones;
            double measured_duty = 0.0;
            if (period > 0) {
                measured_duty = (double)ones_count / (double)period;
                // reconstruct ones fully if period within allowed
                if ((uint64_t)period <= (uint64_t)cfg.max_sample_bits) {
                    ones.reserve(period);
                    u64 s2 = cand.seed;
                    for (int t=0;t<period;++t) { ones.push_back(comparator(s2)); s2 = lfsr_next(s2, cand.mask, cfg.n); }
                } else {
                    // sample
                    int sample_len = std::min(cfg.max_sample_bits, std::max(cfg.min_sample_bits, cand.period));
                    sample_len = std::min(sample_len, 16384);
                    ones.clear(); ones.reserve(sample_len);
                    u64 s2 = cand.seed;
                    uint64_t ones_total_sample = 0;
                    for (int t=0;t<sample_len;++t) { uint8_t b = comparator(s2); ones.push_back(b); ones_total_sample += b; s2 = lfsr_next(s2, cand.mask, cfg.n); }
                    if ((uint64_t)ones_count == 0) measured_duty = (double)ones_total_sample / (double)sample_len;
                    period = sample_len;
                }
            } else {
                // fallback to sampling
                int sample_len = std::min(cfg.max_sample_bits, std::max(cfg.min_sample_bits, cand.period));
                sample_len = std::min(sample_len, 16384);
                ones.clear(); ones.reserve(sample_len);
                u64 s2 = cand.seed; uint64_t ones_total = 0;
                for (int t=0;t<sample_len;++t) { uint8_t b = comparator(s2); ones.push_back(b); ones_total += b; s2 = lfsr_next(s2, cand.mask, cfg.n); }
                measured_duty = (double)ones_total / (double)sample_len;
                period = sample_len;
            }

            if (ones.empty()) continue;

            // sliding-window check using *desired* target on stream produced by comparator(seed)
            double max_block_err = 0.0, max_block_delta = 0.0;
            bool pass = sliding_window_check_sampled(ones, period, cfg.block_size, desired, cf, 4096, cfg.max_block_error, max_block_err, max_block_delta);
            if (!pass) continue;

            // compute final duty error
            double duty_err = fabs(measured_duty - desired);

            // choose candidate with minimal duty_err (tie-breaker in score)
            if (!found_valid || duty_err < best_err - 1e-12 || (fabs(duty_err - best_err) < 1e-12 && cand.score < chosen.score)) {
                found_valid = true;
                best_err = duty_err;
                chosen = cand;
                chosen.actual_duty = measured_duty;
                chosen.error = max_block_err;
            }

            // early accept if within half-bin tolerance
            if (duty_err <= tolerance) break;
        } // per-candidate

        if (found_valid) {
            final_best[bin] = chosen;
        } else {
            // leave as infinity / empty - no candidate passed verification
            final_best[bin].score = std::numeric_limits<double>::infinity();
        }
    } // per-bin

    // Write LUT: leave struct as {mask, seed} to match previous format
    std::ofstream f("lfsr_pwm_lut.h");
    if (!f) { std::cerr << "cannot write LUT\n"; return 1; }
    f << "// Generated LUT (n=" << cfg.n << ")\n#include <stdint.h>\n";
    if (cfg.n <= 16) {
        f << "const struct { uint16_t mask; uint16_t seed; } lfsr_pwm_lut[] = {\n";
        for (int i=0;i<cfg.resolution;++i) {
            if (final_best[i].score == std::numeric_limits<double>::infinity()) {
                f << "{0,0},\n";
            } else {
                f << "{0x" << std::hex << (uint16_t)final_best[i].mask << ", 0x" << (uint16_t)final_best[i].seed << "}, // ";
                f << std::dec << "duty=" << final_best[i].actual_duty << " err=" << final_best[i].error << " LC=" << final_best[i].lc << "\n";
            }
        }
        f << "};\n";
    } else {
        f << "const struct { uint32_t mask; uint32_t seed; } lfsr_pwm_lut[] = {\n";
        for (int i=0;i<cfg.resolution;++i) {
            if (final_best[i].score == std::numeric_limits<double>::infinity()) {
                f << "{0,0},\n";
            } else {
                f << "{0x" << std::hex << (uint32_t)final_best[i].mask << ", 0x" << (uint32_t)final_best[i].seed << "}, // ";
                f << std::dec << "duty=" << final_best[i].actual_duty << " err=" << final_best[i].error << " LC=" << final_best[i].lc << "\n";
            }
        }
        f << "};\n";
    }
    f.close();

    std::cerr << "Wrote lfsr_pwm_lut.h\n";
    std::cerr << "Candidate counts per PWM level (considered during search):\n";
    for (int i=0;i<cfg.resolution;++i) std::cerr << i << " -> " << candidate_counts[i] << "\n";

    // report bins where final selection deviates beyond half-bin
    double tol = 0.5 / (double)(cfg.resolution - 1);
    for (int i=0;i<cfg.resolution;++i) {
        if (final_best[i].score == std::numeric_limits<double>::infinity()) {
            std::cerr << "BIN " << i << ": no verified candidate\n";
            continue;
        }
        double diff = fabs(final_best[i].actual_duty - targets[i]);
        if (diff > tol) {
            std::cerr << "WARNING: bin " << i << " selected mask=0x" << std::hex << (uint32_t)final_best[i].mask
                      << " seed=0x" << (uint32_t)final_best[i].seed << std::dec
                      << " measured_duty=" << final_best[i].actual_duty << " target=" << targets[i]
                      << " diff=" << diff << " block_err=" << final_best[i].error << "\n";
        }
    }

    return 0;
}
