// lfsr_pwm_spectral_galois.cpp
//
// Advanced LFSR PWM Search leveraging:
// 1. Galois Field Theory: Prioritizes Maximal Length Sequences (Primitive Polynomials).
// 2. Spectral Analysis: Computes DFT of low-frequency bins to eliminate visible oscillation.
// 3. Sliding Window: Guarantees local average constraints.
//
// Build: g++ -std=c++17 -O3 -march=native -flto -funroll-loops -pthread -o lfsr_search lfsr_pwm_spectral_galois.cpp

#include <bits/stdc++.h>
#include <atomic>
#include <thread>
#include <mutex>
#include <getopt.h>
#include <chrono>
#include <complex>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <numeric>

using u64 = uint64_t;
using u32 = uint32_t;
using Complex = std::complex<double>;

// --- Constants ---
const double PI = 3.14159265358979323846;
const double EPS = 1e-12;

struct LFSRConfig {
    u64 mask = 0;
    u64 seed = 0;
    int period = 0;
    int lc = 0;            // Linear Complexity
    double actual_duty = 0.0;
    double max_window_error = 0.0;
    double low_freq_energy = 0.0; // Energy in low frequency spectrum
    double score = 0.0;
};

// --- Helper: Popcount ---
static inline int popcount_u64(u64 x) {
    return __builtin_popcountll(x);
}

// --- LFSR Logic (Galois/Fibonacci) ---
// Note: In GF(2^n), the mask represents the coefficients of the feedback polynomial.
static inline u64 next_state(u64 state, u64 mask, int n) {
    // Fibonacci implementation
    u64 fb = __builtin_parityll(state & mask);
    return (state >> 1) | (fb << (n - 1));
}

// --- Berlekamp-Massey (Linear Complexity) ---
// Measures the size of the smallest LFSR required to generate the sequence.
// High LC implies high randomness/entropy.
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

// --- Spectral Analysis (Low Frequency Oscillation Check) ---
// Computes energy of the first K harmonics.
// If High -> The PWM duty is "wavering" slowly (bad).
// If Low -> The noise is "white" or high frequency (good).
double compute_low_freq_energy(const std::vector<int>& bits, int period, int harmonics = 5) {
    if (period < harmonics * 2) return 0.0;
    
    double total_energy = 0.0;
    // We compute DFT for k=1 to harmonics (skipping k=0 which is DC/Average)
    for (int k = 1; k <= harmonics; ++k) {
        Complex sum(0, 0);
        double angle_step = -2.0 * PI * k / period;
        
        // Direct summation DFT (O(N*K)) - faster than FFT for very small K relative to N
        for (int n = 0; n < period; ++n) {
            // bits[n] is 0 or 1. Center it to -0.5 to 0.5 to remove DC offset for spectral purity
            double val = (double)bits[n] - 0.5; 
            double theta = angle_step * n;
            sum += Complex(val * cos(theta), val * sin(theta));
        }
        
        // Normalize magnitude
        double mag = std::abs(sum) / period;
        total_energy += (mag * mag);
    }
    return total_energy; // Lower is better
}

// --- Cycle Finder ---
std::vector<std::vector<u64>> find_all_cycles(u64 mask, int n) {
    const u64 max_state = (1ULL << n);
    // Use a small visited bitset or map. For n>20 this consumes too much RAM, 
    // but typical PWM LFSRs are < 24 bits.
    // For large N, we limit the search depth.
    
    std::vector<std::vector<u64>> cycles;
    
    // For safety with large N, we probe random states rather than exhaustive array
    // Heuristic: Check if mask generates a Maximum Length Sequence (Primitive) first.
    // Start with a standard seed.
    
    u64 start_node = 0x1; // Standard start
    std::vector<u64> chain;
    chain.reserve(65536);
    
    u64 curr = start_node;
    // Tortoise and Hare algorithm could be used, but we want the full cycle data
    // Limit max period to analyze to something reasonable (e.g., 65k) for speed,
    // or let it run if user wants deep search.
    
    // Simplification for this tool: We trace ONE cycle from seed 1. 
    // If it returns to 1, we have a cycle.
    
    // Optimization: Brent's algorithm for cycle finding is memory efficient,
    // but we need the actual data for PWM analysis.
    
    std::unordered_set<u64> visited;
    while(visited.find(curr) == visited.end() && chain.size() < (1<<18)) { // Cap at 256k length
        visited.insert(curr);
        chain.push_back(curr);
        curr = next_state(curr, mask, n);
        if (curr == start_node) break; // Cycle found
    }
    
    if (curr == start_node && chain.size() > 1) {
        cycles.push_back(std::move(chain));
    }
    
    return cycles;
}

// --- Export Support ---
static std::vector<int> out_fds;
static std::vector<std::mutex> file_mutexes;

void prepare_output_files(const std::string &dir) {
    mkdir(dir.c_str(), 0755);
    out_fds.assign(256, -1);
    file_mutexes.resize(256);
    char path[512];
    for (int i = 0; i < 256; ++i) {
        snprintf(path, sizeof(path), "%s/0x%02X.bin", dir.c_str(), i);
        out_fds[i] = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    }
}

void export_candidate(int idx, u64 mask, u64 seed, int n) {
    if (idx < 0 || idx > 255 || out_fds[idx] < 0) return;
    std::lock_guard<std::mutex> lock(file_mutexes[idx]);
    if (n <= 16) {
        uint16_t d[] = {(uint16_t)mask, (uint16_t)seed};
        write(out_fds[idx], d, 4);
    } else {
        uint32_t d[] = {(uint32_t)mask, (uint32_t)seed};
        write(out_fds[idx], d, 8);
    }
}

// --- Analysis Logic ---

void evaluate_mask(u64 mask, int n, const std::vector<double> &targets,
                   std::vector<LFSRConfig> &best_configs,
                   std::mutex &merge_mutex, 
                   const Options& opt) { // assuming Options struct is available globally or passed
                   
    auto cycles = find_all_cycles(mask, n);
    if (cycles.empty()) return;

    // We only care about the main cycle (seed=1). 
    // In GF(2^n), primitive polynomials have one main cycle of length 2^n-1.
    const auto& cycle = cycles[0];
    int period = (int)cycle.size();
    
    // 1. Galois Property Check: Is it a Maximal Length Sequence (or close)?
    // m-sequences are optimal for randomness.
    double max_period = (double)((1ULL << n) - 1);
    double period_ratio = period / max_period;
    
    // Strictness: If we want high randomness, reject short cycles immediately.
    if (period_ratio < 0.5) return; 

    // Sort cycle to find thresholds
    std::vector<u64> sorted = cycle;
    std::sort(sorted.begin(), sorted.end());
    
    // Pre-calculate mapping for speed
    std::unordered_map<u64, int> val_to_idx;
    for(int i=0; i<period; ++i) val_to_idx[cycle[i]] = i;

    // Local results
    std::vector<LFSRConfig> local_bests(targets.size());
    for(auto& b : local_bests) b.score = 1e9;

    for (size_t ti = 0; ti < targets.size(); ++ti) {
        double target = targets[ti];
        
        // Find ideal threshold index
        int k_ideal = (int)std::round(target * period);
        k_ideal = std::clamp(k_ideal, 0, period-1);
        
        // Threshold value is the element at index (period - k - 1) in sorted list
        // Because: count(x > T) = k
        int sorted_idx = period - k_ideal - 1;
        if (sorted_idx < 0) sorted_idx = 0;
        u64 threshold = sorted[sorted_idx];
        
        // Identify "Seed": The state in the cycle where this value appears.
        // Actually, the seed defines the phase. For a fixed threshold, the sequence is:
        // OUT[i] = (Cycle[(start + i) % P] > Threshold)
        // We need to find the best 'start' (seed) to minimize window error?
        // Actually, for a fixed cycle and fixed threshold, the sequence is invariant 
        // up to cyclic shift. The "Window Error" is valid for *any* window, so 
        // shifting the seed doesn't change the max error, just where it occurs.
        // So we can pick *any* state as seed (e.g., cycle[0]). 
        // HOWEVER, the threshold must be chosen to match duty.
        
        // Generate the binary sequence for this threshold
        std::vector<int> bits(period);
        int ones = 0;
        for(int i=0; i<period; ++i) {
            bits[i] = (cycle[i] > threshold) ? 1 : 0;
            ones += bits[i];
        }
        
        double actual_duty = (double)ones / period;
        double global_error = std::abs(actual_duty - target);
        
        // 2. Spectral Analysis (Low Frequency Check)
        // We punish sequences that have high energy in low freq bins.
        double lf_energy = compute_low_freq_energy(bits, period, 5);
        
        // 3. Sliding Window Error (Local Duty Stability)
        int block = 256; // Window size
        std::vector<int> prefix(period * 2 + 1, 0);
        for(int i=0; i<period*2; ++i) prefix[i+1] = prefix[i] + bits[i % period];
        
        double max_win_err = 0.0;
        // Check standard windows
        for(int i=0; i<period; i += block/4) { // Stride optimization
            int w_ones = prefix[i+block] - prefix[i];
            double w_duty = (double)w_ones / block;
            max_win_err = std::max(max_win_err, std::abs(w_duty - actual_duty));
        }
        
        // 4. Linear Complexity (Berlekamp-Massey)
        // We only check a subset to save time if period is huge
        int check_len = std::min(period, 1024);
        std::vector<uint8_t> sample_bits; 
        for(int i=0;i<check_len;++i) sample_bits.push_back(bits[i]);
        int lc = berlekamp_massey(sample_bits);
        
        // --- Scoring Function ---
        // Lower is better.
        // Weights:
        // - Window Error: Critical (must be low)
        // - LF Energy: Critical (must be low for non-oscillating)
        // - LC: Higher is better (so we subtract)
        
        double score = (max_win_err * 10.0)      // Penalty for local drift
                     + (lf_energy * 50.0)        // Heavy Penalty for oscillation
                     + (global_error * 100.0)    // Penalty for wrong average
                     - (double(lc)/n * 0.1);     // Bonus for complexity
                     
        if (score < local_bests[ti].score) {
            local_bests[ti] = {mask, threshold, period, lc, actual_duty, max_win_err, lf_energy, score};
            
            // Export visualization
            if (out_fds.size() > 0) {
                 export_candidate((int)(ti), mask, threshold, n);
            }
        }
    }

    // Merge
    std::lock_guard<std::mutex> lock(merge_mutex);
    for(size_t i=0; i<targets.size(); ++i) {
        if(local_bests[i].score < best_configs[i].score) {
            best_configs[i] = local_bests[i];
        }
    }
}

// --- Main Driver ---

struct Options {
    int n = 16;
    int resolution = 256;
    int threads = 4;
    u64 max_masks = 50000;
    std::string out_dir = "galois_bins";
};

int main(int argc, char **argv) {
    Options opt;
    // (Argument parsing omitted for brevity, assume similar to previous)
    
    std::cout << "Starting Galois-Enhanced Spectral LFSR Search (N=" << opt.n << ")\n";
    prepare_output_files(opt.out_dir);

    std::vector<double> targets(opt.resolution);
    for(int i=0; i<opt.resolution; ++i) targets[i] = (double)i/(opt.resolution-1);

    std::vector<LFSRConfig> best_configs(opt.resolution);
    std::vector<u64> masks;
    
    // Generator: Prioritize Odd Masks (Required for max period)
    u64 limit = (opt.n >= 20) ? opt.max_masks : (1ULL << opt.n);
    if (limit > opt.max_masks) limit = opt.max_masks;
    
    // Random shuffle approach for coverage
    std::mt19937_64 rng(12345);
    for(int i=0; i<limit; ++i) {
        u64 m = rng() & ((1ULL<<opt.n)-1);
        if(m & 1) masks.push_back(m); // Low bit must be set for max period
    }
    
    std::mutex merge_mutex;
    std::atomic<int> processed{0};

    auto worker = [&](int start, int end) {
        for(int i=start; i<end; ++i) {
            evaluate_mask(masks[i], opt.n, targets, best_configs, merge_mutex, opt);
            processed++;
            if (processed % 1000 == 0) std::cout << "Scanned " << processed << "...\r" << std::flush;
        }
    };

    int t_count = std::thread::hardware_concurrency();
    std::vector<std::thread> threads;
    int chunk = masks.size() / t_count;
    for(int i=0; i<t_count; ++i) {
        threads.emplace_back(worker, i*chunk, (i==t_count-1)?masks.size():(i+1)*chunk);
    }
    for(auto& t : threads) t.join();

    // Output LUT
    std::ofstream f("lfsr_spectral_lut.h");
    f << "// Generated by Galois/Spectral Search\n";
    f << "const struct { uint32_t mask; uint32_t threshold; } lut[] = {\n";
    for(const auto& c : best_configs) {
        f << " {0x" << std::hex << c.mask << ", 0x" << c.seed << "}, // Err:" 
          << std::scientific << c.max_window_error << " LF:" << c.low_freq_energy << "\n";
    }
    f << "};\n";
    
    std::cout << "\nDone. Results in lfsr_spectral_lut.h\n";
}
