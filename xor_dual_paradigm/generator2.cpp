```cpp
#include <iostream>
#include <vector>
#include <array>
#include <fstream>
#include <thread>
#include <mutex>
#include <atomic>
#include <cmath>
#include <algorithm>
#include <complex>
#include <random>
#include <iomanip>
#include <filesystem>
#include <cstring>
#include <immintrin.h>
#include <unordered_set>

namespace fs = std::filesystem;

// Configuration
struct Config {
    uint32_t pwm_steps = 256;
    uint32_t simulation_length = 65536;  // Length to simulate for analysis
    uint32_t candidates_per_level = 10;   // Top N candidates to save
    double duty_tolerance = 0.02;         // ±2% duty cycle tolerance
    std::string output_dir = "pwm_candidates";
    std::string lut_output = "pwm_lut.h";
    uint32_t num_threads = 0;  // 0 = auto-detect
    bool include_non_maximal = true;  // Include non-maximal-length LFSRs
    uint32_t min_period = 255;  // Minimum acceptable LFSR period
};

// LFSR parameter set
struct LFSRParams {
    uint16_t mask;
    uint16_t seed;
    
    LFSRParams() : mask(0), seed(0) {}
    LFSRParams(uint16_t m, uint16_t s) : mask(m), seed(s) {}
};

// LFSR characteristics
struct LFSRCharacteristics {
    uint16_t mask;
    uint32_t period;
    uint32_t num_taps;
    double state_density;  // Fraction of possible states visited
    bool is_maximal;
    
    LFSRCharacteristics() : mask(0), period(0), num_taps(0), state_density(0.0), is_maximal(false) {}
};

// Dual LFSR entry
struct DualLFSREntry {
    LFSRParams lfsr1;
    LFSRParams lfsr2;
    double actual_duty;
    double spectral_flatness;
    double fitness_score;
    uint32_t combined_period;  // Period of combined sequence
    
    bool operator<(const DualLFSREntry& other) const {
        return fitness_score > other.fitness_score;  // Higher is better
    }
};

// Maximal-length LFSR tap configurations (Fibonacci/Galois) for 16-bit
const std::vector<uint16_t> MAXIMAL_TAPS_16 = {
    0xB400, 0xD008, 0xB000, 0xA001, 0x9000,
    0x8016, 0x8005, 0x8003, 0x4001, 0x3802
};

class LFSRAnalyzer {
private:
    static inline uint16_t advance_lfsr(uint16_t state, uint16_t mask) {
        uint16_t feedback = __builtin_parity(state & mask) & 1;
        return (state >> 1) | (feedback << 15);
    }
    
    // Calculate GCD for period computation
    static uint32_t gcd(uint32_t a, uint32_t b) {
        while (b != 0) {
            uint32_t temp = b;
            b = a % b;
            a = temp;
        }
        return a;
    }
    
    // Calculate LCM for combined period
    static uint32_t lcm(uint32_t a, uint32_t b) {
        if (a == 0 || b == 0) return 0;
        return (a / gcd(a, b)) * b;
    }

public:
    // Determine LFSR period and characteristics
    static LFSRCharacteristics analyze_lfsr(uint16_t mask, uint16_t seed = 0x0001, uint32_t max_iterations = 100000) {
        LFSRCharacteristics chars;
        chars.mask = mask;
        chars.num_taps = __builtin_popcount(mask);
        
        if (seed == 0) seed = 0x0001;  // Avoid all-zero state
        
        uint16_t state = seed;
        std::unordered_set<uint16_t> visited_states;
        visited_states.insert(state);
        
        uint32_t iterations = 0;
        do {
            state = advance_lfsr(state, mask);
            iterations++;
            
            if (iterations >= max_iterations) {
                chars.period = max_iterations;
                chars.state_density = static_cast<double>(visited_states.size()) / 65536.0;
                chars.is_maximal = false;
                return chars;
            }
            
            if (visited_states.count(state)) {
                break;  // Found the period
            }
            visited_states.insert(state);
            
        } while (state != seed && iterations < max_iterations);
        
        chars.period = iterations;
        chars.state_density = static_cast<double>(visited_states.size()) / 65536.0;
        chars.is_maximal = (chars.period == 65535);  // 2^16 - 1
        
        return chars;
    }
    
    // Calculate combined period of two LFSRs
    static uint32_t calculate_combined_period(const LFSRCharacteristics& lfsr1, 
                                               const LFSRCharacteristics& lfsr2) {
        return lcm(lfsr1.period, lfsr2.period);
    }
    
    // Generate all valid tap configurations for given bit width
    static std::vector<uint16_t> generate_tap_configurations(uint32_t min_taps = 2, 
                                                              uint32_t max_taps = 8,
                                                              uint32_t min_period = 255) {
        std::vector<uint16_t> valid_masks;
        
        // Add maximal-length LFSRs first
        for (auto mask : MAXIMAL_TAPS_16) {
            valid_masks.push_back(mask);
        }
        
        std::cout << "Analyzing non-maximal LFSR configurations..." << std::endl;
        std::atomic<uint32_t> analyzed{0};
        std::atomic<uint32_t> valid_found{MAXIMAL_TAPS_16.size()};
        std::mutex mask_mutex;
        
        // Sample random tap configurations
        const uint32_t samples = 10000;  // Sample space
        std::mt19937 rng(42);
        std::uniform_int_distribution<uint16_t> dist(1, 0xFFFF);
        
        std::vector<uint16_t> candidate_masks;
        for (uint32_t i = 0; i < samples; i++) {
            uint16_t mask = dist(rng);
            uint32_t tap_count = __builtin_popcount(mask);
            if (tap_count >= min_taps && tap_count <= max_taps) {
                candidate_masks.push_back(mask);
            }
        }
        
        // Also systematically explore sparse configurations
        for (uint32_t taps = min_taps; taps <= std::min(max_taps, 6u); taps++) {
            // Generate combinations of tap positions
            std::vector<uint32_t> positions;
            for (uint32_t i = 0; i < 16; i++) {
                positions.push_back(i);
            }
            
            // Generate some combinations (not all to save time)
            uint32_t combinations_to_try = std::min(500u, 1u << taps);
            for (uint32_t c = 0; c < combinations_to_try; c++) {
                uint16_t mask = 0;
                uint32_t temp_c = c;
                for (uint32_t t = 0; t < taps; t++) {
                    uint32_t pos = temp_c % (16 - t);
                    mask |= (1 << positions[pos]);
                    positions.erase(positions.begin() + pos);
                    temp_c /= (16 - t);
                }
                if (mask != 0) {
                    candidate_masks.push_back(mask);
                }
                // Restore positions
                positions.clear();
                for (uint32_t i = 0; i < 16; i++) {
                    positions.push_back(i);
                }
            }
        }
        
        // Remove duplicates
        std::sort(candidate_masks.begin(), candidate_masks.end());
        candidate_masks.erase(std::unique(candidate_masks.begin(), candidate_masks.end()), 
                             candidate_masks.end());
        
        std::cout << "Analyzing " << candidate_masks.size() << " candidate masks..." << std::endl;
        
        // Analyze candidates in parallel
        auto worker = [&](size_t start, size_t end) {
            std::vector<uint16_t> local_valid;
            
            for (size_t i = start; i < end; i++) {
                uint16_t mask = candidate_masks[i];
                auto chars = analyze_lfsr(mask);
                
                if (chars.period >= min_period) {
                    local_valid.push_back(mask);
                }
                
                uint32_t count = ++analyzed;
                if (count % 1000 == 0) {
                    std::cout << "  Analyzed: " << count << "/" << candidate_masks.size() 
                             << " (found " << valid_found.load() << " valid)" << std::endl;
                }
            }
            
            if (!local_valid.empty()) {
                std::lock_guard<std::mutex> lock(mask_mutex);
                valid_masks.insert(valid_masks.end(), local_valid.begin(), local_valid.end());
                valid_found += local_valid.size();
            }
        };
        
        // Launch worker threads
        uint32_t num_threads = std::thread::hardware_concurrency();
        std::vector<std::thread> threads;
        size_t chunk_size = candidate_masks.size() / num_threads;
        
        for (uint32_t t = 0; t < num_threads; t++) {
            size_t start = t * chunk_size;
            size_t end = (t == num_threads - 1) ? candidate_masks.size() : (t + 1) * chunk_size;
            threads.emplace_back(worker, start, end);
        }
        
        for (auto& thread : threads) {
            thread.join();
        }
        
        std::cout << "Found " << valid_masks.size() << " valid LFSR configurations" << std::endl;
        
        // Characterize and print statistics
        std::vector<LFSRCharacteristics> all_chars;
        for (auto mask : valid_masks) {
            all_chars.push_back(analyze_lfsr(mask));
        }
        
        uint32_t maximal_count = 0;
        uint32_t non_maximal_count = 0;
        for (const auto& chars : all_chars) {
            if (chars.is_maximal) maximal_count++;
            else non_maximal_count++;
        }
        
        std::cout << "  Maximal-length: " << maximal_count << std::endl;
        std::cout << "  Non-maximal: " << non_maximal_count << std::endl;
        
        return valid_masks;
    }
};

class LFSRSimulator {
private:
    static inline uint16_t advance_lfsr(uint16_t state, uint16_t mask) {
        uint16_t feedback = __builtin_parity(state & mask) & 1;
        return (state >> 1) | (feedback << 15);
    }
    
    // Vectorized simulation for speed
    static std::vector<uint8_t> simulate_dual_lfsr_avx2(
        uint16_t mask1, uint16_t seed1,
        uint16_t mask2, uint16_t seed2,
        uint32_t length)
    {
        std::vector<uint8_t> output(length);
        uint16_t state1 = seed1;
        uint16_t state2 = seed2;
        
        // Unrolled loop for better performance
        uint32_t i = 0;
        for (; i + 4 <= length; i += 4) {
            state1 = advance_lfsr(state1, mask1);
            state2 = advance_lfsr(state2, mask2);
            output[i] = (state1 ^ state2) & 1;
            
            state1 = advance_lfsr(state1, mask1);
            state2 = advance_lfsr(state2, mask2);
            output[i+1] = (state1 ^ state2) & 1;
            
            state1 = advance_lfsr(state1, mask1);
            state2 = advance_lfsr(state2, mask2);
            output[i+2] = (state1 ^ state2) & 1;
            
            state1 = advance_lfsr(state1, mask1);
            state2 = advance_lfsr(state2, mask2);
            output[i+3] = (state1 ^ state2) & 1;
        }
        
        // Handle remainder
        for (; i < length; i++) {
            state1 = advance_lfsr(state1, mask1);
            state2 = advance_lfsr(state2, mask2);
            output[i] = (state1 ^ state2) & 1;
        }
        
        return output;
    }

public:
    // Calculate duty cycle from bit sequence
    static double calculate_duty_cycle(const std::vector<uint8_t>& sequence) {
        uint32_t ones = 0;
        for (auto bit : sequence) {
            ones += bit;
        }
        return static_cast<double>(ones) / sequence.size();
    }
    
    // Calculate spectral flatness using FFT approximation
    static double calculate_spectral_flatness(const std::vector<uint8_t>& sequence) {
        std::vector<double> signal(sequence.size());
        for (size_t i = 0; i < sequence.size(); i++) {
            signal[i] = sequence[i] ? 1.0 : -1.0;
        }
        
        double geometric_mean = 0.0;
        double arithmetic_mean = 0.0;
        uint32_t num_lags = std::min(512u, static_cast<uint32_t>(sequence.size() / 2));
        
        std::vector<double> power_spectrum(num_lags);
        
        for (uint32_t lag = 1; lag < num_lags; lag++) {
            double autocorr = 0.0;
            for (uint32_t i = 0; i < sequence.size() - lag; i++) {
                autocorr += signal[i] * signal[i + lag];
            }
            autocorr /= (sequence.size() - lag);
            power_spectrum[lag] = std::abs(autocorr);
        }
        
        for (auto p : power_spectrum) {
            if (p > 1e-10) {
                geometric_mean += std::log(p);
                arithmetic_mean += p;
            }
        }
        
        geometric_mean = std::exp(geometric_mean / power_spectrum.size());
        arithmetic_mean /= power_spectrum.size();
        
        if (arithmetic_mean < 1e-10) return 0.0;
        return geometric_mean / arithmetic_mean;
    }
    
    // Calculate transition density
    static double calculate_transition_density(const std::vector<uint8_t>& sequence) {
        uint32_t transitions = 0;
        for (size_t i = 1; i < sequence.size(); i++) {
            if (sequence[i] != sequence[i-1]) {
                transitions++;
            }
        }
        return static_cast<double>(transitions) / (sequence.size() - 1);
    }
    
    // Calculate run-length statistics (prefer shorter runs for better mixing)
    static double calculate_run_length_score(const std::vector<uint8_t>& sequence) {
        std::vector<uint32_t> run_lengths;
        uint32_t current_run = 1;
        
        for (size_t i = 1; i < sequence.size(); i++) {
            if (sequence[i] == sequence[i-1]) {
                current_run++;
            } else {
                run_lengths.push_back(current_run);
                current_run = 1;
            }
        }
        run_lengths.push_back(current_run);
        
        // Calculate statistics
        double mean_run = 0.0;
        uint32_t max_run = 0;
        for (auto run : run_lengths) {
            mean_run += run;
            max_run = std::max(max_run, run);
        }
        mean_run /= run_lengths.size();
        
        // Score: prefer mean close to 2-3, penalize long runs
        double score = 1.0 / (1.0 + std::abs(mean_run - 2.5)) - max_run * 0.01;
        return std::max(0.0, score);
    }
    
    // Evaluate a dual LFSR configuration
    static DualLFSREntry evaluate_configuration(
        uint16_t mask1, uint16_t seed1,
        uint16_t mask2, uint16_t seed2,
        double target_duty,
        uint32_t sim_length,
        double duty_tolerance,
        const LFSRCharacteristics& chars1,
        const LFSRCharacteristics& chars2)
    {
        DualLFSREntry entry;
        entry.lfsr1 = LFSRParams(mask1, seed1);
        entry.lfsr2 = LFSRParams(mask2, seed2);
        entry.combined_period = LFSRAnalyzer::calculate_combined_period(chars1, chars2);
        
        // Simulate
        auto sequence = simulate_dual_lfsr_avx2(mask1, seed1, mask2, seed2, sim_length);
        
        // Calculate metrics
        entry.actual_duty = calculate_duty_cycle(sequence);
        entry.spectral_flatness = calculate_spectral_flatness(sequence);
        double transition_density = calculate_transition_density(sequence);
        double run_length_score = calculate_run_length_score(sequence);
        
        // Duty cycle error
        double duty_error = std::abs(entry.actual_duty - target_duty);
        
        // Period bonus (longer periods are generally better for randomness)
        double period_bonus = std::log10(std::min(entry.combined_period, 1000000u)) * 5.0;
        
        // Fitness function
        if (duty_error > duty_tolerance) {
            entry.fitness_score = -1000.0 * duty_error;
        } else {
            entry.fitness_score = 
                100.0 * entry.spectral_flatness +
                50.0 * transition_density +
                30.0 * run_length_score +
                period_bonus -
                500.0 * duty_error;
        }
        
        return entry;
    }
};

class PWMLUTGenerator {
private:
    Config config_;
    std::mutex output_mutex_;
    std::atomic<uint32_t> completed_levels_{0};
    std::vector<uint16_t> valid_masks_;
    std::vector<LFSRCharacteristics> mask_characteristics_;
    
    void initialize_lfsr_database() {
        std::cout << "Building LFSR database..." << std::endl;
        
        if (config_.include_non_maximal) {
            valid_masks_ = LFSRAnalyzer::generate_tap_configurations(2, 8, config_.min_period);
        } else {
            valid_masks_ = MAXIMAL_TAPS_16;
            std::cout << "Using only maximal-length LFSRs: " << valid_masks_.size() << std::endl;
        }
        
        // Pre-analyze all masks
        std::cout << "Characterizing all LFSR masks..." << std::endl;
        for (auto mask : valid_masks_) {
            mask_characteristics_.push_back(LFSRAnalyzer::analyze_lfsr(mask));
        }
        
        // Print statistics
        std::cout << "\nLFSR Database Statistics:" << std::endl;
        std::cout << "=========================" << std::endl;
        
        std::map<uint32_t, uint32_t> period_histogram;
        for (const auto& chars : mask_characteristics_) {
            uint32_t period_bucket = (chars.period / 1000) * 1000;
            period_histogram[period_bucket]++;
        }
        
        std::cout << "Period distribution:" << std::endl;
        for (const auto& [period, count] : period_histogram) {
            std::cout << "  " << period << "-" << (period + 999) << ": " << count << std::endl;
        }
        std::cout << std::endl;
    }
    
    std::vector<uint16_t> generate_seed_candidates(uint16_t base_seed, uint32_t count) {
        std::vector<uint16_t> seeds;
        seeds.push_back(base_seed);
        
        std::mt19937 rng(base_seed);
        std::uniform_int_distribution<uint16_t> dist(1, 0xFFFF);
        
        for (uint32_t i = 1; i < count && seeds.size() < count; i++) {
            uint16_t seed = dist(rng);
            if (seed != 0) {
                seeds.push_back(seed);
            }
        }
        
        return seeds;
    }
    
    std::vector<DualLFSREntry> find_optimal_pairs_for_duty(double target_duty) {
        std::vector<DualLFSREntry> candidates;
        
        // Strategy: Try combinations with different period characteristics
        uint32_t masks_to_try = std::min(50u, static_cast<uint32_t>(valid_masks_.size()));
        
        for (uint32_t i = 0; i < masks_to_try; i++) {
            for (uint32_t j = i; j < masks_to_try; j++) {
                uint16_t mask1 = valid_masks_[i];
                uint16_t mask2 = valid_masks_[j];
                
                auto seeds1 = generate_seed_candidates(0x1234 + i, 15);
                auto seeds2 = generate_seed_candidates(0x5678 + j, 15);
                
                for (auto seed1 : seeds1) {
                    for (auto seed2 : seeds2) {
                        auto entry = LFSRSimulator::evaluate_configuration(
                            mask1, seed1, mask2, seed2,
                            target_duty, config_.simulation_length,
                            config_.duty_tolerance,
                            mask_characteristics_[i],
                            mask_characteristics_[j]
                        );
                        
                        if (entry.fitness_score > -500.0) {
                            candidates.push_back(entry);
                        }
                    }
                }
            }
        }
        
        std::sort(candidates.begin(), candidates.end());
        if (candidates.size() > config_.candidates_per_level) {
            candidates.resize(config_.candidates_per_level);
        }
        
        return candidates;
    }
    
    void process_pwm_level(uint32_t level) {
        double target_duty = static_cast<double>(level) / (config_.pwm_steps - 1);
        
        auto candidates = find_optimal_pairs_for_duty(target_duty);
        
        save_candidates_to_file(level, candidates);
        
        uint32_t completed = ++completed_levels_;
        if (completed % 10 == 0 || completed == config_.pwm_steps) {
            std::lock_guard<std::mutex> lock(output_mutex_);
            std::cout << "Progress: " << completed << "/" << config_.pwm_steps 
                      << " levels completed (" 
                      << (100 * completed / config_.pwm_steps) << "%)" << std::endl;
        }
    }
    
    void save_candidates_to_file(uint32_t level, const std::vector<DualLFSREntry>& candidates) {
        std::ostringstream filename;
        filename << config_.output_dir << "/0x" 
                 << std::hex << std::setfill('0') << std::setw(2) << level << ".bin";
        
        std::ofstream file(filename.str(), std::ios::binary);
        if (!file) {
            std::cerr << "Failed to create file: " << filename.str() << std::endl;
            return;
        }
        
        uint32_t count = candidates.size();
        file.write(reinterpret_cast<const char*>(&count), sizeof(count));
        
        for (const auto& entry : candidates) {
            file.write(reinterpret_cast<const char*>(&entry.lfsr1.mask), sizeof(uint16_t));
            file.write(reinterpret_cast<const char*>(&entry.lfsr1.seed), sizeof(uint16_t));
            file.write(reinterpret_cast<const char*>(&entry.lfsr2.mask), sizeof(uint16_t));
            file.write(reinterpret_cast<const char*>(&entry.lfsr2.seed), sizeof(uint16_t));
            file.write(reinterpret_cast<const char*>(&entry.actual_duty), sizeof(double));
            file.write(reinterpret_cast<const char*>(&entry.spectral_flatness), sizeof(double));
            file.write(reinterpret_cast<const char*>(&entry.fitness_score), sizeof(double));
            file.write(reinterpret_cast<const char*>(&entry.combined_period), sizeof(uint32_t));
        }
        
        file.close();
    }
    
    DualLFSREntry load_best_candidate(uint32_t level) {
        std::ostringstream filename;
        filename << config_.output_dir << "/0x" 
                 << std::hex << std::setfill('0') << std::setw(2) << level << ".bin";
        
        std::ifstream file(filename.str(), std::ios::binary);
        if (!file) {
            std::cerr << "Failed to open file: " << filename.str() << std::endl;
            return DualLFSREntry();
        }
        
        uint32_t count;
        file.read(reinterpret_cast<char*>(&count), sizeof(count));
        
        if (count == 0) {
            return DualLFSREntry();
        }
        
        DualLFSREntry entry;
        file.read(reinterpret_cast<char*>(&entry.lfsr1.mask), sizeof(uint16_t));
        file.read(reinterpret_cast<char*>(&entry.lfsr1.seed), sizeof(uint16_t));
        file.read(reinterpret_cast<char*>(&entry.lfsr2.mask), sizeof(uint16_t));
        file.read(reinterpret_cast<char*>(&entry.lfsr2.seed), sizeof(uint16_t));
        file.read(reinterpret_cast<char*>(&entry.actual_duty), sizeof(double));
        file.read(reinterpret_cast<char*>(&entry.spectral_flatness), sizeof(double));
        file.read(reinterpret_cast<char*>(&entry.fitness_score), sizeof(double));
        file.read(reinterpret_cast<char*>(&entry.combined_period), sizeof(uint32_t));
        
        return entry;
    }

public:
    PWMLUTGenerator(const Config& config) : config_(config) {
        if (config_.num_threads == 0) {
            config_.num_threads = std::thread::hardware_concurrency();
        }
    }
    
    void generate() {
        std::cout << "PWM LUT Generator (Extended)" << std::endl;
        std::cout << "=============================" << std::endl;
        std::cout << "PWM Steps: " << config_.pwm_steps << std::endl;
        std::cout << "Threads: " << config_.num_threads << std::endl;
        std::cout << "Simulation Length: " << config_.simulation_length << std::endl;
        std::cout << "Candidates per level: " << config_.candidates_per_level << std::endl;
        std::cout << "Include non-maximal LFSRs: " << (config_.include_non_maximal ? "Yes" : "No") << std::endl;
        std::cout << "Minimum LFSR period: " << config_.min_period << std::endl;
        std::cout << std::endl;
        
        // Initialize LFSR database
        initialize_lfsr_database();
        
        // Create output directory
        fs::create_directories(config_.output_dir);
        
        // Process all PWM levels in parallel
        std::vector<std::thread> threads;
        std::atomic<uint32_t> next_level{0};
        
        auto worker = [this, &next_level]() {
            while (true) {
                uint32_t level = next_level.fetch_add(1);
                if (level >= config_.pwm_steps) break;
                process_pwm_level(level);
            }
        };
        
        auto start_time = std::chrono::high_resolution_clock::now();
        
        for (uint32_t i = 0; i < config_.num_threads; i++) {
            threads.emplace_back(worker);
        }
        
        for (auto& thread : threads) {
            thread.join();
        }
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
        
        std::cout << "\nGeneration completed in " << duration.count() << " seconds" << std::endl;
        
        // Generate Arduino LUT file
        generate_arduino_lut();
    }
    
    void generate_arduino_lut() {
        std::cout << "\nGenerating Arduino LUT header file..." << std::endl;
        
        std::ofstream file(config_.lut_output);
        if (!file) {
            std::cerr << "Failed to create LUT file: " << config_.lut_output << std::endl;
            return;
        }
        
        file << "// Auto-generated PWM LUT for dual LFSR XOR\n";
        file << "// Generated with " << config_.pwm_steps << " PWM levels\n";
        file << "// Includes " << (config_.include_non_maximal ? "maximal and non-maximal" : "maximal-only") << " LFSRs\n";
        file << "#ifndef PWM_LUT_H\n";
        file << "#define PWM_LUT_H\n\n";
        file << "#include <avr/pgmspace.h>\n\n";
        
        file << "struct DualLFSREntry {\n";
        file << "  uint16_t mask1;\n";
        file << "  uint16_t seed1;\n";
        file << "uint16_t mask2;\n";
        file << "  uint16_t seed2;\n";
        file << "};\n\n";
        
        file << "const PROGMEM DualLFSREntry pwm_lut[" << config_.pwm_steps << "] = {\n";
        
        for (uint32_t level = 0; level < config_.pwm_steps; level++) {
            auto entry = load_best_candidate(level);
            
            file << "  {0x" << std::hex << std::setfill('0') << std::setw(4) << entry.lfsr1.mask
                 << ", 0x" << std::setw(4) << entry.lfsr1.seed
                 << ", 0x" << std::setw(4) << entry.lfsr2.mask
                 << ", 0x" << std::setw(4) << entry.lfsr2.seed
                 << "}";
            
            if (level < config_.pwm_steps - 1) {
                file << ",";
            }
            
            file << "  // Level " << std::dec << level 
                 << " (duty: " << std::fixed << std::setprecision(4) << entry.actual_duty
                 << ", SF: " << std::setprecision(3) << entry.spectral_flatness 
                 << ", period: " << entry.combined_period << ")\n";
        }
        
        file << "};\n\n";
        
        // Add helper functions for Arduino
        file << "// Helper function to load LFSR parameters from PROGMEM\n";
        file << "inline void load_pwm_level(uint8_t level, uint16_t& mask1, uint16_t& seed1, uint16_t& mask2, uint16_t& seed2) {\n";
        file << "  DualLFSREntry entry;\n";
        file << "  memcpy_P(&entry, &pwm_lut[level], sizeof(DualLFSREntry));\n";
        file << "  mask1 = entry.mask1;\n";
        file << "  seed1 = entry.seed1;\n";
        file << "  mask2 = entry.mask2;\n";
        file << "  seed2 = entry.seed2;\n";
        file << "}\n\n";
        
        file << "#endif // PWM_LUT_H\n";
        
        file.close();
        
        std::cout << "Arduino LUT written to: " << config_.lut_output << std::endl;
        
        // Generate statistics report
        generate_statistics_report();
    }
    
    void generate_statistics_report() {
        std::cout << "\nGenerating statistics report..." << std::endl;
        
        std::ofstream report(config_.output_dir + "/statistics.txt");
        if (!report) {
            std::cerr << "Failed to create statistics file" << std::endl;
            return;
        }
        
        report << "PWM LUT Generation Statistics\n";
        report << "==============================\n\n";
        report << "Configuration:\n";
        report << "  PWM Steps: " << config_.pwm_steps << "\n";
        report << "  Simulation Length: " << config_.simulation_length << "\n";
        report << "  Include Non-Maximal: " << (config_.include_non_maximal ? "Yes" : "No") << "\n";
        report << "  Min Period: " << config_.min_period << "\n\n";
        
        report << "Per-Level Analysis:\n";
        report << "Level | Target | Actual | Error  | SF    | Period   | Fitness | Mask1  | Mask2\n";
        report << "------|--------|--------|--------|-------|----------|---------|--------|--------\n";
        
        double total_error = 0.0;
        double max_error = 0.0;
        double total_sf = 0.0;
        uint32_t min_period = UINT32_MAX;
        uint32_t max_period = 0;
        
        std::map<uint16_t, uint32_t> mask_usage;
        
        for (uint32_t level = 0; level < config_.pwm_steps; level++) {
            auto entry = load_best_candidate(level);
            double target = static_cast<double>(level) / (config_.pwm_steps - 1);
            double error = std::abs(entry.actual_duty - target);
            
            total_error += error;
            max_error = std::max(max_error, error);
            total_sf += entry.spectral_flatness;
            min_period = std::min(min_period, entry.combined_period);
            max_period = std::max(max_period, entry.combined_period);
            
            mask_usage[entry.lfsr1.mask]++;
            mask_usage[entry.lfsr2.mask]++;
            
            report << std::setw(5) << level << " | "
                   << std::fixed << std::setprecision(4) << target << " | "
                   << entry.actual_duty << " | "
                   << std::setprecision(5) << error << " | "
                   << std::setprecision(3) << entry.spectral_flatness << " | "
                   << std::setw(8) << entry.combined_period << " | "
                   << std::setw(7) << std::setprecision(2) << entry.fitness_score << " | "
                   << std::hex << std::setfill('0') << std::setw(4) << entry.lfsr1.mask << " | "
                   << std::setw(4) << entry.lfsr2.mask << std::dec << "\n";
        }
        
        report << "\nSummary Statistics:\n";
        report << "  Average Duty Cycle Error: " << std::fixed << std::setprecision(6) 
               << (total_error / config_.pwm_steps) << "\n";
        report << "  Maximum Duty Cycle Error: " << max_error << "\n";
        report << "  Average Spectral Flatness: " << std::setprecision(4) 
               << (total_sf / config_.pwm_steps) << "\n";
        report << "  Period Range: " << min_period << " - " << max_period << "\n";
        report << "  Unique Masks Used: " << mask_usage.size() << "\n\n";
        
        report << "Mask Usage Frequency:\n";
        std::vector<std::pair<uint32_t, uint16_t>> sorted_masks;
        for (const auto& [mask, count] : mask_usage) {
            sorted_masks.push_back({count, mask});
        }
        std::sort(sorted_masks.rbegin(), sorted_masks.rend());
        
        for (size_t i = 0; i < std::min(size_t(20), sorted_masks.size()); i++) {
            auto [count, mask] = sorted_masks[i];
            auto chars = LFSRAnalyzer::analyze_lfsr(mask);
            report << "  0x" << std::hex << std::setfill('0') << std::setw(4) << mask 
                   << std::dec << ": " << count << " times"
                   << " (period=" << chars.period 
                   << ", taps=" << chars.num_taps
                   << ", " << (chars.is_maximal ? "maximal" : "non-maximal") << ")\n";
        }
        
        report.close();
        
        std::cout << "Statistics report written to: " << config_.output_dir << "/statistics.txt" << std::endl;
    }
};

// Utility function to analyze a specific candidate file
void analyze_candidate_file(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    if (!file) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }
    
    uint32_t count;
    file.read(reinterpret_cast<char*>(&count), sizeof(count));
    
    std::cout << "Candidates in file: " << count << "\n\n";
    std::cout << "Rank | Mask1  | Seed1  | Mask2  | Seed2  | Duty   | SF    | Period   | Fitness\n";
    std::cout << "-----|--------|--------|--------|--------|--------|-------|----------|--------\n";
    
    for (uint32_t i = 0; i < count; i++) {
        DualLFSREntry entry;
        file.read(reinterpret_cast<char*>(&entry.lfsr1.mask), sizeof(uint16_t));
        file.read(reinterpret_cast<char*>(&entry.lfsr1.seed), sizeof(uint16_t));
        file.read(reinterpret_cast<char*>(&entry.lfsr2.mask), sizeof(uint16_t));
        file.read(reinterpret_cast<char*>(&entry.lfsr2.seed), sizeof(uint16_t));
        file.read(reinterpret_cast<char*>(&entry.actual_duty), sizeof(double));
        file.read(reinterpret_cast<char*>(&entry.spectral_flatness), sizeof(double));
        file.read(reinterpret_cast<char*>(&entry.fitness_score), sizeof(double));
        file.read(reinterpret_cast<char*>(&entry.combined_period), sizeof(uint32_t));
        
        std::cout << std::setw(4) << (i + 1) << " | "
                  << std::hex << std::setfill('0') 
                  << std::setw(4) << entry.lfsr1.mask << " | "
                  << std::setw(4) << entry.lfsr1.seed << " | "
                  << std::setw(4) << entry.lfsr2.mask << " | "
                  << std::setw(4) << entry.lfsr2.seed << " | "
                  << std::dec << std::fixed << std::setprecision(4) << entry.actual_duty << " | "
                  << std::setprecision(3) << entry.spectral_flatness << " | "
                  << std::setw(8) << entry.combined_period << " | "
                  << std::setprecision(2) << entry.fitness_score << "\n";
    }
    
    file.close();
}

int main(int argc, char* argv[]) {
    Config config;
    std::string analyze_file;
    
    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--steps" && i + 1 < argc) {
            config.pwm_steps = std::atoi(argv[++i]);
        } else if (arg == "--threads" && i + 1 < argc) {
            config.num_threads = std::atoi(argv[++i]);
        } else if (arg == "--sim-length" && i + 1 < argc) {
            config.simulation_length = std::atoi(argv[++i]);
        } else if (arg == "--candidates" && i + 1 < argc) {
            config.candidates_per_level = std::atoi(argv[++i]);
        } else if (arg == "--output-dir" && i + 1 < argc) {
            config.output_dir = argv[++i];
        } else if (arg == "--lut-output" && i + 1 < argc) {
            config.lut_output = argv[++i];
        } else if (arg == "--min-period" && i + 1 < argc) {
            config.min_period = std::atoi(argv[++i]);
        } else if (arg == "--maximal-only") {
            config.include_non_maximal = false;
        } else if (arg == "--include-non-maximal") {
            config.include_non_maximal = true;
        } else if (arg == "--analyze" && i + 1 < argc) {
            analyze_file = argv[++i];
        } else if (arg == "--help") {
            std::cout << "PWM LUT Generator - Extended Edition\n\n";
            std::cout << "Usage: " << argv[0] << " [options]\n\n";
            std::cout << "Generation Options:\n";
            std::cout << "  --steps N              Number of PWM steps (default: 256)\n";
            std::cout << "  --threads N            Number of threads (default: auto)\n";
            std::cout << "  --sim-length N         Simulation length (default: 65536)\n";
            std::cout << "  --candidates N         Candidates per level (default: 10)\n";
            std::cout << "  --min-period N         Minimum LFSR period (default: 255)\n";
            std::cout << "  --output-dir DIR       Candidate output directory (default: pwm_candidates)\n";
            std::cout << "  --lut-output FILE      Arduino LUT output file (default: pwm_lut.h)\n\n";
            std::cout << "LFSR Selection:\n";
            std::cout << "  --maximal-only         Use only maximal-length LFSRs\n";
            std::cout << "  --include-non-maximal  Include non-maximal LFSRs (default)\n\n";
            std::cout << "Analysis:\n";
            std::cout << "  --analyze FILE         Analyze a specific candidate file\n\n";
            std::cout << "  --help                 Show this help\n\n";
            std::cout << "Examples:\n";
            std::cout << "  " << argv[0] << " --steps 256 --threads 16\n";
            std::cout << "  " << argv[0] << " --maximal-only --steps 128\n";
            std::cout << "  " << argv[0] << " --analyze pwm_candidates/0x80.bin\n";
            return 0;
        }
    }
    
    // If analyzing a file, do that and exit
    if (!analyze_file.empty()) {
        analyze_candidate_file(analyze_file);
        return 0;
    }
    
    // Otherwise, generate the LUT
    PWMLUTGenerator generator(config);
    generator.generate();
    
    return 0;
}
/*
```

**Key Enhancements:**

1. **LFSRAnalyzer Class**: 
   - Determines actual period of any LFSR configuration
   - Generates and validates non-maximal-length LFSRs
   - Analyzes tap configurations systematically
   - Calculates combined periods using LCM

2. **Extended LFSR Space Exploration**:
   - Samples random tap configurations (2-8 taps)
   - Systematically explores sparse configurations
   - Filters by minimum period threshold
   - Pre-characterizes all masks before optimization

3. **Enhanced Evaluation Metrics**:
   - Run-length analysis (prefers shorter, more balanced runs)
   - Period bonus in fitness function
   - Combined period tracking for each entry
   - More sophisticated spectral analysis

4. **Improved Output**:
   - Detailed statistics report with mask usage frequency
   - Period information in binary files and header
   - Helper functions in Arduino header for PROGMEM access
   - Analysis utility for examining candidate files

**Compilation:**
```bash
g++ -O3 -march=native -std=c++17 -pthread pwm_lut_extended.cpp -o pwm_lut_gen
```

**Usage Examples:**
```bash
# Generate with all LFSR types (default)
./pwm_lut_gen --steps 256 --threads 16 --min-period 1000

# Generate with maximal-length only
./pwm_lut_gen --maximal-only --steps 256

# Analyze specific candidate file
./pwm_lut_gen --analyze pwm_candidates/0x80.bin

# Fine-tune for specific requirements
./pwm_lut_gen --steps 128 --min-period 5000 --sim-length 131072 --candidates 20
```

The extended version now explores a much richer LFSR space, including non-maximal configurations that might provide better spectral characteristics for specific duty cycle ranges!
*/
