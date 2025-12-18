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
};

// LFSR parameter set
struct LFSRParams {
    uint16_t mask;
    uint16_t seed;
    
    LFSRParams() : mask(0), seed(0) {}
    LFSRParams(uint16_t m, uint16_t s) : mask(m), seed(s) {}
};

// Dual LFSR entry
struct DualLFSREntry {
    LFSRParams lfsr1;
    LFSRParams lfsr2;
    double actual_duty;
    double spectral_flatness;
    double fitness_score;
    
    bool operator<(const DualLFSREntry& other) const {
        return fitness_score > other.fitness_score;  // Higher is better
    }
};

// Maximal-length LFSR tap configurations (Fibonacci/Galois)
const std::vector<uint16_t> MAXIMAL_TAPS_16 = {
    0xB400, 0xD008, 0xB000, 0xA001, 0x9000,
    0x8016, 0x8005, 0x8003, 0x4001, 0x3802
};

class LFSRSimulator {
private:
    // Fast LFSR advancement using GCC builtin popcount
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
        
        for (uint32_t i = 0; i < length; i++) {
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
        // Convert to centered values for better spectral analysis
        std::vector<double> signal(sequence.size());
        for (size_t i = 0; i < sequence.size(); i++) {
            signal[i] = sequence[i] ? 1.0 : -1.0;
        }
        
        // Simple power spectrum estimation via autocorrelation
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
        
        // Calculate spectral flatness (geometric mean / arithmetic mean)
        for (auto p : power_spectrum) {
            if (p > 1e-10) {  // Avoid log(0)
                geometric_mean += std::log(p);
                arithmetic_mean += p;
            }
        }
        
        geometric_mean = std::exp(geometric_mean / power_spectrum.size());
        arithmetic_mean /= power_spectrum.size();
        
        if (arithmetic_mean < 1e-10) return 0.0;
        return geometric_mean / arithmetic_mean;
    }
    
    // Calculate transition density (prefer more transitions)
    static double calculate_transition_density(const std::vector<uint8_t>& sequence) {
        uint32_t transitions = 0;
        for (size_t i = 1; i < sequence.size(); i++) {
            if (sequence[i] != sequence[i-1]) {
                transitions++;
            }
        }
        return static_cast<double>(transitions) / (sequence.size() - 1);
    }
    
    // Evaluate a dual LFSR configuration
    static DualLFSREntry evaluate_configuration(
        uint16_t mask1, uint16_t seed1,
        uint16_t mask2, uint16_t seed2,
        double target_duty,
        uint32_t sim_length,
        double duty_tolerance)
    {
        DualLFSREntry entry;
        entry.lfsr1 = LFSRParams(mask1, seed1);
        entry.lfsr2 = LFSRParams(mask2, seed2);
        
        // Simulate
        auto sequence = simulate_dual_lfsr_avx2(mask1, seed1, mask2, seed2, sim_length);
        
        // Calculate metrics
        entry.actual_duty = calculate_duty_cycle(sequence);
        entry.spectral_flatness = calculate_spectral_flatness(sequence);
        double transition_density = calculate_transition_density(sequence);
        
        // Duty cycle error
        double duty_error = std::abs(entry.actual_duty - target_duty);
        
        // Fitness function: heavily penalize duty error, reward spectral flatness
        if (duty_error > duty_tolerance) {
            entry.fitness_score = -1000.0 * duty_error;  // Strong penalty
        } else {
            entry.fitness_score = 
                100.0 * entry.spectral_flatness +
                50.0 * transition_density -
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
    
    // Generate seed variations around base values
    std::vector<uint16_t> generate_seed_candidates(uint16_t base_seed, uint32_t count) {
        std::vector<uint16_t> seeds;
        seeds.push_back(base_seed);
        
        // Add variations
        std::mt19937 rng(base_seed);
        std::uniform_int_distribution<uint16_t> dist(1, 0xFFFF);
        
        for (uint32_t i = 1; i < count && seeds.size() < count; i++) {
            uint16_t seed = dist(rng);
            if (seed != 0) {  // Avoid all-zero state
                seeds.push_back(seed);
            }
        }
        
        return seeds;
    }
    
    // Find optimal LFSR pairs for a target duty cycle
    std::vector<DualLFSREntry> find_optimal_pairs_for_duty(double target_duty) {
        std::vector<DualLFSREntry> candidates;
        
        // Strategy: Try combinations of maximal-length LFSRs with varied seeds
        for (auto mask1 : MAXIMAL_TAPS_16) {
            for (auto mask2 : MAXIMAL_TAPS_16) {
                // Generate seed candidates
                auto seeds1 = generate_seed_candidates(0x1234, 20);
                auto seeds2 = generate_seed_candidates(0x5678, 20);
                
                for (auto seed1 : seeds1) {
                    for (auto seed2 : seeds2) {
                        auto entry = LFSRSimulator::evaluate_configuration(
                            mask1, seed1, mask2, seed2,
                            target_duty, config_.simulation_length,
                            config_.duty_tolerance
                        );
                        
                        if (entry.fitness_score > -500.0) {  // Only keep reasonable candidates
                            candidates.push_back(entry);
                        }
                    }
                }
            }
        }
        
        // Sort by fitness and keep top N
        std::sort(candidates.begin(), candidates.end());
        if (candidates.size() > config_.candidates_per_level) {
            candidates.resize(config_.candidates_per_level);
        }
        
        return candidates;
    }
    
    // Process a single PWM level
    void process_pwm_level(uint32_t level) {
        double target_duty = static_cast<double>(level) / (config_.pwm_steps - 1);
        
        auto candidates = find_optimal_pairs_for_duty(target_duty);
        
        // Save candidates to binary file
        save_candidates_to_file(level, candidates);
        
        // Progress update
        uint32_t completed = ++completed_levels_;
        if (completed % 10 == 0 || completed == config_.pwm_steps) {
            std::lock_guard<std::mutex> lock(output_mutex_);
            std::cout << "Progress: " << completed << "/" << config_.pwm_steps 
                      << " levels completed (" 
                      << (100 * completed / config_.pwm_steps) << "%)" << std::endl;
        }
    }
    
    // Save candidates to binary file
    void save_candidates_to_file(uint32_t level, const std::vector<DualLFSREntry>& candidates) {
        std::ostringstream filename;
        filename << config_.output_dir << "/0x" 
                 << std::hex << std::setfill('0') << std::setw(2) << level << ".bin";
        
        std::ofstream file(filename.str(), std::ios::binary);
        if (!file) {
            std::cerr << "Failed to create file: " << filename.str() << std::endl;
            return;
        }
        
        // Write number of candidates
        uint32_t count = candidates.size();
        file.write(reinterpret_cast<const char*>(&count), sizeof(count));
        
        // Write each candidate
        for (const auto& entry : candidates) {
            file.write(reinterpret_cast<const char*>(&entry.lfsr1.mask), sizeof(uint16_t));
            file.write(reinterpret_cast<const char*>(&entry.lfsr1.seed), sizeof(uint16_t));
            file.write(reinterpret_cast<const char*>(&entry.lfsr2.mask), sizeof(uint16_t));
            file.write(reinterpret_cast<const char*>(&entry.lfsr2.seed), sizeof(uint16_t));
            file.write(reinterpret_cast<const char*>(&entry.actual_duty), sizeof(double));
            file.write(reinterpret_cast<const char*>(&entry.spectral_flatness), sizeof(double));
            file.write(reinterpret_cast<const char*>(&entry.fitness_score), sizeof(double));
        }
        
        file.close();
    }
    
    // Load best candidate from file
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
        
        // Read first (best) candidate
        DualLFSREntry entry;
        file.read(reinterpret_cast<char*>(&entry.lfsr1.mask), sizeof(uint16_t));
        file.read(reinterpret_cast<char*>(&entry.lfsr1.seed), sizeof(uint16_t));
        file.read(reinterpret_cast<char*>(&entry.lfsr2.mask), sizeof(uint16_t));
        file.read(reinterpret_cast<char*>(&entry.lfsr2.seed), sizeof(uint16_t));
        file.read(reinterpret_cast<char*>(&entry.actual_duty), sizeof(double));
        file.read(reinterpret_cast<char*>(&entry.spectral_flatness), sizeof(double));
        file.read(reinterpret_cast<char*>(&entry.fitness_score), sizeof(double));
        
        return entry;
    }

public:
    PWMLUTGenerator(const Config& config) : config_(config) {
        if (config_.num_threads == 0) {
            config_.num_threads = std::thread::hardware_concurrency();
        }
    }
    
    void generate() {
        std::cout << "PWM LUT Generator" << std::endl;
        std::cout << "=================" << std::endl;
        std::cout << "PWM Steps: " << config_.pwm_steps << std::endl;
        std::cout << "Threads: " << config_.num_threads << std::endl;
        std::cout << "Simulation Length: " << config_.simulation_length << std::endl;
        std::cout << "Candidates per level: " << config_.candidates_per_level << std::endl;
        std::cout << std::endl;
        
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
        
        // Header
        file << "// Auto-generated PWM LUT for dual LFSR XOR\n";
        file << "// Generated with " << config_.pwm_steps << " PWM levels\n";
        file << "#ifndef PWM_LUT_H\n";
        file << "#define PWM_LUT_H\n\n";
        file << "#include <avr/pgmspace.h>\n\n";
        
        // Structure definition
        file << "struct DualLFSREntry {\n";
        file << "  uint16_t mask1;\n";
        file << "  uint16_t seed1;\n";
        file << "  uint16_t mask2;\n";
        file << "  uint16_t seed2;\n";
        file << "};\n\n";
        
        // LUT array
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
                 << ", SF: " << entry.spectral_flatness << ")\n";
        }
        
        file << "};\n\n";
        file << "#endif // PWM_LUT_H\n";
        
        file.close();
        
        std::cout << "Arduino LUT written to: " << config_.lut_output << std::endl;
    }
};

int main(int argc, char* argv[]) {
    Config config;
    
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
        } else if (arg == "--help") {
            std::cout << "Usage: " << argv[0] << " [options]\n";
            std::cout << "Options:\n";
            std::cout << "  --steps N          Number of PWM steps (default: 256)\n";
            std::cout << "  --threads N        Number of threads (default: auto)\n";
            std::cout << "  --sim-length N     Simulation length (default: 65536)\n";
            std::cout << "  --candidates N     Candidates per level (default: 10)\n";
            std::cout << "  --output-dir DIR   Candidate output directory (default: pwm_candidates)\n";
            std::cout << "  --lut-output FILE  Arduino LUT output file (default: pwm_lut.h)\n";
            std::cout << "  --help             Show this help\n";
            return 0;
        }
    }
    
    PWMLUTGenerator generator(config);
    generator.generate();
    
    return 0;
}
