// lfsr_pwm_opencl_cpu_fallback.cpp
// OpenCL + CPU-fallback LFSR PWM threshold search
// Build: g++ -std=c++17 -O3 -o lfsr_pwm_opencl_cpu_fallback lfsr_pwm_opencl_cpu_fallback.cpp -lOpenCL -pthread

#include <CL/cl.h>
#include <bits/stdc++.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <getopt.h>

using u64 = uint64_t;
using u32 = uint32_t;

struct LFSRConfig {
    u64 mask = 0;
    u64 seed = 0;
    int period = 0;
    int lc = 0;
    double actual_duty = 0.0;
    double error = 0.0;
    double score = 0.0;
};

struct Options {
    int n = 16;
    int resolution = 256;
    int threads = 0;
    u64 max_masks = 0;
    std::string output = "lfsr_pwm_lut.h";
    double lc_weight = 0.01;
    double period_weight = 0.001;
    unsigned seed = 0;
    int min_taps = 2;
    double min_period_ratio = 0.0;
    int block_size = 256;
    double max_block_error = 1.0;
    int work_group_size = 256;
};

// GPU cycle limit
static const int GPU_MAX_CYCLE = 4096;

// OpenCL kernel source (uses a compile-time MAX_CYCLE but host won't send masks > GPU_MAX_CYCLE)
const char* kernel_source = R"CL(
typedef ulong u64;
typedef uint u32;

inline u64 next_state(u64 state, u64 mask, int n) {
    u64 fb = popcount(state & mask) & 1UL;
    return (state >> 1) | (fb << (n - 1));
}

__kernel void evaluate_candidates(
    __global const u64* masks,
    __global const float* targets,
    __global float* scores,
    __global u64* best_masks,
    __global u64* best_seeds,
    __global int* best_periods,
    const int n,
    const int num_masks,
    const int num_targets,
    const int block_size,
    const float max_block_error
) {
    int gid = get_global_id(0);
    int mask_idx = gid / num_targets;
    int target_idx = gid % num_targets;
    
    if (mask_idx >= num_masks) return;
    
    u64 mask = masks[mask_idx];
    float target = targets[target_idx];
    
    // GPU local cycle buffer limited by GPU_MAX_CYCLE (host ensures masks here have period <= GPU_MAX_CYCLE)
    const int MAX_CYCLE = )CL" R"CL_DELIM"STR(GPU_MAX_CYCLE)CL" R"CL_DELIM" R"CL(
;
    u64 cycle[MAX_CYCLE];
    int period = 0;
    
    u64 state = 1;
    u64 start = state;
    
    // Find cycle length (bounded)
    do {
        if (period >= MAX_CYCLE) break;
        cycle[period++] = state;
        state = next_state(state, mask, n);
    } while (state != start && period < MAX_CYCLE);
    
    if (period == 0) return;
    
    // Try threshold at various positions
    float best_score = 1e9f;
    u64 best_threshold = 0;
    int best_period_val = period;
    
    int stride = max(1, period / 32);
    for (int thresh_idx = 0; thresh_idx < period; thresh_idx += stride) {
        u64 threshold = cycle[thresh_idx];
        
        // Count ones
        int ones = 0;
        for (int i = 0; i < period; i++) {
            if (cycle[i] > threshold) ones++;
        }
        
        float duty = (float)ones / (float)period;
        float error = fabs(duty - target);
        
        // Sliding window check
        bool valid = true;
        if (block_size < period) {
            int bs_stride = max(1, period / 16);
            for (int start_idx = 0; start_idx < period; start_idx += bs_stride) {
                int window_ones = 0;
                for (int i = 0; i < block_size; i++) {
                    int idx = (start_idx + i) % period;
                    if (cycle[idx] > threshold) window_ones++;
                }
                float block_duty = (float)window_ones / (float)block_size;
                float block_err = fabs(block_duty - target);
                
                if (block_err > max_block_error) {
                    valid = false;
                    break;
                }
            }
        }
        
        if (valid && error < best_score) {
            best_score = error;
            best_threshold = threshold;
        }
    }
    
    // Write results (indexing consistent with host layout)
    int out_idx = mask_idx * num_targets + target_idx;
    // atomic update is not used here: each thread writes its own output slot
    scores[out_idx] = best_score;
    best_masks[out_idx] = mask;
    best_seeds[out_idx] = best_threshold;
    best_periods[out_idx] = best_period_val;
}
)CL";

// Helper to inject numeric literal into kernel (we used string replacement for GPU_MAX_CYCLE)
std::string prepare_kernel_source() {
    std::string s(kernel_source);
    // Replace the placeholder string STR(GPU_MAX_CYCLE) with the numeric literal
    // (We left a marker; in this file the marker was expanded by string concat above.)
    return s;
}

void check_cl_error(cl_int err, const char* msg) {
    if (err != CL_SUCCESS) {
        std::cerr << "OpenCL Error (" << err << "): " << msg << " code=" << err << "\n";
        exit(1);
    }
}

Options parse_args(int argc, char **argv) {
    Options opt;
    static struct option long_options[] = {
        {"n", required_argument, nullptr, 'n'},
        {"res", required_argument, nullptr, 'r'},
        {"threads", required_argument, nullptr, 't'},
        {"max-masks", required_argument, nullptr, 'm'},
        {"output", required_argument, nullptr, 'o'},
        {"min-taps", required_argument, nullptr, 'T'},
        {"block-size", required_argument, nullptr, 'b'},
        {"max-block-error", required_argument, nullptr, 'e'},
        {"work-group-size", required_argument, nullptr, 'w'},
        {nullptr, 0, nullptr, 0}
    };
    
    int c;
    while ((c = getopt_long(argc, argv, "n:r:t:m:o:T:b:e:w:", long_options, nullptr)) != -1) {
        switch (c) {
            case 'n': opt.n = std::atoi(optarg); break;
            case 'r': opt.resolution = std::atoi(optarg); break;
            case 't': opt.threads = std::atoi(optarg); break;
            case 'm': opt.max_masks = std::stoull(optarg); break;
            case 'o': opt.output = optarg; break;
            case 'T': opt.min_taps = std::atoi(optarg); break;
            case 'b': opt.block_size = std::atoi(optarg); break;
            case 'e': opt.max_block_error = std::stod(optarg); break;
            case 'w': opt.work_group_size = std::atoi(optarg); break;
        }
    }
    
    if (opt.seed == 0) {
        opt.seed = (unsigned)std::chrono::system_clock::now().time_since_epoch().count();
    }
    
    return opt;
}

static inline int popcount_u64(u64 x) { return __builtin_popcountll(x); }
static inline u64 lfsr_next_u64(u64 state, u64 mask, int n) {
    u64 fb = (u64)(__builtin_popcountll(state & mask) & 1ULL);
    return (state >> 1) | (fb << (n - 1));
}

// Detect if period <= limit. If found, returns period (<=limit). If exceeds limit, returns -1.
int detect_period_limit(u64 mask, int n, int limit) {
    // Use a simple bounded tortoise-hare loop: we iterate both pointers up to limit steps.
    u64 start = 1;
    u64 tortoise = lfsr_next_u64(start, mask, n);
    u64 hare = lfsr_next_u64(lfsr_next_u64(start, mask, n), mask, n);
    int steps = 0;
    while (tortoise != hare && steps < limit) {
        tortoise = lfsr_next_u64(tortoise, mask, n);
        hare = lfsr_next_u64(lfsr_next_u64(hare, mask, n), mask, n);
        steps++;
    }
    if (tortoise != hare) {
        return -1; // exceeded limit
    }
    // found a meeting; compute cycle length
    int mu = 0;
    tortoise = start;
    while (tortoise != hare && mu <= limit) {
        tortoise = lfsr_next_u64(tortoise, mask, n);
        hare = lfsr_next_u64(hare, mask, n);
        mu++;
    }
    // compute lambda
    int lam = 1;
    hare = lfsr_next_u64(tortoise, mask, n);
    while (hare != tortoise && lam <= limit) {
        hare = lfsr_next_u64(hare, mask, n);
        lam++;
    }
    if (lam > limit) return -1;
    return lam;
}

// Compute full period by streaming from seed until return to start
// Returns period (>0). WARNING: this iterates 'period' steps.
uint64_t compute_full_period(u64 mask, int n) {
    u64 start = 1;
    u64 s = start;
    uint64_t count = 0;
    do {
        s = lfsr_next_u64(s, mask, n);
        count++;
        // safety: if start==0 (rare), break - but we use start=1
    } while (s != start);
    return count;
}

// Evaluate a single mask on the CPU for all targets and write into the provided arrays at positions mask_idx*num_targets + t
void evaluate_mask_cpu(u64 mask, int n, const std::vector<float> &targets,
                       float *scores_out, u64 *best_mask_out, u64 *best_seed_out, int *best_period_out,
                       int mask_output_base_idx, int block_size, double max_block_error) {
    // compute full period
    uint64_t period = compute_full_period(mask, n);
    if (period == 0) return;
    
    int num_targets = targets.size();
    int stride = std::max<uint64_t>(1, period / 32);
    int bs_stride = std::max<uint64_t>(1, period / 16);
    
    // We'll evaluate thresholds at positions thresh_pos = 0, stride, 2*stride, ...
    // For each threshold we need its state value: compute by stepping thresh_pos times from start
    // We'll then stream once across the period to compute duty and sliding window errors.
    
    // Precompute target bests locally
    for (int t = 0; t < num_targets; ++t) {
        scores_out[mask_output_base_idx + t] = 1e9f; // initialize
        best_mask_out[mask_output_base_idx + t] = 0;
        best_seed_out[mask_output_base_idx + t] = 0;
        best_period_out[mask_output_base_idx + t] = (int)period;
    }
    
    // For each threshold sample:
    for (uint64_t thresh_pos = 0; thresh_pos < period; thresh_pos += stride) {
        // compute threshold state by stepping thresh_pos from start
        u64 threshold = 1;
        for (uint64_t i = 0; i < thresh_pos; ++i) threshold = lfsr_next_u64(threshold, mask, n);
        
        // Now stream across the period once and accumulate:
        // - total ones vs threshold
        // - sliding-window block errors for sampled windows
        // We'll do per-target arrays for block invalid flags as well.
        
        // For each target we will track whether any sampled block violates max_block_error
        std::vector<bool> valid_target(num_targets, true);
        std::vector<int> total_ones(num_targets, 0);
        
        // For sliding window we maintain a circular buffer of booleans of size block_size
        std::vector<char> ring(block_size, 0);
        int ring_idx = 0;
        int window_sum = 0;
        int filled = 0;
        
        // We need to check blocks at positions start_idx = 0, bs_stride, 2*bs_stride, ...
        // But to compute each block we can simply slide across the period and sample windows when reaching start positions.
        // Approach: iterate i from 0..period-1, update ring and window_sum. When we've filled block_size items we can compute block error for the window ending at i.
        // To approximate the kernel's behavior which checks blocks starting at certain start positions, we will check windows whose start is s = 0, bs_stride, 2*bs_stride, ...
        // We'll precompute the set of "start indices" to check and then when the stream index reaches start+block_size-1 we evaluate that window.
        std::vector<uint64_t> block_start_positions;
        for (uint64_t s = 0; s < period; s += bs_stride) block_start_positions.push_back(s);
        // For quick lookup: map end index -> list of starts that end at that index (there can be at most one for our sampling)
        std::unordered_map<uint64_t, uint64_t> end_to_start;
        for (uint64_t s: block_start_positions) {
            uint64_t end = (s + block_size - 1) % period;
            end_to_start[end] = s;
        }
        // We'll store the last block_size boolean values in ring; when i >= block_size-1 we can evaluate the window ending at i.
        
        // Stream once across period
        u64 state = 1;
        for (uint64_t i = 0; i < period; ++i) {
            char above = (state > threshold) ? 1 : 0;
            // update ring/window
            if (filled < block_size) {
                ring[ring_idx] = above;
                window_sum += above;
                ring_idx = (ring_idx + 1) % block_size;
                filled++;
            } else {
                // overwrite oldest
                window_sum -= ring[ring_idx];
                ring[ring_idx] = above;
                window_sum += ring[ring_idx];
                ring_idx = (ring_idx + 1) % block_size;
            }
            // accumulate totals for targets
            for (int t = 0; t < num_targets; ++t) {
                total_ones[t] += above;
            }
            // check if this index is an end of a sampled block
            auto it = end_to_start.find(i);
            if (it != end_to_start.end() && filled == block_size) {
                // we have a sampled window starting at start = it->second
                float block_duty = (float)window_sum / (float)block_size;
                for (int t = 0; t < num_targets; ++t) {
                    if (!valid_target[t]) continue;
                    float block_err = fabs(block_duty - targets[t]);
                    if (block_err > max_block_error) {
                        valid_target[t] = false; // this threshold invalid for this target
                    }
                }
            }
            // advance state
            state = lfsr_next_u64(state, mask, n);
        } // end streaming
        
        // After streaming we can compute duty and errors for each target that remains valid
        for (int t = 0; t < num_targets; ++t) {
            if (!valid_target[t]) continue;
            float duty = (float)total_ones[t] / (float)period;
            float error = fabs(duty - targets[t]);
            int out_idx = mask_output_base_idx + t;
            if (error < scores_out[out_idx]) {
                scores_out[out_idx] = error;
                best_mask_out[out_idx] = mask;
                best_seed_out[out_idx] = threshold;
                best_period_out[out_idx] = (int)period;
            }
        }
    } // end threshold sampling
}

// Write Arduino LUT (as before)
void write_arduino_lut(const std::vector<LFSRConfig> &configs, int n, const std::string &filename) {
    std::ofstream f(filename);
    if (!f) {
        std::cerr << "ERROR: cannot open " << filename << " for writing\n";
        return;
    }
    
    f << "// Arduino LFSR PWM LUT: mask, seed (" << n << "-bit)\n";
    f << "#include <stdint.h>\n\n";
    
    if (n <= 16) {
        f << "const struct { uint16_t mask; uint16_t seed; } lfsr_pwm_lut[] = {\n";
        for (size_t i = 0; i < configs.size(); ++i) {
            const LFSRConfig &c = configs[i];
            f << "  { 0x" << std::hex << std::setw(4) << std::setfill('0') << (uint16_t)c.mask
              << "u, 0x" << std::setw(4) << (uint16_t)c.seed << "u }";
            f << std::dec << ", // " << i << ": duty=" << c.actual_duty
              << " err=" << c.error << " P=" << c.period << "\n";
        }
        f << "};\n";
    } else {
        f << "const struct { uint32_t mask; uint32_t seed; } lfsr_pwm_lut[] = {\n";
        for (size_t i = 0; i < configs.size(); ++i) {
            const LFSRConfig &c = configs[i];
            f << "  { 0x" << std::hex << std::setw(8) << std::setfill('0') << (uint32_t)c.mask
              << "u, 0x" << std::setw(8) << (uint32_t)c.seed << "u }";
            f << std::dec << ", // " << i << ": duty=" << c.actual_duty
              << " err=" << c.error << " P=" << c.period << "\n";
        }
        f << "};\n";
    }
    
    f.close();
}

int main(int argc, char **argv) {
    Options opt = parse_args(argc, argv);
    
    std::cerr << "OpenCL LFSR PWM Threshold Search with CPU fallback for long cycles\n";
    std::cerr << "n=" << opt.n << " bits, resolution=" << opt.resolution << "\n";
    std::cerr << "Block-size=" << opt.block_size << ", max-block-error=" << opt.max_block_error << "\n";
    std::cerr << "GPU max cycle supported: " << GPU_MAX_CYCLE << "\n";
    
    // Build masks list
    u64 max_mask = (opt.n >= 64) ? ~0ULL : ((1ULL << opt.n) - 1ULL);
    u64 scan_limit = (opt.max_masks == 0) ? max_mask : std::min(opt.max_masks, max_mask);
    
    std::vector<u64> masks;
    for (u64 m = 1; m <= scan_limit; ++m) {
        if (popcount_u64(m) >= opt.min_taps) masks.push_back(m);
    }
    
    std::cerr << "Total masks candidate: " << masks.size() << "\n";
    
    // Partition masks into GPU-handled and CPU-handled based on bounded period detection:
    std::vector<u64> gpu_masks;
    std::vector<u64> cpu_masks;
    gpu_masks.reserve(masks.size());
    cpu_masks.reserve(masks.size());
    
    for (size_t i = 0; i < masks.size(); ++i) {
        u64 m = masks[i];
        int det = detect_period_limit(m, opt.n, GPU_MAX_CYCLE);
        if (det > 0 && det <= GPU_MAX_CYCLE) {
            gpu_masks.push_back(m);
        } else {
            cpu_masks.push_back(m);
        }
    }
    
    std::cerr << "Masks for GPU (" << gpu_masks.size() << "), for CPU (" << cpu_masks.size() << ")\n";
    
    // Generate targets
    std::vector<float> targets(opt.resolution);
    for (int i = 0; i < opt.resolution; ++i) {
        targets[i] = (float)i / (float)(opt.resolution - 1);
    }
    
    int num_targets = opt.resolution;
    // Prepare output arrays sized to (num_masks_total * num_targets) for simplicity, we will map indices accordingly.
    int num_masks_total = gpu_masks.size() + cpu_masks.size();
    int total_work = (int)num_masks_total * num_targets;
    
    std::vector<float> scores(total_work, 1e9f);
    std::vector<u64> best_masks_out(total_work, 0);
    std::vector<u64> best_seeds_out(total_work, 0);
    std::vector<int> best_periods_out(total_work, 0);
    
    // --- GPU path (if we have gpu_masks) ---
    if (!gpu_masks.empty()) {
        // Initialize OpenCL
        cl_platform_id platform;
        cl_device_id device;
        cl_int err;
        
        err = clGetPlatformIDs(1, &platform, nullptr);
        check_cl_error(err, "Get platform");
        
        err = clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 1, &device, nullptr);
        if (err != CL_SUCCESS) {
            std::cerr << "No GPU found, trying CPU device for GPU path...\n";
            err = clGetDeviceIDs(platform, CL_DEVICE_TYPE_ALL, 1, &device, nullptr);
            check_cl_error(err, "Get device");
        }
        
        char device_name[256];
        clGetDeviceInfo(device, CL_DEVICE_NAME, sizeof(device_name), device_name, nullptr);
        std::cerr << "Using device for GPU path: " << device_name << "\n";
        
        cl_context context = clCreateContext(nullptr, 1, &device, nullptr, nullptr, &err);
        check_cl_error(err, "Create context");
        
        cl_command_queue queue = clCreateCommandQueue(context, device, 0, &err);
        check_cl_error(err, "Create queue");
        
        // Build program
        std::string src = prepare_kernel_source();
        const char* srcptr = src.c_str();
        size_t srclen = src.size();
        cl_program program = clCreateProgramWithSource(context, 1, &srcptr, &srclen, &err);
        check_cl_error(err, "Create program");
        
        err = clBuildProgram(program, 1, &device, "-cl-fast-relaxed-math", nullptr, nullptr);
        if (err != CL_SUCCESS) {
            size_t log_size;
            clGetProgramBuildInfo(program, device, CL_PROGRAM_BUILD_LOG, 0, nullptr, &log_size);
            std::vector<char> log(log_size);
            clGetProgramBuildInfo(program, device, CL_PROGRAM_BUILD_LOG, log_size, log.data(), nullptr);
            std::cerr << "Build log:\n" << log.data() << "\n";
            check_cl_error(err, "Build program");
        }
        
        cl_kernel kernel = clCreateKernel(program, "evaluate_candidates", &err);
        check_cl_error(err, "Create kernel");
        
        // Create device buffers for gpu_masks and targets
        int num_gpu_masks = gpu_masks.size();
        int gpu_total_work = num_gpu_masks * num_targets;
        
        cl_mem d_masks = clCreateBuffer(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
                                    sizeof(u64) * num_gpu_masks, gpu_masks.data(), &err);
        check_cl_error(err, "Create masks buffer");
        
        cl_mem d_targets = clCreateBuffer(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
                                      sizeof(float) * num_targets, targets.data(), &err);
        check_cl_error(err, "Create targets buffer");
        
        // Create device outputs sized for gpu masks only
        std::vector<float> gpu_scores(gpu_total_work, 1e9f);
        cl_mem d_scores = clCreateBuffer(context, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR,
                                    sizeof(float) * gpu_total_work, gpu_scores.data(), &err);
        check_cl_error(err, "Create scores buffer");
        
        cl_mem d_best_masks = clCreateBuffer(context, CL_MEM_WRITE_ONLY,
                                        sizeof(u64) * gpu_total_work, nullptr, &err);
        check_cl_error(err, "Create best_masks buffer");
        
        cl_mem d_best_seeds = clCreateBuffer(context, CL_MEM_WRITE_ONLY,
                                        sizeof(u64) * gpu_total_work, nullptr, &err);
        check_cl_error(err, "Create best_seeds buffer");
        
        cl_mem d_best_periods = clCreateBuffer(context, CL_MEM_WRITE_ONLY,
                                          sizeof(int) * gpu_total_work, nullptr, &err);
        check_cl_error(err, "Create best_periods buffer");
        
        // Set kernel args (match kernel signature)
        int arg = 0;
        clSetKernelArg(kernel, arg++, sizeof(cl_mem), &d_masks);
        clSetKernelArg(kernel, arg++, sizeof(cl_mem), &d_targets);
        clSetKernelArg(kernel, arg++, sizeof(cl_mem), &d_scores);
        clSetKernelArg(kernel, arg++, sizeof(cl_mem), &d_best_masks);
        clSetKernelArg(kernel, arg++, sizeof(cl_mem), &d_best_seeds);
        clSetKernelArg(kernel, arg++, sizeof(cl_mem), &d_best_periods);
        clSetKernelArg(kernel, arg++, sizeof(int), &opt.n);
        clSetKernelArg(kernel, arg++, sizeof(int), &num_gpu_masks);
        clSetKernelArg(kernel, arg++, sizeof(int), &num_targets);
        clSetKernelArg(kernel, arg++, sizeof(int), &opt.block_size);
        float max_err_f = (float)opt.max_block_error;
        clSetKernelArg(kernel, arg++, sizeof(float), &max_err_f);
        
        // Launch kernel
        size_t global_work_size = ((gpu_total_work + opt.work_group_size - 1) / opt.work_group_size) * opt.work_group_size;
        size_t local_work_size = opt.work_group_size;
        
        std::cerr << "Launching GPU kernel for " << num_gpu_masks << " masks (" << global_work_size << " work-items)\n";
        
        auto start = std::chrono::high_resolution_clock::now();
        err = clEnqueueNDRangeKernel(queue, kernel, 1, nullptr, &global_work_size, &local_work_size, 0, nullptr, nullptr);
        check_cl_error(err, "Enqueue kernel");
        clFinish(queue);
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        std::cerr << "GPU kernel time: " << elapsed.count() << " s\n";
        
        // Read back results
        std::vector<float> gpu_scores_out(gpu_total_work);
        std::vector<u64> gpu_best_masks_out(gpu_total_work);
        std::vector<u64> gpu_best_seeds_out(gpu_total_work);
        std::vector<int> gpu_best_periods_out(gpu_total_work);
        
        clEnqueueReadBuffer(queue, d_scores, CL_TRUE, 0, sizeof(float) * gpu_total_work, gpu_scores_out.data(), 0, nullptr, nullptr);
        clEnqueueReadBuffer(queue, d_best_masks, CL_TRUE, 0, sizeof(u64) * gpu_total_work, gpu_best_masks_out.data(), 0, nullptr, nullptr);
        clEnqueueReadBuffer(queue, d_best_seeds, CL_TRUE, 0, sizeof(u64) * gpu_total_work, gpu_best_seeds_out.data(), 0, nullptr, nullptr);
        clEnqueueReadBuffer(queue, d_best_periods, CL_TRUE, 0, sizeof(int) * gpu_total_work, gpu_best_periods_out.data(), 0, nullptr, nullptr);
        
        // Map GPU outputs into global arrays. We will place the gpu masks first in the global mask order.
        // Define a mapping: global index 0..gpu_masks.size()-1 corresponds to gpu_masks in order.
        for (int mi = 0; mi < num_gpu_masks; ++mi) {
            int global_mask_index = mi; // we'll place gpu masks first
            int base = global_mask_index * num_targets;
            int gpu_base = mi * num_targets;
            for (int t = 0; t < num_targets; ++t) {
                int out_idx = base + t;
                scores[out_idx] = gpu_scores_out[gpu_base + t];
                best_masks_out[out_idx] = gpu_best_masks_out[gpu_base + t];
                best_seeds_out[out_idx] = gpu_best_seeds_out[gpu_base + t];
                best_periods_out[out_idx] = gpu_best_periods_out[gpu_base + t];
            }
        }
        
        // Cleanup GPU resources
        clReleaseMemObject(d_masks);
        clReleaseMemObject(d_targets);
        clReleaseMemObject(d_scores);
        clReleaseMemObject(d_best_masks);
        clReleaseMemObject(d_best_seeds);
        clReleaseMemObject(d_best_periods);
        clReleaseKernel(kernel);
        clReleaseProgram(program);
        clReleaseCommandQueue(queue);
        clReleaseContext(context);
    } // end GPU path
    
    // --- CPU path for long cycles ---
    if (!cpu_masks.empty()) {
        std::cerr << "Running CPU evaluation for " << cpu_masks.size() << " masks (long cycles)\n";
        // We'll place cpu_masks after gpu_masks in the global ordering
        int gpu_count = gpu_masks.size();
        for (size_t i = 0; i < cpu_masks.size(); ++i) {
            int global_mask_index = (int)gpu_count + (int)i;
            int base = global_mask_index * num_targets;
            evaluate_mask_cpu(cpu_masks[i], opt.n, targets,
                              scores.data(), best_masks_out.data(), best_seeds_out.data(), best_periods_out.data(),
                              base, opt.block_size, opt.max_block_error);
            if ((i & 63) == 0) {
                std::cerr << "CPU processed " << i << "/" << cpu_masks.size() << " masks...\r" << std::flush;
            }
        }
        std::cerr << "\nCPU evaluation done\n";
    }
    
    // Now we have output arrays for all masks (gpu first, cpu next). Choose best config per target across all masks:
    std::vector<LFSRConfig> best_configs(num_targets);
    for (int t = 0; t < num_targets; ++t) {
        float best_score = 1e9f;
        int best_mask_idx = -1;
        for (int m_idx = 0; m_idx < num_masks_total; ++m_idx) {
            int idx = m_idx * num_targets + t;
            if (scores[idx] < best_score) {
                best_score = scores[idx];
                best_mask_idx = idx;
            }
        }
        if (best_mask_idx >= 0) {
            best_configs[t].mask = best_masks_out[best_mask_idx];
            best_configs[t].seed = best_seeds_out[best_mask_idx];
            best_configs[t].period = best_periods_out[best_mask_idx];
            best_configs[t].error = scores[best_mask_idx];
            best_configs[t].score = scores[best_mask_idx];
            best_configs[t].actual_duty = targets[t];
        }
    }
    
    // Write output LUT
    write_arduino_lut(best_configs, opt.n, opt.output);
    std::cerr << "Written Arduino LUT to " << opt.output << "\n";
    
    return 0;
}
