// lfsr_pwm_opencl.cpp
// OpenCL-accelerated LFSR PWM threshold search
// Build: g++ -std=c++17 -O3 -o lfsr_pwm_opencl lfsr_pwm_opencl.cpp -lOpenCL -pthread

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

// OpenCL kernel source
const char* kernel_source = R"CL(
typedef ulong u64;
typedef uint u32;

// Next LFSR state
inline u64 next_state(u64 state, u64 mask, int n) {
    u64 fb = popcount(state & mask) & 1UL;
    return (state >> 1) | (fb << (n - 1));
}

// Find cycle starting from a given state
int find_cycle(__global uchar* visited, u64 start, u64 mask, int n, 
               __global u64* cycle_out, int max_cycle_len) {
    const u64 max_state = (1UL << n);
    u64 curr = start;
    int len = 0;
    
    // Walk until we hit a visited state
    while (len < max_cycle_len) {
        int vis_idx = (int)curr;
        if (atomic_cmpxchg(&visited[vis_idx], 0, 1) != 0) {
            break;
        }
        cycle_out[len++] = curr;
        curr = next_state(curr, mask, n);
    }
    
    return len;
}

// Evaluate a single mask-threshold-target combination
__kernel void evaluate_candidates(
    __global const u64* masks,
    __global const u64* thresholds, 
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
    
    // Simple cycle detection (limited length for GPU)
    const int MAX_CYCLE = 4096;
    u64 cycle[MAX_CYCLE];
    int period = 0;
    
    u64 state = 1;
    u64 start = state;
    
    // Find cycle length
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
    
    for (int thresh_idx = 0; thresh_idx < period; thresh_idx += max(1, period/32)) {
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
            for (int start = 0; start < period; start += max(1, period/16)) {
                int window_ones = 0;
                for (int i = 0; i < block_size; i++) {
                    int idx = (start + i) % period;
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
    
    // Write results
    int out_idx = mask_idx * num_targets + target_idx;
    if (best_score < scores[out_idx]) {
        scores[out_idx] = best_score;
        best_masks[out_idx] = mask;
        best_seeds[out_idx] = best_threshold;
        best_periods[out_idx] = best_period_val;
    }
}
)CL";

void check_cl_error(cl_int err, const char* msg) {
    if (err != CL_SUCCESS) {
        std::cerr << "OpenCL Error (" << err << "): " << msg << "\n";
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
    
    std::cerr << "OpenCL LFSR PWM Threshold Search\n";
    std::cerr << "n=" << opt.n << " bits, resolution=" << opt.resolution << "\n";
    std::cerr << "Block-size=" << opt.block_size << ", max-block-error=" << opt.max_block_error << "\n";
    
    // Initialize OpenCL
    cl_platform_id platform;
    cl_device_id device;
    cl_int err;
    
    err = clGetPlatformIDs(1, &platform, nullptr);
    check_cl_error(err, "Get platform");
    
    err = clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 1, &device, nullptr);
    if (err != CL_SUCCESS) {
        std::cerr << "No GPU found, trying CPU...\n";
        err = clGetDeviceIDs(platform, CL_DEVICE_TYPE_CPU, 1, &device, nullptr);
        check_cl_error(err, "Get device");
    }
    
    // Print device info
    char device_name[256];
    clGetDeviceInfo(device, CL_DEVICE_NAME, sizeof(device_name), device_name, nullptr);
    std::cerr << "Using device: " << device_name << "\n";
    
    cl_context context = clCreateContext(nullptr, 1, &device, nullptr, nullptr, &err);
    check_cl_error(err, "Create context");
    
    cl_command_queue queue = clCreateCommandQueue(context, device, 0, &err);
    check_cl_error(err, "Create queue");
    
    // Build program
    cl_program program = clCreateProgramWithSource(context, 1, &kernel_source, nullptr, &err);
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
    
    // Generate masks
    u64 max_mask = (1ULL << opt.n);
    u64 scan_limit = (opt.max_masks == 0) ? max_mask : std::min(opt.max_masks, max_mask);
    
    std::vector<u64> masks;
    for (u64 m = 1; m < scan_limit; ++m) {
        if (popcount_u64(m) >= opt.min_taps) masks.push_back(m);
    }
    
    std::cerr << "Processing " << masks.size() << " masks...\n";
    
    // Generate targets
    std::vector<float> targets(opt.resolution);
    for (int i = 0; i < opt.resolution; ++i) {
        targets[i] = (float)i / (float)(opt.resolution - 1);
    }
    
    // Allocate device buffers
    int num_masks = masks.size();
    int num_targets = opt.resolution;
    int total_work = num_masks * num_targets;
    
    cl_mem d_masks = clCreateBuffer(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
                                    sizeof(u64) * num_masks, masks.data(), &err);
    check_cl_error(err, "Create masks buffer");
    
    cl_mem d_targets = clCreateBuffer(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
                                      sizeof(float) * num_targets, targets.data(), &err);
    check_cl_error(err, "Create targets buffer");
    
    std::vector<float> scores(total_work, 1e9f);
    std::vector<u64> best_masks_out(total_work, 0);
    std::vector<u64> best_seeds_out(total_work, 0);
    std::vector<int> best_periods_out(total_work, 0);
    
    cl_mem d_scores = clCreateBuffer(context, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR,
                                    sizeof(float) * total_work, scores.data(), &err);
    check_cl_error(err, "Create scores buffer");
    
    cl_mem d_best_masks = clCreateBuffer(context, CL_MEM_WRITE_ONLY,
                                        sizeof(u64) * total_work, nullptr, &err);
    check_cl_error(err, "Create best_masks buffer");
    
    cl_mem d_best_seeds = clCreateBuffer(context, CL_MEM_WRITE_ONLY,
                                        sizeof(u64) * total_work, nullptr, &err);
    check_cl_error(err, "Create best_seeds buffer");
    
    cl_mem d_best_periods = clCreateBuffer(context, CL_MEM_WRITE_ONLY,
                                          sizeof(int) * total_work, nullptr, &err);
    check_cl_error(err, "Create best_periods buffer");
    
    // Set kernel arguments
    clSetKernelArg(kernel, 0, sizeof(cl_mem), &d_masks);
    clSetKernelArg(kernel, 1, sizeof(cl_mem), &d_targets);
    clSetKernelArg(kernel, 2, sizeof(cl_mem), &d_targets);
    clSetKernelArg(kernel, 3, sizeof(cl_mem), &d_scores);
    clSetKernelArg(kernel, 4, sizeof(cl_mem), &d_best_masks);
    clSetKernelArg(kernel, 5, sizeof(cl_mem), &d_best_seeds);
    clSetKernelArg(kernel, 6, sizeof(cl_mem), &d_best_periods);
    clSetKernelArg(kernel, 7, sizeof(int), &opt.n);
    clSetKernelArg(kernel, 8, sizeof(int), &num_masks);
    clSetKernelArg(kernel, 9, sizeof(int), &num_targets);
    clSetKernelArg(kernel, 10, sizeof(int), &opt.block_size);
    float max_err_f = (float)opt.max_block_error;
    clSetKernelArg(kernel, 11, sizeof(float), &max_err_f);
    
    // Execute kernel
    size_t global_work_size = ((total_work + opt.work_group_size - 1) / opt.work_group_size) * opt.work_group_size;
    size_t local_work_size = opt.work_group_size;
    
    std::cerr << "Launching kernel with " << global_work_size << " work items...\n";
    
    auto start = std::chrono::high_resolution_clock::now();
    err = clEnqueueNDRangeKernel(queue, kernel, 1, nullptr, &global_work_size, &local_work_size, 0, nullptr, nullptr);
    check_cl_error(err, "Enqueue kernel");
    
    clFinish(queue);
    auto end = std::chrono::high_resolution_clock::now();
    
    std::chrono::duration<double> elapsed = end - start;
    std::cerr << "Kernel execution time: " << elapsed.count() << " seconds\n";
    
    // Read results
    clEnqueueReadBuffer(queue, d_scores, CL_TRUE, 0, sizeof(float) * total_work, scores.data(), 0, nullptr, nullptr);
    clEnqueueReadBuffer(queue, d_best_masks, CL_TRUE, 0, sizeof(u64) * total_work, best_masks_out.data(), 0, nullptr, nullptr);
    clEnqueueReadBuffer(queue, d_best_seeds, CL_TRUE, 0, sizeof(u64) * total_work, best_seeds_out.data(), 0, nullptr, nullptr);
    clEnqueueReadBuffer(queue, d_best_periods, CL_TRUE, 0, sizeof(int) * total_work, best_periods_out.data(), 0, nullptr, nullptr);
    
    // Find best config per target
    std::vector<LFSRConfig> best_configs(num_targets);
    for (int t = 0; t < num_targets; ++t) {
        float best_score = 1e9f;
        int best_idx = -1;
        
        for (int m = 0; m < num_masks; ++m) {
            int idx = m * num_targets + t;
            if (scores[idx] < best_score) {
                best_score = scores[idx];
                best_idx = idx;
            }
        }
        
        if (best_idx >= 0) {
            best_configs[t].mask = best_masks_out[best_idx];
            best_configs[t].seed = best_seeds_out[best_idx];
            best_configs[t].period = best_periods_out[best_idx];
            best_configs[t].error = scores[best_idx];
            best_configs[t].score = scores[best_idx];
            best_configs[t].actual_duty = targets[t];
        }
    }
    
    // Cleanup
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
    
    // Write output
    write_arduino_lut(best_configs, opt.n, opt.output);
    std::cerr << "Written Arduino LUT to " << opt.output << "\n";
    
    return 0;
}
