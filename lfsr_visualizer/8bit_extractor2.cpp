// Put this entire file over your current one.

#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <fstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <algorithm>
#include <cerrno>
#include <cinttypes>

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------
const char* INPUT_FILE = "lfsr_map.bin";
const char* OUT_DIR = "8bit_set";
const size_t DIM_SIZE = 65536;
const size_t FILE_SIZE = DIM_SIZE * DIM_SIZE * sizeof(uint16_t); // 8GB

// Buffer size (in number of elements) before flushing to disk
const size_t WRITE_BUFFER_SIZE = 8192; 

// -----------------------------------------------------------------------------
// Structures
// -----------------------------------------------------------------------------
#pragma pack(push, 1)
struct Entry {
    uint16_t mask;
    uint16_t seed;
};
#pragma pack(pop)

// Global file descriptors for the 256 bins
int out_fds[256];
std::mutex file_mutexes[256];

// -----------------------------------------------------------------------------
// Helper Functions
// -----------------------------------------------------------------------------

void ensure_directory() {
    struct stat st = {0};
    if (stat(OUT_DIR, &st) == -1) {
        #ifdef _WIN32
        _mkdir(OUT_DIR);
        #else
        mkdir(OUT_DIR, 0755);
        #endif
    }
}

// Safe write that handles EINTR and short writes
static ssize_t full_write(int fd, const void* buf, size_t count) {
    const char* ptr = reinterpret_cast<const char*>(buf);
    size_t remaining = count;

    while (remaining > 0) {
        ssize_t w = ::write(fd, ptr, remaining);
        if (w == -1) {
            if (errno == EINTR) continue;
            return -1;
        }
        ptr += w;
        remaining -= static_cast<size_t>(w);
    }
    return static_cast<ssize_t>(count);
}

// Open all 256 bin files (truncate mode)
void prepare_output_files() {
    ensure_directory();
    char filepath[128];
    
    std::cout << "[INFO] creating/truncating output files in " << OUT_DIR << "/...\n";
    
    for (int i = 0; i < 256; ++i) {
        snprintf(filepath, sizeof(filepath), "%s/0x%02X.bin", OUT_DIR, i);
        out_fds[i] = open(filepath, O_WRONLY | O_CREAT | O_TRUNC, 0644);
        if (out_fds[i] == -1) {
            perror("Failed to create output bin");
            exit(1);
        }
    }
}

void close_output_files() {
    for (int i = 0; i < 256; ++i) {
        if (out_fds[i] != -1) close(out_fds[i]);
    }
}

// -----------------------------------------------------------------------------
// Worker Logic
// -----------------------------------------------------------------------------

void worker(uint16_t start_mask, uint16_t end_mask, const uint16_t* input_map, std::atomic<size_t>& processed_count) {
    std::vector<Entry> buffers[256];
    for(int i=0; i<256; ++i) buffers[i].reserve(WRITE_BUFFER_SIZE + 100);

    size_t local_count = 0;

    for (int m = start_mask; m < end_mask; ++m) {
        uint64_t row_offset = (uint64_t)m * DIM_SIZE;
        
        for (int s = 0; s < (int)DIM_SIZE; ++s) {
            uint16_t duty16 = input_map[row_offset + s];
            uint8_t pwm8 = (uint8_t)((duty16 * 255u) / 65535u);

            buffers[pwm8].push_back({(uint16_t)m, (uint16_t)s});

            if (buffers[pwm8].size() >= WRITE_BUFFER_SIZE) {
                std::lock_guard<std::mutex> lock(file_mutexes[pwm8]);
                size_t bytes = buffers[pwm8].size() * sizeof(Entry);
                if (full_write(out_fds[pwm8], buffers[pwm8].data(), bytes) == -1) {
                    perror("write");
                    std::exit(1);
                }
                buffers[pwm8].clear();
            }
        }
        
        local_count += DIM_SIZE;
        if (local_count >= DIM_SIZE * 16) {
            processed_count.fetch_add(local_count, std::memory_order_relaxed);
            local_count = 0;
        }
    }
    
    // Flush remaining data in all buffers
    for (int i = 0; i < 256; ++i) {
        if (!buffers[i].empty()) {
            std::lock_guard<std::mutex> lock(file_mutexes[i]);
            size_t bytes = buffers[i].size() * sizeof(Entry);
            if (full_write(out_fds[i], buffers[i].data(), bytes) == -1) {
                perror("write");
                std::exit(1);
            }
        }
    }
    if (local_count) processed_count.fetch_add(local_count, std::memory_order_relaxed);
}

// -----------------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------------

int main() {
    int fd = open(INPUT_FILE, O_RDONLY);
    if (fd == -1) {
        std::cerr << "[ERROR] Could not open " << INPUT_FILE << ". Run lfsr_viz first.\n";
        return 1;
    }

    struct stat st;
    fstat(fd, &st);
    if ((size_t)st.st_size != FILE_SIZE) {
        std::cerr << "[ERROR] File size mismatch. Expected 8GB.\n";
        close(fd);
        return 1;
    }

    std::cout << "[INFO] Mapping dataset into memory...\n";
    const uint16_t* data = (const uint16_t*)mmap(nullptr, FILE_SIZE, PROT_READ, MAP_SHARED, fd, 0);
    if (data == MAP_FAILED) {
        perror("mmap");
        return 1;
    }

    prepare_output_files();

    int num_threads = std::thread::hardware_concurrency();
    if (num_threads <= 0) num_threads = 4;
    std::cout << "[INFO] processing with " << num_threads << " threads...\n";

    std::vector<std::thread> threads;
    std::atomic<size_t> processed_count(0);
    
    int chunk_size = DIM_SIZE / num_threads;
    auto start_time = std::chrono::high_resolution_clock::now();

    for (int t = 0; t < num_threads; ++t) {
        uint16_t start = t * chunk_size;
        uint16_t end = (t == num_threads - 1) ? DIM_SIZE : (t + 1) * chunk_size;
        threads.emplace_back(worker, start, end, data, std::ref(processed_count));
    }

    size_t total_points = (size_t)DIM_SIZE * DIM_SIZE;
    while (processed_count.load(std::memory_order_relaxed) < total_points) {
        size_t p = processed_count.load(std::memory_order_relaxed);
        double pct = (double)p / total_points * 100.0;
        printf("\r[Analyzing] %.2f%% Complete (%zu / %zu)", pct, p, total_points);
        fflush(stdout);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    for (auto& th : threads) th.join();

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff = end_time - start_time;

    printf("\r[DONE] 100.00%% Complete. Time: %.2fs\n", diff.count());

    close_output_files();
    munmap((void*)data, FILE_SIZE);
    close(fd);

    std::cout << "[INFO] Analysis complete. Data stored in '" << OUT_DIR << "/'.\n";

    return 0;
}
