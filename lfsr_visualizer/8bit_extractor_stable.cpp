// Compile:
// g++ -O3 -std=c++17 -march=native -pthread -o lfsr_analyzer lfsr_analyzer.cpp

#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdio>
#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <chrono>
#include <cstdint> // for int16_t

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------
constexpr const char* INPUT_FILE = "lfsr_map_signed.bin";
constexpr const char* OUT_DIR    = "8bit_set";
constexpr uint32_t DIM_SIZE      = 65536;
constexpr uint64_t FILE_SIZE =
    uint64_t(DIM_SIZE) * uint64_t(DIM_SIZE) * sizeof(int16_t);

// Buffer size per PWM bin
constexpr size_t WRITE_BUFFER_SIZE = 8192;

// -----------------------------------------------------------------------------
// Structures
// -----------------------------------------------------------------------------
#pragma pack(push, 1)
struct Entry {
    uint16_t mask;
    uint16_t seed;
};
#pragma pack(pop)

// Globals
int out_fds[256];
std::mutex file_mutexes[256];

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------
static ssize_t full_write(int fd, const void* buf, size_t count) {
    const char* p = static_cast<const char*>(buf);
    while (count) {
        ssize_t w = write(fd, p, count);
        if (w < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        p += w;
        count -= w;
    }
    return 0;
}

void ensure_directory() {
    struct stat st{};
    if (stat(OUT_DIR, &st) == -1)
        mkdir(OUT_DIR, 0755);
}

void prepare_output_files() {
    ensure_directory();
    char path[128];
    for (int i = 0; i < 256; ++i) {
        snprintf(path, sizeof(path), "%s/0x%02X.bin", OUT_DIR, i);
        out_fds[i] = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
        if (out_fds[i] < 0) {
            perror("open");
            exit(1);
        }
    }
}

// -----------------------------------------------------------------------------
// Worker
// -----------------------------------------------------------------------------

// --- inside top-of-file includes add:
#include <cstdint> // for int16_t

// --- Worker (replace previous worker implementation) ---
void worker(uint32_t start_mask, uint32_t end_mask, const int16_t* input_map, std::atomic<uint64_t>& processed_count) {
    std::vector<Entry> buffers[256];
    for (int i = 0; i < 256; ++i) buffers[i].reserve(WRITE_BUFFER_SIZE + 100);

    uint64_t local_count = 0;

    for (uint32_t m = start_mask; m < end_mask; ++m) {
        uint64_t row_offset = (uint64_t)m * (uint64_t)DIM_SIZE;

        for (uint32_t s = 0; s < DIM_SIZE; ++s) {
            // Read signed 16-bit value
            int16_t val = input_map[row_offset + s];

            // Skip non-positive values (unstable LFSRs)
            if (val <= 0) {
                continue;
            }

            // Quantize to 8-bit using same mapping used earlier (signed -> 0..255)
            // normalized = (val + 32768) / 65535    (val is positive here so result > 0.5 typically)
//            uint8_t pwm8 = static_cast<uint8_t>((static_cast<uint32_t>(static_cast<int32_t>(val) + 32768) * 255u) / 65535u);
            uint8_t pwm8 = static_cast<uint8_t>((static_cast<uint32_t>(static_cast<int32_t>(val)) * 255u) / 32767u);

            buffers[pwm8].push_back({static_cast<uint16_t>(m), static_cast<uint16_t>(s)});

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

    // flush remaining
    for (int i = 0; i < 256; ++i) {
        if (!buffers[i].empty()) {
            std::lock_guard<std::mutex> lock(file_mutexes[i]);
            size_t bytes = buffers[i].size() * sizeof(Entry);
            if (full_write(out_fds[i], buffers[i].data(), bytes) == -1) {
                perror("write");
                std::exit(1);
            }
            buffers[i].clear();
        }
    }

    if (local_count) processed_count.fetch_add(local_count, std::memory_order_relaxed);
}

// -----------------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------------
int main() {
    int fd = open(INPUT_FILE, O_RDONLY);
    if (fd < 0) {
        perror("open input");
        return 1;
    }

    struct stat st;
    fstat(fd, &st);
    if ((uint64_t)st.st_size != FILE_SIZE) {
        std::cerr << "File size mismatch\n";
        return 1;
    }

    const int16_t* data = static_cast<const int16_t*>(
        mmap(nullptr, FILE_SIZE, PROT_READ, MAP_SHARED, fd, 0));

    if (data == MAP_FAILED) {
        perror("mmap");
        return 1;
    }

    prepare_output_files();

    unsigned nt = std::thread::hardware_concurrency();
    if (!nt) nt = 4;

    std::atomic<uint64_t> progress{0};
    std::vector<std::thread> threads;

    uint32_t chunk = (DIM_SIZE + nt - 1) / nt;

    auto t0 = std::chrono::high_resolution_clock::now();

    for (unsigned i = 0; i < nt; ++i) {
        uint32_t s = i * chunk;
        uint32_t e = std::min(s + chunk, DIM_SIZE);
        if (s < e)
            threads.emplace_back(worker, s, e, data, std::ref(progress));
    }

    const uint64_t total = uint64_t(DIM_SIZE) * DIM_SIZE;

    while (progress.load() < total) {
        double pct = 100.0 * progress.load() / double(total);
        printf("\r[Analyzing] %.2f%%", pct);
        fflush(stdout);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    for (auto& t : threads) t.join();

    auto t1 = std::chrono::high_resolution_clock::now();
    printf("\n[DONE] %.2fs\n",
           std::chrono::duration<double>(t1 - t0).count());

    munmap((void*)data, FILE_SIZE);
    close(fd);

    for (int i = 0; i < 256; ++i)
        close(out_fds[i]);

    return 0;
}
