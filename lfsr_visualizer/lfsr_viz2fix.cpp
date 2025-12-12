// lfsr_viz_signed.cpp - FIXED VERSION
//
// Linux SDL2/OpenGL LFSR Polynomial Space Visualizer (Signed Integer Storage)
// 
// Compile: 
// g++ -O3 -std=c++17 -march=native -pthread -o lfsr_viz_signed lfsr_viz_signed.cpp -lSDL2 -lGL
//
// FIXES:
// 1. Proper progress tracking to avoid infinite wait
// 2. Simplified merge detection logic
// 3. Added safety checks for cycle detection
// 4. Better memory initialization

#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>
#include <GL/gl.h>

#include <iostream>
#include <vector>
#include <algorithm>
#include <thread>
#include <atomic>
#include <mutex>
#include <fstream>
#include <cmath>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <immintrin.h>

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------
const char* DATA_FILE = "lfsr_map_signed.bin";
const int N_BITS = 16;
const size_t DIM_SIZE = 1 << N_BITS; // 65536
const size_t TOTAL_POINTS = DIM_SIZE * DIM_SIZE;
const size_t FILE_SIZE = TOTAL_POINTS * sizeof(int16_t);

const int WINDOW_WIDTH = 1600;
const int WINDOW_HEIGHT = 900;
const float MAX_DUTY_SCALER = 32767.0f;

// -----------------------------------------------------------------------------
// LFSR Logic
// -----------------------------------------------------------------------------
static inline uint16_t next_state(uint16_t state, uint16_t mask) {
    uint16_t feedback = __builtin_parity(state & mask);
    return (state >> 1) | (feedback << 15);
}

// -----------------------------------------------------------------------------
// Data Generation (FIXED)
// -----------------------------------------------------------------------------
void generate_dataset_worker(uint16_t start_mask, uint16_t end_mask, int16_t* buffer, 
                            std::atomic<int>& progress, std::atomic<bool>& error_flag) {
    std::vector<uint8_t> visited(DIM_SIZE, 0);
    std::vector<uint16_t> chain;
    chain.reserve(DIM_SIZE);
    
    std::vector<uint16_t> cycle_sorted;
    cycle_sorted.reserve(DIM_SIZE);

    for (uint16_t mask = start_mask; mask < end_mask; ++mask) {
        // Reset visited for this mask
        memset(visited.data(), 0, DIM_SIZE);
        
        uint64_t row_offset = (uint64_t)mask * DIM_SIZE;

        for (uint16_t seed = 0; seed < DIM_SIZE; ++seed) {
            if (visited[seed]) continue;

            uint16_t curr = seed;
            chain.clear();
            
            // Trace chain with safety limit
            const size_t MAX_CHAIN = DIM_SIZE + 10; // Safety margin
            while (!visited[curr] && chain.size() < MAX_CHAIN) {
                visited[curr] = 1;
                chain.push_back(curr);
                curr = next_state(curr, mask);
            }

            if (chain.size() >= MAX_CHAIN) {
                std::cerr << "\n[ERROR] Infinite loop detected at mask=" << mask << " seed=" << seed << "\n";
                error_flag.store(true);
                return;
            }

            // Find where we hit the cycle
            auto cycle_it = std::find(chain.begin(), chain.end(), curr);
            
            size_t cycle_start_idx;
            size_t cycle_len;
            
            if (cycle_it != chain.end()) {
                // We found a cycle within this chain
                cycle_start_idx = std::distance(chain.begin(), cycle_it);
                cycle_len = chain.size() - cycle_start_idx;
            } else {
                // We hit a previously processed state
                // Treat entire chain as transient leading to existing cycle
                cycle_start_idx = chain.size(); // No cycle in this chain
                cycle_len = 0;
            }

            // Process cycle states (if any)
            if (cycle_len > 0) {
                cycle_sorted.clear();
                for (size_t i = cycle_start_idx; i < chain.size(); ++i) {
                    cycle_sorted.push_back(chain[i]);
                }
                std::sort(cycle_sorted.begin(), cycle_sorted.end());

                // Mark cycle states with positive duty
                for (size_t i = 0; i < cycle_sorted.size(); ++i) {
                    uint16_t st = cycle_sorted[i];
                    double duty = (double)(cycle_len - 1 - i) / (double)cycle_len;
                    int16_t duty_val = (int16_t)(duty * MAX_DUTY_SCALER);
                    buffer[row_offset + st] = duty_val;
                }
            }

            // Process transient states
            for (size_t i = 0; i < cycle_start_idx; ++i) {
                uint16_t trans_st = chain[i];
                
                double duty;
                if (cycle_len > 0) {
                    // Calculate duty based on cycle
                    auto it_gt = std::upper_bound(cycle_sorted.begin(), cycle_sorted.end(), trans_st);
                    int count_gt = std::distance(it_gt, cycle_sorted.end());
                    duty = (double)count_gt / (double)cycle_len;
                } else {
                    // Merges into existing cycle - read target duty
                    int16_t target_val = buffer[row_offset + curr];
                    duty = std::abs(target_val) / MAX_DUTY_SCALER;
                }
                
                int16_t duty_val = (int16_t)(duty * MAX_DUTY_SCALER);
                // Store as negative for transient
                buffer[row_offset + trans_st] = (duty_val == 0) ? 0 : -duty_val;
            }
        }
        
        // Update progress atomically
        progress.fetch_add(1, std::memory_order_relaxed);
    }
}

void ensure_dataset_exists() {
    int fd = open(DATA_FILE, O_RDWR | O_CREAT, 0644);
    if (fd == -1) { perror("Error opening file"); exit(1); }

    struct stat st;
    fstat(fd, &st);
    
    if (st.st_size == FILE_SIZE) {
        std::cout << "[INFO] Dataset " << DATA_FILE << " found. Ready.\n";
        close(fd);
        return;
    }

    std::cout << "[INFO] Generating 8GB Signed Dataset... Please wait.\n";
    if (ftruncate(fd, FILE_SIZE) == -1) { perror("ftruncate"); exit(1); }

    int16_t* map = (int16_t*)mmap(nullptr, FILE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (map == MAP_FAILED) { perror("mmap"); exit(1); }

    // Initialize to zero
    std::cout << "[INFO] Initializing memory...\n";
    memset(map, 0, FILE_SIZE);

    int num_threads = std::thread::hardware_concurrency();
    if (num_threads == 0) num_threads = 4;
    
    std::vector<std::thread> threads;
    std::atomic<int> progress(0);
    std::atomic<bool> error_flag(false);
    
    int chunk = DIM_SIZE / num_threads;

    std::cout << "[INFO] Using " << num_threads << " threads\n";

    for (int t = 0; t < num_threads; ++t) {
        uint16_t start = t * chunk;
        uint16_t end = (t == num_threads - 1) ? DIM_SIZE : (t + 1) * chunk;
        threads.emplace_back(generate_dataset_worker, start, end, map, 
                           std::ref(progress), std::ref(error_flag));
    }

    // Progress monitoring with timeout detection
    int last_progress = 0;
    int stall_count = 0;
    const int MAX_STALLS = 30; // 30 seconds without progress = error
    
    while (progress.load(std::memory_order_relaxed) < (int)DIM_SIZE) {
        if (error_flag.load(std::memory_order_relaxed)) {
            std::cerr << "[ERROR] Worker thread encountered error. Aborting.\n";
            for (auto& th : threads) {
                if (th.joinable()) th.join();
            }
            munmap(map, FILE_SIZE);
            close(fd);
            exit(1);
        }
        
        int p = progress.load(std::memory_order_relaxed);
        
        if (p == last_progress) {
            stall_count++;
            if (stall_count >= MAX_STALLS) {
                std::cerr << "\n[ERROR] Generation stalled at " << p << "/" << DIM_SIZE << "\n";
                error_flag.store(true);
                break;
            }
        } else {
            stall_count = 0;
            last_progress = p;
        }
        
        if (p % 100 == 0 || p == last_progress + 10) {
            printf("\r[Generating] %d / %d Masks (%.1f%%)    ", 
                   p, (int)DIM_SIZE, (p * 100.0f) / DIM_SIZE);
            fflush(stdout);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    
    for (auto& th : threads) {
        if (th.joinable()) th.join();
    }
    
    if (error_flag.load()) {
        std::cout << "\n[FAILED] Generation failed.\n";
        munmap(map, FILE_SIZE);
        close(fd);
        exit(1);
    }
    
    std::cout << "\n[DONE] Generation complete.\n";

    munmap(map, FILE_SIZE);
    close(fd);
}

// -----------------------------------------------------------------------------
// Visualization (unchanged except minor cleanup)
// -----------------------------------------------------------------------------

struct Camera {
    float x = 32768.0f; 
    float y = 32768.0f;
    float dist = 70000.0f;
    float pitch = 45.0f;
    float yaw = 0.0f;
};

void draw_text(float x, float y, const std::string& text) {
    glMatrixMode(GL_PROJECTION);
    glPushMatrix(); glLoadIdentity();
    glOrtho(0, WINDOW_WIDTH, WINDOW_HEIGHT, 0, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix(); glLoadIdentity();
    glLineWidth(2.0f);
    glColor3f(1.0f, 1.0f, 1.0f);
    float cx = x; float scale = 10.0f;
    for(char c : text) {
        glBegin(GL_LINES);
        float w = 0.6f * scale, h = 1.0f * scale;
        if (c == ' ') { cx += w + 4; glEnd(); continue; }
        bool t=0,m=0,b=0,tl=0,tr=0,bl=0,br=0;
        if(isdigit(c)) {
            int d = c-'0';
            if(d!=1 && d!=4) t=1; if(d!=0 && d!=1 && d!=7) m=1; if(d!=1 && d!=4 && d!=7) b=1;
            if(d!=1 && d!=2 && d!=3 && d!=7) tl=1; if(d!=5 && d!=6) tr=1;
            if(d==0 || d==2 || d==6 || d==8) bl=1; if(d!=2) br=1;
        } else if(c=='-') m=1;
        else if(c=='S') {t=1;m=1;b=1;tl=1;br=1;}
        else if(c=='T') {t=1; glVertex2f(cx+w/2,y); glVertex2f(cx+w/2,y+h); }
        else { t=b=tl=tr=bl=br=1; }
        
        if(t) {glVertex2f(cx,y); glVertex2f(cx+w,y);}
        if(m) {glVertex2f(cx,y+h/2); glVertex2f(cx+w,y+h/2);}
        if(b) {glVertex2f(cx,y+h); glVertex2f(cx+w,y+h);}
        if(tl){glVertex2f(cx,y); glVertex2f(cx,y+h/2);}
        if(bl){glVertex2f(cx,y+h/2); glVertex2f(cx,y+h);}
        if(tr){glVertex2f(cx+w,y); glVertex2f(cx+w,y+h/2);}
        if(br){glVertex2f(cx+w,y+h/2); glVertex2f(cx+w,y+h);}
        glEnd();
        cx += w + 6.0f;
    }
    glPopMatrix(); glMatrixMode(GL_PROJECTION); glPopMatrix(); glMatrixMode(GL_MODELVIEW);
}

int main(int argc, char** argv) {
    ensure_dataset_exists();

    int fd = open(DATA_FILE, O_RDONLY);
    if (fd == -1) { perror("Error opening data file"); return 1; }
    
    int16_t* data = (int16_t*)mmap(nullptr, FILE_SIZE, PROT_READ, MAP_SHARED, fd, 0);
    if (data == MAP_FAILED) { perror("mmap failed"); return 1; }

    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cerr << "SDL_Init failed: " << SDL_GetError() << "\n";
        return 1;
    }
    
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);

    SDL_Window* window = SDL_CreateWindow("LFSR Map (Signed Transients)", 
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WINDOW_WIDTH, WINDOW_HEIGHT, 
        SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN);
    
    if (!window) {
        std::cerr << "Window creation failed: " << SDL_GetError() << "\n";
        return 1;
    }
    
    SDL_GLContext context = SDL_GL_CreateContext(window);
    if (!context) {
        std::cerr << "GL context creation failed: " << SDL_GetError() << "\n";
        return 1;
    }

    glEnable(GL_DEPTH_TEST);

    Camera cam;
    bool running = true;
    float vel_yaw=0, vel_pitch=0, vel_x=0, vel_y=0;
    bool drag_rot=false, drag_pan=false;
    int last_mx=0, last_my=0;

    struct Vertex { float x, y, z; uint8_t r, g, b, a; };
    std::vector<Vertex> vbuffer;
    vbuffer.reserve(1000000);

    while (running) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) running = false;
            else if (e.type == SDL_MOUSEBUTTONDOWN) {
                if (e.button.button == SDL_BUTTON_LEFT) drag_rot = true;
                if (e.button.button == SDL_BUTTON_RIGHT) drag_pan = true;
            }
            else if (e.type == SDL_MOUSEBUTTONUP) { drag_rot = false; drag_pan = false; }
            else if (e.type == SDL_MOUSEMOTION) {
                int dx = e.motion.x - last_mx; int dy = e.motion.y - last_my;
                last_mx = e.motion.x; last_my = e.motion.y;
                if (drag_rot) { vel_yaw += dx*0.5f; vel_pitch += dy*0.5f; }
                if (drag_pan) {
                    float rad = cam.yaw * 3.14159f / 180.0f;
                    vel_x -= (cos(rad)*dx - sin(rad)*dy) * (cam.dist * 0.001f);
                    vel_y -= (sin(rad)*dx + cos(rad)*dy) * (cam.dist * 0.001f);
                }
            }
            else if (e.type == SDL_MOUSEWHEEL) {
                cam.dist -= e.wheel.y * (cam.dist * 0.1f);
                if (cam.dist < 100) cam.dist = 100;
            }
        }

        cam.yaw += vel_yaw * 0.1f; cam.pitch += vel_pitch * 0.1f;
        cam.x += vel_x * 0.1f; cam.y += vel_y * 0.1f;
        vel_yaw *= 0.9f; vel_pitch *= 0.9f; vel_x *= 0.9f; vel_y *= 0.9f;
        if (cam.pitch > 89) cam.pitch = 89; if (cam.pitch < -89) cam.pitch = -89;

        glClearColor(0.1f, 0.1f, 0.12f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_PROJECTION); glLoadIdentity();
        double ar = (double)WINDOW_WIDTH/WINDOW_HEIGHT;
        double zn=10.0, zf=500000.0, fov=60.0;
        double fh = tan(fov/360*3.14159)*zn;
        glFrustum(-fh*ar, fh*ar, -fh, fh, zn, zf);

        glMatrixMode(GL_MODELVIEW); glLoadIdentity();
        glTranslatef(0,0,-cam.dist);
        glRotatef(cam.pitch, 1,0,0);
        glRotatef(cam.yaw, 0,1,0);
        glTranslatef(-cam.x, 0, -cam.y);

        float speed = sqrt(vel_yaw*vel_yaw + vel_pitch*vel_pitch + vel_x*vel_x + vel_y*vel_y);
        int step = (speed > 1.0f) ? 128 : (speed > 0.1f ? 32 : (cam.dist > 30000 ? 64 : (cam.dist > 10000 ? 16 : (cam.dist > 2000 ? 4 : 1))));

        float range = cam.dist * 1.5f;
        int min_x = std::max(0, (int)(cam.x - range)); 
        int max_x = std::min((int)DIM_SIZE, (int)(cam.x + range));
        int min_y = std::max(0, (int)(cam.y - range)); 
        int max_y = std::min((int)DIM_SIZE, (int)(cam.y + range));

        vbuffer.clear();
        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_COLOR_ARRAY);

        for (int mx = min_x; mx < max_x; mx += step) {
            for (int my = min_y; my < max_y; my += step) {
                int16_t val = data[mx * DIM_SIZE + my];
                
                bool is_transient = (val < 0);
                float abs_norm = std::abs(val) / MAX_DUTY_SCALER;
                float z = abs_norm * 10000.0f;

                uint8_t r, g, b;

                uint16_t ns = next_state((uint16_t)my, (uint16_t)mx);
                bool is_lockup = (ns == (uint16_t)my);

                if (is_lockup) {
                    r = 0; g = 255; b = 255;
                    vbuffer.push_back({(float)mx, z, (float)my, r, g, b, 255});
                    if (step > 1) {
                         vbuffer.push_back({(float)mx, z+50, (float)my, r, g, b, 255});
                         vbuffer.push_back({(float)mx, z+100, (float)my, r, g, b, 255});
                    }
                } 
                else if (is_transient) {
                    r = (uint8_t)(50 + abs_norm * 100);
                    g = (uint8_t)(50 + abs_norm * 100);
                    b = (uint8_t)(70 + abs_norm * 140);
                    vbuffer.push_back({(float)mx, z, (float)my, r, g, b, 255});
                } 
                else {
                    r = (uint8_t)(abs_norm * 255);
                    g = (uint8_t)(sin(abs_norm * 3.14159) * 255);
                    b = (uint8_t)((1.0f - abs_norm) * 50);
                    vbuffer.push_back({(float)mx, z, (float)my, r, g, b, 255});
                }

                if (vbuffer.size() >= 999000) {
                    glVertexPointer(3, GL_FLOAT, sizeof(Vertex), &vbuffer[0].x);
                    glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(Vertex), &vbuffer[0].r);
                    glDrawArrays(GL_POINTS, 0, vbuffer.size());
                    vbuffer.clear();
                }
            }
        }
        if (!vbuffer.empty()) {
            glVertexPointer(3, GL_FLOAT, sizeof(Vertex), &vbuffer[0].x);
            glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(Vertex), &vbuffer[0].r);
            glDrawArrays(GL_POINTS, 0, vbuffer.size());
        }

        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_COLOR_ARRAY);

        int hover_mask = std::clamp((int)cam.x, 0, 65535);
        int hover_seed = std::clamp((int)cam.y, 0, 65535);
        int16_t h_val = data[hover_mask * DIM_SIZE + hover_seed];
        float h_duty = std::abs(h_val) / MAX_DUTY_SCALER;
        
        std::string s_type = (h_val < 0) ? "TRANSIENT" : "STABLE";
        if (next_state(hover_seed, hover_mask) == hover_seed) s_type = "LOCKUP";

        draw_text(20, WINDOW_HEIGHT-80, "MASK: " + std::to_string(hover_mask) + " SEED: " + std::to_string(hover_seed));
        draw_text(20, WINDOW_HEIGHT-50, "TYPE: " + s_type + "  DUTY: " + std::to_string((int)(h_duty*100)) + " " + std::to_string(h_duty));

        SDL_GL_SwapWindow(window);
    }

    munmap(data, FILE_SIZE);
    close(fd);
    SDL_GL_DeleteContext(context);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
