// lfsr_viz.cpp
//
// Linux SDL2/OpenGL LFSR Polynomial Space Visualizer
// 
// compile: g++ -O3 -std=c++17 -march=native -pthread -o lfsr_viz lfsr_viz.cpp -lSDL2 -lGL
//
// This program maps the 16-bit LFSR space (Mask vs Seed).
// X-Axis: Mask (0-65535)
// Y-Axis: Seed/Threshold (0-65535)
// Z-Axis: Duty Cycle (0-65535 mapped from 0.0-1.0)
//
// Blue Cubes represent "Lock-up" states where NextState == CurrentState.

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
// Configuration & Constants
// -----------------------------------------------------------------------------
const char* DATA_FILE = "lfsr_map.bin";
const int N_BITS = 16;
const size_t DIM_SIZE = 1 << N_BITS; // 65536
const size_t TOTAL_POINTS = DIM_SIZE * DIM_SIZE;
const size_t FILE_SIZE = TOTAL_POINTS * sizeof(uint16_t);

// For Visualization
const int WINDOW_WIDTH = 1600;
const int WINDOW_HEIGHT = 900;

// -----------------------------------------------------------------------------
// LFSR Logic
// -----------------------------------------------------------------------------
// Standard Fibonacci LFSR step
static inline uint16_t next_state(uint16_t state, uint16_t mask) {
    uint16_t feedback = __builtin_parity(state & mask);
    return (state >> 1) | (feedback << 15);
}

// -----------------------------------------------------------------------------
// Data Generation
// -----------------------------------------------------------------------------
// We need to generate a 65536x65536 grid of Duty Cycles (uint16_t).
// Value 0xFFFF = 100% duty, 0x0000 = 0% duty.

void generate_dataset_worker(uint16_t start_mask, uint16_t end_mask, uint16_t* buffer, std::atomic<int>& progress) {
    std::vector<uint8_t> visited(DIM_SIZE, 0);
    std::vector<uint16_t> chain;
    chain.reserve(DIM_SIZE);
    
    // Scratchpad for sorting cycles to determine duty rank
    std::vector<uint16_t> cycle_sorted;
    cycle_sorted.reserve(DIM_SIZE);

    for (int m = start_mask; m < end_mask; ++m) {
        uint16_t mask = (uint16_t)m;
        std::fill(visited.begin(), visited.end(), 0);
        
        // Offset in the big buffer for this mask row
        // Map is [Mask][Seed]
        uint64_t row_offset = (uint64_t)mask * DIM_SIZE;

        for (int s = 0; s < DIM_SIZE; ++s) {
            if (visited[s]) continue;

            // Trace chain
            uint16_t curr = s;
            chain.clear();
            
            while (!visited[curr]) {
                visited[curr] = 1;
                chain.push_back(curr);
                curr = next_state(curr, mask);
            }

            // Detect cycle
            auto it = std::find(chain.begin(), chain.end(), curr);
            if (it == chain.end()) {
                // Should not happen in finite state machine, but safety
                continue; 
            }

            // Identify cycle part vs transient part
            size_t cycle_start_idx = std::distance(chain.begin(), it);
            size_t cycle_len = chain.size() - cycle_start_idx;
            
            // Extract cycle for ranking
            cycle_sorted.clear();
            for(size_t i=cycle_start_idx; i<chain.size(); ++i) {
                cycle_sorted.push_back(chain[i]);
            }
            std::sort(cycle_sorted.begin(), cycle_sorted.end());

            // --- Fill Duty Cycles for the Cycle States ---
            // For a state S in cycle C acting as Threshold:
            // Duty = (Count of states > S) / Length
            // In sorted list, if S is at index 'rank', then 'rank' states are <= S.
            // So (Length - 1 - rank) states are > S.
            // Duty ratio = (Length - 1 - rank) / Length.
            
            for(size_t i=0; i<cycle_sorted.size(); ++i) {
                uint16_t st = cycle_sorted[i];
                double duty = (double)(cycle_len - 1 - i) / (double)cycle_len;
                buffer[row_offset + st] = (uint16_t)(duty * 65535.0);
            }

            // --- Fill Duty Cycles for Transients (Pre-cycle) ---
            // A transient state eventually leads to this cycle. 
            // If we use a transient state as a Threshold, the "Duty" is defined by the behavior
            // once it settles into the loop? Or the whole trajectory?
            // The prompt implies "evaluation result" of the PWM.
            // Usually, PWM settles into the cycle. The transient is brief. 
            // We will assign the SAME duty as if it was in the cycle?
            // Actually, if Threshold is transient, it is never 'hit' once in the cycle.
            // If Threshold T is not in Cycle C:
            // Output High if State > T. 
            // We iterate State through C.
            // So Duty = (Count s in C where s > T) / Length.
            // We can compute this using binary search on cycle_sorted.
            
            for(size_t i=0; i<cycle_start_idx; ++i) {
                uint16_t trans_st = chain[i];
                // How many in cycle are > trans_st?
                // cycle_sorted is sorted asc.
                // upper_bound returns first element > value.
                auto it_gt = std::upper_bound(cycle_sorted.begin(), cycle_sorted.end(), trans_st);
                int count_gt = std::distance(it_gt, cycle_sorted.end());
                double duty = (double)count_gt / (double)cycle_len;
                buffer[row_offset + trans_st] = (uint16_t)(duty * 65535.0);
            }
        }
        progress++;
    }
}

void ensure_dataset_exists() {
    int fd = open(DATA_FILE, O_RDWR | O_CREAT, 0644);
    if (fd == -1) {
        perror("Error opening file");
        exit(1);
    }

    // Check size
    struct stat st;
    fstat(fd, &st);
    
    if (st.st_size == FILE_SIZE) {
        std::cout << "[INFO] Dataset found (" << (FILE_SIZE/1024/1024) << " MB). Ready.\n";
        close(fd);
        return;
    }

    std::cout << "[INFO] Generating 8GB Dataset... This may take a minute.\n";
    if (ftruncate(fd, FILE_SIZE) == -1) {
        perror("ftruncate");
        exit(1);
    }

    // Mmap for writing
    uint16_t* map = (uint16_t*)mmap(nullptr, FILE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (map == MAP_FAILED) {
        perror("mmap");
        exit(1);
    }

    // Threads
    int num_threads = std::thread::hardware_concurrency();
    std::vector<std::thread> threads;
    std::atomic<int> progress(0);
    int chunk = DIM_SIZE / num_threads;

    auto start_t = std::chrono::high_resolution_clock::now();

    for (int t = 0; t < num_threads; ++t) {
        int start = t * chunk;
        int end = (t == num_threads - 1) ? DIM_SIZE : (t + 1) * chunk;
        threads.emplace_back(generate_dataset_worker, start, end, map, std::ref(progress));
    }

    // Report progress
    while (progress < DIM_SIZE) {
        int p = progress.load();
        printf("\r[Generating] %d / %d Masks processed (%.1f%%)", p, (int)DIM_SIZE, (p * 100.0f) / DIM_SIZE);
        fflush(stdout);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    
    for (auto& th : threads) th.join();
    
    auto end_t = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff = end_t - start_t;
    std::cout << "\n[DONE] Generation took " << diff.count() << "s.\n";

    munmap(map, FILE_SIZE);
    close(fd);
}

// -----------------------------------------------------------------------------
// Visualization Engine
// -----------------------------------------------------------------------------

struct Camera {
    float x = 32768.0f; // Target Center X (Mask)
    float y = 32768.0f; // Target Center Y (Seed)
    float dist = 80000.0f;
    float pitch = 45.0f;
    float yaw = 0.0f;
};

// Simple Bitmap Font (Numbers + Basic chars) for Info Window
// (Minimal implementation to avoid external assets)
void draw_string(float x, float y, const std::string& text) {
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, WINDOW_WIDTH, WINDOW_HEIGHT, 0, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    // Very crude line-drawing font
    glLineWidth(2.0f);
    glColor3f(1.0f, 1.0f, 1.0f);
    
    float scale = 10.0f;
    float cx = x;
    
    for(char c : text) {
        glBegin(GL_LINES);
        // Simple digit mapping (just a few for stats)
        // Mapping 0-9 and some letters.
        // A complete font in code is too long, we use a simplified stick font logic
        // 0: TL-TR, TR-BR, BR-BL, BL-TL
        float w = 0.6f * scale;
        float h = 1.0f * scale;
        
        // Safety fallback for unknown chars
        if (c == ' ') { cx += w + 4; continue; }
        
        // Define segments
        bool top=0, mid=0, bot=0, tl=0, tr=0, bl=0, br=0, d1=0, d2=0;
        
        if(isdigit(c)) {
            int d = c - '0';
            if(d!=1 && d!=4) top=1;
            if(d!=0 && d!=1 && d!=7) mid=1;
            if(d!=1 && d!=4 && d!=7) bot=1;
            if(d!=1 && d!=2 && d!=3 && d!=7) tl=1;
            if(d!=5 && d!=6) tr=1;
            if(d==0 || d==2 || d==6 || d==8) bl=1;
            if(d!=2) br=1;
        } else if (c == 'X') { d1=1; d2=1; }
        else if (c == 'Y') { tl=1; tr=1; mid=1; br=1; bl=0; } // V shape-ish
        else if (c == 'Z') { top=1; bot=1; d2=1; }
        else if (c == ':') { /* draw dots */ }
        else { // Box for unknown
            top=bot=tl=tr=bl=br=1;
        }

        if(top) { glVertex2f(cx, y); glVertex2f(cx+w, y); }
        if(mid) { glVertex2f(cx, y+h/2); glVertex2f(cx+w, y+h/2); }
        if(bot) { glVertex2f(cx, y+h); glVertex2f(cx+w, y+h); }
        if(tl)  { glVertex2f(cx, y); glVertex2f(cx, y+h/2); }
        if(bl)  { glVertex2f(cx, y+h/2); glVertex2f(cx, y+h); }
        if(tr)  { glVertex2f(cx+w, y); glVertex2f(cx+w, y+h/2); }
        if(br)  { glVertex2f(cx+w, y+h/2); glVertex2f(cx+w, y+h); }
        if(d1)  { glVertex2f(cx, y); glVertex2f(cx+w, y+h); }
        if(d2)  { glVertex2f(cx, y+h); glVertex2f(cx+w, y); }

        glEnd();
        cx += w + 6.0f;
    }

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
}

// -----------------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------------
int main(int argc, char** argv) {
    ensure_dataset_exists();

    // Open Data
    int fd = open(DATA_FILE, O_RDONLY);
    uint16_t* data = (uint16_t*)mmap(nullptr, FILE_SIZE, PROT_READ, MAP_SHARED, fd, 0);
    if (data == MAP_FAILED) { perror("mmap read"); return 1; }

    // Init SDL
    if (SDL_Init(SDL_INIT_VIDEO) < 0) return 1;
    
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);

    SDL_Window* window = SDL_CreateWindow("LFSR Polynomial Space (Ryzen Optimized)", 
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 
        WINDOW_WIDTH, WINDOW_HEIGHT, SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN);
    SDL_GLContext context = SDL_GL_CreateContext(window);

    // Init GL
    glEnable(GL_DEPTH_TEST);
    //glEnable(GL_POINT_SMOOTH); 
    //glPointSize(2.0f); // High DPI might need scaling

    Camera cam;
    bool running = true;
    
    // Inertia variables
    float vel_yaw = 0.0f;
    float vel_pitch = 0.0f;
    float vel_x = 0.0f;
    float vel_y = 0.0f;
    bool dragging_rot = false;
    bool dragging_pan = false;
    int last_mx = 0, last_my = 0;

    // Hover info
    int hover_mask = -1;
    int hover_seed = -1;
    uint16_t hover_val = 0;

    // Client-side Vertex Buffer for immediate drawing
    // We reuse this buffer to stream points to GPU
    const int MAX_VERTS = 1000000; // 1 Million points per batch max
    struct Vertex { float x, y, z; uint8_t r, g, b; };
    std::vector<Vertex> vbuffer;
    vbuffer.reserve(MAX_VERTS);

    while (running) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) running = false;
            else if (e.type == SDL_MOUSEBUTTONDOWN) {
                if (e.button.button == SDL_BUTTON_LEFT) dragging_rot = true;
                if (e.button.button == SDL_BUTTON_RIGHT) dragging_pan = true;
            }
            else if (e.type == SDL_MOUSEBUTTONUP) {
                dragging_rot = false;
                dragging_pan = false;
            }
            else if (e.type == SDL_MOUSEMOTION) {
                int dx = e.motion.x - last_mx;
                int dy = e.motion.y - last_my;
                last_mx = e.motion.x;
                last_my = e.motion.y;

                if (dragging_rot) {
                    vel_yaw += dx * 0.5f;
                    vel_pitch += dy * 0.5f;
                }
                if (dragging_pan) {
                    // Pan relative to yaw
                    float rad = cam.yaw * 3.14159f / 180.0f;
                    float c = cos(rad), s = sin(rad);
                    vel_x -= (c * dx - s * dy) * (cam.dist * 0.001f);
                    vel_y -= (s * dx + c * dy) * (cam.dist * 0.001f);
                }
            }
            else if (e.type == SDL_MOUSEWHEEL) {
                cam.dist -= e.wheel.y * (cam.dist * 0.1f);
                if (cam.dist < 100.0f) cam.dist = 100.0f;
            }
        }

        // Apply Inertia
        cam.yaw += vel_yaw * 0.1f;
        cam.pitch += vel_pitch * 0.1f;
        cam.x += vel_x * 0.1f;
        cam.y += vel_y * 0.1f;
        
        vel_yaw *= 0.90f;
        vel_pitch *= 0.90f;
        vel_x *= 0.90f;
        vel_y *= 0.90f;
        
        // Clamp Pitch
        if (cam.pitch > 89.0f) cam.pitch = 89.0f;
        if (cam.pitch < -89.0f) cam.pitch = -89.0f;

        // --- RENDER ---
        glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Projection
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        double aspect = (double)WINDOW_WIDTH / WINDOW_HEIGHT;
        double zNear = 10.0;
        double zFar = 500000.0;
        double fov = 60.0;
        double fH = tan(fov / 360 * 3.14159) * zNear;
        double fW = fH * aspect;
        glFrustum(-fW, fW, -fH, fH, zNear, zFar);

        // View
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glTranslatef(0, 0, -cam.dist);
        glRotatef(cam.pitch, 1, 0, 0);
        glRotatef(cam.yaw, 0, 1, 0);
        glTranslatef(-cam.x, 0, -cam.y);

        // --- Dynamic LOD Strategy ---
        // Calculate velocity magnitude to determine "Sketchiness"
        float speed = sqrt(vel_yaw*vel_yaw + vel_pitch*vel_pitch + vel_x*vel_x + vel_y*vel_y);
        int step = 1;
        
        // If moving fast, skip many points (Sketchy mode)
        // If static, show detail based on zoom
        if (speed > 1.0f) {
            step = 128; 
        } else if (speed > 0.1f) {
            step = 32;
        } else {
            // Static. Optimize based on distance (simple assumption)
            // If zoomed out (dist > 30000), step higher
            if (cam.dist > 30000) step = 64;
            else if (cam.dist > 10000) step = 16;
            else if (cam.dist > 2000) step = 4;
            else step = 1; 
        }

        // View Frustum Culling (Simple 2D Box)
        // We only iterate X/Y that could be visible.
        // For simplicity, we just iterate a box around the look-at target.
        // Size of box depends on distance.
        float view_range = cam.dist * 1.5f; 
        int min_x = std::max(0, (int)(cam.x - view_range));
        int max_x = std::min((int)DIM_SIZE, (int)(cam.x + view_range));
        int min_y = std::max(0, (int)(cam.y - view_range));
        int max_y = std::min((int)DIM_SIZE, (int)(cam.y + view_range));

        // Always clamp to valid bounds
        if (min_x < 0) min_x = 0; if (max_x > 65536) max_x = 65536;
        if (min_y < 0) min_y = 0; if (max_y > 65536) max_y = 65536;

        vbuffer.clear();

        // 1. Point Cloud Render
        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_COLOR_ARRAY);

        // Iterate grid
        for (int mx = min_x; mx < max_x; mx += step) {
            for (int my = min_y; my < max_y; my += step) {
                // Read from mmap
                uint16_t val = data[mx * DIM_SIZE + my];
                
                // Height scaling (Z)
                float z = (float)val / 65535.0f * 10000.0f; // Scale height for visibility

                // Color Mapping
                // Heatmap: 0=Blue, 0.5=Green, 1.0=Red
                float duty = (float)val / 65535.0f;
                uint8_t r = (uint8_t)(duty * 255);
                uint8_t g = (uint8_t)(sin(duty * 3.14159) * 255);
                uint8_t b = (uint8_t)((1.0f - duty) * 255);

                // --- Special Blue Cubes (Lock-ups) ---
                // Condition: next_state(s) == s
                // Note: Standard LFSR lockup is 0, but checking actual logic
                uint16_t ns = next_state((uint16_t)my, (uint16_t)mx);
                bool is_lock = (ns == (uint16_t)my);

                if (is_lock) {
                    // Bright Cyan/Blue large point
                    vbuffer.push_back({(float)mx, z, (float)my, 0, 255, 255});
                    // Add a few points around it to simulate a "cube" or thick point
                    if (step > 1) {
                         vbuffer.push_back({(float)mx, z+50, (float)my, 0, 255, 255});
                    }
                } else {
                    vbuffer.push_back({(float)mx, z, (float)my, r, g, b});
                }

                // Batch flush
                if (vbuffer.size() >= MAX_VERTS - 10) {
                    glVertexPointer(3, GL_FLOAT, sizeof(Vertex), &vbuffer[0].x);
                    glColorPointer(3, GL_UNSIGNED_BYTE, sizeof(Vertex), &vbuffer[0].r);
                    glDrawArrays(GL_POINTS, 0, vbuffer.size());
                    vbuffer.clear();
                }
            }
        }
        
        // Final flush
        if (!vbuffer.size() == 0) {
            glVertexPointer(3, GL_FLOAT, sizeof(Vertex), &vbuffer[0].x);
            glColorPointer(3, GL_UNSIGNED_BYTE, sizeof(Vertex), &vbuffer[0].r);
            glDrawArrays(GL_POINTS, 0, vbuffer.size());
        }

        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_COLOR_ARRAY);

        // --- Raycasting for Info Window (CPU side approximation) ---
        // Find point closest to center of screen (Camera Target)
        // Or if mouse usage is desired for picking, we project mouse ray.
        // For simplicity in this CAD style, we show info for the center of focus (cam.x, cam.y)
        hover_mask = (int)(cam.x + 0.5f);
        hover_seed = (int)(cam.y + 0.5f);
        if (hover_mask >= 0 && hover_mask < 65536 && hover_seed >= 0 && hover_seed < 65536) {
            hover_val = data[hover_mask * DIM_SIZE + hover_seed];
        }

        // --- UI Overlay ---
        // Draw bottom info bar
        glDisable(GL_DEPTH_TEST);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(0, WINDOW_WIDTH, WINDOW_HEIGHT, 0, -1, 1);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        // Background
        glColor4f(0.0f, 0.0f, 0.0f, 0.7f);
        glBegin(GL_QUADS);
        glVertex2f(0, WINDOW_HEIGHT - 100);
        glVertex2f(WINDOW_WIDTH, WINDOW_HEIGHT - 100);
        glVertex2f(WINDOW_WIDTH, WINDOW_HEIGHT);
        glVertex2f(0, WINDOW_HEIGHT);
        glEnd();

        // Text Info
        std::string info = "MASK: " + std::to_string(hover_mask) + 
                           "  SEED: " + std::to_string(hover_seed) + 
                           "  DUTY: " + std::to_string(hover_val) + "/65535 (" + 
                           std::to_string((int)(hover_val/655.35)) + "%)";
        
        draw_string(20, WINDOW_HEIGHT - 60, info);
        
        std::string controls = "LMB: Rotate | RMB: Pan | Wheel: Zoom";
        draw_string(20, WINDOW_HEIGHT - 30, controls);

        glEnable(GL_DEPTH_TEST);

        SDL_GL_SwapWindow(window);
    }

    // Cleanup
    munmap(data, FILE_SIZE);
    close(fd);
    SDL_GL_DeleteContext(context);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
