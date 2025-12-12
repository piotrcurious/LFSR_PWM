// lfsr_viz_signed_fixed_with_filters_imgui.cpp
//
// FIX: Replaced custom, unreadable draw_text() with Dear ImGui (v1.89.9) for UI rendering.
// Controls and status are now in a dedicated ImGui window.
//
// FIX: Updated compilation instructions to use system libraries via 'libimgui-dev' (Debian).
//
// Compile:
// You should now link against the installed libraries, eliminating the need to include source files.
// Note: Library names might vary. If the first command fails, try the second.
//
// 1. Preferred Compilation (using standard Debian package names):
// g++ -O3 -std=c++17 -march=native -pthread -o lfsr_viz_signed_fixed_with_filters_imgui \
//     lfsr_viz_signed_fixed_with_filters_imgui.cpp $(pkg-config --cflags --libs sdl2) -lGL \
//     -limgui -limgui-sdl -limgui-opengl3
//
// 2. Simpler Compilation (if backends are bundled):
// g++ -O3 -std=c++17 -march=native -pthread -o lfsr_viz_signed_fixed_with_filters_imgui \
//     lfsr_viz_signed_fixed_with_filters_imgui.cpp -lSDL2 -lGL -limgui
//

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

// --- Dear ImGui Includes (Provided by libimgui-dev) ---
#include "imgui.h"
#include "imgui_impl_sdl.h"
#include "imgui_impl_opengl3.h"
// -----------------------------------------------------------------------


// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------
const char* DATA_FILE = "lfsr_map_signed.bin";
const int N_BITS = 16;
const size_t DIM_SIZE = 1u << N_BITS; // 65536
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
// Data Generation (unchanged)
// -----------------------------------------------------------------------------
void generate_dataset_worker(size_t start_mask,
                             size_t end_mask,
                             int16_t* buffer,
                             std::atomic<size_t>& progress,
                             std::atomic<bool>& error_flag)
{
    std::vector<uint8_t> visited(DIM_SIZE, 0);
    std::vector<uint16_t> chain;
    chain.reserve(DIM_SIZE);

    std::vector<uint16_t> cycle_sorted;
    cycle_sorted.reserve(DIM_SIZE);

    for (int m = (int)start_mask; m < (int)end_mask; ++m) {
        uint16_t mask = (uint16_t)m;

        // clearer & robust: reset visited with std::fill
        std::fill(visited.begin(), visited.end(), 0);

        uint64_t row_offset = (uint64_t)mask * DIM_SIZE;

        for (int s = 0; s < (int)DIM_SIZE; ++s) {
            if (visited[s]) continue;

            uint16_t curr = (uint16_t)s;
            chain.clear();

            // 1) Walk chain (mark visited as we go)
            while (!visited[curr]) {
                visited[curr] = 1;
                chain.push_back(curr);
                curr = next_state(curr, mask);
            }

            // 2) Did we discover a new cycle in this chain or merge into an earlier path?
            auto it = std::find(chain.begin(), chain.end(), curr);
            bool merges_into_existing = (it == chain.end());

            if (!merges_into_existing) {
                // New cycle discovered entirely inside this chain
                size_t cycle_start_idx = std::distance(chain.begin(), it);
                size_t cycle_len = chain.size() - cycle_start_idx;

                // collect & sort cycle nodes
                cycle_sorted.clear();
                for (size_t i = cycle_start_idx; i < chain.size(); ++i) cycle_sorted.push_back(chain[i]);
                std::sort(cycle_sorted.begin(), cycle_sorted.end());

                // A) assign stable (positive) duties to cycle nodes
                for (size_t i = 0; i < cycle_sorted.size(); ++i) {
                    uint16_t st = cycle_sorted[i];
                    double duty = (double)(cycle_len - i) / (double)cycle_len; // 1.0 .. 1/cycle_len
                    int32_t dval = (int32_t)std::round(duty * MAX_DUTY_SCALER);
                    if (dval < 1) dval = 1;
                    buffer[row_offset + st] = (int16_t)dval; // stable -> positive
                }

                // B) assign transient (negative) duties for nodes preceding the cycle in chain
                for (size_t i = 0; i < cycle_start_idx; ++i) {
                    uint16_t trans_st = chain[i];
                    auto it_gt = std::upper_bound(cycle_sorted.begin(), cycle_sorted.end(), trans_st);
                    int count_gt = (int)std::distance(it_gt, cycle_sorted.end());
                    double duty = (cycle_len > 0) ? (double)count_gt / (double)cycle_len : 0.0;
                    if (duty <= 0.0) duty = 1.0 / (double)cycle_len;
                    int32_t sval = (int32_t)std::round(duty * MAX_DUTY_SCALER);
                    if (sval < 1) sval = 1;
                    buffer[row_offset + trans_st] = (int16_t)(-sval); // transient -> negative
                }
            } else {
                // Merges into an existing path computed earlier in this mask loop.
                int16_t target_val = buffer[row_offset + curr];
                double target_duty = (double)std::abs((double)target_val) / MAX_DUTY_SCALER;
                if (target_duty <= 0.0) {
                    target_duty = 1.0 / (double)DIM_SIZE;
                }

                size_t L = chain.size();
                for (size_t i = 0; i < L; ++i) {
                    double frac = ((double)(i + 1)) / (double)L; // 1/L .. 1.0
                    double duty = target_duty * frac;
                    int32_t sval = (int32_t)std::round(duty * MAX_DUTY_SCALER);
                    if (sval < 1) sval = 1;
                    uint16_t trans_st = chain[i];
                    buffer[row_offset + trans_st] = (int16_t)(-sval);
                }
            }
        } // end seeds loop

        // explicit atomic increment of progress for each completed mask
        progress.fetch_add(1, std::memory_order_relaxed);
    } // end masks loop
}

// -----------------------------------------------------------------------------
// ensure_dataset_exists (unchanged)
// -----------------------------------------------------------------------------
void ensure_dataset_exists() {
    int fd = open(DATA_FILE, O_RDWR | O_CREAT, 0644);
    if (fd == -1) { perror("Error opening file"); exit(1); }

    struct stat st;
    fstat(fd, &st);

    if ((size_t)st.st_size == FILE_SIZE) {
        std::cout << "[INFO] Dataset " << DATA_FILE << " found. Ready.\n";
        close(fd);
        return;
    }

    std::cout << "[INFO] Generating dataset (" << (FILE_SIZE/(1024ull*1024ull*1024ull)) << " GB) ... Please wait.\n";
    if (ftruncate(fd, FILE_SIZE) == -1) { perror("ftruncate"); exit(1); }

    int16_t* map = (int16_t*)mmap(nullptr, FILE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (map == MAP_FAILED) { perror("mmap"); exit(1); }

    // Initialize to zero
    std::cout << "[INFO] Initializing memory...\n";
    memset(map, 0, FILE_SIZE);

    int num_threads = std::thread::hardware_concurrency();
    if (num_threads <= 0) num_threads = 4;

    std::vector<std::thread> threads;
    std::atomic<size_t> progress{0};
    std::atomic<bool> error_flag{false};

    size_t chunk = DIM_SIZE / (size_t)num_threads;

    std::cout << "[INFO] Using " << num_threads << " threads\n";

    for (int t = 0; t < num_threads; ++t) {
        size_t start = (size_t)t * chunk;
        size_t end = (t == num_threads - 1) ? DIM_SIZE : (size_t)(t + 1) * chunk;
        threads.emplace_back(generate_dataset_worker,
                             start,
                             end,
                             map,
                             std::ref(progress),
                             std::ref(error_flag));
    }

    // Progress monitoring with timeout detection
    size_t last_progress = 0;
    int stall_count = 0;
    const int MAX_STALLS = 30; // 30 seconds without progress = error

    while (progress.load(std::memory_order_relaxed) < DIM_SIZE) {
        if (error_flag.load(std::memory_order_relaxed)) {
            std::cerr << "[ERROR] Worker thread encountered error. Aborting.\n";
            for (auto& th : threads) {
                if (th.joinable()) th.join();
            }
            munmap(map, FILE_SIZE);
            close(fd);
            exit(1);
        }

        size_t p = progress.load(std::memory_order_relaxed);

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
            printf("\r[Generating] %zu / %zu Masks (%.3f%%)    ",
                   p, DIM_SIZE, (p * 100.0f) / (double)DIM_SIZE);
            fflush(stdout);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
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
// Camera & Vertex structs (unchanged)
// -----------------------------------------------------------------------------
struct Camera {
    float x = 32768.0f;
    float y = 32768.0f;
    float dist = 70000.0f;
    float pitch = 45.0f;
    float yaw = 0.0f;
};

// Removed the unreadable draw_text function!
// -----------------------------------------------------------------------------

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

    // --- Configure SDL/GL Context for ImGui (OpenGL 3.2 Core Profile) ---
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);
    // --------------------------------------------------------------------

    SDL_Window* window = SDL_CreateWindow("LFSR Map (Signed Transients) - ImGui UI",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WINDOW_WIDTH, WINDOW_HEIGHT,
        SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);

    if (!window) {
        std::cerr << "Window creation failed: " << SDL_GetError() << "\n";
        return 1;
    }

    SDL_GLContext context = SDL_GL_CreateContext(window);
    if (!context) {
        std::cerr << "GL context creation failed: " << SDL_GetError() << "\n";
        return 1;
    }
    SDL_GL_MakeCurrent(window, context);
    SDL_GL_SetSwapInterval(1); // Enable VSync

    glEnable(GL_DEPTH_TEST);

    // --- ImGui Initialization ---
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;     // Enable Docking
    
    ImGui::StyleColorsDark();

    // Setup Platform/Renderer bindings
    // Use GLSL 150 (Core profile 3.2)
    const char* glsl_version = "#version 150"; 
    ImGui_ImplSDL2_InitForOpenGL(window, context);
    ImGui_ImplOpenGL3_Init(glsl_version);
    // ----------------------------

    Camera cam;
    bool running = true;
    float vel_yaw=0, vel_pitch=0, vel_x=0, vel_y=0;
    bool drag_rot=false, drag_pan=false;
    int last_mx=0, last_my=0;

    // Filtering / UI state
    bool show_transients = true;
    bool show_stable = true;
    bool show_lockups = true;
    float min_duty = 0.0f; // 0.0 .. 1.0
    float max_duty = 1.0f; // 0.0 .. 1.0

    struct Vertex { float x, y, z; uint8_t r, g, b, a; };
    std::vector<Vertex> vbuffer;
    vbuffer.reserve(1000000);

    while (running) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            ImGui_ImplSDL2_ProcessEvent(&e); // Pass events to ImGui first

            if (e.type == SDL_QUIT) running = false;
            
            // Only handle custom controls if ImGui is not actively capturing input
            if (!io.WantCaptureKeyboard && !io.WantCaptureMouse) 
            {
                if (e.type == SDL_MOUSEBUTTONDOWN) {
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
                else if (e.type == SDL_KEYDOWN) {
                    SDL_Keycode kc = e.key.keysym.sym;
                    const float step_size = 0.01f; // 1% increments

                    if (kc == SDLK_ESCAPE) running = false;
                    else if (kc == SDLK_t) show_transients = !show_transients;
                    else if (kc == SDLK_s) show_stable = !show_stable;
                    else if (kc == SDLK_l) show_lockups = !show_lockups;
                    else if (kc == SDLK_r) { // reset filters
                        min_duty = 0.0f; max_duty = 1.0f;
                        show_transients = show_stable = show_lockups = true;
                    }
                    // Filter adjustment keys (using key presses for quick access)
                    else if (kc == SDLK_LEFT) {
                        min_duty = std::max(0.0f, min_duty - step_size);
                        if (min_duty > max_duty) min_duty = max_duty;
                    }
                    else if (kc == SDLK_RIGHT) {
                        min_duty = std::min(1.0f, min_duty + step_size);
                        if (min_duty > max_duty) min_duty = max_duty;
                    }
                    else if (kc == SDLK_DOWN) {
                        max_duty = std::max(0.0f, max_duty - step_size);
                        if (max_duty < min_duty) max_duty = min_duty;
                    }
                    else if (kc == SDLK_UP) {
                        max_duty = std::min(1.0f, max_duty + step_size);
                        if (max_duty < min_duty) max_duty = min_duty;
                    }
                }
            } // end custom input block
        }

        cam.yaw += vel_yaw * 0.1f; cam.pitch += vel_pitch * 0.1f;
        cam.x += vel_x * 0.1f; cam.y += vel_y * 0.1f;
        vel_yaw *= 0.9f; vel_pitch *= 0.9f; vel_x *= 0.9f; vel_y *= 0.9f;
        if (cam.pitch > 89) cam.pitch = 89; if (cam.pitch < -89) cam.pitch = -89;

        // --- VISUALIZATION RENDERING (Original Logic) ---

        glClearColor(0.1f, 0.1f, 0.12f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_PROJECTION); glLoadIdentity();
        int width, height;
        SDL_GetWindowSize(window, &width, &height);
        double ar = (double)width/height;
        double zn=10.0, zf=500000.0, fov=60.0;
        double fh = tan(fov/360*3.14159)*zn;
        glFrustum(-fh*ar, fh*ar, -fh, fh, zn, zf);

        glMatrixMode(GL_MODELVIEW); glLoadIdentity();
        glTranslatef(0,0,-cam.dist);
        glRotatef(cam.pitch, 1,0,0);
        glRotatef(cam.yaw, 0,1,0);
        glTranslatef(-cam.x, 0, -cam.y);

        float speed = sqrt(vel_yaw*vel_yaw + vel_pitch*vel_pitch + vel_x*vel_x + vel_y*vel_y);
        int step;
        const float STATIONARY_THRESHOLD = 0.001f;

        if (speed < STATIONARY_THRESHOLD) {
            step = (cam.dist > 30000 ? 32 : (cam.dist > 10000 ? 8 : (cam.dist > 2000 ? 4 : 1)));
            vel_yaw= 0 ;vel_pitch=0 ; vel_x=0; vel_y=0;// lock the inertia
        } else {
            if (speed > 1.0f) { step = 128; } else if (speed > 0.1f) { step = 32; } else {
                step = (cam.dist > 30000 ? 64 : (cam.dist > 10000 ? 16 : (cam.dist > 2000 ? 4 : 1)));
            }
        }

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

                // --- Filtering logic: skip points according to UI state ---
                // 1) type filtering
                if (is_transient && !show_transients) continue;
                if (!is_transient && !show_stable) continue;
                // check lockup condition separately: if lockups are disabled, skip them
                uint16_t ns = next_state((uint16_t)my, (uint16_t)mx);
                bool is_lockup = (ns == (uint16_t)my);
                if (is_lockup && !show_lockups) continue;

                // 2) duty range filtering
                if (abs_norm < min_duty) continue;
                if (abs_norm > max_duty) continue;
                // ----------------------------------------------------------------

                uint8_t r, g, b;

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
                    glDrawArrays(GL_POINTS, 0, (GLsizei)vbuffer.size());
                    vbuffer.clear();
                }
            }
        }
        if (!vbuffer.empty()) {
            glVertexPointer(3, GL_FLOAT, sizeof(Vertex), &vbuffer[0].x);
            glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(Vertex), &vbuffer[0].r);
            glDrawArrays(GL_POINTS, 0, (GLsizei)vbuffer.size());
        }

        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_COLOR_ARRAY);

        // --- STATUS FOR IMGUI ---
        int hover_mask = std::clamp((int)cam.x, 0, 65535);
        int hover_seed = std::clamp((int)cam.y, 0, 65535);
        int16_t h_val = data[hover_mask * DIM_SIZE + hover_seed];
        float h_duty = std::abs(h_val) / MAX_DUTY_SCALER;

        std::string s_type = (h_val < 0) ? "TRANSIENT" : "STABLE";
        if (next_state(hover_seed, hover_mask) == hover_seed) s_type = "LOCKUP";
        // ------------------------

        // --- IMGUI RENDERING ---
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL2_NewFrame(window);
        ImGui::NewFrame();

        // 1. Controls & Status Window
        ImGui::SetNextWindowSize(ImVec2(350, 0), ImGuiCond_FirstUseEver);
        ImGui::Begin("LFSR Visualizer Controls");

        ImGui::Text("--- Hover Status (X=Mask, Y=Seed) ---");
        ImGui::Text("Mask: %d | Seed: %d", hover_mask, hover_seed);
        ImGui::Text("Type: %s", s_type.c_str());
        ImGui::Text("Duty: %d%% (%.4f)", (int)(h_duty*100), h_duty);
        ImGui::Separator();

        ImGui::Text("--- Filters --- (Keys: T, S, L, R)");
        ImGui::Checkbox("Show Transients (Negative)", &show_transients);
        ImGui::Checkbox("Show Stable Cycles (Positive)", &show_stable);
        ImGui::Checkbox("Show Lockups (Cyan)", &show_lockups);
        ImGui::Spacing();

        ImGui::Text("Duty Range Filter");
        ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x * 0.9f);
        ImGui::SliderFloat("Min Duty (L/R Keys)", &min_duty, 0.0f, max_duty, "%.2f");
        ImGui::SliderFloat("Max Duty (D/U Keys)", &max_duty, min_duty, 1.0f, "%.2f");
        ImGui::PopItemWidth();
        
        if (ImGui::Button("Reset All Filters (R Key)")) {
            min_duty = 0.0f; max_duty = 1.0f;
            show_transients = show_stable = show_lockups = true;
        }
        ImGui::Separator();

        ImGui::Text("--- Camera ---");
        ImGui::Text("Pos (X, Y): %.0f, %.0f", cam.x, cam.y);
        ImGui::Text("Distance: %.0f", cam.dist);
        ImGui::Text("Pitch/Yaw: %.1f, %.1f", cam.pitch, cam.yaw);
        ImGui::Text("Render Step: %d", step);
        ImGui::Separator();

        ImGui::Text("--- Controls ---");
        ImGui::BulletText("Left Mouse: Rotate Camera (Pitch/Yaw)");
        ImGui::BulletText("Right Mouse: Pan (Move X/Y)");
        ImGui::BulletText("Mouse Wheel: Zoom (Dist)");
        ImGui::BulletText("ESC: Exit");

        ImGui::End();

        // 2. Rendering ImGui
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        // -----------------------

        SDL_GL_SwapWindow(window);
    }

    // --- ImGui Shutdown ---
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
    // ----------------------

    munmap(data, FILE_SIZE);
    close(fd);
    SDL_GL_DeleteContext(context);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
