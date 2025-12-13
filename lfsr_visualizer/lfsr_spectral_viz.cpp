// lfsr_spectral_viz.cpp
//
// Compile: g++ -O3 -std=c++17 -march=native -o lfsr_spectral_viz lfsr_spectral_viz.cpp -lfltk -lfltk_gl -lGL -lGLU -lpthread
//
// Extended LFSR Visualizer with Spectral Analysis capabilities.

#include <FL/Fl.H>
#include <FL/Fl_Gl_Window.H>
#include <FL/Fl_Double_Window.H>
#include <FL/Fl_Check_Browser.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Group.H>
#include <FL/Fl_Box.H>
#include <FL/Fl_Value_Slider.H>
#include <FL/Fl_Choice.H>
#include <FL/Fl_Progress.H>
#include <FL/gl.h>
#include <GL/glu.h>

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <complex>
#include <fstream>
#include <algorithm>
#include <thread>
#include <mutex>
#include <atomic>
#include <sys/stat.h>

// -----------------------------------------------------------------------------
// Core Logic & Math
// -----------------------------------------------------------------------------

const int FFT_SIZE = 64; // Size of bitstream sample for analysis
const char* CACHE_DIR = "spectral_cache";

struct Point2D {
    uint16_t mask;
    uint16_t seed;
};

struct SpectralData {
    float peak_freq; // Normalized 0.0 - 1.0 (Nyquist)
    float error;     // Spectral noise / Quantization error magnitude
};

struct BinData {
    int id;
    std::vector<Point2D> points;
    std::vector<SpectralData> spectra;
    bool visible;
    bool has_spectra;
    float r, g, b;
    BinData() : id(-1), visible(false), has_spectra(false), r(1.0f), g(1.0f), b(1.0f) {}
};

// Global State
static std::vector<BinData> g_bins(256);
static std::mutex g_data_mutex;

// Filters
static float g_min_freq = 0.0f;
static float g_max_freq = 1.0f;
static float g_max_error = 1.0f;
static int g_color_mode = 0; // 0=BinColor, 1=Freq, 2=Error

// -----------------------------------------------------------------------------
// LFSR & FFT Logic
// -----------------------------------------------------------------------------

static inline uint16_t next_state(uint16_t state, uint16_t mask) {
    uint16_t feedback = __builtin_parity(state & mask);
    return static_cast<uint16_t>((state >> 1) | (feedback << 15));
}

// Simple DFT for small N (faster than importing heavy FFT lib for just 64 points)
void compute_spectral_metrics(const std::vector<float>& signal, float& out_peak, float& out_err) {
    float max_mag = 0.0f;
    int peak_idx = 0;
    float total_energy = 0.0f;

    // We only care about magnitude, ignoring DC (k=0)
    for (int k = 1; k < FFT_SIZE / 2; ++k) {
        float re = 0.0f, im = 0.0f;
        for (int n = 0; n < FFT_SIZE; ++n) {
            float angle = 2.0f * 3.14159265358979323846f * k * n / FFT_SIZE;
            re += signal[n] * std::cos(angle);
            im -= signal[n] * std::sin(angle);
        }
        float mag = std::sqrt(re*re + im*im);
        total_energy += mag;

        if (mag > max_mag) {
            max_mag = mag;
            peak_idx = k;
        }
    }

    // Peak Frequency normalized (0.0 to 1.0)
    out_peak = static_cast<float>(peak_idx) / (FFT_SIZE / 2.0f);

    // Error = Ratio of energy NOT in the peak (Spectral purity inverse)
    if (total_energy > 0.0001f) {
        out_err = (total_energy - max_mag) / total_energy;
    } else {
        out_err = 0.0f; // DC only or silence
    }
}

// -----------------------------------------------------------------------------
// File I/O & Workers
// -----------------------------------------------------------------------------

void ensure_dir(const char* path) {
#ifdef _WIN32
    _mkdir(path);
#else
    mkdir(path, 0755);
#endif
}

// Worker to generate spectral cache for a bin
void analyze_bin_worker(int bin_id, Fl_Progress* progress) {
    // 1. Load Points if needed (locally)
    std::vector<Point2D> pts;
    char pfile[128];
    std::snprintf(pfile, sizeof(pfile), "8bit_set/0x%02X.bin", bin_id);
    std::ifstream f(pfile, std::ios::binary | std::ios::ate);
    if (!f.is_open()) {
        std::cerr << "analyze_bin_worker: cannot open " << pfile << "\n";
        return;
    }

    std::streamsize fsz = f.tellg();
    if (fsz <= 0 || (fsz % sizeof(Point2D) != 0)) {
        std::cerr << "analyze_bin_worker: invalid file size for " << pfile << "\n";
        return;
    }
    f.seekg(0, std::ios::beg);
    size_t count = static_cast<size_t>(fsz / sizeof(Point2D));
    pts.resize(count);
    if (!f.read(reinterpret_cast<char*>(pts.data()), fsz)) {
        std::cerr << "analyze_bin_worker: read failed for " << pfile << "\n";
        return;
    }

    std::vector<SpectralData> results;
    results.reserve(count);

    // 2. Process
    std::vector<float> signal(FFT_SIZE);

    for (size_t i = 0; i < pts.size(); ++i) {
        uint16_t m = pts[i].mask;
        uint16_t s = pts[i].seed;
        uint16_t curr = s;

        // Generate bitstream
        for (int t = 0; t < FFT_SIZE; ++t) {
            signal[t] = (curr & 1) ? 1.0f : -1.0f;
            curr = next_state(curr, m);
        }

        SpectralData sd;
        compute_spectral_metrics(signal, sd.peak_freq, sd.error);
        results.push_back(sd);

        // Progress update every N iterations (thread-safe)
        if (progress && (i % 1000 == 0)) {
            Fl::lock();
            progress->value(static_cast<float>(i) / static_cast<float>(pts.size()) * 100.0f);
            Fl::unlock();
        }
    }

    // 3. Save Cache
    ensure_dir(CACHE_DIR);
    char cfile[128];
    std::snprintf(cfile, sizeof(cfile), "%s/0x%02X.spec", CACHE_DIR, bin_id);
    std::ofstream of(cfile, std::ios::binary);
    if (of.is_open()) {
        of.write(reinterpret_cast<const char*>(results.data()), results.size() * sizeof(SpectralData));
    } else {
        std::cerr << "analyze_bin_worker: cannot write cache " << cfile << "\n";
    }

    // 4. Update Live Data if loaded
    {
        std::lock_guard<std::mutex> lock(g_data_mutex);
        if (g_bins[bin_id].id == bin_id && !g_bins[bin_id].points.empty()) {
            g_bins[bin_id].spectra = std::move(results);
            g_bins[bin_id].has_spectra = true;
        }
    }

    if (progress) {
        Fl::lock();
        progress->label("Done");
        progress->value(100.0);
        Fl::unlock();
    }
}

void load_bin(int id) {
    std::lock_guard<std::mutex> lock(g_data_mutex);
    if (!g_bins[id].points.empty()) return;

    // Load Points
    char filename[128];
    std::snprintf(filename, sizeof(filename), "8bit_set/0x%02X.bin", id);
    std::ifstream file(filename, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        std::cerr << "load_bin: cannot open " << filename << "\n";
        return;
    }

    std::streamsize size = file.tellg();
    if (size <= 0 || (size % sizeof(Point2D) != 0)) {
        std::cerr << "load_bin: invalid file size for " << filename << "\n";
        return;
    }
    file.seekg(0, std::ios::beg);
    size_t count = static_cast<size_t>(size / sizeof(Point2D));
    g_bins[id].points.resize(count);
    if (!file.read(reinterpret_cast<char*>(g_bins[id].points.data()), size)) {
        std::cerr << "load_bin: read failed for " << filename << "\n";
        g_bins[id].points.clear();
        return;
    }
    g_bins[id].id = id;

    // Base Color
    float t = static_cast<float>(id) / 255.0f;
    g_bins[id].r = t;
    g_bins[id].g = std::sin(t * 3.14159265358979323846f);
    g_bins[id].b = 1.0f - t;

    // Load Spectral Cache if exists
    std::snprintf(filename, sizeof(filename), "%s/0x%02X.spec", CACHE_DIR, id);
    std::ifstream sfile(filename, std::ios::binary | std::ios::ate);
    if (sfile.is_open()) {
        std::streamsize ssize = sfile.tellg();
        sfile.seekg(0, std::ios::beg);
        if (ssize == static_cast<std::streamsize>(count * sizeof(SpectralData))) {
            g_bins[id].spectra.resize(count);
            if (!sfile.read(reinterpret_cast<char*>(g_bins[id].spectra.data()), ssize)) {
                std::cerr << "load_bin: spectrum read failed for " << filename << "\n";
                g_bins[id].spectra.clear();
                g_bins[id].has_spectra = false;
            } else {
                g_bins[id].has_spectra = true;
            }
        } else {
            // size mismatch -> ignore cache
            g_bins[id].has_spectra = false;
        }
    } else {
        g_bins[id].has_spectra = false;
    }
}

void unload_bin(int id) {
    std::lock_guard<std::mutex> lock(g_data_mutex);
    std::vector<Point2D>().swap(g_bins[id].points);
    std::vector<SpectralData>().swap(g_bins[id].spectra);
    g_bins[id].has_spectra = false;
    g_bins[id].visible = false;
}

// -----------------------------------------------------------------------------
// OpenGL View
// -----------------------------------------------------------------------------

class AnalyticsView : public Fl_Gl_Window {
    float cam_dist = 85000.0f, cam_pitch = 45.0f, cam_yaw = 0.0f;
    float cam_x = 32768.0f, cam_y = 32768.0f;
    int last_mx = 0, last_my = 0;
public:
    AnalyticsView(int x, int y, int w, int h) : Fl_Gl_Window(x, y, w, h) {}

    void draw() override {
        if (!valid()) {
            // Init GL once
            glEnable(GL_DEPTH_TEST);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glEnable(GL_POINT_SMOOTH);
            glPointSize(2.0f);
            glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
            // CRITICAL: setup viewport when context becomes valid
            glViewport(0, 0, w(), h());
        }

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(60.0, (double)w() / (h() > 0 ? h() : 1), 100.0, 300000.0);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glTranslatef(0.0f, 0.0f, -cam_dist);
        glRotatef(cam_pitch, 1.0f, 0.0f, 0.0f);
        glRotatef(cam_yaw, 0.0f, 1.0f, 0.0f);
        glTranslatef(-cam_x, 0.0f, -cam_y);

        // Grid
        glColor3f(0.2f, 0.2f, 0.2f);
        glLineWidth(1.0f);
        glBegin(GL_LINE_LOOP);
        glVertex3f(0.0f, 0.0f, 0.0f); glVertex3f(65536.0f, 0.0f, 0.0f);
        glVertex3f(65536.0f, 0.0f, 65536.0f); glVertex3f(0.0f, 0.0f, 65536.0f);
        glEnd();

        glBegin(GL_POINTS);

        // Determine LOD
        size_t total = 0;
        {
            std::lock_guard<std::mutex> lock(g_data_mutex);
            for (const auto &b : g_bins) if (b.visible) total += b.points.size();
        }
        int skip = 1;
        if (total > 2000000) skip = static_cast<int>(total / 2000000);
        if (skip < 1) skip = 1;

        // Render
        {
            std::lock_guard<std::mutex> lock(g_data_mutex);
            for (const auto& bin : g_bins) {
                if (!bin.visible || bin.points.empty()) continue;

                bool use_spec = bin.has_spectra;
                float z_h = static_cast<float>(bin.id) / 255.0f * 15000.0f;

                const size_t ptsz = bin.points.size();
                const size_t specsz = bin.spectra.size();

                for (size_t i = 0; i < ptsz; i += skip) {
                    if (use_spec && i >= specsz) {
                        // Spectrum cache size mismatch; fallback
                        if (g_color_mode > 0) glColor3f(0.3f, 0.3f, 0.3f);
                        else glColor3f(bin.r, bin.g, bin.b);
                    } else if (use_spec) {
                        float f = bin.spectra[i].peak_freq;
                        float e = bin.spectra[i].error;
                        if (f < g_min_freq || f > g_max_freq) continue;
                        if (e > g_max_error) continue;

                        if (g_color_mode == 1) { // Freq Heatmap
                            glColor3f(f, 1.0f - f, 0.0f);
                        } else if (g_color_mode == 2) { // Error Heatmap
                            glColor3f(e, 0.0f, 1.0f - e);
                        } else {
                            glColor3f(bin.r, bin.g, bin.b);
                        }
                    } else {
                        // No spectral data
                        if (g_color_mode > 0) glColor3f(0.3f, 0.3f, 0.3f);
                        else glColor3f(bin.r, bin.g, bin.b);
                    }

                    glVertex3f(static_cast<float>(bin.points[i].mask), z_h, static_cast<float>(bin.points[i].seed));
                }
            }
        }
        glEnd();
    }

    int handle(int event) override {
        switch (event) {
            case FL_PUSH:
                last_mx = Fl::event_x(); last_my = Fl::event_y();
                return 1;
            case FL_DRAG: {
                int dx = Fl::event_x() - last_mx;
                int dy = Fl::event_y() - last_my;
                last_mx = Fl::event_x(); last_my = Fl::event_y();
                if (Fl::event_button() == FL_LEFT_MOUSE) {
                    cam_yaw += dx * 0.5f;
                    cam_pitch += dy * 0.5f;
                } else {
                    float r = cam_yaw * 0.017453292519943295f;
                    cam_x -= (std::cos(r)*dx - std::sin(r)*dy) * cam_dist * 0.001f;
                    cam_y -= (std::sin(r)*dx + std::cos(r)*dy) * cam_dist * 0.001f;
                }
                redraw();
                return 1;
            }
            case FL_MOUSEWHEEL:
                cam_dist -= Fl::event_dy() * cam_dist * 0.1f;
                if (cam_dist < 100.0f) cam_dist = 100.0f;
                redraw();
                return 1;
            default:
                return Fl_Gl_Window::handle(event);
        }
    }
};

// -----------------------------------------------------------------------------
// UI Construction
// -----------------------------------------------------------------------------

static AnalyticsView* gl_view = nullptr;
static Fl_Check_Browser* browser = nullptr;
static Fl_Progress* pbar = nullptr;

void update_filters(Fl_Widget*, void*) { if (gl_view) gl_view->redraw(); }

void slider_freq_min_cb(Fl_Widget* w, void*) { g_min_freq = static_cast<float>(((Fl_Value_Slider*)w)->value()); if (gl_view) gl_view->redraw(); }
void slider_freq_max_cb(Fl_Widget* w, void*) { g_max_freq = static_cast<float>(((Fl_Value_Slider*)w)->value()); if (gl_view) gl_view->redraw(); }
void slider_err_max_cb(Fl_Widget* w, void*)  { g_max_error = static_cast<float>(((Fl_Value_Slider*)w)->value()); if (gl_view) gl_view->redraw(); }

void choice_color_cb(Fl_Widget* w, void*) {
    g_color_mode = ((Fl_Choice*)w)->value();
    if (gl_view) gl_view->redraw();
}

void btn_analyze_cb(Fl_Widget*, void*) {
    std::vector<int> to_process;
    {
        std::lock_guard<std::mutex> lk(g_data_mutex);
        for (int i = 1; i <= 256; ++i) {
            if (browser->checked(i)) {
                int bin_idx = i - 1;
                if (g_bins[bin_idx].visible && !g_bins[bin_idx].has_spectra) {
                    to_process.push_back(bin_idx);
                }
            }
        }
    }

    if (to_process.empty()) return;

    pbar->label("Analyzing...");
    pbar->activate();
    pbar->value(0.0);

    // Launch background thread (FLTK lock already enabled once in main)
    std::thread([to_process]() {
        for (int id : to_process) {
            analyze_bin_worker(id, pbar);
        }
        Fl::lock();
        pbar->label("Analysis Complete");
        pbar->value(100.0);
        if (gl_view) gl_view->redraw();
        Fl::unlock();
    }).detach();
}

void browser_cb(Fl_Widget*, void*) {
    // Avoid holding g_data_mutex while doing file loads (load_bin/unload_bin lock internally).
    std::vector<int> to_load;
    std::vector<int> to_unload;

    {
        // Brief lock to sample state and check which bins need loads/unloads
        std::lock_guard<std::mutex> lock(g_data_mutex);
        for (int i = 0; i < 256; ++i) {
            bool checked = browser->checked(i + 1);
            if (checked && !g_bins[i].visible) {
                to_load.push_back(i);
            } else if (!checked && g_bins[i].visible) {
                to_unload.push_back(i);
            }
        }
    }

    // Perform loads (these will lock internally)
    for (int id : to_load) {
        load_bin(id); // load_bin locks internally
        // Set visible flag according to whether load succeeded
        {
            std::lock_guard<std::mutex> lock(g_data_mutex);
            g_bins[id].visible = !g_bins[id].points.empty();
        }
    }

    // Perform unloads
    for (int id : to_unload) {
        unload_bin(id); // unload_bin locks internally and clears visible
    }

    if (gl_view) gl_view->redraw();
}

int main(int argc, char **argv) {
    Fl_Double_Window* win = new Fl_Double_Window(1400, 800, "LFSR Spectral Analytics");

    // Left: GL View
    gl_view = new AnalyticsView(10, 10, 1080, 780);

    // Right: Controls group
    Fl_Group* grp = new Fl_Group(1100, 10, 290, 780);
    grp->begin(); // ensure children are parented into the group

    browser = new Fl_Check_Browser(1100, 10, 290, 300, "Bins");
    for (int i = 0; i < 256; ++i) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "Bin 0x%02X", i);
        browser->add(buf);
    }
    browser->callback(browser_cb);
    browser->when(FL_WHEN_CHANGED);

    // Spectral Controls
    Fl_Box* lbl = new Fl_Box(1100, 320, 290, 25, "--- Spectral Analysis ---");
    lbl->labelfont(FL_BOLD);

    Fl_Button* btn_run = new Fl_Button(1100, 350, 290, 30, "Run Analysis on Selected");
    btn_run->callback(btn_analyze_cb);

    pbar = new Fl_Progress(1100, 385, 290, 20);
    pbar->selection_color(FL_GREEN);

    // Filters
    Fl_Box* lbl2 = new Fl_Box(1100, 420, 290, 25, "--- Filters ---");
    lbl2->labelfont(FL_BOLD);

    Fl_Value_Slider* sl_min_f = new Fl_Value_Slider(1100, 450, 290, 25, "Min Freq");
    sl_min_f->type(FL_HOR_SLIDER); sl_min_f->bounds(0.0, 1.0); sl_min_f->value(0.0);
    sl_min_f->callback(slider_freq_min_cb);
    sl_min_f->align(FL_ALIGN_TOP);

    Fl_Value_Slider* sl_max_f = new Fl_Value_Slider(1100, 500, 290, 25, "Max Freq");
    sl_max_f->type(FL_HOR_SLIDER); sl_max_f->bounds(0.0, 1.0); sl_max_f->value(1.0);
    sl_max_f->callback(slider_freq_max_cb);
    sl_max_f->align(FL_ALIGN_TOP);

    Fl_Value_Slider* sl_err = new Fl_Value_Slider(1100, 550, 290, 25, "Max Error (Noise)");
    sl_err->type(FL_HOR_SLIDER); sl_err->bounds(0.0, 1.0); sl_err->value(1.0);
    sl_err->callback(slider_err_max_cb);
    sl_err->align(FL_ALIGN_TOP);

    // Visualization Mode
    Fl_Choice* ch_mode = new Fl_Choice(1100, 600, 290, 25, "Color Mode");
    ch_mode->add("Standard (Bin Color)");
    ch_mode->add("Spectral Frequency");
    ch_mode->add("Quantization Error");
    ch_mode->value(0);
    ch_mode->callback(choice_color_cb);

    grp->end();
    win->end();

    win->resizable(gl_view);
    win->show(argc, argv);

    // One-time enable FLTK thread locking (so worker threads can call Fl::lock()/unlock())
    Fl::lock();

    return Fl::run();
}
