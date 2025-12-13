// lfsr_spectral_viz.cpp
//
// Compile: g++ -O3 -std=c++17 -march=native -o lfsr_spectral_viz lfsr_spectral_viz.cpp -lfltk -lfltk_gl -lGL -lGLU -lpthread
//
// Description:
// Extended LFSR Visualizer with Spectral Analysis capabilities.
// Features: FFT processing, Spectral Caching, Frequency/Error filtering.

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
    bool visible = false;
    bool has_spectra = false;
    float r, g, b;
};

// Global State
std::vector<BinData> g_bins(256);
std::mutex g_data_mutex;

// Filters
float g_min_freq = 0.0f;
float g_max_freq = 1.0f;
float g_max_error = 1.0f;
int g_color_mode = 0; // 0=BinColor, 1=Freq, 2=Error

// -----------------------------------------------------------------------------
// LFSR & FFT Logic
// -----------------------------------------------------------------------------

static inline uint16_t next_state(uint16_t state, uint16_t mask) {
    uint16_t feedback = __builtin_parity(state & mask);
    return (state >> 1) | (feedback << 15);
}

// Simple DFT for small N (faster than importing heavy FFT lib for just 64 points)
void compute_spectral_metrics(const std::vector<float>& signal, float& out_peak, float& out_err) {
    float max_mag = 0.0f;
    int peak_idx = 0;
    float total_energy = 0.0f;
    
    // We only care about magnitude, ignoring DC (k=0)
    for (int k = 1; k < FFT_SIZE / 2; ++k) {
        float re = 0, im = 0;
        for (int n = 0; n < FFT_SIZE; ++n) {
            float angle = 2.0f * 3.14159f * k * n / FFT_SIZE;
            re += signal[n] * cos(angle);
            im -= signal[n] * sin(angle);
        }
        float mag = sqrt(re*re + im*im);
        total_energy += mag;
        
        if (mag > max_mag) {
            max_mag = mag;
            peak_idx = k;
        }
    }
    
    // Peak Frequency normalized (0.0 to 1.0)
    out_peak = (float)peak_idx / (FFT_SIZE / 2.0f);
    
    // Error = Ratio of energy NOT in the peak (Spectral purity inverse)
    if (total_energy > 0.001f) {
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
    // 1. Load Points if needed (temporarily)
    std::vector<Point2D> pts;
    char pfile[64];
    sprintf(pfile, "8bit_set/0x%02X.bin", bin_id);
    std::ifstream f(pfile, std::ios::binary | std::ios::ate);
    if (!f.is_open()) return;
    
    size_t sz = f.tellg();
    f.seekg(0);
    pts.resize(sz / sizeof(Point2D));
    f.read((char*)pts.data(), sz);
    
    std::vector<SpectralData> results;
    results.reserve(pts.size());
    
    // 2. Process
    std::vector<float> signal(FFT_SIZE);
    
    for(size_t i=0; i<pts.size(); ++i) {
        uint16_t m = pts[i].mask;
        uint16_t s = pts[i].seed;
        uint16_t curr = s;
        
        // Generate bitstream
        for(int t=0; t<FFT_SIZE; ++t) {
            // LFSR output is usually the LSB or MSB. Let's use LSB.
            signal[t] = (curr & 1) ? 1.0f : -1.0f;
            curr = next_state(curr, m);
        }
        
        SpectralData sd;
        compute_spectral_metrics(signal, sd.peak_freq, sd.error);
        results.push_back(sd);

        if (progress && (i % 1000 == 0)) {
             Fl::lock();
             progress->value((float)i / pts.size() * 100.0);
             Fl::unlock();
        }
    }
    
    // 3. Save Cache
    ensure_dir(CACHE_DIR);
    char cfile[64];
    sprintf(cfile, "%s/0x%02X.spec", CACHE_DIR, bin_id);
    std::ofstream of(cfile, std::ios::binary);
    of.write((char*)results.data(), results.size() * sizeof(SpectralData));
    
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
        Fl::unlock();
    }
}

void load_bin(int id) {
    if (!g_bins[id].points.empty()) return;

    // Load Points
    char filename[64];
    sprintf(filename, "8bit_set/0x%02X.bin", id);
    std::ifstream file(filename, std::ios::binary | std::ios::ate);
    if (!file.is_open()) return;
    
    size_t size = file.tellg();
    file.seekg(0);
    size_t count = size / sizeof(Point2D);
    g_bins[id].points.resize(count);
    file.read((char*)g_bins[id].points.data(), size);
    g_bins[id].id = id;
    
    // Base Color
    float t = id / 255.0f;
    g_bins[id].r = t; 
    g_bins[id].g = sin(t * 3.14159f); 
    g_bins[id].b = 1.0f - t;

    // Load Spectral Cache if exists
    sprintf(filename, "%s/0x%02X.spec", CACHE_DIR, id);
    std::ifstream sfile(filename, std::ios::binary | std::ios::ate);
    if (sfile.is_open()) {
        size_t ssize = sfile.tellg();
        sfile.seekg(0);
        if (ssize == count * sizeof(SpectralData)) {
            g_bins[id].spectra.resize(count);
            sfile.read((char*)g_bins[id].spectra.data(), ssize);
            g_bins[id].has_spectra = true;
        }
    } else {
        g_bins[id].has_spectra = false;
    }
}

void unload_bin(int id) {
    std::vector<Point2D>().swap(g_bins[id].points);
    std::vector<SpectralData>().swap(g_bins[id].spectra);
    g_bins[id].has_spectra = false;
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
            glEnable(GL_DEPTH_TEST);
            glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
        }
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_PROJECTION); glLoadIdentity();
        gluPerspective(60.0, (double)w() / h(), 100.0, 300000.0);
        glMatrixMode(GL_MODELVIEW); glLoadIdentity();
        glTranslatef(0, 0, -cam_dist);
        glRotatef(cam_pitch, 1, 0, 0);
        glRotatef(cam_yaw, 0, 1, 0);
        glTranslatef(-cam_x, 0, -cam_y);

        // Grid
        glColor3f(0.2f, 0.2f, 0.2f);
        glBegin(GL_LINE_LOOP);
        glVertex3f(0,0,0); glVertex3f(65536,0,0); 
        glVertex3f(65536,0,65536); glVertex3f(0,0,65536);
        glEnd();

        glPointSize(1.0f);
        glBegin(GL_POINTS);
        
        std::lock_guard<std::mutex> lock(g_data_mutex);
        
        // Auto-LOD
        size_t total = 0;
        for(auto& b : g_bins) if(b.visible) total += b.points.size();
        int skip = 1;
        if(total > 2000000) skip = total / 2000000;

        for (const auto& bin : g_bins) {
            if (!bin.visible || bin.points.empty()) continue;

            // Pre-calc visualization props
            bool use_spec = bin.has_spectra;
            float z_h = (float)bin.id / 255.0f * 15000.0f;

            for (size_t i = 0; i < bin.points.size(); i += skip) {
                // Apply Filters
                if (use_spec) {
                    float f = bin.spectra[i].peak_freq;
                    float e = bin.spectra[i].error;
                    if (f < g_min_freq || f > g_max_freq) continue;
                    if (e > g_max_error) continue;

                    // Color Modes
                    if (g_color_mode == 1) { // Freq Heatmap
                        glColor3f(f, 1.0f-f, 0.0f);
                    } else if (g_color_mode == 2) { // Error Heatmap
                        glColor3f(e, 0.0f, 1.0f-e);
                    } else {
                        glColor3f(bin.r, bin.g, bin.b);
                    }
                } else {
                    // If filter active but no data, hide? Or show grey?
                    // We show grey if specific filters are requested but data missing
                    if (g_color_mode > 0) glColor3f(0.3f, 0.3f, 0.3f);
                    else glColor3f(bin.r, bin.g, bin.b);
                }

                glVertex3f(bin.points[i].mask, z_h, bin.points[i].seed);
            }
        }
        glEnd();
    }

    int handle(int event) override {
        if (event == FL_PUSH) {
            last_mx = Fl::event_x(); last_my = Fl::event_y();
            return 1;
        } else if (event == FL_DRAG) {
            int dx = Fl::event_x() - last_mx;
            int dy = Fl::event_y() - last_my;
            last_mx = Fl::event_x(); last_my = Fl::event_y();
            if (Fl::event_button() == FL_LEFT_MOUSE) {
                cam_yaw += dx * 0.5f; cam_pitch += dy * 0.5f;
            } else {
                float r = cam_yaw * 0.01745f;
                cam_x -= (cos(r)*dx - sin(r)*dy) * cam_dist * 0.001f;
                cam_y -= (sin(r)*dx + cos(r)*dy) * cam_dist * 0.001f;
            }
            redraw();
            return 1;
        } else if (event == FL_MOUSEWHEEL) {
            cam_dist -= Fl::event_dy() * cam_dist * 0.1f;
            redraw();
            return 1;
        }
        return Fl_Gl_Window::handle(event);
    }
};

// -----------------------------------------------------------------------------
// UI Construction
// -----------------------------------------------------------------------------

AnalyticsView* gl_view;
Fl_Check_Browser* browser;
Fl_Progress* pbar;

void update_filters(Fl_Widget*, void*) { gl_view->redraw(); }

void slider_freq_min_cb(Fl_Widget* w, void*) { g_min_freq = ((Fl_Value_Slider*)w)->value(); gl_view->redraw(); }
void slider_freq_max_cb(Fl_Widget* w, void*) { g_max_freq = ((Fl_Value_Slider*)w)->value(); gl_view->redraw(); }
void slider_err_max_cb(Fl_Widget* w, void*)  { g_max_error = ((Fl_Value_Slider*)w)->value(); gl_view->redraw(); }

void choice_color_cb(Fl_Widget* w, void*) {
    g_color_mode = ((Fl_Choice*)w)->value();
    gl_view->redraw();
}

void btn_analyze_cb(Fl_Widget*, void*) {
    // Find first selected bin that is missing data
    std::vector<int> to_process;
    for(int i=1; i<=256; ++i) {
        if(browser->checked(i)) {
            int bin_idx = i-1;
            {
                std::lock_guard<std::mutex> lk(g_data_mutex);
                if(g_bins[bin_idx].visible && !g_bins[bin_idx].has_spectra) {
                    to_process.push_back(bin_idx);
                }
            }
        }
    }

    if(to_process.empty()) {
        fl_alert("No visible bins selected that require analysis.");
        return;
    }

    // Launch Thread (Handling one for simplicity in this demo)
    pbar->label("Analyzing...");
    pbar->activate();
    
    std::thread([to_process]() {
        for(int id : to_process) {
            analyze_bin_worker(id, pbar);
        }
        Fl::lock();
        pbar->label("Analysis Complete");
        pbar->value(100);
        gl_view->redraw();
        Fl::unlock();
    }).detach();
}

void browser_cb(Fl_Widget*, void*) {
    std::lock_guard<std::mutex> lock(g_data_mutex);
    for (int i = 0; i < 256; ++i) {
        if (browser->checked(i + 1)) {
            if (!g_bins[i].visible) {
                load_bin(i);
                g_bins[i].visible = true;
            }
        } else {
            if (g_bins[i].visible) {
                unload_bin(i);
                g_bins[i].visible = false;
            }
        }
    }
    gl_view->redraw();
}

int main(int argc, char **argv) {
    Fl_Double_Window* win = new Fl_Double_Window(1400, 800, "LFSR Spectral Analytics");

    // Left: GL View
    gl_view = new AnalyticsView(10, 10, 1080, 780);

    // Right: Controls
    Fl_Group* grp = new Fl_Group(1100, 10, 290, 780);
    
    browser = new Fl_Check_Browser(1100, 10, 290, 300, "Bins");
    for(int i=0; i<256; ++i) {
        char buf[32]; sprintf(buf, "Bin 0x%02X", i);
        browser->add(buf);
    }
    browser->callback(browser_cb);

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
    return Fl::run();
}
