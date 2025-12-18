// spectral_lut_gui.cpp
// FLTK + OpenGL GUI frontend for LUT generator and Spectral Prune tools
// - Allows loading mask cache and pairs, tuning parameters, launching the two command-line tools

#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Group.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Input.H>
#include <FL/Fl_Slider.H>
#include <FL/Fl_Value_Slider.H>
#include <FL/Fl_Box.H>
#include <FL/Fl_Text_Display.H>
#include <FL/Fl_Table_Row.H>
#include <FL/Fl_Native_File_Chooser.H>
#include <complex>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <cstdint>

// Type aliases
using u16 = uint16_t;
using u32 = uint32_t;
using cd = std::complex<double>;

// ---------- Data record formats ----------
#pragma pack(push,1)
struct MaskCacheRecord {
    u16 mask;
    u32 rep_period;
    uint8_t good_seed_pct;
    uint8_t is_maximal;
    u16 max_sampled_period;
    uint16_t unused;
};
#pragma pack(pop)

// ---------- Quick LFSR and FFT utilities ----------
static inline u16 galois_step(u16 state, u16 mask) {
    u16 out = state & 1u;
    state >>= 1;
    if (out) state ^= mask;
    return state;
}

uint64_t gcd_u64(uint64_t a, uint64_t b) {
    while (b) {
        uint64_t t = a % b;
        a = b;
        b = t;
    }
    return a;
}

uint64_t lcm_u64(uint64_t a, uint64_t b) {
    if (!a || !b) return 0;
    return (a / gcd_u64(a, b)) * b;
}

// FFT implementation
void fft(std::vector<cd>& a) {
    size_t n = a.size();
    for (size_t i = 1, j = 0; i < n; i++) {
        size_t bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) std::swap(a[i], a[j]);
    }

    for (size_t len = 2; len <= n; len <<= 1) {
        double angle = -2 * M_PI / len;
        cd wlen = cd(cos(angle), sin(angle));
        for (size_t i = 0; i < n; i += len) {
            cd w = 1;
            for (size_t j = 0; j < len / 2; j++) {
                cd u = a[i + j], v = a[i + j + len / 2] * w;
                a[i + j] = u + v;
                a[i + j + len / 2] = u - v;
                w *= wlen;
            }
        }
    }
}

// Spectral entropy calculation
double spectral_entropy(const std::vector<double>& psd) {
    double tot = std::accumulate(psd.begin(), psd.end(), 0.0);
    if (tot <= 0) return 0.0;

    double ent = 0;
    for (double v : psd) {
        double p = v / tot;
        if (p > 0) ent -= p * log(p);
    }
    return ent / log(2.0);
}

// Metrics structure
struct PairVizMetrics {
    double duty;
    double duty_std;
    double entropy;
    std::vector<double> psd;
};

// ---------- Main Application Window ----------
int main_window_width = 1200, main_window_height = 720;

int main(int argc, char** argv) {
    Fl_Window* win = new Fl_Window(main_window_width, main_window_height);

    // Left controls
    Fl_Group* left = new Fl_Group(10, 10, 320, 700);
    left->box(FL_FLAT_BOX);

    Fl_Box* title = new Fl_Box(15, 15, 300, 30, "Spectral LUT GUI");
    title->labelfont(FL_BOLD + FL_ITALIC);
    title->labelsize(16);

    Fl_Button* btn_load_cache = new Fl_Button(15, 60, 130, 28, "Load cache");
    Fl_Button* btn_load_pairs = new Fl_Button(160, 60, 130, 28, "Load pairs");

    Fl_Input* inp_cache = new Fl_Input(15, 95, 275, 24, "cache:");
    inp_cache->readonly(1);

    Fl_Input* inp_pairs = new Fl_Input(15, 125, 275, 24, "pairs:");
    inp_pairs->readonly(1);

    left->end();

    // Right: GL visualization + table
    Fl_Group* right = new Fl_Group(340, 10, 840, 700);

    right->end();

    win->end();
    win->show(argc, argv);
    return Fl::run();
}
