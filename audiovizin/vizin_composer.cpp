// lfsr_visualizer.cpp
// A Linux/FLTK/OpenGL tool to visualize, analyze, and select LFSR PWM candidates.
//
// Features:
// - Loads binary datasets (0x00.bin ... 0xFF.bin)
// - Real-time re-evaluation of Spectral Purity and Linear Complexity
// - OpenGL rendering of Waveforms, FFT Spectrum, and Phase Space
// - "Composer" mode to pick the best candidate for each PWM level
// - Exports custom 'lut.h'
//
// Build: g++ -std=c++17 -O3 -flto lfsr_visualizer.cpp -o lfsr_vis -lfltk -lfltk_gl -lGL -lGLU -lpthread

#include <FL/Fl.H>
#include <FL/Fl_Double_Window.H>
#include <FL/Fl_Gl_Window.H>
#include <FL/Fl_Hold_Browser.H>
#include <FL/Fl_Table_Row.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Value_Output.H>
#include <FL/Fl_Group.H>
#include <FL/Fl_File_Chooser.H>
#include <FL/fl_draw.H>
#include <GL/gl.h>
#include <GL/glu.h>

#include <vector>
#include <complex>
#include <cmath>
#include <algorithm>
#include <string>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <mutex>
#include <thread>
#include <atomic>
#include <numeric>

// --- Math & LFSR Engine (Replicated from Generator) ---

using u64 = uint64_t;
const double PI = 3.14159265358979323846;

// Candidate structure with computed metrics
struct Candidate {
    u64 mask;
    u64 seed;
    int period;
    int lc;                 // Linear Complexity
    double duty;
    double lf_energy;       // Low Freq Energy (Flicker metric)
    double score;           // Composite score
    std::vector<int> bits;  // The generated sequence (0/1)
};

struct Bin {
    int pwm_level; // 0-255
    std::vector<Candidate> candidates;
};

// Global State
std::vector<Candidate> current_bin_candidates;
Candidate selected_candidate;
Candidate composer_lut[256]; // User's final choices
bool has_selection = false;
int current_pwm_level = -1;

// --- LFSR Helpers ---
static inline int popcount_u64(u64 x) { return __builtin_popcountll(x); }
static inline u64 next_state(u64 state, u64 mask, int n) {
    u64 fb = __builtin_parityll(state & mask);
    return (state >> 1) | (fb << (n - 1));
}

// Berlekamp-Massey for Linear Complexity
int calculate_lc(const std::vector<int>& bits) {
    int n = bits.size();
    if (n == 0) return 0;
    std::vector<int> C(n, 0), B(n, 0);
    C[0] = 1; B[0] = 1;
    int L = 0, m = -1;
    for (int N = 0; N < n; ++N) {
        int d = 0;
        for (int i = 0; i <= L; ++i) d ^= (C[i] & bits[N - i]);
        if (d) {
            std::vector<int> T = C;
            int p = N - m;
            for (int i = 0; i + p < n; ++i) C[i + p] ^= B[i];
            if (L <= N / 2) {
                L = N + 1 - L;
                m = N;
                B.swap(T);
            }
        }
    }
    return L;
}

// Generate cycle and compute basic metrics
void analyze_candidate(Candidate& cand, int n_bits) {
    std::vector<u64> states;
    states.reserve(65536);
    u64 curr = 1; // Start generator at 1 to find cycle period
    
    // 1. Find Period (Generator Cycle)
    // Simplified: run until repeat or max
    u64 start_node = 0x1;
    // Fast cycle check
    curr = start_node;
    do {
        states.push_back(curr);
        curr = next_state(curr, cand.mask, n_bits);
    } while (curr != start_node && states.size() < (1<<17));
    
    cand.period = states.size();
    
    // 2. Generate Bits based on Threshold (Seed)
    cand.bits.resize(cand.period);
    int ones = 0;
    for(int i=0; i<cand.period; ++i) {
        // Output logic: (State > Threshold)
        cand.bits[i] = (states[i] > cand.seed) ? 1 : 0;
        ones += cand.bits[i];
    }
    cand.duty = (double)ones / cand.period;

    // 3. Low Frequency Energy (DFT of first 10 harmonics)
    double energy = 0.0;
    for (int k = 1; k <= 10; ++k) {
        std::complex<double> sum(0, 0);
        double angle_step = -2.0 * PI * k / cand.period;
        for (int t = 0; t < cand.period; ++t) {
            double val = (double)cand.bits[t] - 0.5; // Remove DC
            sum += std::polar(val, angle_step * t);
        }
        energy += std::norm(sum) / (cand.period * cand.period); // Power
    }
    cand.lf_energy = energy;

    // 4. Linear Complexity (on a subset to be fast)
    int lc_sample = std::min(cand.period, 512);
    std::vector<int> sample(cand.bits.begin(), cand.bits.begin() + lc_sample);
    cand.lc = calculate_lc(sample);
}

// --- OpenGL Widget ---

class VisualizerGL : public Fl_Gl_Window {
public:
    VisualizerGL(int x, int y, int w, int h, const char* l = 0) : Fl_Gl_Window(x, y, w, h, l) {}

    void draw() override {
        if (!valid()) {
            glViewport(0, 0, w(), h());
            glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        }
        glClear(GL_COLOR_BUFFER_BIT);

        if (!has_selection || selected_candidate.bits.empty()) return;

        int W = w();
        int H = h();
        
        // --- 1. Waveform (Top) ---
        glViewport(0, H * 2 / 3, W, H / 3);
        glMatrixMode(GL_PROJECTION); glLoadIdentity(); gluOrtho2D(0, std::min(512, selected_candidate.period), -0.1, 1.1);
        glMatrixMode(GL_MODELVIEW); glLoadIdentity();

        glColor3f(0.0f, 1.0f, 0.0f); // Green trace
        glBegin(GL_LINE_STRIP);
        for (int i = 0; i < std::min(512, selected_candidate.period); ++i) {
            glVertex2f(i, selected_candidate.bits[i]);
            glVertex2f(i + 1, selected_candidate.bits[i]); // Hold step
        }
        glEnd();
        
        // Draw Grid
        glColor3f(0.3f, 0.3f, 0.3f);
        glBegin(GL_LINES);
        glVertex2f(0, 0.5); glVertex2f(std::min(512, selected_candidate.period), 0.5); // DC line
        glEnd();

        // --- 2. Spectrum / FFT (Middle) ---
        // Compute simple spectrum for display
        glViewport(0, H / 3, W, H / 3);
        glMatrixMode(GL_PROJECTION); glLoadIdentity(); 
        // Log scale X often better, but linear is easier for harmonics. 
        // Y scale usually Log.
        gluOrtho2D(0, 64, 0, 1.0); // Show first 64 harmonics normalized
        glMatrixMode(GL_MODELVIEW); glLoadIdentity();

        glColor3f(0.2f, 0.2f, 0.2f);
        glBegin(GL_QUADS); glVertex2f(0,0); glVertex2f(64,0); glVertex2f(64,1); glVertex2f(0,1); glEnd(); // BG

        glColor3f(1.0f, 0.8f, 0.0f); // Orange spectrum
        glBegin(GL_LINES);
        for (int k = 1; k < 64; ++k) {
            std::complex<double> sum(0, 0);
            double angle_step = -2.0 * PI * k / selected_candidate.period;
            // Decimate calculation for speed in render loop if period is huge
            int step = 1;
            if(selected_candidate.period > 2048) step = selected_candidate.period / 2048;
            
            for (int t = 0; t < selected_candidate.period; t+=step) {
                double val = (double)selected_candidate.bits[t] - 0.5;
                sum += std::polar(val, angle_step * t);
            }
            double mag = std::abs(sum) / (selected_candidate.period/step);
            // Normalize for visual (heuristic)
            glVertex2f(k, 0);
            glVertex2f(k, mag * 5.0); // Scale up
        }
        glEnd();

        // --- 3. Phase Space / Return Map (Bottom) ---
        // Plot (x_t, x_{t+1}) is boring for binary (only 4 points: 00, 01, 10, 11).
        // Instead, let's plot "Run Length Histogram" or "State vs Next State" (if we had full states).
        // Since we have full states in analysis but only bits here, let's reconstruct states roughly
        // or just visualize the bits as a raster map.
        
        // Let's do a Raster Map (Bits as pixels) to see patterns
        glViewport(0, 0, W, H / 3);
        glMatrixMode(GL_PROJECTION); glLoadIdentity(); 
        int dim = (int)sqrt(selected_candidate.period);
        if(dim == 0) dim = 1;
        gluOrtho2D(0, dim, 0, dim);
        glMatrixMode(GL_MODELVIEW); glLoadIdentity();
        
        glPointSize(2.0f);
        glBegin(GL_POINTS);
        for(int i=0; i<selected_candidate.period; ++i) {
            if(selected_candidate.bits[i]) glColor3f(1.0f, 1.0f, 1.0f);
            else glColor3f(0.0f, 0.0f, 0.0f);
            
            int x = i % dim;
            int y = i / dim;
            glVertex2i(x, y);
        }
        glEnd();
    }
};

// --- GUI Logic ---

Fl_Hold_Browser* bin_browser;
Fl_Hold_Browser* cand_browser;
VisualizerGL* gl_view;
Fl_Value_Output* out_duty;
Fl_Value_Output* out_lc;
Fl_Value_Output* out_lf;
Fl_Button* btn_commit;
std::string dataset_path = ".";

void load_bin(int pwm_level) {
    current_bin_candidates.clear();
    cand_browser->clear();
    current_pwm_level = pwm_level;

    char filename[512];
    snprintf(filename, sizeof(filename), "%s/0x%02X.bin", dataset_path.c_str(), pwm_level);
    
    std::ifstream f(filename, std::ios::binary);
    if (!f) return;

    // Detect format size based on file size/alignment or user N
    // Assuming 16-bit entries for now (N<=16) as per previous default
    // If N>16, we need a toggle or detection.
    // Heuristic: Read 4 bytes.
    
    struct Entry16 { uint16_t m; uint16_t s; };
    struct Entry32 { uint32_t m; uint32_t s; };
    
    // We will assume 16-bit for N<=16. 
    // Todo: Add a UI toggle for "32-bit Mode". For now, hardcoded 16.
    bool mode_16 = true;

    f.seekg(0, std::ios::end);
    size_t sz = f.tellg();
    f.seekg(0, std::ios::beg);
    
    size_t count = mode_16 ? sz / sizeof(Entry16) : sz / sizeof(Entry32);
    
    // Limit load for performance?
    if (count > 2000) count = 2000; 

    std::vector<Candidate> temp_cands;
    temp_cands.reserve(count);

    // Read and basic parse
    for(size_t i=0; i<count; ++i) {
        Candidate c;
        if(mode_16) {
            Entry16 e; f.read((char*)&e, sizeof(e));
            c.mask = e.m; c.seed = e.s;
        } else {
            Entry32 e; f.read((char*)&e, sizeof(e));
            c.mask = e.m; c.seed = e.s;
        }
        // Defer heavy analysis? No, user wants to sort.
        // We must analyze to sort.
        // For N=16, analysis is fast (<1ms). 2000 items = 2 seconds.
        // Acceptable freeze for "Loading Bin".
        analyze_candidate(c, 16); // Hardcoded N=16 for this visualizer demo
        temp_cands.push_back(c);
    }
    
    // Sort by LF Energy (Best first)
    std::sort(temp_cands.begin(), temp_cands.end(), [](const Candidate& a, const Candidate& b){
        return a.lf_energy < b.lf_energy;
    });

    current_bin_candidates = temp_cands;

    // Populate List
    for (const auto& c : current_bin_candidates) {
        char buf[256];
        sprintf(buf, "LF:%.4f  LC:%d  Per:%d", c.lf_energy, c.lc, c.period);
        cand_browser->add(buf);
    }
    
    // Check if this level has a committed selection
    if (composer_lut[pwm_level].period != 0) {
        // Mark it visually?
        btn_commit->label("@+ Check (Selected)");
        btn_commit->color(FL_GREEN);
    } else {
        btn_commit->label("Select for LUT");
        btn_commit->color(FL_BACKGROUND_COLOR);
    }
}

void cb_bins(Fl_Widget*, void*) {
    int line = bin_browser->value();
    if (line == 0) return;
    int level = line - 1; // 1-based index
    load_bin(level);
}

void cb_cands(Fl_Widget*, void*) {
    int line = cand_browser->value();
    if (line == 0) return;
    int idx = line - 1;
    
    if (idx < current_bin_candidates.size()) {
        selected_candidate = current_bin_candidates[idx];
        has_selection = true;
        
        out_duty->value(selected_candidate.duty);
        out_lc->value(selected_candidate.lc);
        out_lf->value(selected_candidate.lf_energy);
        
        gl_view->redraw();
    }
}

void cb_commit(Fl_Widget*, void*) {
    if (!has_selection || current_pwm_level < 0) return;
    composer_lut[current_pwm_level] = selected_candidate;
    btn_commit->label("@+ Saved!");
    btn_commit->color(FL_GREEN);
    
    // Update bin browser text to show selection exists
    char buf[64];
    sprintf(buf, "Level %d [SET]", current_pwm_level);
    bin_browser->text(current_pwm_level + 1, buf);
}

void cb_export(Fl_Widget*, void*) {
    // Write header
    std::ofstream f("custom_lfsr_lut.h");
    f << "#ifndef LFSR_LUT_H\n#define LFSR_LUT_H\n\n";
    f << "#include <stdint.h>\n\n";
    f << "struct LFSR_Entry { uint16_t mask; uint16_t threshold; };\n\n";
    f << "const LFSR_Entry lfsr_lut[256] = {\n";
    
    int filled = 0;
    for(int i=0; i<256; ++i) {
        if (composer_lut[i].period == 0) {
            // Fill empty with zeros or warning
            f << "  { 0x0000, 0x0000 }, // EMPTY\n";
        } else {
            f << "  { 0x" << std::hex << std::setw(4) << std::setfill('0') << composer_lut[i].mask
              << ", 0x" << std::setw(4) << composer_lut[i].seed << " }, // Duty: " 
              << std::dec << std::fixed << std::setprecision(3) << composer_lut[i].duty << "\n";
            filled++;
        }
    }
    f << "};\n\n#endif\n";
    f.close();
    fl_message("Exported lut.h with %d entries.", filled);
}

void cb_dir(Fl_Widget*, void*) {
    Fl_File_Chooser chooser(".", "*", Fl_File_Chooser::DIRECTORY, "Select Bin Directory");
    chooser.show();
    while(chooser.shown()) Fl::wait();
    if (chooser.value()) {
        dataset_path = chooser.value();
        // Refresh bin list
        bin_browser->clear();
        for(int i=0; i<256; ++i) {
            char buf[64];
            sprintf(buf, "Level %d", i);
            bin_browser->add(buf);
        }
    }
}

int main(int argc, char **argv) {
    Fl_Double_Window *win = new Fl_Double_Window(1000, 700, "LFSR PWM Visualizer & Composer");

    // --- Layout ---
    
    // Left: Bins (PWM Levels)
    bin_browser = new Fl_Hold_Browser(10, 40, 150, 620, "PWM Levels");
    bin_browser->callback(cb_bins);
    for(int i=0; i<256; ++i) {
        char buf[64];
        sprintf(buf, "Level %d", i);
        bin_browser->add(buf);
    }

    Fl_Button* btn_dir = new Fl_Button(10, 10, 150, 25, "Open Dataset Dir");
    btn_dir->callback(cb_dir);

    // Middle: Candidates
    cand_browser = new Fl_Hold_Browser(170, 40, 200, 500, "Candidates (Sorted by Quality)");
    cand_browser->textfont(FL_COURIER);
    cand_browser->callback(cb_cands);

    // Info Box (Below Candidates)
    Fl_Group* grp_info = new Fl_Group(170, 550, 200, 110, "Details");
    grp_info->box(FL_ENGRAVED_BOX);
    out_duty = new Fl_Value_Output(220, 560, 140, 25, "Duty:");
    out_lc = new Fl_Value_Output(220, 590, 140, 25, "LC:");
    out_lf = new Fl_Value_Output(220, 620, 140, 25, "Flicker:");
    grp_info->end();

    // Right: GL View
    gl_view = new VisualizerGL(380, 40, 610, 600, "Visualization (Waveform / FFT / Map)");
    gl_view->box(FL_DOWN_FRAME);

    // Bottom Bar
    btn_commit = new Fl_Button(380, 650, 200, 40, "Select for LUT");
    btn_commit->callback(cb_commit);
    
    Fl_Button* btn_export = new Fl_Button(790, 650, 200, 40, "Export Custom lut.h");
    btn_export->callback(cb_export);

    win->resizable(gl_view);
    win->show(argc, argv);
    
    // Trigger initial load if dir exists
    // load_bin(0);

    return Fl::run();
}
