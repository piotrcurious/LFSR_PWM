// lfsr_visualizer.cpp
// A Linux/FLTK/OpenGL tool to visualize, analyze, and select LFSR PWM candidates.
//
// Features:
// - Real-time Audio Playback (ALSA) of LFSR sequences
// - OpenGL rendering of Waveforms, FFT Spectrum, and Phase Space
// - "Composer" mode to pick the best candidate for each PWM level
//
// Build: g++ -std=c++17 -O3 -flto lfsr_visualizer.cpp -o lfsr_vis -lfltk -lfltk_gl -lGL -lGLU -lpthread -lasound

#include <FL/Fl.H>
#include <FL/Fl_Double_Window.H>
#include <FL/Fl_Gl_Window.H>
#include <FL/Fl_Hold_Browser.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Check_Button.H>
#include <FL/Fl_Hor_Value_Slider.H>
#include <FL/Fl_Value_Output.H>
#include <FL/Fl_Group.H>
#include <FL/Fl_File_Chooser.H>
#include <FL/fl_draw.H>
#include <GL/gl.h>
#include <GL/glu.h>
#include <alsa/asoundlib.h> // ALSA Audio

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

// --- Math & LFSR Engine ---

using u64 = uint64_t;
const double PI = 3.14159265358979323846;

struct Candidate {
    u64 mask;
    u64 seed;
    int period;
    int lc;             
    double duty;
    double lf_energy;   
    double score;       
    std::vector<int> bits;
};

// Global State
std::vector<Candidate> current_bin_candidates;
Candidate selected_candidate;
Candidate composer_lut[256];
bool has_selection = false;
int current_pwm_level = -1;

// --- Audio Global State ---
std::atomic<bool> audio_enabled(false);
std::atomic<double> audio_rate_hz(4000.0); // Default playback speed
std::vector<int> audio_buffer;             // The currently playing sequence
std::mutex audio_mutex;
std::thread audio_thread_handle;
std::atomic<bool> app_running(true);

// --- LFSR Helpers ---
static inline u64 next_state(u64 state, u64 mask, int n) {
    u64 fb = __builtin_parityll(state & mask);
    return (state >> 1) | (fb << (n - 1));
}

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

void analyze_candidate(Candidate& cand, int n_bits) {
    std::vector<u64> states;
    states.reserve(65536);
    u64 curr = 1;
    
    // 1. Find Period
    u64 start_node = 0x1;
    curr = start_node;
    do {
        states.push_back(curr);
        curr = next_state(curr, cand.mask, n_bits);
    } while (curr != start_node && states.size() < (1<<17));
    
    cand.period = states.size();
    
    // 2. Generate Bits
    cand.bits.resize(cand.period);
    int ones = 0;
    for(int i=0; i<cand.period; ++i) {
        cand.bits[i] = (states[i] > cand.seed) ? 1 : 0;
        ones += cand.bits[i];
    }
    cand.duty = (double)ones / cand.period;

    // 3. Flicker Energy
    double energy = 0.0;
    for (int k = 1; k <= 10; ++k) {
        std::complex<double> sum(0, 0);
        double angle_step = -2.0 * PI * k / cand.period;
        for (int t = 0; t < cand.period; ++t) {
            double val = (double)cand.bits[t] - 0.5;
            sum += std::polar(val, angle_step * t);
        }
        energy += std::norm(sum) / (cand.period * cand.period);
    }
    cand.lf_energy = energy;

    // 4. Linear Complexity
    int lc_sample = std::min(cand.period, 512);
    std::vector<int> sample(cand.bits.begin(), cand.bits.begin() + lc_sample);
    cand.lc = calculate_lc(sample);
}

// --- Audio Thread Logic ---
// Standalone handler to suppress ALSA error messages
void alsa_error_handler(const char *file, int line, const char *function, int err, const char *fmt, ...) {
    // Do nothing to suppress output
}

void audio_worker() {
    snd_pcm_t *pcm_handle;
    unsigned int rate = 44100;
    
    // Suppress ALSA error output to stderr
    snd_lib_error_set_handler(alsa_error_handler); 

    if (snd_pcm_open(&pcm_handle, "default", SND_PCM_STREAM_PLAYBACK, 0) < 0) {
        std::cerr << "Audio Error: Cannot open default ALSA device." << std::endl;
        return;
    }

    if (snd_pcm_set_params(pcm_handle,
                           SND_PCM_FORMAT_S16_LE,
                           SND_PCM_ACCESS_RW_INTERLEAVED,
                           1, // Channels
                           rate,
                           1, // Soft resample
                           50000) < 0) { // Latency 50ms
        std::cerr << "Audio Error: Cannot set params." << std::endl;
        return;
    }

    // Audio Loop variables
    double phase = 0.0;
    std::vector<int> local_bits;
    const int chunk_size = 1024;
    short buffer[chunk_size];

    while (app_running) {
        // 1. Check if audio is enabled
        if (!audio_enabled) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            // Reset phase to prevent jumps when re-enabling?
            // phase = 0; 
            continue;
        }

        // 2. Fetch data safely
        {
            std::lock_guard<std::mutex> lock(audio_mutex);
            if (!audio_buffer.empty()) {
                // Only copy if changed or if local is empty to save bandwidth
                // Actually, just checking size or ID would be better, but copy is safe
                if(local_bits.size() != audio_buffer.size() || local_bits != audio_buffer) {
                     local_bits = audio_buffer;
                     phase = 0.0; // Reset phase on new track
                }
            }
        }

        if (local_bits.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        // 3. Synthesize Chunk
        double current_rate = audio_rate_hz;
        double phase_inc = current_rate / (double)rate;
        int period = local_bits.size();

        for (int i = 0; i < chunk_size; ++i) {
            int idx = (int)phase % period;
            int bit = local_bits[idx];
            
            // Map 0 -> -Vol, 1 -> +Vol
            // Amplitude 8000 (approx 25% volume) to be gentle
            buffer[i] = bit ? 6000 : -6000;

            phase += phase_inc;
            if (phase >= period) phase -= period;
        }

        // 4. Write to ALSA
        snd_pcm_sframes_t frames = snd_pcm_writei(pcm_handle, buffer, chunk_size);
        if (frames < 0) {
            frames = snd_pcm_recover(pcm_handle, frames, 0);
        }
    }

    snd_pcm_close(pcm_handle);
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

        glColor3f(0.0f, 1.0f, 0.0f); 
        glBegin(GL_LINE_STRIP);
        for (int i = 0; i < std::min(512, selected_candidate.period); ++i) {
            glVertex2f(i, selected_candidate.bits[i]);
            glVertex2f(i + 1, selected_candidate.bits[i]); 
        }
        glEnd();
        
        glColor3f(0.3f, 0.3f, 0.3f);
        glBegin(GL_LINES);
        glVertex2f(0, 0.5); glVertex2f(std::min(512, selected_candidate.period), 0.5); 
        glEnd();

        // --- 2. Spectrum (Middle) ---
        glViewport(0, H / 3, W, H / 3);
        glMatrixMode(GL_PROJECTION); glLoadIdentity(); 
        gluOrtho2D(0, 64, 0, 1.0); 
        glMatrixMode(GL_MODELVIEW); glLoadIdentity();

        glColor3f(0.2f, 0.2f, 0.2f);
        glBegin(GL_QUADS); glVertex2f(0,0); glVertex2f(64,0); glVertex2f(64,1); glVertex2f(0,1); glEnd(); 

        glColor3f(1.0f, 0.8f, 0.0f); 
        glBegin(GL_LINES);
        for (int k = 1; k < 64; ++k) {
            std::complex<double> sum(0, 0);
            double angle_step = -2.0 * PI * k / selected_candidate.period;
            int step = 1;
            if(selected_candidate.period > 2048) step = selected_candidate.period / 2048;
            
            for (int t = 0; t < selected_candidate.period; t+=step) {
                double val = (double)selected_candidate.bits[t] - 0.5;
                sum += std::polar(val, angle_step * t);
            }
            double mag = std::abs(sum) / (selected_candidate.period/step);
            glVertex2f(k, 0);
            glVertex2f(k, mag * 5.0); 
        }
        glEnd();

        // --- 3. Raster Map (Bottom) ---
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
Fl_Check_Button* chk_audio;   // New: Audio Toggle
Fl_Hor_Value_Slider* sld_rate; // New: Rate Slider
std::string dataset_path = ".";

void load_bin(int pwm_level) {
    current_bin_candidates.clear();
    cand_browser->clear();
    current_pwm_level = pwm_level;

    char filename[512];
    snprintf(filename, sizeof(filename), "%s/0x%02X.bin", dataset_path.c_str(), pwm_level);
    
    std::ifstream f(filename, std::ios::binary);
    if (!f) return;

    struct Entry16 { uint16_t m; uint16_t s; };
    
    f.seekg(0, std::ios::end);
    size_t sz = f.tellg();
    f.seekg(0, std::ios::beg);
    
    size_t count = sz / sizeof(Entry16);
    if (count > 2000) count = 2000; 

    std::vector<Candidate> temp_cands;
    temp_cands.reserve(count);

    for(size_t i=0; i<count; ++i) {
        Candidate c;
        Entry16 e; f.read((char*)&e, sizeof(e));
        c.mask = e.m; c.seed = e.s;
        analyze_candidate(c, 16);
        temp_cands.push_back(c);
    }
    
    std::sort(temp_cands.begin(), temp_cands.end(), [](const Candidate& a, const Candidate& b){
        return a.lf_energy < b.lf_energy;
    });

    current_bin_candidates = temp_cands;

    for (const auto& c : current_bin_candidates) {
        char buf[256];
        sprintf(buf, "LF:%.4f  LC:%d  Per:%d", c.lf_energy, c.lc, c.period);
        cand_browser->add(buf);
    }
    
    if (composer_lut[pwm_level].period != 0) {
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
    int level = line - 1; 
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

        // Update Audio Thread
        {
            std::lock_guard<std::mutex> lock(audio_mutex);
            audio_buffer = selected_candidate.bits;
        }
    }
}

void cb_commit(Fl_Widget*, void*) {
    if (!has_selection || current_pwm_level < 0) return;
    composer_lut[current_pwm_level] = selected_candidate;
    btn_commit->label("@+ Saved!");
    btn_commit->color(FL_GREEN);
    char buf[64];
    sprintf(buf, "Level %d [SET]", current_pwm_level);
    bin_browser->text(current_pwm_level + 1, buf);
}

void cb_export(Fl_Widget*, void*) {
    std::ofstream f("custom_lfsr_lut.h");
    f << "#ifndef LFSR_LUT_H\n#define LFSR_LUT_H\n\n";
    f << "#include <stdint.h>\n\n";
    f << "struct LFSR_Entry { uint16_t mask; uint16_t threshold; };\n\n";
    f << "const LFSR_Entry lfsr_lut[256] = {\n";
    
    int filled = 0;
    for(int i=0; i<256; ++i) {
        if (composer_lut[i].period == 0) {
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
        bin_browser->clear();
        for(int i=0; i<256; ++i) {
            char buf[64];
            sprintf(buf, "Level %d", i);
            bin_browser->add(buf);
        }
    }
}

// Audio Callbacks
void cb_audio_toggle(Fl_Widget* w, void*) {
    audio_enabled = ((Fl_Check_Button*)w)->value();
}
void cb_audio_rate(Fl_Widget* w, void*) {
    audio_rate_hz = ((Fl_Hor_Value_Slider*)w)->value();
}

int main(int argc, char **argv) {
    Fl_Double_Window *win = new Fl_Double_Window(1000, 700, "LFSR PWM Visualizer & Composer");

    // Left: Bins
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
    cand_browser = new Fl_Hold_Browser(170, 40, 200, 500, "Candidates (Sorted)");
    cand_browser->textfont(FL_COURIER);
    cand_browser->callback(cb_cands);

    // Info Box
    Fl_Group* grp_info = new Fl_Group(170, 550, 200, 110, "Details");
    grp_info->box(FL_ENGRAVED_BOX);
    out_duty = new Fl_Value_Output(220, 560, 140, 25, "Duty:");
    out_lc = new Fl_Value_Output(220, 590, 140, 25, "LC:");
    out_lf = new Fl_Value_Output(220, 620, 140, 25, "Flicker:");
    grp_info->end();

    // Right: GL View
    gl_view = new VisualizerGL(380, 40, 610, 600, "Visualization");
    gl_view->box(FL_DOWN_FRAME);

    // Bottom Bar
    btn_commit = new Fl_Button(380, 650, 150, 40, "Select for LUT");
    btn_commit->callback(cb_commit);
    
    // --- New Audio Controls ---
    chk_audio = new Fl_Check_Button(540, 650, 60, 40, "Audio");
    chk_audio->callback(cb_audio_toggle);
    
    sld_rate = new Fl_Hor_Value_Slider(610, 650, 170, 40, "Hz");
    sld_rate->bounds(100.0, 22050.0); // Audible range of clock speeds
    sld_rate->value(4000.0);
    sld_rate->step(100);
    sld_rate->callback(cb_audio_rate);
    // --------------------------

    Fl_Button* btn_export = new Fl_Button(790, 650, 200, 40, "Export Custom lut.h");
    btn_export->callback(cb_export);

    win->resizable(gl_view);
    win->show(argc, argv);
    
    // Start Audio Thread
    audio_thread_handle = std::thread(audio_worker);

    int ret = Fl::run();
    
    // Cleanup
    app_running = false;
    if(audio_thread_handle.joinable()) audio_thread_handle.join();
    
    return ret;
}
